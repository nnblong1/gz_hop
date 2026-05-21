#!/usr/bin/env python3
"""
Rerun UAM controller autotune trials.

Each trial:
  - generates a temporary controller YAML,
  - starts PX4 SITL gz_x500_hop,
  - starts ROS2 uam_qgc_mode.launch.py with the data logger,
  - arms, takes off, enables the external rate controller,
  - optionally starts arm motion,
  - scores the generated summary.json.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

try:
    import yaml
except Exception as exc:  # pragma: no cover
    yaml = None
    YAML_ERROR = exc
else:
    YAML_ERROR = None


PX4_ROOT = Path("/home/wicom/PX4-Autopilot")
GZ_ROOT = PX4_ROOT / "Tools/simulation/gz"
ROS2_WS = Path("/home/wicom/ros2_ws")
DEFAULT_BASE_CONFIG = ROS2_WS / "src/uam_controller/config/uam_controller_params.yaml"
DEFAULT_OUTPUT_ROOT = Path("/home/wicom/uam_results")

PROCESS_PATTERNS = (
    "px4_sitl",
    "bin/px4",
    "px4 starting",
    "gz sim",
    "gz-server",
    "gz-gui",
    "MicroXRCEAgent",
    "ros2 launch uam_controller uam_qgc_mode.launch.py",
    "uam_backstepping_rbfnn_node",
    "arm_dynamics_node.py",
    "arm_gazebo_command_node.py",
    "arm_gazebo_joint_state_bridge.py",
    "arm_virtual_state_node.py",
    "arm_initial_pose.py",
    "uam_telemetry_monitor.py",
    "rbfnn_data_logger.py",
    "qgc_rbfnn_trigger.py",
    "arm_trajectory_generator.py",
)


def now_stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def finite(value: Any, default: float = math.nan) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default


def clamp(value: float, lo: float, hi: float) -> float:
    return min(max(value, lo), hi)


def sample_around(rng: random.Random, base: float, lo: float, hi: float, span: float) -> float:
    return clamp(base + rng.uniform(-span, span), lo, hi)


def shlex_quote(text: str) -> str:
    return "'" + text.replace("'", "'\"'\"'") + "'"


def load_yaml(path: Path) -> dict[str, Any]:
    if yaml is None:
        raise RuntimeError(f"PyYAML is required: {YAML_ERROR}")
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise RuntimeError(f"YAML is not a mapping: {path}")
    return data


def write_yaml(path: Path, data: dict[str, Any]) -> None:
    if yaml is None:
        raise RuntimeError(f"PyYAML is required: {YAML_ERROR}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)


def controller_params(data: dict[str, Any]) -> dict[str, Any]:
    return data.setdefault("uam_adaptive_controller", {}).setdefault("ros__parameters", {})


def arm_dynamics_params(data: dict[str, Any]) -> dict[str, Any]:
    return data.setdefault("arm_dynamics_node", {}).setdefault("ros__parameters", {})


def base_param(data: dict[str, Any], name: str, default: float) -> float:
    return finite(controller_params(data).get(name), default)


def bool_base_param(data: dict[str, Any], name: str, default: bool) -> bool:
    value = controller_params(data).get(name, default)
    if isinstance(value, bool):
        return value
    return finite(value, 1.0 if default else 0.0) > 0.5


def cleanup_environment() -> None:
    for pattern in PROCESS_PATTERNS:
        subprocess.run(
            ["pkill", "-TERM", "-f", pattern],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    time.sleep(2.0)
    for pattern in PROCESS_PATTERNS:
        subprocess.run(
            ["pkill", "-KILL", "-f", pattern],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    time.sleep(1.0)


class ManagedProcess:
    def __init__(
        self,
        name: str,
        command: list[str],
        log_path: Path,
        cwd: Path | None = None,
        stdin: bool = False,
    ) -> None:
        self.name = name
        self.log_path = log_path
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_handle = self.log_path.open("w", encoding="utf-8", errors="replace")
        self.proc = subprocess.Popen(
            command,
            cwd=str(cwd) if cwd else None,
            stdin=subprocess.PIPE if stdin else subprocess.DEVNULL,
            stdout=self.log_handle,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            preexec_fn=os.setsid,
        )

    def send(self, line: str) -> None:
        if self.proc.stdin and self.proc.poll() is None:
            self.proc.stdin.write(line.rstrip() + "\n")
            self.proc.stdin.flush()

    def terminate(self, timeout_s: float = 8.0) -> None:
        if self.proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                self.proc.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.proc.wait(timeout=3.0)
        self.log_handle.flush()
        self.log_handle.close()


def bash_ros(command: str, ros_log_dir: Path) -> list[str]:
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    return [
        "/bin/bash",
        "-lc",
        "set +u; "
        f"export ROS_LOG_DIR={shlex_quote(str(ros_log_dir))}; "
        "source /opt/ros/humble/setup.bash; "
        f"source {shlex_quote(str(ROS2_WS / 'install/setup.bash'))}; "
        f"{command}",
    ]


def service_call_enable(ros_log_dir: Path, log_path: Path, timeout_s: float) -> bool:
    deadline = time.time() + timeout_s
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("a", encoding="utf-8", errors="replace") as log:
        while time.time() < deadline:
            cmd = bash_ros(
                "timeout 12s ros2 service call /uam/enable_external_controller std_srvs/srv/Trigger",
                ros_log_dir,
            )
            proc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, text=True, check=False)
            log.flush()
            if proc.returncode == 0:
                text = log_path.read_text(encoding="utf-8", errors="replace")
                if "success=True" in text or "success: true" in text:
                    return True
            time.sleep(5.0)
    return False


def start_arm_motion(args: argparse.Namespace, trial_dir: Path, ros_log_dir: Path) -> ManagedProcess | None:
    if args.arm_amplitude <= 0.0:
        return None
    cmd = (
        "ros2 run uam_controller arm_trajectory_generator.py "
        f"--pattern {args.arm_pattern} "
        f"--duration {args.arm_duration_s} "
        f"--amplitude {args.arm_amplitude} "
        f"--rate {args.arm_rate_hz} "
        f"--step-hold-time {args.arm_step_hold_s} "
        f"--transition-time {args.arm_transition_s}"
    )
    return ManagedProcess(
        "arm_trajectory",
        bash_ros(cmd, ros_log_dir),
        trial_dir / "arm_trajectory.log",
        cwd=GZ_ROOT,
    )


def candidate_params(
    stage: str,
    trial_id: int,
    rng: random.Random,
    base: dict[str, Any],
    refine_scale: float,
) -> dict[str, Any]:
    p = controller_params(base)
    if trial_id == 1:
        out: dict[str, Any] = {
            "rate_Kp_roll": finite(p.get("rate_Kp_roll"), 0.09),
            "rate_Kp_pitch": finite(p.get("rate_Kp_pitch"), 0.09),
            "rate_Kp_yaw": finite(p.get("rate_Kp_yaw"), 0.055),
            "rate_Ki_roll": finite(p.get("rate_Ki_roll"), 0.025),
            "rate_Ki_pitch": finite(p.get("rate_Ki_pitch"), 0.025),
            "rate_Ki_yaw": finite(p.get("rate_Ki_yaw"), 0.015),
            "rate_Kd_roll": finite(p.get("rate_Kd_roll"), 0.0045),
            "rate_Kd_pitch": finite(p.get("rate_Kd_pitch"), 0.0045),
            "rate_Kd_yaw": finite(p.get("rate_Kd_yaw"), 0.0018),
            "base_roll_offset": finite(p.get("base_roll_offset"), 0.0),
            "base_pitch_offset": finite(p.get("base_pitch_offset"), -0.08),
            "rbfnn_lr": finite(p.get("rbfnn_lr"), 0.01),
            "rbfnn_gaussian_width": finite(p.get("rbfnn_gaussian_width"), 0.8),
            "rbfnn_e_modification": finite(p.get("rbfnn_e_modification"), 0.02),
            "rbfnn_num_neurons": int(p.get("rbfnn_num_neurons", 35)),
            "arm_ff_timeout_s": finite(p.get("arm_ff_timeout_s"), 0.18),
            "arm_ff_lpf_alpha": finite(p.get("arm_ff_lpf_alpha"), 0.2),
            "arm_ff_start_delay_s": finite(p.get("arm_ff_start_delay_s"), 1.0),
            "arm_ff_ramp_s": finite(p.get("arm_ff_ramp_s"), 5.0),
            "arm_ff_rate_limit_nm_s": finite(p.get("arm_ff_rate_limit_nm_s"), 0.12),
            "arm_ff_max_roll_nm": finite(p.get("arm_ff_max_roll_nm"), 0.08),
            "arm_ff_max_pitch_nm": finite(p.get("arm_ff_max_pitch_nm"), 0.08),
            "arm_ff_max_yaw_nm": finite(p.get("arm_ff_max_yaw_nm"), 0.04),
            "arm_ff_scale_roll": finite(p.get("arm_ff_scale_roll"), 0.0),
            "arm_ff_scale_pitch": finite(p.get("arm_ff_scale_pitch"), 0.0),
            "arm_ff_scale_yaw": finite(p.get("arm_ff_scale_yaw"), 0.0),
            "arm_cg_comp_enable": bool_base_param(base, "arm_cg_comp_enable", False),
            "arm_cg_roll_gain": finite(p.get("arm_cg_roll_gain"), 0.0),
            "arm_cg_pitch_gain": finite(p.get("arm_cg_pitch_gain"), 0.0),
            "arm_cg_max_norm": finite(p.get("arm_cg_max_norm"), 0.04),
            "arm_cg_lpf_alpha": finite(p.get("arm_cg_lpf_alpha"), 0.1),
        }
    else:
        scale = clamp(refine_scale, 0.05, 1.5)
        out = {
            "rate_Kp_roll": sample_around(rng, base_param(base, "rate_Kp_roll", 0.09), 0.04, 0.16, 0.045 * scale),
            "rate_Kp_pitch": sample_around(rng, base_param(base, "rate_Kp_pitch", 0.09), 0.035, 0.14, 0.045 * scale),
            "rate_Kp_yaw": sample_around(rng, base_param(base, "rate_Kp_yaw", 0.055), 0.025, 0.10, 0.020 * scale),
            "rate_Ki_roll": sample_around(rng, base_param(base, "rate_Ki_roll", 0.025), 0.0, 0.06, 0.020 * scale),
            "rate_Ki_pitch": sample_around(rng, base_param(base, "rate_Ki_pitch", 0.025), 0.0, 0.06, 0.020 * scale),
            "rate_Ki_yaw": sample_around(rng, base_param(base, "rate_Ki_yaw", 0.015), 0.0, 0.04, 0.010 * scale),
            "rate_Kd_roll": sample_around(rng, base_param(base, "rate_Kd_roll", 0.0045), 0.0008, 0.008, 0.0020 * scale),
            "rate_Kd_pitch": sample_around(rng, base_param(base, "rate_Kd_pitch", 0.0045), 0.0008, 0.008, 0.0020 * scale),
            "rate_Kd_yaw": sample_around(rng, base_param(base, "rate_Kd_yaw", 0.0018), 0.0003, 0.0035, 0.0008 * scale),
            "base_roll_offset": sample_around(rng, base_param(base, "base_roll_offset", 0.0), -0.05, 0.05, 0.018 * scale),
            "base_pitch_offset": sample_around(rng, base_param(base, "base_pitch_offset", -0.08), -0.13, -0.02, 0.025 * scale),
            "rbfnn_lr": sample_around(rng, base_param(base, "rbfnn_lr", 0.003), 0.0002, 0.01, 0.003 * scale),
            "rbfnn_gaussian_width": sample_around(rng, base_param(base, "rbfnn_gaussian_width", 0.8), 0.45, 1.5, 0.25 * scale),
            "rbfnn_e_modification": sample_around(rng, base_param(base, "rbfnn_e_modification", 0.02), 0.005, 0.08, 0.020 * scale),
            "rbfnn_num_neurons": rng.choice((21, 25, 31, 35)),
            "arm_ff_timeout_s": sample_around(rng, base_param(base, "arm_ff_timeout_s", 0.18), 0.08, 0.28, 0.05 * scale),
            "arm_ff_lpf_alpha": sample_around(rng, base_param(base, "arm_ff_lpf_alpha", 0.2), 0.05, 0.45, 0.14 * scale),
            "arm_ff_start_delay_s": sample_around(rng, base_param(base, "arm_ff_start_delay_s", 1.0), 0.5, 4.0, 0.8 * scale),
            "arm_ff_ramp_s": sample_around(rng, base_param(base, "arm_ff_ramp_s", 5.0), 3.0, 12.0, 2.0 * scale),
            "arm_ff_rate_limit_nm_s": sample_around(rng, base_param(base, "arm_ff_rate_limit_nm_s", 0.12), 0.02, 0.18, 0.04 * scale),
            "arm_ff_max_roll_nm": sample_around(rng, base_param(base, "arm_ff_max_roll_nm", 0.08), 0.01, 0.16, 0.05 * scale),
            "arm_ff_max_pitch_nm": sample_around(rng, base_param(base, "arm_ff_max_pitch_nm", 0.08), 0.01, 0.16, 0.05 * scale),
            "arm_ff_max_yaw_nm": sample_around(rng, base_param(base, "arm_ff_max_yaw_nm", 0.04), 0.005, 0.08, 0.025 * scale),
            "arm_ff_scale_roll": rng.uniform(-0.40, 0.40),
            "arm_ff_scale_pitch": rng.uniform(-0.40, 0.40),
            "arm_ff_scale_yaw": rng.uniform(-0.25, 0.25),
            "arm_cg_roll_gain": sample_around(rng, base_param(base, "arm_cg_roll_gain", 0.0), -0.01, 0.01, 0.004 * scale),
            "arm_cg_pitch_gain": sample_around(rng, base_param(base, "arm_cg_pitch_gain", 0.0), -0.015, 0.015, 0.006 * scale),
            "arm_cg_max_norm": sample_around(rng, base_param(base, "arm_cg_max_norm", 0.04), 0.01, 0.06, 0.010 * scale),
            "arm_cg_lpf_alpha": sample_around(rng, base_param(base, "arm_cg_lpf_alpha", 0.1), 0.03, 0.30, 0.06 * scale),
        }

    if stage in ("bs_arm_no_ff", "rbfnn_no_ff"):
        out["arm_ff_enable"] = False
        out["arm_ff_scale_roll"] = 0.0
        out["arm_ff_scale_pitch"] = 0.0
        out["arm_ff_scale_yaw"] = 0.0
        out["arm_cg_comp_enable"] = False
    elif stage in ("bs_arm_rne_static", "rbfnn_residual_arm"):
        out["arm_ff_enable"] = True
        out["arm_ff_input_frame"] = "flu"
        out["arm_ff_reaction_sign"] = 1.0
    else:
        raise ValueError(f"Unsupported stage: {stage}")

    out["arm_dynamics.use_sdf_kinematics"] = True
    out["arm_dynamics.use_base_motion"] = True
    out["arm_dynamics.use_base_linear_acc"] = False
    return out


def make_config(base: dict[str, Any], params: dict[str, Any], path: Path) -> None:
    data = json.loads(json.dumps(base))
    c = controller_params(data)
    a = arm_dynamics_params(data)
    for key, value in params.items():
        if key.startswith("arm_dynamics."):
            a[key.split(".", 1)[1]] = value
        else:
            c[key] = value
    write_yaml(path, data)


def latest_summary(output_root: Path) -> Path | None:
    summaries = sorted(output_root.rglob("summary.json"), key=lambda p: p.stat().st_mtime)
    return summaries[-1] if summaries else None


def required_arm_span(args: argparse.Namespace) -> float:
    if args.arm_amplitude <= 0.0:
        return 0.0
    return max(args.arm_actual_span_min_rad, args.arm_amplitude * args.arm_actual_span_min_ratio)


def score_summary(summary: dict[str, Any], args: argparse.Namespace) -> tuple[str, float, dict[str, float]]:
    alt_rmse = finite(summary.get("altitude", {}).get("rmse_error_m"))
    xy_mean = finite(summary.get("xy_drift", {}).get("mean_m"))
    xy_max = finite(summary.get("xy_drift", {}).get("max_m"))
    roll_rms = abs(finite(summary.get("attitude", {}).get("roll_rms_deg")))
    pitch_rms = abs(finite(summary.get("attitude", {}).get("pitch_rms_deg")))
    angle_rms = max(roll_rms, pitch_rms)
    angle_max = finite(summary.get("attitude", {}).get("roll_pitch_abs_max_deg"))
    analysis_duration = finite(summary.get("analysis_duration_s"), 0.0)
    analysis_samples = finite(summary.get("analysis_samples"), 0.0)
    analysis_phase = str(summary.get("analysis_phase", ""))
    arm_cmd_span = finite(summary.get("arm_motion", {}).get("joint_cmd_span_max_rad"), 0.0)
    arm_span = finite(summary.get("arm_motion", {}).get("joint_pos_span_max_rad"))
    min_arm_span = required_arm_span(args)
    arm_span_ratio = arm_span / arm_cmd_span if arm_cmd_span > 1e-6 and math.isfinite(arm_span) else 0.0
    ext_frac = finite(summary.get("external_enabled_fraction"), 0.0)
    ext_duration = finite(summary.get("external_enabled_duration_s"), 0.0)
    rate_err_rms = finite(summary.get("rate_tracking", {}).get("e_omega_norm_rms_radps"), 0.0)
    rate_err_max = finite(summary.get("rate_tracking", {}).get("e_omega_norm_max_radps"), 0.0)

    metrics = {
        "alt_rmse_m": alt_rmse,
        "xy_mean_m": xy_mean,
        "xy_max_m": xy_max,
        "angle_rms_deg": angle_rms,
        "angle_max_deg": angle_max,
        "rate_err_rms_radps": rate_err_rms,
        "rate_err_max_radps": rate_err_max,
        "arm_cmd_span_rad": arm_cmd_span,
        "arm_actual_span_rad": arm_span,
        "arm_required_span_rad": min_arm_span,
        "arm_span_ratio": arm_span_ratio,
        "external_enabled_fraction": ext_frac,
        "external_enabled_duration_s": ext_duration,
        "analysis_duration_s": analysis_duration,
        "analysis_samples": analysis_samples,
    }

    if not math.isfinite(alt_rmse) or not math.isfinite(xy_mean):
        return "NO_DATA", 1e9, metrics
    if not analysis_phase.startswith("external_enabled") or ext_frac < args.min_external_fraction:
        return "NO_EXTERNAL", 1e9, metrics
    if analysis_duration < args.min_external_duration_s:
        return "NO_EXTERNAL_DURATION", 1e9 + args.min_external_duration_s - analysis_duration, metrics
    if args.arm_amplitude > 0.0 and arm_span < min_arm_span:
        return "FAIL_ARM_NO_MOTION", 1e5 + 1000.0 * (min_arm_span - max(0.0, arm_span)), metrics
    if args.arm_amplitude > 0.0 and arm_cmd_span > 1e-6 and arm_span_ratio < args.arm_span_cmd_ratio_min:
        return "FAIL_ARM_TRACKING", 1e5 + 1000.0 * (args.arm_span_cmd_ratio_min - arm_span_ratio), metrics
    if alt_rmse > args.good_alt_rmse_m or summary.get("failure_flags", {}).get("altitude_outside_1_to_3m"):
        return "FAIL_ALT", 1e5 + 1000.0 * alt_rmse, metrics
    if xy_max > args.fail_xy_m:
        return "FAIL_XY", 1e5 + 100.0 * xy_max, metrics
    if angle_max > args.fail_angle_deg:
        return "FAIL_ANGLE", 1e5 + 100.0 * angle_max, metrics
    if rate_err_rms > args.fail_rate_rms_radps:
        return "FAIL_SHAKING", 1e5 + 1000.0 * rate_err_rms, metrics

    score = (
        150.0 * alt_rmse
        + 120.0 * xy_mean
        + 40.0 * xy_max
        + 15.0 * angle_rms
        + 4.0 * angle_max
        + 120.0 * rate_err_rms
        + 8.0 * rate_err_max
    )
    if args.arm_amplitude > 0.0:
        score += 250.0 * max(0.0, min_arm_span - arm_span)
    good = (
        alt_rmse <= args.good_alt_rmse_m
        and xy_mean <= args.good_xy_mean_m
        and angle_rms <= args.good_angle_rms_deg
        and angle_max <= args.good_angle_max_deg
        and rate_err_rms <= args.good_rate_rms_radps
    )
    return ("GOOD" if good else "OK"), score, metrics


def run_trial(trial_id: int, config_path: Path, args: argparse.Namespace, output_dir: Path) -> dict[str, Any]:
    trial_dir = output_dir / f"{args.stage}_trial_{trial_id:03d}"
    trial_dir.mkdir(parents=True, exist_ok=True)
    logger_root = trial_dir / "logger"
    ros_log_dir = trial_dir / "ros_logs"
    cleanup_environment()

    px4 = None
    ros = None
    arm = None
    try:
        px4 = ManagedProcess(
            "px4",
            ["make", "px4_sitl", "gz_x500_hop"],
            trial_dir / "px4.log",
            cwd=PX4_ROOT,
            stdin=True,
        )
        time.sleep(args.px4_wait_s)
        px4.send("param set MC_RATE_EXT_EN 1")
        px4.send("param set COM_RC_IN_MODE 4")

        rbfnn_output = "true" if args.stage in ("rbfnn_no_ff", "rbfnn_residual_arm") else "false"
        arm_ff_launch = "true" if args.stage in ("bs_arm_rne_static", "rbfnn_residual_arm") else "false"
        gazebo_arm_visual = "true" if args.use_gazebo_arm_visual else "false"
        ros_cmd = (
            "ros2 launch uam_controller uam_qgc_mode.launch.py "
            "sim:=true "
            "enable_rbfnn:=true "
            "external_handoff_mode:=manual "
            f"rbfnn_output_enable:={rbfnn_output} "
            f"arm_ff_enable:={arm_ff_launch} "
            f"arm_state_source:={args.arm_state_source} "
            f"use_gazebo_arm_visual:={gazebo_arm_visual} "
            f"config_file:={shlex_quote(str(config_path))} "
            "start_data_logger:=true "
            f"experiment_case:={args.stage}_trial_{trial_id:03d} "
            f"experiment_output_root:={shlex_quote(str(logger_root))} "
            "experiment_log_rate_hz:=20.0"
        )
        ros = ManagedProcess(
            "ros2",
            bash_ros(ros_cmd, ros_log_dir),
            trial_dir / "uam_qgc_mode.log",
            cwd=GZ_ROOT,
        )
        time.sleep(args.ros_wait_s)
        time.sleep(args.post_ros_settle_s)

        px4.send("commander arm -f")
        time.sleep(args.arm_wait_s)
        px4.send("commander takeoff")
        time.sleep(args.takeoff_wait_s)

        enabled = service_call_enable(ros_log_dir, trial_dir / "service_call.log", args.handoff_timeout_s)
        if not enabled:
            return {
                "trial_id": trial_id,
                "stage": args.stage,
                "verdict": "NO_EXTERNAL",
                "score": 1e9,
                "config_path": str(config_path),
                "summary_json": "",
                "metrics": {},
            }
        time.sleep(args.handoff_settle_s)
        arm = start_arm_motion(args, trial_dir, ros_log_dir)
        time.sleep(args.flight_time_s)
    finally:
        if arm:
            arm.terminate()
        if ros:
            ros.terminate()
        if px4:
            px4.send("commander land")
            time.sleep(1.0)
            px4.terminate()
        cleanup_environment()

    summary_path = latest_summary(logger_root)
    if not summary_path:
        return {
            "trial_id": trial_id,
            "stage": args.stage,
            "verdict": "NO_SUMMARY",
            "score": 1e9,
            "config_path": str(config_path),
            "summary_json": "",
            "metrics": {},
        }
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    verdict, score, metrics = score_summary(summary, args)
    result = {
        "trial_id": trial_id,
        "stage": args.stage,
        "verdict": verdict,
        "score": score,
        "config_path": str(config_path),
        "summary_json": str(summary_path),
        "metrics": metrics,
    }
    (trial_dir / "scored_result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result


def append_scoreboard(path: Path, result: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "trial_id",
        "stage",
        "verdict",
        "score",
        "alt_rmse_m",
        "xy_mean_m",
        "xy_max_m",
        "angle_rms_deg",
        "angle_max_deg",
        "rate_err_rms_radps",
        "rate_err_max_radps",
        "arm_cmd_span_rad",
        "arm_actual_span_rad",
        "arm_required_span_rad",
        "arm_span_ratio",
        "external_enabled_fraction",
        "external_enabled_duration_s",
        "analysis_duration_s",
        "analysis_samples",
        "config_path",
        "summary_json",
    ]
    new_file = not path.exists()
    with path.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        if new_file:
            writer.writeheader()
        row = {
            "trial_id": result.get("trial_id"),
            "stage": result.get("stage"),
            "verdict": result.get("verdict"),
            "score": result.get("score"),
            "config_path": result.get("config_path"),
            "summary_json": result.get("summary_json"),
        }
        row.update(result.get("metrics", {}))
        writer.writerow(row)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Rerun UAM Backstepping/RBFNN autotune trials.")
    parser.add_argument("--stage", choices=("bs_arm_no_ff", "rbfnn_no_ff", "bs_arm_rne_static", "rbfnn_residual_arm"), default="rbfnn_residual_arm")
    parser.add_argument("--trials", type=int, default=12)
    parser.add_argument("--seed", type=int, default=121)
    parser.add_argument("--refine-scale", type=float, default=0.2)
    parser.add_argument("--base-config", type=Path, default=DEFAULT_BASE_CONFIG)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_ROOT / f"uam_rerun_{now_stamp()}")
    parser.add_argument("--px4-wait-s", type=float, default=30.0)
    parser.add_argument("--ros-wait-s", type=float, default=20.0)
    parser.add_argument("--post-ros-settle-s", type=float, default=60.0)
    parser.add_argument("--arm-wait-s", type=float, default=5.0)
    parser.add_argument("--takeoff-wait-s", type=float, default=30.0)
    parser.add_argument("--handoff-timeout-s", type=float, default=90.0)
    parser.add_argument("--handoff-settle-s", type=float, default=4.0)
    parser.add_argument("--flight-time-s", type=float, default=55.0)
    parser.add_argument("--arm-pattern", choices=("slow_step", "sin", "step", "combined"), default="slow_step")
    parser.add_argument("--arm-amplitude", type=float, default=0.02)
    parser.add_argument("--arm-duration-s", type=float, default=120.0)
    parser.add_argument("--arm-rate-hz", type=int, default=5)
    parser.add_argument("--arm-step-hold-s", type=float, default=15.0)
    parser.add_argument("--arm-transition-s", type=float, default=5.0)
    parser.add_argument("--arm-state-source", choices=("commanded", "gazebo"), default="commanded")
    parser.add_argument("--use-gazebo-arm-visual", action="store_true")
    parser.add_argument("--arm-actual-span-min-rad", type=float, default=0.004)
    parser.add_argument("--arm-actual-span-min-ratio", type=float, default=0.35)
    parser.add_argument("--arm-span-cmd-ratio-min", type=float, default=0.20)
    parser.add_argument("--min-external-fraction", type=float, default=0.20)
    parser.add_argument("--min-external-duration-s", type=float, default=20.0)
    parser.add_argument("--fail-angle-deg", type=float, default=8.5)
    parser.add_argument("--fail-xy-m", type=float, default=0.45)
    parser.add_argument("--fail-rate-rms-radps", type=float, default=0.30)
    parser.add_argument("--good-alt-rmse-m", type=float, default=0.05)
    parser.add_argument("--good-xy-mean-m", type=float, default=0.10)
    parser.add_argument("--good-angle-rms-deg", type=float, default=1.2)
    parser.add_argument("--good-angle-max-deg", type=float, default=4.0)
    parser.add_argument("--good-rate-rms-radps", type=float, default=0.15)
    parser.add_argument("--stop-when-good", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if yaml is None:
        print(f"PyYAML is required but unavailable: {YAML_ERROR}", file=sys.stderr)
        return 2
    if not args.base_config.exists():
        print(f"Base config not found: {args.base_config}", file=sys.stderr)
        return 2

    args.output_dir.mkdir(parents=True, exist_ok=True)
    configs_dir = args.output_dir / "configs"
    scoreboard = args.output_dir / "scoreboard.csv"
    best_path = args.output_dir / "best_result.json"
    best_config = args.output_dir / "best_uam_controller_params.yaml"
    base = load_yaml(args.base_config)
    shutil.copy2(args.base_config, args.output_dir / "base_config.yaml")

    print("UAM rerun autotune")
    print(f"  stage      : {args.stage}")
    print(f"  base config: {args.base_config}")
    print(f"  output dir : {args.output_dir}")
    print(f"  trials     : {args.trials}")
    print(f"  arm        : {args.arm_pattern}, amp={args.arm_amplitude}")
    print(f"  arm source : {args.arm_state_source}, gazebo_visual={args.use_gazebo_arm_visual}")
    print("")

    rng = random.Random(args.seed)
    best: dict[str, Any] | None = None
    for trial_id in range(1, args.trials + 1):
        params = candidate_params(args.stage, trial_id, rng, base, args.refine_scale)
        config_path = configs_dir / f"{args.stage}_trial_{trial_id:03d}.yaml"
        make_config(base, params, config_path)

        result = run_trial(trial_id, config_path, args, args.output_dir)
        append_scoreboard(scoreboard, result)
        verdict = result["verdict"]
        score = finite(result["score"], 1e9)
        metrics = result.get("metrics", {})
        print(
            f"[trial {trial_id:03d}] {verdict:20s} score={score:10.3f} "
            f"xy={finite(metrics.get('xy_mean_m'), 0.0):.3f}/"
            f"{finite(metrics.get('xy_max_m'), 0.0):.3f}m "
            f"angle={finite(metrics.get('angle_rms_deg'), 0.0):.2f}/"
            f"{finite(metrics.get('angle_max_deg'), 0.0):.2f}deg "
            f"arm={finite(metrics.get('arm_actual_span_rad'), 0.0):.4f}/"
            f"{finite(metrics.get('arm_required_span_rad'), 0.0):.4f} "
            f"ext={finite(metrics.get('analysis_duration_s'), 0.0):.1f}s"
        )
        if best is None or score < finite(best.get("score"), 1e9):
            best = result
            best_path.write_text(json.dumps(best, indent=2), encoding="utf-8")
            shutil.copy2(config_path, best_config)
        if args.stop_when_good and verdict == "GOOD":
            break

    print("")
    if best:
        print("Best candidate:")
        print(f"  verdict: {best['verdict']}")
        print(f"  score  : {best['score']}")
        print(f"  config : {best_config}")
        print(f"  result : {best_path}")
    else:
        print("No result produced.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
