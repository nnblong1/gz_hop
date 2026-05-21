#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GZ_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

TIMESTAMP="${TIMESTAMP:-$(date +%Y%m%d_%H%M%S)}"
BASE_CONFIG="${BASE_CONFIG:-/home/wicom/ros2_ws/src/uam_controller/config/uam_controller_params.yaml}"
OUTPUT_ROOT="${OUTPUT_ROOT:-/home/wicom/uam_results/rbfnn_best_param_search_${TIMESTAMP}}"

BS_TRIALS="${BS_TRIALS:-8}"
RBFNN_FIXED_TRIALS="${RBFNN_FIXED_TRIALS:-10}"
ARM_TRIALS="${ARM_TRIALS:-12}"
SEED="${SEED:-151}"
REFINE_SCALE="${REFINE_SCALE:-0.20}"

ARM_AMPLITUDES="${ARM_AMPLITUDES:-0.02 0.03 0.05}"
ARM_PATTERN="${ARM_PATTERN:-slow_step}"
ARM_RATE_HZ="${ARM_RATE_HZ:-5}"
ARM_STEP_HOLD_S="${ARM_STEP_HOLD_S:-15}"
ARM_TRANSITION_S="${ARM_TRANSITION_S:-5}"
ARM_DURATION_S="${ARM_DURATION_S:-120}"
ARM_STATE_SOURCE="${ARM_STATE_SOURCE:-commanded}"
USE_GAZEBO_ARM_VISUAL="${USE_GAZEBO_ARM_VISUAL:-0}"

PX4_WAIT_S="${PX4_WAIT_S:-30}"
ROS_WAIT_S="${ROS_WAIT_S:-20}"
POST_ROS_SETTLE_S="${POST_ROS_SETTLE_S:-60}"
ARM_WAIT_S="${ARM_WAIT_S:-5}"
TAKEOFF_WAIT_S="${TAKEOFF_WAIT_S:-30}"
HANDOFF_TIMEOUT_S="${HANDOFF_TIMEOUT_S:-90}"
HANDOFF_SETTLE_S="${HANDOFF_SETTLE_S:-4}"
FLIGHT_TIME_S="${FLIGHT_TIME_S:-55}"

FAIL_ANGLE_DEG="${FAIL_ANGLE_DEG:-8.5}"
FAIL_XY_M="${FAIL_XY_M:-0.45}"
FAIL_RATE_RMS_RADPS="${FAIL_RATE_RMS_RADPS:-0.30}"
GOOD_ALT_RMSE_M="${GOOD_ALT_RMSE_M:-0.05}"
GOOD_XY_MEAN_M="${GOOD_XY_MEAN_M:-0.10}"
GOOD_ANGLE_RMS_DEG="${GOOD_ANGLE_RMS_DEG:-1.2}"
GOOD_ANGLE_MAX_DEG="${GOOD_ANGLE_MAX_DEG:-4.0}"
GOOD_RATE_RMS_RADPS="${GOOD_RATE_RMS_RADPS:-0.15}"

ARM_ACTUAL_SPAN_MIN_RAD="${ARM_ACTUAL_SPAN_MIN_RAD:-0.004}"
ARM_ACTUAL_SPAN_MIN_RATIO="${ARM_ACTUAL_SPAN_MIN_RATIO:-0.35}"
ARM_SPAN_CMD_RATIO_MIN="${ARM_SPAN_CMD_RATIO_MIN:-0.20}"
MIN_EXTERNAL_FRACTION="${MIN_EXTERNAL_FRACTION:-0.20}"
MIN_EXTERNAL_DURATION_S="${MIN_EXTERNAL_DURATION_S:-20.0}"

STOP_WHEN_GOOD="${STOP_WHEN_GOOD:-0}"
ACCEPT_OK_INTERMEDIATE="${ACCEPT_OK_INTERMEDIATE:-1}"
FORCE="${FORCE:-0}"
DRY_RUN="${DRY_RUN:-0}"
QUICK="${QUICK:-0}"
RUN_STAGE_BEST_YAML=""

if [[ "${QUICK}" == "1" ]]; then
  BS_TRIALS="${BS_TRIALS_QUICK:-2}"
  RBFNN_FIXED_TRIALS="${RBFNN_FIXED_TRIALS_QUICK:-2}"
  ARM_TRIALS="${ARM_TRIALS_QUICK:-2}"
  ARM_AMPLITUDES="${ARM_AMPLITUDES_QUICK:-0.02}"
  POST_ROS_SETTLE_S="${POST_ROS_SETTLE_S_QUICK:-25}"
  TAKEOFF_WAIT_S="${TAKEOFF_WAIT_S_QUICK:-20}"
  HANDOFF_TIMEOUT_S="${HANDOFF_TIMEOUT_S_QUICK:-45}"
  FLIGHT_TIME_S="${FLIGHT_TIME_S_QUICK:-35}"
fi

usage() {
  cat <<EOF
Usage:
  ./tools/run_rbfnn_best_param_search.sh

Useful environment overrides:
  QUICK=1                         run a short smoke tune
  FORCE=1                         rerun phases even if result files exist
  DRY_RUN=1                       print commands without starting PX4/ROS2
  BASE_CONFIG=/path/params.yaml   starting controller config
  OUTPUT_ROOT=/path/output        result directory
  ARM_AMPLITUDES="0.02 0.03"      arm amplitudes for dynamic-arm phases
  ARM_STATE_SOURCE=commanded      use virtual commanded joint states for arm dynamics
  USE_GAZEBO_ARM_VISUAL=1         also send arm commands to Gazebo visual model

Current output:
  ${OUTPUT_ROOT}
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

json_value() {
  local path="$1"
  local key="$2"
  python3 - "$path" "$key" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
keys = sys.argv[2].split(".")
try:
    data = json.loads(path.read_text(encoding="utf-8"))
    value = data
    for key in keys:
        value = value[key]
except Exception:
    value = ""
print(value)
PY
}

amp_tag() {
  local amp="$1"
  python3 - "$amp" <<'PY'
import sys
amp = float(sys.argv[1])
print(f"{int(round(amp * 1000)):03d}")
PY
}

stop_sim() {
  if [[ -x "${SCRIPT_DIR}/stop_uam_sim.sh" ]]; then
    "${SCRIPT_DIR}/stop_uam_sim.sh" >/dev/null 2>&1 || true
  fi
}

common_args() {
  local extra_seed="$1"
  shift
  printf '%s\n' \
    --seed "${extra_seed}" \
    --refine-scale "${REFINE_SCALE}" \
    --px4-wait-s "${PX4_WAIT_S}" \
    --ros-wait-s "${ROS_WAIT_S}" \
    --post-ros-settle-s "${POST_ROS_SETTLE_S}" \
    --arm-wait-s "${ARM_WAIT_S}" \
    --takeoff-wait-s "${TAKEOFF_WAIT_S}" \
    --handoff-timeout-s "${HANDOFF_TIMEOUT_S}" \
    --handoff-settle-s "${HANDOFF_SETTLE_S}" \
    --flight-time-s "${FLIGHT_TIME_S}" \
    --arm-pattern "${ARM_PATTERN}" \
    --arm-duration-s "${ARM_DURATION_S}" \
    --arm-rate-hz "${ARM_RATE_HZ}" \
    --arm-step-hold-s "${ARM_STEP_HOLD_S}" \
    --arm-transition-s "${ARM_TRANSITION_S}" \
    --arm-state-source "${ARM_STATE_SOURCE}" \
    --arm-actual-span-min-rad "${ARM_ACTUAL_SPAN_MIN_RAD}" \
    --arm-actual-span-min-ratio "${ARM_ACTUAL_SPAN_MIN_RATIO}" \
    --arm-span-cmd-ratio-min "${ARM_SPAN_CMD_RATIO_MIN}" \
    --min-external-fraction "${MIN_EXTERNAL_FRACTION}" \
    --min-external-duration-s "${MIN_EXTERNAL_DURATION_S}" \
    --fail-angle-deg "${FAIL_ANGLE_DEG}" \
    --fail-xy-m "${FAIL_XY_M}" \
    --fail-rate-rms-radps "${FAIL_RATE_RMS_RADPS}" \
    --good-alt-rmse-m "${GOOD_ALT_RMSE_M}" \
    --good-xy-mean-m "${GOOD_XY_MEAN_M}" \
    --good-angle-rms-deg "${GOOD_ANGLE_RMS_DEG}" \
    --good-angle-max-deg "${GOOD_ANGLE_MAX_DEG}" \
    --good-rate-rms-radps "${GOOD_RATE_RMS_RADPS}" \
    "$@"
}

append_global_scoreboard() {
  local phase="$1"
  local result_json="$2"
  local scoreboard="${OUTPUT_ROOT}/scoreboard.csv"
  local verdict score alt xy_mean xy_max angle_rms angle_max rate_rms rate_max
  local arm_cmd arm_actual arm_required arm_ratio ext_frac ext_dur analysis_dur config

  verdict="$(json_value "${result_json}" "verdict")"
  score="$(json_value "${result_json}" "score")"
  alt="$(json_value "${result_json}" "metrics.alt_rmse_m")"
  xy_mean="$(json_value "${result_json}" "metrics.xy_mean_m")"
  xy_max="$(json_value "${result_json}" "metrics.xy_max_m")"
  angle_rms="$(json_value "${result_json}" "metrics.angle_rms_deg")"
  angle_max="$(json_value "${result_json}" "metrics.angle_max_deg")"
  rate_rms="$(json_value "${result_json}" "metrics.rate_err_rms_radps")"
  rate_max="$(json_value "${result_json}" "metrics.rate_err_max_radps")"
  arm_cmd="$(json_value "${result_json}" "metrics.arm_cmd_span_rad")"
  arm_actual="$(json_value "${result_json}" "metrics.arm_actual_span_rad")"
  arm_required="$(json_value "${result_json}" "metrics.arm_required_span_rad")"
  arm_ratio="$(json_value "${result_json}" "metrics.arm_span_ratio")"
  ext_frac="$(json_value "${result_json}" "metrics.external_enabled_fraction")"
  ext_dur="$(json_value "${result_json}" "metrics.external_enabled_duration_s")"
  analysis_dur="$(json_value "${result_json}" "metrics.analysis_duration_s")"
  config="$(json_value "${result_json}" "config_path")"

  if [[ ! -f "${scoreboard}" ]]; then
    printf '%s\n' 'phase,verdict,score,alt_rmse_m,xy_mean_m,xy_max_m,angle_rms_deg,angle_max_deg,rate_err_rms_radps,rate_err_max_radps,arm_cmd_span_rad,arm_actual_span_rad,arm_required_span_rad,arm_span_ratio,external_enabled_fraction,external_enabled_duration_s,analysis_duration_s,config' >"${scoreboard}"
  fi
  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "${phase}" "${verdict}" "${score}" "${alt}" "${xy_mean}" "${xy_max}" \
    "${angle_rms}" "${angle_max}" "${rate_rms}" "${rate_max}" \
    "${arm_cmd}" "${arm_actual}" "${arm_required}" "${arm_ratio}" \
    "${ext_frac}" "${ext_dur}" "${analysis_dur}" "${config}" >>"${scoreboard}"
}

is_accepted() {
  local result_json="$1"
  local verdict
  verdict="$(json_value "${result_json}" "verdict")"
  [[ "${verdict}" == "GOOD" ]] && return 0
  [[ "${ACCEPT_OK_INTERMEDIATE}" == "1" && "${verdict}" == "OK" ]] && return 0
  return 1
}

run_stage() {
  local phase="$1"
  local stage="$2"
  local trials="$3"
  local arm_amp="$4"
  local base_config="$5"
  local seed_offset="$6"
  local phase_dir="${OUTPUT_ROOT}/${phase}"
  local result_json="${phase_dir}/best_result.json"
  local best_yaml="${phase_dir}/best_uam_controller_params.yaml"
  local seed_value=$((SEED + seed_offset))
  local stop_flag=()

  if [[ "${STOP_WHEN_GOOD}" == "1" ]]; then
    stop_flag=(--stop-when-good)
  fi

  if [[ -f "${result_json}" && -f "${best_yaml}" && "${FORCE}" != "1" ]]; then
    echo "[skip] ${phase}: using existing ${result_json}"
    append_global_scoreboard "${phase}" "${result_json}"
    RUN_STAGE_BEST_YAML="${best_yaml}"
    return 0
  fi

  mkdir -p "${phase_dir}"
  if [[ "${DRY_RUN}" != "1" ]]; then
    stop_sim
  fi

  mapfile -t args < <(common_args "${seed_value}" \
    --stage "${stage}" \
    --trials "${trials}" \
    --base-config "${base_config}" \
    --output-dir "${phase_dir}" \
    --arm-amplitude "${arm_amp}" \
    "${stop_flag[@]}")

  if [[ "${USE_GAZEBO_ARM_VISUAL}" == "1" ]]; then
    args+=(--use-gazebo-arm-visual)
  fi

  echo "[run] ${phase}: stage=${stage}, trials=${trials}, arm_amp=${arm_amp}, seed=${seed_value}, arm_source=${ARM_STATE_SOURCE}"
  if [[ "${DRY_RUN}" == "1" ]]; then
    printf 'python3 %q' "${SCRIPT_DIR}/uam_rerun_autotune.py"
    printf ' %q' "${args[@]}"
    printf '\n'
    RUN_STAGE_BEST_YAML="${base_config}"
    return 0
  fi

  (
    cd "${GZ_ROOT}"
    python3 "${SCRIPT_DIR}/uam_rerun_autotune.py" "${args[@]}"
  )
  stop_sim

  if [[ ! -f "${result_json}" || ! -f "${best_yaml}" ]]; then
    echo "[error] ${phase}: missing best result or YAML" >&2
    return 2
  fi

  append_global_scoreboard "${phase}" "${result_json}"
  if ! is_accepted "${result_json}"; then
    echo "[warn] ${phase}: best verdict=$(json_value "${result_json}" "verdict"), continuing with best candidate"
  fi
  RUN_STAGE_BEST_YAML="${best_yaml}"
}

main() {
  if [[ ! -f "${BASE_CONFIG}" ]]; then
    echo "[error] BASE_CONFIG not found: ${BASE_CONFIG}" >&2
    exit 2
  fi
  if [[ ! -f "${SCRIPT_DIR}/uam_rerun_autotune.py" ]]; then
    echo "[error] Missing helper: ${SCRIPT_DIR}/uam_rerun_autotune.py" >&2
    exit 2
  fi

  mkdir -p "${OUTPUT_ROOT}"
  cat >"${OUTPUT_ROOT}/summary.txt" <<EOF
RBFNN best parameter search
timestamp: ${TIMESTAMP}
base_config: ${BASE_CONFIG}
output_root: ${OUTPUT_ROOT}
quick: ${QUICK}

Phases:
  01_bs_hover: baseline backstepping, no arm feed-forward
  02_rbfnn_fixed: RBFNN residual enabled, arm feedforward off, arm static
  03_rbfnn_arm_ampXXX: RBFNN residual + virtual-arm internal-wrench feedforward, moving arm

Arm model:
  arm_state_source: ${ARM_STATE_SOURCE}
  use_gazebo_arm_visual: ${USE_GAZEBO_ARM_VISUAL}

Scoring gates:
  external_duration >= ${MIN_EXTERNAL_DURATION_S}s, external_fraction >= ${MIN_EXTERNAL_FRACTION}
  arm_actual_span >= max(${ARM_ACTUAL_SPAN_MIN_RAD}, amplitude * ${ARM_ACTUAL_SPAN_MIN_RATIO})
  arm_actual_span / arm_cmd_span >= ${ARM_SPAN_CMD_RATIO_MIN}
EOF

  echo "Output root: ${OUTPUT_ROOT}"
  echo "Base config: ${BASE_CONFIG}"
  echo ""

  local current_base="${BASE_CONFIG}"
  local phase_name=""

  phase_name="01_bs_hover"
  run_stage "${phase_name}" "bs_arm_no_ff" "${BS_TRIALS}" "0.0" "${current_base}" 0
  current_base="${RUN_STAGE_BEST_YAML}"

  phase_name="02_rbfnn_fixed"
  run_stage "${phase_name}" "rbfnn_no_ff" "${RBFNN_FIXED_TRIALS}" "0.0" "${current_base}" 1000
  current_base="${RUN_STAGE_BEST_YAML}"

  local idx=0
  local amp tag
  for amp in ${ARM_AMPLITUDES}; do
    idx=$((idx + 1))
    tag="$(amp_tag "${amp}")"
    phase_name="03_rbfnn_arm_amp${tag}"
    run_stage "${phase_name}" "rbfnn_residual_arm" "${ARM_TRIALS}" "${amp}" "${current_base}" $((2000 + idx * 100))
    current_base="${RUN_STAGE_BEST_YAML}"
  done

  local final_result="${OUTPUT_ROOT}/${phase_name}/best_result.json"
  local final_yaml="${OUTPUT_ROOT}/final_best_uam_controller_params.yaml"
  local final_json="${OUTPUT_ROOT}/final_best_result.json"
  if [[ "${DRY_RUN}" != "1" && -f "${current_base}" && -f "${final_result}" ]]; then
    cp "${current_base}" "${final_yaml}"
    cp "${final_result}" "${final_json}"
    {
      echo ""
      echo "Final best:"
      echo "  config: ${final_yaml}"
      echo "  result: ${final_json}"
      echo "  scoreboard: ${OUTPUT_ROOT}/scoreboard.csv"
    } >>"${OUTPUT_ROOT}/summary.txt"
  fi

  echo ""
  echo "Done."
  echo "Scoreboard: ${OUTPUT_ROOT}/scoreboard.csv"
  if [[ "${DRY_RUN}" != "1" ]]; then
    echo "Final config: ${final_yaml}"
    echo "Final result: ${final_json}"
  fi
}

main "$@"
