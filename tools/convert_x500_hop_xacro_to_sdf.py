#!/usr/bin/env python3
"""Recover the x500_hop Gazebo model from the uploaded Xacro backup.

Input expected after the workspace loss:
  Tools/simulation/gz/x500_hop/uav_arm.xacro
  Tools/simulation/gz/x500_hop/mesh_backup/meshes/*.stl

Output:
  Tools/simulation/gz/models/x500_hop/model.sdf
  Tools/simulation/gz/models/x500_hop/model.config
  Tools/simulation/gz/models/x500_hop/mesh_backup/meshes/*.stl

The generated SDF preserves the visual link names from the CAD/Xacro model,
adds PX4 Gazebo sensors and motor plugins, and uses a simple base collision to
avoid CAD mesh contact lock during takeoff.
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path
from xml.sax.saxutils import escape


GZ_ROOT = Path("/home/wicom/PX4-Autopilot/Tools/simulation/gz")
DEFAULT_SOURCE_DIR = GZ_ROOT / "x500_hop"
DEFAULT_SOURCE_XACRO = DEFAULT_SOURCE_DIR / "uav_arm.xacro"
DEFAULT_MODEL_DIR = GZ_ROOT / "models/x500_hop"
DEFAULT_CONVERTED_DIR = DEFAULT_MODEL_DIR / "converted"

ARM_JOINT_RENAME = {
    "Revolute_28": "Revolute_20",
    "Revolute_30": "Revolute_22",
    "Revolute_31": "Revolute_23",
    "Revolute_34": "Revolute_26",
    "Revolute_36": "Revolute_28",
    "Revolute_38": "Revolute_30",
}

ARM_JOINTS = [
    "Revolute_20",
    "Revolute_22",
    "Revolute_23",
    "Revolute_26",
    "Revolute_28",
    "Revolute_30",
]

ROTOR_JOINT_OVERRIDES = {
    # Keep recovered rotor links, but attach rotor joints directly to base_link
    # like the previously working x500_hop logic. This avoids losing Gazebo
    # thrust through a deep fixed-link CAD chain.
    "Revolute_14": ("base_link", "0.158912 0.15890099999999999 0.69135100000000005 0 0 0"),
    "Revolute_15": ("base_link", "-0.15890099999999999 0.158912 0.69135100000000005 0 0 0"),
    "Revolute_16": ("base_link", "-0.158912 -0.15890099999999999 0.69135100000000005 0 0 0"),
    "Revolute_17": ("base_link", "0.15890099999999999 -0.158912 0.69135100000000005 0 0 0"),
}

PX4_SENSOR_BLOCK = """
      <sensor name="air_pressure_sensor" type="air_pressure">
        <always_on>1</always_on>
        <update_rate>50</update_rate>
        <air_pressure>
          <pressure>
            <noise type="gaussian">
              <mean>0</mean>
              <stddev>3</stddev>
            </noise>
          </pressure>
        </air_pressure>
      </sensor>
      <sensor name="magnetometer_sensor" type="magnetometer">
        <always_on>1</always_on>
        <update_rate>100</update_rate>
        <magnetometer>
          <x><noise type="gaussian"><stddev>0.0001</stddev></noise></x>
          <y><noise type="gaussian"><stddev>0.0001</stddev></noise></y>
          <z><noise type="gaussian"><stddev>0.0001</stddev></noise></z>
        </magnetometer>
      </sensor>
      <sensor name="imu_sensor" type="imu">
        <always_on>1</always_on>
        <update_rate>250</update_rate>
        <imu>
          <angular_velocity>
            <x><noise type="gaussian"><mean>0.0</mean><stddev>0.0008726646</stddev></noise></x>
            <y><noise type="gaussian"><mean>0.0</mean><stddev>0.0008726646</stddev></noise></y>
            <z><noise type="gaussian"><mean>0.0</mean><stddev>0.0008726646</stddev></noise></z>
          </angular_velocity>
          <linear_acceleration>
            <x><noise type="gaussian"><mean>0.0</mean><stddev>0.00637</stddev></noise></x>
            <y><noise type="gaussian"><mean>0.0</mean><stddev>0.00637</stddev></noise></y>
            <z><noise type="gaussian"><mean>0.0</mean><stddev>0.00686</stddev></noise></z>
          </linear_acceleration>
        </imu>
      </sensor>
      <sensor name="navsat_sensor" type="navsat">
        <always_on>1</always_on>
        <update_rate>30</update_rate>
      </sensor>
"""


def normalize_joint_name(name: str) -> str:
    name = re.sub(r"\b(Revolute|Rigid) ([0-9]+)\b", r"\1_\2", name)
    name = name.replace(" ", "_")
    return ARM_JOINT_RENAME.get(name, name)


def attr(elem: ET.Element | None, name: str, default: str) -> str:
    if elem is None:
        return default
    return elem.attrib.get(name, default)


def origin_pose(elem: ET.Element | None) -> str:
    origin = elem.find("origin") if elem is not None else None
    xyz = attr(origin, "xyz", "0 0 0")
    rpy = attr(origin, "rpy", "0 0 0")
    return f"{xyz} {rpy}"


def fnum(value: str | float) -> str:
    return f"{float(value):.17g}"


def xml_tag(lines: list[str], indent: int, text: str) -> None:
    lines.append(" " * indent + text)


def inertia_is_positive_definite(vals: dict[str, float]) -> bool:
    ixx = vals.get("ixx", 0.0)
    ixy = vals.get("ixy", 0.0)
    ixz = vals.get("ixz", 0.0)
    iyy = vals.get("iyy", 0.0)
    iyz = vals.get("iyz", 0.0)
    izz = vals.get("izz", 0.0)
    minor2 = ixx * iyy - ixy * ixy
    det = (
        ixx * (iyy * izz - iyz * iyz)
        - ixy * (ixy * izz - iyz * ixz)
        + ixz * (ixy * iyz - iyy * ixz)
    )
    return ixx > 0.0 and minor2 > 0.0 and det > 0.0


def sanitize_inertia_values(vals: dict[str, float]) -> dict[str, float]:
    ixx = max(vals.get("ixx", 0.0), 1e-9)
    iyy = max(vals.get("iyy", 0.0), 1e-9)
    izz = max(vals.get("izz", 0.0), 1e-9)
    eps = 1e-12
    if ixx + iyy <= izz:
        izz = max(1e-9, (ixx + iyy) * 0.999)
    if ixx + izz <= iyy:
        iyy = max(1e-9, (ixx + izz) * 0.999)
    if iyy + izz <= ixx:
        ixx = max(1e-9, (iyy + izz) * 0.999)
    out = dict(vals)
    out["ixx"] = ixx + eps
    out["iyy"] = iyy + eps
    out["izz"] = izz + eps
    if not inertia_is_positive_definite(out):
        out["ixy"] = 0.0
        out["ixz"] = 0.0
        out["iyz"] = 0.0
        max_diag = max(out["ixx"], out["iyy"], out["izz"], 1e-6)
        out["ixx"] = max(out["ixx"], max_diag * 0.1, 1e-6)
        out["iyy"] = max(out["iyy"], max_diag * 0.1, 1e-6)
        out["izz"] = max(out["izz"], max_diag * 0.1, 1e-6)
    return out


def copy_uploaded_model(source_dir: Path, model_dir: Path) -> None:
    source_dir = source_dir.resolve()
    model_dir = model_dir.resolve()
    mesh_src = source_dir / "mesh_backup"
    mesh_dst = model_dir / "mesh_backup"
    if not mesh_src.exists():
        raise FileNotFoundError(mesh_src)
    model_dir.mkdir(parents=True, exist_ok=True)

    # If source and destination are the same model directory, keep mesh_backup
    # in place. Moving it would delete the source we are converting from.
    if source_dir != model_dir:
        if mesh_dst.exists():
            backup = model_dir / f"mesh_backup.before_recover_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            shutil.move(str(mesh_dst), str(backup))
            print(f"Backed up old meshes: {backup}")
        shutil.copytree(mesh_src, mesh_dst)
        shutil.copy2(source_dir / "uav_arm.xacro", model_dir / "uav_arm.xacro")

    meshes = mesh_dst / "meshes"
    alias = meshes / "base_link.stl"
    actual = meshes / "base_link (1).stl"
    if not alias.exists() and actual.exists():
        shutil.copy2(actual, alias)


def sanitize_xacro(src: Path, dst: Path) -> None:
    text = src.read_text(encoding="utf-8", errors="ignore")
    out_lines: list[str] = []
    for line in text.splitlines():
        if "<xacro:include" in line:
            continue
        line = line.replace(
            "package://uav_arm_description/meshes/",
            "model://x500_hop/mesh_backup/meshes/",
        )
        out_lines.append(line)
    text = "\n".join(out_lines) + "\n"
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(text, encoding="utf-8")


def add_geometry(lines: list[str], indent: int, geom: ET.Element | None) -> None:
    xml_tag(lines, indent, "<geometry>")
    mesh = geom.find("mesh") if geom is not None else None
    if mesh is not None:
        xml_tag(lines, indent + 2, "<mesh>")
        xml_tag(lines, indent + 4, f"<uri>{escape(attr(mesh, 'filename', ''))}</uri>")
        scale = attr(mesh, "scale", "")
        if scale:
            xml_tag(lines, indent + 4, f"<scale>{escape(scale)}</scale>")
        xml_tag(lines, indent + 2, "</mesh>")
    else:
        xml_tag(lines, indent + 2, "<box><size>0.001 0.001 0.001</size></box>")
    xml_tag(lines, indent, "</geometry>")


def add_simple_takeoff_collision(lines: list[str]) -> None:
    xml_tag(lines, 6, "<collision name='simple_takeoff_collision'>")
    xml_tag(lines, 8, "<pose>0 0 0.08 0 0 0</pose>")
    xml_tag(lines, 8, "<geometry>")
    xml_tag(lines, 10, "<box><size>0.32 0.32 0.04</size></box>")
    xml_tag(lines, 8, "</geometry>")
    xml_tag(lines, 8, "<surface>")
    xml_tag(lines, 10, "<contact>")
    xml_tag(lines, 12, "<ode>")
    xml_tag(lines, 14, "<kp>100000</kp>")
    xml_tag(lines, 14, "<kd>1</kd>")
    xml_tag(lines, 14, "<max_vel>0.1</max_vel>")
    xml_tag(lines, 14, "<min_depth>0.001</min_depth>")
    xml_tag(lines, 12, "</ode>")
    xml_tag(lines, 10, "</contact>")
    xml_tag(lines, 10, "<friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction>")
    xml_tag(lines, 8, "</surface>")
    xml_tag(lines, 6, "</collision>")


def build_link_block(link: ET.Element, *, relative_to: str | None, keep_mesh_collisions: bool) -> list[str]:
    lines: list[str] = []
    name = link.attrib["name"]
    xml_tag(lines, 4, f"<link name='{escape(name)}'>")
    if relative_to:
        xml_tag(lines, 6, f"<pose relative_to='{escape(relative_to)}'>0 0 0 0 0 0</pose>")
    xml_tag(lines, 6, "<gravity>true</gravity>")
    xml_tag(lines, 6, "<self_collide>false</self_collide>")
    xml_tag(lines, 6, "<velocity_decay/>")

    inertial = link.find("inertial")
    if inertial is not None:
        mass = max(float(attr(inertial.find("mass"), "value", "0")), 1e-9)
        inertia = inertial.find("inertia")
        vals = {
            key: float(attr(inertia, key, "0"))
            for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
        }
        vals = sanitize_inertia_values(vals)
        xml_tag(lines, 6, "<inertial>")
        xml_tag(lines, 8, f"<pose>{escape(origin_pose(inertial))}</pose>")
        xml_tag(lines, 8, f"<mass>{fnum(mass)}</mass>")
        xml_tag(lines, 8, "<inertia>")
        for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
            xml_tag(lines, 10, f"<{key}>{fnum(vals[key])}</{key}>")
        xml_tag(lines, 8, "</inertia>")
        xml_tag(lines, 6, "</inertial>")

    for idx, visual in enumerate(link.findall("visual")):
        xml_tag(lines, 6, f"<visual name='{escape(name)}_visual_{idx}'>")
        xml_tag(lines, 8, f"<pose>{escape(origin_pose(visual))}</pose>")
        add_geometry(lines, 8, visual.find("geometry"))
        xml_tag(lines, 8, "<material>")
        xml_tag(lines, 10, "<ambient>0.7 0.7 0.7 1</ambient>")
        xml_tag(lines, 10, "<diffuse>0.7 0.7 0.7 1</diffuse>")
        xml_tag(lines, 8, "</material>")
        xml_tag(lines, 6, "</visual>")

    if keep_mesh_collisions:
        for idx, collision in enumerate(link.findall("collision")):
            xml_tag(lines, 6, f"<collision name='{escape(name)}_collision_{idx}'>")
            xml_tag(lines, 8, f"<pose>{escape(origin_pose(collision))}</pose>")
            add_geometry(lines, 8, collision.find("geometry"))
            xml_tag(lines, 6, "</collision>")
    elif name == "base_link":
        add_simple_takeoff_collision(lines)

    if name == "base_link":
        lines.extend(PX4_SENSOR_BLOCK.rstrip("\n").splitlines())

    xml_tag(lines, 4, "</link>")
    return lines


def build_joint_block(joint: ET.Element) -> list[str]:
    lines: list[str] = []
    name = normalize_joint_name(joint.attrib["name"])
    urdf_type = joint.attrib.get("type", "fixed")
    sdf_type = "fixed" if urdf_type == "fixed" else "revolute"
    parent = joint.find("parent").attrib["link"]
    child = joint.find("child").attrib["link"]
    pose = origin_pose(joint)
    if name in ROTOR_JOINT_OVERRIDES:
        parent, pose = ROTOR_JOINT_OVERRIDES[name]

    xml_tag(lines, 4, f"<joint name='{escape(name)}' type='{sdf_type}'>")
    xml_tag(lines, 6, f"<pose relative_to='{escape(parent)}'>{escape(pose)}</pose>")
    xml_tag(lines, 6, f"<parent>{escape(parent)}</parent>")
    xml_tag(lines, 6, f"<child>{escape(child)}</child>")

    if sdf_type == "revolute":
        axis = joint.find("axis")
        limit = joint.find("limit")
        xml_tag(lines, 6, "<axis>")
        xml_tag(lines, 8, f"<xyz>{escape(attr(axis, 'xyz', '0 0 1'))}</xyz>")
        xml_tag(lines, 8, "<limit>")
        if name in ROTOR_JOINT_OVERRIDES:
            xml_tag(lines, 10, "<lower>-1e+16</lower>")
            xml_tag(lines, 10, "<upper>1e+16</upper>")
        elif urdf_type == "continuous":
            xml_tag(lines, 10, "<lower>-inf</lower>")
            xml_tag(lines, 10, "<upper>inf</upper>")
        else:
            xml_tag(lines, 10, f"<lower>{escape(attr(limit, 'lower', '-inf'))}</lower>")
            xml_tag(lines, 10, f"<upper>{escape(attr(limit, 'upper', 'inf'))}</upper>")
            xml_tag(lines, 10, f"<effort>{escape(attr(limit, 'effort', '100'))}</effort>")
            xml_tag(lines, 10, f"<velocity>{escape(attr(limit, 'velocity', '100'))}</velocity>")
        xml_tag(lines, 8, "</limit>")
        xml_tag(lines, 8, "<dynamics>")
        if name in ARM_JOINTS:
            xml_tag(lines, 10, "<damping>5.0</damping>")
            xml_tag(lines, 10, "<friction>0.5</friction>")
        xml_tag(lines, 10, "<spring_reference>0</spring_reference>")
        xml_tag(lines, 10, "<spring_stiffness>0</spring_stiffness>")
        xml_tag(lines, 8, "</dynamics>")
        xml_tag(lines, 6, "</axis>")

    xml_tag(lines, 4, "</joint>")
    return lines


def build_px4_plugins() -> list[str]:
    plugins: list[str] = []
    motor_plugins = [
        ("Revolute_15", "quat_2_1", "cw", 0),
        ("Revolute_17", "quat_4_1", "ccw", 1),
        ("Revolute_16", "quat_1_1", "ccw", 2),
        ("Revolute_14", "quat_3_1", "cw", 3),
    ]
    for joint, link, direction, motor_number in motor_plugins:
        plugins.append(
            f"""
    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>{joint}</jointName>
      <linkName>{link}</linkName>
      <turningDirection>{direction}</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>2200</maxRotVelocity>
      <motorConstant>3.7e-06</motorConstant>
      <momentConstant>0.016</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>{motor_number}</motorNumber>
      <rotorDragCoefficient>8.06e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>"""
        )

    for joint in ARM_JOINTS:
        plugins.append(
            f"""
    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>{joint}</joint_name>
      <topic>/model/x500_hop/joint/{joint}/cmd_pos</topic>
      <p_gain>50.0</p_gain>
      <i_gain>1.0</i_gain>
      <d_gain>2.0</d_gain>
      <cmd_max>20.0</cmd_max>
      <cmd_min>-20.0</cmd_min>
    </plugin>"""
        )

    joint_names = "\n".join(f"      <joint_name>{joint}</joint_name>" for joint in ARM_JOINTS)
    plugins.append(
        f"""
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
{joint_names}
      <topic>/model/x500_hop/joint_state</topic>
      <update_rate>50</update_rate>
    </plugin>"""
    )
    plugins.append(
        """
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <odom_topic>/model/x500_hop/odometry</odom_topic>
      <tf_topic>/model/x500_hop/tf</tf_topic>
      <update_rate>50</update_rate>
      <dimensions>3</dimensions>
    </plugin>"""
    )
    plugins.append(
        """
    <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
      <publish_link_pose>true</publish_link_pose>
      <publish_visual_pose>false</publish_visual_pose>
      <publish_collision_pose>false</publish_collision_pose>
      <publish_sensor_pose>false</publish_sensor_pose>
      <update_rate>50</update_rate>
      <topic>/model/x500_hop/pose</topic>
    </plugin>"""
    )
    return "\n".join(plugins).splitlines()


def write_sdf(urdf: Path, sdf: Path, *, model_pose: str, keep_mesh_collisions: bool) -> None:
    root = ET.parse(urdf).getroot()
    links = {link.attrib["name"]: link for link in root.findall("link")}
    joints = root.findall("joint")
    children_by_parent: dict[str, list[ET.Element]] = {}
    child_to_joint: dict[str, ET.Element] = {}
    for joint in joints:
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        children_by_parent.setdefault(parent, []).append(joint)
        child_to_joint[child] = joint

    lines: list[str] = [
        "<sdf version='1.11'>",
        "  <model name='x500_hop' canonical_link='base_link'>",
    ]
    xml_tag(lines, 4, f"<pose>{escape(model_pose)}</pose>")
    xml_tag(lines, 4, "<static>false</static>")
    xml_tag(lines, 4, "<self_collide>false</self_collide>")

    emitted: set[str] = set()

    def emit_link_tree(link_name: str, relative_to: str | None = None) -> None:
        if link_name in emitted:
            return
        lines.extend(
            build_link_block(
                links[link_name],
                relative_to=relative_to,
                keep_mesh_collisions=keep_mesh_collisions,
            )
        )
        emitted.add(link_name)
        for joint in children_by_parent.get(link_name, []):
            joint_name = normalize_joint_name(joint.attrib["name"])
            child = joint.find("child").attrib["link"]
            lines.extend(build_joint_block(joint))
            emit_link_tree(child, relative_to=joint_name)

    roots = [name for name in links if name not in child_to_joint]
    for root_name in roots:
        emit_link_tree(root_name)
    for link_name in links:
        emit_link_tree(link_name)

    lines.extend(build_px4_plugins())
    lines.append("  </model>")
    lines.append("</sdf>")
    sdf.parent.mkdir(parents=True, exist_ok=True)
    sdf.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_model_config(model_dir: Path) -> None:
    text = """<?xml version="1.0"?>
<model>
  <name>x500_hop</name>
  <version>1.0</version>
  <sdf version="1.11">model.sdf</sdf>
  <author>
    <name>UAM recovered model</name>
    <email>none</email>
  </author>
  <description>Recovered x500_hop UAV-arm model with PX4 Gazebo plugins.</description>
</model>
"""
    (model_dir / "model.config").write_text(text, encoding="utf-8")


def check_meshes(sdf_path: Path, model_dir: Path) -> list[str]:
    text = sdf_path.read_text(encoding="utf-8", errors="ignore")
    refs = sorted(set(re.findall(r"model://x500_hop/([^<]+\.stl)", text)))
    return [ref for ref in refs if not (model_dir / ref).exists()]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-dir", type=Path, default=DEFAULT_SOURCE_DIR)
    parser.add_argument("--source-xacro", type=Path, default=DEFAULT_SOURCE_XACRO)
    parser.add_argument("--model-dir", type=Path, default=DEFAULT_MODEL_DIR)
    parser.add_argument("--converted-dir", type=Path, default=DEFAULT_CONVERTED_DIR)
    parser.add_argument(
        "--model-pose",
        default="0 0 0.85 0 0 0",
        help=(
            "Initial SDF model pose. Keep this above the visual landing stand; "
            "the stand is visual-only and does not physically support the UAV."
        ),
    )
    parser.add_argument("--keep-mesh-collisions", action="store_true")
    args = parser.parse_args()

    if not args.source_xacro.exists():
        raise FileNotFoundError(args.source_xacro)

    if (args.model_dir / "model.sdf").exists():
        backup = args.model_dir / f"model.sdf.before_recover_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        shutil.copy2(args.model_dir / "model.sdf", backup)
        print(f"Backed up old model.sdf: {backup}")

    copy_uploaded_model(args.source_dir, args.model_dir)

    sanitized = args.converted_dir / "uav_arm_sanitized.urdf"
    final_sdf = args.model_dir / "model.sdf"
    sanitize_xacro(args.model_dir / "uav_arm.xacro", sanitized)
    write_sdf(
        sanitized,
        final_sdf,
        model_pose=args.model_pose,
        keep_mesh_collisions=args.keep_mesh_collisions,
    )
    write_model_config(args.model_dir)

    missing = check_meshes(final_sdf, args.model_dir)
    check = subprocess.run(
        ["gz", "sdf", "-k", str(final_sdf)],
        capture_output=True,
        text=True,
        check=False,
    )

    print(f"Source xacro : {args.source_xacro}")
    print(f"Sanitized URDF: {sanitized}")
    print(f"Model SDF    : {final_sdf}")
    print(f"Model config : {args.model_dir / 'model.config'}")
    print(f"Missing mesh : {len(missing)}")
    for ref in missing:
        print(f"  - {ref}")
    print("SDF check    :")
    print((check.stdout + check.stderr).strip() or f"exit={check.returncode}")

    if missing or check.returncode != 0:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
