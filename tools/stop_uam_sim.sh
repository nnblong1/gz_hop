#!/usr/bin/env bash
set -euo pipefail

patterns=(
  "px4_sitl"
  "bin/px4"
  "px4 starting"
  "gz sim"
  "gz-server"
  "gz-gui"
  "MicroXRCEAgent"
  "ros2 launch uam_controller uam_qgc_mode.launch.py"
  "uam_backstepping_rbfnn_node"
  "arm_dynamics_node.py"
  "arm_gazebo_command_node.py"
  "arm_gazebo_joint_state_bridge.py"
  "arm_virtual_state_node.py"
  "arm_initial_pose.py"
  "uam_telemetry_monitor.py"
  "rbfnn_data_logger.py"
  "qgc_rbfnn_trigger.py"
  "arm_trajectory_generator.py"
)

for pattern in "${patterns[@]}"; do
  pkill -TERM -f "${pattern}" >/dev/null 2>&1 || true
done

sleep 2

for pattern in "${patterns[@]}"; do
  pkill -KILL -f "${pattern}" >/dev/null 2>&1 || true
done

echo "UAM simulation processes stopped."
