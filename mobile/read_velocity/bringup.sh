#!/usr/bin/env bash
set -euo pipefail

WS="/home/pan/ros2_ws"
EKF_YAML="${WS}/src/mobile/config/ekf.yaml"
ROS_DISTRO="${ROS_DISTRO:-humble}"

cleanup() {
  echo ""
  echo "[CLEANUP] Stopping processes..."
  # kill job(s) started by this script (launch)
  jobs -p | xargs -r kill 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[1/5] cd ros2_ws"
cd ros2_ws

echo "[2/5] colcon build"
colcon build 

# echo "[3/5] source ROS + workspace"
# source install/setup.bash

echo "[4/5] start launch: ros2 launch mobile setup.launch.py"
ros2 launch mobile setup.launch.py &
# LAUNCH_PID=$!

# # chờ 1 chút cho node lên ổn định (tăng/giảm tùy máy)
# sleep 2

# echo "[5/5] start EKF with params: ${EKF_YAML}"
# ros2 run robot_localization ekf_node --ros-args --params-file "${EKF_YAML}"