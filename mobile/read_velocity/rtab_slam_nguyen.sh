#!/usr/bin/env bash
set -eo pipefail

# =========================
# CONFIG
# =========================
WS="$HOME/ros2_ws"
SESSION="camera"

export AMENT_PYTHON_EXECUTABLE=/usr/bin/python3
export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3

# =========================
# SOURCE ROS
# =========================
source /opt/ros/humble/setup.bash

cd "$WS" || exit 1
source install/setup.bash

# =========================
# KILL OLD SESSION
# =========================
tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

# =========================
# START CAMERA
# =========================
echo "[1] Start RealSense Camera"

tmux new-session -d -s "$SESSION" bash -c "
source /opt/ros/humble/setup.bash
cd $WS
source install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  enable_color:=true \
  enable_depth:=true \
  pointcloud.enable:=true \
  initial_reset:=true
"

# =========================
# WAIT CAMERA
# =========================
echo "[2] Wait camera..."
sleep 8

# =========================
# START EKF
# =========================
echo "[3] Start EKF"

tmux new-window -t "$SESSION" -n ekf bash -c "
source /opt/ros/humble/setup.bash
cd $WS
source install/setup.bash

ros2 launch mobile ekf.launch.py
"

# =========================
# START RTABMAP
# =========================
echo "[4] Start RTABMAP GUI"

tmux new-window -t "$SESSION" -n rtabmap bash -c "
source /opt/ros/humble/setup.bash
cd $WS
source install/setup.bash

ros2 launch mobile rtab_mapping.launch.py
"

# =========================
# DONE
# =========================
echo "[5] DONE"
echo "Attach: tmux attach -t $SESSION"
echo "Kill  : tmux kill-session -t $SESSION"