#!/usr/bin/env bash
set -euo pipefail

SESSION="mobile"

# ====== Ports ======
LIDAR_DEV="/dev/cp210x_lidar"
CAN_DEV="/dev/ch34x_can"
IMU_DEV="/dev/ch34x_imu"

# ====== Workspace ======
WS="$HOME/ros2_ws"

# (Tuỳ bạn) source ROS distro trước nếu cần:
# source /opt/ros/humble/setup.bash

echo "[1/3] Configure serial ports..."
sudo stty -F "$LIDAR_DEV" 921600 raw -echo
sudo stty -F "$CAN_DEV"   2000000 raw -echo
sudo stty -F "$IMU_DEV"   115200  raw -echo

echo "[2/3] Build once (incremental)..."
cd "$WS"
colcon build
source install/setup.bash

echo "[3/3] Launch everything in tmux session: $SESSION"

# Nếu session đã tồn tại thì không tạo mới
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "Session '$SESSION' already exists."
  echo "Attach with: tmux attach -t $SESSION"
  exit 0
fi

# Tạo session mới (detached)
tmux new-session -d -s "$SESSION" -n "can"

# Window 1: CAN python
tmux send-keys -t "$SESSION:can" "cd $WS && source install/setup.bash && python3 $WS/src/mobile/read_velocity/can_communicate.py" C-m

# Window 2: IMU wt901
tmux new-window -t "$SESSION" -n "imu"
tmux send-keys -t "$SESSION:imu" "cd $WS && source install/setup.bash && ros2 run wt901_ros2 wt901_node" C-m

# Window 3: LiDAR bluesea2
tmux new-window -t "$SESSION" -n "lidar"
tmux send-keys -t "$SESSION:lidar" "cd $WS && source install/setup.bash && ros2 launch bluesea2 uart_lidar.launch" C-m

# Window 4: SLAM
tmux new-window -t "$SESSION" -n "slam"
tmux send-keys -t "$SESSION:slam" "cd $WS && source install/setup.bash && ros2 launch mobile slam_toolbox.launch.py" C-m

# Window 5: Bringup
tmux new-window -t "$SESSION" -n "bringup"
tmux send-keys -t "$SESSION:bringup" "cd $WS && source install/setup.bash && ros2 launch mobile bring_up.launch.py" C-m

echo "Done."
echo "Attach: tmux attach -t $SESSION"
echo "Detach (giữ chạy nền): Ctrl-b rồi nhấn d"