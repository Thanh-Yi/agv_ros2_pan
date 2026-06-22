#!/usr/bin/env bash
# start_webui.sh — Chạy 1 lần khi bắt đầu dùng web UI.
# Tạo tmux session "webui" — TÁCH BIỆT với session "mobile".
#
# Cách dùng:
#   bash ~/ros2_ws/src/mobile/read_velocity/start_webui.sh
#
# Để tắt:
#   tmux kill-session -t webui

set -e

SESSION="webui"
WS="$HOME/ros2_ws"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "webui session đang chạy rồi"
  echo "Attach: tmux attach -t $SESSION"
  exit 0
fi

echo "[1/3] Tạo tmux session '$SESSION'..."
tmux new-session -d -s "$SESSION" -n "rosbridge"

# ── Cửa sổ 1: ROSBridge ──────────────────────────────────────────
tmux send-keys -t "$SESSION:rosbridge" "
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
" C-m

sleep 2

# ── Cửa sổ 2: Robot Launcher Node ───────────────────────────────
echo "[2/3] Khởi động robot_launcher_node..."
tmux new-window -t "$SESSION" -n "launcher"
tmux send-keys -t "$SESSION:launcher" "
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
python3 $WS/src/mobile/read_velocity/robot_launcher_node.py
" C-m

sleep 1

# ── Cửa sổ 3: System Stats Publisher ────────────────────────────
echo "[3/3] Khởi động system_stats_publisher..."
tmux new-window -t "$SESSION" -n "stats"
tmux send-keys -t "$SESSION:stats" "
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
python3 $WS/src/mobile/scripts/system_stats_publisher.py
" C-m

echo ""
echo "✓ Web UI services đã sẵn sàng!"
echo ""
echo "  tmux '$SESSION' gồm 3 cửa sổ:"
echo "    rosbridge — cổng kết nối web ↔ ROS (ws://$(hostname -I | awk '{print $1}'):9090)"
echo "    launcher  — nhận lệnh start/stop SLAM và Navigation từ web"
echo "    stats     — publish CPU/RAM/disk lên /system_stats"
echo ""
echo "Attach : tmux attach -t $SESSION"
echo "Detach : Ctrl-b rồi nhấn d"
echo "Kill   : tmux kill-session -t $SESSION"