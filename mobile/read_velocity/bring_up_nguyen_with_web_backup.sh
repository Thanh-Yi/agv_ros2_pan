#!/usr/bin/env bash
set -Eeuo pipefail

# =======================
# CONFIG
# =======================
SESSION="mobile"

WS="$HOME/ros2_ws"
BLUESEA_WS="$WS/src/mobile/bluesea2"
WEB_DIR="$HOME/web_robot_dashboard_real_model_fix"

IMU_DEV="/dev/ch34x_imu"
CAN_DEV="/dev/ch34x_can"
LIDAR_DEV="/dev/cp210x_lidar"

# CAN_PY="$WS/src/mobile/read_velocity/can_communicate.py"
CAN_PY="$WS/src/mobile/read_velocity/can_communicate_v0.py"
FIX_USB="/usr/local/sbin/fix-usb-serial.sh"

TIMEOUT_IMU_BUILD=240
TIMEOUT_LIDAR_BUILD=240
TIMEOUT_CAN_START=30
TIMEOUT_CAMERA_START=30
TIMEOUT_SLAM_START=60
TIMEOUT_BRINGUP_START=90
TIMEOUT_ODOM_RELAY_START=20
TIMEOUT_WEB_BRIDGE_START=20
TIMEOUT_WEB_VIDEO_START=15
TIMEOUT_ROSBRIDGE_START=25
TIMEOUT_WEB_SERVER_START=10

# =======================
# LOAD ENV
# =======================
echo "[ENV] Loading ROS2 environment..."
set +u
source /opt/ros/humble/setup.bash || { echo "Cannot source ROS"; exit 1; }
source "$WS/install/setup.bash" || { echo "Cannot source WS"; exit 1; }
set -u

export PATH=$PATH:/usr/local/bin
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}:/usr/local/lib

# =======================
# HELPERS
# =======================
log()  { echo -e "\033[1;32m$*\033[0m"; }
warn() { echo -e "\033[1;33m$*\033[0m"; }
die()  { echo -e "\033[1;31m$*\033[0m" >&2; exit 1; }

need() { command -v "$1" >/dev/null 2>&1 || die "Missing '$1'"; }

# =======================
# WAIT DEVICES
# =======================
wait_devs() {
  for _ in {1..50}; do
    if [[ -e "$IMU_DEV" && -e "$CAN_DEV" && -e "$LIDAR_DEV" ]]; then
      IMU_REAL=$(readlink -f "$IMU_DEV" || true)
      CAN_REAL=$(readlink -f "$CAN_DEV" || true)
      LIDAR_REAL=$(readlink -f "$LIDAR_DEV" || true)

      [[ -e "$IMU_REAL" && -e "$CAN_REAL" && -e "$LIDAR_REAL" ]] && return 0
    fi
    sleep 0.1
  done
  return 1
}

# =======================
# CHECKS
# =======================
need tmux
need colcon
need flock
need timeout
need python3

[[ -d "$WS" ]] || die "Workspace not found: $WS"
[[ -d "$BLUESEA_WS" ]] || die "BlueSea folder not found: $BLUESEA_WS"
[[ -f "$CAN_PY" ]] || die "CAN script not found: $CAN_PY"
[[ -x "$FIX_USB" ]] || die "Fix script missing: $FIX_USB"
[[ -d "$WEB_DIR" ]] || die "Web dashboard folder not found: $WEB_DIR"

# =======================
# FIX USB
# =======================
log "[0/5] Fix USB serial..."

for _ in {1..3}; do
  CNT=$(ls /dev/ttyUSB* 2>/dev/null | wc -l)
  [[ "$CNT" -ge 3 ]] && break
  sleep 0.2
done

for i in {1..10}; do
  log "USB fix attempt $i..."

  sudo udevadm control --reload-rules
  sudo udevadm trigger
  sudo udevadm settle
  sleep 2

  sudo "$FIX_USB" || true
  sleep 1

  wait_devs || continue

  IMU_REAL=$(readlink -f "$IMU_DEV" || true)
  CAN_REAL=$(readlink -f "$CAN_DEV" || true)

  [[ "$IMU_REAL" != "$CAN_REAL" ]] && break

  warn "Duplicate detected, retry..."
done

IMU_REAL=$(readlink -f "$IMU_DEV" || true)
CAN_REAL=$(readlink -f "$CAN_DEV" || true)

[[ "$IMU_REAL" == "$CAN_REAL" ]] && die "USB mapping FAILED!"

log "Devices:"
ls -l "$IMU_DEV" "$CAN_DEV" "$LIDAR_DEV"

log "Resolved:"
sleep 4
echo "IMU   -> $IMU_REAL"
echo "CAN   -> $CAN_REAL"
echo "LIDAR -> $(readlink -f "$LIDAR_DEV")"

sleep 3

# =======================
# PRE-STTY
# =======================
log "[1/5] Pre-config serial"

for _ in {1..10}; do
  if timeout 1s stty -F "$LIDAR_DEV" 921600 cs8 -cstopb -parenb -crtscts raw -echo 2>/dev/null; then
    log "stty OK: $LIDAR_DEV"
    break
  fi
  warn "Retry stty..."
  sleep 0.3
done

# =======================
# TMUX
# =======================
if tmux has-session -t "$SESSION" 2>/dev/null; then
  warn "tmux session '$SESSION' already exists."
  echo "Attach: tmux attach -t ${SESSION}"
  echo "Kill  : tmux kill-session -t ${SESSION}"
  exit 0
fi

log "[2/5] Create tmux session..."
tmux new-session -d -s "$SESSION" -n "imu"

# -----------------------
# Window: IMU
# -----------------------
tmux send-keys -t "$SESSION:imu" \
  "tmux wait-for STAGE_IMU; \
   cd \"$WS\" && colcon build && source install/setup.bash && \
   tmux wait-for -S IMU_OK; \
   flock -n /tmp/lock_imu.lock python3 \"$WS/src/mobile/read_velocity/imu.py\" --ros-args -p port:=\"$IMU_DEV\" -p baud:=115200" C-m

# -----------------------
# Window: LiDAR
# -----------------------
tmux new-window -t "$SESSION" -n "lidar"
tmux send-keys -t "$SESSION:lidar" \
  "tmux wait-for STAGE_LIDAR; \
   cd \"$BLUESEA_WS\" && colcon build && source install/setup.bash && \
   tmux wait-for -S LIDAR_OK; \
   flock -n /tmp/lock_lidar.lock ros2 launch bluesea2 uart_lidar.launch" C-m

# -----------------------
# Window: CAN
# -----------------------
tmux new-window -t "$SESSION" -n "can"
tmux send-keys -t "$SESSION:can" \
  "tmux wait-for STAGE_CAN; \
   tmux wait-for -S CAN_OK; \
   flock -n /tmp/lock_can.lock python3 \"$CAN_PY\" \"$CAN_DEV\"" C-m

# -----------------------
# Window: RealSense Camera
# Tạo topic màu: /camera/camera/color/image_raw
# Lệnh này giống lệnh bạn chạy thủ công trong terminal.
# -----------------------
tmux new-window -t "$SESSION" -n "camera"
tmux send-keys -t "$SESSION:camera" \
  "tmux wait-for STAGE_CAMERA; \
   cd \"$WS\" && source install/setup.bash && \
   tmux wait-for -S CAMERA_OK; \
   echo '[CAMERA] Starting RealSense D455'; \
   ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true" C-m

# -----------------------
# Window: EKF
# -----------------------
tmux new-window -t "$SESSION" -n "ekf"
tmux send-keys -t "$SESSION:ekf" \
  "tmux wait-for STAGE_EKF; \
   cd \"$WS\" && source install/setup.bash && \
   tmux wait-for -S EKF_OK; \
   ros2 launch mobile ekf.launch.py" C-m

# -----------------------
# Window: BRINGUP / NAV2
# File này đang dùng nav2nguyen.launch.py theo nội dung bạn gửi.
# -----------------------
tmux new-window -t "$SESSION" -n "bringup"
tmux send-keys -t "$SESSION:bringup" \
  "tmux wait-for STAGE_BRINGUP; \
   cd \"$WS\" && source install/setup.bash && \
   tmux wait-for -S BRINGUP_OK; \
   ros2 launch mobile nav2nguyen.launch.py" C-m

# -----------------------
# Window: ODOM RELAY
# Robot thật đang có /odometry/filtered; web đang cần /odom.
# Nếu thiếu topic_tools: sudo apt install ros-humble-topic-tools
# -----------------------
tmux new-window -t "$SESSION" -n "odom_relay"
tmux send-keys -t "$SESSION:odom_relay" \
  "tmux wait-for STAGE_ODOM_RELAY; \
   cd \"$WS\" && source install/setup.bash && \
   tmux wait-for -S ODOM_RELAY_OK; \
   echo '[ODOM_RELAY] /odometry/filtered -> /odom'; \
   ros2 run topic_tools relay /odometry/filtered /odom" C-m

# -----------------------
# Window: WEB GOAL BRIDGE
# Nhận /web_goal_pose, /web_cancel_goal rồi gửi sang Nav2 /navigate_to_pose.
# -----------------------
tmux new-window -t "$SESSION" -n "web_bridge"
tmux send-keys -t "$SESSION:web_bridge" \
  "tmux wait-for STAGE_WEB_BRIDGE; \
   cd \"$WS\" && source install/setup.bash && \
   tmux wait-for -S WEB_BRIDGE_OK; \
   echo '[WEB_BRIDGE] Starting web_goal_to_nav2'; \
   ros2 run web_nav_bridge web_goal_to_nav2" C-m

# -----------------------
# Window: WEB VIDEO SERVER
# Camera stream: http://JETSON_IP:8080/stream?topic=/camera/camera/color/image_raw
# -----------------------
tmux new-window -t "$SESSION" -n "web_video"
tmux send-keys -t "$SESSION:web_video" \
  "tmux wait-for STAGE_WEB_VIDEO; \
   cd \"$WS\" && source install/setup.bash && \
   tmux wait-for -S WEB_VIDEO_OK; \
   echo '[WEB_VIDEO] http://0.0.0.0:8080'; \
   ros2 run web_video_server web_video_server" C-m

# -----------------------
# Window: WEB SYSTEM MANAGER
# Publish /web_map_list, nhận /web_system_command
# -----------------------

# tmux new-window -t "$SESSION" -n "web_system"
# tmux send-keys -t "$SESSION:web_system" \
#   "cd \"$WS\" && source install/setup.bash && \
#    echo '[WEB_SYSTEM] Starting web_system_manager'; \
#    python3 \"$WS/src/mobile/run_file/web_system_manager.py\"" C-m

# -----------------------
# Window: ROSBRIDGE
# Web HTML/JS kết nối ROS2 qua ws://JETSON_IP:9090
# -----------------------
tmux new-window -t "$SESSION" -n "rosbridge"
tmux send-keys -t "$SESSION:rosbridge" \
  "tmux wait-for STAGE_ROSBRIDGE; \
   cd \"$WS\" && source install/setup.bash && \
   tmux wait-for -S ROSBRIDGE_OK; \
   echo '[ROSBRIDGE] ws://0.0.0.0:9090'; \
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml" C-m

# -----------------------
# Window: WEB SERVER
# Giao diện web: http://JETSON_IP:8000
# -----------------------
tmux new-window -t "$SESSION" -n "web_server"
tmux send-keys -t "$SESSION:web_server" \
  "tmux wait-for STAGE_WEB_SERVER; \
   cd \"$WEB_DIR\" && \
   tmux wait-for -S WEB_SERVER_OK; \
   IP=\$(hostname -I | awk '{print \$1}'); \
   echo '[WEB_SERVER] Open: http://'\$IP':8000'; \
   python3 -m http.server 8000 --bind 0.0.0.0" C-m

# =======================
# ORCHESTRATE ORDER
# =======================
log "[3/5] Run order: IMU -> LiDAR -> CAN -> CAMERA -> EKF -> BRINGUP/NAV2 -> ODOM_RELAY -> WEB_BRIDGE -> WEB_VIDEO -> ROSBRIDGE -> WEB_SERVER"

tmux wait-for -S STAGE_IMU
timeout "${TIMEOUT_IMU_BUILD}s" tmux wait-for IMU_OK || die "IMU stage timed out"

tmux wait-for -S STAGE_LIDAR
timeout "${TIMEOUT_LIDAR_BUILD}s" tmux wait-for LIDAR_OK || die "LiDAR stage timed out"

tmux wait-for -S STAGE_CAN
timeout "${TIMEOUT_CAN_START}s" tmux wait-for CAN_OK || die "CAN stage timed out"

tmux wait-for -S STAGE_CAMERA
timeout "${TIMEOUT_CAMERA_START}s" tmux wait-for CAMERA_OK || die "CAMERA stage timed out"
# Chờ RealSense tạo topic trước khi các phần còn lại bắt đầu.
sleep 3

tmux wait-for -S STAGE_EKF
timeout "${TIMEOUT_SLAM_START}s" tmux wait-for EKF_OK || die "EKF stage timed out"

tmux wait-for -S STAGE_BRINGUP
timeout "${TIMEOUT_BRINGUP_START}s" tmux wait-for BRINGUP_OK || die "BRINGUP stage timed out"

tmux wait-for -S STAGE_ODOM_RELAY
timeout "${TIMEOUT_ODOM_RELAY_START}s" tmux wait-for ODOM_RELAY_OK || die "ODOM_RELAY stage timed out"

tmux wait-for -S STAGE_WEB_BRIDGE
timeout "${TIMEOUT_WEB_BRIDGE_START}s" tmux wait-for WEB_BRIDGE_OK || die "WEB_BRIDGE stage timed out"

tmux wait-for -S STAGE_WEB_VIDEO
timeout "${TIMEOUT_WEB_VIDEO_START}s" tmux wait-for WEB_VIDEO_OK || die "WEB_VIDEO stage timed out"

tmux wait-for -S STAGE_ROSBRIDGE
timeout "${TIMEOUT_ROSBRIDGE_START}s" tmux wait-for ROSBRIDGE_OK || die "ROSBRIDGE stage timed out"

tmux wait-for -S STAGE_WEB_SERVER
timeout "${TIMEOUT_WEB_SERVER_START}s" tmux wait-for WEB_SERVER_OK || die "WEB_SERVER stage timed out"

log "[4/5] Quick check commands:"
echo "ros2 topic list | grep -E 'web|odom|map|amcl|costmap|cmd_vel|camera|image_raw'"
echo "ros2 action info /navigate_to_pose"
echo "ros2 topic info /cmd_vel -v"
echo "ros2 topic info /camera/camera/color/image_raw"
echo "ss -lntp | grep -E '8000|8080|9090'"

log "[5/5] DONE."
IP=$(hostname -I | awk '{print $1}')
echo "Web        : http://${IP}:8000"
echo "Camera URL : http://${IP}:8080/stream?topic=/camera/camera/color/image_raw"
echo "ROS WS     : ws://${IP}:9090"
echo "Attach     : tmux attach -t ${SESSION}"
echo "Detach     : Ctrl-b rồi nhấn d"
echo "Kill       : tmux kill-session -t ${SESSION}"
