#!/usr/bin/env bash
set -Eeuo pipefail

# =========================================================
# BRINGUP REAL ROBOT + CAMERA + RTAB-MAP LOCALIZATION + NAV2 + WEB
# Jetson target path:
#   /home/pan/ros2_ws/src/mobile/read_velocity/bring_up_nguyen_rtab_with_web.sh
#
# Purpose:
# - Keep stable LiDAR / IMU / CAN / EKF / Web flow.
# - Run camera explicitly for Visual SLAM / RTAB-Map.
# - Replace AMCL localization with RTAB-Map localization.
# - Run Nav2 with RTAB-specific launch / params.
# - Keep old AMCL script untouched.
# =========================================================

# =======================
# CONFIG
# =======================
SESSION="mobile"

WS="$HOME/ros2_ws"
BLUESEA_WS="$WS/src/mobile/bluesea2"

# Unzip the RTAB web zip to this folder. The script auto-detects whether
# index.html is directly inside this folder or inside web_robot_dashboard/.
WEB_BASE="$HOME/web_robot_dashboard_rtab"
WEB_ROOT="$WEB_BASE"

IMU_DEV="/dev/ch34x_imu"
CAN_DEV="/dev/ch34x_can"
LIDAR_DEV="/dev/cp210x_lidar"

# CAN_PY="$WS/src/mobile/read_velocity/can_communicate.py"
CAN_PY="$WS/src/mobile/read_velocity/can_communicate_v0.py"
FIX_USB="/usr/local/sbin/fix-usb-serial.sh"

# RTAB-Map localization and Nav2 navigation launch files.
RTAB_LOCAL_LAUNCH="rtab_local.launch.py"
NAV2_RTAB_LAUNCH="nav2nguyen.launch.py"

# Camera is required for Visual SLAM. If your rtab_local.launch.py already
# starts RealSense, run this script as:
#   RUN_CAMERA_IN_SCRIPT=false ./bring_up_nguyen_rtab_with_web.sh
RUN_CAMERA_IN_SCRIPT="${RUN_CAMERA_IN_SCRIPT:-true}"
CAMERA_COLOR_TOPIC="/camera/camera/color/image_raw"
CAMERA_DEPTH_TOPIC="/camera/camera/aligned_depth_to_color/image_raw"
CAMERA_LAUNCH_CMD="ros2 launch realsense2_camera rs_launch.py camera_name:=camera camera_namespace:=camera enable_color:=true enable_depth:=true align_depth.enable:=true pointcloud.enable:=true rgb_camera.profile:=640x480x30 depth_module.profile:=640x480x30"

WEB_MAPPING_BACKEND="$WS/src/mobile/read_velocity/web_mapping_backend.py"

TIMEOUT_IMU_BUILD=240
TIMEOUT_LIDAR_BUILD=240
TIMEOUT_CAN_START=30
TIMEOUT_EKF_START=60
TIMEOUT_CAMERA_START=60
TIMEOUT_RTAB_LOCAL_START=90
TIMEOUT_NAV2_START=90
TIMEOUT_ODOM_RELAY_START=20
TIMEOUT_WEB_BRIDGE_START=20
TIMEOUT_WEB_MAPPING_BACKEND_START=15
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
need ros2

[[ -d "$WS" ]] || die "Workspace not found: $WS"
[[ -d "$BLUESEA_WS" ]] || die "BlueSea folder not found: $BLUESEA_WS"
[[ -f "$CAN_PY" ]] || die "CAN script not found: $CAN_PY"
[[ -x "$FIX_USB" ]] || die "Fix script missing: $FIX_USB"
[[ -d "$WEB_BASE" ]] || die "Web dashboard folder not found: $WEB_BASE"

if [[ -f "$WEB_BASE/web_robot_dashboard/index.html" ]]; then
  WEB_ROOT="$WEB_BASE/web_robot_dashboard"
elif [[ -f "$WEB_BASE/index.html" ]]; then
  WEB_ROOT="$WEB_BASE"
else
  die "index.html not found in $WEB_BASE or $WEB_BASE/web_robot_dashboard"
fi

[[ -f "$WS/src/mobile/launch/$RTAB_LOCAL_LAUNCH" ]] || die "RTAB launch not found: $WS/src/mobile/launch/$RTAB_LOCAL_LAUNCH"
[[ -f "$WS/src/mobile/launch/$NAV2_RTAB_LAUNCH" ]] || die "Nav2 RTAB launch not found: $WS/src/mobile/launch/$NAV2_RTAB_LAUNCH"
[[ -f "$WEB_MAPPING_BACKEND" ]] || warn "Mapping backend not found: $WEB_MAPPING_BACKEND. Mapping page save/start will not work until you copy it."

# =======================
# FIX USB
# =======================
log "[0/6] Fix USB serial..."

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
log "[1/6] Pre-config serial"

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

log "[2/6] Create tmux session..."
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
# Window: EKF
# -----------------------
tmux new-window -t "$SESSION" -n "ekf"
tmux send-keys -t "$SESSION:ekf" \
  "tmux wait-for STAGE_EKF; \
   cd \"$WS\" && source install/setup.bash && \
   tmux wait-for -S EKF_OK; \
   ros2 launch mobile ekf.launch.py" C-m

# -----------------------
# Window: CAMERA HIGH / RealSense D455
# -----------------------
tmux new-window -t "$SESSION" -n "camera"
tmux send-keys -t "$SESSION:camera" \
  "tmux wait-for STAGE_CAMERA; \
   cd \"$WS\" && source install/setup.bash && \
   if [[ \"$RUN_CAMERA_IN_SCRIPT\" == \"true\" ]]; then \
     echo '[CAMERA] Starting RealSense D455'; \
     echo '[CAMERA] Command: $CAMERA_LAUNCH_CMD'; \
     eval '$CAMERA_LAUNCH_CMD' & \
     CAM_PID=\$!; \
     echo '[CAMERA] Waiting for $CAMERA_COLOR_TOPIC'; \
     for i in {1..60}; do \
       ros2 topic list | grep -q '^$CAMERA_COLOR_TOPIC$' && break; \
       sleep 1; \
     done; \
     tmux wait-for -S CAMERA_OK; \
     echo '[CAMERA] Ready signal sent.'; \
     wait \$CAM_PID; \
   else \
     echo '[CAMERA] RUN_CAMERA_IN_SCRIPT=false, skip camera launch.'; \
     tmux wait-for -S CAMERA_OK; \
     sleep infinity; \
   fi" C-m

# -----------------------
# Window: RTAB LOCALIZATION
# -----------------------
tmux new-window -t "$SESSION" -n "rtab_local"
tmux send-keys -t "$SESSION:rtab_local" \
  "tmux wait-for STAGE_RTAB_LOCAL; \
   cd \"$WS\" && source install/setup.bash && \
   echo '[RTAB_LOCAL] Starting mobile $RTAB_LOCAL_LAUNCH'; \
   ros2 launch mobile $RTAB_LOCAL_LAUNCH & \
   RTAB_PID=\$!; \
   echo '[RTAB_LOCAL] Waiting for RTAB-Map node/topics...'; \
   for i in {1..60}; do \
     ros2 node list | grep -q '/rtabmap/rtabmap' && break; \
     sleep 1; \
   done; \
   sleep 5; \
   tmux wait-for -S RTAB_LOCAL_OK; \
   echo '[RTAB_LOCAL] Ready signal sent.'; \
   wait \$RTAB_PID" C-m

# -----------------------
# Window: NAV2 with RTAB localization
# -----------------------
tmux new-window -t "$SESSION" -n "nav2_rtab"
tmux send-keys -t "$SESSION:nav2_rtab" \
  "tmux wait-for STAGE_NAV2_RTAB; \
   cd \"$WS\" && source install/setup.bash && \
   echo '[NAV2_RTAB] Starting mobile $NAV2_RTAB_LAUNCH'; \
   ros2 launch mobile $NAV2_RTAB_LAUNCH & \
   NAV2_PID=\$!; \
   echo '[NAV2_RTAB] Waiting for Nav2 action server...'; \
   for i in {1..60}; do \
     ros2 action info /navigate_to_pose 2>/dev/null | grep -q 'Action servers: 1' && break; \
     sleep 1; \
   done; \
   tmux wait-for -S NAV2_RTAB_OK; \
   echo '[NAV2_RTAB] Ready signal sent.'; \
   wait \$NAV2_PID" C-m

# -----------------------
# Window: ODOM RELAY
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
# -----------------------
tmux new-window -t "$SESSION" -n "web_bridge"
tmux send-keys -t "$SESSION:web_bridge" \
  "tmux wait-for STAGE_WEB_BRIDGE; \
   cd \"$WS\" && source install/setup.bash && \
   tmux wait-for -S WEB_BRIDGE_OK; \
   echo '[WEB_BRIDGE] Starting web_goal_to_nav2'; \
   ros2 run web_nav_bridge web_goal_to_nav2" C-m

# -----------------------
# Window: WEB MAPPING BACKEND
# -----------------------
tmux new-window -t "$SESSION" -n "web_mapping"
tmux send-keys -t "$SESSION:web_mapping" \
  "tmux wait-for STAGE_WEB_MAPPING_BACKEND; \
   cd \"$WS\" && source install/setup.bash && \
   tmux wait-for -S WEB_MAPPING_BACKEND_OK; \
   if [[ -f \"$WEB_MAPPING_BACKEND\" ]]; then \
     echo '[WEB_MAPPING] Starting web_mapping_backend.py'; \
     python3 \"$WEB_MAPPING_BACKEND\"; \
   else \
     echo '[WEB_MAPPING] Missing $WEB_MAPPING_BACKEND'; \
     sleep infinity; \
   fi" C-m

# -----------------------
# Window: WEB VIDEO SERVER
# -----------------------
tmux new-window -t "$SESSION" -n "web_video"
tmux send-keys -t "$SESSION:web_video" \
  "tmux wait-for STAGE_WEB_VIDEO; \
   cd \"$WS\" && source install/setup.bash && \
   tmux wait-for -S WEB_VIDEO_OK; \
   echo '[WEB_VIDEO] http://0.0.0.0:8080'; \
   ros2 run web_video_server web_video_server" C-m

# -----------------------
# Window: ROSBRIDGE
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
# -----------------------
tmux new-window -t "$SESSION" -n "web_server"
tmux send-keys -t "$SESSION:web_server" \
  "tmux wait-for STAGE_WEB_SERVER; \
   cd \"$WEB_ROOT\" && \
   tmux wait-for -S WEB_SERVER_OK; \
   IP=\$(hostname -I | awk '{print \$1}'); \
   echo '[WEB_SERVER] Open: http://'\$IP':8000'; \
   python3 -m http.server 8000 --bind 0.0.0.0" C-m

# =======================
# ORCHESTRATE ORDER
# =======================
log "[3/6] Run order: IMU -> LiDAR -> CAN -> EKF -> CAMERA -> RTAB_LOCAL -> NAV2_RTAB -> ODOM_RELAY -> WEB_BRIDGE -> WEB_MAPPING -> WEB_VIDEO -> ROSBRIDGE -> WEB_SERVER"

tmux wait-for -S STAGE_IMU
timeout "${TIMEOUT_IMU_BUILD}s" tmux wait-for IMU_OK || die "IMU stage timed out"

tmux wait-for -S STAGE_LIDAR
timeout "${TIMEOUT_LIDAR_BUILD}s" tmux wait-for LIDAR_OK || die "LiDAR stage timed out"

tmux wait-for -S STAGE_CAN
timeout "${TIMEOUT_CAN_START}s" tmux wait-for CAN_OK || die "CAN stage timed out"
sleep 2

tmux wait-for -S STAGE_EKF
timeout "${TIMEOUT_EKF_START}s" tmux wait-for EKF_OK || die "EKF stage timed out"
sleep 4

tmux wait-for -S STAGE_CAMERA
timeout "${TIMEOUT_CAMERA_START}s" tmux wait-for CAMERA_OK || die "CAMERA stage timed out"
sleep 3

tmux wait-for -S STAGE_RTAB_LOCAL
timeout "${TIMEOUT_RTAB_LOCAL_START}s" tmux wait-for RTAB_LOCAL_OK || die "RTAB_LOCAL stage timed out"
sleep 5

tmux wait-for -S STAGE_NAV2_RTAB
timeout "${TIMEOUT_NAV2_START}s" tmux wait-for NAV2_RTAB_OK || die "NAV2_RTAB stage timed out"
sleep 3

tmux wait-for -S STAGE_ODOM_RELAY
timeout "${TIMEOUT_ODOM_RELAY_START}s" tmux wait-for ODOM_RELAY_OK || die "ODOM_RELAY stage timed out"

tmux wait-for -S STAGE_WEB_BRIDGE
timeout "${TIMEOUT_WEB_BRIDGE_START}s" tmux wait-for WEB_BRIDGE_OK || die "WEB_BRIDGE stage timed out"

tmux wait-for -S STAGE_WEB_MAPPING_BACKEND
timeout "${TIMEOUT_WEB_MAPPING_BACKEND_START}s" tmux wait-for WEB_MAPPING_BACKEND_OK || die "WEB_MAPPING_BACKEND stage timed out"

tmux wait-for -S STAGE_WEB_VIDEO
timeout "${TIMEOUT_WEB_VIDEO_START}s" tmux wait-for WEB_VIDEO_OK || die "WEB_VIDEO stage timed out"

tmux wait-for -S STAGE_ROSBRIDGE
timeout "${TIMEOUT_ROSBRIDGE_START}s" tmux wait-for ROSBRIDGE_OK || die "ROSBRIDGE stage timed out"

tmux wait-for -S STAGE_WEB_SERVER
timeout "${TIMEOUT_WEB_SERVER_START}s" tmux wait-for WEB_SERVER_OK || die "WEB_SERVER stage timed out"

log "[4/6] Quick check commands:"
echo "ros2 node list | grep -E 'camera|rtab|planner|controller|bt|nav|web|rosbridge|video'"
echo "ros2 topic list | grep -E 'odom|map|rtabmap|costmap|cmd_vel|web|camera.*image_raw|scan'"
echo "ros2 run tf2_ros tf2_echo map base_footprint"
echo "ros2 action info /navigate_to_pose"
echo "ros2 topic info /cmd_vel -v"
echo "ros2 topic hz $CAMERA_COLOR_TOPIC"

log "[5/6] DONE."
IP=$(hostname -I | awk '{print $1}')
echo "Web    : http://${IP}:8000"
echo "ROS WS : ws://${IP}:9090"
echo "Video  : http://${IP}:8080/stream?topic=${CAMERA_COLOR_TOPIC}&type=mjpeg"
echo "Attach : tmux attach -t ${SESSION}"
echo "Detach : Ctrl-b rồi nhấn d"
echo "Kill   : tmux kill-session -t ${SESSION}"

log "[6/6] Localization mode: RTAB-Map replaces AMCL. Use old bring_up_nguyen_with_web.sh for AMCL mode."
