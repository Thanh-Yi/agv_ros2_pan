#!/usr/bin/env bash
set -Eeuo pipefail

# =========================================================
# BRINGUP REAL ROBOT + RTAB-MAP LOCALIZATION + NAV2 + WEB
# New file name suggested on Jetson:
#   /home/pan/ros2_ws/src/mobile/read_velocity/bring_up_nguyen_rtab_with_web.sh
#
# Purpose:
# - Keep the old stable LiDAR / IMU / CAN / EKF / Web flow.
# - Replace AMCL/map_server bringup with RTAB-Map localization.
# - Run Nav2 navigation using nav2nguyen.launch.py.
# - Keep old file bring_up_nguyen_with_web.sh untouched.
# =========================================================

# =======================
# CONFIG
# =======================
SESSION="mobile"

WS="$HOME/ros2_ws"
BLUESEA_WS="$WS/src/mobile/bluesea2"
WEB_DIR="$HOME/web_robot_dashboard_rtab"

IMU_DEV="/dev/ch34x_imu"
CAN_DEV="/dev/ch34x_can"
LIDAR_DEV="/dev/cp210x_lidar"

# Keep your current CAN script choice from the old stable file.
# CAN_PY="$WS/src/mobile/read_velocity/can_communicate.py"
CAN_PY="$WS/src/mobile/read_velocity/can_communicate_v0.py"
FIX_USB="/usr/local/sbin/fix-usb-serial.sh"

# RTAB-Map localization and Nav2 navigation launch files.
RTAB_LOCAL_LAUNCH="rtab_local.launch.py"
NAV2_RTAB_LAUNCH="nav2nguyen.launch.py"

TIMEOUT_IMU_BUILD=240
TIMEOUT_LIDAR_BUILD=240
TIMEOUT_CAN_START=30
TIMEOUT_EKF_START=60
TIMEOUT_RTAB_LOCAL_START=90
TIMEOUT_NAV2_START=90
TIMEOUT_ODOM_RELAY_START=20
TIMEOUT_WEB_BRIDGE_START=20
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
[[ -f "$WS/src/mobile/launch/$RTAB_LOCAL_LAUNCH" ]] || die "RTAB launch not found: $WS/src/mobile/launch/$RTAB_LOCAL_LAUNCH"
[[ -f "$WS/src/mobile/launch/$NAV2_RTAB_LAUNCH" ]] || die "Nav2 RTAB launch not found: $WS/src/mobile/launch/$NAV2_RTAB_LAUNCH"

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
# Build workspace first, then run IMU.
# Other stages wait for IMU_OK, so they source the rebuilt install.
# -----------------------
tmux send-keys -t "$SESSION:imu" \
  "tmux wait-for STAGE_IMU; \
   cd \"$WS\" && colcon build && source install/setup.bash && \
   tmux wait-for -S IMU_OK; \
   flock -n /tmp/lock_imu.lock python3 \"$WS/src/mobile/read_velocity/imu.py\" --ros-args -p port:=\"$IMU_DEV\" -p baud:=115200" C-m

# -----------------------
# Window: LiDAR
# Keep this stable LiDAR section from your old script.
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
# Publishes /odometry/filtered for RTAB-Map and odom relay.
# -----------------------
tmux new-window -t "$SESSION" -n "ekf"
tmux send-keys -t "$SESSION:ekf" \
  "tmux wait-for STAGE_EKF; \
   cd \"$WS\" && source install/setup.bash && \
   tmux wait-for -S EKF_OK; \
   ros2 launch mobile ekf.launch.py" C-m

# -----------------------
# Window: RTAB LOCALIZATION
# This replaces AMCL/map-server localization.
# It runs your file:
#   /home/pan/ros2_ws/src/mobile/launch/rtab_local.launch.py
# That file loads the RTAB database and publishes map->odom TF.
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
# This uses your file:
#   /home/pan/ros2_ws/src/mobile/launch/nav2nguyen.launch.py
# It should run Nav2 navigation stack without AMCL/map_server.
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
# Robot thật đang có /odometry/filtered; web/other nodes may need /odom.
# If topic_tools is missing: sudo apt install ros-humble-topic-tools
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
# Window: WEB SYSTEM MANAGER
# Disabled because map loading is fixed in robot launch/config.
# -----------------------
# tmux new-window -t "$SESSION" -n "web_system"
# tmux send-keys -t "$SESSION:web_system" \
#   "cd \"$WS\" && source install/setup.bash && \
#    echo '[WEB_SYSTEM] Starting web_system_manager'; \
#    python3 \"$WS/src/mobile/run_file/web_system_manager.py\"" C-m

# -----------------------
# Window: ROSBRIDGE
# Web HTML/JS connects ROS2 via ws://JETSON_IP:9090
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
# Web UI: http://JETSON_IP:8000
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
log "[3/5] Run order: IMU -> LiDAR -> CAN -> EKF -> RTAB_LOCAL -> NAV2_RTAB -> ODOM_RELAY -> WEB_BRIDGE -> ROSBRIDGE -> WEB_SERVER"

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

tmux wait-for -S STAGE_ROSBRIDGE
timeout "${TIMEOUT_ROSBRIDGE_START}s" tmux wait-for ROSBRIDGE_OK || die "ROSBRIDGE stage timed out"

tmux wait-for -S STAGE_WEB_SERVER
timeout "${TIMEOUT_WEB_SERVER_START}s" tmux wait-for WEB_SERVER_OK || die "WEB_SERVER stage timed out"

log "[4/5] Quick check commands:"
echo "ros2 node list | grep -E 'rtab|planner|controller|bt|nav|web|rosbridge'"
echo "ros2 topic list | grep -E 'odom|map|rtabmap|costmap|cmd_vel|web'"
echo "ros2 run tf2_ros tf2_echo map base_footprint"
echo "ros2 action info /navigate_to_pose"
echo "ros2 topic info /cmd_vel -v"

log "[5/5] DONE."
IP=$(hostname -I | awk '{print $1}')
echo "Web    : http://${IP}:8000"
echo "ROS WS : ws://${IP}:9090"
echo "Attach : tmux attach -t ${SESSION}"
echo "Detach : Ctrl-b rồi nhấn d"
echo "Kill   : tmux kill-session -t ${SESSION}"
