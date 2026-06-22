#!/usr/bin/env bash
set -Eeuo pipefail

# =======================
# CONFIG
# =======================
SESSION="mobile"

WS="$HOME/ros2_ws"
BLUESEA_WS="$WS/src/mobile/bluesea2"

IMU_DEV="/dev/ch34x_imu"
CAN_DEV="/dev/ch34x_can"
LIDAR_DEV="/dev/cp210x_lidar"

# CAN_PY="$WS/src/mobile/read_velocity/can_communicate.py"
CAN_PY="$WS/src/mobile/read_velocity/can_communicate_v0.py"
FIX_USB="/usr/local/sbin/fix-usb-serial.sh"

TIMEOUT_IMU_BUILD=240
TIMEOUT_LIDAR_BUILD=240
TIMEOUT_CAN_START=30
TIMEOUT_SLAM_START=60
TIMEOUT_BRINGUP_START=60
# TIMEOUT_velocity_smoother=60


# =======================
# LOAD ENV (🔥 QUAN TRỌNG NHẤT)
# =======================
echo "[ENV] Loading ROS2 environment..."
set +u  # 🔹 tắt 'unbound variable' check tạm thời
source /opt/ros/humble/setup.bash || { echo "Cannot source ROS"; exit 1; }
source "$WS/install/setup.bash" || { echo "Cannot source WS"; exit 1; }
set -u  # 🔹 bật lại để tiếp tục script
export PATH=$PATH:/usr/local/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib



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

[[ -d "$WS" ]] || die "Workspace not found: $WS"
[[ -d "$BLUESEA_WS" ]] || die "BlueSea folder not found"
[[ -f "$CAN_PY" ]] || die "CAN script not found"
[[ -x "$FIX_USB" ]] || die "Fix script missing"

# =======================
# FIX USB
# =======================
log "[0/4] Fix USB serial..."

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


sleep 3   # 🔥 cực quan trọng


# =======================
# PRE-STTY
# =======================
log "[1/4] Pre-config serial"

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
  warn "tmux exists"
  echo "Attach: tmux attach -t ${SESSION}"
  exit 0
fi

log "[2/4] Create tmux..."
tmux new-session -d -s "$SESSION" -n "imu"

# -----------------------
# Window: IMU (build WS + run IMU)
# -----------------------
tmux send-keys -t "$SESSION:imu" \
  "tmux wait-for STAGE_IMU; \
   cd \"$WS\" && colcon build && source install/setup.bash && \
   tmux wait-for -S IMU_OK; \
   flock -n /tmp/lock_imu.lock python3 \"$WS/src/mobile/read_velocity/imu.py\" --ros-args -p port:=\"$IMU_DEV\" -p baud:=115200" C-m
# -----------------------
# Window: LiDAR (build bluesea2 + run lidar)
# -----------------------
tmux new-window -t "$SESSION" -n "lidar"
tmux send-keys -t "$SESSION:lidar" \
  "tmux wait-for STAGE_LIDAR; \
   cd \"$BLUESEA_WS\" && colcon build && source install/setup.bash && \
   tmux wait-for -S LIDAR_OK; \
   flock -n /tmp/lock_lidar.lock ros2 launch bluesea2 uart_lidar.launch" C-m

# -----------------------
# Window: CAN (run can)
# NOTE: can_communicate.py phải nhận argv[1] là port.
# -----------------------
tmux new-window -t "$SESSION" -n "can"
tmux send-keys -t "$SESSION:can" \
  "tmux wait-for STAGE_CAN; \
   tmux wait-for -S CAN_OK; \
   flock -n /tmp/lock_can.lock python3 \"$CAN_PY\" \"$CAN_DEV\"" C-m

# -----------------------
# Window: ekf
# -----------------------
tmux new-window -t "$SESSION" -n "ekf"
tmux send-keys -t "$SESSION:ekf" \
  "tmux wait-for STAGE_ekf; \
   cd \"$WS\" && source install/setup.bash && \
   tmux wait-for -S ekf_OK; \
   ros2 launch mobile ekf.launch.py" C-m


# -----------------------
# Window: BRINGUP
# -----------------------







# tmux new-window -t "$SESSION" -n "bringup"
# tmux send-keys -t "$SESSION:bringup" \
#   "tmux wait-for STAGE_BRINGUP; \
#    cd \"$WS\" && source install/setup.bash && \
#    tmux wait-for -S BRINGUP_OK; \
#    ros2 launch mobile nav2nguyen.launch.py" C-m







# -----------------------
# Window: Velocity Smoother
# -----------------------
# tmux new-window -t "$SESSION" -n "velocity_smoother"
# tmux send-keys -t "$SESSION:velocity_smoother" \
#   "echo 'Waiting velocity smoother...'; \
#    tmux wait-for STAGE_velocity_smoother; \
#    echo 'Starting velocity smoother'; \
#    python3 /home/pan/ros2_ws/src/mobile/read_velocity/test_smooth_velocity.py" C-m
# =======================
# Orchestrate order
# =======================
log "[3/4] Run order: IMU -> LiDAR -> CAN  -> EKF-> BRINGUP"

tmux wait-for -S STAGE_IMU
timeout "${TIMEOUT_IMU_BUILD}s" tmux wait-for IMU_OK || die "IMU stage timed out"

tmux wait-for -S STAGE_LIDAR
timeout "${TIMEOUT_LIDAR_BUILD}s" tmux wait-for LIDAR_OK || die "LiDAR stage timed out"

tmux wait-for -S STAGE_CAN
timeout "${TIMEOUT_CAN_START}s" tmux wait-for CAN_OK || die "CAN stage timed out"


tmux wait-for -S STAGE_ekf
timeout "${TIMEOUT_SLAM_START}s" tmux wait-for ekf_OK || die "ekf stage timed out"



# tmux wait-for -S STAGE_velocity_smoother
# timeout "${TIMEOUT_velocity_smoother}s" tmux wait-for velocity_smoother_OK || die "velocity_smoother stage timed out"

log "[4/4] DONE."
echo "Attach : tmux attach -t ${SESSION}"
echo "Detach : Ctrl-b rồi nhấn d"
echo "Kill   : tmux kill-session -t ${SESSION}"


