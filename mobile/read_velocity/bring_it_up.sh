#!/usr/bin/env bash
set -Eeuo pipefail

# =======================
# CONFIG
# =======================
SESSION="mobile"

WS="$HOME/ros2_ws"
BLUESEA_WS="$WS/src/mobile/bluesea2"

# Fixed udev symlinks (đã khóa theo lỗ)
IMU_DEV="/dev/ch34x_imu"
CAN_DEV="/dev/ch34x_can"
LIDAR_DEV="/dev/cp210x_lidar"

CAN_PY="$WS/src/mobile/read_velocity/can_communicate.py"
FIX_USB="/usr/local/sbin/fix-usb-serial.sh"

# timeouts
TIMEOUT_IMU_BUILD=240
TIMEOUT_LIDAR_BUILD=240
TIMEOUT_CAN_START=30
TIMEOUT_SLAM_START=60
TIMEOUT_BRINGUP_START=60

# =======================
# HELPERS
# =======================
log()  { echo -e "\033[1;32m$*\033[0m"; }
warn() { echo -e "\033[1;33m$*\033[0m"; }
die()  { echo -e "\033[1;31m$*\033[0m" >&2; exit 1; }

need() { command -v "$1" >/dev/null 2>&1 || die "Missing '$1' (install it first)"; }

stty_one() {
  local dev="$1" baud="$2"
  if [[ -e "$dev" ]]; then
    # Không sudo: udev đã set GROUP dialout + MODE 0660
    stty -F "$dev" "$baud" cs8 -cstopb -parenb -crtscts raw -echo || true
    log "stty OK: $dev @ $baud"
  else
    warn "Skip stty: $dev not found: $dev"
  fi
}

wait_devs() {
  # udev trigger có thể async -> đợi symlink xuất hiện
  for _ in {1..50}; do
    [[ -e "$IMU_DEV" && -e "$CAN_DEV" && -e "$LIDAR_DEV" ]] && return 0
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
[[ -d "$BLUESEA_WS" ]] || die "BlueSea folder not found: $BLUESEA_WS"
[[ -f "$CAN_PY" ]] || die "CAN script not found: $CAN_PY"
[[ -x "$FIX_USB" ]] || die "Fix script missing/not executable: $FIX_USB"

# =======================
# FIX USB SERIAL (NO PASSWORD via sudoers)
# =======================
log "[0/4] Fix USB serial (no password)..."
sudo "$FIX_USB"

# ensure udev finished
sudo udevadm settle --timeout=5 || true
wait_devs || die "Device symlinks not ready: $IMU_DEV $CAN_DEV $LIDAR_DEV"

log "Devices:"
ls -l "$IMU_DEV" "$CAN_DEV" "$LIDAR_DEV" || true

# =======================
# PRE-STTY (chỉ LiDAR; IMU/CAN để code tự set baud)
# =======================
log "[1/4] Pre-config serial (LiDAR only)"
stty_one "$LIDAR_DEV" 921600

# =======================
# TMUX session
# =======================
if tmux has-session -t "$SESSION" 2>/dev/null; then
  warn "tmux session '${SESSION}' already exists."
  echo "Attach: tmux attach -t ${SESSION}"
  echo "Kill  : tmux kill-session -t ${SESSION}"
  exit 0
fi

log "[2/4] Create tmux session/windows..."
tmux new-session -d -s "$SESSION" -n "imu"

# -----------------------
# Window: IMU (build WS + run IMU)
# -----------------------
tmux send-keys -t "$SESSION:imu" \
  "tmux wait-for STAGE_IMU; \
   cd \"$WS\" && colcon build && source install/setup.bash && \
   tmux wait-for -S IMU_OK; \
   flock -n /tmp/lock_imu.lock ros2 run wt901_ros2 wt901_node --ros-args -p port:=\"$IMU_DEV\"" C-m

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
# Window: SLAM
# -----------------------
tmux new-window -t "$SESSION" -n "slam"
tmux send-keys -t "$SESSION:slam" \
  "tmux wait-for STAGE_SLAM; \
   cd \"$WS\" && source install/setup.bash && \
   tmux wait-for -S SLAM_OK; \
   ros2 launch mobile slam_toolbox.launch.py" C-m

# -----------------------
# Window: BRINGUP
# -----------------------
tmux new-window -t "$SESSION" -n "bringup"
tmux send-keys -t "$SESSION:bringup" \
  "tmux wait-for STAGE_BRINGUP; \
   cd \"$WS\" && source install/setup.bash && \
   tmux wait-for -S BRINGUP_OK; \
   ros2 launch mobile nav2.launch.py" C-m

# =======================
# Orchestrate order
# =======================
log "[3/4] Run order: IMU -> LiDAR -> CAN -> SLAM -> BRINGUP"

tmux wait-for -S STAGE_IMU
timeout "${TIMEOUT_IMU_BUILD}s" tmux wait-for IMU_OK || die "IMU stage timed out"

tmux wait-for -S STAGE_LIDAR
timeout "${TIMEOUT_LIDAR_BUILD}s" tmux wait-for LIDAR_OK || die "LiDAR stage timed out"

tmux wait-for -S STAGE_CAN
timeout "${TIMEOUT_CAN_START}s" tmux wait-for CAN_OK || die "CAN stage timed out"

tmux wait-for -S STAGE_SLAM
timeout "${TIMEOUT_SLAM_START}s" tmux wait-for SLAM_OK || die "SLAM stage timed out"

tmux wait-for -S STAGE_BRINGUP
timeout "${TIMEOUT_BRINGUP_START}s" tmux wait-for BRINGUP_OK || die "BRINGUP stage timed out"

log "[4/4] DONE."
echo "Attach : tmux attach -t ${SESSION}"
echo "Detach : Ctrl-b rồi nhấn d"
echo "Kill   : tmux kill-session -t ${SESSION}"


