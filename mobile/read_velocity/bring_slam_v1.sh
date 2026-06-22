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

CAN_PY="$WS/src/mobile/read_velocity/can_communicate.py"
FIX_USB="/usr/local/sbin/fix-usb-serial.sh"

TIMEOUT_IMU_BUILD=240
TIMEOUT_LIDAR_BUILD=240
TIMEOUT_CAN_START=30
TIMEOUT_SLAM_START=60

# =======================
# HELPERS
# =======================
log()  { echo -e "\033[1;32m$*\033[0m"; }
warn() { echo -e "\033[1;33m$*\033[0m"; }
die()  { echo -e "\033[1;31m$*\033[0m" >&2; exit 1; }

need() { command -v "$1" >/dev/null 2>&1 || die "Missing '$1'"; }

# --- WAIT DEVICE EXISTS ---
wait_devs() {
  for _ in {1..50}; do
    [[ -e "$IMU_DEV" && -e "$CAN_DEV" && -e "$LIDAR_DEV" ]] && return 0
    sleep 0.1
  done
  return 1
}

# --- WAIT DEVICE OPENABLE (QUAN TRỌNG) ---
wait_openable() {
  local dev="$1"
  for _ in {1..25}; do
    if [[ -e "$dev" ]]; then
      if timeout 1 bash -c "echo > $dev" 2>/dev/null; then
        log "Device ready: $dev"
        return 0
      fi
    fi
    sleep 0.2
  done
  return 1
}

# --- SAFE STTY (KHÔNG TREO) ---
stty_one() {
  local dev="$1" baud="$2"
  if [[ -e "$dev" ]]; then
    if timeout 2 stty -F "$dev" "$baud" cs8 -cstopb -parenb -crtscts raw -echo; then
      log "stty OK: $dev @ $baud"
    else
      warn "stty timeout/fail: $dev"
    fi
  else
    warn "Skip stty: $dev not found"
  fi
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
[[ -x "$FIX_USB" ]] || die "Fix script missing: $FIX_USB"

# =======================
# FIX USB
# =======================
log "[0/4] Fix USB serial..."
sudo "$FIX_USB"

sudo udevadm settle --timeout=5 || true

wait_devs || die "Device symlinks not ready"

log "Devices:"
ls -l "$IMU_DEV" "$CAN_DEV" "$LIDAR_DEV" || true

# 🔥 QUAN TRỌNG: đợi USB ổn định thật
log "Waiting devices to be fully ready..."
wait_openable "$LIDAR_DEV" || warn "Lidar not fully ready"

sleep 2   # thêm delay chống race condition

# =======================
# PRE-STTY
# =======================
log "[1/4] Pre-config serial (LiDAR only)"
stty_one "$LIDAR_DEV" 921600

# =======================
# TMUX
# =======================
if tmux has-session -t "$SESSION" 2>/dev/null; then
  warn "tmux session exists"
  echo "Attach: tmux attach -t ${SESSION}"
  exit 0
fi

log "[2/4] Create tmux..."
tmux new-session -d -s "$SESSION" -n "imu"

# ---------------- IMU ----------------
tmux send-keys -t "$SESSION:imu" \
"tmux wait-for STAGE_IMU; \
cd \"$WS\" && colcon build && source install/setup.bash && \
tmux wait-for -S IMU_OK; \
flock -n /tmp/lock_imu.lock python3 \"$WS/src/mobile/read_velocity/imu.py\" --ros-args -p port:=\"$IMU_DEV\" -p baud:=115200" C-m

# ---------------- LiDAR ----------------
tmux new-window -t "$SESSION" -n "lidar"
tmux send-keys -t "$SESSION:lidar" \
"tmux wait-for STAGE_LIDAR; \
cd \"$BLUESEA_WS\" && colcon build && source install/setup.bash && \
sleep 2; \
tmux wait-for -S LIDAR_OK; \
for i in {1..3}; do \
  echo \"Start LiDAR try \$i\"; \
  ros2 launch bluesea2 uart_lidar.launch && break; \
  echo \"Retry LiDAR...\"; \
  sleep 2; \
done" C-m

# ---------------- CAN ----------------
tmux new-window -t "$SESSION" -n "can"
tmux send-keys -t "$SESSION:can" \
"tmux wait-for STAGE_CAN; \
tmux wait-for -S CAN_OK; \
flock -n /tmp/lock_can.lock python3 \"$CAN_PY\" \"$CAN_DEV\"" C-m

# ---------------- SLAM ----------------
tmux new-window -t "$SESSION" -n "slam"
tmux send-keys -t "$SESSION:slam" \
"tmux wait-for STAGE_SLAM; \
cd \"$WS\" && source install/setup.bash && \
tmux wait-for -S SLAM_OK; \
ros2 launch mobile slam_toolbox.launch.py" C-m

# =======================
# ORDER
# =======================
log "[3/4] Run order"

tmux wait-for -S STAGE_IMU
timeout "${TIMEOUT_IMU_BUILD}s" tmux wait-for IMU_OK || die "IMU timeout"

tmux wait-for -S STAGE_LIDAR
timeout "${TIMEOUT_LIDAR_BUILD}s" tmux wait-for LIDAR_OK || die "LiDAR timeout"

tmux wait-for -S STAGE_CAN
timeout "${TIMEOUT_CAN_START}s" tmux wait-for CAN_OK || die "CAN timeout"

tmux wait-for -S STAGE_SLAM
timeout "${TIMEOUT_SLAM_START}s" tmux wait-for SLAM_OK || die "SLAM timeout"

log "[4/4] DONE"
echo "Attach: tmux attach -t ${SESSION}"
echo "Detach : Ctrl-b rồi nhấn d"
echo "Kill   : tmux kill-session -t ${SESSION}"