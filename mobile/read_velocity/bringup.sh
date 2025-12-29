#!/usr/bin/env bash
set -euo pipefail

WS="$HOME/ros2_ws"
PIDDIR="/tmp/robot_run_pids"
LOGDIR="/tmp/robot_logs"

ENCODER_PY="$WS/src/mobile/read_velocity/gui_encoder.py"

# ---------- Helpers ----------
safe_source_overlay() {
  set +u
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
  set -u
}

stop_all() {
  echo -e "\n[bringup] Stopping..."

  shopt -s nullglob
  local pids=()

  for f in "$PIDDIR"/*.pid; do
    local pid=""
    pid="$(cat "$f" 2>/dev/null || true)"
    [[ -n "${pid:-}" ]] && pids+=("$pid")
  done

  # INT -> TERM -> KILL
  for sig in INT TERM KILL; do
    for pid in "${pids[@]}"; do
      if kill -0 "$pid" 2>/dev/null; then
        kill "-$sig" "$pid" 2>/dev/null || true
      fi
    done
    sleep 0.6
  done

  rm -f "$PIDDIR"/*.pid 2>/dev/null || true
  echo "[bringup] Stopped."
}

start_all() {
  mkdir -p "$PIDDIR" "$LOGDIR"
  rm -f "$PIDDIR"/*.pid 2>/dev/null || true

  echo "[bringup] Building workspace..."
  cd "$WS"
  colcon build

  echo "[bringup] Sourcing overlay..."
  safe_source_overlay

  echo "[bringup] Starting encoder + imu (logs in $LOGDIR)"

  # Encoder
  (
    set -euo pipefail
    cd "$WS"
    set +u; source install/setup.bash; set -u
    echo $$ > "$PIDDIR/encoder.pid"
    exec python3 "$ENCODER_PY" >>"$LOGDIR/encoder.log" 2>&1
  ) &

  # IMU
  (
    set -euo pipefail
    cd "$WS"
    set +u; source install/setup.bash; set -u
    echo $$ > "$PIDDIR/imu.pid"
    exec ros2 run wt901_ros2 wt901_node >>"$LOGDIR/imu.log" 2>&1
  ) &

  echo "[bringup] Started:"
  echo "  - encoder: tail -f $LOGDIR/encoder.log"
  echo "  - imu:     tail -f $LOGDIR/imu.log"
  echo "  - stop:    $0 stop"
}

# ---------- Main ----------
case "${1:-start}" in
  start)
    trap 'stop_all; exit 0' INT TERM
    start_all
    # giữ process sống để bắt Ctrl+C
    while true; do sleep 1; done
    ;;
  stop)
    stop_all
    ;;
  *)
    echo "Usage:"
    echo "  $0        # start"
    echo "  $0 stop   # stop all"
    exit 1
    ;;
esac