#!/usr/bin/env bash
set -euo pipefail

# Web driver station session helper:
# - Optionally starts base_bringup (spawn_drive:=true) in the background
# - Runs the driver station server (HTTP + WebSocket)
# - Cleans up processes on exit

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

NO_BRINGUP=0
if [[ "${1:-}" == "--no-bringup" ]]; then
  NO_BRINGUP=1
  shift
fi

echo "[driver_station_session] SAFETY: off-ground first; keep E-stop accessible."

if pgrep -f "driver_station_server.py" >/dev/null 2>&1; then
  echo "[driver_station_session] Stopping existing driver station server..."
  pkill -TERM -f "driver_station_server.py" 2>/dev/null || true
  sleep 0.5
  pkill -KILL -f "driver_station_server.py" 2>/dev/null || true
fi

"$ROOT_DIR/scripts/ros_clean.sh" --force || true

# Ensure pigpio daemon is running (required for l298n_hardware).
if ! pgrep -x pigpiod >/dev/null 2>&1; then
  echo "[driver_station_session] Starting pigpiod..."
  sudo -n systemctl start pigpiod || sudo -n /usr/local/bin/pigpiod -g || sudo -n pigpiod -g || true
  sleep 0.5
fi

set +u
source /opt/ros/humble/setup.bash
[[ -f "$ROOT_DIR/install/setup.bash" ]] && source "$ROOT_DIR/install/setup.bash"
set -u

cleanup() {
  local code=$?
  if [[ -n "${BR_PGID:-}" ]]; then
    echo "[driver_station_session] Stopping bringup (pgid=$BR_PGID)"
    kill -INT -"$BR_PGID" 2>/dev/null || true; sleep 1 || true
    kill -TERM -"$BR_PGID" 2>/dev/null || true; sleep 1 || true
    kill -KILL -"$BR_PGID" 2>/dev/null || true
  fi
  "$ROOT_DIR/scripts/ros_clean.sh" --force || true
  echo "[driver_station_session] Exit code: $code"
}
trap cleanup EXIT INT TERM

if [[ "$NO_BRINGUP" -eq 0 ]]; then
  TS=$(date +%Y%m%d_%H%M%S)
  BR_LOG="$ROOT_DIR/out/driver_station_bringup_${TS}.log"
  mkdir -p "$ROOT_DIR/out"

  echo "[driver_station_session] Starting base bringup (logs: $BR_LOG)"
  setsid bash -lc "set +u; source /opt/ros/humble/setup.bash; [[ -f '$ROOT_DIR/install/setup.bash' ]] && source '$ROOT_DIR/install/setup.bash'; set -u; ros2 launch alpha_viam_bringup base_bringup.launch.py spawn_drive:=true" >"$BR_LOG" 2>&1 &
  BR_PID=$!
  sleep 6
  BR_PGID=$(ps -o pgid= "$BR_PID" | tr -d ' ')

  echo "[driver_station_session] Waiting for cmd topic (/controller_manager/cmd_vel_unstamped)..."
  for _ in $(seq 1 40); do
    if ros2 topic list 2>/dev/null | grep -Fxq "/controller_manager/cmd_vel_unstamped"; then
      break
    fi
    sleep 0.25
  done
fi

echo "[driver_station_session] Driver station starting."
echo "[driver_station_session] UI: http://alpha-viam.local:8090/ (override: --http-port)"
echo "[driver_station_session] Video (MJPEG): http://alpha-viam.local:8080/stream (override: --mjpeg-port)"

echo "[driver_station_session] Tip: hold the on-screen button to drive (deadman)."

"$ROOT_DIR/scripts/driver_station.sh" "$@"
