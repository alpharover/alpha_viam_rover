#!/usr/bin/env bash
set -euo pipefail

# Launch a ROS 2 launch file for a bounded time, then terminate the whole group.
# Usage: scripts/launch_with_timeout.sh <seconds> <package> <launch_file> [launch_args...]

if [[ $# -lt 3 ]]; then
  echo "Usage: $0 <seconds> <package> <launch_file> [launch_args...]" >&2
  exit 2
fi

DUR="$1"; shift
PKG="$1"; shift
LAUNCH="$1"; shift

set +u
source /opt/ros/humble/setup.bash
set -u
if [[ -f "install/setup.bash" ]]; then
  set +u
  . install/setup.bash
  set -u
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Ensure ament index can see C++ plugins (e.g., l298n_hardware)
if [[ -d "$ROOT_DIR/install/l298n_hardware" ]]; then
  export AMENT_PREFIX_PATH="$ROOT_DIR/install/l298n_hardware:${AMENT_PREFIX_PATH:-}"
fi

echo "[launch_with_timeout] Starting $PKG $LAUNCH for ${DUR}s..."
set +e
setsid ros2 launch "$PKG" "$LAUNCH" "$@" &
LAUNCH_PID=$!
set -e

sleep "$DUR" || true

if ps -p "$LAUNCH_PID" >/dev/null 2>&1; then
  echo "[launch_with_timeout] Sending INT to process group -$LAUNCH_PID"
  kill -INT -"$LAUNCH_PID" 2>/dev/null || true
  sleep 3 || true
fi

if ps -p "$LAUNCH_PID" >/dev/null 2>&1; then
  echo "[launch_with_timeout] Sending TERM to process group -$LAUNCH_PID"
  kill -TERM -"$LAUNCH_PID" 2>/dev/null || true
  sleep 2 || true
fi

if ps -p "$LAUNCH_PID" >/dev/null 2>&1; then
  echo "[launch_with_timeout] Sending KILL to process group -$LAUNCH_PID"
  kill -KILL -"$LAUNCH_PID" 2>/dev/null || true
fi

echo "[launch_with_timeout] Cleaning residual nodes..."
"$(dirname "$0")/ros_clean.sh" --force
echo "[launch_with_timeout] Done."
