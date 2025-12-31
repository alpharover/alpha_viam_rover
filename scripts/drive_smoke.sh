#!/usr/bin/env bash
set -euo pipefail

# Drive smoke test: launch bring-up, record MCAP, send short /cmd_vel bursts, then cleanly stop.
# Off-ground only.
# Usage: scripts/drive_smoke.sh [duration_seconds]

DUR=${1:-25}
TS=$(date +%Y%m%d_%H%M%S)
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

BAG_DIR="$ROOT_DIR/bags/phase3_offground_${TS}"
mkdir -p "$BAG_DIR"

CMD_TOPIC_OVERRIDE=${CMD_TOPIC:-}
LINEAR_X=${LINEAR_X:-0.8}
FWD_SEC=${FWD_SEC:-1.6}
REV_SEC=${REV_SEC:-1.6}
STOP_SEC=${STOP_SEC:-0.2}
BURST_RATE=${BURST_RATE:-15}

if command -v systemctl >/dev/null 2>&1; then
  echo "[drive_smoke] Checking pigpio daemon..."
  if ! pgrep -x pigpiod >/dev/null 2>&1; then
    sudo systemctl start pigpiod || sudo /usr/local/bin/pigpiod || sudo pigpiod || true
    sleep 0.5
  fi
fi

set +u
source /opt/ros/humble/setup.bash
[ -f "$ROOT_DIR/install/setup.bash" ] && source "$ROOT_DIR/install/setup.bash"
set -u

# Ensure plugin index includes l298n_hardware
export AMENT_PREFIX_PATH="$ROOT_DIR/install/l298n_hardware:${AMENT_PREFIX_PATH:-}"

echo "[drive_smoke] Starting bring-up..."
setsid bash -lc "set +u; source /opt/ros/humble/setup.bash; [ -f \"$ROOT_DIR/install/setup.bash\" ] && source \"$ROOT_DIR/install/setup.bash\"; export AMENT_PREFIX_PATH=\"$ROOT_DIR/install/l298n_hardware:\${AMENT_PREFIX_PATH:-}\"; set -u; cd \"$ROOT_DIR\"; ros2 launch alpha_viam_bringup base_bringup.launch.py spawn_drive:=true" &
BR_PID=$!
sleep 7
BR_PGID=$(ps -o pgid= $BR_PID | tr -d ' ')

echo "[drive_smoke] Controllers:" && (timeout 4s ros2 control list_controllers || true)

detect_topic() {
  local attempt topics candidate
  for attempt in $(seq 1 20); do
    topics="$(timeout 2s ros2 topic list 2>/dev/null || true)"
    for candidate in "$@"; do
      if echo "$topics" | grep -Fxq "$candidate"; then
        echo "$candidate"
        return 0
      fi
    done
    sleep 0.25
  done
  return 1
}

if [[ -z "$CMD_TOPIC_OVERRIDE" ]]; then
  CMD_TOPIC="$(detect_topic /controller_manager/cmd_vel_unstamped /diff_drive_controller/cmd_vel_unstamped /cmd_vel_unstamped /cmd_vel || true)"
  if [[ -n "$CMD_TOPIC" ]]; then
    echo "[drive_smoke] Auto-detected CMD_TOPIC=$CMD_TOPIC"
  else
    CMD_TOPIC="/controller_manager/cmd_vel_unstamped"
    echo "[drive_smoke] WARN: could not auto-detect cmd topic; using $CMD_TOPIC"
  fi
else
  CMD_TOPIC="$CMD_TOPIC_OVERRIDE"
  echo "[drive_smoke] Using CMD_TOPIC override: $CMD_TOPIC"
fi

echo "[drive_smoke] Recording MCAP for ${DUR}s..."
TOPICS=("$CMD_TOPIC" /joint_states /tf /tf_static /odometry/filtered /controller_manager/odom /diff_drive_controller/odom)
timeout "${DUR}s" ros2 bag record -s mcap -o "$BAG_DIR/phase3_offground" "${TOPICS[@]}" &
REC_PID=$!
sleep 3

echo "[drive_smoke] Forward burst on ${CMD_TOPIC} (linear_x=${LINEAR_X}, ${FWD_SEC}s)"
python3 "$ROOT_DIR/scripts/pub_twist.py" --topic "$CMD_TOPIC" --duration "$FWD_SEC" --rate "$BURST_RATE" --linear_x "$LINEAR_X" || true
sleep 0.6
echo "[drive_smoke] Reverse burst on ${CMD_TOPIC} (linear_x=-${LINEAR_X}, ${REV_SEC}s)"
python3 "$ROOT_DIR/scripts/pub_twist.py" --topic "$CMD_TOPIC" --duration "$REV_SEC" --rate "$BURST_RATE" --linear_x "-${LINEAR_X}" || true
sleep 0.3
echo "[drive_smoke] Stop"
python3 "$ROOT_DIR/scripts/pub_twist.py" --topic "$CMD_TOPIC" --duration "$STOP_SEC" --rate 3 --linear_x 0.0 || true

wait $REC_PID || true

echo "[drive_smoke] Stopping bring-up..."
kill -INT -"$BR_PGID" 2>/dev/null || true; sleep 2 || true
kill -TERM -"$BR_PGID" 2>/dev/null || true; sleep 2 || true
kill -KILL -"$BR_PGID" 2>/dev/null || true

"$ROOT_DIR/scripts/ros_clean.sh" --force || true

echo "[drive_smoke] Bag at: $BAG_DIR"
ros2 bag info "$BAG_DIR/phase3_offground" || true
