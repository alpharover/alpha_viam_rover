#!/usr/bin/env bash
set -euo pipefail

# Drive smoke test: launch bring-up, record MCAP, send short /cmd_vel bursts, then cleanly stop.
# Usage: scripts/drive_smoke.sh [duration_seconds]

DUR=${1:-25}
TS=$(date +%Y%m%d_%H%M%S)
BAG_DIR="bags/phase3_offground_${TS}"
mkdir -p "$BAG_DIR"

set +u
source /opt/ros/humble/setup.bash
[ -f install/setup.bash ] && source install/setup.bash
set -u

# Ensure plugin index includes l298n_hardware
export AMENT_PREFIX_PATH="$(pwd)/install/l298n_hardware:${AMENT_PREFIX_PATH:-}"

echo "[drive_smoke] Starting bring-up..."
setsid bash -lc 'set +u; source /opt/ros/humble/setup.bash; [ -f install/setup.bash ] && source install/setup.bash; export AMENT_PREFIX_PATH="'"$(pwd)/install/l298n_hardware:${AMENT_PREFIX_PATH}"'"; set -u; ros2 launch alpha_viam_bringup base_bringup.launch.py spawn_drive:=true' &
BR_PID=$!
sleep 7
BR_PGID=$(ps -o pgid= $BR_PID | tr -d ' ')

echo "[drive_smoke] Controllers:" && (timeout 4s ros2 control list_controllers || true)

echo "[drive_smoke] Recording MCAP for ${DUR}s..."
TOPICS=(/cmd_vel /joint_states /tf /tf_static /odometry/filtered /diff_drive_controller/odom)
timeout "${DUR}s" ros2 bag record -s mcap -o "$BAG_DIR/phase3_offground" "${TOPICS[@]}" &
REC_PID=$!
sleep 3

CMD_TOPIC=${CMD_TOPIC:-/diff_drive_controller/cmd_vel_unstamped}

echo "[drive_smoke] Forward burst on ${CMD_TOPIC}"
timeout 1.6s ros2 topic pub -r 10 "$CMD_TOPIC" geometry_msgs/msg/Twist '{linear: {x: 0.20}}' || true
sleep 0.6
echo "[drive_smoke] Reverse burst on ${CMD_TOPIC}"
timeout 1.6s ros2 topic pub -r 10 "$CMD_TOPIC" geometry_msgs/msg/Twist '{linear: {x: -0.20}}' || true
timeout 1.2s ros2 topic pub --once "$CMD_TOPIC" geometry_msgs/msg/Twist '{linear: {x: 0.0}}' || true

wait $REC_PID || true

echo "[drive_smoke] Stopping bring-up..."
kill -INT -"$BR_PGID" 2>/dev/null || true; sleep 2 || true
kill -TERM -"$BR_PGID" 2>/dev/null || true; sleep 2 || true
kill -KILL -"$BR_PGID" 2>/dev/null || true

"$(dirname "$0")/ros_clean.sh" --force || true

echo "[drive_smoke] Bag at: $BAG_DIR"
ros2 bag info "$BAG_DIR/phase3_offground" || true
