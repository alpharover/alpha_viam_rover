#!/usr/bin/env bash
set -euo pipefail

# One-command smoke test for diff_drive_controller.

DUR=${1:-10}
TS=$(date +%Y%m%d_%H%M%S)
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG_DIR="$ROOT_DIR/bags/diff_${TS}"
mkdir -p "$BAG_DIR"

set +u
source /opt/ros/humble/setup.bash
[ -f "$ROOT_DIR/install/setup.bash" ] && source "$ROOT_DIR/install/setup.bash"
set -u

echo "[diff_drive_smoke] Launching drive_min..."
setsid bash -lc 'set +u; source /opt/ros/humble/setup.bash; [ -f install/setup.bash ] && source install/setup.bash; set -u; ros2 launch alpha_viam_bringup drive_min.launch.py' &
BR_PID=$!
sleep 5
BR_PGID=$(ps -o pgid= $BR_PID | tr -d ' ')

echo "[diff_drive_smoke] Controllers:" && (ros2 control list_controllers || true)
echo "[diff_drive_smoke] Interfaces:" && (ros2 control list_hardware_interfaces || true)

echo "[diff_drive_smoke] Recording ${DUR}s MCAP..."
timeout "${DUR}s" ros2 bag record -s mcap -o "$BAG_DIR/diff_run" /diff_drive_controller/cmd_vel_unstamped /joint_states /tf /tf_static || true &
REC_PID=$!
sleep 1

echo "[diff_drive_smoke] Forward burst via pub_twist.py"
python3 "$ROOT_DIR/scripts/pub_twist.py" --topic /diff_drive_controller/cmd_vel_unstamped --duration 1.6 --rate 15 --linear_x 0.8 || true
sleep 0.5
echo "[diff_drive_smoke] Stop"
python3 "$ROOT_DIR/scripts/pub_twist.py" --topic /diff_drive_controller/cmd_vel_unstamped --duration 0.2 --rate 3 --linear_x 0.0 || true

wait $REC_PID || true

echo "[diff_drive_smoke] Stopping..."
kill -INT -"$BR_PGID" 2>/dev/null || true; sleep 1 || true
kill -TERM -"$BR_PGID" 2>/dev/null || true; sleep 1 || true
kill -KILL -"$BR_PGID" 2>/dev/null || true

"$ROOT_DIR/scripts/ros_clean.sh" --force || true

echo "[diff_drive_smoke] Bag: $BAG_DIR"
ros2 bag info "$BAG_DIR/diff_run" || true

