#!/usr/bin/env bash
set -euo pipefail

# One-command smoke test for l298n_direct (no ros2_control).
# Launches drive_direct.launch.py, sends a /cmd_vel burst, records brief MCAP, and tears down.

DUR=${1:-10}
TS=$(date +%Y%m%d_%H%M%S)
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
BAG_DIR="$ROOT_DIR/bags/direct_${TS}"
mkdir -p "$BAG_DIR"

echo "[direct_smoke] Checking pigpio daemon..."
if ! pigs hwver >/dev/null 2>&1; then
  echo "[direct_smoke] Starting pigpiod..."
  if command -v systemctl >/dev/null 2>&1; then
    sudo systemctl start pigpiod || sudo /usr/local/bin/pigpiod -g || sudo pigpiod -g
  else
    sudo /usr/local/bin/pigpiod -g || sudo pigpiod -g
  fi
fi

set +u
source /opt/ros/humble/setup.bash
[ -f "$ROOT_DIR/install/setup.bash" ] && source "$ROOT_DIR/install/setup.bash"
set -u

echo "[direct_smoke] Launching l298n_direct..."
setsid bash -lc 'set +u; source /opt/ros/humble/setup.bash; [ -f install/setup.bash ] && source install/setup.bash; set -u; ros2 launch alpha_viam_bringup drive_direct.launch.py' &
BR_PID=$!
sleep 4
BR_PGID=$(ps -o pgid= $BR_PID | tr -d ' ')

echo "[direct_smoke] /cmd_vel subscriber check:" && (ros2 node list | grep -q l298n_direct && echo "subscriber present (node: l298n_direct)" || echo "subscriber not yet visible")

echo "[direct_smoke] Recording ${DUR}s MCAP..."
timeout "${DUR}s" ros2 bag record -s mcap -o "$BAG_DIR/direct_run" /cmd_vel /tf /tf_static /joint_states &
REC_PID=$!
sleep 1

echo "[direct_smoke] Forward burst via pub_twist.py"
python3 "$ROOT_DIR/scripts/pub_twist.py" --topic /cmd_vel --duration 1.6 --rate 15 --linear_x 1.0 || true
sleep 0.5
echo "[direct_smoke] Stop"
python3 "$ROOT_DIR/scripts/pub_twist.py" --topic /cmd_vel --duration 0.2 --rate 3 --linear_x 0.0 || true

wait $REC_PID || true

echo "[direct_smoke] Stopping..."
kill -INT -"$BR_PGID" 2>/dev/null || true; sleep 1 || true
kill -TERM -"$BR_PGID" 2>/dev/null || true; sleep 1 || true
kill -KILL -"$BR_PGID" 2>/dev/null || true

"$ROOT_DIR/scripts/ros_clean.sh" --force || true

echo "[direct_smoke] Bag: $BAG_DIR"
ros2 bag info "$BAG_DIR/direct_run" || true
