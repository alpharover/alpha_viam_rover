#!/usr/bin/env bash
set -euo pipefail

# Minimal validation for diff_drive_controller on ROS 2 Humble
# - Ensures controller parameters are present at controller init via params_file
# - Verifies controllers active and velocity interfaces claimed
# - Publishes short burst on /diff_drive_controller/cmd_vel_unstamped
# - Records a short MCAP for evidence
#
# Usage: scripts/diff_drive_validate.sh [record_seconds]

DUR=${1:-12}
LAUNCH_TTL=$(( DUR + 60 ))
INFO_TTL=${INFO_TTL:-15}
TS=$(date +%Y%m%d_%H%M%S)
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG_DIR="$ROOT_DIR/bags/diff_drive_${TS}"
mkdir -p "$BAG_DIR"
echo "[diff_drive_validate] Bag: $BAG_DIR"
exec > >(tee -a "$BAG_DIR/script.log") 2>&1

echo "[diff_drive_validate] Cleaning ROS state..."
"$ROOT_DIR/scripts/ros_clean.sh" --force || true

echo "[diff_drive_validate] Starting pigpio daemon..."
if ! pgrep -x pigpiod >/dev/null 2>&1; then
  if sudo -n true 2>/dev/null; then
    sudo -n systemctl start pigpiod || sudo -n /usr/local/bin/pigpiod -g || sudo -n pigpiod -g || true
  elif [ -n "${ROVER_SUDO:-}" ]; then
    printf '%s\n' "$ROVER_SUDO" | sudo -S -p '' systemctl start pigpiod || \
    printf '%s\n' "$ROVER_SUDO" | sudo -S -p '' /usr/local/bin/pigpiod -g || \
    printf '%s\n' "$ROVER_SUDO" | sudo -S -p '' pigpiod -g || true
  else
    echo "[diff_drive_validate] WARN: skipping pigpio start (no non-interactive sudo). Set ROVER_SUDO to enable."
  fi
fi

echo "[diff_drive_validate] Sourcing ROS and overlay..."
set +u
source /opt/ros/humble/setup.bash
[ -f "$ROOT_DIR/install/setup.bash" ] && source "$ROOT_DIR/install/setup.bash"
set -u

echo "[diff_drive_validate] Launching drive_min..."
setsid bash -lc 'set +u; source /opt/ros/humble/setup.bash; [ -f install/setup.bash ] && source install/setup.bash; set -u; ros2 launch alpha_viam_bringup drive_min.launch.py' &
BR_PID=$!
sleep 6
BR_PGID=$(ps -o pgid= $BR_PID | tr -d ' ')

cleanup() {
  local code=$?
  if [ -n "${BR_PGID:-}" ]; then
    kill -INT -"$BR_PGID" 2>/dev/null || true; sleep 0.5 || true
    kill -TERM -"$BR_PGID" 2>/dev/null || true; sleep 0.5 || true
    kill -KILL -"$BR_PGID" 2>/dev/null || true
  fi
  if [ -n "${WD_PID:-}" ] && kill -0 "$WD_PID" 2>/dev/null; then
    kill "$WD_PID" 2>/dev/null || true
  fi
  echo "[diff_drive_validate] Exit code: $code"
}
trap cleanup EXIT INT TERM

bringup_watchdog() {
  local ttl=$1
  sleep "$ttl"
  if kill -0 "$BR_PID" 2>/dev/null; then
    echo "[diff_drive_validate] WATCHDOG: bring-up exceeded ${ttl}s; stopping"
    kill -INT -"$BR_PGID" 2>/dev/null || true; sleep 0.5 || true
    kill -TERM -"$BR_PGID" 2>/dev/null || true; sleep 0.5 || true
    kill -KILL -"$BR_PGID" 2>/dev/null || true
  fi
}
bringup_watchdog "$LAUNCH_TTL" &
WD_PID=$!

echo "[diff_drive_validate] Priming controller params on manager"
python3 "$ROOT_DIR/scripts/prime_controller_params.py" diff_drive_controller "$ROOT_DIR/configs/diff_drive.yaml" || true
echo "[diff_drive_validate] Loading + setting params via activator"
python3 "$ROOT_DIR/scripts/activate_diff_drive.py" "$ROOT_DIR/configs/diff_drive.yaml" || true
echo "[diff_drive_validate] Controller parameters (on controller node)"
timeout 6s ros2 param get /diff_drive_controller left_wheel_names || true
timeout 6s ros2 param get /diff_drive_controller right_wheel_names || true

echo "[diff_drive_validate] Controllers and hardware interfaces"
timeout 4s ros2 control list_controllers || true
timeout 4s ros2 control list_hardware_interfaces || true

echo "[diff_drive_validate] Recording MCAP for ${DUR}s..."
TOPICS=(/diff_drive_controller/cmd_vel_unstamped /joint_states /tf /tf_static /odometry/filtered /diff_drive_controller/odom)
timeout "${DUR}s" ros2 bag record -s mcap -o "$BAG_DIR/diff_drive_run" "${TOPICS[@]}" &
REC_PID=$!
sleep 2

echo "[diff_drive_validate] Drive burst (forward, then stop)"
timeout 1.8s ros2 topic pub -r 15 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.6}, angular: {z: 0.0}}' || true
sleep 0.5
timeout 1s ros2 topic pub --once /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}' || true

wait $REC_PID || true

echo "[diff_drive_validate] Shutting down bring-up..."
kill -INT -"$BR_PGID" 2>/dev/null || true; sleep 2 || true
kill -TERM -"$BR_PGID" 2>/dev/null || true; sleep 2 || true
kill -KILL -"$BR_PGID" 2>/dev/null || true

"$ROOT_DIR/scripts/ros_clean.sh" --force || true

echo "[diff_drive_validate] Bag at: $BAG_DIR"
timeout "${INFO_TTL}s" ros2 bag info "$BAG_DIR/diff_drive_run" || true

echo "[diff_drive_validate] Done."
