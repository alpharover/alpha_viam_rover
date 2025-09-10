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
TS=$(date +%Y%m%d_%H%M%S)
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG_DIR="$ROOT_DIR/bags/diff_drive_${TS}"
mkdir -p "$BAG_DIR"

echo "[diff_drive_validate] Cleaning ROS state..."
"$ROOT_DIR/scripts/ros_clean.sh" --force || true

echo "[diff_drive_validate] Starting pigpio daemon..."
if command -v systemctl >/dev/null 2>&1; then
  sudo systemctl start pigpiod || sudo /usr/local/bin/pigpiod || sudo pigpiod || true
else
  sudo /usr/local/bin/pigpiod || sudo pigpiod || true
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

echo "[diff_drive_validate] Controller parameters (on controller node)"
timeout 4s ros2 param get /diff_drive_controller left_wheel_names || true
timeout 4s ros2 param get /diff_drive_controller right_wheel_names || true

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
ros2 bag info "$BAG_DIR/diff_drive_run" || true

echo "[diff_drive_validate] Done."

