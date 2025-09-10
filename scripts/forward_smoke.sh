#!/usr/bin/env bash
set -euo pipefail

DUR=${1:-8}

pass() { printf "\e[32m%s\e[0m\n" "$*"; }
warn() { printf "\e[33m%s\e[0m\n" "$*"; }
err()  { printf "\e[31m%s\e[0m\n" "$*"; }

if ! pgrep -x pigpiod >/dev/null 2>&1; then
  sudo systemctl start pigpiod || sudo /usr/local/bin/pigpiod || true
  sleep 0.5
fi

set +u
source /opt/ros/humble/setup.bash
[ -f install/setup.bash ] && source install/setup.bash
set -u

LOGROOT="log/agent_runs"; mkdir -p "$LOGROOT"; ts=$(date +%Y%m%d_%H%M%S); LOGDIR="$LOGROOT/$ts"; mkdir -p "$LOGDIR"
LAUNCH_LOG="$LOGDIR/drive_forward.log"

setsid ros2 launch alpha_viam_bringup drive_forward.launch.py >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
sleep 3

if ! timeout 20s python3 scripts/activate_forward.py configs/wheels_forward.yaml >"$LOGDIR/activate_forward.log" 2>&1; then
  warn "activate_forward helper reported a non-zero exit; continuing"
fi

if ! timeout 4s ros2 control list_controllers | rg -q "left_wheel_velocity_controller.*active"; then
  err "Left controller not active"; fi
if ! timeout 4s ros2 control list_controllers | rg -q "right_wheel_velocity_controller.*active"; then
  err "Right controller not active"; fi

setsid timeout ${DUR}s ros2 bag record -s mcap -o "$LOGDIR/bag" \
  /joint_states /tf /tf_static /left_wheel_velocity_controller/commands /right_wheel_velocity_controller/commands >/dev/null 2>&1 &
REC_PID=$!
sleep 1

timeout 2s ros2 topic pub -r 15 /left_wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray '{data: [2.0]}' >/dev/null 2>&1 || true
timeout 2s ros2 topic pub -r 15 /right_wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray '{data: [2.0]}' >/dev/null 2>&1 || true
sleep 0.6
timeout 2s ros2 topic pub -r 15 /left_wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray '{data: [-1.5]}' >/dev/null 2>&1 || true
timeout 2s ros2 topic pub -r 15 /right_wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray '{data: [-1.5]}' >/dev/null 2>&1 || true

sleep 1
kill $REC_PID 2>/dev/null || true
sleep 0.4
kill $LAUNCH_PID 2>/dev/null || true

pass "Done. Logs: $LOGDIR"
