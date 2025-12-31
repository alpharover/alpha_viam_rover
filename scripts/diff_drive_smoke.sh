#!/usr/bin/env bash
set -euo pipefail

# One-command smoke test for diff_drive_controller.

DUR=${1:-10}
LAUNCH_TTL=$(( DUR + 60 ))
BUILD_TTL=${BUILD_TTL:-900}
INFO_TTL=${INFO_TTL:-15}
TS=$(date +%Y%m%d_%H%M%S)
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
BAG_DIR="$ROOT_DIR/bags/diff_${TS}"
mkdir -p "$BAG_DIR"

CMD_TOPIC_OVERRIDE=${CMD_TOPIC:-}
LINEAR_X=${LINEAR_X:-0.8}

# Announce bag path early and capture script logs
echo "[diff_drive_smoke] Bag: $BAG_DIR"
exec > >(tee -a "$BAG_DIR/script.log") 2>&1

# Ensure we always tear down background processes
cleanup() {
  local code=$?
  if [ -n "${BR_PGID:-}" ]; then
    echo "[diff_drive_smoke] Cleanup: stopping bring-up (pgid=$BR_PGID)"
    kill -INT -"$BR_PGID" 2>/dev/null || true; sleep 0.5 || true
    kill -TERM -"$BR_PGID" 2>/dev/null || true; sleep 0.5 || true
    kill -KILL -"$BR_PGID" 2>/dev/null || true
  fi
  if [ -n "${WD_PID:-}" ] && kill -0 "$WD_PID" 2>/dev/null; then
    kill "$WD_PID" 2>/dev/null || true
  fi
  echo "[diff_drive_smoke] Exit code: $code"
}
trap cleanup EXIT INT TERM

set +u
source /opt/ros/humble/setup.bash
[ -f "$ROOT_DIR/install/setup.bash" ] && source "$ROOT_DIR/install/setup.bash"
set -u

# Rebuild hardware + bringup to ensure latest plugin/debug is installed
echo "[diff_drive_smoke] Building l298n_hardware + bringup (timeout ${BUILD_TTL}s)..."
set +u
timeout "${BUILD_TTL}s" colcon build --packages-select l298n_hardware alpha_viam_bringup --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo || true
[ -f "$ROOT_DIR/install/setup.bash" ] && source "$ROOT_DIR/install/setup.bash"
set -u

echo "[diff_drive_smoke] Ensuring pigpio daemon is running..."
if ! pgrep -x pigpiod >/dev/null 2>&1; then
  echo "[diff_drive_smoke] Starting pigpiod (non-interactive sudo)..."
  if sudo -n true 2>/dev/null; then
    sudo -n systemctl start pigpiod || sudo -n /usr/local/bin/pigpiod -g || sudo -n pigpiod -g || true
  elif [ -n "${ROVER_SUDO:-}" ]; then
    printf '%s\n' "$ROVER_SUDO" | sudo -S -p '' systemctl start pigpiod || \
    printf '%s\n' "$ROVER_SUDO" | sudo -S -p '' /usr/local/bin/pigpiod -g || \
    printf '%s\n' "$ROVER_SUDO" | sudo -S -p '' pigpiod -g || true
  else
    echo "[diff_drive_smoke] WARN: skipping pigpio start (no non-interactive sudo). Set ROVER_SUDO to enable."
  fi
fi

echo "[diff_drive_smoke] Launching cm_only (manager only)..."
LOCAL_LAUNCH="$ROOT_DIR/bringup/alpha_viam_bringup/alpha_viam_bringup/launch/cm_only.launch.py"
if [ -f "$LOCAL_LAUNCH" ]; then
  LAUNCH_CMD=(ros2 launch "$LOCAL_LAUNCH")
else
  LAUNCH_CMD=(ros2 launch alpha_viam_bringup cm_only.launch.py)
fi
LAUNCH_STR=$(printf '%q ' "${LAUNCH_CMD[@]}")
# Force-add local package overlays for plugin discovery, even if setup.bash was missed
SETSRC=("source /opt/ros/humble/setup.bash" \
        "[ -f install/setup.bash ] && source install/setup.bash" \
        "[ -f install/l298n_hardware/share/l298n_hardware/local_setup.bash ] && source install/l298n_hardware/share/l298n_hardware/local_setup.bash" \
        "[ -f install/alpha_viam_bringup/share/alpha_viam_bringup/local_setup.bash ] && source install/alpha_viam_bringup/share/alpha_viam_bringup/local_setup.bash")
SETSRC_STR=$(IFS=';'; echo "${SETSRC[*]}")
setsid bash -lc "set +u; ${SETSRC_STR}; set -u; echo '[diff_drive_smoke] AMENT_PREFIX_PATH='\"\$AMENT_PREFIX_PATH\"; ${LAUNCH_STR} > '${BAG_DIR}/bringup.log' 2>&1" &
BR_PID=$!
sleep 5
BR_PGID=$(ps -o pgid= $BR_PID | tr -d ' ')

# Watchdog to prevent runaway launches
bringup_watchdog() {
  local ttl=$1
  sleep "$ttl"
  if kill -0 "$BR_PID" 2>/dev/null; then
    echo "[diff_drive_smoke] WATCHDOG: bring-up exceeded ${ttl}s, stopping (pgid=$BR_PGID)"
    kill -INT -"$BR_PGID" 2>/dev/null || true; sleep 0.5 || true
    kill -TERM -"$BR_PGID" 2>/dev/null || true; sleep 0.5 || true
    kill -KILL -"$BR_PGID" 2>/dev/null || true
  fi
}
bringup_watchdog "$LAUNCH_TTL" &
WD_PID=$!

echo "[diff_drive_smoke] Waiting for /controller_manager services..."
svc_ready=0
for i in $(seq 1 24); do
  if ros2 service list | grep -q "/controller_manager/list_controllers"; then
    svc_ready=1
    break
  fi
  sleep 0.5
done
if [ "$svc_ready" -ne 1 ]; then
  echo "[diff_drive_smoke] ERROR: controller_manager services not available after 12s."
  echo "[diff_drive_smoke] Hint: overlay not sourced or plugin discovery failed."
  echo "[diff_drive_smoke] AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH" | cut -c1-240
  # Tear down bring-up before exiting
  kill -INT -"$BR_PGID" 2>/dev/null || true; sleep 1 || true
  kill -TERM -"$BR_PGID" 2>/dev/null || true; sleep 1 || true
  kill -KILL -"$BR_PGID" 2>/dev/null || true
  exit 2
fi

echo "[diff_drive_smoke] Spawn controllers via spawner"
timeout 10s ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /controller_manager || true
timeout 20s ros2 run controller_manager spawner diff_drive_controller --controller-manager /controller_manager --activate --unload-on-kill || true
# Ensure params are present even if spawner --param-file is ineffective on this image
python3 "$ROOT_DIR/scripts/prime_controller_params.py" diff_drive_controller "$ROOT_DIR/configs/diff_drive.yaml" || true
python3 "$ROOT_DIR/scripts/activate_diff_drive.py" "$ROOT_DIR/configs/diff_drive.yaml" || true

echo "[diff_drive_smoke] Controllers:" && (ros2 control list_controllers || true)
echo "[diff_drive_smoke] Interfaces:" && (ros2 control list_hardware_interfaces || true)

echo "[diff_drive_smoke] Param sanity (wheel names on controller node)"
timeout 4s ros2 param get /diff_drive_controller left_wheel_names || true
timeout 4s ros2 param get /diff_drive_controller right_wheel_names || true

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
  CMD_TOPIC="$(detect_topic /diff_drive_controller/cmd_vel_unstamped /controller_manager/cmd_vel_unstamped /cmd_vel_unstamped /cmd_vel || true)"
  if [[ -n "$CMD_TOPIC" ]]; then
    echo "[diff_drive_smoke] Auto-detected CMD_TOPIC=$CMD_TOPIC"
  else
    CMD_TOPIC="/diff_drive_controller/cmd_vel_unstamped"
    echo "[diff_drive_smoke] WARN: could not auto-detect cmd topic; using $CMD_TOPIC"
  fi
else
  CMD_TOPIC="$CMD_TOPIC_OVERRIDE"
  echo "[diff_drive_smoke] Using CMD_TOPIC override: $CMD_TOPIC"
fi

STAMPED_TOPIC="$(detect_topic /diff_drive_controller/cmd_vel /controller_manager/cmd_vel || true)"
if [[ -n "$STAMPED_TOPIC" ]]; then
  echo "[diff_drive_smoke] Also detected STAMPED_TOPIC=$STAMPED_TOPIC"
fi

echo "[diff_drive_smoke] Subscriber sanity for cmd topics"
timeout 2s ros2 topic info "$CMD_TOPIC" || true
if [[ -n "$STAMPED_TOPIC" ]]; then
  timeout 2s ros2 topic info "$STAMPED_TOPIC" || true
fi

echo "[diff_drive_smoke] Recording ${DUR}s MCAP..."
TOPICS=("$CMD_TOPIC" /joint_states /tf /tf_static /rosout /controller_manager/odom /diff_drive_controller/odom)
if [[ -n "$STAMPED_TOPIC" ]]; then
  TOPICS+=("$STAMPED_TOPIC")
fi
timeout "${DUR}s" ros2 bag record -s mcap -o "$BAG_DIR/diff_run" "${TOPICS[@]}" || true &
REC_PID=$!
sleep 1

echo "[diff_drive_smoke] Forward burst on ${CMD_TOPIC} (linear_x=${LINEAR_X})"
python3 "$ROOT_DIR/scripts/pub_twist.py" --topic "$CMD_TOPIC" --duration 1.6 --rate 15 --linear_x "$LINEAR_X" || true
if [[ -n "$STAMPED_TOPIC" ]]; then
  python3 "$ROOT_DIR/scripts/pub_twist.py" --topic "$STAMPED_TOPIC" --duration 1.6 --rate 15 --linear_x "$LINEAR_X" --stamped || true
fi
sleep 0.5
echo "[diff_drive_smoke] Stop"
python3 "$ROOT_DIR/scripts/pub_twist.py" --topic "$CMD_TOPIC" --duration 0.2 --rate 3 --linear_x 0.0 || true
if [[ -n "$STAMPED_TOPIC" ]]; then
  python3 "$ROOT_DIR/scripts/pub_twist.py" --topic "$STAMPED_TOPIC" --duration 0.2 --rate 3 --linear_x 0.0 --stamped || true
fi

wait $REC_PID || true

echo "[diff_drive_smoke] Stopping..."
kill -INT -"$BR_PGID" 2>/dev/null || true; sleep 1 || true
kill -TERM -"$BR_PGID" 2>/dev/null || true; sleep 1 || true
kill -KILL -"$BR_PGID" 2>/dev/null || true

"$ROOT_DIR/scripts/ros_clean.sh" --force || true

echo "[diff_drive_smoke] Bag: $BAG_DIR"
timeout "${INFO_TTL}s" ros2 bag info "$BAG_DIR/diff_run" || true

# Write a brief run summary for the architect
{
  echo "timestamp=$(date -Iseconds)"
  echo "bag_dir=$BAG_DIR"
  echo "cmd_topic=${CMD_TOPIC:-}"
  echo "stamped_topic=${STAMPED_TOPIC:-}"
  echo "param_left=$(ros2 param get /diff_drive_controller left_wheel_names 2>/dev/null | tr -d '\n')"
  echo "param_right=$(ros2 param get /diff_drive_controller right_wheel_names 2>/dev/null | tr -d '\n')"
  echo "controllers=$(ros2 control list_controllers 2>/dev/null | tr '\n' ';')"
  echo "interfaces=$(ros2 control list_hardware_interfaces 2>/dev/null | tr '\n' ';')"
  echo "rosout_log=$BAG_DIR/bringup.log"
} > "$BAG_DIR/run_summary.txt" 2>/dev/null || true
echo "[diff_drive_smoke] Summary: $BAG_DIR/run_summary.txt"
