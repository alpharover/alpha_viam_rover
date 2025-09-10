#!/usr/bin/env bash
set -euo pipefail

# Kill common ROS 2 processes that may have been left running by previous attempts.
# Usage: scripts/ros_clean.sh [--force]

FORCE=${1:-}

echo "[ros_clean] Searching for stale ROS 2 processes..."

PATS=(
  "/opt/ros/.*/lib/robot_state_publisher/robot_state_publisher"
  "/opt/ros/.*/lib/robot_localization/ekf_node"
  "/opt/ros/.*/lib/diagnostic_aggregator/aggregator_node"
  "foxglove_bridge"
  "/opt/ros/.*/bin/ros2 bag record"
  "ina219_monitor"
  "mpu6050_node"
  "/opt/ros/.*/lib/controller_manager/ros2_control_node"
  # helper scripts and spawners
  "activate_diff_drive.py"
  "activate_controllers.py"
  "controller_manager spawner"
  "ros2 run controller_manager spawner"
  "ros2 launch alpha_viam_bringup base_bringup.launch.py"
  "ros2 launch alpha_viam_bringup drive_min.launch.py"
)

FOUND_PIDS=()
for pat in "${PATS[@]}"; do
  while read -r pid; do
    [[ -z "$pid" ]] && continue
    FOUND_PIDS+=("$pid")
  done < <(ps -eo pid,comm,args | awk '{print $1" "$2" "$0}' | grep -E "$pat" | awk '{print $1}' | sort -u)
done

if [[ ${#FOUND_PIDS[@]} -eq 0 ]]; then
  echo "[ros_clean] No matching processes found."
  exit 0
fi

echo "[ros_clean] PIDs: ${FOUND_PIDS[*]}"

for pid in "${FOUND_PIDS[@]}"; do
  # Try INT first for graceful shutdown
  kill -INT "$pid" 2>/dev/null || true
done
sleep 2

for pid in "${FOUND_PIDS[@]}"; do
  if ps -p "$pid" >/dev/null 2>&1; then
    echo "[ros_clean] PID $pid still alive; sending TERM"
    kill -TERM "$pid" 2>/dev/null || true
  fi
done
sleep 2

if [[ "${FORCE}" == "--force" ]]; then
  for pid in "${FOUND_PIDS[@]}"; do
    if ps -p "$pid" >/dev/null 2>&1; then
      echo "[ros_clean] PID $pid still alive; sending KILL"
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done
fi

echo "[ros_clean] Done."
