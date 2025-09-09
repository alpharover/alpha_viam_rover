#!/usr/bin/env bash
set -euo pipefail

# Record a short MCAP bag with common rover topics.
# Usage: scripts/record_mcap.sh [duration_seconds]
# Note: rosbag2 does not auto-stop on duration; use `timeout` to limit runtime.

DURATION="${1:-30}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BASE="${STAMP}_bench"
OUTROOT="bags/samples"
BAGDIR="$OUTROOT/$BASE"
mkdir -p "$OUTROOT"

echo "Recording ${DURATION}s to $BAGDIR (storage: mcap)"
set +e
timeout "${DURATION}s" ros2 bag record \
  --storage mcap \
  --output "$BAGDIR" \
  /tf /tf_static /odom /joint_states /cmd_vel /imu/data /power/ina219 /scan \
  --max-cache-size 256 \
  --max-bag-size 0
rc=$?
set -e
if [[ $rc -eq 124 ]]; then
  echo "Recorder stopped after ${DURATION}s (timeout)."
elif [[ $rc -ne 0 ]]; then
  echo "ros2 bag record exited with code $rc" >&2
  exit $rc
fi

echo "Done: $BAGDIR"
