#!/usr/bin/env bash
set -euo pipefail

# Record a short MCAP bag with common rover topics.
# Usage: scripts/record_mcap.sh [duration_seconds]

DURATION="${1:-30}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTDIR="bags/samples"
mkdir -p "$OUTDIR"
OUTFILE="$OUTDIR/${STAMP}_bench.mcap"

echo "Recording ${DURATION}s to $OUTFILE"
ros2 bag record \
  --storage mcap \
  --output "$OUTFILE" \
  /tf /tf_static /odom /joint_states /cmd_vel /imu/data /power/ina219 /scan \
  --max-cache-size 256 \
  --max-bag-size 0 \
  --max-bag-duration $DURATION

echo "Done: $OUTFILE"

