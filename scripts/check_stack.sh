#!/usr/bin/env bash
set -euo pipefail

echo "== ROS 2 Environment =="
if command -v ros2 >/dev/null 2>&1; then
  echo "ros2: $(command -v ros2)"
  ros2 --help | head -n 1 || true
  echo
  echo "== ros2 doctor =="
  ros2 doctor || true
else
  echo "ros2 CLI not found in PATH. If installed, run: source /opt/ros/humble/setup.bash"
fi

echo
echo "== colcon =="
if command -v colcon >/dev/null 2>&1; then
  echo "colcon: $(command -v colcon)"
else
  echo "colcon not found. Install: sudo apt-get install -y python3-colcon-common-extensions"
fi

echo
echo "== Groups for current user =="
id || true
