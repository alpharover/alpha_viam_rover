#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

set +u
source /opt/ros/humble/setup.bash
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  source "$ROOT_DIR/install/setup.bash"
fi
set -u

# Dependencies (non-interactive; requires passwordless sudo)
python3 -c 'import aiohttp' >/dev/null 2>&1 || (sudo -n apt-get update -y && sudo -n apt-get install -y python3-aiohttp)
command -v ustreamer >/dev/null 2>&1 || (sudo -n apt-get update -y && sudo -n apt-get install -y ustreamer)

exec python3 -u "$ROOT_DIR/scripts/driver_station_server.py" "$@"
