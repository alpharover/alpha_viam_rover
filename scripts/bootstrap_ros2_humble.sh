#!/usr/bin/env bash
set -euo pipefail

# Bootstrap ROS 2 Humble on Ubuntu 22.04 (RPi4 friendly).
# Idempotent: safe to re-run. Requires sudo.

GREEN="\033[0;32m"; YELLOW="\033[1;33m"; RED="\033[0;31m"; NC="\033[0m"
say() { echo -e "${GREEN}[*]${NC} $*"; }
warn() { echo -e "${YELLOW}[!]${NC} $*"; }
die() { echo -e "${RED}[x]${NC} $*"; exit 1; }

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  warn "This script will use sudo as needed. You may be prompted for your password."
fi

source /etc/os-release || die "Cannot read /etc/os-release"
if [[ "${ID}" != "ubuntu" || "${VERSION_ID}" != "22.04" ]]; then
  die "This bootstrap targets Ubuntu 22.04 (jammy). Detected: ${PRETTY_NAME}"
fi

say "Updating APT and base packages"
sudo apt-get update -y
sudo apt-get install -y --no-install-recommends \
  locales curl gnupg2 lsb-release ca-certificates software-properties-common apt-transport-https

say "Configuring UTF-8 locale"
sudo locale-gen en_US en_US.UTF-8 >/dev/null
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

say "Enabling 'universe' repo (idempotent)"
sudo add-apt-repository -y universe >/dev/null 2>&1 || true

KEYRING=/usr/share/keyrings/ros-archive-keyring.gpg
LIST=/etc/apt/sources.list.d/ros2.list

say "Adding ROS 2 APT key (if missing)"
if [[ ! -f "$KEYRING" ]]; then
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    sudo gpg --dearmor -o "$KEYRING"
else
  warn "Keyring already present: $KEYRING"
fi

say "Configuring ROS 2 APT source (if missing)"
UBUNTU_CODENAME=${UBUNTU_CODENAME:-$(. /etc/os-release && echo "$UBUNTU_CODENAME")}
if [[ ! -f "$LIST" ]]; then
  echo "deb [arch=$(dpkg --print-architecture) signed-by=$KEYRING] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" | \
    sudo tee "$LIST" >/dev/null
else
  warn "APT source already present: $LIST"
fi

say "Installing ROS 2 Humble (ros-base)"
sudo apt-get update -y
sudo apt-get install -y ros-humble-ros-base

say "Installing dev tools (colcon, rosdep, vcstool, build-essential)"
sudo apt-get install -y python3-colcon-common-extensions python3-rosdep python3-vcstool build-essential

say "Initializing rosdep (idempotent)"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init || true
else
  warn "rosdep sources already initialized"
fi
rosdep update || warn "rosdep update failed (network hiccup?). You can re-run later."

say "Adding current user to helpful groups (dialout,i2c,spi,gpio)"
CURRENT_USER=${SUDO_USER:-$USER}
for grp in dialout i2c spi gpio; do
  if getent group "$grp" >/dev/null 2>&1; then
    sudo usermod -a -G "$grp" "$CURRENT_USER" || true
  fi
done
warn "You may need to log out/in for new group memberships to take effect."

SOURCELINE='source /opt/ros/humble/setup.bash'
if ! grep -qs "$SOURCELINE" "/home/${CURRENT_USER}/.bashrc"; then
  say "Adding ROS 2 environment sourcing to ~/.bashrc"
  echo "$SOURCELINE" | sudo tee -a "/home/${CURRENT_USER}/.bashrc" >/dev/null
else
  warn "ROS 2 environment already sourced in ~/.bashrc"
fi

say "Verifying installation"
set +e
source /opt/ros/humble/setup.bash 2>/dev/null
if command -v ros2 >/dev/null 2>&1; then
  ros2 --version || true
  ros2 doctor || true
else
  warn "'ros2' not found in PATH yet (new shell may be required)."
fi

say "Bootstrap complete. Open a new shell or run: 'source /opt/ros/humble/setup.bash'"

