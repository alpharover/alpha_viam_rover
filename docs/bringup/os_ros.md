# OS and ROS 2 Bring-up (Ubuntu 22.04, RPi4)

This guide installs ROS 2 Humble on Ubuntu Server 22.04 (jammy) for Raspberry Pi 4. It is safe to re-run and targets a headless setup.

Prereqs
- Fresh Ubuntu 22.04 64-bit on RPi4 with network access.
- User with sudo (default: `ubuntu` or your own).

Quick Start (automated)
1) From repo root, run the bootstrap script:
```
scripts/bootstrap_ros2_humble.sh
```
2) Open a new shell (or run `source /opt/ros/humble/setup.bash`).
3) Verify:
```
scripts/check_stack.sh
```

Manual Steps (if you prefer)
1) Locales and repos
```
sudo apt-get update -y
sudo apt-get install -y locales curl gnupg2 lsb-release ca-certificates software-properties-common apt-transport-https
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository -y universe
```
2) ROS 2 APT key and source
```
sudo mkdir -p /usr/share/keyrings
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
  sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
```
3) Install ROS 2 Humble (ros-base)
```
sudo apt-get update -y
sudo apt-get install -y ros-humble-ros-base
```
4) Dev tools and rosdep
```
sudo apt-get install -y python3-colcon-common-extensions python3-rosdep python3-vcstool build-essential
sudo rosdep init || true
rosdep update
```
5) Shell environment
```
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```
6) Useful groups (serial/I²C/SPI/GPIO)
```
sudo usermod -a -G dialout,i2c,spi,gpio $USER
# log out/in for group changes to take effect
```

Verification
- `ros2 --version` prints a Humble version.
- `ros2 doctor` reports OK (some warnings are fine on a clean system).
- `colcon --help` is available.

Next
- Proceed to `bringup/AGENTS.md` acceptance: create base bring-up launch, ensure `/tf`, `/odom`, and `/diagnostics` appear, and run a 10-minute stability test.
