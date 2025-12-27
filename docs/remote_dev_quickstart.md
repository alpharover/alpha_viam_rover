# Rover Remote Dev + Testing Quickstart

Goal: re-establish contact with the rover and resume development/testing without hunting through old notes.

Last verified: 2025-12-26

## Known-good identifiers

- Rover hostname (mDNS): `alpha-viam.local`
- Rover SSH user: `alpha_viam`
- Rover repo path: `~/alpha_viam_rover` (i.e., `/home/alpha_viam/alpha_viam_rover`)
- Git remote on rover:
  - `origin git@github.com:alpharover/alpha_viam_rover.git`
- Last observed active branch on rover:
  - `chore/lint-fixes` (tracks `origin/chore/lint-fixes`)
- Useful repo config references:
  - `ansible/inventory.example` (host/user)
  - `configs/network.yaml` (hostname, Foxglove port, suggested ROS_DOMAIN_ID)

## Find the rover on the LAN

### Preferred: use mDNS/Bonjour

```bash
# resolve
ping -c 1 alpha-viam.local

dns-sd -G v4 alpha-viam.local

# discover advertised SSH hosts
dns-sd -B _ssh._tcp local
```

If mDNS is flaky, check your router/DHCP leases for host `alpha-viam`.

### What “contact” looks like

- `alpha-viam.local` resolves and pings
- SSH connects: `ssh alpha_viam@alpha-viam.local`

## Connect (SSH)

```bash
ssh alpha_viam@alpha-viam.local
```

Once connected, sanity check:

```bash
hostname
whoami
cd ~/alpha_viam_rover
git status -sb
```

## How we were syncing development

Development was synced via git between:
- **Your laptop** (this repo checkout)
- **The rover** at `~/alpha_viam_rover`

### Recommended workflow: edit locally, run on rover

On your laptop:

```bash
cd /path/to/alpha_viam_rover

# make changes

git status
# run unit/config tests (fast, no ROS required)
python3 -m pytest -q

# optional local lint (matches CI intent)
ruff check .
black --check .
yamllint .

# commit + push your branch
git add -A
git commit -m "..."
git push -u origin HEAD
```

On the rover:

```bash
ssh alpha_viam@alpha-viam.local
cd ~/alpha_viam_rover

git fetch --all --prune
# if you’re tracking a branch already
git pull --ff-only

# OR if you pushed a new branch name from laptop
git checkout <your-branch>
git pull --ff-only
```

### Alternate workflow: quick rover edits, then pull back

On the rover:

```bash
cd ~/alpha_viam_rover
# edit, commit

git push -u origin HEAD
```

On your laptop:

```bash
git fetch --all --prune
git checkout <branch>
git pull --ff-only
```

## Tools on the rover

- `codex` is installed at `/usr/bin/codex`.
  - Typical usage: SSH in, `cd ~/alpha_viam_rover`, then run `codex`.

## ROS 2 / networking notes

- `configs/network.yaml` suggests `ros_domain_id: 20`, but it may not be exported automatically in shells/services.
- If you want to run `ros2 ...` on your laptop and see/talk to the rover’s nodes, ensure both sides match:

```bash
export ROS_DOMAIN_ID=20
```

If you’re unsure what the rover is using, start with default `ROS_DOMAIN_ID` (unset/0). If discovery doesn’t work, try `20` on both.

## Build on the rover (ROS 2 Humble)

From an SSH session:

```bash
cd ~/alpha_viam_rover

# ROS base
source /opt/ros/humble/setup.bash

# build (adjust package list as needed)
colcon build --packages-select l298n_hardware alpha_viam_bringup --symlink-install

# overlay
source install/setup.bash
```

## Safety rules (read before any motion)

- Wheels off the ground for first tests.
- Don’t run pigpio “direct” tests while `controller_manager` (`ros2_control_node`) is running.
- Use cleanup aggressively between runs:

```bash
scripts/ros_clean.sh --force
```

## Motor / drive smoke tests (rover)

### 0) Ensure pigpio daemon is running

```bash
sudo systemctl start pigpiod || sudo /usr/local/bin/pigpiod || sudo pigpiod
```

### 1) Pigpio-only sanity (no ROS control)

```bash
scripts/pigs_smoke.sh
```

### 2) Direct ROS driver demo (no ros2_control)

```bash
scripts/direct_smoke.sh 10
```

### 3) diff_drive_controller (ros2_control) smoke

```bash
scripts/diff_drive_smoke.sh 12
```

### 4) diff_drive_controller validation (checks params + interfaces)

```bash
scripts/diff_drive_validate.sh 12
```

### 5) Full bring-up smoke (base_bringup + cmd topic publish)

```bash
# defaults publish to /diff_drive_controller/cmd_vel_unstamped
scripts/drive_smoke.sh 25

# override if needed
CMD_TOPIC=/cmd_vel LINEAR_X=1.0 scripts/drive_smoke.sh 25
```

## Launch files you’ll likely use

- Minimal controller manager only:
  - `ros2 launch alpha_viam_bringup cm_only.launch.py`
- Minimal drive stack:
  - `ros2 launch alpha_viam_bringup drive_min.launch.py`
- Full base stack (IMU, power, diagnostics, Foxglove, controller manager):
  - `ros2 launch alpha_viam_bringup base_bringup.launch.py spawn_drive:=true`
- Direct driver demo:
  - `ros2 launch alpha_viam_bringup drive_direct.launch.py`

## Foxglove

- Port is configured as `8765` in `configs/network.yaml`.
- It will only be open when `foxglove_bridge` is running (e.g., via `base_bringup.launch.py`).
- Quick check from laptop:

```bash
# should connect when bringup is running
python3 -c 'import socket; s=socket.create_connection(("alpha-viam.local",8765),timeout=2); s.close(); print("ok")'
```

## Known roadblocks / fast triage

### A) Pluginlib can’t find `l298n_hardware/L298NSystemHardware`

Symptoms (typical): controller manager logs “class … does not exist”.

Fast checks:

```bash
# after sourcing install
echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | head

ros2 pkg prefix l298n_hardware
```

Notes:
- Missing/empty `AMENT_PREFIX_PATH` breaks ament index discovery.
- Under launch files, use `additional_env` (merge) rather than replacing the entire environment.
- Avoid relying on CWD for paths; prefer absolute repo-root-resolved paths.

### B) diff_drive_controller fails init because wheel params are empty

This was treated as a Humble requirement: provide controller params at load time via `params_file`.

References:
- `docs/ADR/0005-humble-controller-activation-report.md`
- `configs/controllers.yaml` (uses `diff_drive_controller.params_file`)
- `configs/diff_drive_params.yaml`

Sanity checks:

```bash
ros2 control list_controllers
ros2 param get /diff_drive_controller left_wheel_names
ros2 param get /diff_drive_controller right_wheel_names
```

### C) Stale spawners / zombie nodes

Use:

```bash
scripts/ros_clean.sh --force
ros2 daemon stop || true
ros2 daemon start || true
```

## Quick “resume” checklist

1) Laptop: confirm you’re on the right Wi‑Fi/Ethernet, then `ping alpha-viam.local`.
2) `ssh alpha_viam@alpha-viam.local`.
3) Rover: `cd ~/alpha_viam_rover && git pull --ff-only`.
4) Rover: build + source overlay.
5) Rover: `scripts/ros_clean.sh --force`.
6) Rover: run `scripts/diff_drive_smoke.sh 12` (or `direct_smoke.sh` if you’re demoing).

