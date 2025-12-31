# Teleop + Visualization

Drive the rover and (optionally) connect Foxglove for visualization.

## Prereqs

- Rover: ROS 2 Humble + `foxglove_bridge`
- Keyboard teleop options:
  - Recommended (repo-native): `scripts/teleop_keyboard.sh`
  - Optional: `teleop_twist_keyboard` (ROS package)

## Recommended (on the rover)

### One-command session (SSH)

- `cd ~/alpha_viam_rover`
- `scripts/teleop_session.sh`
- ASCII in-terminal video: `scripts/teleop_session.sh --video-mode ascii`
- Disable video: `scripts/teleop_session.sh --video-mode off`

### Two-terminal session

1) Start the full base bring-up and spawn the drive controller:

- `ros2 launch alpha_viam_bringup base_bringup.launch.py spawn_drive:=true`

2) In a second SSH terminal, start keyboard teleop:

- `cd ~/alpha_viam_rover`
- `scripts/teleop_keyboard.sh`

Tuning / troubleshooting:

- This rover has high stiction; defaults are floored to get wheel spin (min speed/turn: `0.72 m/s` and `4.86 rad/s`).
- To go faster: `q` (scales both speed+turn). To go slower: `z` (won’t go below the stiction floor).
- If you have a very narrow terminal, Textual will truncate long fields; widen the SSH window for best layout.
- USB cam (default): teleop starts a real MJPEG stream. Open the URL shown in the TUI (defaults to `http://alpha-viam.local:8080/` or direct stream `http://alpha-viam.local:8080/stream`).
- ASCII fallback (in-TUI): `scripts/teleop_keyboard.sh --video-mode ascii`
- Disable video: `scripts/teleop_keyboard.sh --video-mode off`
- Camera device selection: `--camera-device /dev/video0` (or `/dev/video1`)
- Stream tuning: `--mjpeg-source-size 640x480`, `--mjpeg-fps 30`, `--mjpeg-quality 80`
- ASCII tuning: `--ascii-fps 10`, `--ascii-preview-width 160`, `--ascii-preview-height 120`
- If missing deps: `sudo apt-get install ffmpeg v4l-utils` and `python3 -m pip install --user textual`.

Override the output topic if needed:

- `scripts/teleop_keyboard.sh --topic /controller_manager/cmd_vel_unstamped`

3) On your Mac, open Foxglove and connect to:

- `ws://<rover-hostname>:<foxglove_ws_port>` (see `configs/network.yaml`)

## Web Driver Station (browser)

A single-page "driver station" that combines live video + an on-screen joystick.

On the rover:

- `cd ~/alpha_viam_rover`
- `scripts/driver_station_session.sh`
  - Skip bring-up if it's already running: `scripts/driver_station_session.sh --no-bringup`

On your Mac:

- Open: `http://alpha-viam.local:8090/`
- Hold the on-screen button to drive (release = stop).

Notes:

- Video remains MJPEG via `ustreamer` (low latency): `http://alpha-viam.local:8080/stream`
- Control publishes to `/controller_manager/cmd_vel_unstamped` and enforces a deadman + linear stiction floor (`0.72 m/s`).
- Turning is continuous (no angular stiction floor); use the `MAX_ANG` slider to tune how aggressively it sweeps.
- The UI shows drive telemetry (encoder-derived wheel speeds + odom); `WHEEL_DIFF` near `0` indicates matched wheel speed in straight-line motion.
- ASCII preview mode still exists in the SSH TUI: `scripts/teleop_session.sh --video-mode ascii`

Troubleshooting:

- If `ODOM_*` changes but `WHEEL_*` stays `0.00`, you may be in open-loop odom or the drivetrain isn’t actually moving; confirm the command topic is receiving messages (`/controller_manager/cmd_vel_unstamped`) and that `pigpiod` + `ros2_control_node` are running.
- If the UI behavior doesn’t match your code changes, restart `scripts/driver_station_session.sh` (and rebuild `l298n_hardware` if you changed C++).
- Quick sanity check: `curl http://alpha-viam.local:8090/api/config` should show `"min_turn": 0.0` (no hard-turn floor).

## Dev-only (bridge + optional teleop, no bring-up)

- `ros2 launch ./launch/teleop_viz.launch.py`
- With keyboard (requires `teleop_twist_keyboard`): `ros2 launch ./launch/teleop_viz.launch.py use_teleop:=true`

## Notes

- On this rover (Humble), diff drive listens on `/controller_manager/cmd_vel_unstamped` when `use_stamped_vel=false`.
- `foxglove_bridge` should bind `address=0.0.0.0` for remote connections; use `rover_hostname` only for the client URL.
