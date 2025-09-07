# Teleop + Visualization

Launch the Foxglove bridge and (optionally) keyboard teleop.

Prereqs

- `foxglove_bridge` installed on the rover
- Optional: `teleop_twist_keyboard` for keyboard control

Launch

- Preferred (reads `configs/network.yaml`):
  - `ros2 launch launch/teleop_viz.launch.py`
  - Overrides: `ros2 launch launch/teleop_viz.launch.py ws_port:=8765 ws_address:=0.0.0.0 use_teleop:=true`
- XML fallback (manual args only):
  - `ros2 launch launch/teleop_viz.launch.xml ws_port:=8765 ws_address:=0.0.0.0 use_teleop:=true`

Notes

- The WebSocket port should correspond to `foxglove_ws_port` in `configs/network.yaml`.
- In Foxglove, connect to `ws://<rover-hostname>:<port>` (hostname from `configs/network.yaml`).
- Keyboard teleop publishes to `/cmd_vel`. Ensure your controller stack (diff_drive) is running to act on it.
