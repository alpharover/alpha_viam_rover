# Foxglove Bridge and Visualization

Notes on running `foxglove_bridge` on the rover and connecting from a dev machine.

- WebSocket port, security
- Panels for IMU, LaserScan, Odometry, TF
- Saved layouts and troubleshooting

Step-by-step

1) Ensure the bridge launches on the rover.
   - Via teleop launch: `ros2 launch launch/teleop_viz.launch.xml`
   - Or via bring-up: `ros2 launch alpha_viam_bringup base_bringup.launch.py`
2) Confirm the port/address from `configs/network.yaml` (defaults: `0.0.0.0:8765`).
3) On your Mac, open Foxglove and add a ROS 2 connection: `ws://<rover-hostname>:<port>`.
4) Import a layout from `configs/foxglove/` (start with `default_layout.json`).
5) Verify topics match layout (adjust as needed):
   - `/tf`, `/tf_static` (TF Tree)
   - `/odometry/filtered` (EKF Odometry)
   - `/scan` (LiDAR, when present)
   - `/joint_states` (Joints)
   - Power (INA219): `/power/bus_voltage`, `/power/current`, `/power/power` (watts)
   - IMU: `/imu/data` (raw) and `/imu/data_fused` (filtered orientation)
   - Wi‑Fi telemetry (wlan1): `/wifi/signal_dBm`, `/wifi/link_ok`, `/wifi/iface`, `/wifi/flap_count`

Screenshot (add later)

Place a screenshot at `docs/tools/images/foxglove_layout.png` once you’ve connected; add it to your PR as evidence.

Quick start (when on the rover)

- Bridge: launch `foxglove_bridge` on the rover with WebSocket port from `configs/network.yaml` (`foxglove_ws_port`).
- Connect: Open Foxglove on the Mac, add a ROS 2 connection to `ws://<rover-hostname>:<port>` where `<rover-hostname>` matches `rover_hostname`.
- Layout: Import a layout from `configs/foxglove/` (start with `default_layout.json`) to get LiDAR, TF, Odometry, JointStates, and Power panels.

Layouts

- `configs/foxglove/default_layout.json` — balanced starter layout for LiDAR, TF, Odometry, Joints, Power (edit topics as needed).
- `configs/foxglove/viam_rover_bringup_legacy.json` — legacy bring-up layout used during early validation.
- `configs/foxglove/viam_rover_rnd_dashboard_legacy.json` — legacy R&D dashboard with additional panels.

Tip: Load a legacy layout first to explore, then export your personalized version and commit it back under `configs/foxglove/` with a clear name.

Wi‑Fi telemetry tips

- Pin panels to wlan1: use `/wifi/signal_dBm` and confirm `/wifi/iface` reads `wlan1`.
- If the Alfa re‑enumerates during development, `/wifi/link_ok` may briefly drop; the layout will stay on wlan1 and report disconnected rather than switching to wlan0.
- For testing over the data plane, connect Foxglove to the wlan1 IP (e.g., `ws://192.168.0.106:8765`); keep SSH on wlan0 to avoid control impact.

Launch helper

- Use the provided launch file to start Foxglove bridge (and optionally keyboard teleop):
  - `ros2 launch launch/teleop_viz.launch.xml`
  - With teleop: `ros2 launch launch/teleop_viz.launch.xml use_teleop:=true`

Recording policy

- Use `scripts/record_mcap.sh <seconds>` to capture short evidence clips in `bags/samples/` with MCAP.
- Name format: `YYYYMMDD_HHMM_env_testcase.mcap` (script uses timestamp + `bench`).

Common pitfalls

- Avoid publishing different message types on the same topic name. For example, do not reuse `/set_pose` for `Twist` and `PoseWithCovarianceStamped` from multiple panels — it will cause foxglove_bridge schema conflicts. Use standard topic names:
  - `/cmd_vel` (geometry_msgs/Twist) for velocity teleop
  - `/initialpose` (geometry_msgs/PoseWithCovarianceStamped) for 2D pose estimate
  - `/move_base_simple/goal` (geometry_msgs/PoseStamped) for 2D nav goal
  - `/clicked_point` (geometry_msgs/PointStamped) for clicked points
