# Foxglove Bridge and Visualization

Notes on running `foxglove_bridge` on the rover and connecting from a dev machine.

- WebSocket port, security
- Panels for IMU, LaserScan, Odometry, TF
- Saved layouts and troubleshooting

TODO: Add step-by-step bringup and screenshots.

Quick start (when on the rover)

- Bridge: launch `foxglove_bridge` on the rover with WebSocket port from `configs/network.yaml` (`foxglove_ws_port`).
- Connect: Open Foxglove on the Mac, add a ROS 2 connection to `ws://<rover-hostname>:<port>` where `<rover-hostname>` matches `rover_hostname`.
- Layout: Import `configs/foxglove/default_layout.json` to get LiDAR, TF, Odometry, JointStates, and Power panels.

Launch helper

- Use the provided launch file to start Foxglove bridge (and optionally keyboard teleop):
  - `ros2 launch launch/teleop_viz.launch.xml`
  - With teleop: `ros2 launch launch/teleop_viz.launch.xml use_teleop:=true`

Recording policy

- Use `scripts/record_mcap.sh <seconds>` to capture short evidence clips in `bags/samples/` with MCAP.
- Name format: `YYYYMMDD_HHMM_env_testcase.mcap` (script uses timestamp + `bench`).
