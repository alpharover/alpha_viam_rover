# Travel-Mode Backlog (Copy into GitHub Issues)

> Titles + acceptance criteria ready to paste as Issues. Add labels: Phase, Subsystem, Size, Evidence. Link to PRs and MCAPs on completion.

1) Encoders → odometry via `ros2_control`
- AC: `/joint_states` ≥50 Hz; `/odom` stable; diff_drive params in `configs/diff_drive.yaml`; wheels spin symmetrically off-ground; watchdog halts on stale `cmd_vel`.
- Evidence: Short MCAP, Foxglove screenshot of TF/odom.

2) IMU (MPU‑6050) integration
- AC: `/imu/data` at 100–200 Hz; correct frame `imu_link`; optional `imu_filter_madgwick` node outputs fused orientation.
- Evidence: MCAP with IMU at rest and during small motion.

3) Power sensing (INA219)
- AC: Topic `/power/ina219` publishes voltage/current; units documented in `docs/sensors/imu_power.md`.
- Evidence: MCAP segment showing changes under motor load.

4) YDLIDAR G4 bring-up
- AC: `/scan` at expected Hz; frame `lidar_link`; ranges match tape measure; config at `configs/ydlidar_g4.yaml`.
- Evidence: Foxglove LiDAR panel screenshot + MCAP.

5) Foxglove bridge service
- AC: `foxglove_bridge` systemd unit running on rover with port from `configs/network.yaml`.
- Evidence: External connection from Mac; panels populated using `configs/foxglove/default_layout.json`.

6) MCAP recording policy + script
- AC: `scripts/record_mcap.sh` records to `bags/samples/` with MCAP; naming policy documented in `docs/data/mcap.md`.
- Evidence: Sample bag committed (or LFS) with metadata.

7) EKF configuration (robot_localization)
- AC: `configs/ekf.yaml` fuses wheel odom + IMU; publishes `odom` frame; REP‑105 compliant.
- Evidence: MCAP showing stable `odom`; comparison vs raw integration.

8) SLAM Toolbox starter
- AC: Launch starts slam_toolbox with LiDAR; produces `map` and `map->odom` TF; params in `configs/slam_toolbox.yaml`.
- Evidence: Small loop map artifact + MCAP.

9) Nav2 starter
- AC: Launch Nav2 with basic costmaps; RViz/Foxglove shows planners; successful short waypoint.
- Evidence: MCAP + screenshot; config under `configs/nav2/`.

10) URDF → add ros2_control tags
- AC: Wheel joints annotated for diff drive; transmissions defined; matches `diff_drive_controller` names.
- Evidence: Controller starts without errors; `/joint_states` present.

11) Ansible: ROS 2 install task
- AC: Role `ros` installs Humble; `ros2` CLI available; workspace layout under `/opt/ros/humble` and user overlay.
- Evidence: Ansible run log snippet; `ros2 doctor` clean (later on device).

12) Udev rules for USB devices
- AC: Predictable symlinks for LiDAR (e.g., `/dev/ydlidar`); permissions set for user access.
- Evidence: `udevadm info` snippet; working port in driver config.

13) DDS networking template
- AC: `configs/dds/example_profile.xml` reviewed; `configs/network.yaml` parameters wired; doc `docs/mrdds/networking.md` notes DOMAIN_ID policy.
- Evidence: Two-device discovery test later; placeholder doc updated.

14) Teleop + viz launch
- AC: `launch/teleop_viz.launch.xml` starts foxglove_bridge and any diagnostics; parameterized ports/frames.
- Evidence: Launch output; Foxglove connection screenshot.

15) Tests: config schema validation (CI required)
- AC: Extend `tests/config/test_yaml_schemas.py` to cover all configs; CI makes `pytest` required (remove `|| true`).
- Evidence: Green CI on PR with tests; failure case demonstration.

