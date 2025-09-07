# Planned TF Tree

Frames and ownership for the Viam Rover v2 stack.

- map: produced by `slam_toolbox` when mapping (or static when loading a saved map)
- odom: produced by `robot_localization` EKF (wheel odom + IMU)
- base_link: robot base frame at chassis rotational center
  - left_wheel_link: attached via `left_wheel_joint` (continuous)
  - right_wheel_link: attached via `right_wheel_joint` (continuous)
  - imu_link: fixed transform to base (IMU placement)
  - lidar_link: fixed transform to base (LiDAR mount)

Conventions

- REP-105/103 frames/units (SI). `odom -> base_link` is continuous; `map -> odom` may jump on localization updates.
- Sensor extrinsics (imu_link, lidar_link) will be measured and updated in URDF once hardware positions are finalized.

Ownership

- `map`: slam_toolbox
- `odom`: robot_localization EKF
- `base_link`: URDF
- wheel joints: `ros2_control` hardware + `diff_drive_controller`
- sensor links: URDF + static transform publishers if needed during bring-up

