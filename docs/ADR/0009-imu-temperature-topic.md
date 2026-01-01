# ADR-0009: Publish IMU Temperature Topic

Status: Proposed (2025-12-31)

Context

The MPU-6050 IMU driver (`drivers/mpu6050_driver`) already reads die temperature during each I2C transaction but discards it. The `_read_raw()` method computes `temp_c = (temp_raw / 340.0) + 36.53` per the datasheet, but `_tick()` ignores this value. Temperature is useful for:

- Thermal monitoring of the IMU (detect overheating)
- Compensating for temperature-dependent sensor drift (future)
- General rover health telemetry in the driver station UI

The standard `sensor_msgs/Imu` message does not include a temperature field, so a separate topic is required.

Decision

Publish MPU-6050 die temperature as `sensor_msgs/msg/Temperature` on `/imu/temperature`.

Parameters (added to the IMU node):
- `publish_temperature` (bool, default: `true`) - enable/disable temperature publishing
- `temperature_topic` (string, default: `/imu/temperature`) - topic name
- `temperature_rate_hz` (float, default: `1.0`) - publish rate (decimated from 100Hz IMU reads)

Message population:
- `header.stamp`: same timestamp as the corresponding IMU sample
- `header.frame_id`: same as IMU (`imu_link`)
- `temperature`: die temperature in Celsius (float64)
- `variance`: `0.0` (unknown/uncalibrated)

Implementation notes:
- Reuse existing I2C read (no additional bus traffic)
- Decimate publishing via monotonic time check in `_tick()`, not a separate timer
- Gate entirely behind `publish_temperature` parameter

Details

Topic naming follows existing pattern (`/imu/data` for IMU, `/imu/temperature` for temperature). Using `sensor_msgs/Temperature` over `std_msgs/Float32` provides:
- Explicit Celsius units per message definition
- Timestamp for correlation with other sensor data
- Frame ID for consistency
- Variance field for future calibration

Default 1Hz rate is sufficient since die temperature changes slowly. This minimizes bandwidth and bag file bloat while still capturing thermal trends.

Consequences

- New topic `/imu/temperature` added to the rover's topic graph
- Requires update to `configs/imu.yaml` for parameter defaults
- Requires extension of `bringup/.../base_bringup.launch.py` allowed keys
- Driver station server/UI will subscribe and display the value
- Backward compatible: no changes to existing `/imu/data` behavior

Alternatives Considered

- `std_msgs/Float32` on `/imu/temperature_c`: simpler but lacks timestamp/frame, inconsistent with sensor message conventions. Would match existing `/power/*` scalar topics but loses ROS tooling benefits.
- Publish via `/diagnostics`: rejected; temperature is useful as real-time telemetry, not just diagnostics.
- Embed in custom IMU message: rejected; breaks compatibility with standard tools and filters.

Verification

- `ros2 topic hz /imu/temperature` shows ~1Hz (configurable)
- `ros2 topic echo --once /imu/temperature` shows `frame_id: imu_link` and plausible Celsius value (~25-45C typical operating range)
- MCAP recording includes both `/imu/data` and `/imu/temperature`
- Driver station UI displays temperature in IMU panel

References

- MPU-6050 datasheet temperature formula: `temp_c = (raw / 340.0) + 36.53`
- `sensor_msgs/msg/Temperature` definition
- Existing driver: `drivers/mpu6050_driver/mpu6050_driver/node.py`
- Config: `configs/imu.yaml`
