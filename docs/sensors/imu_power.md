# IMU and Power (INA219)

Bench notes and bring-up details for MPU-6050 (IMU) and INA219 (power).

What’s implemented

- IMU (MPU-6050)
  - Node: `mpu6050_driver` (ament_python)
  - Topic: `/imu/data` (`sensor_msgs/Imu`), 100 Hz default
  - Frame: `imu_link`
  - Orientation: identity quaternion, covariance[0] = -1 (orientation not estimated)
  - Bias: gyro bias measured at startup (`calibrate_gyro`, `calib_samples`)
- Power (INA219)
  - Node: `ina219_monitor`
  - Topics: `/power/bus_voltage` (V), `/power/current` (A), 10 Hz default

Hardware & addresses

- Bus: I²C-1 (GPIO2 SDA, GPIO3 SCL)
- Addresses: MPU‑6050 at `0x68` (AD0=0), INA219 at `0x40`

Run locally

- Bring-up (includes IMU + Power):
  - `ros2 launch alpha_viam_bringup base_bringup.launch.py`
- IMU only (manual):
  - `ros2 run mpu6050_driver mpu6050_node --ros-args -p i2c_bus:=1 -p address:=0x68 -p rate_hz:=100.0 -p frame_id:=imu_link`
- Power only (manual):
  - `ros2 run ina219_monitor ina219_monitor`

Configuration knobs

- `configs/imu.yaml`
  - `imu.frame_id`, `imu.i2c_bus`, `imu.address`, `imu.rate_hz`
  - `imu.accel_range_g` (2/4/8/16), `imu.gyro_range_dps` (250/500/1000/2000)
  - Optional: `filter` block reserved (e.g., `madgwick`) — not used yet
- `configs/power.yaml`
  - `power.shunt_ohms`, topics, and rate

Acceptance checks

- IMU rate: `ros2 topic hz /imu/data` ≈ 100 Hz
- IMU sample: `ros2 topic echo --once /imu/data` shows `frame_id: imu_link` and plausible accel/gyro
- INA219: bus voltage roughly matches battery; current rises under load

Evidence (recent)

- MCAP (on-device): `bags/samples/20250908_230210_bench` — 6.65 s, 554 messages on `/imu/data`

Next steps

- Optional orientation: integrate `imu_filter_madgwick` to fuse accel+gyro and publish orientation
- Calibrate INA219 against a bench meter, document shunt value and scaling
- Record stationary bias and noise plots; document axes relative to `base_link`
