Scope
Low‑level I/O: IMU (MPU‑6050), INA219 power, wheel encoders, L298N drive via `ros2_control`.

Hardware map (RPi4)

* I²C: SDA=GPIO2, SCL=GPIO3 → MPU‑6050 (0x68/0x69), INA219
* Motors (L298N): ENA=GPIO26 (PWM), IN1=GPIO19, IN2=GPIO13, IN3=GPIO6, IN4=GPIO5, ENB=GPIO22 (PWM)
* Encoders (single-channel): Left=GPIO20 (P38), Right=GPIO21 (P40) (use glitch filtering)

Packages & patterns

* IMU: community `ros2_mpu6050` (or equivalent) → `sensor_msgs/Imu`; fuse with `imu_filter_madgwick`.
* Power: Kernel `ina2xx` (INA219) via hwmon + small ROS publisher → `sensor_msgs/BatteryState`.
* Drive: `ros2_control` custom HardwareInterface for L298N + `diff_drive_controller`; pigpio for PWM + encoder callbacks.
* Conventions: REP‑103/105 units/frames; publish `odom → base_link`. (Nearest AGENTS.md governs details; root defines global rules.)

Acceptance (bench first, then floor)

* IMU: `/imu/data` 100–200 Hz; fused orientation stable at rest
* Power: Voltage/current plausible; current rises under motor load
* Drive: `/cmd_vel` → symmetric wheel response off the ground; on‑floor straight‑line and 360° spin within tolerance; watchdog halts on stale commands
* Encoders: clean ticks, monotonic position; no missed edges during a slow–fast sweep
* Evidence: Short MCAP + Foxglove screenshot of TF/odom/IMU

Safety
Do not raise PWM ranges, invert polarity, or change pin maps without architect approval (label `needs-architect`). Enforced by CODEOWNERS.

