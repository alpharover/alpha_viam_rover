# Viam Rover v2 — ROS 2 Humble Integration Roadmap

This document is the authoritative roadmap for integrating a Viam Rover v2 chassis with ROS 2 Humble on a Raspberry Pi 4 (Ubuntu 22.04 Server). It is designed so that any experienced developer can jump in and work phase‑by‑phase. Each phase includes clear acceptance tests and documentation artifacts to produce before proceeding.

---

## 1) Hardware & Interfaces

### 1.1 Bill of Materials (current)

* Compute: Raspberry Pi 4, Ubuntu Server 22.04 (ROS 2 Humble target) ([ROS Documentation][1])
* Chassis: Viam Rover v2 kit
* IMU: MPU‑6050 (I²C) — default address 0x68, AD0=1→0x69 ([TDK InvenSense][2], [Adafruit Learning System][3])
* Current/Power: INA219 (I²C) — supported by Linux hwmon `ina2xx` driver ([Kernel Documentation][4])
* Motor Driver: L298N dual H‑bridge (DC motors) — note significant saturation/voltage drop at higher currents ([STMicroelectronics][5])
* Encoders: Left & Right wheel encoders (GPIOs TBD)
* 2D LiDAR: YDLIDAR G4 via USB (serial) — supported by official `ydlidar_ros2_driver` ([GitHub][6], [Génération Robots][7])
* Future: USB camera(s), 2D lidar (present), depth camera, MOSFET‑driven nav lights, Waveshare CAN HAT (MCP2515/SN65HVD230) or transceiver, Foxglove for teleop/vis, Jetson Orin Nano expansion.

### 1.2 GPIO / Bus Map (current)

| Subsystem         |           Board Pin(s) |                    Pi GPIO | Notes                                                                                 |
| ----------------- | ---------------------: | -------------------------: | ------------------------------------------------------------------------------------- |
| I²C bus (shared)  | SDA1 pin 3, SCL1 pin 5 | GPIO 2 (SDA), GPIO 3 (SCL) | System I²C bus; MPU‑6050 + INA219 share; enable I²C on Pi 4. ([ROS Documentation][8]) |
| L298N ENA         |                      — |                    GPIO 26 | PWM recommended                                                                       |
| L298N IN1         |                      — |                    GPIO 19 | Direction A                                                                           |
| L298N IN2         |                      — |                    GPIO 13 | Direction A                                                                           |
| L298N IN3         |                      — |                     GPIO 6 | Direction B                                                                           |
| L298N IN4         |                      — |                     GPIO 5 | Direction B                                                                           |
| L298N ENB         |                      — |                    GPIO 22 | PWM recommended                                                                       |
| Encoder Left A/B  |                      — |                        TBD | Choose interrupt‑friendly GPIOs                                                       |
| Encoder Right A/B |                      — |                        TBD | Choose interrupt‑friendly GPIOs                                                       |

Notes:

* Prefer PWM‑capable pins (GPIO 12/13/18/19) for hardware PWM; pigpio can generate hardware‑timed PWM and waveforms with DMA assistance. ([Raspberry Pi Stack Exchange][9], [Abyz.me.uk][10])
* Use pigpio’s glitch/noise filters on encoder lines for deglitching. ([Abyz.me.uk][11])

---

## 2) System Architecture (overview)

* OS & ROS: Ubuntu 22.04 (server), ROS 2 Humble via debs. ([ROS Documentation][1])
* Middleware: DDS (default RMW), configurable for multi‑robot (Fast DDS discovery server or Cyclone DDS XML profiles). ([answers.ros.org][12], [Fast DDS Documentation][13])
* Control: `ros2_control` with `diff_drive_controller`; a custom hardware interface provides PWM+GPIO for the L298N and encoder feedback. ([ROS Control][14], [GitHub][15])
* Sensors:

  * IMU (`mpu6050` ROS 2 driver) → `sensor_msgs/Imu` → optional `imu_filter_madgwick` for fused orientation. ([GitHub][16], [ROS Documentation][17])
  * Power (`ina2xx` kernel hwmon) → ROS node publishes `sensor_msgs/BatteryState` (or use `battery_state_broadcaster` with a battery sensor component). ([Kernel Documentation][4], [ROS Documentation][18])
  * LiDAR (`ydlidar_ros2_driver`) → `sensor_msgs/LaserScan`. ([GitHub][6])
  * Cameras: `v4l2_camera` for UVC devices; depth camera specific driver later. ([ROS Documentation][19])
* Localization & Nav: `robot_localization` (EKF/UKF), `slam_toolbox` for mapping, Nav2 for planning/control. ([ROS Documentation][20], [Foxglove Docs][21])
* Telemetry & Teleop: Foxglove via `foxglove_bridge` (C++ WS bridge), rosbag2 with MCAP storage plugin. ([Foxglove Docs][22], [ROS Documentation][23])

Frame conventions follow REP‑105/REP‑103 (`map`, `odom`, `base_link`, SI units). ([ROS][24])

---

## 3) Phase Plan (with acceptance tests)

> Build sequentially. Do not advance unless acceptance tests pass. All phases require adding artifacts under the documentation plan (see Section 4).

### Phase 0 — OS & ROS 2 baseline

**Objectives**

* Ubuntu 22.04 Server on Pi 4 set up with hostname `alpha-viam`, SSH enabled, Wi‑Fi configured.
* ROS 2 Humble installed from debs; locale, repo, environment sourcing configured. ([ROS Documentation][1])

**Acceptance tests**

* `ros2` CLI available; demo nodes communicate locally (talker/listener per docs). ([ROS Documentation][1])
* Time sync working (NTP/chrony); correct timezone.

**Artifacts**

* `docs/bringup/os_ros.md` (steps, decisions, versions, hostname/networking).
* `ansible/` or `scripts/` bootstrap (idempotent).

---

### Phase 1 — GPIO & Buses bring‑up

**Objectives**

* Enable I²C bus on GPIO 2/3; confirm devices: MPU‑6050 (0x68/0x69) and INA219 present. ([ROS Documentation][8], [TDK InvenSense][2])
* Install and enable pigpio daemon for hardware‑timed PWM and edge callbacks. ([Abyz.me.uk][10])
* Wire L298N as per table; verify correct motor polarity mechanically (without ROS).

**Acceptance tests**

* I²C scan shows expected addresses.
* pigpio daemon is active; test toggling GPIOs (LED or logic probe).
* Power subsystem safe: motors disabled on boot; emergency stop plan defined.

**Artifacts**

* `docs/bringup/gpio_i2c.md` (pinout, photos, I²C addresses used).
* `hw/pinmap.yaml` (authoritative pin assignment).

---

### Phase 2 — IMU & Power telemetry

**Objectives**

* IMU: choose a ROS 2 MPU‑6050 driver (e.g., `kimsniper/ros2_mpu6050` or similar), publishing `sensor_msgs/Imu`. Pair with `imu_filter_madgwick` (or `imu_complementary_filter`) to produce orientation (`imu/data`). ([GitHub][16], [ROS Documentation][17], [ROS Index][25])
* Power: use the Linux `ina2xx` hwmon driver for INA219 and a small ROS node to publish `sensor_msgs/BatteryState` (or integrate via `battery_state_broadcaster` if modeled as a `ros2_control` sensor). ([Kernel Documentation][4], [ROS Documentation][18])

**Acceptance tests**

* IMU topics stream with plausible accel/gyro; fused orientation stable when stationary.
* Battery/voltage/current telemetry updates at target rate; values match multimeter within tolerance.

**Artifacts**

* `configs/imu.yaml` (driver + filter params, frame IDs).
* `configs/power.yaml` (scaling, shunt value, topic names).
* `docs/sensors/imu_power.md` (calibration notes, noise characteristics).

---

### Phase 3 — Motor control & encoder odometry

**Objectives**

* Implement a `ros2_control` hardware interface for the rover base (L298N). Command interfaces: left/right wheel velocity; state interfaces: wheel position/velocity. Use pigpio for PWM on ENA/ENB and GPIO for IN1..IN4; observe L298N current/voltage drop limitations from datasheet (plan for upgrade later). ([GitHub][15], [ROS Control][14], [STMicroelectronics][5])
* Encoders: use pigpio callbacks on two channels per wheel; apply `set_glitch_filter` to reduce spurious edges; compute ticks, velocity. ([Abyz.me.uk][11])
* Configure `diff_drive_controller` with wheel radius, separation, publish rate, and TF (`odom`→`base_link`). ([ROS Control][14])

**Acceptance tests**

* With wheels off ground, commanded `cmd_vel` results in symmetric wheel velocities; zero command yields stop.
* On ground, straight drive test over known distance yields odometry within agreed tolerance; in‑place rotation \~360° within tolerance.
* TF tree: `odom`→`base_link` present and continuous; no controller timeouts.
* Safety: watchdog halts motors on stale `cmd_vel`.

**Artifacts**

* `urdf/rover.urdf.xacro` with wheels/joints and `ros2_control` tags.
* `configs/diff_drive.yaml` (controller params).
* `docs/control/ros2_control_hw.md` (hardware interface design, pin mapping, PWM frequency, saturation handling).

---

### Phase 4 — LiDAR integration (YDLIDAR G4)

**Objectives**

* Bring up `ydlidar_ros2_driver` with correct serial port and rotation speed; publish `sensor_msgs/LaserScan`. Confirm G4 range/frequency parameters. ([GitHub][6], [Génération Robots][7])

**Acceptance tests**

* LiDAR topic at expected Hz; RViz shows stable 2D scan; ranges match tape‑measured objects.

**Artifacts**

* `configs/ydlidar_g4.yaml` (port, frame, freq).
* `docs/sensors/lidar.md` (mounting, FOV, blind spots).

---

### Phase 5 — Teleoperation & visualization

**Objectives**

* Install and launch `foxglove_bridge` on the rover; connect from MacBook Foxglove via WebSocket. Use Foxglove Panels for IMU, LaserScan, Odometry, TF. ([ROS Documentation][26], [Foxglove Docs][22])
* Optional: set rosbag2 storage to MCAP for efficient logging and cross‑platform playback. ([ROS Documentation][23])

**Acceptance tests**

* Foxglove connects reliably over LAN; live plots of velocity/IMU; start/stop MCAP recordings; playback inspects topics.

**Artifacts**

* `launch/teleop_viz.launch.xml` (bridge + useful defaults).
* `docs/tools/foxglove.md` (ports, security, saved layouts).
* `docs/data/mcap.md` (recording policies, retention). ([MCAP][27])

---

### Phase 6 — Cameras

**Objectives**

* USB camera via `v4l2_camera` (or device‑specific driver) producing `sensor_msgs/Image` and `CameraInfo`. ([ROS Documentation][19])
* Depth camera in a later subphase with vendor ROS 2 driver.

**Acceptance tests**

* RGB stream visible in Foxglove/RViz; timestamps synchronized; exposure acceptable.

**Artifacts**

* `configs/camera.yaml` (format, resolution, fps).
* `docs/sensors/camera.md` (lens, mounting, extrinsics vs `base_link`).

---

### Phase 7 — Mapping & Navigation

**Objectives**

* `robot_localization` EKF fusing wheel odom + IMU; publish `odom` frame. Follow REP‑105/REP‑103 for frames/units. ([ROS Documentation][20], [ROS][24])
* `slam_toolbox` online mapping to produce `map` and `map→odom` TF; later switch to saved maps. ([Foxglove Docs][21])
* Integrate Nav2 for local planners, costmaps using LiDAR and optionally depth data.

**Acceptance tests**

* Stable EKF odom with low drift; loop‑closure map with `slam_toolbox` in a small lab yields consistent occupancy grid; Nav2 can execute a simple waypoint.

**Artifacts**

* `configs/ekf.yaml`, `configs/slam_toolbox.yaml`, `configs/nav2/`.
* `docs/nav/frames.md` (TF tree, static transforms, sensor frames). ([ROS Documentation][28])

---

### Phase 8 — Lighting & GPIO peripherals

**Objectives**

* MOSFET‑driven navigation lights: create a small ROS 2 node exposing services/topics to control light patterns; ensure pigpio doesn’t conflict with motor control PWM.

**Acceptance tests**

* Commands toggle and pattern lights; no PWM jitter on motors.

**Artifacts**

* `docs/hw/lights.md` (wiring, current budget).
* `configs/lights.yaml`.

---

### Phase 9 — CAN expansion (servos via Waveshare MCP2515)

**Objectives**

* Add a Waveshare CAN HAT/transceiver (MCP2515 + SN65HVD230) and enable SocketCAN (`can0`) via device tree overlay (SPI enabled; oscillator and interrupt set for the specific HAT). ([PragmaticLinux][29], [harrisonsand.com][30])
* Bridge CAN to ROS via a SocketCAN bridge (select a ROS 2 package that publishes frames and/or device‑specific drivers for servos).

**Acceptance tests**

* `can0` up with expected bit‑rate; loopback test passes; commanded servo frames elicit expected motion on the bus.

**Artifacts**

* `docs/buses/can.md` (overlay params used, wiring, bit‑rates, termination).
* `configs/can.yaml` (bit‑rate, filters).

---

### Phase 10 — Compute expansion (Jetson Orin Nano)

**Objectives**

* Add Jetson Orin Nano with JetPack 6 (Ubuntu 22.04 base) to offload perception; install ROS 2 Humble natively and consider NVIDIA Isaac ROS GEMs for accelerated pipelines. ([NVIDIA Developer Forums][31], [NVIDIA Isaac ROS][32])
* Exchange data with the Pi over ROS 2 DDS; optionally run Nav2/SLAM on Jetson.

**Acceptance tests**

* DDS discovery between Pi and Jetson is reliable; camera/depth pipelines run at required FPS on Jetson; latency budget met.

**Artifacts**

* `docs/compute/jetson.md` (JetPack, drivers, accelerated nodes, QoS).

---

### Phase 11 — Multi‑robot (“twin” rover) comms

**Objectives**

* DDS configuration for multiple robots: per‑robot ROS 2 namespaces and DOMAIN\_IDs; optionally use Fast DDS Discovery Server or Cyclone DDS profile to constrain traffic. ([answers.ros.org][12], [Fast DDS Documentation][13])
* Define cross‑robot topics/services for collaboration (e.g., map sharing, task allocation).

**Acceptance tests**

* Two rovers operate concurrently without topic crosstalk; cross‑robot messages only on intended channels; Wi‑Fi bandwidth and CPU stay within budget under logging.

**Artifacts**

* `docs/mrdds/networking.md` (RMW, discovery topology, Wi‑Fi QoS).
* `configs/dds/*.xml` (RMW vendor configs).

---

## 4) Documentation & Repo Structure

Keep human‑readable docs and machine‑readable configs close to code. Suggested layout:

* `docs/`

  * `bringup/` (OS/ROS, GPIO/I²C)
  * `sensors/` (imu\_power, lidar, camera)
  * `control/` (ros2\_control\_hw, safety)
  * `nav/` (frames, SLAM, Nav2)
  * `tools/` (foxglove, bagging/MCAP)
  * `buses/` (CAN)
  * `compute/` (jetson)
  * `ADR/` Architecture Decision Records (small numbered markdowns)
* `configs/`

  * YAML for controllers, sensors, EKF, SLAM, Nav2, Foxglove, bagging (MCAP), DDS XML
* `urdf/` robot description
* `launch/` launch files
* `hw/` schematics, pinmap.yaml, photos
* `scripts/` or `ansible/` provisioning
* `calibration/` per‑sensor calibration dumps (IMU bias, camera intrinsics, LiDAR extrinsics)
* `bags/` example MCAP logs (short, anonymized) and `.readme` with context

Add a small “handoff” template (`docs/handoff.md`) capturing:

* What changed, why (link ADR), how to test, expected outputs, gotchas.

---

## 5) Conventions & Quality Gates

* Frames & units: Follow REP‑105 and REP‑103; `base_link` origin at chassis rotational center; SI units throughout. ([ROS][24])
* Topics: Use standard messages (`Imu`, `LaserScan`, `BatteryState`, `Odometry`). ([ROS Documentation][33], [docs.ros2.org][34])
* Logging: Use rosbag2 MCAP as default storage (fast, self‑describing; compressible). ([ROS Documentation][23], [GitHub][35])
* Visualization: Prefer Foxglove via `foxglove_bridge` for unified dashboards. ([Foxglove Docs][22])
* Safety: Controller watchdogs; motors disabled on boot; E‑stop documented.
* CI smoke: Lint, URDF check, static config validation; optional sim bring‑up using ros2\_control demos as reference patterns. ([GitHub][15])

**Phase Gate (global):** Do not advance if TF tree is inconsistent, sensors are unstable, or control watchdogs aren’t in place.

---

## 6) Packages (recommended)

* System: ROS 2 Humble via debs on Ubuntu 22.04. ([ROS Documentation][1])
* Control: `ros2_control`, `ros2_controllers` (`diff_drive_controller`). ([ROS Control][14])
* Hardware layer: custom hardware interface for L298N + pigpio (PWM/dir); pigpio daemon for timing/encoder callbacks. ([Abyz.me.uk][10])
* IMU: `ros2_mpu6050` (community) + `imu_filter_madgwick` or `imu_complementary_filter`. ([GitHub][16], [ROS Documentation][17], [ROS Index][25])
* Power: Linux `ina2xx` + ROS publisher (or `battery_state_broadcaster` with sensor component). ([Kernel Documentation][4], [ROS Documentation][18])
* LiDAR: `ydlidar_ros2_driver`. ([GitHub][6])
* Cameras: `v4l2_camera`. ([ROS Documentation][19])
* Localization: `robot_localization`. ([ROS Documentation][20])
* SLAM: `slam_toolbox`. ([Foxglove Docs][21])
* Telemetry: `foxglove_bridge`. ([ROS Documentation][26])
* Recording: `rosbag2_storage_mcap`. ([ROS Documentation][23])
* Multi‑robot networking: Configure DDS (Fast DDS discovery server or Cyclone DDS profiles). ([answers.ros.org][12], [Fast DDS Documentation][13])
* CAN: SocketCAN (`can0`) via MCP2515 overlay; ROS 2 bridge of choice for CAN frames. ([PragmaticLinux][29])

---

## 7) Calibration & Tuning Notes

* IMU: Record stationary bias; tune `imu_filter_madgwick` gains; document orientation of the module relative to `base_link`. ([ROS Documentation][17])
* Encoders: Determine ticks‑per‑revolution accurately; verify sign conventions (forward positive).
* Diff drive: Measure wheel radius & separation (axle width); configure `diff_drive_controller` limits, covariance, and TF publication. ([ROS Control][14])
* LiDAR: Verify serial baud and rotation frequency; mount rigidly and document transform to `base_link`. ([GitHub][6])
* Power: Set INA219 shunt value and scaling; publish `BatteryState` voltage/current; optionally estimate SOC with a simple model. ([Kernel Documentation][4])

---

## 8) Risks & Mitigations

* L298N losses: Bipolar H‑bridge has high saturation voltage (expect noticeable voltage drop/heating at higher current). Consider migrating to a MOSFET driver (e.g., TB6612FNG) later. ([STMicroelectronics][5])
* GPIO timing: Use pigpio for hardware‑timed PWM and robust edge capture; set glitch filters on encoder lines. ([Abyz.me.uk][10])
* DDS noise on Wi‑Fi: Apply discovery server or Cyclone DDS whitelisting; use namespaces/Domain IDs per robot. ([answers.ros.org][12], [Fast DDS Documentation][13])
* I²C bus: MPU‑6050 and INA219 share SDA/SCL; confirm addresses (MPU AD0 pin) to avoid conflicts. ([TDK InvenSense][2])
* CAN overlay parameters: Ensure oscillator and interrupt GPIOs match the Waveshare HAT variant. ([harrisonsand.com][30])

---

## 9) Testing (quick matrix)

| Area        | Test                     | Pass criteria                                                              |
| ----------- | ------------------------ | -------------------------------------------------------------------------- |
| Sensors     | IMU bias & stability     | < specified drift at rest; orientation holds when stationary               |
| Power       | Voltage/current accuracy | Within tolerance vs bench meter; consistent update rate                    |
| Control     | Straight‑line & rotation | Odometry error within tolerance over distance; 360° spin < threshold error |
| Safety      | Watchdog & E‑stop        | Motors stop on stale `cmd_vel`; explicit E‑stop verified                   |
| LiDAR       | Range validation         | Measured obstacle distances match LaserScan                                |
| Telemetry   | Foxglove & MCAP          | Stable WS connection; record & replay MCAP                                 |
| Multi‑robot | Traffic isolation        | No crosstalk; discovery and QoS tuned for Wi‑Fi                            |

---

## 10) How to Work This Roadmap (for new contributors)

1. Pick the next open item in the current phase.
2. Branch: `feature/phaseX-itemY-shortdesc`.
3. Implement; keep configs in `configs/`; add a short ADR for any design decision.
4. Update docs; include photos/plots; attach a short MCAP bag if relevant.
5. Verify acceptance tests; open PR with test evidence (screenshots, Foxglove layout export, bag snippet).
6. Reviewer checks against this roadmap and phase gate.

---

### Appendix: Reference Links

* ROS 2 Humble install on Ubuntu 22.04 (official). ([ROS Documentation][1])
* REP‑105 Coordinate Frames, REP‑103 Units & conventions. ([ROS][24])
* `ros2_control` and `diff_drive_controller` docs & demos. ([ROS Control][14], [GitHub][15])
* pigpio library (hardware‑timed PWM, callbacks, glitch filter). ([Abyz.me.uk][10])
* L298N datasheet (saturation voltage/efficiency concerns). ([STMicroelectronics][5])
* MPU‑6050 address via AD0, pinouts. ([TDK InvenSense][2], [Adafruit Learning System][3])
* INA219 Linux `ina2xx` driver (hwmon). ([Kernel Documentation][4])
* YDLIDAR G4 & ROS 2 driver. ([Génération Robots][7], [GitHub][6])
* `robot_localization`. ([ROS Documentation][20])
* `slam_toolbox`. ([Foxglove Docs][21])
* Foxglove bridge & ROS 2 docs. ([ROS Documentation][26], [Foxglove Docs][22])
* rosbag2 MCAP storage plugin. ([ROS Documentation][23])
* SocketCAN on Raspberry Pi (MCP2515 overlay guidance). ([PragmaticLinux][29])
* Multi‑robot DDS: Fast DDS discovery server; Cyclone DDS profiles. ([answers.ros.org][12], [Fast DDS Documentation][13])
* Jetson Orin Nano + Isaac ROS overview. ([NVIDIA Developer Forums][31], [NVIDIA Isaac ROS][32])

---

This roadmap is meant to be living: update ADRs and configs as you discover better parameters, but keep the acceptance tests strict. When you’re ready, start at Phase 0 or pick up the next open item in the active phase and push the rover forward.

[1]: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html?utm_source=chatgpt.com "Ubuntu (deb packages) — ROS 2 Documentation"
[2]: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf?utm_source=chatgpt.com "MPU-6000 and MPU-6050 Product Specification Revision ..."
[3]: https://learn.adafruit.com/mpu6050-6-dof-accelerometer-and-gyro/pinouts?utm_source=chatgpt.com "Pinouts | MPU6050 6-DoF Accelerometer and Gyro"
[4]: https://docs.kernel.org/hwmon/ina2xx.html?utm_source=chatgpt.com "Kernel driver ina2xx"
[5]: https://www.st.com/resource/en/datasheet/l298.pdf?utm_source=chatgpt.com "Datasheet - L298 - Dual full-bridge driver"
[6]: https://github.com/YDLIDAR/ydlidar_ros2_driver?utm_source=chatgpt.com "ydlidar driver package under ros2"
[7]: https://static.generation-robots.com/media/ydlidar-g4-user-manual.pdf?utm_source=chatgpt.com "YDLIDAR G4 Lidar User Manual"
[8]: https://docs.ros.org/en/humble/p/v4l2_camera/?utm_source=chatgpt.com "v4l2_camera 0.6.2 documentation"
[9]: https://raspberrypi.stackexchange.com/questions/138850/using-hardware-pwm-in-raspberry-pi-4?utm_source=chatgpt.com "Using Hardware PWM in Raspberry Pi 4"
[10]: https://abyz.me.uk/rpi/pigpio/?utm_source=chatgpt.com "pigpio library"
[11]: https://abyz.me.uk/rpi/pigpio/pdif2.html?utm_source=chatgpt.com "pigpiod C Interface - pigpio library"
[12]: https://answers.ros.org/question/415693?utm_source=chatgpt.com "How to change Fast-DDS XML configuration ..."
[13]: https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/discovery_server/ros2_discovery_server.html?utm_source=chatgpt.com "16.2. Use ROS 2 with Fast-DDS Discovery Server - 3.3.0"
[14]: https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html?utm_source=chatgpt.com "diff_drive_controller — ROS2_Control: Humble Sep 2025 ..."
[15]: https://github.com/ros-controls/ros2_control_demos?utm_source=chatgpt.com "ros-controls/ros2_control_demos - GitHub"
[16]: https://github.com/kimsniper/ros2_mpu6050?utm_source=chatgpt.com "kimsniper/ros2_mpu6050: This repository contains the MPU6050 C++ package dedicated for ROS2."
[17]: https://docs.ros.org/en/humble/p/imu_filter_madgwick/?utm_source=chatgpt.com "imu_filter_madgwick: Humble 2.1.5 documentation"
[18]: https://docs.ros.org/en/rolling/p/battery_state_broadcaster/?utm_source=chatgpt.com "battery_state_broadcaster: Rolling 1.0.2 documentation"
[19]: https://docs.ros.org/en/foxy/Tutorials/Advanced/Discovery-Server/Discovery-Server.html?utm_source=chatgpt.com "Using Fast DDS Discovery Server as ..."
[20]: https://docs.ros.org/en/melodic/api/robot_localization/html/index.html?utm_source=chatgpt.com "robot_localization wiki"
[21]: https://docs.foxglove.dev/docs/getting-started/frameworks/ros2?utm_source=chatgpt.com "ROS 2 | Foxglove Docs"
[22]: https://docs.foxglove.dev/docs/visualization/ros-foxglove-bridge?utm_source=chatgpt.com "ROS Foxglove bridge"
[23]: https://docs.ros.org/en/humble/p/rosbag2_storage_mcap/?utm_source=chatgpt.com "rosbag2_storage_mcap: Humble 0.15.15 documentation"
[24]: https://www.ros.org/reps/rep-0105.html?utm_source=chatgpt.com "REP 105 -- Coordinate Frames for Mobile Platforms ..."
[25]: https://index.ros.org/p/imu_complementary_filter/?utm_source=chatgpt.com "ROS Package: imu_complementary_filter"
[26]: https://docs.ros.org/en/rolling/Related-Projects/Visualizing-ROS-2-Data-With-Foxglove.html?utm_source=chatgpt.com "Visualizing ROS 2 data with Foxglove"
[27]: https://mcap.dev/guides/getting-started/ros-2?utm_source=chatgpt.com "ROS 2"
[28]: https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html?utm_source=chatgpt.com "Writing a static broadcaster (Python) — ROS 2 Documentation"
[29]: https://www.pragmaticlinux.com/2021/10/can-communication-on-the-raspberry-pi-with-socketcan/?utm_source=chatgpt.com "CAN communication on the Raspberry PI with SocketCAN"
[30]: https://harrisonsand.com/posts/can-on-the-raspberry-pi/?utm_source=chatgpt.com "CAN on the Raspberry Pi - Harrison's Sandbox"
[31]: https://forums.developer.nvidia.com/t/title-beginner-seeking-setup-help-for-ros2-on-jetson-orin-nano-and-onboard-simulation-options/313529?utm_source=chatgpt.com "Beginner Seeking Setup Help for ROS2 on Jetson Orin ..."
[32]: https://nvidia-isaac-ros.github.io/?utm_source=chatgpt.com "NVIDIA Isaac ROS — isaac_ros_docs documentation"
[33]: https://docs.ros.org/en/humble/p/sensor_msgs/?utm_source=chatgpt.com "sensor_msgs: Humble 4.9.0 documentation"
[34]: https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html?utm_source=chatgpt.com "nav_msgs/msg/Odometry Message"
[35]: https://github.com/ros2/rosbag2?utm_source=chatgpt.com "ros2/rosbag2"
