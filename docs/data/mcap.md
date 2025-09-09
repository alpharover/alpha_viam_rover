# MCAP Recording

Recording policies, retention, and playback notes for evidence capture.

- Storage: rosbag2 with MCAP storage plugin.
- Location: short evidence clips live under `bags/samples/` and may use Git LFS if large.
- Naming: `YYYYMMDD_HHMM_[env]_[testcase].mcap` (script uses timestamp + `bench`).

Recommended topics (minimum)

- `/tf`, `/tf_static`, `/odom`, `/joint_states`, `/cmd_vel`, `/imu/data`, `/imu/data_fused`, `/power/ina219`, `/scan` (when LiDAR is present).

Quick start

- Use `scripts/record_mcap.sh <seconds>` to capture a short clip with the recommended topics. Output goes to `bags/samples/`.

Retention

- Keep evidence clips short (< 60s) to support code review and PR artifacts.
- For long experiments, store bags externally or attach to a GitHub Release; only keep small, anonymized samples in the repo.

Playback

- `ros2 bag info <file.mcap>` to inspect metadata.
- Use Foxglove to load the bag and validate the TF tree, odometry, and sensor quality.

Example metadata (`ros2 bag info`)

```
Files:             test_20250101_120000_bench.mcap
Bag size:          1.2 MiB
Storage id:        mcap
Duration:          12.5s
Start:             Jan 01 2025 12:00:00
End:               Jan 01 2025 12:00:12
Messages:          6,200
Topic information: Topic: /tf (type: tf2_msgs/msg/TFMessage, count: 250)
                   Topic: /tf_static (type: tf2_msgs/msg/TFMessage, count: 2)
                   Topic: /odom (type: nav_msgs/msg/Odometry, count: 625)
                   Topic: /joint_states (type: sensor_msgs/msg/JointState, count: 625)
                   Topic: /imu/data (type: sensor_msgs/msg/Imu, count: 1,250)
                   Topic: /scan (type: sensor_msgs/msg/LaserScan, count: 1,250)
```

Recent on-device sample

- `bags/samples/20250908_230210_bench` (6.65 s) — contains `/imu/data` at ~100 Hz while base bring-up was running.
