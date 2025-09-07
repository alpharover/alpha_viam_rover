# MCAP Recording

Recording policies, retention, and playback notes for evidence capture.

- Storage: rosbag2 with MCAP storage plugin.
- Location: short evidence clips live under `bags/samples/` and may use Git LFS if large.
- Naming: `YYYYMMDD_HHMM_[env]_[testcase].mcap` (script uses timestamp + `bench`).

Recommended topics (minimum)

- `/tf`, `/tf_static`, `/odom`, `/joint_states`, `/cmd_vel`, `/imu/data`, `/power/ina219`, `/scan` (when LiDAR is present).

Quick start

- Use `scripts/record_mcap.sh <seconds>` to capture a short clip with the recommended topics. Output goes to `bags/samples/`.

Retention

- Keep evidence clips short (< 60s) to support code review and PR artifacts.
- For long experiments, store bags externally or attach to a GitHub Release; only keep small, anonymized samples in the repo.

Playback

- `ros2 bag info <file.mcap>` to inspect metadata.
- Use Foxglove to load the bag and validate the TF tree, odometry, and sensor quality.
