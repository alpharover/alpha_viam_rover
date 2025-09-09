Scope
OS/ROS bring‑up, launch composition, and robot “health” checks.

You will

* Verify Ubuntu 22.04 + ROS 2 Humble environment; `rosdep`, `colcon`, `ros2` cli available.
* Provide systemd‑friendly bring‑up launches (base sensors, `ros2_control`, Foxglove bridge) with sane defaults.

Acceptance (evidence required)

* `ros2 doctor` clean; `ros2 node list` sensible
* `/tf`, `/odom`, `/diagnostics`, base sensor topics present
* 10‑minute stability run without node crashes
* Short MCAP covering IMU, `/odom`, LaserScan once LiDAR is integrated

Progress
Log completion in `AGENTS_PROGRESS.md` with MCAP link.

Utilities

- `scripts/launch_with_timeout.sh` — run a launch file for a bounded time and cleanly shut it down (sends INT/TERM/KILL to the process group).
- `scripts/ros_clean.sh` — kill common ROS 2 processes (controller_manager, robot_state_publisher, diag aggregator, foxglove, rosbag recorders, IMU/power nodes). Use `--force` to SIGKILL any stubborn remnants.
