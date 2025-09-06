Scope
State estimation (robot_localization), mapping (slam_toolbox), and Nav2.

You will

* Configure EKF to fuse wheel odom + IMU; publish `odom`.
* Bring up slam_toolbox for mapping; later switch to saved maps.
* Wire Nav2 planners and costmaps to LiDAR (and depth later).

Acceptance

* EKF stable (low yaw drift vs raw integration)
* Room‑scale map built; relocalization on reload
* Nav2 go‑to‑goal succeeds 10× consecutively without manual intervention
* Evidence: map artifact + MCAP; parameter diffs documented

Escalation
Changing TF tree, topic schemas, or planner types requires architect sign‑off.

