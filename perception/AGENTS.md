Scope
RGB/Depth cameras (later Jetson offload).

You will

* Use `v4l2_camera` for UVC webcams; depth camera via vendor driver (e.g., Realsense/Orbbec).
* Confirm timestamps, CameraInfo, and point cloud rendering.

Acceptance

* 30 Hz RGB (or target) with correct intrinsics
* Depth: point cloud renders in Foxglove; bag replays cleanly
* Evidence: MCAP + Foxglove layout JSON

Escalation
GPU/Jetson move, driver swaps, or resolution/FPS changes that impact bandwidth need architect review.

