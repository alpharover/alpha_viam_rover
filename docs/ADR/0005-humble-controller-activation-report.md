# ADR-0005: Humble — Controller Params via Spawner Param-File

Status: Superseded (see ADR-0006)

Date: 2025-09-10 (revised per architect guidance; superseded 2025-12-26)

## Decision

On ROS 2 Humble, we will pass `diff_drive_controller` parameters using the supported spawner mechanism with `--param-file`, and we will remove any controller-manager-side `params_file`. The controller parameter file is controller-scoped (root key `diff_drive_controller:`) and installed into the bringup package share for deterministic pathing.

## Update (2025-12-26)

On the alpha-viam rover Humble image, the diff-drive controller is namespaced under `/controller_manager/*` and controller-scoped YAML via spawner `--param-file` did not apply as expected. The working solution is captured in ADR-0006 (set `/controller_manager` param `diff_drive_controller.params_file` to the wildcard YAML `configs/diff_drive_params.yaml`, then spawn without `--param-file`).

## TL;DR — Required changes

1) `configs/controllers.yaml` lists only controller types (no `params_file`).

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50
    use_sim_time: false

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController
```

2) Controller-scoped param file `configs/diff_drive.params.yaml`:

```yaml
diff_drive_controller:
  ros__parameters:
    left_wheel_names:  [left_wheel_joint]
    right_wheel_names: [right_wheel_joint]
    wheel_separation:  0.30
    wheel_radius:      0.06
    wheels_per_side:   1
    use_stamped_vel:   false
    cmd_vel_timeout:   0.5
    publish_rate:      50.0
    enable_odom_tf:    true
    odom_frame_id:     odom
    base_frame_id:     base_link
    open_loop:         true
```

3) Launch: spawn JSB, then diff drive with `--param-file` and `--unload-on-kill`:

```bash
ros2 run controller_manager spawner joint_state_broadcaster \
  --controller-manager /controller_manager

ros2 run controller_manager spawner diff_drive_controller \
  --controller-manager /controller_manager \
  --activate --unload-on-kill \
  --param-file $(ros2 pkg prefix alpha_viam_bringup)/share/alpha_viam_bringup/configs/diff_drive.params.yaml
```

## Why this fixes the failure

`diff_drive_controller` rejects empty `left_wheel_names/right_wheel_names` during initialization. The Humble spawner loads the controller’s param file into the node before `configure`, satisfying the controller’s requirement when it transitions. Using a controller-scoped root avoids wildcard fragility on Humble.

## Sanity checks

- Controller node parameters present:
  - `ros2 param get /diff_drive_controller left_wheel_names`
  - `ros2 param get /diff_drive_controller right_wheel_names`
- Active controller with velocity interfaces claimed:
  - `ros2 control list_controllers`
  - `ros2 control list_hardware_interfaces`
- Input topic with `use_stamped_vel=false` is `/diff_drive_controller/cmd_vel_unstamped`.

## What to remove/avoid

- No `controller_manager.<ctrl>.params_file` in Humble.
- No wildcard YAML root (`/**:`) for controller params; use controller name root.
- Avoid ad‑hoc env injection; rely on a sourced overlay and package share paths.

## Validation sequence (off-ground)

```bash
# Clean + pigpio
scripts/ros_clean.sh --force
sudo systemctl start pigpiod || sudo /usr/local/bin/pigpiod

# Launch manager
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch alpha_viam_bringup drive_min.launch.py

# Verify params & controllers
timeout 4s ros2 param get /diff_drive_controller left_wheel_names
timeout 4s ros2 param get /diff_drive_controller right_wheel_names
ros2 control list_controllers
ros2 control list_hardware_interfaces

# Move (unstamped)
timeout 2s ros2 topic pub -r 15 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}"
sleep 0.5
timeout 1s ros2 topic pub --once /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Acceptance checklist

- [x] `controllers.yaml` lists controller types only.
- [x] `diff_drive.params.yaml` is controller-scoped and installed into package share.
- [x] JSB spawns; diff drive spawns with `--param-file` and unloads on kill.
- [ ] Controller node shows correct wheel arrays; controller active with velocity interfaces claimed.
- [ ] Short burst on `/diff_drive_controller/cmd_vel_unstamped` produces motion; MCAP recorded and logged.
