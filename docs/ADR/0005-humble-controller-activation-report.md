# ADR-0005: Humble Controller Activation — params at load-time via params_file

Status: Accepted

Date: 2025-09-10

## Decision

On ROS 2 Humble, `diff_drive_controller` must receive its required parameters at controller creation time. We will provide those parameters via the controller manager’s `<controller_name>.params_file`, referenced from `configs/controllers.yaml` as an absolute path. We will not pass a param file to the spawner for `diff_drive_controller`.

## TL;DR — Required changes

1) `configs/controllers.yaml` uses controller types and `params_file` (absolute path):

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50
    use_sim_time: false

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController
      params_file: /home/alpha_viam/alpha_viam_rover/configs/diff_drive_params.yaml
```

2) `configs/diff_drive_params.yaml` contains the controller’s parameters. On this Humble image the safest match is the wildcard root (`/**`) so the controller node picks them up at creation:

```yaml
/**:
  ros__parameters:
    left_wheel_names: [left_wheel_joint]
    right_wheel_names: [right_wheel_joint]
    wheel_separation: 0.30
    wheel_radius: 0.06
    wheels_per_side: 1
    use_stamped_vel: false
    cmd_vel_timeout: 0.5
    publish_rate: 50.0
    enable_odom_tf: true
    odom_frame_id: odom
    base_frame_id: base_link
  open_loop: true
```

3) Launch: spawn JSB and diff drive without `--param-file`:

```bash
ros2 run controller_manager spawner joint_state_broadcaster \
  --controller-manager /controller_manager --activate

ros2 run controller_manager spawner diff_drive_controller \
  --controller-manager /controller_manager --activate --unload-on-kill
```

## Why this fixes the failure

`diff_drive_controller` validates `left_wheel_names` and `right_wheel_names` during initialization. On Humble, spawner `--param-file` applies parameters before configure, i.e., too late for `on_init`. By using `controller_manager....params_file`, the controller receives parameters at creation and passes initialization.

## Sanity checks

- Absolute path in `params_file` (no CWD assumptions).
- Controller node parameters present:
  - `ros2 param get /diff_drive_controller left_wheel_names`
  - `ros2 param get /diff_drive_controller right_wheel_names`
- Active controller with velocity interfaces claimed:
  - `ros2 control list_controllers`
  - `ros2 control list_hardware_interfaces`
- Input topic with `use_stamped_vel=false` is `/diff_drive_controller/cmd_vel_unstamped`.

## What to remove/avoid

- Do not nest `diff_drive_controller.ros__parameters` under the manager in the same YAML passed to `ros2_control_node`.
- Do not pass `--param-file` to the diff-drive spawner on Humble.

## Optional fallback (for demos only)

Forward controllers remain available for quick spins and debugging using `configs/wheels_forward.yaml` and `spawner -p` per-controller; this path is no longer primary.

## Validation sequence (off-ground)

```bash
# Clean + pigpio
scripts/ros_clean.sh --force
sudo systemctl start pigpiod || sudo /usr/local/bin/pigpiod

# Launch
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch alpha_viam_bringup drive_min.launch.py

# Sanity: controller params exist on the controller node
timeout 4s ros2 param get /diff_drive_controller left_wheel_names
timeout 4s ros2 param get /diff_drive_controller right_wheel_names

# Interfaces claimed
timeout 4s ros2 control list_controllers
timeout 4s ros2 control list_hardware_interfaces

# Move (unstamped)
timeout 2s ros2 topic pub -r 15 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}"
sleep 0.5
timeout 1s ros2 topic pub --once /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Acceptance checklist

- [x] `controllers.yaml` uses `type` + `params_file` (absolute path).
- [x] `diff_drive_params.yaml` contains required keys with sane values.
- [x] JSB spawns; diff drive spawns without `--param-file`.
- [ ] Controller node shows correct wheel arrays; controller active with velocity interfaces claimed.
- [ ] Short burst on `/diff_drive_controller/cmd_vel_unstamped` produces motion; MCAP recorded and logged.
