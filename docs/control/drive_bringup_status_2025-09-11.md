# Drive Bring-up Status — 2025-09-11

Summary
- Hardware (l298n_hardware) loads; watchdog triggers as expected until /cmds arrive.
- DiffDrive on this Humble image fails at init because typed params aren’t applied at load; spawner `--param-file` only sets a string on /controller_manager.
- Forward controllers path (two ForwardCommandControllers) is the near‑term motion smoke path; requires controller‑scoped YAML.

What works now
- Manager-only bring-up (`cm_only.launch.py`) with URDF via topic.
- JSB spawns and activates.
- Forward controllers: load→set params→configure→activate pipeline is scripted; use repo path `configs/spawner/forward.yaml` when package share not in scope.

What’s blocked
- DiffDriveController init due to missing typed parameters at creation time.

Plan
1) Validate motion with forward controllers; record MCAP evidence with commands + joint_states + TF.
2) ADR‑0006: adopt a Humble‑compatible controller param bridge (shim/preloader or version pin); after approval, switch back to DiffDrive.

Operator steps (tomorrow)
```
# Terminal A
ros2 launch alpha_viam_bringup cm_only.launch.py

# Terminal B
ros2 run controller_manager spawner joint_state_broadcaster --controller-manager /controller_manager --activate || true
SP="$PWD/configs/spawner/forward.yaml"
ros2 run controller_manager spawner left_wheel_velocity_controller  --controller-manager /controller_manager --controller-type forward_command_controller/ForwardCommandController  --param-file "$SP" --inactive
ros2 run controller_manager spawner right_wheel_velocity_controller --controller-manager /controller_manager --controller-type forward_command_controller/ForwardCommandController --param-file "$SP" --inactive
ros2 service call /controller_manager/configure_controller controller_manager_msgs/srv/ConfigureController "{name: 'left_wheel_velocity_controller'}"
ros2 service call /controller_manager/configure_controller controller_manager_msgs/srv/ConfigureController "{name: 'right_wheel_velocity_controller'}"
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['left_wheel_velocity_controller','right_wheel_velocity_controller'], strictness: 2, timeout: {sec: 3, nanosec: 0}}"
# quick command burst
ros2 topic pub -r 10 /left_wheel_velocity_controller/commands  std_msgs/msg/Float64MultiArray "{data: [2.0]}"
ros2 topic pub -r 10 /right_wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [2.0]}"
```

Evidence
- Bags: `bags/diff_drive_20250910_193019` (JSB only), `log/agent_runs/20250910_200608/bag_0.mcap` (JSB + TF), multiple forward attempts under `log/agent_runs/`.

