# ADR-0005: Humble Controller Activation — diff_drive init-time params; forward-controller fallback

Status: Proposed (needs-architect)

Date: 2025-09-10

## Executive Summary

On ROS 2 Humble (Raspberry Pi 4), `diff_drive_controller` fails to load because its required parameters (`left_wheel_names`, `right_wheel_names`) are validated during controller initialization. Parameter delivery via the spawner `--param-file` occurs after instantiation, so validation trips with empty defaults. Manager-level YAML attempts (both top-level and nested under `controller_manager`) did not place the params at init time in this environment. As a result, `diff_drive_controller` never becomes active and motors do not move.

To maintain velocity-command semantics while we resolve Humble’s timing edge case, we prepared a fallback using two `forward_command_controller` instances (one per wheel). Initial spawner attempts also showed `joints parameter was empty` with our first YAML shape; we added a helper to set params first and then activate, keeping all waits bounded with timeouts. This provides a path to validate hardware motion immediately while awaiting guidance on diff drive.

We request architect decisions on the YAML pattern to ensure `diff_drive_controller` receives parameters at initialization on Humble, or approval to proceed with the forward-controller fallback for demos/testing and defer diff drive until a distro where spawner timing is improved.

## Environment & Safety

- Hardware: Raspberry Pi 4; L298N H-bridge; pigpio daemon (`pigpiod`) for PWM/encoders.
- OS/ROS: Humble (binary packages). Workspace overlay with custom `l298n_hardware` plugin.
- Safety: Wheels off ground, watchdog active; pigpio daemon only; no concurrent GPIO users.

## What Works

- Plugin discovery: `l298n_hardware` loads under `ros2_control_node` when the overlay is sourced; hardware initializes and activates successfully.
- Hardware interfaces: `left_wheel_joint/velocity` and `right_wheel_joint/velocity` command interfaces exported; state interfaces present.
- JSB: `joint_state_broadcaster` loads, configures, and activates.

## What Fails (Key Symptoms)

1) diff drive — init-time param validation

- Error: `Invalid value set during initialization for parameter 'left_wheel_names': Parameter 'left_wheel_names' cannot be empty`.
- Observed repeatedly whether the controller type + params are provided (a) nested under `controller_manager` or (b) at top-level under `diff_drive_controller` in the YAML handed to `ros2_control_node`.
- Spawner `--param-file` is too late; helper that uses `set_parameters` also too late because `load_controller` triggers initialization before parameters can be set.

2) forward controllers — YAML shape via spawner

- Initial spawner runs with param-file produced `joints parameter was empty` at configure. The YAML file used the controller-name-prefixed shape:

  ```yaml
  /left_wheel_velocity_controller:
    ros__parameters:
      joints: [left_wheel_joint]
      interface_name: velocity
  ```

- On Humble, spawner appears to expect a flat shape for the targeted node:

  ```yaml
  ros__parameters:
    joints: [left_wheel_joint]
    interface_name: velocity
  ```

- We added a helper (`scripts/activate_forward.py`) to load → set params → activate both controllers with bounded waits. This avoids relying on the spawner’s param timing.

## Evidence & Logs (on-device)

- Diff drive init-time failure: `log/agent_runs/20250910_091145/drive_min.log` and `.../20250910_103159/drive_min.log` show the “cannot be empty” error during the initialization stage.
- Hardware OK: Same logs show `RoverSystem` (l298n_hardware) loading, configuring, and activating successfully.
- Forward controllers via spawner: `log/agent_runs/20250910_104046/drive_forward.log` and `.../20250910_104520/drive_forward_retry.log` show the `joints parameter was empty` at configure using our first YAML shape.
- AGENTS_PROGRESS.md updated with timestamps and paths to logs/bags.

## Actions Taken in Repo (branch: `chore/lint-fixes`)

- Launches (`drive_min`, `base_bringup`):
  - Deliver `robot_description` via topic (recommended on Humble).
  - Wrap spawner calls with `timeout` and `--unload-on-kill` to avoid hangs.
- Controller YAML:
  - `configs/controllers.yaml` now carries controller types and a nested params block under `controller_manager.diff_drive_controller.ros__parameters` (Humble attempt). We also tested the top-level `diff_drive_controller: ros__parameters: {…}` shape; both yielded the same init-time failure here.
- Forward path:
  - `configs/wheels_forward.yaml` and `bringup/.../drive_forward.launch.py` (JSB only; forward controllers handled via helper rather than param at spawn time).
  - Helper `scripts/activate_forward.py` to load → set params → activate left/right controllers; all waits bounded.
- Timeouts everywhere: launch spawners, CLI pubs, bag recording, and service waits use explicit time bounds.

## Hypothesis (Humble timing/shape)

- Humble’s `diff_drive_controller` validates required params during the controller’s initialization step. On this build, neither the manager-nested nor top-level YAML forms provided those params at the moment of instantiation. The spawner `--param-file` applies too late (pre-configure but post-init). Hence the consistent init-time failure.
- For `forward_command_controller`, our first spawner YAML included the controller node name as a top-level key. Humble’s spawner seems to expect a flat `ros__parameters` mapping for that specific target node. Our helper bypasses this by setting params directly before activation.

## Requests for Architect Decision

1) diff drive on Humble — choose path:
   - A) Approve forward-controller fallback for motion smoke, demos, and dev while we park diff drive until an upgraded distro (Iron/Jazzy) where spawner parameter timing is improved and documented.
   - B) If we must keep Humble + diff drive, approve a controller activation shim that patches the initial parameter delivery by instantiating the controller with parameters (requires code in or around controller_manager; higher risk/maintenance).

2) Param-file conventions — confirm desired shapes:
   - For spawner (Humble), use flat `ros__parameters:` files per controller (no prefixed node keys). We can migrate our spawner files accordingly.

3) Topic semantics — confirm preferred cmd_vel entrypoint:
   - Keep controller-local `~/cmd_vel_unstamped` and provide a small bridge from `/cmd_vel`, or standardize on the controller-local topic for now.

4) Plugin discovery — approve minor environment hardening:
   - Ensure `install/setup.bash` is sourced in all workflows and that the C++ hardware plugin remains discoverable without manual env exports.

## Proposed Immediate Plan (if approved)

1) Motion today: switch to forward controllers with corrected flat param files and the helper; record a short MCAP; append to AGENTS_PROGRESS.md.
2) Diff drive later: revisit on Iron/Jazzy or adopt a manager-side enhancement to guarantee init-time parameter presence.
3) Clean up spawner YAMLs under `configs/spawner/` to the flat `ros__parameters` shape and adjust docs accordingly.

## Acceptance Criteria

- Forward controllers active; small positive/negative velocity bursts produce motion off-ground; MCAP recorded in `bags/samples/`.
- No hangs during bring-up; all long operations bounded by timeouts.
- Architect provides guidance on Humble strategy vs. distro upgrade for diff drive.

## Risks

- Continuing on Humble with diff drive may force more bespoke shims around controller_manager that we would later remove when upgrading.
- L298N + Pi timing: acceptable for demos, but we will consider a motor driver upgrade (MOSFET-based) for efficiency and control headroom.

## Appendix — Commands (bounded)

```bash
# Ensure pigpio
sudo systemctl start pigpiod || sudo /usr/local/bin/pigpiod

# Forward path (Terminal A)
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch alpha_viam_bringup drive_forward.launch.py

# Forward activation (Terminal B)
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/activate_forward.py configs/wheels_forward.yaml
ros2 control list_controllers
timeout 2s ros2 topic pub -r 15 /left_wheel_velocity_controller/commands  std_msgs/msg/Float64MultiArray '{data: [2.0]}'
timeout 2s ros2 topic pub -r 15 /right_wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray '{data: [2.0]}'
```

