# ADR-0004: Controller Activation on ROS 2 Humble (Spawner Param Delivery Failure)

Status: Proposed (2025-09-10)

Context

On the rover’s Ubuntu 22.04 + ROS 2 Humble image, attempts to load `diff_drive_controller` via the controller spawner with `--param-file` repeatedly fail during controller initialization:

- Error at load: `Invalid value set during initialization for parameter 'left_wheel_names': Parameter 'left_wheel_names' cannot be empty`.
- Evidence from sessions on 2025-09-09 shows the spawner sets a `params_file` string parameter on the controller node, but the typed parameters (e.g., `left_wheel_names`) are not applied before the controller validates them at `on_init`.
- Our helper (`scripts/activate_diff_drive.py`) which loads → sets parameters via `/<controller>/set_parameters` → activates, also failed when the controller was not yet created (load returned `ok=false`).
- Under the same image, the hardware plugin (`l298n_hardware`) activates successfully, and direct pigpio tests move the wheels; the failure scope is limited to controller parameter delivery at load.

Decision

Adopt a two-track approach:

1) Short-term for Phase 3 evidence and safe demos (Path B):
   - Provide a minimal “direct” driver that subscribes to `/cmd_vel` and maps linear.x to L298N PWM via pigpio, with a watchdog and deadband, while ROS control is stopped.
   - Launch: `ros2 launch alpha_viam_bringup drive_direct.launch.py`.
   - Guard: refuse to run if `controller_manager` is active to avoid GPIO contention; all test publishes use timeouts.

2) Primary path to restore `diff_drive_controller` (Path A):
   - Use a deterministic activation sequence that guarantees parameters exist before the controller validates them:
     a) Start `ros2_control_node` (controller_manager) after `/robot_description` is available.
     b) Spawn `joint_state_broadcaster` (no params).
     c) For diff drive, use one of:
        - A patched spawner call that truly applies `ros__parameters` at load time (preferred if reliable on this image), or
        - An activation shim that pre-wires parameters into the controller’s node context prior to validation (e.g., launch a transient node that creates the controller process with overrides), or
        - A two-step: load a pair of `forward_command_controller`s with reliable param delivery; map `/cmd_vel` → wheel velocity topics; once verified, switch back to `diff_drive_controller`.

Consequences

- Demos can proceed immediately with `drive_direct` while keeping all safety guards in place.
- For `diff_drive_controller`, we avoid hangs/races by removing reliance on the non-deterministic param-file behavior seen on this image.
- Adds one temporary launch (+small Python node) that should be removed once Path A is stable.

Alternatives Considered

- Keeping only the helper path (load → set → activate): Not viable when load fails due to missing params at init; the controller node must have parameters before its `on_init` validation.
- Embedding params under `controller_manager.ros__parameters.<controller>`: Observed not to apply to the controller at load time on this image.

Operational Notes

- Use `scripts/ros_clean.sh --force` before any test to kill stale ROS 2 processes, spawners, helpers, and launches.
- Never run pigpio scripts or the direct driver when `controller_manager` is active; GPIO contention can cause “moved once, then dead” symptoms.
- For L298N, ensure ≥50% duty for visible spin; our drive commands use 1.0 m/s (≈>80% duty under current scaling) for off-ground tests.

Open Questions for Architect

1) Activation strategy preference on Humble:
   - OK to add/maintain a small activation shim that injects controller parameters pre-load, or should we pin to a specific spawner version/patch?
2) Controller choice during interim:
   - Is it acceptable to use a pair of `forward_command_controller/ForwardCommandController` instances (one per wheel) with a small `/cmd_vel` to wheel velocity node while we debug diff drive parameterization on this image?
3) Pigpio mode:
   - We standardized on the pigpio daemon (`pigpiod_if2`) in ADR‑0003 to match the current implementation. Any preference to switch back to in-process pigpio and drop the daemon for latency/robustness?
4) Parameter file convention:
   - Confirm team-wide convention for controller param YAML shape. We added `configs/diff_drive_params.yaml` using top-level `ros__parameters` per spawner docs.

References

- ADR‑0001 L298N ros2_control Hardware Interface
- ADR‑0002 Controller Parameter Bridge (Proposed)
- ADR‑0003 Pigpio Mode — Daemon Client
- docs/control/phase3_drive_report.md (updated)
- Launches: `drive_min.launch.py`, `drive_direct.launch.py`

