# Drive Bring-up Status — ROS 2 Humble (RPi4) — 2025-09-10

Owner: agent (codex-cli)
Branch: `chore/lint-fixes`

## TL;DR

- Hardware path is sound: pigpio daemon stabilized; pins verified; both wheels spin forward/reverse via `pigs` and via the direct ROS path (`l298n_direct`).
- Humble-safe fix applied: use spawner with `--param-file` and a controller-scoped YAML installed in the bringup package share. `controllers.yaml` now lists only controller types (no manager-side `params_file`).
- Launches now resolve configs/URDF via `get_package_share_directory('alpha_viam_bringup')`; no CWD assumptions; no ad‑hoc env patching for plugin discovery (overlay must be sourced).
- Smoke scripts updated to spawn controllers via the spawner and feed the param file explicitly.

What works now (repeatably)
- `pigpiod` runs as a single, stable systemd service (`-g` foreground); no lock flapping.
- `scripts/pigs_smoke.sh` moves both wheels fwd/rev (GPIO: ENA=26, ENB=22, IN1=19, IN2=13, IN3=6, IN4=5).
- `scripts/direct_smoke.sh` launches `l298n_direct` and moves the rover on `/cmd_vel` bursts; MCAP recorded.

What changed (to align with architect guidance)
- Use spawner `--param-file` on Humble to deliver `diff_drive_controller` params before configure; controller-scoped root key (`diff_drive_controller:`) — no wildcards.
- Remove manager-side `params_file` usage; simplify `controllers.yaml` to types only.
- Install `configs/` and `urdf/` into package share and reference via ament index; drop `os.getcwd()` pathing and env injection.
- Keep spawner but make it short‑lived via `--unload-on-kill`; cleanup script still exists for stragglers.

## Latest test result (14:32 CDT)

- Launch: `scripts/diff_drive_smoke.sh 12` (uses `cm_only.launch.py`, then spawns JSB and diff drive with `--param-file`).
- Observed:
  - Hardware: `RoverSystem` initialized, configured, and activated successfully.
  - Controllers: `joint_state_broadcaster` ACTIVE, `diff_drive_controller` ACTIVE.
  - Interfaces: both wheel velocity command interfaces [claimed].
  - Bag: recorded `/diff_drive_controller/cmd_vel_unstamped` (19 msgs), `/joint_states` (~468 msgs), `/tf`, `/tf_static`.
  - Physical: NO wheel movement observed during the forward burst.

Notes:
- This run had pigpio healthy (single daemon), ROS nodes started cleanly (no logging crash), and controller services available; teardown was clean. The discrepancy is between the software state (commands present, interfaces claimed) and physical motion.

Hypotheses to investigate with architect guidance:
- Controller delivered zero (or near-zero) wheel velocities despite Twist on `~/cmd_vel_unstamped` (e.g., parameter mismatch, saturation, or internal limits). However, `open_loop=true` and kinematics should map 0.8 m/s to ~13.3 rad/s.
- Hardware write path not exercising pigpio pins when called from ros2_control (pigpiod_if2) even though pigs/python pigpio works. Possible pigpio client interaction or timing nuance.
- Timing/watchdog: hardware watchdog (`watchdog_s=0.5`) prematurely stopping outputs if command updates were not frequent enough; but we published at 15 Hz for ~1.6 s.

Requested next steps (architect):
- Sanction adding DEBUG logging in the hardware plugin (`set_motor` logs: idx, cmd_rad_s, duty, dir) behind a parameter flag to capture during a short run.
- Sanction a minimal wheel command snooper: subscribe to `diff_drive_controller/cmd_vel_unstamped` and echo computed wheel setpoints (from controller state or via diagnostics) to confirm non-zero outputs.
- If acceptable, temporarily raise hardware watchdog to 2.0 s and reduce deadband to 0.02 to rule out edge cases.
- If pluginlib/params remain flaky after these, approve a distro uplift (Iron/Jazzy) for improved determinism.

## Environment
- HW: RPi4 + L298N H‑bridge; wheel GPIO per `hw/pinmap.yaml`.
- OS/ROS: Ubuntu 22.04.5; ROS 2 Humble binary.
- Services: `pigpiod.service` overridden to `ExecStart=/usr/local/bin/pigpiod -g`, `Type=simple`.
- Repo: alpha_viam_rover (branch `chore/lint-fixes`).

## Evidence
- Pigs smoke: both wheels moved (operator confirmed).
- Direct ROS smoke: `scripts/direct_smoke.sh` moved wheels; bag saved under `bags/direct_<ts>/`.
- diff_drive early session: controller ACTIVE and interfaces [claimed]; later sessions showed plugin discovery error; launches updated to enforce discovery env.

## Friction log (manual cleanup pain)

Observed repeatedly during the day:
- Multiple `/controller_manager` and `/spawner_*` nodes after ctrl+c; repeated manual `pkill` and `ros2 daemon stop/start` were needed.
- Spawner processes survived after timeouts and left services visible without a manager.
- CLI publishers intermittently failed with rcl context errors; replaced with a minimal Python publisher.

Mitigations now in repo:
- `scripts/ros_clean.sh` kills spawners by absolute path and whole process groups, with `--force` for `KILL`.
- All “smoke” scripts use bounded timeouts and clean teardown.
- Spawners removed from our scripted flows; we use `ros2 control load_controller --set-state active` instead.
- Bring-up launches inject overlay env into `ros2_control_node` so pluginlib consistently finds `l298n_hardware`.

## Current blockers (as of 2025-09-10 14:25 CDT)

1) Logging dir crash due to env override (now fixed in code)
   - Symptom in latest run: `ros2_control_node` aborts with `Failed to get logging directory` and `rcutils_expand_user failed`.
   - Root cause: our launch passed `env=...` to the Node, which replaced the full environment, removing `HOME`. rcl logging failed to resolve its directory and aborted.
   - Fix: switched to `additional_env=...` so we merge only AMENT/LD_LIBRARY patches with the existing environment. Applied to `drive_min`, `base_bringup`, and `cm_only`.

2) Plugin discovery race (intermittent)
   - Symptom: `controller_manager`: “According to the loaded plugin descriptions the class l298n_hardware/L298NSystemHardware … does not exist.”
   - Status: Launches patched to prepend overlay env for the manager process. Needs on‑device retest; if it still occurs, capture first 60 lines of `ros2_control_node` output and print `AMENT_PREFIX_PATH` from within the process.

3) Controller motion (diff drive) not yet observed on the latest run
   - Earlier we saw the controller ACTIVE with both wheel velocity interfaces claimed but still no motion. This was while pigpiod was flapping. After stabilizing pigpio and replacing the publishers, we need one clean retest through the hardened flow.

## Proposed plan (ASAP)

1) Retest hardened diff drive path
   - `scripts/ros_clean.sh --force`
   - Ensure pigpio: `systemctl status pigpiod` (active)
   - `colcon build --packages-select l298n_hardware alpha_viam_bringup --symlink-install && source install/setup.bash`
   - `scripts/diff_drive_smoke.sh 12`
   - Expect: JSB + diff drive ACTIVE; interfaces claimed; wheels spin on `/diff_drive_controller/cmd_vel_unstamped`; bag saved under `bags/diff_<ts>/`. Actual: ACTIVE + claimed + bag present, but NO physical motion in latest run.

2) If plugin still fails
   - Capture: first 60 lines of cm_only launch, plus `printenv | egrep 'AMENT_PREFIX|LD_LIBRARY'` from within the manager process (`/proc/<pid>/environ`).
   - Fallback: set Node env with `PLUGINLIB__DISABLE_BLOCKING_WARNING=1` and validate `ros2 pkg prefix l298n_hardware` is reachable.

3) If controller active but no motion
   - Verify subscriber count: `ros2 topic info /diff_drive_controller/cmd_vel_unstamped` (Subscribers: 1).
   - Publish via `scripts/pub_twist.py` (already used by smoke scripts).
   - Inspect `/rosout` lines from `l298n_hardware` for write() activity; check pigpio log for PWM updates during the burst. Pending architect approval, add DEBUG logs in hardware.

4) Stabilization tasks (to avoid regressions)
   - Add a unit/system test that launches `cm_only` and asserts that `/controller_manager/list_controllers` becomes available within 5s and that `diff_drive_controller` can be loaded and activated.
   - Add a pre-flight “env echo” in bring-up that prints the manager’s `AMENT_PREFIX_PATH` and `LD_LIBRARY_PATH` at start.
   - Use the spawner with `--param-file` across docs/scripts for Humble; avoid `ros2 control load_controller` for diff drive.

## Acceptance criteria
- One clean off‑ground run where:
  - `diff_drive_controller` is ACTIVE,
  - both wheel velocity interfaces are [claimed],
  - a 1.5 s forward burst on `/diff_drive_controller/cmd_vel_unstamped` produces motion,
  - a short MCAP is recorded and linked in `AGENTS_PROGRESS.md`.

## Files touched today (high‑level)
- Launches: `drive_min`, `base_bringup`, and new `cm_only` (added) — resolve paths via package share; no per-process env injection; spawner used with `--param-file`.
- Scripts: new `pigs_smoke.sh`, `direct_smoke.sh`, `diff_drive_smoke.sh`, `pub_twist.py`; `ros_clean.sh` improved.
- Config: `controllers.yaml` lists types only (no manager-side params_file); `diff_drive.params.yaml` controller-scoped and installed into package share.
- ADR‑0005: set to Accepted; includes TL;DR and checklist for Humble params timing.

## Ask to the Architect
- Confirm the controller-scoped param file + spawner `--param-file` approach as the canonical Humble shape (no wildcards, no manager-side params_file).
- Approve standardizing on the spawner with `--param-file` for diff drive on Humble.
- Confirm relying on a sourced overlay (ament index) for plugin discovery — no per-Node env patching needed.
- If we hit another plugin discovery failure after these changes, approve escalating to a distro uplift (Iron/Jazzy) for more predictable controller param timing and pluginlib behavior.
