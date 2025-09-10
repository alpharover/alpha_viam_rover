# Drive Bring-up Status — ROS 2 Humble (RPi4) — 2025-09-10

Owner: agent (codex-cli)
Branch: `chore/lint-fixes`

## TL;DR

- Hardware path is sound: pigpio daemon stabilized; pins verified; both wheels spin forward/reverse via `pigs` and via the direct ROS path (`l298n_direct`).
- `diff_drive_controller` parameter delivery on Humble is fixed by passing a controller-specific `params_file` at load-time. We validated this pattern and saw the controller become ACTIVE and claim both wheel velocity interfaces earlier today.
- The remaining blocker is deterministically starting `ros2_control_node` with our L298N plugin on this image. In some flows it loads and runs; in others it fails pluginlib discovery and hangs spawners waiting for services. We hardened discovery by exporting the overlay paths in the launch env; this should make bring-up reliable.
- We also eliminated long‑lived spawner processes and added robust cleanup scripts to stop the “process whack‑a‑mole”.

What works now (repeatably)
- `pigpiod` runs as a single, stable systemd service (`-g` foreground); no lock flapping.
- `scripts/pigs_smoke.sh` moves both wheels fwd/rev (GPIO: ENA=26, ENB=22, IN1=19, IN2=13, IN3=6, IN4=5).
- `scripts/direct_smoke.sh` launches `l298n_direct` and moves the rover on `/cmd_vel` bursts; MCAP recorded.

What is flaky and what we changed
- Plugin discovery for `l298n_hardware/L298NSystemHardware` inside `ros2_control_node` would sometimes fail with “class … does not exist” even though the package was built and sourced. Root cause: launch environment occasionally lacked the overlay entries at the moment the manager spawned.
- Fix: In every bring-up launch where we start `ros2_control_node`, we now prepend the overlay explicitly to the process env: `AMENT_PREFIX_PATH+=install/l298n_hardware` and `LD_LIBRARY_PATH+=install/l298n_hardware/lib`.
- Spawner orphans led to persistent `/spawner_*` nodes requiring manual kill. We stopped using the spawner for diff drive and switched to `ros2 control load_controller --set-state active …`. We also taught `scripts/ros_clean.sh` to match the spawner by absolute path.

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
   - Expect: JSB + diff drive ACTIVE; interfaces claimed; wheels spin on `/diff_drive_controller/cmd_vel_unstamped`; bag saved under `bags/diff_<ts>/`.

2) If plugin still fails
   - Capture: first 60 lines of cm_only launch, plus `printenv | egrep 'AMENT_PREFIX|LD_LIBRARY'` from within the manager process (`/proc/<pid>/environ`).
   - Fallback: set Node env with `PLUGINLIB__DISABLE_BLOCKING_WARNING=1` and validate `ros2 pkg prefix l298n_hardware` is reachable.

3) If controller active but no motion
   - Verify subscriber count: `ros2 topic info /diff_drive_controller/cmd_vel_unstamped` (Subscribers: 1).
   - Publish via `scripts/pub_twist.py` (already used by smoke scripts).
   - Inspect `/rosout` lines from `l298n_hardware` for write() activity; check pigpio log for PWM updates during the burst.

4) Stabilization tasks (to avoid regressions)
   - Add a unit/system test that launches `cm_only` and asserts that `/controller_manager/list_controllers` becomes available within 5s and that `diff_drive_controller` can be loaded and activated.
   - Add a pre-flight “env echo” in bring-up that prints the manager’s `AMENT_PREFIX_PATH` and `LD_LIBRARY_PATH` at start.
   - Keep using `ros2 control load_controller` across all docs/scripts; avoid the spawner except for debugging.

## Acceptance criteria
- One clean off‑ground run where:
  - `diff_drive_controller` is ACTIVE,
  - both wheel velocity interfaces are [claimed],
  - a 1.5 s forward burst on `/diff_drive_controller/cmd_vel_unstamped` produces motion,
  - a short MCAP is recorded and linked in `AGENTS_PROGRESS.md`.

## Files touched today (high‑level)
- Launches: `drive_min`, `base_bringup`, and new `cm_only` (added) — plugin discovery env injected; spawner removed from scripted path.
- Scripts: new `pigs_smoke.sh`, `direct_smoke.sh`, `diff_drive_smoke.sh`, `pub_twist.py`; `ros_clean.sh` improved.
- Config: `controllers.yaml` uses `params_file` (absolute path) for diff drive; `diff_drive_params.yaml` tuned for Humble.
- ADR‑0005: set to Accepted; includes TL;DR and checklist for Humble params timing.

## Ask to the Architect
- Confirm the `params_file` approach and the `/**` root in `diff_drive_params.yaml` as the canonical Humble shape.
- Approve using `ros2 control load_controller --set-state active` instead of spawner across docs/scripts to avoid process creep.
- Confirm the plugin discovery hardening (env injection in manager Node) is acceptable for Humble, or prefer a different mechanism (e.g., ament index checks in a preflight node).
- If we hit another plugin discovery failure after these changes, approve escalating to a distro uplift (Iron/Jazzy) for more predictable controller param timing and pluginlib behavior.
