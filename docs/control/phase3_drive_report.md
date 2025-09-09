# Phase 3 — Base Drive Bring‑up Report (Off‑ground)

Date: 2025‑09‑09
Author: codex-cli (on‑device)

Scope
- Implement ros2_control hardware for L298N + encoders; wire URDF; bring up controllers; verify off‑ground motion; capture evidence.

Summary
- Implemented a C++ ros2_control SystemInterface (`l298n_hardware`) using pigpio’s daemon client (pigpiod_if2). Added encoder decoding (quadrature, glitch filter), PWM scaling, and a steady‑clock watchdog. Wired into URDF and bring‑up.
- Verified hardware path with manual pigpio tests: motors move reliably when ROS is fully stopped. When controller_manager is running, ros2_control owns the pins and watchdog writes zeros at 50 Hz, which overrides pigpio (no motion in that state). Root cause of intermittent “moved once, then not again.”
- On this Humble image, the controller spawner’s `--param-file` does not set parameters on controllers; `diff_drive_controller` rejects empty `left_wheel_names`, and per‑wheel `forward_command_controller` rejects empty `joints` (stays `unconfigured`). Params do appear under `/controller_manager` but aren’t propagated to the controller at init time.
- Temporary mitigation: added `scripts/motor_test.sh` with a ROS guard and clear forward/stop/reverse sequence for safe manual validation when ROS is not running.

Environment notes
- RPi4 + Ubuntu 22.04 + ROS 2 Humble.
- pigpio built from source under `/usr/local`; client libs installed via `libpigpiod-if-dev` (used by plugin).

What works
- `l298n_hardware` loads and activates cleanly; joint_state_broadcaster loads/activates.
- Motors move as expected with pigpio when ROS is not running (controller_manager stopped). Movement directions match expectations.

What’s blocked
- Controller parameter injection path on this Humble build. Spawner `--param-file` and manager‑level params are not being applied to controllers during `load_controller`.

Root cause detail
- Two distinct issues were observed:
  1) Ownership conflict: ros2_control watchdog writes zeros to PWM/IN pins while tests tried to use pigpio → no motion. Fixed by killing ROS before pigpio tests.
  2) Param propagation: Controller params not applied by spawner; symptoms: `diff_drive_controller` → “left_wheel_names cannot be empty”; per‑wheel forward controllers → “‘joints’ parameter was empty.”

Recommendations
- Short‑term: Use a small helper to `load_controller` → call `/<controller>/set_parameters` with our YAML → `switch_controller` to `active`. This is robust across Humble variants and bypasses the spawner’s param quirks.
- Add an enable/disable ramp in the hardware mapping (small PWM “kick” on transition) if needed after ROS path is verified, but L298N typically doesn’t require it once enabled.
- Always run manual motor tests only with ROS stopped (use `scripts/ros_clean.sh --force`, then `scripts/motor_test.sh`).

Acceptance status (off‑ground)
- Hardware path validated via pigpio (Forward/Reverse tests observed). ✔
- ROS controllers: joint_state_broadcaster active; diff drive pending param bridge. ◻

Follow‑ups
- Implement controller parameter bridge (helper node/script) and complete /cmd_vel off‑ground spin; record short MCAP.
- Wire encoders physically and validate ticks/velocity; tune `ticks_per_rev` and inversion.
- Open ADR for the param bridge and spawner behavior on Humble.

Risks
- Pin map drift between boards; keep `hw/pinmap.yaml` authoritative and align URDF + plugin params.
- pigpio daemon must be available for hardware plugin.

Appendix (key files)
- drivers/l298n_hardware/* — ros2_control plugin
- urdf/rover.urdf.xacro — ros2_control tags and params
- bringup/alpha_viam_bringup/launch/base_bringup.launch.py — controller_manager + spawners
- scripts/motor_test.sh — guarded pigpio test
- docs/ADR/0001-l298n-ros2-control.md — decision record

