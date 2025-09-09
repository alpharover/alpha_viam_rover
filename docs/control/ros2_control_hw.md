# ros2_control Hardware Interface (L298N + Encoders)

This rover uses an L298N dual H‑bridge for the base drive and quadrature encoders on each wheel. We expose a ros2_control `SystemInterface` so `diff_drive_controller` can command wheel velocities and read joint states (position/velocity) from encoders.

Package: `drivers/l298n_hardware` (C++ / ament_cmake)

- Plugin: `l298n_hardware/L298NSystemHardware` (exported via pluginlib)
- Library uses `pigpio` for PWM and GPIO with optional glitch filtering for encoders.

Pin Map (default; BCM numbering)

- Left motor: `ENA/PWM=GPIO26`, `IN1=GPIO19`, `IN2=GPIO13`
- Right motor: `ENB/PWM=GPIO22`, `IN3=GPIO6`, `IN4=GPIO5`
- Encoders: config via URDF params `left_enc_a/b`, `right_enc_a/b` (set to `-1` to disable)

URDF Wiring

`urdf/rover.urdf.xacro` includes a `<ros2_control>` block pointing to the plugin and listing pins/params. Joints must be named `left_wheel_joint` and `right_wheel_joint` to match controller configs.

Control & Safety

- PWM: pigpio software‑timed PWM at `pwm_freq=20000` Hz (out of audible range) with `pwm_range=255`.
- Scaling: commanded wheel velocity (rad/s) is linearly mapped to PWM based on `max_wheel_rad_s`. Start conservative (e.g., `≤ 20 rad/s`) and tune after encoder verification.
- Direction: sign sets `INx` polarity; per‑wheel inversion via `invert_left/right`.
- Brake/Coast: `brake_on_zero=false` (coast) by default; set `true` to drive both inputs high on zero command (L298N brake).
- Encoders: quadrature decoded with a small state table, `encoder_glitch_us=100` default. Set `ticks_per_rev` to your measured value (post‑gear, including any quadrature multiplication effect).
- Watchdog: hardware layer clamps outputs to zero if no writes occur within `watchdog_s` (default `0.5`). Controller adds `cmd_vel_timeout` as a second guard.

Controller Parameters

`configs/controllers.yaml` and `configs/diff_drive.yaml` align on:

- `left_wheel_names`, `right_wheel_names`
- `wheel_separation`, `wheel_radius`
- `publish_rate`
- `enable_odom_tf`
- `cmd_vel_timeout` (0.5 s)
- `odom_frame_id=odom`, `base_frame_id=base_link`

Acceptance Procedure (Phase 3)

1) Off‑ground test: enable `spawn_drive:=true`; command small `cmd_vel` forward and reverse; verify symmetric wheel spin; stop on zero.
2) On‑floor test: straight‑line distance over a marked course (tape measure) within tolerance; in‑place rotation ≈ 360° within tolerance.
3) TF/Controller: `odom→base_link` continuous; no controller timeouts.
4) Evidence: short MCAP with `/cmd_vel`, `/joint_states`, `/odometry/filtered` (after EKF), `/tf`; Foxglove screenshot.

Operational Notes (Humble quirk)

- On this rover’s Humble image, the controller spawner’s `--param-file` does not set controller parameters; controllers fail to configure due to missing params. Use a parameter bridge (see ADR‑0002) that loads the controller, sets parameters via `/<controller>/set_parameters`, and activates via `/controller_manager/switch_controller`.
- Manual motor tests must not run while `controller_manager` is active. Use `scripts/ros_clean.sh --force` first, then `scripts/motor_test.sh` (guarded) to verify motion.

Escalation & ADR

Changes to PWM ranges, pin maps, watchdogs, or encoder wiring require `needs-architect` and an ADR. See `docs/ADR/0001-l298n-ros2-control.md` (added with this change).

Notes

- Begin with wheels off the ground; confirm directions and inversion before any floor test.
- If encoders are not wired yet, the plugin will integrate commands as a fallback for position/velocity; acceptance requires encoders to be active.
- pigpio must be available (install `pigpio`/`pigpio-dev`); the plugin initializes pigpio internally and does not require `pigpiod`.
