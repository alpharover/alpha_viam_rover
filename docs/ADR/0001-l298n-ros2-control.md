# ADR-0001: L298N ros2_control Hardware Interface

Status: Accepted (2025-09-09)

Context

The rover uses an L298N dual H-bridge to drive two DC motors and wheel encoders for odometry. We need a robust, real-time-ish control path compatible with `diff_drive_controller`, plus safe defaults (watchdogs, braking/coast) and accurate joint state feedback from encoders.

Decision

Implement a C++ ros2_control `SystemInterface` (`l298n_hardware/L298NSystemHardware`) using the `pigpio` C library (daemon client `pigpiod_if2`) for PWM/GPIO and in-process quadrature decoding with glitch filtering. Wire this hardware into the URDF `ros2_control` block; keep controller configs in `configs/controllers.yaml` and `configs/diff_drive.yaml` aligned. Add a hardware-layer watchdog and keep motors disabled on boot.

Details

- PWM: pigpio PWM at 20 kHz (out of audible range), range 255.
- Mapping: linear from wheel rad/s to PWM duty; `max_wheel_rad_s` sets the full-scale.
- Direction: sign sets IN pins; `invert_left/right` parameters handle mechanical swaps.
- Encoders: quadrature with small finite-state table; `encoder_glitch_us` for debouncing; `ticks_per_rev` config.
- Watchdogs: hardware `watchdog_s` plus controller `cmd_vel_timeout`.
- Pins: defaults match `hw/pinmap.yaml` (ENA=26, ENB=22, IN1=19, IN2=13, IN3=6, IN4=5). Encoders are parameters; set proper GPIOs when wired.

Consequences

- Enables `diff_drive_controller` with real wheel feedback and consistent TF (`odom→base_link`).
- Requires pigpio daemon on the rover; plugin uses `pigpiod_if2` to connect and cleans up on deactivate. See ADR‑0003 for the mode selection.
- Open-loop PWM mapping is simple to start; can later add closed-loop velocity control (PI) using encoder velocity.

Alternatives Considered

- Python node proxy (rclpy) without ros2_control: rejected; does not integrate cleanly with controllers/TF.
- Using pigpio in-process: deferred; current approach uses the daemon client for operational simplicity (see ADR‑0003).

Follow-ups

- Measure actual `ticks_per_rev` and signs; update params.
- Optionally implement PI velocity control per wheel.
- Add endurance test and thermal/voltage monitoring overlayed in Foxglove.
