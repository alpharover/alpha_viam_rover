# ros2_control Hardware Interface

Design notes for the rover base hardware interface and safety.

- L298N mapping (ENA/ENB PWM; IN1..IN4)
- pigpio usage, PWM frequency
- Encoder callbacks and filtering
- Watchdogs and timeouts

Planning artifacts

- `configs/diff_drive.yaml` defines `diff_drive_controller` parameters (wheel separation/radius, names, publish rate).
- `configs/ros2_control.yaml` (placeholder) lists wheel joints and simple transmissions for planning; will be replaced by proper `ros2_control` parameters when the hardware interface is implemented.

Notes

- PWM: prefer hardware-timed PWM (pigpio) for stable control; start conservative and clamp PWM ranges.
- Encoders: apply pigpio glitch filters; measure ticks-per-rev and verify sign conventions.
- Safety: ensure `cmd_vel` watchdog stops motors on stale commands; motors disabled on boot.

TODO: Add diagrams and timing details; pinout table and accepted ranges.
