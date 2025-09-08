# Next Pi Session — Checklist (Fast Path)

Run these on the rover to close Phase 0 and start Phase 1.

- Networking (optional): If using Alfa AWUS036ACH, follow `docs/networking/awus036ach.md` first.
- Provisioning: `ansible-playbook -i ansible/inventory ansible/playbook.yml` (overlays, pigpio, ROS, udev, systemd).
- Reboot.
- Bus checks: capture `i2cdetect -y 1`, `ls -l /dev/spidev*`, and UART info into `docs/hw/bus_checks.md` with artifacts in `docs/hw/artifacts/phase1/`.
- GPIO pulse: `python3 scripts/gpio_pulse.py --pin 12 --hz 10 --cycles 20` to validate PWM/GPIO path.
- Time sync + health: fill `docs/bringup/time_sync.md` with `timedatectl`, `chronyc sources`, and `ros2 doctor --report`.
- Talker/listener bag: record 10–20 s to `bags/phase0_talker_listener/` per `docs/bringup/phase0.md`.
- Foxglove: `ros2 launch launch/teleop_viz.launch.py use_teleop:=true`, connect Foxglove, screenshot, and add to `docs/tools/foxglove.md`.
- Log: Append results + links to `AGENTS_PROGRESS.md`.

When done, move to Phase 2 (IMU + INA219) with the steps in the Issues.

# Handoff Template

Use this template when handing off a change:

- What changed:
- Why (link ADR):
- How to test:
- Expected outputs:
- Gotchas:
