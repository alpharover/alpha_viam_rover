# ADR-0003: Pigpio Mode — Daemon Client (`pigpiod_if2`)

Status: Accepted (2025-09-10)

Context

ADR‑0001 described an in‑process pigpio usage, but the implemented hardware interface (`l298n_hardware`) links against the pigpio daemon client library (`pigpiod_if2`) and expects the `pigpiod` service. Mixed references in docs/scripts increased confusion.

Decision

Standardize on the pigpio daemon mode for the base drive on the RPi4:

- Use `pigpiod` (systemd service) and link the plugin against `pigpiod_if2`.
- Ensure only one owner of the GPIOs at a time; manual tests (`motor_test.sh`) refuse to run if `controller_manager` is active.

Consequences

- Lower latency than user‑space Python but slightly higher than in‑process C API; acceptable for DC drive at 50 Hz update.
- Operational clarity: `pigpiod` must be running; managed by Ansible and systemd.

Notes / Follow-ups

- If a future refactor switches to in‑process pigpio, update this ADR or supersede it with a new one, and remove the daemon dependency across Ansible and scripts.
- Keep encoder glitch filtering and PWM timing under the daemon client; performance is sufficient for our tick rates.

References

- ADR‑0001, ADR‑0002
- docs/control/ros2_control_hw.md (updated)
