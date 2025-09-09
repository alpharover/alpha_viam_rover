# Wi‑Fi Policy: Data vs Control, Ordering, and Telemetry

Summary of the project‑standard policy and configuration for dual‑NIC Wi‑Fi on the rover.

- Data plane on `wlan1` (external Alfa), control plane on `wlan0` (builtin).
- Route metrics: `wlan1` = 50 (preferred), `wlan0` = 600 (fallback/control).
- Persistent naming: systemd‑link with `MACAddress=<Alfa MAC>` → `Name=wlan1`.
- Supplicant: disable MAC randomization (`mac_addr=0`); optionally pin 5 GHz and reduce background scans during teleop.
- Startup ordering: `wpa_supplicant@wlan1.service` should `BindsTo=` and start `After=` the device unit `sys-subsystem-net-devices-wlan1.device` and `systemd-udev-settle.service`.
- USB hardening: powered hub, short shielded cable, strain relief; disable USB autosuspend for VID:PID `0bda:8812`.
- Foxglove telemetry: pin panels to `wlan1` topics and display `/wifi/iface`, `/wifi/link_ok`, `/wifi/signal_dBm`, `/wifi/flap_count`.

Acceptance test

- 30‑minute motion test; pass if:
  - No kernel USB errors (−32/−71) in `journalctl -k`.
  - `wlan1` remains present/named; `wpa_supplicant@wlan1` stays active.
  - `/wifi/iface = wlan1`, `/wifi/flap_count` ≤ 1, no UI “yo‑yo”.
  - SSH via `wlan0` stays up regardless of `wlan1`.

See also

- `docs/networking/awus036ach.md` for detailed setup steps.
- `docs/tools/foxglove.md` for panel/topic configuration.
