# Wi‑Fi Policy: Network Configuration and Telemetry

Summary of the project‑standard policy and configuration for networking on the rover.

## Network Interfaces

| Interface | Role | Route Metric | Status |
|-----------|------|--------------|--------|
| `wlan1` | Primary (Alfa USB adapter) | 100 | Active |
| `eth0` | Fallback (wired Ethernet) | 700 | Optional |
| `wlan0` | Onboard WiFi | N/A | **Disabled** |

## Configuration

- Persistent naming: systemd‑link with `MACAddress=<Alfa MAC>` → `Name=wlan1`.
- Supplicant: disable MAC randomization (`mac_addr=0`); optionally pin 5 GHz and reduce background scans during teleop.
- Startup ordering: `wpa_supplicant@wlan1.service` should `BindsTo=` and start `After=` the device unit `sys-subsystem-net-devices-wlan1.device` and `systemd-udev-settle.service`.
- USB hardening: powered hub, short shielded cable, strain relief; disable USB autosuspend for VID:PID `0bda:8812`.
- ROS config: `configs/network.yaml` sets `wifi_iface: wlan1` for the `wifi_monitor` node.
- Web UI/Foxglove telemetry: pin panels to `wlan1` topics and display `/wifi/iface`, `/wifi/link_ok`, `/wifi/signal_dBm`, `/wifi/flap_count`.

## Acceptance test

30‑minute motion test; pass if:
- No kernel USB errors (−32/−71) in `journalctl -k`.
- `wlan1` remains present/named; `wpa_supplicant@wlan1` stays active.
- `/wifi/iface = wlan1`, `/wifi/flap_count` ≤ 1, no UI "yo‑yo".
- mDNS resolution (`alpha-viam.local`) works consistently.

See also

- `docs/networking/awus036ach.md` for detailed setup steps.
- `docs/tools/foxglove.md` for panel/topic configuration.
