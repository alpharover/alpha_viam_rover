# Alfa AWUS036ACH (RTL8812AU) — USB Wi‑Fi Adapter

This adapter uses the Realtek 8812AU chipset and typically requires a DKMS driver on Ubuntu 22.04. Proceed if you accept DKMS maintenance.

Recommended path (RPi4 + netplan + systemd‑networkd)

1) Driver (aircrack‑ng `rtl88xxau` via DKMS)
```bash
sudo apt-get update && sudo apt-get install -y dkms git build-essential
sudo git clone https://github.com/aircrack-ng/rtl8812au.git /usr/src/rtl8812au
cd /usr/src/rtl8812au
sudo make dkms_install
# Load the module name used by this tree
sudo modprobe 88XXau
```

2) Regulatory domain
```bash
echo 'REGDOMAIN=US' | sudo tee /etc/default/crda
sudo iw reg set US
```

3) Persistent naming to `wlan1`

- Find the Alfa MAC via `ip -br link` (e.g., `00:c0:ca:b1:22:3b`).
- Create `/etc/systemd/network/10-alfa-wlan1.link`:
```ini
[Match]
MACAddress=AA:BB:CC:DD:EE:FF

[Link]
Name=wlan1
```
- Reload udev: `sudo udevadm control --reload && sudo udevadm trigger -c add -s net`

4) Netplan (keep internal wlan0 as fallback; prefer Alfa)

- `/etc/netplan/99-alfa-wifi.yaml`:
```yaml
network:
  version: 2
  renderer: networkd
  wifis:
    wlan1:
      dhcp4: true
      optional: true
      access-points:
        "<SSID>":
          password: "<PSK>"
      dhcp4-overrides:
        route-metric: 50
```
- `/etc/netplan/99-wlan0-metric.yaml`:
```yaml
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: true
      optional: true
      dhcp4-overrides:
        route-metric: 600
```
- Apply: `sudo netplan apply`

5) Full‑power profile (disable powersave + set txpower)

- Disable Wi‑Fi powersave:
```ini
# /etc/systemd/system/wifi-powersave@.service
[Unit]
Description=Disable WiFi powersave on %I
After=netplan-wpa-%i.service network-online.target
Wants=netplan-wpa-%i.service

[Service]
Type=oneshot
ExecStart=/usr/sbin/iw dev %I set power_save off

[Install]
WantedBy=multi-user.target
```
- Set transmit power to 30 dBm (if driver/regulatory allow):
```ini
# /etc/systemd/system/wifi-txpower@.service
[Unit]
Description=Set WiFi txpower for %I
After=netplan-wpa-%i.service network-online.target
Wants=netplan-wpa-%i.service

[Service]
Type=oneshot
ExecStart=/usr/sbin/iw dev %I set txpower fixed 3000

[Install]
WantedBy=multi-user.target
```
- Enable for Alfa: `sudo systemctl enable --now wifi-powersave@wlan1 wifi-txpower@wlan1`
- Optional USB full‑power udev rule:
```udev
# /etc/udev/rules.d/80-alfa-rtl8812au-power.rules
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="0bda", ATTR{idProduct}=="8812", TEST=="power/control", ATTR{power/control}="on"
```

6) Verify
```bash
iw dev wlan1 info           # shows txpower and current channel
ip -br addr show wlan1      # shows IP
ip route                    # default should prefer wlan1 (metric 50)
```

Notes
- Actual max txpower depends on band/channel, regulatory domain, and driver. 2.4 GHz often caps at ~20 dBm; some 5 GHz channels allow ~30 dBm. The rtl88xxau driver may permit an override via `iw` even on 2.4 GHz.
- Keep wlan0 connected with a higher metric as a fallback path for headless access.
- Avoid committing PSKs to version control; use Ansible Vault or host‑only files.
