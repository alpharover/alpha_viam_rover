# Alfa AWUS036ACH (RTL8812AU) — USB Wi‑Fi Adapter

Note: This adapter uses the Realtek 8812AU chipset and typically requires an out‑of‑tree DKMS driver on Ubuntu 22.04. Prefer in‑kernel chipsets for long‑term stability; proceed with ACH if you accept DKMS maintenance.

Steps (RPi4, Ubuntu 22.04)

1) Install DKMS driver (aircrack‑ng’s rtl8812au)
```
sudo apt-get update && sudo apt-get install -y dkms git build-essential
sudo git clone https://github.com/aircrack-ng/rtl8812au.git /usr/src/rtl8812au
cd /usr/src/rtl8812au
sudo make dkms_install
```

2) Set regulatory domain and prefer 5 GHz
```
echo 'REGDOMAIN=US' | sudo tee /etc/default/crda
sudo iw reg set US
```

3) Persistent naming (optional)

- Use systemd‑networkd .link file to give the Alfa a stable name by MAC.
- Find MAC: `ip link` (the Alfa often shows as `wlx<mac>`).
- Create `/etc/systemd/network/10-alfa.link`:
```
[Match]
MACAddress=AA:BB:CC:DD:EE:FF

[Link]
Name=wlan1
```

4) NetworkManager profile (example)
```
nmcli connection add type wifi ifname wlan1 con-name rover-wifi ssid <SSID>
nmcli connection modify rover-wifi wifi.band a  wifi.cloned-mac-address preserve
nmcli connection modify rover-wifi wifi-sec.key-mgmt wpa-psk wifi-sec.psk <PASSWORD>
```

5) Throughput test and ROS 2 echo
```
sudo apt-get install -y iperf3
iperf3 -c <host> -t 10
ros2 topic echo /chatter -n 10
```

6) Udev tag for identification (optional)

Create `/etc/udev/rules.d/90-alfa.rules` (ids are commonly `0bda:8812`):
```
SUBSYSTEM=="usb", ATTR{idVendor}=="0bda", ATTR{idProduct}=="8812", SYMLINK+="usb/alfa8812"
```

Caveats

- Kernel updates may require `dkms autoinstall` (the package handles this automatically).
- For production‑grade reliability, consider an in‑kernel Alfa like ACHM/ACM instead.

