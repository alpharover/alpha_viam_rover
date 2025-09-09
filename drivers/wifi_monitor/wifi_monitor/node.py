from __future__ import annotations

import re
import shlex
import subprocess
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


def _run(cmd: str, timeout: float = 1.5) -> str:
    try:
        out = subprocess.check_output(shlex.split(cmd), timeout=timeout)
        return out.decode("utf-8", errors="replace")
    except Exception:
        return ""


class WifiMonitor(Node):
    def __init__(self) -> None:
        super().__init__("wifi_monitor")
        self.declare_parameter("iface", "wlan1")
        self.declare_parameter("rate_hz", 1.0)

        self._iface = str(self.get_parameter("iface").value)
        rate_hz = float(self.get_parameter("rate_hz").value)
        self._period = max(0.2, 1.0 / max(0.1, rate_hz))

        self._pub_rssi = self.create_publisher(Float32, "/wifi/signal_dBm", 10)
        self._pub_link = self.create_publisher(Int32, "/wifi/link_ok", 10)
        self._pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        self._timer = self.create_timer(self._period, self._tick)
        self.get_logger().info(f"wifi_monitor watching iface={self._iface} at {rate_hz:.1f} Hz")

    def _tick(self) -> None:
        iface = self._iface
        link_txt = _run(f"iw dev {shlex.quote(iface)} link")
        info_txt = _run(f"iw dev {shlex.quote(iface)} info")

        connected = "Not connected" not in link_txt and bool(link_txt.strip())
        rssi_dbm: Optional[float] = None
        ssid = ""
        bssid = ""
        txpower_dbm: Optional[float] = None

        # Parse signal and SSID/BSSID
        m = re.search(r"signal:\s*(-?\d+)\s*dBm", link_txt)
        if m:
            try:
                rssi_dbm = float(m.group(1))
            except Exception:
                rssi_dbm = None
        m = re.search(r"ssid\s+(.+)", link_txt)
        if m:
            ssid = m.group(1).strip()
        m = re.search(r"Connected to\s+([0-9a-fA-F:]{17})", link_txt)
        if m:
            bssid = m.group(1)

        # txpower
        m = re.search(r"txpower\s+(\d+(?:\.\d+)?)\s*dBm", info_txt)
        if m:
            try:
                txpower_dbm = float(m.group(1))
            except Exception:
                txpower_dbm = None

        # Publish simple topics
        self._pub_link.publish(Int32(data=1 if connected else 0))
        if rssi_dbm is not None:
            self._pub_rssi.publish(Float32(data=rssi_dbm))

        # Diagnostics
        status = DiagnosticStatus()
        status.name = f"wifi/{iface}"
        status.hardware_id = iface
        status.level = DiagnosticStatus.OK if connected else DiagnosticStatus.WARN
        status.message = "Connected" if connected else "Not connected"
        kvs: list[KeyValue] = []
        if rssi_dbm is not None:
            kv = KeyValue(key="signal_dBm", value=f"{rssi_dbm:.0f}")
            kvs.append(kv)
        if ssid:
            kvs.append(KeyValue(key="ssid", value=ssid))
        if bssid:
            kvs.append(KeyValue(key="bssid", value=bssid))
        if txpower_dbm is not None:
            kvs.append(KeyValue(key="txpower_dBm", value=f"{txpower_dbm:.1f}"))
        status.values = kvs

        arr = DiagnosticArray()
        arr.status = [status]
        self._pub_diag.publish(arr)


def main() -> None:  # pragma: no cover
    rclpy.init()
    node = WifiMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

