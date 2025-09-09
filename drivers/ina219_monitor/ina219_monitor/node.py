from __future__ import annotations

import time

try:
    # Prefer smbus2 if present
    from smbus2 import SMBus  # type: ignore
except Exception:  # pragma: no cover - fallback for minimal installs
    from smbus import SMBus  # type: ignore

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


_REG_SHUNT_VOLT = 0x01
_REG_BUS_VOLT = 0x02


def _twos_complement(val: int, bits: int) -> int:
    if val & (1 << (bits - 1)):
        val -= 1 << bits
    return val


class INA219Monitor(Node):
    def __init__(self) -> None:
        super().__init__("ina219_monitor")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("address", 0x40)
        self.declare_parameter("shunt_ohms", 0.1)
        self.declare_parameter("frame_id", "power")
        self.declare_parameter("bus_voltage_topic", "/power/bus_voltage")
        self.declare_parameter("current_topic", "/power/current")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("power_topic", "/power/power")

        bus_no = int(self.get_parameter("i2c_bus").value)
        addr = int(self.get_parameter("address").value)
        self._shunt_ohms = float(self.get_parameter("shunt_ohms").value)
        bus_topic = str(self.get_parameter("bus_voltage_topic").value)
        cur_topic = str(self.get_parameter("current_topic").value)
        rate_hz = float(self.get_parameter("rate_hz").value)
        power_topic = str(self.get_parameter("power_topic").value)
        self._period = max(0.01, 1.0 / rate_hz)

        self._bus = None  # type: ignore[assignment]
        try:
            self._bus = SMBus(bus_no)
        except Exception as e:  # pragma: no cover
            self.get_logger().error(f"Failed to open I2C bus {bus_no}: {e}")
            raise
        self._addr = addr

        self._pub_bus = self.create_publisher(Float32, bus_topic, 10)
        self._pub_cur = self.create_publisher(Float32, cur_topic, 10)
        self._pub_pow = self.create_publisher(Float32, power_topic, 10)

        self._timer = self.create_timer(self._period, self._tick)
        self.get_logger().info(
            f"INA219 monitor started on i2c-{bus_no} addr 0x{addr:02x}, shunt={self._shunt_ohms}Ω, rate={rate_hz:.1f} Hz"
        )

    def _read_u16(self, reg: int) -> int:
        assert self._bus is not None
        data = self._bus.read_i2c_block_data(self._addr, reg, 2)
        return (data[0] << 8) | data[1]

    def _tick(self) -> None:
        try:
            raw_bus = self._read_u16(_REG_BUS_VOLT)
            # Bus voltage: bits 15..3 valid, LSB=4mV
            bus_mv = ((raw_bus >> 3) & 0x1FFF) * 4.0
            v_bus = bus_mv / 1000.0

            raw_shunt = self._read_u16(_REG_SHUNT_VOLT)
            raw_shunt = _twos_complement(raw_shunt, 16)
            # Shunt LSB is 10 uV
            v_shunt = raw_shunt * 10e-6
            current = v_shunt / self._shunt_ohms

            self._pub_bus.publish(Float32(data=float(v_bus)))
            self._pub_cur.publish(Float32(data=float(current)))
            self._pub_pow.publish(Float32(data=float(v_bus * current)))
        except Exception as e:
            # Log at throttled rate to avoid floods
            if not hasattr(self, "_last_err"):
                self._last_err = 0.0  # type: ignore[attr-defined]
            now = time.monotonic()
            if now - self._last_err > 5.0:  # type: ignore[attr-defined]
                self.get_logger().warn(f"INA219 read error: {e}")
                self._last_err = now  # type: ignore[attr-defined]


def main() -> None:  # pragma: no cover
    rclpy.init()
    node = INA219Monitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
