from __future__ import annotations

import math
import time
from typing import Tuple

try:
    # Prefer smbus2 if present
    from smbus2 import SMBus  # type: ignore
except Exception:  # pragma: no cover - fallback for minimal installs
    from smbus import SMBus  # type: ignore

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


# MPU-6050 registers
_REG_PWR_MGMT_1 = 0x6B
_REG_SMPLRT_DIV = 0x19
_REG_CONFIG = 0x1A
_REG_GYRO_CONFIG = 0x1B
_REG_ACCEL_CONFIG = 0x1C
_REG_ACCEL_XOUT_H = 0x3B


def _twos(val: int, bits: int) -> int:
    if val & (1 << (bits - 1)):
        val -= 1 << bits
    return val


def _accel_lsb_per_g(range_g: int) -> float:
    # 2g=16384, 4g=8192, 8g=4096, 16g=2048
    return {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}.get(range_g, 16384.0)


def _gyro_lsb_per_dps(range_dps: int) -> float:
    # 250=131, 500=65.5, 1000=32.8, 2000=16.4
    return {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}.get(range_dps, 131.0)


class Mpu6050Node(Node):
    def __init__(self) -> None:
        super().__init__("mpu6050_node")

        # Parameters
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("address", 0x68)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("accel_range_g", 2)  # 2,4,8,16
        self.declare_parameter("gyro_range_dps", 250)  # 250,500,1000,2000
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("calibrate_gyro", True)
        self.declare_parameter("calib_samples", 500)

        bus_no = int(self.get_parameter("i2c_bus").value)
        addr_param = self.get_parameter("address").value
        addr = int(addr_param, 0) if isinstance(addr_param, str) else int(addr_param)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._rate_hz = float(self.get_parameter("rate_hz").value)
        self._period = max(0.001, 1.0 / self._rate_hz)
        self._accel_range = int(self.get_parameter("accel_range_g").value)
        self._gyro_range = int(self.get_parameter("gyro_range_dps").value)
        imu_topic = str(self.get_parameter("imu_topic").value)
        self._calib_gyro = bool(self.get_parameter("calibrate_gyro").value)
        self._calib_samples = int(self.get_parameter("calib_samples").value)

        self._bus = None  # type: ignore[assignment]
        try:
            self._bus = SMBus(bus_no)
        except Exception as e:  # pragma: no cover
            self.get_logger().error(f"Failed to open I2C bus {bus_no}: {e}")
            raise
        self._addr = addr

        # Initialize device
        self._init_device()

        # Calibration (gyro bias only)
        self._gyro_bias = (0.0, 0.0, 0.0)
        if self._calib_gyro:
            try:
                self._gyro_bias = self._calibrate_gyro(self._calib_samples)
                self.get_logger().info(
                    f"Gyro bias (rad/s): {self._gyro_bias[0]:.5f}, {self._gyro_bias[1]:.5f}, {self._gyro_bias[2]:.5f}"
                )
            except Exception as e:
                self.get_logger().warn(f"Gyro calibration failed: {e}")

        # Publisher and timer
        self._pub_imu = self.create_publisher(Imu, imu_topic, 20)
        self._timer = self.create_timer(self._period, self._tick)
        self.get_logger().info(
            "MPU-6050 started on i2c-%d addr 0x%02x, accel=%dg, gyro=%d dps, rate=%.1f Hz"
            % (bus_no, addr, self._accel_range, self._gyro_range, self._rate_hz)
        )

    # Low-level I2C helpers
    def _write_u8(self, reg: int, val: int) -> None:
        assert self._bus is not None
        self._bus.write_byte_data(self._addr, reg, val & 0xFF)

    def _read_block(self, reg: int, length: int) -> list[int]:
        assert self._bus is not None
        return self._bus.read_i2c_block_data(self._addr, reg, length)

    def _init_device(self) -> None:
        # Wake up device and set clock source
        # Write 0x01: clock source = PLL with X axis gyroscope reference
        self._write_u8(_REG_PWR_MGMT_1, 0x01)
        time.sleep(0.05)

        # Set DLPF to ~44 Hz for accel/gyro (CONFIG=3)
        self._write_u8(_REG_CONFIG, 0x03)
        time.sleep(0.01)

        # Set gyro full-scale range
        gyro_fs_sel = {250: 0, 500: 1, 1000: 2, 2000: 3}.get(self._gyro_range, 0)
        self._write_u8(_REG_GYRO_CONFIG, (gyro_fs_sel & 0x03) << 3)

        # Set accel full-scale range
        accel_fs_sel = {2: 0, 4: 1, 8: 2, 16: 3}.get(self._accel_range, 0)
        self._write_u8(_REG_ACCEL_CONFIG, (accel_fs_sel & 0x03) << 3)

        # Use SMPLRT_DIV to optionally reduce internal sample rate; set to 0 for 1kHz/(1+0)=1kHz base
        # We will control ROS publish rate via timer; keep internal sampling fast.
        self._write_u8(_REG_SMPLRT_DIV, 0x00)

    def _calibrate_gyro(self, samples: int) -> Tuple[float, float, float]:
        gx_off = 0.0
        gy_off = 0.0
        gz_off = 0.0
        lsb_per_dps = _gyro_lsb_per_dps(self._gyro_range)
        for i in range(max(1, samples)):
            ax, ay, az, gx, gy, gz, _ = self._read_raw()
            gx_off += gx
            gy_off += gy
            gz_off += gz
            time.sleep(0.002)
        gx_off /= samples
        gy_off /= samples
        gz_off /= samples
        # Convert to rad/s
        gx_bias = (gx_off / lsb_per_dps) * (math.pi / 180.0)
        gy_bias = (gy_off / lsb_per_dps) * (math.pi / 180.0)
        gz_bias = (gz_off / lsb_per_dps) * (math.pi / 180.0)
        return (gx_bias, gy_bias, gz_bias)

    def _read_raw(self) -> Tuple[int, int, int, int, int, int, float]:
        # Read 14 bytes: accel(6), temp(2), gyro(6)
        data = self._read_block(_REG_ACCEL_XOUT_H, 14)
        ax = _twos((data[0] << 8) | data[1], 16)
        ay = _twos((data[2] << 8) | data[3], 16)
        az = _twos((data[4] << 8) | data[5], 16)
        temp_raw = _twos((data[6] << 8) | data[7], 16)
        gx = _twos((data[8] << 8) | data[9], 16)
        gy = _twos((data[10] << 8) | data[11], 16)
        gz = _twos((data[12] << 8) | data[13], 16)
        # Convert temperature per datasheet
        temp_c = (temp_raw / 340.0) + 36.53
        return ax, ay, az, gx, gy, gz, temp_c

    def _tick(self) -> None:
        try:
            ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, _ = self._read_raw()

            # Scales
            acc_lsb_g = _accel_lsb_per_g(self._accel_range)
            gyr_lsb_dps = _gyro_lsb_per_dps(self._gyro_range)

            # Convert
            ax = (ax_raw / acc_lsb_g) * 9.80665
            ay = (ay_raw / acc_lsb_g) * 9.80665
            az = (az_raw / acc_lsb_g) * 9.80665

            gx = (gx_raw / gyr_lsb_dps) * (math.pi / 180.0)
            gy = (gy_raw / gyr_lsb_dps) * (math.pi / 180.0)
            gz = (gz_raw / gyr_lsb_dps) * (math.pi / 180.0)

            # Apply gyro bias
            gx -= self._gyro_bias[0]
            gy -= self._gyro_bias[1]
            gz -= self._gyro_bias[2]

            # Compose Imu message
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._frame_id

            # Orientation unknown; use identity quaternion and set covariance[0] = -1
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.0
            msg.orientation.w = 1.0
            msg.orientation_covariance[0] = -1.0

            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz
            # Set conservative fixed covariance for gyro
            msg.angular_velocity_covariance[0] = 0.02
            msg.angular_velocity_covariance[4] = 0.02
            msg.angular_velocity_covariance[8] = 0.02

            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az
            # Conservative fixed covariance for accel
            msg.linear_acceleration_covariance[0] = 0.04
            msg.linear_acceleration_covariance[4] = 0.04
            msg.linear_acceleration_covariance[8] = 0.04

            self._pub_imu.publish(msg)
        except Exception as e:
            if not hasattr(self, "_last_err"):
                self._last_err = 0.0  # type: ignore[attr-defined]
            now = time.monotonic()
            if now - self._last_err > 5.0:  # type: ignore[attr-defined]
                self.get_logger().warn(f"MPU-6050 read error: {e}")
                self._last_err = now  # type: ignore[attr-defined]


def main() -> None:  # pragma: no cover
    rclpy.init()
    node = Mpu6050Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
