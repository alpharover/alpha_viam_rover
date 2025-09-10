#!/usr/bin/env python3
from __future__ import annotations

import sys
import time
import signal
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    import pigpio  # type: ignore
except Exception as e:  # pragma: no cover
    pigpio = None


@dataclass
class Pins:
    left_pwm: int = 26
    left_in1: int = 19
    left_in2: int = 13
    right_pwm: int = 22
    right_in3: int = 6
    right_in4: int = 5


class L298NDirect(Node):
    def __init__(self) -> None:
        super().__init__("l298n_direct")
        if pigpio is None:
            raise RuntimeError("pigpio python module not available")

        # Params
        self.pins = Pins(
            left_pwm=self.declare_parameter("left_pwm", 26).get_parameter_value().integer_value,
            left_in1=self.declare_parameter("left_in1", 19).get_parameter_value().integer_value,
            left_in2=self.declare_parameter("left_in2", 13).get_parameter_value().integer_value,
            right_pwm=self.declare_parameter("right_pwm", 22).get_parameter_value().integer_value,
            right_in3=self.declare_parameter("right_in3", 6).get_parameter_value().integer_value,
            right_in4=self.declare_parameter("right_in4", 5).get_parameter_value().integer_value,
        )
        self.freq = int(self.declare_parameter("pwm_freq", 20000).get_parameter_value().integer_value)
        self.range = int(self.declare_parameter("pwm_range", 255).get_parameter_value().integer_value)
        self.deadband = float(self.declare_parameter("deadband_mps", 0.05).get_parameter_value().double_value)
        self.max_mps = float(self.declare_parameter("max_linear_mps", 1.2).get_parameter_value().double_value)
        self.invert_left = bool(self.declare_parameter("invert_left", False).get_parameter_value().bool_value)
        self.invert_right = bool(self.declare_parameter("invert_right", False).get_parameter_value().bool_value)
        self.watchdog_s = float(self.declare_parameter("watchdog_s", 0.5).get_parameter_value().double_value)

        # Guard: refuse to run if controller_manager is active (avoid GPIO contention)
        from subprocess import run, PIPE

        proc = run(["ros2", "service", "list"], stdout=PIPE, stderr=PIPE, text=True)
        if "/controller_manager" in proc.stdout:
            raise RuntimeError("controller_manager detected; stop ROS control before l298n_direct")

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon not running. Start pigpiod.")

        for p in (self.pins.left_in1, self.pins.left_in2, self.pins.right_in3, self.pins.right_in4):
            self.pi.set_mode(p, pigpio.OUTPUT)
            self.pi.write(p, 0)
        for p in (self.pins.left_pwm, self.pins.right_pwm):
            self.pi.set_mode(p, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(p, self.freq)
            self.pi.set_PWM_range(p, self.range)
            self.pi.set_PWM_dutycycle(p, 0)

        self.last_cmd_time = time.monotonic()
        self.sub = self.create_subscription(Twist, "/cmd_vel", self.cb_twist, 10)
        self.timer = self.create_timer(0.05, self.on_timer)

        # Ensure we stop on SIGINT
        signal.signal(signal.SIGINT, lambda *_: self.shutdown())

    def set_motor(self, left: float, right: float) -> None:
        # left/right in m/s
        def drive(pwm_pin: int, a: int, b: int, v: float, invert: bool) -> None:
            sign = -1.0 if invert else 1.0
            v *= sign
            if abs(v) < self.deadband:
                self.pi.write(a, 0); self.pi.write(b, 0)
                self.pi.set_PWM_dutycycle(pwm_pin, 0)
                return
            mag = min(abs(v) / max(1e-6, self.max_mps), 1.0)
            duty = int(round(mag * self.range))
            if v >= 0:
                self.pi.write(a, 1); self.pi.write(b, 0)
            else:
                self.pi.write(a, 0); self.pi.write(b, 1)
            self.pi.set_PWM_dutycycle(pwm_pin, duty)

        drive(self.pins.left_pwm, self.pins.left_in1, self.pins.left_in2, left, self.invert_left)
        drive(self.pins.right_pwm, self.pins.right_in3, self.pins.right_in4, right, self.invert_right)

    def cb_twist(self, msg: Twist) -> None:
        v = float(msg.linear.x)
        self.last_cmd_time = time.monotonic()
        self.set_motor(v, v)

    def on_timer(self) -> None:
        if time.monotonic() - self.last_cmd_time > self.watchdog_s:
            self.set_motor(0.0, 0.0)

    def shutdown(self) -> None:
        try:
            self.set_motor(0.0, 0.0)
        finally:
            if self.pi is not None:
                self.pi.stop()


def main() -> int:
    rclpy.init()
    node = L298NDirect()
    try:
        rclpy.spin(node)
        return 0
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())

