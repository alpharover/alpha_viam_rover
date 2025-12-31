#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


def main() -> int:
    parser = argparse.ArgumentParser(description="Publish Twist/TwistStamped bursts")
    parser.add_argument("--topic", required=True)
    parser.add_argument("--duration", type=float, default=1.5)
    parser.add_argument("--rate", type=float, default=15.0)
    parser.add_argument("--linear_x", type=float, default=0.6)
    parser.add_argument("--stamped", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = Node("twist_burst_pub")
    if args.stamped:
        pub = node.create_publisher(TwistStamped, args.topic, 10)
    else:
        pub = node.create_publisher(Twist, args.topic, 10)

    period = 1.0 / max(1e-3, args.rate)
    end = time.monotonic() + max(0.0, args.duration)

    try:
        while rclpy.ok() and time.monotonic() < end:
            if args.stamped:
                msg = TwistStamped()
                msg.twist.linear.x = float(args.linear_x)
                pub.publish(msg)
            else:
                msg = Twist()
                msg.linear.x = float(args.linear_x)
                pub.publish(msg)
            time.sleep(period)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
