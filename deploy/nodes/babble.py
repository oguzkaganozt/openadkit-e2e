#!/usr/bin/env python3
"""Spoof invalid follower Controls to test guard babble/invalid handling.

Publishes `count` Control messages with NaN velocity on the raw follower
topic (domain set by env) at `rate_hz`. The guard must answer EMERGENCY.
"""
import sys
import time

import rclpy
from autoware_control_msgs.msg import Control


def main() -> None:
    count = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    rate_hz = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    rclpy.init()
    node = rclpy.create_node("t_babble")
    pub = node.create_publisher(
        Control, "/control/trajectory_follower/control_cmd", 1
    )
    msg = Control()
    msg.longitudinal.velocity = float("nan")
    msg.longitudinal.acceleration = 0.0
    msg.lateral.steering_tire_angle = 0.0
    period = 1.0 / rate_hz
    for _ in range(count):
        msg.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        time.sleep(period)
    node.destroy_node()
    rclpy.shutdown()
    print("SPOOFED %d" % count)


if __name__ == "__main__":
    sys.exit(main())
