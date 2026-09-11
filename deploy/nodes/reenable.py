#!/usr/bin/env python3
"""One-shot re-enable pulse for the SI guard (domain set by env)."""
import sys
import time

import rclpy
from std_msgs.msg import Bool


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("t_reenable")
    pub = node.create_publisher(Bool, "/guard/re_enable", 1)
    time.sleep(1.0)
    pub.publish(Bool(data=True))
    time.sleep(1.0)
    node.destroy_node()
    rclpy.shutdown()
    print("PUBLISHED")


if __name__ == "__main__":
    sys.exit(main())
