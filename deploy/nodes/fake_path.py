#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class FakePath(Node):
    def __init__(self):
        super().__init__("fake_path")
        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(Path, "/vehicle/lane_path", qos)
        self.create_timer(0.1, self._tick)

    def _tick(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        for i in range(13):
            x = float(i) * 2.0
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakePath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
