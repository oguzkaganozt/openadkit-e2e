#!/usr/bin/env python3
import json
import socket

import rclpy
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class PathRelayTx(Node):
    def __init__(self):
        super().__init__("path_relay_tx")
        self.declare_parameter("udp_host", "127.0.0.1")
        self.declare_parameter("udp_port", 17501)
        host = self.get_parameter("udp_host").value
        port = int(self.get_parameter("udp_port").value)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._dest = (host, port)
        self._count = 0
        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Path, "/vehicle/lane_path", self._on_path, qos)
        self.get_logger().info("path_relay_tx -> %s:%s" % (host, port))

    def _on_path(self, msg: Path):
        payload = {
            "stamp_sec": int(msg.header.stamp.sec),
            "stamp_nsec": int(msg.header.stamp.nanosec),
            "frame_id": msg.header.frame_id,
            "poses": [
                {
                    "x": p.pose.position.x,
                    "y": p.pose.position.y,
                    "z": p.pose.position.z,
                    "qx": p.pose.orientation.x,
                    "qy": p.pose.orientation.y,
                    "qz": p.pose.orientation.z,
                    "qw": p.pose.orientation.w,
                }
                for p in msg.poses
            ],
        }
        self._sock.sendto(json.dumps(payload).encode("utf-8"), self._dest)
        self._count += 1
        if self._count == 1 or self._count % 100 == 0:
            if msg.poses:
                first = msg.poses[0].pose.position
                last = msg.poses[-1].pose.position
                self.get_logger().info(
                    "forwarded Path #%d (%d poses, first=(%.2f, %.2f), last=(%.2f, %.2f))"
                    % (
                        self._count,
                        len(msg.poses),
                        first.x,
                        first.y,
                        last.x,
                        last.y,
                    )
                )
            else:
                self.get_logger().info("forwarded Path #%d (empty)" % self._count)


def main():
    rclpy.init()
    node = PathRelayTx()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
