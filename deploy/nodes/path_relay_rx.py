#!/usr/bin/env python3
import json
import socket

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class PathRelayRx(Node):
    def __init__(self):
        super().__init__("path_relay_rx")
        self.declare_parameter("udp_host", "127.0.0.1")
        self.declare_parameter("udp_port", 17501)
        host = self.get_parameter("udp_host").value
        port = int(self.get_parameter("udp_port").value)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((host, port))
        self._sock.setblocking(False)
        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(Path, "/vehicle/lane_path_relay", qos)
        self.create_timer(0.02, self._poll)
        self.get_logger().info("path_relay_rx <- %s:%s" % (host, port))

    def _poll(self):
        while True:
            try:
                data, _ = self._sock.recvfrom(65535)
            except BlockingIOError:
                return
            try:
                payload = json.loads(data.decode("utf-8"))
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue
            msg = Path()
            msg.header.stamp.sec = int(payload.get("stamp_sec", 0))
            msg.header.stamp.nanosec = int(payload.get("stamp_nsec", 0))
            msg.header.frame_id = str(payload.get("frame_id") or "base_link")
            for p in payload.get("poses", []):
                ps = PoseStamped()
                ps.header = msg.header
                ps.pose.position.x = float(p["x"])
                ps.pose.position.y = float(p["y"])
                ps.pose.position.z = float(p["z"])
                ps.pose.orientation.x = float(p["qx"])
                ps.pose.orientation.y = float(p["qy"])
                ps.pose.orientation.z = float(p["qz"])
                ps.pose.orientation.w = float(p["qw"])
                msg.poses.append(ps)
            self.pub.publish(msg)


def main():
    rclpy.init()
    node = PathRelayRx()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
