#!/usr/bin/env python3
"""Observe the CARLA camera image stream and report publication gaps.

Subscribes to the bridge's `/carla/hero/main_cam/image` with the same
reliable/keep-last QoS and logs every arrival gap above GAP_SEC, a 10 s
arrival-count window, and the full gap list at exit. This separates a
stalling image producer (bridge/DDS) from a stalling consumer
(VisionPilot): the rig host runs it on domain 1.

  ROS_DOMAIN_ID=1 python3 image_stream_observer.py
"""

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image

GAP_SEC = float(os.environ.get("GAP_SEC", "0.5"))
RUN_SEC = float(os.environ.get("RUN_SEC", "180"))
TOPIC = os.environ.get("IMAGE_TOPIC", "/carla/hero/main_cam/image")


class ImageStreamObserver(Node):
    def __init__(self):
        super().__init__("image_stream_observer")
        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.last = None
        self.count = 0
        self.gaps = []
        self.window_count = 0
        self.create_subscription(Image, TOPIC, self._on_image, qos)
        self.create_timer(10.0, self._window)

    def _on_image(self, msg):
        now = time.monotonic()
        if self.last is not None:
            gap = now - self.last
            if gap > GAP_SEC:
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                self.get_logger().warn(
                    "image gap %.2fs after #%d (sim stamp %.3f)"
                    % (gap, self.count, stamp)
                )
                self.gaps.append((round(gap, 2), round(time.time(), 3)))
        self.last = now
        self.count += 1
        self.window_count += 1

    def _window(self):
        self.get_logger().info("images in last 10s: %d" % self.window_count)
        self.window_count = 0


def main():
    rclpy.init()
    node = ImageStreamObserver()
    end = time.monotonic() + RUN_SEC
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.get_logger().info(
        "done: %d images, %d gaps > %.1fs" % (node.count, len(node.gaps), GAP_SEC)
    )
    print("IMAGE_GAPS: %s" % node.gaps, flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()