#!/usr/bin/env python3
"""Stub /system/operation_mode/state so the SI follower is under control."""

import rclpy
from autoware_adapi_v1_msgs.msg import OperationModeState
from rclpy.node import Node

AUTONOMOUS = 2


class OperationModeStub(Node):
    def __init__(self):
        super().__init__("operation_mode_stub")
        self.pub = self.create_publisher(
            OperationModeState, "/system/operation_mode/state", 1
        )
        self.create_timer(0.1, self._tick)

    def _tick(self):
        msg = OperationModeState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mode = AUTONOMOUS
        msg.is_autoware_control_enabled = True
        msg.is_in_transition = False
        msg.is_stop_mode_available = True
        msg.is_autonomous_mode_available = True
        msg.is_local_mode_available = True
        msg.is_remote_mode_available = True
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = OperationModeStub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
