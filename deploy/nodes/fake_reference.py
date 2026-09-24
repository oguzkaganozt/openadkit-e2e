#!/usr/bin/env python3
"""Synthetic VP reference for adapter/SI testing without VisionPilot.

Publishes a straight-ahead DrivingReference stamped with the latest CARLA
ego stamp from /localization/kinematic_state, so the adapter's exact
same-frame match succeeds. Run it in the adapter image (which carries
visionpilot_msgs) with ROS_DOMAIN_ID=1:

    python3 deploy/nodes/fake_reference.py
"""
import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visionpilot_msgs.msg import DrivingReference

HORIZON_LEN = 20
HORIZON_DT_S = 0.05
CRUISE_MPS = 3.0
PATH_X_MAX_M = 30.0


class FakeReference(Node):
    def __init__(self):
        super().__init__("fake_reference")
        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._source_stamp = None
        self._cycle = 0
        self.pub = self.create_publisher(
            DrivingReference, "/vehicle/driving_reference", qos
        )
        self.create_subscription(
            Odometry, "/localization/kinematic_state", self._on_odom, qos
        )
        self.create_timer(0.1, self._tick)

    def _on_odom(self, msg: Odometry) -> None:
        self._source_stamp = Time(
            sec=msg.header.stamp.sec, nanosec=msg.header.stamp.nanosec
        )

    def _tick(self) -> None:
        if self._source_stamp is None:
            return
        msg = DrivingReference()
        msg.stamp = self.get_clock().now().to_msg()
        msg.source_stamp = self._source_stamp
        msg.has_source_stamp = True
        msg.session = 1
        msg.cycle = self._cycle
        self._cycle += 1
        msg.valid = True
        msg.path_valid = True
        msg.path_a = 0.0
        msg.path_b = 0.0
        msg.path_c = 0.0
        msg.path_x_max_m = PATH_X_MAX_M
        msg.horizon_dt_s = HORIZON_DT_S
        msg.speed_horizon_mps = [CRUISE_MPS] * HORIZON_LEN
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeReference()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()