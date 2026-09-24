#!/usr/bin/env python3
"""Publish map -> base_link TF from the bridged CARLA odometry.

Configuration 3 (Autoware Universe planning on the CARLA rig) needs a TF
tree; localization comes from CARLA ground truth, which the rig's bridge
publishes as Odometry in CARLA's inverted map frame (x, -y) -- the same
frame the y-inverted CARLA Lanelet2 maps use with
`map_projector_info.yaml: projector_type: Local`. This node turns each
odometry sample into the map->base_link transform, keeping the CARLA
simulation-time stamp so TF lookups work under use_sim_time.

Run in the Autoware image (tf2_ros present) on the rig's domain 1:

    python3 /opt/nodes/odom_to_tf.py
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    def __init__(self):
        super().__init__("odom_to_tf")
        qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, "/localization/kinematic_state", self._on_odom, qos
        )

    def _on_odom(self, msg: Odometry) -> None:
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = msg.header.frame_id or "map"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self._broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()