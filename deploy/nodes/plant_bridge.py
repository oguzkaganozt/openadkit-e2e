#!/usr/bin/env python3
"""CARLA plant ↔ SI topics. Does not subscribe to VP steering_cmd."""

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import SteeringReport
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64


class PlantBridge(Node):
    def __init__(self):
        super().__init__("plant_bridge")
        self.declare_parameter("carla_odom_topic", "/carla/hero/odometry")
        self.declare_parameter(
            "carla_control_topic", "/carla/hero/ackermann_control_cmd"
        )
        odom_topic = self.get_parameter("carla_odom_topic").value
        control_topic = self.get_parameter("carla_control_topic").value

        self.odom_pub = self.create_publisher(
            Odometry, "/localization/kinematic_state", 1
        )
        self.accel_pub = self.create_publisher(
            AccelWithCovarianceStamped, "/localization/acceleration", 1
        )
        self.steer_pub = self.create_publisher(
            SteeringReport, "/vehicle/status/steering_status", 1
        )
        self.speed_pub = self.create_publisher(Float64, "/vehicle/speed", 1)
        self.ackermann_pub = self.create_publisher(
            AckermannDriveStamped, control_topic, 1
        )

        self.create_subscription(Odometry, odom_topic, self._on_odom, 1)
        self.create_subscription(
            Control, "/control/trajectory_follower/control_cmd", self._on_control, 1
        )
        self._last_steer = 0.0

    def _on_odom(self, msg: Odometry):
        out = Odometry()
        out.header = msg.header
        if not out.header.frame_id:
            out.header.frame_id = "map"
        out.child_frame_id = msg.child_frame_id or "base_link"
        out.pose = msg.pose
        out.twist = msg.twist
        self.odom_pub.publish(out)

        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.speed_pub.publish(Float64(data=speed))

        accel = AccelWithCovarianceStamped()
        accel.header = out.header
        accel.accel.accel.linear.x = msg.twist.twist.linear.x
        self.accel_pub.publish(accel)

        steer = SteeringReport()
        steer.stamp = out.header.stamp
        steer.steering_tire_angle = self._last_steer
        self.steer_pub.publish(steer)

    def _on_control(self, msg: Control):
        self._last_steer = float(msg.lateral.steering_tire_angle)
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "hero"
        cmd.drive.steering_angle = self._last_steer
        cmd.drive.speed = float(msg.longitudinal.velocity)
        cmd.drive.acceleration = float(msg.longitudinal.acceleration)
        self.ackermann_pub.publish(cmd)


def main():
    rclpy.init()
    node = PlantBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
