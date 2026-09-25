#!/usr/bin/env python3
"""Request a reproducible Town04 lanelet route and await a real trajectory.

Use inside the planning container after the CARLA bridge and odom-to-TF are
running. The goal is on lanelet 21693, ~150 m down its centerline from spawn
184 (map coordinates with Local projection and inverted CARLA y). This calls
the *mission planner's* service, not the optional ADAPI/vehicle interface.
Failure is nonzero; run-loop must not start the SI follower on an unready
route or a made-up trajectory.
"""

import math
import time

import rclpy
from autoware_planning_msgs.msg import Trajectory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tier4_planning_msgs.srv import SetWaypointRoute


GOAL_X = -460.1
GOAL_Y = -376.6
GOAL_YAW_DEG = -45.9
ROUTE_SERVICE = "/planning/mission_planning/mission_planner/set_waypoint_route"
TRAJECTORY_TOPIC = "/planning/scenario_planning/trajectory"


class RouteProbe(Node):
    def __init__(self):
        super().__init__(
            "town04_route_probe",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.odom = None
        self.trajectory = None
        self.create_subscription(
            Odometry, "/localization/kinematic_state", self._on_odom,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE),
        )
        self.create_subscription(Trajectory, TRAJECTORY_TOPIC, self._on_trajectory, 10)
        self.route_client = self.create_client(SetWaypointRoute, ROUTE_SERVICE)

    def _on_odom(self, msg):
        self.odom = msg

    def _on_trajectory(self, msg):
        if len(msg.points) >= 5:
            self.trajectory = msg


def main():
    rclpy.init()
    node = RouteProbe()
    try:
        deadline = time.monotonic() + 180.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.odom is not None and node.get_clock().now().nanoseconds > 0 \
                    and node.route_client.service_is_ready():
                break
        else:
            raise RuntimeError(
                "route prerequisites unavailable: odom=%s sim_time_ns=%d service=%s"
                % (node.odom is not None, node.get_clock().now().nanoseconds,
                   node.route_client.service_is_ready())
            )

        # The route planner must have loaded the Lanelet2 map before it can
        # answer; retry explicit unready responses (never publish a synthetic
        # candidate). A service success is not sufficient: wait for a fresh
        # nonempty planning output before starting SI.
        while time.monotonic() < deadline:
            req = SetWaypointRoute.Request()
            req.header.frame_id = "map"
            req.header.stamp = node.get_clock().now().to_msg()
            req.goal_pose.position.x = GOAL_X
            req.goal_pose.position.y = GOAL_Y
            req.goal_pose.orientation.z = math.sin(math.radians(GOAL_YAW_DEG) / 2.0)
            req.goal_pose.orientation.w = math.cos(math.radians(GOAL_YAW_DEG) / 2.0)
            req.allow_modification = False
            future = node.route_client.call_async(req)
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
            if not future.done():
                break
            response = future.result()
            if response is not None and response.status.success:
                node.get_logger().info(
                    "Town04 route accepted: goal=(%.1f, %.1f), yaw=%.1f deg"
                    % (GOAL_X, GOAL_Y, GOAL_YAW_DEG)
                )
                break
            status = response.status if response is not None else None
            node.get_logger().warn("route not ready: %s" % (status,))
            until = time.monotonic() + 2.0
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.2)
        else:
            raise RuntimeError("mission planner never accepted Town04 route")
        if not future.done():
            raise RuntimeError("mission planner route request timed out")

        node.trajectory = None  # Only accept a trajectory after route success.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.trajectory is not None:
                node.get_logger().info(
                    "Autoware trajectory ready: %d points, source stamp %d.%09d"
                    % (len(node.trajectory.points), node.trajectory.header.stamp.sec,
                       node.trajectory.header.stamp.nanosec)
                )
                return
        raise RuntimeError("no nonempty Autoware trajectory after route acceptance")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
