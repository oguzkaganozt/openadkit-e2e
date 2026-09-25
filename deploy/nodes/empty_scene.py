#!/usr/bin/env python3
"""Simulation-only empty perception feed for the lead-free Town04 rig.

Autoware's planning modules wait for objects, an occupancy grid, and point
cloud samples even when the scene is empty. This fixture only runs when the
scenario JSON declares no NPCs and no lead vehicle. All samples use the
bridged CARLA odometry frame's simulation timestamp. This is NOT a real
perception module; do not use it for runs with other road users.
"""

import json

import rclpy
from autoware_perception_msgs.msg import PredictedObjects
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


# Covers the bounded 150 m route from spawn 184 on Town04. Values are free
# only because the configured CARLA world contains no NPCs or lead vehicle.
GRID_WIDTH = 400
GRID_HEIGHT = 400
EMPTY_CLOUD_TOPICS = (
    "/perception/obstacle_segmentation/pointcloud",
    "/planning/scenario_planning/lane_driving/behavior_planning/compare_map_filtered/pointcloud",
    "/planning/scenario_planning/lane_driving/behavior_planning/vector_map_inside_area_filtered/pointcloud",
)


class EmptyScene(Node):
    def __init__(self):
        super().__init__("empty_scene_fixture")
        with open("/opt/config/carla-rig.json", encoding="utf-8") as config_file:
            config = json.load(config_file)
        if config.get("npc_vehicles") != [] or \
                config.get("lead_vehicle", {}).get("enabled") is not False:
            raise RuntimeError("empty-scene fixture requires no NPCs and no lead vehicle")
        self.pub = self.create_publisher(
            PredictedObjects, "/perception/object_recognition/objects", 1
        )
        self.grid_pub = self.create_publisher(
            OccupancyGrid, "/perception/occupancy_grid_map/map", 1
        )
        self.cloud_pubs = [
            self.create_publisher(PointCloud2, topic, 1) for topic in EMPTY_CLOUD_TOPICS
        ]
        self.grid = OccupancyGrid()
        self.grid.info.resolution = 1.0
        self.grid.info.width = GRID_WIDTH
        self.grid.info.height = GRID_HEIGHT
        self.grid.info.origin.position.x = -600.0
        self.grid.info.origin.position.y = -500.0
        self.grid.info.origin.orientation.w = 1.0
        self.grid.data = [0] * (GRID_WIDTH * GRID_HEIGHT)
        self.cloud = PointCloud2()
        self.cloud.height = 1
        self.cloud.width = 0
        self.cloud.point_step = 12
        self.cloud.row_step = 0
        self.cloud.is_dense = True
        self.cloud.fields = [
            PointField(name=name, offset=4 * index, datatype=PointField.FLOAT32, count=1)
            for index, name in enumerate(("x", "y", "z"))
        ]
        self.frame_count = 0
        self.create_subscription(
            Odometry, "/localization/kinematic_state", self._on_odom,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE),
        )
        self.get_logger().info("empty-scene fixture active (no NPCs or lead vehicle)")

    def _on_odom(self, msg: Odometry) -> None:
        self.frame_count += 1
        objects = PredictedObjects()
        objects.header = msg.header
        self.pub.publish(objects)
        if self.frame_count % 5 == 0:  # about 4 Hz
            self.cloud.header = msg.header
            for pub in self.cloud_pubs:
                pub.publish(self.cloud)
        if self.frame_count % 10 == 0:  # about 2 Hz, 160 kB per grid
            self.grid.header = msg.header
            self.grid.info.map_load_time = msg.header.stamp
            self.grid_pub.publish(self.grid)


def main() -> None:
    rclpy.init()
    node = EmptyScene()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
