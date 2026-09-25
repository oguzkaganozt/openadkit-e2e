#!/usr/bin/env python3
"""Read-only ROS-domain-1 publisher audit for the Autoware→SI run.

Run inside the autoware-planning container after run-loop starts SI. This
checks candidate provenance and absence of an Autoware control publisher;
it does NOT claim the legacy CARLA bridge is already ApprovedRequest-only.
"""

import json
import time

import rclpy
from rclpy.node import Node


TOPICS = (
    "/planning/scenario_planning/trajectory",
    "/vehicle/driving_reference",
    "/perception/object_recognition/objects",
    "/control/trajectory_follower/control_cmd",
    "/control/command/control_cmd",
    "/control/command/actuation_cmd",
)


def main() -> None:
    rclpy.init()
    node = Node("autoware_rig_graph_audit")
    try:
        # DDS discovery can lag the container startup; let the graph settle.
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.get_publishers_info_by_topic(TOPICS[0]):
                break

        publishers = {
            topic: sorted(
                f"{endpoint.node_namespace.rstrip('/')}/{endpoint.node_name}"
                for endpoint in node.get_publishers_info_by_topic(topic)
            ) for topic in TOPICS
        }
        print(json.dumps({"domain": 1, "publishers": publishers}, indent=2), flush=True)

        candidate = publishers[TOPICS[0]]
        if len(candidate) != 1 or not candidate[0].startswith("/planning/"):
            raise RuntimeError(f"expected one Autoware trajectory writer, got {candidate}")
        if publishers["/vehicle/driving_reference"]:
            raise RuntimeError("VisionPilot reference is still being published")
        if publishers["/perception/object_recognition/objects"] != ["/empty_scene_fixture"]:
            raise RuntimeError("empty-world fixture absent or multiple perception writers")
        for topic in TOPICS[3:]:
            if any(not writer.startswith("/safety_island_bridge_")
                   for writer in publishers[topic]):
                raise RuntimeError(f"unexpected control writer on {topic}: {publishers[topic]}")
        if len(publishers["/control/trajectory_follower/control_cmd"]) != 1:
            raise RuntimeError("legacy SI bridge output not found on domain 1")
        print("Autoware candidate isolation: PASS", flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
