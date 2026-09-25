#!/usr/bin/env python3
"""Observe both trajectory publishers and the selected SI output on domain 2.

Run after *both* publishers start on one fresh CARLA rig. This checks the
cross-DDS candidate wire identity; the selected-source-loss fault injection
is a separate step in the same clean attempt.
"""

import json
import os
import time

import rclpy
from autoware_planning_msgs.msg import Trajectory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from safety_island_msgs.msg import ApprovedRequest, TrajectoryCandidate


class IsolationProbe(Node):
    def __init__(self):
        super().__init__("candidate_isolation_probe")
        qos = QoSProfile(
            depth=50,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.vp = 0
        self.autoware = 0
        self.vp_ids = set()
        self.normal_ids = set()
        self.normal = 0
        self.stops = 0
        self.stop_ids = set()
        self.unexpected_modes = 0
        self.create_subscription(
            TrajectoryCandidate, "/planning/visionpilot/trajectory_candidate",
            self.on_vp, qos
        )
        self.create_subscription(
            Trajectory, "/planning/scenario_planning/trajectory", self.on_autoware, qos
        )
        self.create_subscription(
            ApprovedRequest, "/control/safety_island/approved_request",
            self.on_approved, qos
        )

    def on_vp(self, msg):
        self.vp += 1
        self.vp_ids.add((int(msg.source_session), int(msg.source_cycle)))

    def on_autoware(self, msg):
        self.autoware += 1

    def on_approved(self, msg):
        if int(msg.mode) != 0 or int(msg.selected_source) != 0:
            self.unexpected_modes += 1
        if int(msg.decision) == 0:
            self.normal += 1
            self.normal_ids.add((int(msg.source_session), int(msg.source_cycle)))
        elif int(msg.decision) == 1:
            self.stops += 1
            self.stop_ids.add((int(msg.source_session), int(msg.source_cycle)))


def main():
    expected = os.environ.get("EXPECTED_SOURCE", "")
    if expected not in ("vp", "autoware"):
        raise SystemExit("EXPECTED_SOURCE must be vp or autoware")
    remaining = os.environ.get("EXPECT_REMAINING", "")
    if remaining and (remaining not in ("vp", "autoware") or remaining == expected):
        raise SystemExit("EXPECT_REMAINING must be the other publisher")
    duration = float(os.environ.get("RUN_SEC", "20"))
    rclpy.init()
    node = IsolationProbe()
    until = time.monotonic() + duration
    try:
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.2)
        result = {
            "expected_source": expected,
            "vp_candidates": node.vp,
            "autoware_trajectories": node.autoware,
            "normal_approved": node.normal,
            "stop_approved": node.stops,
            "unexpected_modes": node.unexpected_modes,
            "normal_source_ids": sorted(node.normal_ids)[:8],
            "stop_source_ids": sorted(node.stop_ids)[:8],
            "vp_normal_matches": len(node.normal_ids & node.vp_ids),
        }
        print(json.dumps(result), flush=True)
        if remaining:
            other_count = node.vp if remaining == "vp" else node.autoware
            if other_count < 3 or node.stops < 3 or node.normal or node.unexpected_modes:
                raise SystemExit("FAIL: unselected publisher must remain live while SI_STOP stays latched")
            if expected == "autoware" and node.stop_ids != {(0, 0)}:
                raise SystemExit("FAIL: stopped Autoware selection changed identity")
            if expected == "vp" and (0, 0) in node.stop_ids:
                raise SystemExit("FAIL: stopped VP selection lost its identity")
            print("selected-source loss isolation PASS", flush=True)
            return
        if (node.vp < 3 or node.autoware < 3 or node.normal < 3 or
                node.stops or node.unexpected_modes):
            raise SystemExit("FAIL: both inputs and uninterrupted NORMAL SI output required")
        if expected == "vp":
            if (0, 0) in node.normal_ids or not (node.normal_ids & node.vp_ids):
                raise SystemExit("FAIL: NORMAL SI output lacks matched VP identity")
        elif node.normal_ids != {(0, 0)}:
            raise SystemExit("FAIL: Autoware output carried an unselected VP identity")
        print("candidate isolation probe PASS", flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
