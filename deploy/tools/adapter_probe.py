#!/usr/bin/env python3
"""Validate the E2E adapter trajectory against the VP reference behind it.

Subscribes on ROS domain 1 to /vehicle/driving_reference (visionpilot_msgs)
and to the adapter's Autoware Trajectory, then reports periodically and at
exit:

- trajectory count and inter-arrival gaps (a silent window during a fault
  test shows up as a 10 s window with zero trajectories);
- how many trajectories carry a header stamp that was seen as a reference
  source_stamp (exact same-frame transcription);
- point-count range and velocity extrema; count of all-zero profiles (an
  adapter-authored stop would show up here; expected 0);
- reference count and session/cycle span.

Runs in an image carrying visionpilot_msgs and autoware_planning_msgs (the
adapter image); see deploy/tools/run-adapter-probe.sh.
"""
import json
import os
import time

import rclpy
from autoware_planning_msgs.msg import Trajectory
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from visionpilot_msgs.msg import DrivingReference

RUN_SEC = float(os.environ.get("RUN_SEC", "60"))


def stamp_key(stamp) -> tuple[int, int]:
    return (int(stamp.sec), int(stamp.nanosec))


class AdapterProbe(Node):
    def __init__(self):
        super().__init__("adapter_probe")
        qos = QoSProfile(
            depth=50,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.ref_stamps: dict[tuple[int, int], tuple[int, int]] = {}
        self.ref_order: list[tuple[int, int]] = []
        self.ref_count = 0
        self.sessions: set[int] = set()
        self.cycles: list[int] = []
        self.traj_count = 0
        self.traj_matched = 0
        self.traj_zero = 0
        self.pts_min = None
        self.pts_max = None
        self.v_min = None
        self.v_max = None
        self.window: list[float] = []
        self.last_traj_at = None
        self.max_gap_s = 0.0
        self.t0 = time.monotonic()
        self.done = False
        self.create_subscription(
            DrivingReference, "/vehicle/driving_reference", self.on_ref, qos
        )
        self.create_subscription(
            Trajectory,
            "/planning/scenario_planning/trajectory",
            self.on_traj,
            qos,
        )
        self.create_timer(10.0, self.report_window)

    def on_ref(self, msg: DrivingReference) -> None:
        self.ref_count += 1
        self.sessions.add(int(msg.session))
        self.cycles.append(int(msg.cycle))
        key = stamp_key(msg.source_stamp)
        if key not in self.ref_stamps:
            self.ref_order.append(key)
        self.ref_stamps[key] = (int(msg.session), int(msg.cycle))
        while len(self.ref_order) > 4000:
            self.ref_stamps.pop(self.ref_order.pop(0), None)

    def on_traj(self, msg: Trajectory) -> None:
        now = time.monotonic()
        self.traj_count += 1
        if self.last_traj_at is not None:
            self.max_gap_s = max(self.max_gap_s, now - self.last_traj_at)
        self.last_traj_at = now
        self.window.append(now)
        if stamp_key(msg.header.stamp) in self.ref_stamps:
            self.traj_matched += 1
        points = msg.points
        n = len(points)
        self.pts_min = n if self.pts_min is None else min(self.pts_min, n)
        self.pts_max = n if self.pts_max is None else max(self.pts_max, n)
        if points:
            vs = [float(p.longitudinal_velocity_mps) for p in points]
            self.v_min = min(vs) if self.v_min is None else min(self.v_min, min(vs))
            self.v_max = max(vs) if self.v_max is None else max(self.v_max, max(vs))
            if max(abs(v) for v in vs) < 1e-9:
                self.traj_zero += 1

    def report_window(self) -> None:
        if self.done:
            return
        now = time.monotonic()
        recent = [t for t in self.window if t >= now - 10.0]
        line = {
            "t_s": round(now - self.t0, 1),
            "traj_total": self.traj_count,
            "traj_last_10s": len(recent),
            "ref_total": self.ref_count,
            "matched": self.traj_matched,
            "zero_profiles": self.traj_zero,
            "max_gap_s": round(self.max_gap_s, 3),
        }
        print(json.dumps(line), flush=True)

    def summary(self) -> dict:
        return {
            "run_sec": round(time.monotonic() - self.t0, 1),
            "references": self.ref_count,
            "sessions": sorted(self.sessions),
            "cycle_min": min(self.cycles) if self.cycles else None,
            "cycle_max": max(self.cycles) if self.cycles else None,
            "trajectories": self.traj_count,
            "trajectory_stamp_seen_in_references": self.traj_matched,
            "zero_velocity_profiles": self.traj_zero,
            "points_min": self.pts_min,
            "points_max": self.pts_max,
            "velocity_min": self.v_min,
            "velocity_max": self.v_max,
            "max_gap_s": round(self.max_gap_s, 3),
        }


def main() -> None:
    rclpy.init()
    node = AdapterProbe()
    deadline = time.monotonic() + RUN_SEC
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.done = True
    print(json.dumps(node.summary()), flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()