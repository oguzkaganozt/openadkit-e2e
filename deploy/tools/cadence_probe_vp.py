#!/usr/bin/env python3
"""Measure the source-tagged VisionPilot outputs and bridge cadence.

Runs inside the VisionPilot image (ROS 2 Jazzy, has visionpilot_msgs):
records arrival times, rates and identity consistency of

  /vehicle/driving_command    visionpilot_msgs/DrivingCommand
  /vehicle/driving_reference  visionpilot_msgs/DrivingReference
  /vehicle/steering_cmd       std_msgs/Float64      (legacy, unchanged)
  /vehicle/throttle_cmd       std_msgs/Float64      (legacy, unchanged)
  /carla/hero/main_cam/image  sensor_msgs/Image     (bridge)
  /localization/kinematic_state / acceleration, /vehicle/speed (bridge)

Checks that matter for SI #62:
- command/reference share one cycle and one source_stamp,
- source_stamp equals a camera capture stamp seen by the probe,
- session is constant and cycle strictly increases,
- speed horizon length and step.
"""

import json
import os
import time

import rclpy
from nav_msgs.msg import Odometry, Path  # noqa: F401 (Path kept for compatibility runs)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

from visionpilot_msgs.msg import DrivingCommand, DrivingReference

TOPICS = {
    "/carla/hero/main_cam/image": Image,
    "/localization/kinematic_state": Odometry,
    "/vehicle/speed": Float64,
    "/vehicle/steering_cmd": Float64,
    "/vehicle/throttle_cmd": Float64,
    "/vehicle/driving_command": DrivingCommand,
    "/vehicle/driving_reference": DrivingReference,
}


def _percentile(values, fraction):
    if not values:
        return None
    values = sorted(values)
    if len(values) == 1:
        return values[0]
    index = fraction * (len(values) - 1)
    low = int(index)
    high = min(low + 1, len(values) - 1)
    weight = index - low
    return values[low] * (1 - weight) + values[high] * weight


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class Probe(Node):
    def __init__(self):
        super().__init__("cadence_probe_vp")
        qos = QoSProfile(
            depth=200,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.samples = {topic: [] for topic in TOPICS}
        self.commands = []
        self.references = []
        self.camera_stamps = []
        self.ego_stamps = []
        self.last_cycle = None
        self.cycle_anomalies = 0
        self.sessions = set()
        self.t0 = time.monotonic()

        for topic, message_type in TOPICS.items():
            self.create_subscription(
                message_type, topic, self._make_callback(topic), qos
            )

    def _record(self, topic, wall, stamp=None):
        self.samples[topic].append((time.monotonic(), wall, stamp))

    def _make_callback(self, topic):
        def callback(msg):
            wall = time.time()
            if topic == "/vehicle/driving_command":
                stamp = _stamp_seconds(msg.stamp)
                source = _stamp_seconds(msg.source_stamp)
                self.commands.append(
                    {
                        "wall": wall,
                        "stamp": stamp,
                        "source_stamp": source,
                        "has_source_stamp": bool(msg.has_source_stamp),
                        "session": int(msg.session),
                        "cycle": int(msg.cycle),
                        "valid": bool(msg.valid),
                        "steering": float(msg.steering_tire_angle_rad),
                        "target_speed": float(msg.target_speed_mps),
                        "acceleration": float(msg.acceleration_mps2),
                    }
                )
                self.sessions.add(int(msg.session))
                cycle = int(msg.cycle)
                if self.last_cycle is not None and cycle <= self.last_cycle:
                    self.cycle_anomalies += 1
                self.last_cycle = cycle
                self._record(topic, wall, source)
                return
            if topic == "/vehicle/driving_reference":
                self.references.append(
                    {
                        "wall": wall,
                        "stamp": _stamp_seconds(msg.stamp),
                        "source_stamp": _stamp_seconds(msg.source_stamp),
                        "has_source_stamp": bool(msg.has_source_stamp),
                        "session": int(msg.session),
                        "cycle": int(msg.cycle),
                        "valid": bool(msg.valid),
                        "path_valid": bool(msg.path_valid),
                        "path_a": float(msg.path_a),
                        "path_b": float(msg.path_b),
                        "path_c": float(msg.path_c),
                        "path_x_max_m": float(msg.path_x_max_m),
                        "horizon_dt_s": float(msg.horizon_dt_s),
                        "horizon_len": len(msg.speed_horizon_mps),
                        "v0": float(msg.speed_horizon_mps[0]) if msg.speed_horizon_mps else None,
                        "vN": float(msg.speed_horizon_mps[-1]) if msg.speed_horizon_mps else None,
                    }
                )
                self._record(topic, wall, _stamp_seconds(msg.source_stamp))
                return
            header = getattr(msg, "header", None)
            stamp = None
            if header is not None:
                stamp = _stamp_seconds(header.stamp)
                if topic == "/carla/hero/main_cam/image":
                    self.camera_stamps.append(stamp)
                    if len(self.camera_stamps) > 2000:
                        del self.camera_stamps[:1000]
                elif topic == "/localization/kinematic_state":
                    self.ego_stamps.append(stamp)
                    if len(self.ego_stamps) > 2000:
                        del self.ego_stamps[:1000]
            self._record(topic, wall, stamp)

        return callback

    def _summary(self, topic, window=None):
        now = time.monotonic()
        samples = [s for s in self.samples[topic] if window is None or s[0] >= now - window]
        deltas = [
            (samples[i][0] - samples[i - 1][0]) * 1000.0
            for i in range(1, len(samples))
        ]
        ages = [
            (wall - stamp) * 1000.0
            for _, wall, stamp in samples
            # Only host-time (Unix epoch) stamps have a meaningful arrival
            # age; CARLA sim-time stamps are source identity, not wall time.
            if stamp is not None and stamp > 1e6
        ]
        elapsed = max(now - self.t0, 1e-9)
        if window is not None:
            elapsed = window
        return {
            "total": len(self.samples[topic]),
            "recent": len(samples),
            "rate_hz_recent": round(len(samples) / elapsed, 3),
            "inter_arrival_ms": {
                "p50": round(_percentile(deltas, 0.5), 2) if deltas else None,
                "p95": round(_percentile(deltas, 0.95), 2) if deltas else None,
                "max": round(max(deltas), 2) if deltas else None,
            },
            "age_ms": {
                "p50": round(_percentile(ages, 0.5), 2) if ages else None,
                "p95": round(_percentile(ages, 0.95), 2) if ages else None,
                "max": round(max(ages), 2) if ages else None,
            },
        }

    def identity_report(self):
        camera = set(round(s, 6) for s in self.camera_stamps)
        ego = set(round(s, 6) for s in self.ego_stamps)
        pairs = 0
        same_source = 0
        matched_camera = 0
        matched_ego = 0
        no_source = 0
        # Pair command/reference samples by cycle.
        refs_by_cycle = {r["cycle"]: r for r in self.references}
        for command in self.commands:
            reference = refs_by_cycle.get(command["cycle"])
            if reference is None:
                continue
            pairs += 1
            if abs(command["source_stamp"] - reference["source_stamp"]) < 1e-9:
                same_source += 1
            if not command["has_source_stamp"] or not reference["has_source_stamp"]:
                no_source += 1
            else:
                key = round(command["source_stamp"], 6)
                if key in camera:
                    matched_camera += 1
                if key in ego:
                    matched_ego += 1
        horizons = [r["horizon_len"] for r in self.references if r["horizon_len"]]
        steps = [r["horizon_dt_s"] for r in self.references if r["horizon_dt_s"] > 0]
        return {
            "commands": len(self.commands),
            "references": len(self.references),
            "paired_cycles": pairs,
            "command_reference_same_source_stamp": same_source,
            "command_source_stamp_seen_in_camera": matched_camera,
            "command_source_stamp_seen_in_ego": matched_ego,
            "missing_source_stamp": no_source,
            "sessions_seen": sorted(self.sessions),
            "cycle_anomalies": self.cycle_anomalies,
            "last_cycle": self.last_cycle,
            "horizon_len_min": min(horizons) if horizons else None,
            "horizon_len_max": max(horizons) if horizons else None,
            "horizon_dt_s": steps[0] if steps else None,
            "path_invalid": sum(1 for r in self.references if not r["path_valid"]),
            "invalid_references": sum(1 for r in self.references if not r["valid"]),
        }


def main():
    run_sec = float(os.environ.get("RUN_SEC", "120"))
    report_every = float(os.environ.get("REPORT_EVERY_SEC", "10"))
    out_path = os.environ.get("OUT_JSON", "/tmp/cadence_probe_vp.json")
    rclpy.init()
    node = Probe()
    last_report = time.monotonic()
    try:
        while time.monotonic() - node.t0 < run_sec:
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            if now - last_report >= report_every:
                last_report = now
                report = {
                    "t_monotonic": round(now, 3),
                    "topics": {
                        topic: node._summary(topic, report_every)
                        for topic in TOPICS
                    },
                    "identity": node.identity_report(),
                }
                node.get_logger().info(json.dumps(report))
        final = {
            "run_sec": run_sec,
            "topics": {topic: node._summary(topic) for topic in TOPICS},
            "identity": node.identity_report(),
        }
        with open(out_path, "w") as handle:
            json.dump(final, handle, indent=2)
        print(json.dumps(final, indent=2), flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()