#!/usr/bin/env python3
"""Observe the Safety Island supervisor output (ApprovedRequest) on domain 2.

Subscribes to /control/safety_island/approved_request (the single
actuator-facing SI output) and reports per window and at exit:

- decision histogram (NORMAL / SI_STOP / HOLD) and, for stops, the count of
  explicit-stop payloads (velocity 0 with negative acceleration);
- SI sessions observed and output_sequence monotonicity / gap counts
  (duplicates or gaps in the sequence on a healthy link);
- fault ids seen per session and the wall-clock time of each latch edge
  (fault_id increments and clears are what the fault-detection timeline is
  built from);
- the proposed control payload extrema (velocity / acceleration) as a
  behavior crosscheck alongside the CARLA ground truth.

Run in the adapter image (carries safety_island_msgs) on the SI domain:

  ROS_DOMAIN_ID=2 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \\
  docker run --rm --network host --ipc host ... adapter_probe-style command

See deploy/tools/adapter-fault-test.sh for the fault-injection wrapper.
"""
import json
import os
import time

import rclpy
from safety_island_msgs.msg import ApprovedRequest
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

RUN_SEC = float(os.environ.get("RUN_SEC", "60"))

DECISION_NAMES = {0: "NORMAL", 1: "SI_STOP", 2: "HOLD"}


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class SiProbe(Node):
    def __init__(self):
        super().__init__("si_probe")
        qos = QoSProfile(
            depth=50,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.t0 = time.monotonic()
        self.wall0 = time.time()
        self.done = False
        self.approved_count = 0
        self.decisions = {0: 0, 1: 0, 2: 0}
        self.modes = {}
        self.selected_sources = {}
        self.source_sessions = {}
        self.source_cycle_min = None
        self.source_cycle_max = None
        self.stop_payloads = 0
        self.sessions = {}
        self.last_seq = None
        self.seq_gaps = 0
        self.seq_backwards = 0
        self.faults = {}  # session -> {fault_id -> [wall_first, wall_last]}
        self.v_min = None
        self.v_max = None
        self.a_min = None
        self.a_max = None
        self.window = {0: 0, 1: 0, 2: 0}
        self.window_at = time.monotonic()
        self.create_subscription(
            ApprovedRequest, "/control/safety_island/approved_request",
            self.on_approved, qos,
        )
        self.create_timer(10.0, self.report_window)

    def on_approved(self, msg: ApprovedRequest) -> None:
        now_wall = time.time()
        self.approved_count += 1
        decision = int(msg.decision)
        if decision in self.decisions:
            self.decisions[decision] += 1
            self.window[decision] = self.window.get(decision, 0) + 1
        mode = int(msg.mode)
        source = int(msg.selected_source)
        self.modes[mode] = self.modes.get(mode, 0) + 1
        self.selected_sources[source] = self.selected_sources.get(source, 0) + 1
        if decision == 0:
            source_session = int(msg.source_session)
            source_cycle = int(msg.source_cycle)
            self.source_sessions[source_session] = self.source_sessions.get(source_session, 0) + 1
            self.source_cycle_min = (
                source_cycle if self.source_cycle_min is None
                else min(self.source_cycle_min, source_cycle)
            )
            self.source_cycle_max = (
                source_cycle if self.source_cycle_max is None
                else max(self.source_cycle_max, source_cycle)
            )
        if decision == 1:
            control = msg.control
            if (
                abs(float(control.longitudinal.velocity)) < 1e-6
                and float(control.longitudinal.acceleration) < -1e-3
            ):
                self.stop_payloads += 1
        session_info = self.sessions.setdefault(
            int(msg.session), {"count": 0, "seq_max": None, "gaps": 0, "backwards": 0}
        )
        session_info["count"] += 1
        seq = int(msg.output_sequence)
        if session_info["seq_max"] is not None:
            if seq > session_info["seq_max"] + 1:
                session_info["gaps"] += seq - session_info["seq_max"] - 1
            elif seq < session_info["seq_max"]:
                session_info["backwards"] += 1
        session_info["seq_max"] = max(seq, session_info["seq_max"] or seq)
        if int(msg.fault_id) and decision == 1:
            fault_info = self.faults.setdefault(int(msg.session), {})
            entry = fault_info.setdefault(
                int(msg.fault_id),
                {"first_wall": now_wall, "last_wall": now_wall, "reason_seen": None},
            )
            entry["last_wall"] = now_wall
        velocity = float(msg.control.longitudinal.velocity)
        accel = float(msg.control.longitudinal.acceleration)
        self.v_min = velocity if self.v_min is None else min(self.v_min, velocity)
        self.v_max = velocity if self.v_max is None else max(self.v_max, velocity)
        self.a_min = accel if self.a_min is None else min(self.a_min, accel)
        self.a_max = accel if self.a_max is None else max(self.a_max, accel)

    def report_window(self) -> None:
        if self.done:
            return
        now = time.monotonic()
        window_sec = round(now - self.window_at, 1)
        line = {
            "t_s": round(now - self.t0, 1),
            "window_s": window_sec,
            "windows_decisions": {
                DECISION_NAMES.get(k, k): v for k, v in self.window.items()
            },
            "approved_total": self.approved_count,
            "stop_payloads": self.stop_payloads,
        }
        print(json.dumps(line), flush=True)
        self.window = {0: 0, 1: 0, 2: 0}
        self.window_at = now

    def summary(self) -> dict:
        sessions_out = {
            session: info for session, info in self.sessions.items()
        }
        return {
            "run_sec": round(time.monotonic() - self.t0, 1),
            "approved_total": self.approved_count,
            "decisions": {
                DECISION_NAMES.get(k, k): v for k, v in self.decisions.items()
            },
            "modes": self.modes,
            "selected_sources": self.selected_sources,
            "normal_source_sessions": self.source_sessions,
            "normal_source_cycle_min_max": [self.source_cycle_min, self.source_cycle_max],
            "stop_payloads": self.stop_payloads,
            "sessions": sessions_out,
            "faults_per_session": {
                session: {fid: info["first_wall"] for fid, info in faults.items()}
                for session, faults in self.faults.items()
            },
            "velocity_min_max": [self.v_min, self.v_max],
            "accel_min_max": [self.a_min, self.a_max],
            "note_wall_time0": self.wall0,
        }


def main() -> None:
    rclpy.init()
    node = SiProbe()
    deadline = time.monotonic() + RUN_SEC
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.done = True
    print(json.dumps(node.summary()), flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
