#!/usr/bin/env python3
"""Check VP session switch against a fixed SI_CONTROL process in one world.

Capture a healthy VP session (before), recreate only the VP producer, then
observe a new candidate session and the SI's approved source (after). If the
restart incurred an SI_STOP, issue one explicit re-enable after new candidates
arrive; the latch must never clear on its own or change mode/source.
"""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from safety_island_msgs.msg import ApprovedRequest, TrajectoryCandidate
from std_msgs.msg import Bool


class RestartProbe(Node):
    def __init__(self, old_session):
        super().__init__("vp_restart_probe")
        qos = QoSProfile(
            depth=50, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.old_session = old_session
        self.candidate_sessions = {}
        self.normal_ids = set()
        self.stop_count = 0
        self.unrequested_resume = 0
        self.bad_selection = 0
        self.old_approved_after_new = 0
        self.new_selected = False
        self.reenable_sent = False
        self.reenable = self.create_publisher(Bool, "/control/safety_island/reenable", qos)
        self.create_subscription(
            TrajectoryCandidate, "/planning/visionpilot/trajectory_candidate",
            self.on_candidate, qos
        )
        self.create_subscription(
            ApprovedRequest, "/control/safety_island/approved_request", self.on_approved, qos
        )

    def on_candidate(self, msg):
        session = int(msg.source_session)
        self.candidate_sessions[session] = self.candidate_sessions.get(session, 0) + 1
        if self.old_session is not None and session != self.old_session and \
                self.candidate_sessions[session] >= 3 and self.stop_count and \
                not self.reenable_sent:
            self.reenable.publish(Bool(data=True))
            self.reenable_sent = True

    def on_approved(self, msg):
        if int(msg.mode) != 0 or int(msg.selected_source) != 0:
            self.bad_selection += 1
        if self.old_session is not None and int(msg.source_session) != self.old_session:
            self.new_selected = True
        if int(msg.decision) == 1:
            self.stop_count += 1
        if int(msg.decision) == 0:
            if self.new_selected and int(msg.source_session) == self.old_session:
                self.old_approved_after_new += 1
            if self.stop_count and not self.reenable_sent:
                self.unrequested_resume += 1
            self.normal_ids.add((int(msg.source_session), int(msg.source_cycle)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("phase", choices=("before", "after"))
    parser.add_argument("--old-session", type=int)
    args = parser.parse_args()
    if args.phase == "after" and args.old_session is None:
        parser.error("after requires --old-session")
    rclpy.init()
    node = RestartProbe(args.old_session)
    try:
        deadline = time.monotonic() + (7 if args.phase == "before" else 25)
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.phase == "after" and len([
                key for key in node.normal_ids if key[0] != args.old_session
            ]) >= 3:
                break
        print(json.dumps({
            "phase": args.phase,
            "old_session": args.old_session,
            "candidate_sessions": node.candidate_sessions,
            "normal_source_ids": sorted(node.normal_ids)[:10],
            "stop_count": node.stop_count,
            "reenable_sent": node.reenable_sent,
            "unrequested_resume": node.unrequested_resume,
            "bad_selection": node.bad_selection,
            "old_approved_after_new": node.old_approved_after_new,
        }), flush=True)
        if node.bad_selection or node.unrequested_resume or node.old_approved_after_new:
            raise SystemExit("FAIL: SI changed mode/source or cleared its latch without re-enable")
        if args.phase == "before":
            healthy = {session for session, count in node.candidate_sessions.items()
                       if count >= 3 and any(key[0] == session for key in node.normal_ids)}
            if node.stop_count or len(healthy) != 1:
                raise SystemExit("FAIL: no single healthy original VP session")
            print(f"ORIGINAL_SESSION={healthy.pop()}", flush=True)
        else:
            new_sessions = set(node.candidate_sessions) - {args.old_session}
            matched = {session for session, _ in node.normal_ids} & new_sessions
            if len(new_sessions) != 1 or matched != new_sessions:
                raise SystemExit("FAIL: SI did not approve the new VP session")
            print("VP restart on unchanged SI ingress PASS", flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
