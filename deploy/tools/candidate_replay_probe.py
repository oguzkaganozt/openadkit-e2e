#!/usr/bin/env python3
"""Inject a deliberately reordered VP trajectory into the selected SI ingress.

Run in a *fresh* VP→SI_CONTROL world. Both VP and adapter keep publishing;
one old cycle on the same source session must latch SI_STOP immediately, and
the later valid candidates must not clear the latch without operator re-enable.
"""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from safety_island_msgs.msg import ApprovedRequest, TrajectoryCandidate
from std_msgs.msg import Bool


class ReplayProbe(Node):
    def __init__(self):
        super().__init__("candidate_replay_probe")
        qos = QoSProfile(
            depth=50, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(
            TrajectoryCandidate, "/planning/visionpilot/trajectory_candidate", qos
        )
        self.reenable_publisher = self.create_publisher(
            Bool, "/control/safety_island/reenable", qos
        )
        self.create_subscription(
            TrajectoryCandidate, "/planning/visionpilot/trajectory_candidate",
            self.on_candidate, qos
        )
        self.create_subscription(
            ApprovedRequest, "/control/safety_island/approved_request",
            self.on_approved, qos
        )
        self.latest = None
        self.latest_at = None
        self.last_normal = None
        self.last_normal_at = None
        self.last_decision = None
        self.sent_at = None
        self.sent_id = None
        self.stops = 0
        self.normal_after_stop = 0
        self.candidates_after_stop = 0
        self.source_changed = False
        self.reenable_sent = False
        self.normal_after_reenable = 0
        self.reenabled_ids = set()

    def on_candidate(self, msg):
        if self.sent_at is not None and self.stops and (
            int(msg.source_session), int(msg.source_cycle)
        ) != tuple(self.sent_id):
            self.candidates_after_stop += 1
        self.latest = msg
        self.latest_at = time.monotonic()

    def on_approved(self, msg):
        if int(msg.mode) != 0 or int(msg.selected_source) != 0:
            self.source_changed = True
        self.last_decision = int(msg.decision)
        if self.last_decision == 0:
            if self.stops:
                if self.reenable_sent:
                    self.normal_after_reenable += 1
                    self.reenabled_ids.add((int(msg.source_session), int(msg.source_cycle)))
                else:
                    self.normal_after_stop += 1
            self.last_normal = (int(msg.source_session), int(msg.source_cycle))
            self.last_normal_at = time.monotonic()
        elif int(msg.decision) == 1 and self.sent_at is not None:
            self.stops += 1

    def inject(self):
        msg = self.latest
        if not msg or self.last_normal is None:
            return False
        now = time.monotonic()
        if self.last_decision != 0 or now - self.last_normal_at > 0.25 or \
                now - self.latest_at > 0.25:
            return False
        session, cycle = self.last_normal
        if int(msg.source_session) != session:
            return False
        # Reorder within *the same session*: a newer camera frame cannot
        # legitimize a lower VP cycle, and the chosen cycle is clearly behind
        # the last one SI approved (no accept/reject race). Never mutate the
        # selected source's normal publisher or the Autoware trajectory topic.
        stale_cycle = min(cycle, int(msg.source_cycle)) - 5
        if stale_cycle < 1:
            return False
        msg.source_cycle = stale_cycle
        self.sent_id = [int(msg.source_session), int(msg.source_cycle)]
        self.sent_at = time.monotonic()
        self.publisher.publish(msg)
        return True


def main():
    rclpy.init()
    node = ReplayProbe()
    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and not node.inject():
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.sent_at is None:
            raise SystemExit("FAIL: no matching live VP candidate and NORMAL selection")
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        result = {
            "injected_session_cycle": node.sent_id,
            "stop_decisions": node.stops,
            "normal_after_stop": node.normal_after_stop,
            "valid_candidates_after_stop": node.candidates_after_stop,
            "mode_or_source_changed": node.source_changed,
        }
        print(json.dumps(result), flush=True)
        if not node.stops or node.normal_after_stop or not node.candidates_after_stop or node.source_changed:
            raise SystemExit("FAIL: reordered VP candidate did not latch the selected SI source")
        print("VP candidate regression and persistent latch PASS", flush=True)
        node.reenable_sent = True
        node.reenable_publisher.publish(Bool(data=True))
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and node.normal_after_reenable < 3:
            rclpy.spin_once(node, timeout_sec=0.05)
        print(json.dumps({
            "explicit_reenable_normal": node.normal_after_reenable,
            "reenabled_source_ids": sorted(node.reenabled_ids)[:5],
            "mode_or_source_changed": node.source_changed,
        }), flush=True)
        if node.normal_after_reenable < 3 or node.source_changed or any(
            session != node.sent_id[0] for session, _ in node.reenabled_ids
        ):
            raise SystemExit("FAIL: explicit re-enable did not resume the same selected source")
        print("explicit re-enable of unchanged VP source PASS", flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
