#!/usr/bin/env python3
"""VP_CONTROL verbatim-passthrough check.

Capture VP's `/vehicle/driving_command` (domain 1) and the SI's
`/control/safety_island/approved_request` (domain 2), then join them on
(session, cycle) and verify the SI passed VP's steering, target speed and
acceleration through unchanged. VP carries float64 values and the SI Control
payload is float32, so equality is checked within 1e-3.

Usage (run each side in a container with the matching domain):

  python3 vp_passthrough_check.py capture-vp --out vp.jsonl --sec 15
  python3 vp_passthrough_check.py capture-si --out si.jsonl --sec 15
  python3 vp_passthrough_check.py join vp.jsonl si.jsonl
"""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

TOLERANCE = 1e-3


def capture_qos():
    return QoSProfile(
        depth=200,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class Capture(Node):
    def __init__(self, side, out_path):
        super().__init__(f"vp_passthrough_{side}")
        self._out = open(out_path, "w", encoding="utf-8")
        self.count = 0
        if side == "vp":
            from visionpilot_msgs.msg import DrivingCommand

            self.create_subscription(
                DrivingCommand, "/vehicle/driving_command", self._on_vp, capture_qos()
            )
        else:
            from safety_island_msgs.msg import ApprovedRequest

            self.create_subscription(
                ApprovedRequest,
                "/control/safety_island/approved_request",
                self._on_si,
                capture_qos(),
            )

    def _record(self, record):
        self.count += 1
        self._out.write(json.dumps(record) + "\n")
        self._out.flush()

    def _on_vp(self, msg):
        if not msg.valid:
            return
        self._record(
            {
                "session": int(msg.session),
                "cycle": int(msg.cycle),
                "steer": float(msg.steering_tire_angle_rad),
                "speed": float(msg.target_speed_mps),
                "accel": float(msg.acceleration_mps2),
            }
        )

    def _on_si(self, msg):
        if int(msg.decision) != 0 or int(msg.selected_source) != 1:
            return
        self._record(
            {
                "session": int(msg.source_session),
                "cycle": int(msg.source_cycle),
                "steer": float(msg.control.lateral.steering_tire_angle),
                "speed": float(msg.control.longitudinal.velocity),
                "accel": float(msg.control.longitudinal.acceleration),
            }
        )


def capture(side, out_path, seconds):
    rclpy.init()
    node = Capture(side, out_path)
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.try_shutdown()
    print(f"{side}: captured {node.count} samples to {out_path}")


def read_records(path):
    records = {}
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            record = json.loads(line)
            records[(record["session"], record["cycle"])] = record
    return records


def join(vp_path, si_path):
    vp = read_records(vp_path)
    si = read_records(si_path)
    common = sorted(set(vp) & set(si))
    mismatches = 0
    worst = 0.0
    for key in common:
        for field in ("steer", "speed", "accel"):
            diff = abs(vp[key][field] - si[key][field])
            worst = max(worst, diff)
            if diff > TOLERANCE:
                mismatches += 1
                if mismatches <= 5:
                    print("MISMATCH", key, field, vp[key][field], si[key][field])
    summary = {
        "vp_samples": len(vp),
        "si_samples": len(si),
        "matched_cycles": len(common),
        "mismatched_fields": mismatches,
        "max_abs_diff": worst,
        "vp_only": len(set(vp) - set(si)),
        "si_only": len(set(si) - set(vp)),
    }
    print(json.dumps(summary, indent=2))
    if not common or mismatches:
        raise SystemExit(1)
    print("VERBATIM PASS: SI passed VP's command unchanged")


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    command = sys.argv[1]
    if command == "capture-vp":
        capture("vp", sys.argv[sys.argv.index("--out") + 1],
                float(sys.argv[sys.argv.index("--sec") + 1]))
    elif command == "capture-si":
        capture("si", sys.argv[sys.argv.index("--out") + 1],
                float(sys.argv[sys.argv.index("--sec") + 1]))
    elif command == "join":
        join(sys.argv[2], sys.argv[3])
    else:
        raise SystemExit(__doc__)


if __name__ == "__main__":
    main()