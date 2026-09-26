#!/usr/bin/env python3
"""Mean |lane_off| on curves vs straights from the CARLA trace (finding 013).

Curve samples are |lane curvature| > 0.002 1/m; off/k is the mean offset per
curvature (the ~110 m x kappa steady offset of 013). Samples below 3 m/s and
the first 30 m are skipped.

    python3 deploy/tools/analysis/curve_offset.py <run-dir> [...]
"""

import csv
import math
import re
import sys

CURVE_KAPPA = 0.002


def mean(values):
    return sum(values) / len(values) if values else float("nan")


def main():
    for run in sys.argv[1:]:
        run = run.rstrip("/")
        dist, prev = 0.0, None
        curve, straight, per_kappa = [], [], []
        with open(f"{run}/steer-trace.csv", newline="") as f:
            for row in csv.DictReader(f):
                xy = float(row["x"]), float(row["y"])
                if prev:
                    dist += math.hypot(xy[0] - prev[0], xy[1] - prev[1])
                prev = xy
                if dist < 30 or float(row["speed_mps"]) < 3:
                    continue
                kappa = float(row["lane_curve_rad_per_m"])
                off = float(row["lane_off_m"])
                if abs(kappa) > CURVE_KAPPA:
                    curve.append(abs(off))
                    per_kappa.append(off / kappa)
                else:
                    straight.append(abs(off))
        collided = "COLLISION" in open(f"{run}/scenario.log", errors="replace").read()
        conf = re.search(r"VISIONPILOT_CONF=(\S+)", open(f"{run}/env.txt").read())
        print(f"{run.split('/')[-1]:50s} {(conf.group(1) if conf else '-'):34s} "
              f"dist={dist:5.0f} coll={'yes' if collided else 'no '} "
              f"curve|off|={mean(curve):.2f} (off/k={mean(per_kappa):+.0f} m) "
              f"straight|off|={mean(straight):.2f}")


if __name__ == "__main__":
    main()
