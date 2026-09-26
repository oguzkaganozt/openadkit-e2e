#!/usr/bin/env python3
"""Lane-keeping summary of run-evidence.sh directories (findings 013).

Per run: distance to the first |lane_off| > 1 m, max |lane_off| in the first
300 m, first collision (sim time and odometer), SI latches, final pose, and the
lag (in VP cycles) at which VP's filtered CTE best correlates with its raw CTE.

    python3 deploy/tools/analysis/lane_metrics.py <run-dir> [...]
"""

import csv
import math
import re
import sys


def trace_metrics(run):
    dist, prev = 0.0, None
    first_1m, worst_300 = None, 0.0
    with open(f"{run}/steer-trace.csv", newline="") as f:
        for row in csv.DictReader(f):
            xy = float(row["x"]), float(row["y"])
            if prev:
                dist += math.hypot(xy[0] - prev[0], xy[1] - prev[1])
            prev = xy
            off = row["lane_off_m"]
            off = 0.0 if off == "nan" else abs(float(off))
            if dist <= 300:
                worst_300 = max(worst_300, off)
            if first_1m is None and off > 1.0 and float(row["speed_mps"]) > 1:
                first_1m = dist
    return first_1m, worst_300


def scenario_metrics(run):
    lines = open(f"{run}/scenario.log", errors="replace").read().splitlines()
    poses = [l for l in lines if "INFO: pose" in l]
    collisions = [l for l in lines if "COLLISION" in l]
    collision = None
    if collisions:
        sim_t = re.search(r"t=([\d.]+)s", collisions[0]).group(1)
        before = [p for p in poses if p[:26] <= collisions[0][:26]]
        odo = re.search(r"dist=(\d+)", before[-1]).group(1) if before else "?"
        collision = f"t={sim_t}s dist={odo} m"
    final = " ".join(poses[-1].split()[4:11]) if poses else "no pose"
    return collision, final


def filter_lag(run):
    """Lag (cycles) maximising corr(raw CTE, filtered CTE) while moving."""
    pairs, speed = [], 0.0
    for line in open(f"{run}/visionpilot.log", errors="replace"):
        m = re.search(r"ego_speed=([\d.]+)", line)
        if m:
            speed = float(m.group(1))
            continue
        m = re.search(r"cte=([-\d.]+)m\(raw=([-\d.]+)m\)", line)
        if m and speed > 3:
            pairs.append((float(m.group(1)), float(m.group(2))))
    if len(pairs) <= 50:
        return None, len(pairs)
    filt = [p[0] for p in pairs]
    raw = [p[1] for p in pairs]

    def corr(lag):
        n = len(raw) - lag
        a, b = raw[:n], filt[lag:lag + n]
        ma, mb = sum(a) / n, sum(b) / n
        num = sum((x - ma) * (y - mb) for x, y in zip(a, b))
        den = math.sqrt(sum((x - ma) ** 2 for x in a) * sum((y - mb) ** 2 for y in b))
        return num / den if den else 0.0

    return max(range(20), key=corr), len(pairs)


def main():
    for run in sys.argv[1:]:
        run = run.rstrip("/")
        env = open(f"{run}/env.txt").read()
        get = lambda key, default: (re.search(rf"{key}=(\S+)", env) or [None, default])[1]
        first_1m, worst_300 = trace_metrics(run)
        collision, final = scenario_metrics(run)
        latches = open(f"{run}/si.log", errors="replace").read().count("SI_STOP latched")
        lag, cycles = filter_lag(run)
        print(f"{run.split('/')[-1]}: img={get('VISIONPILOT_IMAGE', 'default')} "
              f"conf={get('VISIONPILOT_CONF', '-')} spawn={get('SPAWN_INDEX', 184)}")
        print(f"   first |off|>1m at {first_1m and round(first_1m)} m; "
              f"max |off| in first 300 m={worst_300:.2f}; "
              f"collision={collision or 'none'}; SI latch={latches}; end: {final}")
        print(f"   VP CTE filter lag={lag} cycles over {cycles} moving cycles")


if __name__ == "__main__":
    main()
