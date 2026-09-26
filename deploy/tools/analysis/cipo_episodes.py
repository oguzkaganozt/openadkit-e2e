#!/usr/bin/env python3
"""List VP CIPO episodes and what confirmed them (finding 011).

An episode is a run of consecutive `plan:` lines with cipo=true. For each:
start time, frames, minimum fused distance and acceleration, and the AutoSpeed
(AS+H) / AutoDrive (AD) measurements from the preceding `[Fusion]` lines —
AS=(none) throughout means an AutoDrive-only CIPO. Needs fusion.debug = true.

    python3 deploy/tools/analysis/cipo_episodes.py <visionpilot.log> [...]
"""

import re
import sys

FUSION = re.compile(r"\[Fusion\] src=(\w+) \| AD=([^|]+)\| AS\+H=([^|]+)\|.*Fused=([\d.]+) m")
PLAN = re.compile(r"plan: .*accel=([-\d.]+) .*cipo=(true|false)\s+dist=([\d.]+)")


def episodes(path):
    found, current, last_fusion = [], None, None
    for line in open(path, errors="replace"):
        m = FUSION.search(line)
        if m:
            last_fusion = (m.group(2).strip(), m.group(3).strip())
            continue
        m = PLAN.search(line)
        if not m:
            continue
        accel, cipo, dist = float(m.group(1)), m.group(2) == "true", float(m.group(3))
        if not cipo:
            current = None
            continue
        if current is None:
            current = {"t": line[11:23], "n": 0, "min_acc": 9.0, "min_d": 999.0,
                       "ad": set(), "as": set()}
            found.append(current)
        current["n"] += 1
        current["min_acc"] = min(current["min_acc"], accel)
        current["min_d"] = min(current["min_d"], dist)
        if last_fusion:
            current["ad"].add(last_fusion[0])
            current["as"].add(last_fusion[1])
    return found


def main():
    for path in sys.argv[1:]:
        found = episodes(path)
        print(f"== {path.split('/')[-1]}: {len(found)} episodes")
        for e in found:
            print(f"  {e['t']} frames={e['n']} min_d={e['min_d']:.1f} "
                  f"min_acc={e['min_acc']:.2f} AS={sorted(e['as'])} AD={sorted(e['ad'])[:3]}")


if __name__ == "__main__":
    main()
