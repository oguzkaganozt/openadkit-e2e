#!/usr/bin/env python3
"""Align VP stage timing with adapter output gaps (findings 005/012).

Needs a VP build with the fork's `diag/vp-stage-timing` branch for the
`[Timing]` / `[Watchdog]` lines (without it only the adapter gaps are shown).
A gap whose VP time sits in `wait` is input starvation (no camera frame), not
VP work.

    python3 deploy/tools/analysis/cadence_stalls.py <run-dir>
"""

import re
import statistics
import sys
from datetime import datetime, timezone

STAGES = "cyc t wait pre infer ad as asp plan pub viz render busy".split()
TIMING = re.compile(
    r"\[Timing\] cyc=(\d+) t=([\d.]+) wait=([\d.]+) pre=([\d.]+) "
    r"infer=([\d.]+)\(ad=([\d.]+) as=([\d.]+) asp=([\d.]+)\) plan=([\d.]+) "
    r"pub=([\d.]+) viz=([\d.]+) render=([\d.]+) busy=([\d.]+)")


def docker_ts(line):
    return datetime.strptime(line[:26], "%Y-%m-%dT%H:%M:%S.%f").replace(
        tzinfo=timezone.utc).timestamp()


def main():
    run = sys.argv[1].rstrip("/")
    cycles, watchdog = [], []
    for line in open(f"{run}/visionpilot.log", errors="replace"):
        m = TIMING.search(line)
        if m:
            cycles.append(dict(zip(STAGES, map(float, m.groups()))))
        elif "[Watchdog]" in line:
            watchdog.append((docker_ts(line), line.split("[Watchdog]")[1].strip()))

    print(f"cycles={len(cycles)}")
    steady = cycles[5:]  # skip TensorRT warm-up
    for stage in STAGES[2:]:
        values = sorted(c[stage] for c in steady)
        if values:
            print(f"  {stage:6s} median={statistics.median(values):7.1f} "
                  f"p99={values[int(0.99 * len(values))]:7.1f} max={values[-1]:8.1f}")
    slow = [c for c in steady if c["busy"] + c["wait"] > 500]
    print(f"cycles with wait+busy>500ms: {len(slow)}")
    for c in slow[:30]:
        top = max("wait pre infer plan pub viz render".split(), key=lambda s: c[s])
        print(f"  cyc={int(c['cyc'])} t={c['t']:.3f} wait={c['wait']:.0f} "
              f"busy={c['busy']:.0f} top={top}:{c[top]:.0f}")
    print(f"watchdog lines: {len(watchdog)}")
    for ts, text in watchdog[:30]:
        print(f"  {ts:.3f} {text}")

    xfers = [docker_ts(l) for l in open(f"{run}/adapter.log", errors="replace") if "xfer #" in l]
    gaps = [(b - a, a) for a, b in zip(xfers, xfers[1:]) if b - a > 0.5]
    print(f"adapter xfers={len(xfers)} gaps>0.5s={len(gaps)} "
          f"max={max((g for g, _ in gaps), default=0):.2f}")
    for gap, start in gaps[:30]:
        print(f"  gap {gap:.2f}s starting {start:.3f}")


if __name__ == "__main__":
    main()
