#!/usr/bin/env python3
"""Join VP CIPO dump lines to CARLA ground truth by camera sim stamp."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def load_jsonl(path: Path) -> list[dict]:
    rows = []
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            rows.append(json.loads(line))
    return rows


def nearest_gt(gt_rows: list[dict], sim_t: float, max_dt: float) -> dict | None:
    best = None
    best_dt = max_dt
    for row in gt_rows:
        dt = abs(float(row["sim_t"]) - sim_t)
        if dt <= best_dt:
            best = row
            best_dt = dt
    return best


def vp_sim_t(row: dict) -> float | None:
    stamp = int(row.get("cam_stamp_ns") or 0)
    if stamp > 0:
        return stamp / 1e9
    return None


def nearest_gt_wall(gt_rows: list[dict], wall_ns: int, max_ns: int) -> dict | None:
    best = None
    best_dt = max_ns
    for row in gt_rows:
        dt = abs(int(row["wall_ns"]) - wall_ns)
        if dt <= best_dt:
            best = row
            best_dt = dt
    return best


def cam_offset_s(vp_rows: list[dict], gt_rows: list[dict]) -> float | None:
    deltas = []
    for vp in vp_rows:
        sim_t = vp_sim_t(vp)
        wall = int(vp.get("wall_ns") or 0)
        if sim_t is None or wall <= 0:
            continue
        gt = nearest_gt_wall(gt_rows, wall, 250_000_000)
        if gt is None:
            continue
        deltas.append(sim_t - float(gt["sim_t"]))
        if len(deltas) >= 40:
            break
    if len(deltas) < 8:
        return None
    deltas.sort()
    return deltas[len(deltas) // 2]


def summarize(joined: list[dict]) -> dict:
    fused = [r for r in joined if r["vp"]["latch"]["in_fused"]]
    latched = [r for r in joined if r["vp"]["latch"]["latched"]]
    dist_err = []
    vel_err = []
    for row in fused:
        along = row["gt"].get("along")
        rel_v = row["gt"].get("rel_v")
        if along is not None:
            dist_err.append(row["vp"]["fus"]["dist"] - along)
        if rel_v is not None:
            vel_err.append(row["vp"]["fus"]["vel"] - rel_v)

    def first(rows, pred):
        for row in rows:
            if pred(row):
                return row
        return None

    first_fused = first(joined, lambda r: r["vp"]["latch"]["in_fused"])
    first_latch = first(joined, lambda r: r["vp"]["latch"]["latched"])
    first_hit = first(joined, lambda r: r["gt"].get("collision") == 1)

    def mae(vals):
        if not vals:
            return None
        return sum(abs(v) for v in vals) / len(vals)

    return {
        "n_joined": len(joined),
        "n_fused": len(fused),
        "n_latched": len(latched),
        "mae_dist_m": mae(dist_err),
        "mae_vel_ms": mae(vel_err),
        "first_fused": None
        if first_fused is None
        else {
            "sim_t": first_fused["sim_t"],
            "fus_dist": first_fused["vp"]["fus"]["dist"],
            "gt_along": first_fused["gt"].get("along"),
            "gt_bumper": first_fused["gt"].get("bumper"),
            "hero_v": first_fused["gt"].get("hero_v"),
        },
        "first_latch": None
        if first_latch is None
        else {
            "sim_t": first_latch["sim_t"],
            "last_dist": first_latch["vp"]["latch"]["last_dist"],
            "out_dist": first_latch["vp"]["latch"]["out_dist"],
            "gt_along": first_latch["gt"].get("along"),
            "hero_v": first_latch["gt"].get("hero_v"),
        },
        "first_collision": None
        if first_hit is None
        else {
            "sim_t": first_hit["sim_t"],
            "gt_along": first_hit["gt"].get("along"),
            "gt_bumper": first_hit["gt"].get("bumper"),
            "hero_v": first_hit["gt"].get("hero_v"),
        },
    }


def join_dumps(dump_dir: Path, max_dt: float = 0.08) -> tuple[list[dict], dict]:
    vp_rows = load_jsonl(dump_dir / "vp.jsonl")
    gt_rows = load_jsonl(dump_dir / "gt.jsonl")
    offset = cam_offset_s(vp_rows, gt_rows)
    joined = []
    skipped = 0
    for vp in vp_rows:
        sim_t = vp_sim_t(vp)
        gt = None
        if sim_t is not None and offset is not None:
            gt = nearest_gt(gt_rows, sim_t - offset, max_dt)
        if gt is None and sim_t is not None:
            gt = nearest_gt(gt_rows, sim_t, max_dt)
        if gt is None:
            wall = int(vp.get("wall_ns") or 0)
            if wall > 0:
                gt = nearest_gt_wall(gt_rows, wall, int(max_dt * 1e9))
        if gt is None:
            skipped += 1
            continue
        joined.append({"sim_t": float(gt["sim_t"]), "vp": vp, "gt": gt})
    summary = summarize(joined)
    summary["n_vp"] = len(vp_rows)
    summary["n_gt"] = len(gt_rows)
    summary["n_skipped"] = skipped
    summary["cam_offset_s"] = offset
    return joined, summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dump_dir", type=Path)
    parser.add_argument("--max-dt", type=float, default=0.08)
    args = parser.parse_args(argv)
    dump_dir = args.dump_dir
    if not (dump_dir / "vp.jsonl").is_file() or not (dump_dir / "gt.jsonl").is_file():
        print(f"missing vp.jsonl or gt.jsonl in {dump_dir}", file=sys.stderr)
        return 2
    joined, summary = join_dumps(dump_dir, args.max_dt)
    (dump_dir / "joined.jsonl").write_text(
        "".join(json.dumps(row, separators=(",", ":")) + "\n" for row in joined),
        encoding="utf-8",
    )
    (dump_dir / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
