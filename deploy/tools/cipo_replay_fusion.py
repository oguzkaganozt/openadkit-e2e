#!/usr/bin/env python3
"""Replay camera-only CIPO fusion on a recorded dump (fork vs upstream)."""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cipo_join import join_dumps, load_jsonl  # noqa: E402

D_MAX = 150.0
CIPO_PROB_MIN = 0.40
N_PARTICLES = 500
PROCESS_NOISE_DIST = 2.0
PROCESS_NOISE_VEL = 0.50
AD_NOISE_MIN = 1.5
AD_NOISE_MAX = 8.0
CIPO_NOISE = 3.0
RESET_GATE = 25.0
HOMOGRAPHY_REF = 30.0
DT_DEFAULT = 0.10


def _ad_stddev(flag: float) -> float:
    p = min(1.0, max(0.0, flag))
    return AD_NOISE_MIN + (AD_NOISE_MAX - AD_NOISE_MIN) * (1.0 - p)


def _homography_noise(dist: float) -> float:
    scale = dist / HOMOGRAPHY_REF
    return min(200.0, max(0.25 * CIPO_NOISE, CIPO_NOISE * scale * scale))


class CameraFusion:
    def __init__(self, policy: str, rng: random.Random):
        if policy not in ("fork", "upstream"):
            raise ValueError(policy)
        self.policy = policy
        self.rng = rng
        self.particles: list[tuple[float, float, float]] = []
        self.initialised = False
        self.prev_cut_in = False

    def reset(self) -> None:
        self.particles = []
        self.initialised = False

    def _gauss(self, mean: float, std: float) -> float:
        return self.rng.gauss(mean, std)

    def init_from(self, dist: float, stddev: float) -> None:
        parts = []
        for _ in range(N_PARTICLES):
            d = min(D_MAX, max(0.0, self._gauss(dist, stddev)))
            v = self._gauss(0.0, 2.0)
            parts.append((d, v, 0.0))
        self.particles = parts
        self.initialised = True

    def predict(self, dt: float) -> None:
        nxt = []
        for dist, vel, log_w in self.particles:
            dist = min(
                D_MAX,
                max(0.0, dist + vel * dt + self._gauss(0.0, PROCESS_NOISE_DIST)),
            )
            vel = vel + self._gauss(0.0, PROCESS_NOISE_VEL)
            nxt.append((dist, vel, log_w))
        self.particles = nxt

    def _cloud_mean(self) -> float:
        return sum(p[0] for p in self.particles) / len(self.particles)

    def _weight(self, ad, as_h) -> None:
        nxt = []
        for dist, vel, log_w in self.particles:
            if ad is not None:
                err = (ad[0] - dist) / ad[1]
                log_w += -0.5 * err * err
            if as_h is not None:
                err = (as_h[0] - dist) / as_h[1]
                log_w += -0.5 * err * err
            nxt.append((dist, vel, log_w))
        self.particles = nxt

    def _weights(self) -> list[float]:
        max_lw = max(p[2] for p in self.particles)
        raw = [math.exp(p[2] - max_lw) for p in self.particles]
        total = sum(raw)
        if total < 1e-12:
            w = 1.0 / len(raw)
            return [w] * len(raw)
        return [x / total for x in raw]

    def _effective_n(self) -> float:
        w = self._weights()
        return 1.0 / (sum(x * x for x in w) + 1e-12)

    def _resample(self) -> None:
        n = len(self.particles)
        w = self._weights()
        cs = []
        acc = 0.0
        for wi in w:
            acc += wi
            cs.append(acc)
        u0 = self.rng.random() / n
        j = 0
        nxt = []
        for i in range(n):
            thr = u0 + i / n
            while j < n - 1 and cs[j] < thr:
                j += 1
            dist, vel, _ = self.particles[j]
            nxt.append((dist, vel, 0.0))
        self.particles = nxt

    def _mean(self) -> tuple[float, float]:
        w = self._weights()
        mean_d = sum(wi * p[0] for wi, p in zip(w, self.particles))
        mean_v = sum(wi * p[1] for wi, p in zip(w, self.particles))
        return mean_d, mean_v

    def update(self, ad_valid, flag, ad_dist, as_dist, cut_in, dt) -> dict:
        as_box = as_dist is not None
        if self.policy == "fork":
            ad_ok = bool(ad_valid) and flag >= CIPO_PROB_MIN
            as_ok = as_box
        else:
            ad_ok = bool(ad_valid) and (as_box or flag >= CIPO_PROB_MIN)
            as_ok = as_box

        if not ad_ok and not as_ok:
            self.reset()
            self.prev_cut_in = False
            return {"valid": True, "dist": D_MAX, "vel": 0.0, "reset": True}

        ad = None
        if ad_ok and ad_dist is not None and ad_dist > 1.0:
            ad = (float(ad_dist), _ad_stddev(flag))
        as_h = None
        if as_ok:
            std = CIPO_NOISE if self.policy == "fork" else _homography_noise(as_dist)
            as_h = (float(as_dist), std)

        if not self.initialised:
            if self.policy == "fork":
                if ad is not None:
                    self.init_from(*ad)
                elif as_h is not None:
                    self.init_from(*as_h)
                else:
                    return {"valid": False, "dist": D_MAX, "vel": 0.0, "reset": True}
            else:
                if ad is not None:
                    self.init_from(*ad)
                elif as_h is not None:
                    self.init_from(*as_h)
                else:
                    return {"valid": False, "dist": D_MAX, "vel": 0.0, "reset": True}
        else:
            self.predict(dt if dt > 1e-6 else DT_DEFAULT)
            cloud = self._cloud_mean()
            cut_in_edge = bool(cut_in) and not self.prev_cut_in
            if self.policy == "fork":
                if as_h is not None and abs(as_h[0] - cloud) > RESET_GATE:
                    self.init_from(*as_h)
                elif ad is not None and (ad[0] - cloud) > RESET_GATE:
                    self.init_from(*ad)
            else:
                if cut_in_edge:
                    if as_h is not None:
                        self.init_from(*as_h)
                    elif ad is not None:
                        self.init_from(*ad)
                elif ad is not None and abs(ad[0] - cloud) > RESET_GATE:
                    if as_h is not None:
                        self.init_from(*as_h)
                    else:
                        self.init_from(*ad)
        self.prev_cut_in = bool(cut_in)
        self._weight(ad, as_h)
        if self._effective_n() < 0.5 * N_PARTICLES:
            self._resample()
        dist, vel = self._mean()
        return {"valid": True, "dist": dist, "vel": vel, "reset": False}


def frame_meas(vp: dict) -> tuple[bool, float, float | None, float | None, bool]:
    ad = vp["ad"]
    as_dist = vp["fus"]["raw_dist"] if vp["fus"].get("raw") else None
    if as_dist is None:
        dets = [d for d in vp["as"]["dets"] if d["cls"] in (1, 2)]
        if dets:
            as_dist = None
    return (
        bool(ad.get("valid")),
        float(ad.get("flag") or 0.0),
        float(ad["dist_m"]) if ad.get("valid") else None,
        as_dist,
        bool(vp["fus"].get("cut_in")),
    )


def frame_dt(prev: dict | None, cur: dict) -> float:
    if prev is None:
        return DT_DEFAULT
    a = int(prev.get("cam_stamp_ns") or 0)
    b = int(cur.get("cam_stamp_ns") or 0)
    if a > 0 and b > a:
        dt = (b - a) / 1e9
        if 0.02 <= dt <= 0.5:
            return dt
    return DT_DEFAULT


def mae(vals: list[float]) -> float | None:
    if not vals:
        return None
    return sum(abs(v) for v in vals) / len(vals)


def replay_joined(joined: list[dict], policy: str, seed: int) -> list[dict]:
    fusion = CameraFusion(policy, random.Random(seed))
    out = []
    prev = None
    for row in joined:
        vp = row["vp"]
        ad_valid, flag, ad_dist, as_dist, cut_in = frame_meas(vp)
        est = fusion.update(
            ad_valid, flag, ad_dist, as_dist, cut_in, frame_dt(prev, vp)
        )
        out.append(est)
        prev = vp
    return out


def score(joined: list[dict], estimates: list[dict]) -> dict:
    fused = []
    vel = []
    n_fused = 0
    n_reset = 0
    first = None
    for row, est in zip(joined, estimates):
        if est.get("reset") or est["dist"] >= D_MAX - 0.5:
            n_reset += 1
            continue
        n_fused += 1
        if first is None:
            first = {
                "sim_t": row["sim_t"],
                "dist": est["dist"],
                "gt_along": row["gt"].get("along"),
            }
        along = row["gt"].get("along")
        rel = row["gt"].get("rel_v")
        if along is not None:
            fused.append(est["dist"] - along)
        if rel is not None:
            vel.append(est["vel"] - rel)
    return {
        "n_fused": n_fused,
        "n_reset": n_reset,
        "mae_dist_m": mae(fused),
        "mae_vel_ms": mae(vel),
        "first_fused": first,
    }


def median_score(joined: list[dict], policy: str, seeds: list[int]) -> dict:
    scores = [score(joined, replay_joined(joined, policy, s)) for s in seeds]
    keys = ["n_fused", "n_reset", "mae_dist_m", "mae_vel_ms"]
    out = {}
    for key in keys:
        vals = [s[key] for s in scores if s[key] is not None]
        vals.sort()
        out[key] = vals[len(vals) // 2] if vals else None
    out["first_fused"] = scores[0]["first_fused"]
    return out


def live_score(joined: list[dict]) -> dict:
    fake = []
    for row in joined:
        fus = row["vp"]["fus"]
        fused = bool(row["vp"]["latch"]["in_fused"])
        fake.append(
            {
                "dist": fus["dist"] if fused else D_MAX,
                "vel": fus["vel"] if fused else 0.0,
                "reset": not fused,
            }
        )
    return score(joined, fake)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dump_dir", type=Path)
    parser.add_argument("--seeds", type=int, default=5)
    args = parser.parse_args(argv)
    dump_dir = args.dump_dir
    joined_path = dump_dir / "joined.jsonl"
    if joined_path.is_file():
        joined = load_jsonl(joined_path)
    else:
        joined, _ = join_dumps(dump_dir)
    seeds = list(range(args.seeds))
    result = {
        "live": live_score(joined),
        "fork": median_score(joined, "fork", seeds),
        "upstream": median_score(joined, "upstream", seeds),
        "n_joined": len(joined),
        "seeds": args.seeds,
    }
    (dump_dir / "replay_summary.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
