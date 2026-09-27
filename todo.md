# TODO — status after the 2026-09-25/27 work sessions

Read `AGENTS.md` first. Every measurement needs a **fresh world**; use
`deploy/tools/run-evidence.sh <label> [sec]` (trace + 1 Hz frames + all logs)
and end each result as a numbered entry in `docs/vp-findings.md`. Look at the
frames, not only the logs. The 2026-09-25 VPS rig has been released; every
`/root/...` path cited in `docs/` lives in the owner's local archives
`~/openadkit-e2e-evidence-20260925.tar.zst` (`vps-records/` = `/root/records`,
`local-opencode/` = `/tmp/opencode`) and
`~/openadkit-e2e-evidence-20260925-vps-root.tar.zst` (the rest of `/root`,
incl. `si-*-evidence-20260925/`). The 2026-09-26 RTX 5080 rig's runs (SI
re-enable A/B, 015, 016, final validation) are in
`~/openadkit-e2e-evidence-20260926/` (`si-validate/`, `vps-records/`); the
2026-09-27 RTX 5060 Ti rig's runs (native ingress, Ackermann actuator, example
clips) are in `~/openadkit-e2e-evidence-20260927/`. A new
rig needs `deploy/setup.sh` (socket buffers) before any measurement.

## Closed

| Item | Finding | Result |
|---|---|---|
| P2 — VP "planning stalls" | 012 | Camera-frame starvation on the CycloneDDS→FastDDS UDP path with 212 KB socket buffers, not VP work. `setup.sh` persists 64/16 MiB limits; `run-loop.sh` refuses smaller. 10 min at 4 m/s: 0 gaps > 0.5 s, VP at the full 10 Hz. |
| P3 — −7.5 m/s² command | 011 | Phantom AutoDrive-only CIPO on the guardrail in curves. Fork fix: AD-only frames may continue an AS-started track, not start one. Phantoms 7 → 0 on spawn 100; real slow lead still followed at ~11.5 m. |
| P1 — 9 m/s lane loss (006/008–010) | 013 | Lateral filter lag (~0.7 s) drove a growing weave. Tuned `fusion.lat.*` in the rig confs: spawn 184 at 9 m/s 1076–1088 m in 6/6 runs, no contact (default filter: guardrail at 212–284 m). No regression at 4 m/s. |
| Actuator tire-angle realisation | 013 | CARLA's speed-dependent steering curve delivered 0.87× the approved angle at 9 m/s; the actuator now compensates (contract fix, no lane-keeping change). |
| CARLA server stall | 014 | One run: the simulator stopped; SI latched on stale odometry as designed. Watch for recurrence. |
| P5 — housekeeping | — | `tests/__pycache__` deleted; evidence archived; VP clips re-recorded on the new pin (`docs/media/README.md`). |
| 4 m/s speed oscillation (3.7–4.3 m/s) | `carla_actuator.py` | The hand-made pedal map (0.12·v feedforward) fought the automatic gearbox (0.40 throttle holds 4.45 m/s, 0.49 runs away past 16 m/s). The actuator now drives through CARLA's Ackermann controller with the 013 steering compensation: 4 m/s holds 3.98–4.00 m/s (std 0.168 → 0.003), speed-tracking error halved at 9 m/s, tire angle realised 0.99, gates 13/5/4 ms, Autoware drive to its goal at 4.17 m/s. |
| Native VP ↔ SI ingress (no adapter) | `si-ingress-faults.md` | SI `e88f78f` reads `DrivingReference` itself; adapter container removed. Replay/restart PASS, gates 11/12/7 ms, 9 m/s drive 1363 m vs 1362 m with the adapter. |
| SI re-enable 15 ms stop artifact | `si-ingress-faults.md` | SI `65b6875`: the latch-clearing tick resumes NORMAL. A/B: 1 → 0 `fault=0` stops; replay/restart PASS; gates 9/11/8 ms. |

Pin: `upstream/vision_pilot` → `rig/vp-e2e-demo` `9cae16f9` (fork), validated
in fresh worlds (013, `…final-*` runs).

## Open

### 1. Curve offset (013 remainder)
The car holds a steady, curvature-proportional offset in curves
(≈ 110 m·κ at 9 m/s, ≈ 150 m·κ at 4 m/s; ≤ 0.8 m on spawn 184). VP measures it
(its CTE agrees), so it is not a camera bias; the actuator gain and reading
CTE/yaw at the vehicle were ruled out (the latter crashes at launch). It is
VP's MPC: the SI follower drives the same curve centred (−0.04 m). Raising the
MPC `cte_weight` 10 → 40 (fork `exp/vp-mpc-cte-weight-40`) cuts the offset to
0.18–0.25 m but spun the car out once at the ~630 m merge where the CTE jumps.
Next: gate CTE jumps at lane splits/merges (`lateral_fusion.cpp`), then retry a
stiffer `cte_weight` (20–40); gate: curve |lane_off| and ≥ 6/6 no-contact
runs on spawn 184 at 9 m/s. Item 2 shares the lane-split root.

### 2. Lane splits / ramps / junctions (016, VP-side)
Open-road lane keeping holds (2 × ~2.5 km at 9 m/s on spawn 184, 1780 m on
spawn 100). Contacts happen only at the spawn-184 fork (~615 m, intermittent),
the end-of-highway signalized junction (~2520 m, 2/2) and the spawn-100 ramp
(~1270 m, 3/3 at default speed, intermittent at 9 m/s). Causes in 016: no
lane-split handling, and a curve speed limit without preview (removed upstream
in `6305ea90`). Report upstream (issue); drive-length tests should stop before
2500 m on spawn 184.

### 3. Stopped lead at speed (002 → 015, VP-side)
The pin hits a stopped lead at ~9 m/s. Causes are measured in 015: an IDM
closing-speed sign regression (plain code bug), two fusion-design issues
(low-flag AD distance, unobservable track velocity) and a perception limit
(no reliable range below ~8 m; AS+H reads ~2.4 m long). Per scope we record
and report, not work around: file an upstream issue with 015's data, and
draft PRs for the code-level branches below. Near-field heuristics
(`exp/vp-near-field-hold`) were tried and dropped. Test with
`carla-rig.json` (stopped lead), `carla-rig-stop-go.json`,
`carla-rig-slow-lead.json`.

### 4. Upstream PRs (opened as drafts 2026-09-27)
SI: autowarefoundation/autoware-safety-island#66 (`feat/si-supervisor-v0-1`,
CI green). VisionPilot, from the public fork `oguzkaganozt/vision_pilot`, each
on upstream `main` `d4d9be13` and built with the Dockerfile builder stage:

| PR | Branch | Content |
|---|---|---|
| #423 (updated) | `feat/ros2-native-motion-intent` | typed `DrivingCommand`/`DrivingReference` + steering sign |
| #427 | `pr/longitudinal-fixes` | IDM closing-speed sign (015) + AD-only CIPO (011) |
| #428 | `pr/fusion-lat-config` | `fusion.lat.*` keys (013) |
| #429 | `pr/viz-mjpeg-hud` | MJPEG viewer + speed-HUD exposure |
| #430 | `pr/build-and-log` | ENABLE_RADAR default + line-buffered stdout (012) |

Not PRs: `fix/vp-ad-dist-needs-flag` and `fix/vp-long-fusion-config` (015) go
into the stopped-lead issue as measured proposals. Issues: #431 (015,
stopped lead) and #432 (016, lane splits/ramps/junction, incl. 013's offset). Upstream CI: DCO and spell-check fail on merged PRs too;
`semantic-pull-request` needs a lowercase subject, and PRs opened together
cancel each other's run (shared concurrency group); a body edit re-triggers it.
If anything merges, re-pin and re-validate.

## Do not

- Do not wire VP outputs straight to CARLA; `carla_actuator.py` stays the only
  `apply_control()` caller.
- Do not add runtime mode/source fallback to the SI or relax the ±6.0 m/s²
  envelope to "make a run pass".
- Do not reuse a world after an SI latch; use a fresh world or the explicit
  re-enable path.
- Do not measure on a host with small socket buffers (012); a "VP stall" there
  is not VP.
- Do not commit raw recordings; compress to `docs/media/` only.
