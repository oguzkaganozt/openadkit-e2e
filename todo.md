# TODO — remaining work after the 2026-09-25 evidence push

Read `AGENTS.md` first. Every measurement below needs a **fresh world**
(`run-loop.sh` tears down the stack each attempt) and must end as a numbered
entry in `docs/vp-findings.md` (template at the end of that file). Do not
retry injections in a latched world, and do not present the 4 m/s demo point
as a fix for high-speed behavior. The SI and actuator sides are done for the
agreed scope; don't change them without a new finding.

The runs cited below were made on a GPU rig host; the raw traces/logs live
under `/tmp/opencode/` on that machine and are **temporary** (see P5).
Findings 008–010 list each evidence file per run.

## P1 — VP lane measurement/fusion instability at speed (006/008/009/010)

**Symptoms.** On the spawn-184 curve at 9 m/s the car stops following the road
mid-curve: yaw rate collapses from ~0.05 rad/s (tracking) to ~0.004 rad/s while
the road still needs ~0.050 rad/s (v·κ); the lane offset grows +0.5 → +2.5 m in
~3 s and the car hits the guardrail at ~120–140 m. The raw camera CTE tracks
the drift then **flips sign** (+1.55 → −1.54 m in 0.23 s) near the lane edge;
the particle-filtered CTE lags (−0.3..−0.6 m/s) and the commanded tire angle
never exceeds ~0.05 rad (steady-state curve angle would be L·κ ≈ 0.016 rad).
The same speed on spawn 100 (upstream's 0.9.16 spawn) survives 614 m with
±2 m wandering, so route/spawn matters.

**Reproduce (spawn 184, fresh world).**
```bash
RIG_MODE=vp SI_MODE=vp RIG_JSON=carla-rig-empty.json \
  VISIONPILOT_CONF=vision_pilot.diag9.conf ./deploy/run-loop.sh --gpu
# then, in a second shell while the car drives:
/tmp/carla-venv/bin/python deploy/tools/passive_steering_probe.py \
  --out /tmp/steer-trace-p1.csv --seconds 110
```
Expected: first guardrail contact around sim t≈37–41 s, ~120–140 m. Compare
with the spawn-100 variant (`SPAWN_INDEX=100`) which should exceed 400 m.

**Instrumentation (VP-side, needs a test image).**
- Raw path measurement, particle filter and the `[Lateral]` log line:
  `upstream/vision_pilot/VisionPilot/modules/safety_guardian/fusion/src/lateral_fusion.cpp`
  (`raw_CTE`, `pts`, `xmin`, camera κ, AD-κ).
- Lane waypoints feeding the fit: `.../modules/models/src/auto_steer.cpp`,
  `.../modules/models/src/inference.cpp` (`H_resized`, AD curvature).
- Controller/feedforward: `.../safety_guardian/planning/src/lateral_planning.cpp`
  and `planning.cpp`; longitudinal: `.../longitudinal_planning.cpp`.
- Build a test image without touching the pinned tag:
  `cd upstream/vision_pilot/VisionPilot/docker && ./build.sh --gpu --ros2 --tag visionpilot:gpu-ros2-p1`
  then run with `VISIONPILOT_IMAGE=visionpilot:gpu-ros2-p1`.

**Candidate hypotheses to test individually** (one fresh-world A/B each):
lane-line association flipping to the wrong boundary; near-lane-edge gating;
fusion gain/particle lag; feedforward not using the fused κ; yaw estimate
scaling. Record what changed and what did not.

**Done when:** the failure mechanism is documented with instrumented evidence
and a fresh-world A/B at 9 m/s on spawn 184 holds |lane_off| ≤ 1.0 m for
≥300 m — or an honest, narrowed negative result is recorded. Any VP change goes
to the fork, the gitlink is updated, and `docs/media/` gets a new clip only
after the fix is validated.

## P2 — VP planning stalls drop source freshness (005, top long-run blocker)

**Symptoms.** VP stops publishing for 0.5–16 s while the process stays up and
logs in bursts afterwards; CPU busy, GPU idle, camera feed healthy. SI latches
on the 1.0 s source watchdog and stops safely, so every long run eventually
dies. On the current host the demo worlds showed 1.34–1.39 s gaps.

**Next steps.** Reproduce with `deploy/tools/image_stream_observer.py`
(camera-side health) plus adapter `xfer` gap counting
(`deploy/tools/measure-cadence.sh`, `cadence_probe_vp.py`) while VP runs.
Instrument VP per-stage timing (camera→preprocess→infer→publish) with the
existing stopwatch macros; the finding says the stall is inside the process,
so find the stage that blocks. A VP-side "no frame processed in N s" watchdog
would surface it as a first-class fault instead of input staleness.

**Done when:** the blocking stage is identified, and either fixed or exposed
as a VP fault with fresh-world evidence that long runs no longer latch.

## P3 — VP commands acceleration beyond the SI envelope

At spawn 100 the run ended because VP commanded **−7.513 m/s²** (cycle 547)
against the SI limit ±6.0; the SI refused and stopped safely. Earlier that run
had −4.09/−3.69/−3.5 (allowed). Find why VP produced it: trace
`compute_acceleration` in `.../planning/src/longitudinal_planning.cpp` and the
CIPO/lead state around that cycle (`.../fusion/src/longitudinal_fusion.cpp`,
`planning.cpp` lines ~114–118 classify ≤ −5 m/s²).

**Done when:** the source is explained (path/CIPO glitch vs deliberate
emergency decay) and either fixed in VP or documented as an intended command
that the SI correctly bounds.

## P4 — Upstream the fork fixes (optional for the rig, needed for a clean host)

Fork commits to propose to `autowarefoundation/vision_pilot`:
- `fb147561` DrivingCommand steering published in the contract sign;
- `2ecc4146` MJPEG viewer for the rendered frame;
- `491743a9` speed-HUD exposure fix (also the `rig/vp-e2e-demo` tip this repo
  pins).
Optionally the SI branch `feat/si-supervisor-v0-1` (`0df316e`) to
`autoware-safety-island` if that work should land upstream.

**Done when:** PRs are open (or a decision not to upstream is recorded), and if
anything merges, the gitlink and `docs/*` references are updated and re-validated
in a fresh world.

## P5 — Housekeeping (optional)

- Raw evidence referenced by findings 008–010 lives in `/tmp/opencode/` and on
  the rig host under `/root/records`; archive it off-repo if those machines
  are reclaimed before the work above is finished.
- The committed VP clips predate the H/C mount fix (`8077d53`); a controlled
  A/B showed nominal behavior unchanged, but re-record them after the next
  validated VP change anyway (provenance note in `docs/media/README.md`).
- `tests/__pycache__` is untracked stale data from removed test sources; safe
  to delete.

## Do not

- Do not wire VP outputs straight to CARLA; `carla_actuator.py` stays the only
  `apply_control()` caller.
- Do not add runtime mode/source fallback to the SI or relax the ±6.0 m/s²
  envelope to "make a run pass".
- Do not reuse a world after an SI latch; use a fresh world or the explicit
  re-enable path.
- Do not commit raw recordings; compress to `docs/media/` only.