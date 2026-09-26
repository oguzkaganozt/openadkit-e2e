# AGENTS.md

Closed-loop CARLA 0.9.16 rig: VisionPilot (VP) or Autoware plans, the Safety
Island (SI) decides, exactly one process actuates. Current contract and
evidence live in `docs/e2e2-stop-gate.md` (ingress + stop gate),
`docs/si-ingress-faults.md` (latch/re-enable), and `docs/vp-findings.md`
(numbered findings with an entry template; open work is listed in
`todo.md`: 013's curve offset, 015's stopped lead, 016's lane splits/junctions).

## Commands

```bash
./deploy/setup.sh                                 # once per host
./deploy/build.sh --dds-interface <nic> [--cpu|--gpu] [--run]
./deploy/run-loop.sh --gpu                        # one fresh-world attempt
python3 -m unittest discover -s adapter -v        # the only runnable test suite
```

- Test sources exist only in `adapter/`.
- Stop-gate measurement on a driving rig: `deploy/tools/stop-gate-test.sh`
  (see its header for the gate definition). Fresh-world evidence wrappers:
  `deploy/tools/run-clean-gate.sh`, `run-clean-vp-{replay,restart}.sh`.
- Example clips: `deploy/tools/record-example.sh <vp-control|vp-si|autoware-si> [sec]`
  then `deploy/tools/package-example.sh <raw-run-dir> docs/media/<mode>.mp4`.
- CARLA host venv (installed by `build.sh`): `/tmp/carla-venv/bin/python`;
  read-only steering/pose tracing: `deploy/tools/passive_steering_probe.py`.
  Start probes **after** `run-loop.sh` returns (it recreates CARLA; an earlier
  client never sees the new hero).
- 1 Hz images for every driving run: `deploy/tools/frame_sampler.py --chase
  --out <run>/frames` (front camera, VP HUD, chase camera; `index.csv` maps
  wall ms to files). Look at the frames at the moments the logs point to.
- One-shot evidence run (fresh world + trace + frames + all container logs):
  `deploy/tools/run-evidence.sh <label> [drive_sec]`, rig selected by the
  same env as `run-loop.sh`. Findings 011–014 used this shape; the numbers
  in them come from `deploy/tools/analysis/*.py` run on those directories.

`run-loop.sh` env: `RIG_MODE=vp|autoware`, `SI_MODE=si|vp`, `RIG_JSON=<rig>.json`,
`VISIONPILOT_IMAGE/RUNTIME/CONF`, `SPAWN_INDEX`, `BOTH_SOURCES=1` (requires
`SI_MODE=si` + `carla-rig-empty.json`). `autoware` + `SI_MODE=vp` is rejected.

## Architecture facts that are easy to get wrong

- `deploy/nodes/carla_actuator.py` is the **only** caller of `apply_control()`:
  it consumes SI `ApprovedRequest` on domain 2 and contains no watchdog, mode,
  or latch logic. It divides steer by the car's speed-dependent
  `steering_curve` so the approved tire angle is realised
  (`ACTUATOR_STEER_CURVE_COMP=0` = legacy mapping; 013). `carla_bridge.py` is
  telemetry only (camera, odom, steering, clock, preview on :8090) and must
  never actuate.
- One SI binary reads `SI_SUPERVISION_MODE`/`SI_TRAJECTORY_SOURCE` once at
  startup and fails closed on bad values. Do not add runtime fallback,
  mode switching, or an adapter-authored stop; freshness and stops are SI-owned.
- Domains: VP and the adapter on 1, SI and the actuator on 2.
  `deploy/config/bridge-config.yaml` is the complete list of bridged inputs
  (no control topic is bridged back).
- VP is ROS Jazzy/FastDDS; the adapter is Humble/CycloneDDS. Keep VP on
  FastDDS — unifying RMW fails on string deserialization.
- The 7.4 MB camera frames reach VP over cross-vendor UDP. With stock 212 KB
  socket buffers VP starves for up to seconds and the SI latches (finding
  012). `setup.sh` raises the limits and `run-loop.sh` refuses to run below
  16 MiB `rmem_default`; a "VP stall" on a host without this is not VP.
- CARLA needs an NVIDIA GPU even with `--cpu` (CPU only switches VP
  inference), and CARLA's `--ros2` is unsupported here; do not enable it.
- `deploy/config/H.yaml` and `deploy/config/homography_C_matrix.yaml` must be
  a **matched pair**. After any camera/H change run
  `deploy/tools/gen-homography-c.sh`; compose mounts both, and the image's
  built-in C comes from an unrelated example H.
- `deploy/maps/town04` is gitignored; fetch it with
  `deploy/tools/fetch-town04-map.sh` (run-loop verifies its checksums for the
  Autoware profile).
- Commit only compressed `docs/media/*.mp4`; raw recordings and logs stay out
  of git.

## Workflow rules

- Every `run-loop.sh` attempt tears down all containers and force-recreates
  CARLA, so measurements are always from a fresh world.
- Never retry an injected fault in the same world: `SI_STOP` latches; clearing
  needs an explicit `/control/safety_island/reenable` (all sources fresh) or a
  fresh run.
- Report the stop gate as SI detection → first CARLA-applied brake frame
  (≤500 ms). The source watchdog (cut → detection) is a separate number and
  must not be folded in.
- The submodule gitlinks are deliberate. `upstream/vision_pilot` is a fork;
  the pinned `rig/vp-e2e-demo` commit adds, on top of upstream, the
  `DrivingCommand` interface and steering-sign fix, the MJPEG viewer, the
  speed-HUD exposure fix, `fusion.lat.*` config keys (013), line-buffered
  stdout (012) and the AD-only-CIPO fix (011). Don't bump a gitlink without a
  fresh-world validation and a findings entry.
- VP's source stalls (005) are resolved by 012 (host socket buffers); the
  9 m/s weave (006/008–010) by 013's tuned lateral filter, which the rig confs
  set (`fusion.lat.*`; `diag4`/`diag9` keep the old filter on purpose as A/B
  baselines). Still open: a curvature-proportional curve offset (≤ 0.8 m at
  9 m/s, 013), lane splits / ramps / junctions (016), and a stopped lead hit at ~9 m/s
  (015: IDM sign bug, fusion velocity lag, near-field range).
  `vision_pilot.demo*.conf` stays at 4 m/s for the example clips.
- VP perception/model limits are recorded as findings and reported upstream,
  not worked around in the rig or the fork (015 dropped its near-field
  heuristics for this reason).
- `fusion.lat.cte_eval_x_m = 0` (branch `fix/vp-lat-eval-at-vehicle`, not in
  the pin) crashes the car at launch (013); don't reintroduce it.

## Docs

`README.md` (architecture, modes, status) and `deploy/README.md` (run knobs,
services, SI interfaces, tools) describe the current single-binary SI ingress
and the separate actuator; keep them in step with code changes.
`docs/vp-si-integration-plan.md` is explicitly historical. When prose
conflicts, trust `deploy/run-loop.sh`, `deploy/docker-compose.yaml`, and
`docs/e2e2-stop-gate.md`.
