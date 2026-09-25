# AGENTS.md

Closed-loop CARLA 0.9.16 rig: VisionPilot (VP) or Autoware plans, the Safety
Island (SI) decides, exactly one process actuates. Current contract and
evidence live in `docs/e2e2-stop-gate.md` (ingress + stop gate),
`docs/si-ingress-faults.md` (latch/re-enable), and `docs/vp-findings.md`
(numbered findings with an entry template; 005/006/008–010 are open).

## Commands

```bash
./deploy/setup.sh                                 # once per host
./deploy/build.sh --dds-interface <nic> [--cpu|--gpu] [--run]
./deploy/run-loop.sh --gpu                        # one fresh-world attempt
python3 -m unittest discover -s adapter -v        # the only runnable test suite
```

- Test sources exist only in `adapter/`; `tests/` is untracked stale pycache.
- Stop-gate measurement on a driving rig: `deploy/tools/stop-gate-test.sh`
  (see its header for the gate definition). Fresh-world evidence wrappers:
  `deploy/tools/run-clean-gate.sh`, `run-clean-vp-{replay,restart}.sh`.
- Example clips: `deploy/tools/record-example.sh <vp-control|vp-si|autoware-si> [sec]`
  then `deploy/tools/package-example.sh <raw-run-dir> docs/media/<mode>.mp4`.
- CARLA host venv (installed by `build.sh`): `/tmp/carla-venv/bin/python`;
  read-only steering/pose tracing: `deploy/tools/passive_steering_probe.py`.

`run-loop.sh` env: `RIG_MODE=vp|autoware`, `SI_MODE=si|vp`, `RIG_JSON=<rig>.json`,
`VISIONPILOT_IMAGE/RUNTIME/CONF`, `SPAWN_INDEX`, `BOTH_SOURCES=1` (requires
`SI_MODE=si` + `carla-rig-empty.json`). `autoware` + `SI_MODE=vp` is rejected.

## Architecture facts that are easy to get wrong

- `deploy/nodes/carla_actuator.py` is the **only** caller of `apply_control()`:
  it consumes SI `ApprovedRequest` on domain 2 and contains no watchdog, mode,
  or latch logic. `carla_bridge.py` is telemetry only (camera, odom, steering,
  clock, preview on :8090) and must never actuate.
- One SI binary reads `SI_SUPERVISION_MODE`/`SI_TRAJECTORY_SOURCE` once at
  startup and fails closed on bad values. Do not add runtime fallback,
  mode switching, or an adapter-authored stop; freshness and stops are SI-owned.
- Domains: VP and the adapter on 1, SI and the actuator on 2.
  `deploy/config/bridge-config.yaml` is the complete list of bridged inputs
  (no control topic is bridged back).
- VP is ROS Jazzy/FastDDS; the adapter is Humble/CycloneDDS. Keep VP on
  FastDDS — unifying RMW fails on string deserialization.
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
- The submodule gitlinks are deliberate. `upstream/vision_pilot` is a fork; the
  pinned commit contains the `DrivingCommand` steering-sign fix, the MJPEG
  viewer, and the speed-HUD exposure fix. Don't bump a gitlink without a
  fresh-world validation and a findings entry.
- VP's source stalls (005) and high-speed lane keeping (006/008–010) are open.
  The stable demo operating point is 4 m/s (`vision_pilot.demo*.conf`,
  `vision_pilot.diag*.conf`); do not present it as a fix for high-speed behavior.

## Stale docs

`README.md` and `deploy/README.md` predate the single-binary SI ingress and the
separate actuator; they still describe adapter-authored stops and a
`control_cmd` applied by the bridge. `docs/vp-si-integration-plan.md` is
explicitly historical. When prose conflicts, trust `deploy/run-loop.sh`,
`deploy/docker-compose.yaml`, and `docs/e2e2-stop-gate.md`.