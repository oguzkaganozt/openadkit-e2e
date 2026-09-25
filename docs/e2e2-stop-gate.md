# E2E #2 evidence: single-writer actuator and the 500 ms applied-stop gate

Measured on the new GPU host (vast.ai), 2026-09-25, with the single-writer
actuator. Every measurement below came from its own **freshly recreated rig
and CARLA world** (`run-loop.sh`); a failed or polluted attempt was never
reused.

## Architecture under test

- `deploy/nodes/carla_actuator.py` is the only process that calls
  `vehicle.apply_control()`. It subscribes to the SI's
  `/control/safety_island/approved_request` **directly on domain 2** and maps
  each decision: NORMAL actuates the approved payload, SI_STOP actuates SI's
  explicit stop payload, HOLD keeps the previous payload, unknown is HOLD.
  It has no watchdog brake, speed target, latch or mode logic.
- `deploy/nodes/carla_bridge.py` was reduced to telemetry (camera, ground
  truth odometry, steering report, `/clock`); it has no control subscription,
  and `docker logs openadkit-e2e-carla-bridge | grep -c "applied control"`
  returns 0 in every run.
- `control/trajectory_follower/control_cmd` is no longer bridged to domain 1.

## Measurement

`deploy/tools/stop-gate-test.sh` injects a source outage by stopping the
selected source's container, then reports:

- `detection_latency_ms`: injection wall time → SI latch (SI's own
  `HH:MM:SS.mmm` clock). For multi-node containers this includes the
  container's docker-stop grace period and is **not** an SI property;
- `si_source_watchdog_ms`: the age the SI itself states at detection
  ("arrived X s ago") — the true selected-source watchdog latency;
- `si_to_actuator_transport_ms`: SI latch → actuator `GATE STOP_RECV`;
- `applied_gate_ms`: SI latch → actuator `GATE STOP_APPLIED`, the first
  CARLA frame with throttle ≤ 0.01 and brake > 0.05.

Preflight requires the SI not latched and CARLA ground speed above
`MIN_SPEED_MPS` (read from the scenario's 1 Hz telemetry).

## Results — all three configurations

| Configuration | Injected outage | Preflight speed | injection→latch | SI source watchdog | transport | **applied gate** |
| --- | --- | --- | --- | --- | --- | --- |
| SI_CONTROL + VP trajectory (`RIG_MODE=vp`, `SI_MODE=si`) | stop adapter | 8.22 m/s | 1024 ms | 1.00 s | 1 ms | **8 ms** |
| VP_CONTROL (`SI_MODE=vp`) | stop visionpilot | 1.72 m/s | 1093 ms | 1.04 s | 4 ms | **10 ms** |
| SI_CONTROL + Autoware (`RIG_MODE=autoware`) | stop autoware-planning | 4.07 m/s | 11186 ms* | 1.15 s | 1 ms | **8 ms** |

\* The Autoware container takes ~10 s to shut down gracefully while still
publishing, so the injection→latch number is dominated by docker stop; the
SI latched 1.15 s after the last trajectory sample.

Wire-level machine lines (one per configuration):

```
# SI_CONTROL + VP trajectory, frame 837
[09:50:19.114] | SI_STOP latched: trajectory (fault_id 1)
GATE STOP_RECV  session=1475927726 seq=84 fault=1 wall_ns=1790329819115293371
GATE STOP_APPLIED session=1475927726 seq=84 fault=1 frame=837 ... brake=0.400

# VP_CONTROL, frame 667, mode=1 source=1 proves the selected source is VP
[10:20:33.247] | SI_STOP latched: vp command (fault_id 1)
GATE STOP_RECV  session=1477751009 seq=23 mode=1 source=1 fault=1
GATE STOP_APPLIED session=1477751009 seq=23 mode=1 source=1 fault=1 frame=667 ... brake=0.400

# SI_CONTROL + Autoware, frame 2045
[10:23:31.294] | SI_STOP latched: trajectory (fault_id 1)
GATE STOP_RECV  session=1477881359 seq=341 mode=0 source=0 fault=1
GATE STOP_APPLIED session=1477881359 seq=341 mode=0 source=0 fault=1 frame=2045 ... brake=0.400
```

CARLA ground truth: in the VP-trajectory run the car went `8.88 m/s → 0.01`
within one second of injection; the Autoware run latched from `4.07 m/s` and
braked to standstill.

### VP_CONTROL verbatim passthrough

`deploy/tools/vp_passthrough_check.py` captured VP's
`/vehicle/driving_command` (domain 1) and the SI's `approved_request`
(domain 2) on a fresh VP_CONTROL run and joined them on `(session, cycle)`:

- 20 matched cycles, **0 mismatched fields**,
- max absolute difference **5.9e-08** (float64 → float32 rounding only).

The SI passed VP's steering tire angle, target speed and acceleration
through unchanged, as the contract requires.

### Autoware isolation and supervision

- Domain-1 audit, PASS: sole `/planning/scenario_planning/trajectory`
  writer `/planning/planning_validator`; no `/vehicle/driving_reference`;
  zero publishers on the legacy `control_cmd` and on Autoware's
  `/control/command/*` topics.
- Domain-2 probe, 30 s while driving: 199 ApprovedRequest, all NORMAL,
  one session, sequence gaps 0, `mode=0` (SI_CONTROL), `selected_source=0`
  (FOLLOWER), speed 0.88–4.17 m/s.

## Limits

- The SI stop gate is measured from SI detection; the selected-source
  watchdog (1.0 s nominal) is reported separately, as the contract requires.
- If SI stops publishing entirely, the actuator writes nothing new and CARLA
  keeps the last applied control — no rig stop guarantee when SI is silent.
- VisionPilot stalls (findings 005) caused organic latches around these
  measurements; each recorded injection started from a NORMAL, moving state
  on a fresh world.

Raw logs: `/root/si-gate-evidence-moving-20260925/`,
`/root/si-vpcontrol-evidence-20260925/` and
`/root/si-autoware-evidence-20260925/` on the rig host (local copies under
`/tmp/opencode/`).