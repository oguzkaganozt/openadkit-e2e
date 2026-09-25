# E2E #2 evidence: single-writer actuator and the 500 ms applied-stop gate

Run on the new GPU host (vast.ai), 2026-09-25 09:50 UTC, configuration 1
(`RIG_MODE=vp SI_MODE=si`) with the single-writer actuator.

## Architecture under test

- `deploy/nodes/carla_actuator.py` is the only process that calls
  `vehicle.apply_control()`. It subscribes to the SI's
  `/control/safety_island/approved_request` **directly on domain 2** and maps
  each decision: NORMAL actuates the approved payload, SI_STOP actuates SI's
  explicit stop payload, HOLD keeps the previous payload, unknown is HOLD.
  It has no watchdog brake, speed target, latch or mode logic.
- `deploy/nodes/carla_bridge.py` was reduced to telemetry (camera, ground
  truth odometry, steering report, `/clock`); it has no control subscription.
- `control/trajectory_follower/control_cmd` is no longer bridged to domain 1.

## Measurement

`deploy/tools/stop-gate-test.sh` injects a candidate-source outage by stopping
the adapter container, then reads:

- `detection_latency_ms`: injection wall time → SI latch line (its own
  `HH:MM:SS.mmm` clock), i.e. the 1.0 s selected-source watchdog plus SI
  processing — reported separately from the gate, per the shared contract;
- `si_to_actuator_transport_ms`: SI latch → actuator `GATE STOP_RECV`;
- `applied_gate_ms`: SI latch → actuator `GATE STOP_APPLIED`, the first CARLA
  frame with throttle ≤ 0.01 and brake > 0.05 attributable to the SI stop.

Preflight requires the SI not latched and CARLA ground speed above
`MIN_SPEED_MPS` (default 1.0 m/s), read from the scenario's 1 Hz telemetry.

## Result (moving car)

| Metric | Value |
| --- | --- |
| Preflight ground speed | 8.22 m/s |
| **detection_latency_ms** | **1024 ms** (watchdog-bound, reported separately) |
| SI → actuator transport | 1 ms |
| **applied_gate_ms** (SI detection → CARLA-applied stop) | **8 ms** ✅ ≤ 500 ms |

Machine lines:

```
[09:50:19.114] | SI_STOP latched: trajectory (fault_id 1)
GATE STOP_RECV  session=1475927726 seq=84 fault=1 wall_ns=1790329819115293371
GATE STOP_APPLIED session=1475927726 seq=84 fault=1 frame=837 wall_ns=1790329819122653101 throttle=0.000 brake=0.400
```

CARLA ground truth around the injection: `t=28.0 s v=8.88 m/s` →
`t=29.0 s v=0.01 m/s`, then stationary — the applied stop brought the car to
standstill in about one second.

Earlier in the same session the gate was also measured from standstill
(detection 915 ms, transport 2 ms, gate 8 ms); the moving measurement above
supersedes it.

## Single-writer check

`docker logs openadkit-e2e-carla-bridge | grep -c "applied control"`
returned **0**: the telemetry bridge never wrote a control. The actuator's
`applied control #` lines are the only CARLA actuation evidence in the run.

## Limits

- The SI fault-detection latency here is the 1.0 s candidate watchdog by
  design; the gate does not include it.
- If SI stops publishing entirely, the actuator writes nothing new and CARLA
  keeps the last applied control — the shared contract's "no stop guarantee if
  SI/bridge is silent" still holds.
- The run's VisionPilot pipeline stalled repeatedly (see findings 005), which
  caused organic SI latches between measurements; the gate test's preflight
  ensures the recorded injection starts from a driving, unlatch state.

Raw logs: `/root/si-gate-evidence-moving-20260925/` on the rig host (also
copied to the local machine under `/tmp/opencode/`).