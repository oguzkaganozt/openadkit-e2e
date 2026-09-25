# E2E #2 evidence: single-writer actuator and the 500 ms applied-stop gate

Measured on the GPU rig on 2026-09-25. Every measurement came from its own
**freshly recreated rig and CARLA world**
(`deploy/tools/run-clean-gate.sh`, which never retries a failed attempt on
the same world); containers, CARLA server and scenario were recreated by
`run-loop.sh` each time.

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

### Final ingress (2026-09-25): one binary, startup source selection

- One SI binary (`build/freertos-posix/actuation_freertos`) reads
  `SI_SUPERVISION_MODE` (`si`/`vp`) and `SI_TRAJECTORY_SOURCE`
  (`autoware`/`vp`) **once at startup**, before any DDS endpoint exists.
  Invalid or missing combinations exit 1. During a run the process never
  changes mode/source and never falls back to the unselected publisher.
- In SI_CONTROL the SI subscribes only to the selected source:
  - native `autoware_planning_msgs/msg/Trajectory` on
    `/planning/scenario_planning/trajectory` (no Autoware-side wrapper); or
  - `safety_island_msgs/msg/TrajectoryCandidate` on
    `/planning/visionpilot/trajectory_candidate`, a standard trajectory plus
    the original VP `source_session`/`source_cycle`.
- A 13-point VP candidate serializes to **1188 B CDR**, within the adapter's
  1300 B budget. Candidate validation ignores duplicates (same cycle or
  repeated camera stamp) without refreshing the freshness watchdog, latches
  SI_STOP on cycle/stamp regression, and keeps retired VP sessions rejected
  after a VP restart.
- A fault latches SI_STOP; clearing it requires the explicit
  `/control/safety_island/reenable` request while every source is fresh
  again. Neither a fault nor a re-enable can change mode or source.

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

Preflight requires the SI not latched, the latest applied decision NORMAL,
and CARLA ground speed above `MIN_SPEED_MPS` (scenario 1 Hz telemetry). The
first applied stop frame must carry the same session/sequence/fault as the
received SI_STOP, so an earlier or unrelated stop cannot be measured.

## Results — final binary (SHA-256 `a5481877…c8c82`)

| Configuration | Publishers active | Injected outage | Preflight speed | injection→latch | SI source watchdog | transport | **applied gate** |
| --- | --- | --- | --- | --- | --- | --- | --- |
| SI_CONTROL + VP candidate (`RIG_MODE=vp` `SI_MODE=si`) | VP + Autoware | stop adapter | 8.95 m/s | 1062 ms | 1.010 s | 2 ms | **14 ms** |
| VP_CONTROL (`SI_MODE=vp`) | VP | stop visionpilot | 1.59 m/s | 866 ms | 1.100 s | 2 ms | **10 ms** |
| SI_CONTROL + Autoware (`RIG_MODE=autoware` `SI_MODE=si`) | VP + Autoware | stop autoware-planning | 4.38 m/s | 11152 ms* | 1.090 s | 2 ms | **6 ms** |

\* The Autoware container takes ~10 s to shut down gracefully while still
publishing, so the injection→latch figure is dominated by `docker stop`; the
SI latched 1.09 s after the last trajectory sample. This latency is reported
separately, as the contract requires.

Wire-level machine lines (one per configuration):

```
# SI_CONTROL + VP candidate, frame 1382
[12:19:39.567] | SI_STOP latched: trajectory (fault_id 1)
GATE STOP_RECV  session=1484886082 seq=98 mode=0 source=0 fault=1 wall_ns=1790338779569750316
GATE STOP_APPLIED session=1484886082 seq=98 mode=0 source=0 fault=1 frame=1382 ... brake=0.400

# VP_CONTROL, frame 642, mode=1 source=1 proves the selected source is VP
[12:29:49.223] | SI_STOP latched: vp command (fault_id 1)
GATE STOP_RECV  session=1485507138 seq=22 mode=1 source=1 fault=1 wall_ns=1790339389225419558
GATE STOP_APPLIED session=1485507138 seq=22 mode=1 source=1 fault=1 frame=642 ... brake=0.400

# SI_CONTROL + Autoware, frame 1691
[12:23:29.582] | SI_STOP latched: trajectory (fault_id 1)
GATE STOP_RECV  session=1485100347 seq=203 mode=0 source=0 fault=1 wall_ns=1790339009584741060
GATE STOP_APPLIED session=1485100347 seq=203 mode=0 source=0 fault=1 frame=1691 ... brake=0.400
```

## Concurrent-publisher isolation

Both final SI_CONTROL worlds ran **both** planners at once
(`BOTH_SOURCES=1`) and `deploy/tools/candidate_isolation_probe.py` observed
the candidate topics and the SI output before and after the injection:

- **VP selected**: before the injection, 61 VP candidates and 81 Autoware
  trajectories were live; 53 NORMAL approvals all carried the live VP
  session/cycle (46 exact matches with observed candidates) and 0 unexpected
  mode/source. After the adapter was stopped, Autoware kept publishing (80
  samples) and the SI emitted only SI_STOP with the stopped VP source's
  identity — it did **not** select or follow Autoware.
- **Autoware selected**: before the injection, `normal_source_ids` was
  exactly `[[0, 0]]` while 41 VP candidates were live — the unselected VP
  input did not reach the SI output. After `autoware-planning` was stopped,
  VP kept publishing (48 samples) and the SI stayed in SI_STOP.

## VP_CONTROL verbatim passthrough

`deploy/tools/vp_passthrough_check.py` captured VP's `/vehicle/driving_command`
(domain 1) and the SI's `approved_request` (domain 2) on a fresh VP_CONTROL
world and joined them on `(session, cycle)`:

- 69 matched cycles, **0 mismatched fields**,
- max absolute difference **4.7e-07** (float64 → float32 rounding only).

The SI passed VP's steering tire angle, target speed and acceleration through
unchanged, as the contract requires. (That world was later discarded when
VisionPilot steered off-lane and stopped the car; the join itself ran while
both streams were healthy.)

## Identity fault injection and VP restart

See `docs/si-ingress-faults.md` for the replayed-cycle latch/re-enable and
VP restart evidence on the same binary.

## Previous architecture (superseded)

The first ingress — adapter published a plain Trajectory to the shared topic,
compile-time mode selection, two binaries — passed the same gates on
2026-09-25 with applied gates of 8 ms (VP trajectory), 10 ms (VP_CONTROL) and
8 ms (Autoware). Those runs are kept only as history; the tables above
supersede them.

## Limits

- The SI stop gate is measured from SI detection; the selected-source
  watchdog (1.0 s nominal) is reported separately, as the contract requires.
- If SI stops publishing entirely, the actuator writes nothing new and CARLA
  keeps the last applied control — no rig stop guarantee when SI is silent.
- VisionPilot stalls (findings 005) caused organic latches around these
  measurements; each recorded injection started from a NORMAL, moving state
  on a fresh world. Discarded worlds (organic latch or a stopped car) are
  kept under the evidence directories and are not part of the table.

Raw logs: `/root/si-ingress-evidence-20260925/` on the rig host
(`both-selected-vp-final`, `vp-control-gate-final`,
`both-selected-aw-final`, `vp-control-final`, `vp-si-gate-ready`,
`autoware-gate-final`, `vp-replay-final`, `vp-restart-final`). The earlier
scan is under `/root/si-gate-evidence-moving-20260925/`,
`/root/si-vpcontrol-evidence-20260925/` and
`/root/si-autoware-evidence-20260925/`.