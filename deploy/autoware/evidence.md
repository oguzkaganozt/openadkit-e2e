# Town04 Autoware → SI_CONTROL smoke evidence

Run on the shared VPS rig, 2026-09-25 ~00:10 UTC. Fresh world and all rig
containers were recreated by:

```bash
RIG_MODE=autoware SI_MODE=si PREVIEW_AUTO=0 ./deploy/run-loop.sh --gpu
```

## Pinned inputs and topology

| Input | Value |
| --- | --- |
| CARLA server | `carlasim/carla:0.9.16@sha256:aaf1df22702780ece072069e23d03c4879b002ae028c79744b09c4c7ddbae953` |
| CARLA Python API wheel | `carla-0.9.16-cp310-cp310-manylinux_2_31_x86_64.whl`; SHA-256 `52b1f2fafb0655e25954f9f6d1e97c211a4da404217fd1d0094b4b7350737c95` (`deploy/build.sh`) |
| Autoware image | `ghcr.io/autowarefoundation/autoware:universe-20250207@sha256:5482c148addbd13c005e86452fc9c40502a8c87759f679b8b61d83943059ada7` |
| SI_CONTROL binary | SHA-256 `3ca8374c6b5eee2b4448913a77a4db5439f04264c0127bb61861d2243b1641b0`; built from `build/freertos-posix-si` (two-binary compile-time mode, since superseded by the single binary with startup configuration) |
| World and ego | CARLA Town04, spawn index 184, road 45 / lane -4; `vehicle.lincoln.mkz_2020` role `hero` |
| CARLA clock | Synchronous ticks, `fixed_delta_seconds=0.05` in `deploy/nodes/scenario.py`; ROS `/clock` from `carla_bridge.py` |
| Scenario | `deploy/config/carla-rig-empty.json` (no NPCs, no lead vehicle) |
| Lanelet2 map | CARLA Autoware Contents Town04 (inverted y, `projector_type: Local`); SHA-256 `41e9149cfc74621d48a4af22b66fca90a12e51792779913106c9ac2ab73019ef` |
| PCD map | CARLA Autoware Contents Town04; SHA-256 `9703b93441a5f5854b6e7cbb31a4d0d4043fe1080b57dfd5c784d5095f811593` |
| Route | Initial map pose approximately `(-515.25, -240.96)`, yaw `-89.9°`; lanelet 21693 goal `(-460.1, -376.6)`, yaw `-45.9°`; `deploy/autoware/set_route.py` |
| Planning launch | `deploy/autoware/planning_only.launch.xml`: map + planning on, vehicle/system/sensing/localization/perception/control/API off; `use_sim_time=true`, simulation auto mode on |

The empty-world-only fixture in `deploy/nodes/empty_scene.py` provided
empty PredictedObjects, a free 400 × 400 m occupancy grid and empty point
clouds with CARLA-frame timestamps. It refuses a scenario config with NPCs
or a lead vehicle. CARLA ground-truth odometry and `odom_to_tf.py` supplied
localization and `map → base_link` TF; operation-mode state was published
TRANSIENT_LOCAL to satisfy the Autoware behavior planner.

## Observations

- Mission planner accepted the route after map load (`getMainLanelets: [ids:
  21693]`). Autoware produced a **121-point** final trajectory. Startup
  waited for this real trajectory before starting SI; no synthetic route or
  trajectory was published.
- Runtime domain-1 graph audit (`deploy/autoware/audit_graph.py`) **PASS**:
  sole publisher of `/planning/scenario_planning/trajectory` was
  `/planning/planning_validator`; `/vehicle/driving_reference` had zero
  publishers; **zero** publishers on `/control/trajectory_follower/control_cmd`,
  `/control/command/control_cmd` and `/control/command/actuation_cmd`. The VP
  and adapter containers were absent from `docker ps`. On domain 2 the DDS
  bridge was the sole trajectory publisher, the bare-DDS SI was its
  subscriber, and `carla_actuator.py` is the only process that writes CARLA
  controls, consuming `ApprovedRequest` directly.
- Domain-2 `deploy/tools/si_probe.py`, 35 s: **234 ApprovedRequest messages,
  234 NORMAL, 0 SI_STOP, 0 HOLD**, `mode=0` (SI_CONTROL),
  `selected_source=0` (FOLLOWER), one SI session, sequence gaps 0,
  backwards 0, fault IDs none. `FOLLOWER` identifies the controller, not
  the trajectory vendor; the exclusive graph publisher and process profile
  establish that this run selected Autoware.
- CARLA ground truth from scenario: first `(x=-515.2, y=241.0)`, final
  `(x=-460.4, y=376.2)`, net displacement **145.9 m** toward the map goal
  `(x=-460.1, y=-376.6)` (map y is inverted). Maximum speed **4.66 m/s**;
  maximum absolute road-center offset **0.07 m**. At the goal speed was
  **0.00 m/s** and the bridge applied throttle `0.000` / brake `0.400` on
  a `cmd_v=0.00` SI control message. No SI fault latch appeared in the run.

Raw run logs were saved on the VPS at `/root/si-autoware-evidence-20260925/`
and copied to `/tmp/opencode/si-autoware-evidence-20260925/` locally before
the rig was shut down.

## Limits / remaining gates

Re-validated on a fresh world after E2E #2 landed: the CARLA controls came
from the ApprovedRequest-only single-writer actuator, the domain-1 audit
showed no control writers of any kind, a 30 s domain-2 probe recorded 199
NORMAL decisions (mode=0, selected_source=0) and stopping the
`autoware-planning` container produced an SI latch and a CARLA-applied stop
within **8 ms** of SI detection (see `docs/e2e2-stop-gate.md`, "SI_CONTROL +
Autoware" row). Empty-world perception remains a simulation fixture, not a
real-world perception solution, and the prebuilt OSM still logs a missing
`format_version` warning despite loading and routing successfully.
