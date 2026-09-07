# openadkit-e2e

L2 closed-loop simulation: **VisionPilot plans, Autoware Safety Island drives, CARLA is the plant, Open AD Kit deploys.**

This repo is the integration layer (adapter, compose, topic contract). Upstream changes belong in forks of those projects and are contributed back via pull request.

## Architecture

```
CARLA 0.9.16 (--ros2)
  camera / odom / steer / accel
        │
        ▼
VisionPilot (local Path publisher)      DDS domain 1
  AutoSpeed 2.0 + AutoSteer 2.0 + AutoDrive
  → /vehicle/lane_path (nav_msgs/Path)
        │
        ▼
adapter (this repo)
  Path → autoware_planning_msgs/Trajectory
        │
        ▼
domain_bridge 1 → 2
        │
        ▼
Autoware Safety Island                  DDS domain 2
  MPC lateral + PID longitudinal
  → /control/trajectory_follower/control_cmd
        │
        ▼
CARLA actuators
```

One controller: SI. Disable VisionPilot actuation when SI is on.

| Piece | Role |
| --- | --- |
| CARLA 0.9.16 | Plant (camera, odom, steer, accel) |
| Open AD Kit | Containers, compose, mixed-criticality deploy |
| VisionPilot 1.0 | Hybrid L2 perception + plan (path, CIPO) |
| Autoware SI | Isolated trajectory follower — not perception |
| This repo | Path→Trajectory adapter, compose, overlays |

Phase 1 is NVIDIA-style SI: independent **compute**, shared camera. Do not claim independent perception.

## Topic contract (phase 1)

SI subscriptions (domain 2):

| Topic | Type | Source |
| --- | --- | --- |
| `/planning/scenario_planning/trajectory` | `autoware_planning_msgs/msg/Trajectory` | adapter from VP `/vehicle/lane_path` |
| `/localization/kinematic_state` | `nav_msgs/msg/Odometry` | CARLA odom |
| `/localization/acceleration` | `geometry_msgs/msg/AccelWithCovarianceStamped` | CARLA |
| `/vehicle/status/steering_status` | `autoware_vehicle_msgs/msg/SteeringReport` | CARLA |
| `/system/operation_mode/state` | `autoware_adapi_v1_msgs/msg/OperationModeState` | stub `AUTONOMOUS` |

SI publication: `/control/trajectory_follower/control_cmd` (`autoware_control_msgs/msg/Control`) → CARLA.

## Phases

1. **Ship:** VP plans, SI drives. Adapter is the work. SI on `freertos-posix`, then FVP.
2. RSS/CIPO selector may clip the trajectory before the bridge (same camera, different rules).
3. Optional CARLA radar as a diverse CIPO check. Not a CES blocker.

Do not run VP control and SI control together. Do not feed SI Ackermann. Do not put AutoE2E in this loop.

## Adapter

`adapter/path_to_trajectory.py` converts VP `/vehicle/lane_path` (`base_link`) to an SI-sized `/planning/scenario_planning/trajectory` (odom/map, ≤13 points, 25 m, ≤1200 B).

```bash
python3 -m unittest discover -s adapter -v
cd deploy && docker compose --env-file config.env up
./run-si.sh
```

See `deploy/README.md`.

## Upstream

Git submodules under `upstream/` pin the SHAs this integration is built against:

- [vision_pilot](https://github.com/autowarefoundation/vision_pilot)
- [openadkit](https://github.com/autowarefoundation/openadkit)
- [autoware-safety-island](https://github.com/autowarefoundation/autoware-safety-island)

```bash
git submodule update --init --recursive
```

CARLA is the `carlasim/carla:0.9.16` image, not a git checkout.
