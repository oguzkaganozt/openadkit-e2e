# openadkit-e2e

L2 closed-loop simulation: **VisionPilot plans, Autoware Safety Island drives, CARLA is the plant.**

This repo **is** an Open AD Kit deployment: mixed-criticality Compose (CARLA plant, VP planning, isolated SI follower). It does not vendor the `openadkit` git tree; the stack lives here. The only forked upstream is VisionPilot (`feat/lane-path`), which publishes `/vehicle/lane_path`. Safety Island is a pinned Autoware Foundation checkout.

## Architecture

```
CARLA 0.9.16 (no --ros2)
  scenario → hero + sensors + synchronous tick
  carla_bridge (Python RPC :2000) → image / odom / apply_control
        │
        ▼
VisionPilot                              DDS domain 1 (Jazzy)
  → /vehicle/lane_path
        │
        ▼
UDP relay → adapter
  Path → Trajectory (3 m/s, ≤13 points)
        │
        ▼
domain_bridge 1 ↔ 2
        │
        ▼
Safety Island (FreeRTOS POSIX)           DDS domain 2
  → /control/trajectory_follower/control_cmd
        │
        ▼
carla_bridge → CARLA
```

One controller: SI. VP `steering_cmd` / `throttle_cmd` are not connected to CARLA.

| Piece | Role |
| --- | --- |
| This repo | Open AD Kit deployment (Compose, adapter, overlays) |
| CARLA 0.9.16 | Plant (RPC, no native ROS) |
| carla_bridge | CARLA image, odom, measured steer, and control I/O |
| VisionPilot | Lane Path in `base_link` |
| UDP relay | Jazzy Path → Humble |
| adapter | Path → SI-sized Trajectory |
| domain_bridge | ROS domain 1 ↔ 2 |
| Safety Island | Trajectory follower |
| scenario | CARLA world, actors, and synchronous 20 Hz tick |

## Run

```bash
./deploy/setup.sh
./deploy/build.sh --dds-interface ens3
python3 -m unittest discover -s adapter -v
./deploy/run-loop.sh
```

`run-loop.sh` starts the full Compose stack (CARLA, scenario, CARLA bridge, VP, relays, adapter, domain bridge, SI). Camera preview: `http://127.0.0.1:8090/`.

Details: `deploy/README.md`.

## VisionPilot CPU or GPU

CARLA stays on NVIDIA. VisionPilot inference can be CUDA or CPU. Set all three together:

| | GPU | CPU |
| --- | --- | --- |
| `VISIONPILOT_IMAGE` in `deploy/config.env` | `visionpilot:gpu-ros2` | `visionpilot:cpu-ros2` |
| `VISIONPILOT_RUNTIME` in `deploy/config.env` | `nvidia` | `runc` |
| `engine.provider` in `deploy/config/vision_pilot.conf` | `cuda` | `cpu` |

Build the matching image once:

```bash
cd upstream/vision_pilot/VisionPilot/docker
./build.sh --gpu --ros2   # visionpilot:gpu-ros2
./build.sh --cpu --ros2   # visionpilot:cpu-ros2
```

Then `./deploy/run-loop.sh`. CPU Path publish rate is lower than the 10 Hz camera.

## Topic contract

SI subscriptions (domain 2):

| Topic | Type | Source |
| --- | --- | --- |
| `/planning/scenario_planning/trajectory` | `autoware_planning_msgs/msg/Trajectory` | adapter from VP Path |
| `/localization/kinematic_state` | `nav_msgs/msg/Odometry` | carla_bridge |
| `/localization/acceleration` | `geometry_msgs/msg/AccelWithCovarianceStamped` | carla_bridge |
| `/vehicle/status/steering_status` | `autoware_vehicle_msgs/msg/SteeringReport` | carla_bridge (measured wheel) |
| `/system/operation_mode/state` | `autoware_adapi_v1_msgs/msg/OperationModeState` | stub `AUTONOMOUS` |

SI publication: `/control/trajectory_follower/control_cmd` → carla_bridge → CARLA.

An empty VP Path becomes a 0 m/s stop Trajectory. The CARLA bridge drops stale `control_cmd` after 0.5 s.

## Known issues

### VisionPilot path instability at lane splits and merges

In CARLA Town04, VisionPilot can briefly alternate between plausible lanes at splits and merges. This can shorten or abruptly change `/vehicle/lane_path`, make the vehicle weave, and cause Safety Island to report `MPC: failed due to getting MPC Data (too large yaw error)`. The path usually recovers without stopping, but if VisionPilot publishes an empty Path, the adapter intentionally sends a 0 m/s stop Trajectory and the vehicle can remain stopped. This is currently treated as a VisionPilot path-selection limitation; the deployment adapter does not mask it.

## Adapter

`adapter/path_to_trajectory.py` converts VP `/vehicle/lane_path` (`base_link`) to `/planning/scenario_planning/trajectory` (map, ≤13 points, 25 m, ≤1200 B, 3 m/s).

## Upstream

Submodules under `upstream/`:

- [vision_pilot](https://github.com/oguzkaganozt/autoware_vision_pilot) (`feat/lane-path`)
- [autoware-safety-island](https://github.com/autowarefoundation/autoware-safety-island)

```bash
git submodule update --init --recursive
```

CARLA is `carlasim/carla:0.9.16`, not a git checkout. Do not pass `--ros2` to CARLA.
