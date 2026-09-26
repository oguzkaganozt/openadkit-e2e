# Deployment guide

Run VisionPilot or Autoware, the Safety Island and CARLA as one closed-loop
Compose stack. See the [main README](../README.md#how-it-works) for the
architecture; the contract and its evidence are in
[`docs/e2e2-stop-gate.md`](../docs/e2e2-stop-gate.md).

## Setup and build

Run commands from the repository root unless a block starts with `cd`.

1. Ubuntu x86-64 with a working NVIDIA driver (`nvidia-smi`), sudo, Git, curl
   and Python 3.10 (the CARLA wheel is CPython 3.10).
2. `./deploy/setup.sh` once per host: Docker, Compose, python3-venv, the NVIDIA
   container runtime, and persistent socket-buffer limits for DDS
   (`/etc/sysctl.d/60-openadkit-e2e-dds.conf`, 64 MiB max / 16 MiB default;
   finding 012). Log out and back in if prompted.
3. Autoware profile only: `./deploy/tools/fetch-town04-map.sh` (the map is
   gitignored; `run-loop.sh` verifies its checksums).
4. Build with a multicast-capable interface:

   ```bash
   ./deploy/build.sh --dds-interface ens3
   ```

The build initializes the pinned submodules, downloads and verifies the CARLA
0.9.16 wheel into `/tmp/` and `/tmp/carla-venv`, builds `visionpilot:gpu-ros2`
from the fork pin, builds the SI binary
(`upstream/autoware-safety-island/build/freertos-posix/actuation_freertos`),
the adapter and domain-bridge images (the adapter's base is the Autoware
runtime image), and pulls CARLA. Add `--run` to start a loop afterwards.

## Run

Every `run-loop.sh` call tears the stack down, force-recreates CARLA, starts
the selected services, waits for each stage to be healthy and starts the SI
last — one fresh world per call.

```bash
./deploy/run-loop.sh --gpu                                     # VP → SI_CONTROL
RIG_MODE=vp SI_MODE=vp ./deploy/run-loop.sh --gpu              # VP_CONTROL
RIG_MODE=autoware SI_MODE=si ./deploy/run-loop.sh --gpu        # Autoware → SI_CONTROL
```

| Variable | Default | Meaning |
| --- | --- | --- |
| `RIG_MODE` | `vp` | Planner and SI trajectory source: `vp` or `autoware` |
| `SI_MODE` | `si` | `si` = SI_CONTROL (SI follows), `vp` = VP_CONTROL (VP command passthrough); `autoware` + `vp` is rejected |
| `RIG_JSON` | `carla-rig.json` (vp), `carla-rig-empty.json` (autoware) | CARLA rig in `config/` |
| `SPAWN_INDEX` | `184` | Town04 spawn point |
| `VISIONPILOT_IMAGE` / `_CONF` / `_RUNTIME` | `visionpilot:gpu-ros2`, `vision_pilot.conf`, `nvidia` | VP build, config and runtime |
| `BOTH_SOURCES` | `0` | `1` also starts the unselected planner (isolation test; needs `SI_MODE=si` and the empty rig) |
| `ACTUATOR_STEER_CURVE_COMP` | `1` | `0` = legacy `steer = tire / max_steer` (finding 013) |
| `DDS_BUFFER_CHECK` | `1` | `0` skips the socket-buffer preflight — measurements are then not trustworthy |
| `PREVIEW_HOST` / `PREVIEW_AUTO` | auto | Camera preview link (`PREVIEW_AUTO=0` skips public-IP detection) |

The camera preview is <http://127.0.0.1:8090/>; with a viewer config
(`visualization_on = true`) VP's HUD is on <http://127.0.0.1:8080/>.

Inspect and stop (use the profile you ran, `vp` or `autoware`):

```bash
cd deploy
docker compose --env-file config.env --profile vp ps
docker compose --env-file config.env --profile vp logs -f si carla-actuator
docker compose --env-file config.env --profile vp down
```

A latched SI stays in SI_STOP. Clear it with an explicit re-enable once every
source is fresh, or start a fresh world; never repeat an injected fault in the
same world:

```bash
docker run --rm --network host --ipc host --entrypoint bash \
  -e ROS_DOMAIN_ID=2 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///autoware/cyclonedds.xml \
  -v "$PWD/deploy/config/cyclonedds.xml:/autoware/cyclonedds.xml:ro" \
  openadkit-e2e-adapter:latest -lc 'source /opt/ros/humble/setup.bash &&
  ros2 topic pub --once /control/safety_island/reenable std_msgs/msg/Bool "{data: true}"'
```

(Same container setup as `tools/run-clean-vp-restart.sh`, whose
`vp_restart_probe.py` sends the re-enable in the recorded evidence.)

## Evidence and measurement tools

All in `deploy/tools/`; each script's header documents its gate or output.

| Tool | Use |
| --- | --- |
| `run-evidence.sh <label> [sec]` | Fresh world + steering trace + 1 Hz frames + every container log in one directory |
| `frame_sampler.py` | 1 Hz front camera, VP HUD and chase-camera images with a wall-ms index |
| `passive_steering_probe.py` | Read-only 20 Hz CARLA trace: pose, lane offset, commanded and actual wheel angles |
| `stop-gate-test.sh`, `run-clean-gate.sh` | Source-cut injection and the 500 ms applied-stop gate |
| `run-clean-vp-replay.sh`, `run-clean-vp-restart.sh` | Identity faults, latch/re-enable, VP restart |
| `record-example.sh`, `package-example.sh` | Example clips for `docs/media/` |
| `image_stream_observer.py`, `measure-cadence.sh` | Camera-stream and VP cadence gaps |

Run the CARLA-side probes with `/tmp/carla-venv/bin/python`, **after**
`run-loop.sh` returns: it recreates CARLA, and an earlier client never sees the
new hero.

## Configuration

| File | Settings |
| --- | --- |
| [`config.env`](config.env) | Image pins, container runtime, ROS/RMW defaults (sourced by Bash and Compose) |
| [`config/vision_pilot.conf`](config/vision_pilot.conf), [`.cpu.conf`](config/vision_pilot.cpu.conf) | Default VP config (GPU / CPU), 33.3 m/s limit, tuned `fusion.lat.*` |
| [`config/vision_pilot.demo.conf`](config/vision_pilot.demo.conf), [`.demo-quiet.conf`](config/vision_pilot.demo-quiet.conf), [`.viewer.conf`](config/vision_pilot.viewer.conf) | 4 m/s example clips (with / without fusion logs); HUD viewer |
| `config/vision_pilot.diag4.conf`, `.diag9.conf`, `.diag9-latfast.conf` | A/B configs of findings 008–013 (diag4/diag9 keep the old lateral filter on purpose) |
| [`config/vision_pilot.carla.conf`](config/vision_pilot.carla.conf) | VP ROS topics on the rig |
| [`config/H.yaml`](config/H.yaml) + [`config/homography_C_matrix.yaml`](config/homography_C_matrix.yaml) | Camera homography; a **matched pair** — regenerate C with `tools/gen-homography-c.sh` after any H change |
| `config/carla-rig*.json` | Rigs: `carla-rig` (lead that brakes to a stop, 3 NPCs), `-empty`, `-slow-lead` (4.09 m/s lead), `-traffic` (14 NPCs), `-autoware-demo` |
| [`config/bridge-config.yaml`](config/bridge-config.yaml), [`config/cyclonedds.xml`](config/cyclonedds.xml) | Domain 1 → 2 topics (the complete list) and CycloneDDS networking |
| [`docker-compose.yaml`](docker-compose.yaml) | Services and mounts; Python rig nodes live in [`nodes/`](nodes/) |

To override image pins, export `CARLA_IMAGE`, `AUTOWARE_IMAGE` or
`SI_BUILD_IMAGE` before building and keep them when running. CARLA must match
the verified 0.9.16 wheel; do not enable its `--ros2` (finding 004).

### Compute mode: GPU or CPU

`setup.sh`, `build.sh` and `run-loop.sh` share `COMPUTE=cpu|gpu` (default
`auto`: GPU when `nvidia-smi` works); `--cpu` / `--gpu` are shorthand. CPU
mode builds `visionpilot:cpu-ros2` and runs it with `runc` and
`config/vision_pilot.cpu.conf`. CARLA itself still needs an NVIDIA GPU — CPU
mode only switches VP inference (~2.5 Hz on a 28-core host vs 10 Hz on GPU).
Mixed mode on a GPU host:

```bash
./deploy/build.sh --dds-interface ens3 --cpu
VISIONPILOT_IMAGE=visionpilot:cpu-ros2 VISIONPILOT_RUNTIME=runc \
VISIONPILOT_CONF=vision_pilot.cpu.conf CARLA_RUNTIME=nvidia ./deploy/run-loop.sh
```

## Runtime reference

| Service | Profile | Role |
| --- | --- | --- |
| `carla` | all | CARLA 0.9.16 server |
| `scenario` | all | Owns the world: synchronous ticks, ego/NPC/lead actors, 1 Hz ground-truth pose, gap and collision log |
| `carla-bridge` | all | Telemetry only (domain 1): camera, odometry, acceleration, steering report, speed, `/clock`, preview on :8090. Never actuates |
| `carla-actuator` | all | Domain 2: sole `apply_control()` caller, maps each SI `ApprovedRequest` (NORMAL / SI_STOP / HOLD) to CARLA control |
| `domain-bridge` | all | Domain 1 → 2 for the topics in `bridge-config.yaml` only |
| `operation-mode` | all | `AUTONOMOUS` operation-mode stub for the SI |
| `si` | all | The SI binary; `SI_SUPERVISION_MODE` / `SI_TRAJECTORY_SOURCE` from `SI_MODE` / `RIG_MODE`, read once, fail closed |
| `visionpilot` | vp | VP (Jazzy, FastDDS): `/vehicle/driving_command`, `/vehicle/driving_reference` |
| `adapter` | vp | Accepts a valid `DrivingReference` whose `source_stamp` matches an ego sample, publishes a ≤ 1300 B `TrajectoryCandidate`; rejects (never stops) otherwise |
| `autoware-planning`, `odom-to-tf`, `empty-scene` | autoware | Autoware planning-only launch on Town04 with the empty-scene fixture |

### Safety Island interfaces

Inputs, bridged from domain 1 to domain 2:

| Topic | Type | Source |
| --- | --- | --- |
| `/planning/visionpilot/trajectory_candidate` | `safety_island_msgs/msg/TrajectoryCandidate` | Adapter (SI_CONTROL, `RIG_MODE=vp`) |
| `/planning/scenario_planning/trajectory` | `autoware_planning_msgs/msg/Trajectory` | Autoware (SI_CONTROL, `RIG_MODE=autoware`) |
| `/vehicle/driving_command` | `visionpilot_msgs/msg/DrivingCommand` | VisionPilot (VP_CONTROL) |
| `/localization/kinematic_state` | `nav_msgs/msg/Odometry` | CARLA bridge |
| `/localization/acceleration` | `geometry_msgs/msg/AccelWithCovarianceStamped` | CARLA bridge |
| `/vehicle/status/steering_status` | `autoware_vehicle_msgs/msg/SteeringReport` | CARLA bridge (measured) |
| `/system/operation_mode/state` | `autoware_adapi_v1_msgs/msg/OperationModeState` | `AUTONOMOUS` stub |

On domain 2 the SI publishes `/control/safety_island/approved_request`
(`safety_island_msgs/msg/ApprovedRequest`, consumed by `carla-actuator`) and
listens on `/control/safety_island/reenable` (`std_msgs/msg/Bool`). Nothing is
bridged back to domain 1.

Discovery needs a multicast-capable interface (`build.sh --dds-interface`);
on weak-multicast networks topic discovery can be slow or flaky.

## Development

Adapter tests, no ROS needed:

```bash
python3 -m unittest discover -s adapter -v
```

Synthetic VP reference without VisionPilot: in the adapter image with
`ROS_DOMAIN_ID=1`, run `python3 deploy/nodes/fake_reference.py` (it stamps
each reference with the latest ego frame so the adapter accepts it).

Rebuild the Safety Island directly in its build environment (what `build.sh`
runs):

```bash
cd upstream/autoware-safety-island
./build.sh --platform freertos-posix -d build/freertos-posix --control-output DDS_ONLY --dds-interface ens3
```
