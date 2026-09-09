# Deployment guide

Run VisionPilot, Safety Island, and CARLA as a closed-loop Compose stack.
See the [main README](../README.md#how-it-works) for the architecture.

## Setup and build

Run commands from the repository root unless a block starts with `cd`.

1. Use Ubuntu x86-64 with a working NVIDIA driver (`nvidia-smi`), sudo, Git,
   curl, and Python 3.10. The CARLA wheel requires CPython 3.10.
2. Run `./deploy/setup.sh` once to install Docker, Compose, Python venv support,
   and the NVIDIA container runtime. Log out and back in if prompted.
3. Build with your multicast-capable network interface:

   ```bash
   ./deploy/build.sh --dds-interface ens3
   ```

The build initializes submodules, verifies and downloads the CARLA 0.9.16 wheel
to `/tmp/`, builds VisionPilot GPU and Safety Island, pulls runtime images, and
builds the DDS domain bridge. Add `--run` to start the loop after building.

## Start, inspect, and stop

```bash
./deploy/run-loop.sh
```

The script starts the full stack and checks that paths, trajectories, and control
commands are flowing. View the camera at **<http://127.0.0.1:8090/>**.

For service status, logs, and shutdown:

```bash
cd deploy
docker compose --env-file config.env --profile vp ps
docker compose --env-file config.env --profile vp logs -f
docker compose --env-file config.env --profile vp down
```

## Configuration

| File | Settings |
| --- | --- |
| [`config.env`](config.env) | Image pins, container runtime, and ROS defaults |
| [`config/vision_pilot.conf`](config/vision_pilot.conf) | VisionPilot inference provider |
| [`config/vision_pilot.carla.conf`](config/vision_pilot.carla.conf), [`config/H.yaml`](config/H.yaml) | VisionPilot ROS topics and camera calibration |
| [`config/carla-rig.json`](config/carla-rig.json) | Vehicle and camera rig |
| [`config/bridge-config.yaml`](config/bridge-config.yaml), [`config/cyclonedds.xml`](config/cyclonedds.xml) | DDS topic routing and networking |
| [`docker-compose.yaml`](docker-compose.yaml) | Services and mounts; Python processes live in [`nodes/`](nodes/) |

Keep `config.env` compatible with both Bash and Compose; `build.sh` sources it.
To override image pins, export `CARLA_IMAGE`, `AUTOWARE_IMAGE`, or `SI_BUILD_IMAGE`
before building. Keep runtime overrides consistent when starting the loop.
CARLA overrides must match the verified 0.9.16 Python wheel; do not enable
`--ros2`, because native control is broken in this version.

### VisionPilot CPU or GPU

CARLA always needs NVIDIA. To change VisionPilot inference, match all three settings:

| Setting | GPU (default) | CPU |
| --- | --- | --- |
| `VISIONPILOT_IMAGE` in `config.env` | `visionpilot:gpu-ros2` | `visionpilot:cpu-ros2` |
| `VISIONPILOT_RUNTIME` in `config.env` | `nvidia` | `runc` |
| `engine.provider` in `config/vision_pilot.conf` | `cuda` | `cpu` |

The deployment build creates the GPU image. For CPU inference, build the CPU image:

```bash
cd upstream/vision_pilot/VisionPilot/docker
./build.sh --cpu --ros2
```

Then start the loop again. CPU path publication is slower than the 10 Hz camera.

## Runtime reference

- **Adapter:** converts `/vehicle/lane_path_relay` from `base_link` to a `map`
  trajectory, targeting 3 m/s with up to 13 points, a 25 m extent budget, and ≤1200 B.
- **Stop behavior:** an empty path produces a 0 m/s trajectory. The CARLA bridge
  drops control commands older than 0.5 s.
- **CARLA bridge:** uses Python RPC on port 2000 and maps Safety Island's velocity
  and acceleration commands to throttle using feedforward and speed error.
- **Domains:** VisionPilot and the adapter use domain 1; Safety Island uses domain 2.
  The UDP relay transfers paths from Jazzy to Humble; the DDS bridge connects domains.

### Safety Island topics

Inputs are bridged from domain 1 to domain 2:

| Topic | Type | Source |
| --- | --- | --- |
| `/planning/scenario_planning/trajectory` | `autoware_planning_msgs/msg/Trajectory` | Adapter |
| `/localization/kinematic_state` | `nav_msgs/msg/Odometry` | CARLA bridge |
| `/localization/acceleration` | `geometry_msgs/msg/AccelWithCovarianceStamped` | CARLA bridge |
| `/vehicle/status/steering_status` | `autoware_vehicle_msgs/msg/SteeringReport` | CARLA bridge, measured steering |
| `/system/operation_mode/state` | `autoware_adapi_v1_msgs/msg/OperationModeState` | `AUTONOMOUS` stub |

Output: `/control/trajectory_follower/control_cmd` (`autoware_control_msgs/msg/Control`),
bridged back to domain 1 and applied by the CARLA bridge.

## Development

Run adapter tests without ROS:

```bash
python3 -m unittest discover -s adapter -v
```

For an SI-only run, omit `--profile vp` from Compose startup. In a ROS 2 Humble
environment with `ROS_DOMAIN_ID=1`, run `python3 deploy/nodes/fake_path.py`
to feed a synthetic path to the adapter.

To rebuild Safety Island directly in its build environment:

```bash
cd upstream/autoware-safety-island
./build.sh --platform freertos-posix -d build/freertos-posix --control-output DDS_ONLY --dds-interface ens3
```
