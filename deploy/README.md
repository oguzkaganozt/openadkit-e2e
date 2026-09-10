# Deployment guide

Run VisionPilot, Safety Island, and CARLA as a closed-loop Compose stack.
See the [main README](../README.md#how-it-works) for the architecture.

## Setup and build

Run commands from the repository root unless a block starts with `cd`.

1. Use Ubuntu x86-64 with a working NVIDIA driver (`nvidia-smi`), sudo, Git,
   curl, and Python 3.10. The CARLA wheel requires CPython 3.10. For a GPU-less
   host, skip the driver and use `--cpu` (see [compute mode](#compute-mode-gpu-or-cpu)).
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
The final link advertises the public IP when auto-detectable
(override with `PREVIEW_HOST=<ip>`, disable with `PREVIEW_AUTO=0`).

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

### Compute mode: GPU or CPU

All three scripts share one decision, `COMPUTE=cpu|gpu` (default: `auto`,
which uses the GPU when `nvidia-smi` works). Flags are shorthand:

```bash
./deploy/setup.sh --cpu
./deploy/build.sh --dds-interface ens3 --cpu
./deploy/run-loop.sh --cpu
```

CPU mode builds `visionpilot:cpu-ros2` and runs it with the `runc` runtime
and `config/vision_pilot.cpu.conf` (`engine.provider = cpu`); GPU mode uses
`visionpilot:gpu-ros2`, the `nvidia` runtime, and `config/vision_pilot.conf`.
Manual overrides (`VISIONPILOT_IMAGE`, `VISIONPILOT_RUNTIME`,
`VISIONPILOT_CONF`, `CARLA_RUNTIME` exports) still win over the mode defaults.

CPU path publication is slower than the 10 Hz camera (about 2 Hz measured on a
28-core host). Note: CARLA itself still requires an NVIDIA GPU — CPU mode only
switches VisionPilot inference; fully GPU-less single-host operation is not
supported (UE 4.26 is Vulkan-only and crashes on software GL).

### Mixed mode: GPU CARLA + CPU VisionPilot

On a GPU host, build once for CPU and pin CARLA to NVIDIA at run time:

```bash
./deploy/build.sh --dds-interface ens3 --cpu
VISIONPILOT_IMAGE=visionpilot:cpu-ros2 VISIONPILOT_RUNTIME=runc \
VISIONPILOT_CONF=vision_pilot.cpu.conf CARLA_RUNTIME=nvidia ./deploy/run-loop.sh
```

Verified: 10 Hz camera, ~2 Hz CPU planning, trajectory + SI control nominal.

## Runtime reference

- **Adapter:** converts `/vehicle/lane_path` from `base_link` to a `map`
  trajectory, targeting 3 m/s with up to 13 points, a 25 m extent budget, and ≤1200 B.
- **Stop behavior:** an empty path produces a 0 m/s trajectory. The CARLA bridge
  drops control commands older than 0.5 s.
- **CARLA bridge:** uses Python RPC on port 2000 and maps Safety Island's velocity
  and acceleration commands to throttle using feedforward and speed error.
- **Domains:** VisionPilot (ROS 2 Jazzy, FastDDS) and the adapter (ROS 2 Humble,
  CycloneDDS) use domain 1; Safety Island uses domain 2. The adapter subscribes
  to VisionPilot's path directly across the distro/RMW boundary (see known
  limitations in the [main README](../README.md#known-limitation)); the DDS
  bridge connects domains. Discovery needs a multicast-capable interface
  (`build.sh --dds-interface`); on weak-multicast networks (e.g. Wi-Fi without
  multicast on `lo`) topic discovery can be slow or flaky.

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
