# Deploy

```
CARLA (no --ros2)
  spawn overlay → hero + 1920x1280 cam
  plant_bridge (Python API) → image / odom / apply_control
  VisionPilot → /vehicle/lane_path
  UDP relay → adapter → Trajectory
  domain_bridge 1↔2
  SI posix → control_cmd → plant → CARLA
```

VP `steering_cmd` is not connected to CARLA.

## Layout

- `setup.sh`: one-time Ubuntu host setup.
- `build.sh`: download and build runtime artifacts.
- `run-loop.sh`: start the closed loop and check readiness.
- `docker-compose.yaml`: runtime services and mounts.
- `config.env`: shared image pins and ROS environment defaults. Keep it compatible with both Bash and Compose env-file syntax; `build.sh` sources it.
- `config/`: CARLA rig, DDS bridge, and VisionPilot settings and calibration.
- `nodes/`: runtime Python processes, including `spawn.py` for ego spawn and synchronous ticking.

## Run

```bash
# 0. once per Ubuntu GPU host: Docker, Compose, python3-venv, NVIDIA runtime
./deploy/setup.sh

# 1. build/download pinned runtime artifacts
./deploy/build.sh --dds-interface ens3

# 2. adapter tests
python3 -m unittest discover -s adapter -v

# 3. closed loop (CARLA + VP + adapter + SI)
./deploy/run-loop.sh
```

`setup.sh` installs Docker, Compose, python3-venv, and the NVIDIA container runtime on
Ubuntu. Keep it separate from `build.sh`: host packages need sudo and should run once,
while `build.sh` rebuilds repo artifacts. `build.sh` requires Git, curl, Python 3 with venv, Docker Compose, an NVIDIA driver, and the
Docker NVIDIA runtime. It verifies the official CARLA 0.9.16 CPython 3.10 wheel,
builds `visionpilot:gpu-ros2`, builds the Safety Island in its pinned devcontainer,
pulls the runtime images, and builds the domain bridge. Pass `--run` to start the
loop after a successful build.

CARLA, Autoware, and SI build image pins live only in `config.env`. Export
`CARLA_IMAGE`, `AUTOWARE_IMAGE`, or `SI_BUILD_IMAGE` to override them for a build;
use the same runtime overrides when starting the loop. CARLA image overrides must
remain compatible with the verified 0.9.16 Python wheel.

The closed drive loop is `run-loop.sh`. It starts the Compose stack: CARLA, spawn/tick, plant, VisionPilot, relays, adapter, domain bridge, and SI. VP `steering_cmd` is not connected to CARLA.

`docker compose --env-file config.env up` without `--profile vp` is SI-only (no camera planning). For that path run `python3 deploy/nodes/fake_path.py` so the adapter still receives a Trajectory.

Plant talks to CARLA over the Python API (RPC :2000). Do not pass `--ros2` to CARLA — native control is broken on 0.9.16. The build stages the matching CARLA wheel in `/tmp/`. SI posix needs `--dds-interface` on a multicast-capable NIC. An empty VisionPilot Path becomes a 0 m/s stop Trajectory. Plant drops stale `control_cmd` after 0.5 s.

The spawn client advances CARLA synchronously at a requested 20 Hz, while the 1920x1280 rig camera runs at 10 Hz. The adapter publishes a fixed 3 m/s target. Plant maps SI velocity/acceleration to CARLA throttle with a cruise feedforward plus speed error term. The rig camera preview is available from the plant on port 8090.

Build SI:

```bash
cd upstream/autoware-safety-island
./build.sh --platform freertos-posix -d build/freertos-posix --control-output DDS_ONLY --dds-interface <nic>
```
