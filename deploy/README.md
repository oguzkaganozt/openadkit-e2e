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

```bash
# 0. once per Ubuntu GPU host: Docker, Compose, python3-venv, NVIDIA runtime
./deploy/setup.sh

# 1. fresh GPU host: build/download all pinned runtime artifacts
./deploy/build.sh --dds-interface ens3

# 2. adapter tests
python3 -m unittest discover -s adapter -v

# 3. stack (domain 1)
cd deploy
docker compose --env-file config.env up

# 4. one clean drive: spawn ego, wait for a Trajectory, then start SI (domain 2)
./run-loop.sh

# optional VP + path relay
docker compose --env-file config.env --profile vp up
```

`setup.sh` installs Docker, Compose, python3-venv, and the NVIDIA container runtime on
Ubuntu. Keep it separate from `build.sh`: host packages need sudo and should run once,
while `build.sh` rebuilds repo artifacts. `build.sh` requires Git, curl, Python 3 with venv, Docker Compose, an NVIDIA driver, and the
Docker NVIDIA runtime. It verifies the official CARLA 0.9.16 CPython 3.10 wheel,
builds `visionpilot:gpu-ros2`, builds the Safety Island in its pinned devcontainer,
pulls the runtime images, and builds the domain bridge. Pass `--run` to start the
loop after a successful build.

Plant talks to CARLA over the Python API (RPC :2000). Do not pass `--ros2` to CARLA — native control is broken on 0.9.16. The build stages the matching CARLA wheel in `/tmp/`. `run-loop.sh` keeps exactly one SI process alive. SI posix needs `--dds-interface` on a multicast-capable NIC.

The spawn client advances CARLA synchronously at a requested 20 Hz, while the 1920x1280 rig camera runs at 10 Hz. The adapter publishes a fixed 3 m/s target. Plant maps SI velocity/acceleration to CARLA throttle with a cruise feedforward plus speed error term. The rig camera preview is available from the plant on port 8090.

Build SI:

```bash
cd upstream/autoware-safety-island
./build.sh --platform freertos-posix -d build/freertos-posix --control-output DDS_ONLY --dds-interface <nic>
```
