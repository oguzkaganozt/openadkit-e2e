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
# 1. adapter tests
python3 -m unittest discover -s adapter -v

# 2. stack (domain 1)
cd deploy
docker compose --env-file config.env up

# 3. one clean drive: spawn ego, wait for a Trajectory, then start SI (domain 2)
./run-loop.sh

# optional VP + path relay
docker compose --env-file config.env --profile vp up
```

Plant talks to CARLA over the Python API (RPC :2000). Do not pass `--ros2` to CARLA — native control is broken on 0.9.16. Copy the matching `carla-0.9.16-cp310` wheel to `/tmp/` before `compose up`. `run-loop.sh` keeps exactly one SI process alive. SI posix needs `--dds-interface` on a multicast-capable NIC.

The spawn client advances CARLA synchronously at a requested 20 Hz, and the adapter publishes a fixed 3 m/s target. The rig camera preview is available from the plant on port 8090.

Build SI:

```bash
cd upstream/autoware-safety-island
./build.sh --platform freertos-posix -d build/freertos-posix --control-output DDS_ONLY --dds-interface <nic>
```
