# Deploy

```
CARLA --ros2
  camera → VisionPilot → /vehicle/lane_path
                         adapter → Trajectory
  odom/camera → plant_bridge (CARLA Python API) → kinematic_state / speed / image
  operation_mode stub → AUTONOMOUS
  domain_bridge 1↔2
  SI posix (host) → control_cmd → plant_bridge → CARLA RPC
  Jazzy /vehicle/lane_path → UDP relay → Humble /vehicle/lane_path_relay
```

VP `steering_cmd` is not connected to CARLA.

```bash
# 1. adapter tests
python3 -m unittest discover -s adapter -v

# 2. stack (domain 1)
cd deploy
docker compose --env-file config.env up

# 3. SI on the host (domain 2)
./run-si.sh

# optional VP + path relay
docker compose --env-file config.env --profile vp up
./run-spawn.sh
```

Plant talks to CARLA over the Python API (RPC :2000), not native ROS 2. Copy the matching `carla-0.9.16-cp310` wheel to `/tmp/` before `compose up`. SI posix needs `--dds-interface` on a multicast-capable NIC.

Build SI:

```bash
cd upstream/autoware-safety-island
./build.sh --platform freertos-posix -d build/freertos-posix --control-output DDS_ONLY
```
