# Deploy

```
CARLA --ros2
  camera → VisionPilot → /vehicle/lane_path
                         adapter → Trajectory
  odom   → plant_bridge → kinematic_state / speed / steer report
  operation_mode stub → AUTONOMOUS
  domain_bridge 1↔2
  SI posix (host) → control_cmd → plant_bridge → CARLA ackermann
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

# optional VP container
docker compose --env-file config.env --profile vp up
```

CARLA must publish `/carla/hero/odometry` and accept `/carla/hero/ackermann_control_cmd`. Override plant params if your rig differs.

Build SI:

```bash
cd upstream/autoware-safety-island
./build.sh --platform freertos-posix -d build/freertos-posix --control-output DDS_ONLY
```
