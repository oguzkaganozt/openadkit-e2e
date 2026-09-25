# Autoware Universe → SI_CONTROL on the Town04 rig

This configuration replaces VisionPilot's trajectory candidate with the
Autoware Universe planning output. It **does not** run Autoware control,
vehicle interface, localization, sensing, perception, or the dummy planning
simulator. CARLA 0.9.16 is the plant; `carla_bridge.py` publishes ground-truth
odometry and `/clock`, `odom_to_tf.py` broadcasts `map → base_link`, and the SI
follower receives Autoware's `/planning/scenario_planning/trajectory` through
the DDS domain bridge (domain 1 → 2). The VP adapter and VP process are absent.
Because planning waits for perception inputs even on an empty road,
`empty_scene.py` publishes empty `PredictedObjects`, free occupancy grid
(400 × 400 m around the bounded 150 m route), and empty point clouds with
CARLA odometry-frame stamps **only** when the scenario JSON has no NPCs or lead
vehicle. This is a simulation-only fixture, not perception; do not use it
around other actors or extend the route beyond the grid without replacing it.

The maps are the y-axis-inverted Town04 Lanelet2 and pointcloud maps from
[CARLA Autoware Contents](https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/),
verified by SHA-256 in `deploy/tools/fetch-town04-map.sh`. With
`projector_type: Local`, their coordinates agree with the bridge's CARLA
`(x, -y)` map frame. The route tool targets lanelet 21693 at about 150 m
from spawn 184: `(-460.1, -376.6)`, yaw `-45.9°` in the map frame.

On the rig host (after deploying this repository and the SI_CONTROL binary):

```bash
./deploy/tools/fetch-town04-map.sh  # ~557 MB, skipped if hashes match
RIG_MODE=autoware SI_MODE=si ./deploy/run-loop.sh --gpu
```

`run-loop.sh` stops/removes all previous rig containers, recreates CARLA and
the world from scratch, then starts map/planning, TF, and the empty-scene
fixture. It calls the mission
planner's `set_waypoint_route` service, waits for a nonempty final trajectory,
and only then starts SI. A route or trajectory failure **fails the attempt**;
there is no synthetic candidate or automatic fallback to VP.

Use a lead-free world (`carla-rig-empty.json`, the default for this mode)
because no perception is running. `run-loop.sh` requires the distinct
`build/freertos-posix-si/actuation_freertos` SI binary and stages it separately
from `build/freertos-posix-vp/actuation_freertos`; neither mode overwrites the
other's build output.

**Current limit:** CARLA control still goes through the legacy bridge's
`control_cmd` subscriber. The separate ApprovedRequest-only actuator (E2E #2)
is required before this run demonstrates final single-writer authority or
the ≤500 ms applied-stop gate. Treat the present run as a planning/SI source
smoke test, not that timing proof.
