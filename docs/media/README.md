# VP–SI example clips

Short H.264 clips from **fresh** CARLA 0.9.16 Town04 worlds. Each clip starts
with normal driving, then deliberately stops the selected planning source.
The labeled transition to CARLA's camera occurs when the source is cut: VP's
own rendered-frame stream necessarily ends when its process is stopped.
CARLA keeps recording so the SI-authorized brake and the stationary car remain
visible. The gate shown in a clip is measured **from SI fault detection to
the first CARLA frame with applied stop**, not from source-cut initiation.

| Startup selection | Clip | Conditions and result |
|---|---|---|
| `RIG_MODE=vp SI_MODE=vp` (VP_CONTROL) | [vp-control.mp4](vp-control.mp4) | VP 4.0 m/s limit; 14 NPCs spawned, ClearNoon / zero fog; lane offset +0.74…+0.81 m in the curve (013); source cut after ~33 s recorded, SI detection→applied stop **12 ms**, no collision. |
| `RIG_MODE=vp SI_MODE=si` (VP→SI) | [vp-si.mp4](vp-si.mp4) | VP 4.0 m/s limit, quiet fusion logging; 14 NPCs spawned, ClearNoon / zero fog; lane offset −0.03…−0.04 m in the same curve; source cut after ~31 s recorded, SI detection→applied stop **12 ms**, no collision. |
| `RIG_MODE=autoware SI_MODE=si` | [autoware-si.mp4](autoware-si.mp4) | Autoware empty-scene planning fixture (no NPCs), ClearNoon / zero fog; lane offset ≈ −0.06 m while driving ~4 m/s; source cut after ~30 s recorded, SI detection→applied stop **6 ms**, no collision. The 11.2 s source-stop time reported in the gate log is the multi-node Autoware container's Docker shutdown time, not the SI watchdog (1.07 s) or the applied-stop gate. |

The VP_CONTROL HUD displays a *Right Lane Departure* warning in this run;
CARLA's map-relative offset was around +0.8 m while the car remained on-road.
That is VP's own lateral controller holding a curvature-proportional offset
(013): with the same VP perception, the SI's follower (vp-si) drives the curve
centred.
The clip is evidence of the commanded drive, source cut and SI stop, **not**
proof that the VP lane-departure classifier is correct or that this is a full
reproduction of the CES 2027 CARLA 0.10 / X5H setup. NPCs in the scene may
leave the camera's field of view during the low-speed drive.

The VP clips (re-recorded 2026-09-25 22:40 UTC) use rig image
`visionpilot:gpu-ros2`, built by `deploy/build.sh` from the pinned fork
commit `rig/vp-e2e-demo` `9cae16f9`: the VP→SI interface, the MJPEG HUD
viewer, the publish-boundary steering-sign fix, the speed-HUD exposure fix,
the tuned lateral filter (`fusion.lat.*` in the demo confs, 013), line-buffered
logs (012) and the AD-only-CIPO fix (011). They include the H/C pair fix
(`8077d53`), the host socket-buffer limits (012) and the actuator's
steering-curve compensation (013). The `vp-si` run used
`vision_pilot.demo-quiet.conf` (no per-frame fusion logs); driving code is
identical. `autoware-si.mp4` is unchanged (it does not run VP).

To reproduce a raw run on the GPU host, use
`deploy/tools/record-example.sh <vp-control|vp-si|autoware-si> [drive_seconds]`.
After reviewing its `gate.log`, `scenario.log`, `si.log` and raw videos, create
the small deliverable with `deploy/tools/package-example.sh <raw-run-dir>
docs/media/<mode>.mp4`. The raw recordings and verbose logs are **not**
committed. Autoware currently uses an empty-scene planning fixture, so its
clip cannot truthfully be labeled as 14-NPC traffic.
