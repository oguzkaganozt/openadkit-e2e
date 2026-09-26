# VP findings log (CARLA, reproducible)

Each entry: what was observed, exact reproduction, evidence, status.
Statuses: `confirmed` (seen ≥2 runs or multi-source) · `single-run` · `open`.

Scope: vanilla VisionPilot behavior (plus no-behavior-change shims).
`vanilla-vp-with-small-fixes@0166cd7e` counts as vanilla for lateral/
longitudinal findings: its 4 commits add topics + build/perf only
(lane_path, speed_horizon, ORT opt, mirror fix), no planner changes.

Common reproduction host (all entries below unless noted):
VPS `92.107.248.150`, RTX PRO 4000, `./deploy/run-loop.sh`,
Town04 spawn 184, e2e `main@25aa746`, SI `6fd68f7`.

## 001 — Lateral cold-start swerve at launch [confirmed]

At launch the hero spawns exactly on the lane center, yet VP's first
cross-track estimates read ~±1.0–1.7 m and converge within seconds.
SI tracks the path faithfully, so the car visibly throws itself sideways
on launch and can graze the guardrail while barely moving or stopped.

Reproduction:
`RIG_JSON=carla-rig-slow-lead.json ./deploy/run-loop.sh`
(VP image `visionpilot:gpu-ros2` built from
`vanilla-vp-with-small-fixes@0166cd7e`; e2e CARLA-calibrated `H.yaml`).

Evidence (run 2026-09-22, VPS `/root/run-slowlead3.log` + scenario log):
- VP: `raw_CTE=1.31m … Fused CTE=1.31m` while ground truth
  `lane_off=-0.04` (physically centered).
- Ground truth: `lane_off` −0.05 → +0.10 (t=13) → +0.32 (t=14),
  then stop (`v=0.01`, dist frozen at 42 m).
- `COLLISION t=33.9s with static.guardrail` while stopped/slow.

Why estimator noise, not calibration:
- Sign flips run to run (+1.3 m one run, −1.0 m another); a calibration
  error would pull the same direction every run.
- Converges within seconds; calibration error would not converge.
- Same `H.yaml` drove 600 m+ centered in A/B runs (see main README).

Earlier instances: fork latch-v7 runs grazed the right guardrail at
~0.65 m/s from the same spawn (main README, cold-start section).

Not reproduced 2026-09-26 on pin `9cae16f9`: in all 25 fresh-world launches
from spawn 184 (stopped, slow and stop-go leads; 015's runs) VP's parked CTE
stayed ≤ 0.20 m and the truth `lane_off` ≤ 0.05 m over the first 60 m, no
guardrail. Not bisected; the H/C pair fix (009) is the likely cause.

## 002 — Stopped-lead contact without CIPO latch [confirmed]

Without the latch hold model, the ego does not hold short: it creeps to
the stopped lead and touches it. Same rig/image as 001 (small-fixes,
no behavior-change commits), default lead rig (`carla-rig.json`:
lead cruises 5 m/s for 15 s, then brakes to a stop).

Reproduction: `RIG_JSON=carla-rig.json ./deploy/run-loop.sh`
(VPS `/root/run-smallfixes.log` + `/root/run-confirm002.log`,
2026-09-22).

Evidence, run 1:
- Ground truth: `gap center=4.7m hero_v=0.03 lane_off=+0.16` (on-lane,
  nearly stopped) at first contact.
- `COLLISION t=130.0–142.x with vehicle.tesla.model3` (lead is always
  a Tesla Model 3, hardcoded in `scenario.py`), repeated impulses.
- VP sequence: `cipo=true dist=8.2–8.6m` (flicker) →
  `cipo=false dist=150.0m` (close-range blindness, free-road) →
  creep → contact.
- Contrast: latch-v7 image on the same rig held `6.4 m` with zero
  contact (main README A/B section).

Evidence, run 2 (confirming, `/root/run-confirm002.log`):
- Identical signature: `gap center=4.7m hero_v=0.01 lane_off=+0.18`,
  same pose (`x=-495.6 y=326.3`), continuous grinding contact
  `t=150.0–168.x`. Deterministic across runs.

Revalidated 2026-09-26 on pin `9cae16f9`: still reproduces, at ~9 m/s with
the default conf; causes and A/B in 015.

## 003 — Slow-lead rear-end without CIPO latch [confirmed]

Without the latch, the ego does not slow for a slower rolling lead:
it follows centered, then rear-ends it at full speed.

Reproduction: `RIG_JSON=carla-rig-slow-lead.json ./deploy/run-loop.sh`
(lead rolls whole run, ~4.09 m/s).

Evidence, run 1 (`/root/run-slowlead2.log`, 2026-09-22):
- `lane_off ≤ 0.05` for 196 m, accelerating to ~14.7 m/s, gap
  65 m → 18 m with no slowdown.
- `COLLISION t=46.2s with vehicle.tesla.model3`
  `impulse=(11197.7,3595.2,928.0)`; hero ends off-road
  (`lane_off=+3.01`, `v=0.0`), lead drives on (gap 384 m).

Evidence, run 2 (`/root/run-confirm003.log`, 2026-09-22):
- Same signature: `gap 19.8 m → 9.4 m` at `hero_v=14.73, lead_v=4.09`,
  `lane_off=+0.06`; `COLLISION t=41.5s`
  `impulse=(9981.7,4331.3,834.1)`, then grinding. Deterministic.

## 004 — CARLA 0.9.16 native `--ros2` unreadable by modern ROS stacks [confirmed]

CARLA 0.9.16's built-in `--ros2` DDS endpoint cannot be read by standard
ROS 2 Humble/Jazzy subscribers on the same host. This is a CARLA-side
finding, not VP: VP drove fine once fed standard topics via a PythonAPI
shim (`/root/vanilla_bridge.py` pattern).

Version set: CARLA server `carlasim/carla:0.9.16` (UE 4.26, Foxy-era
bundled DDS) · wheel `0.9.16-cp310` · ROS 2 Humble (`fastrtps 2.6.12`,
`rmw 6.2.10`) · ROS 2 Jazzy (FastDDS 3.x, vanilla image) · Ubuntu 22.04.

Evidence (all 2026-09-22, VPS):
- Every native topic (`Image` 7 MB down to `Imu`, `Float64` speed)
  fails on Humble with FastCDR `sequence size exceeds remaining buffer`;
  Jazzy subscriber gets `FRAMES: None` (silent).
- 800x600 retry: same failure → not message size.
- Stale `/dev/shm/fastrtps_*` segments poisoned ALL local DDS traffic
  (even `ros2 doctor` and empty chatter loopback); cleanup fixed
  loopback but not CARLA topics.
- Built FastDDS 2.14.4 from source; swapping `libfastrtps.so.2.6`
  runtime → `rcl node's rmw handle is invalid` (rmw 6.2 cannot run on
  2.14 runtime). Proper fix = rebuild rmw against 2.14 (out of scope).
- Upstream context: 0.9.16 (Sep 2025) tested native ROS2 on Humble +
  Kilted with latest FastDDS 2.x (PR #9451, issue #9551), i.e. NOT the
  apt-pinned 2.6. Autoware Universe (`autoware_carla_interface`,
  0.9.15/Humble) and `carla_ros_bridge` both use PythonAPI shims and
  never enable native `--ros2` — consistent with this finding.

## 005 — VisionPilot planning stalls for seconds, dropping source freshness [confirmed]

VisionPilot's planning pipeline periodically stops processing for ~2–5 s
and then resumes with a backlog burst. The CARLA/ego inputs stay healthy
(bridge camera at ~10 Hz, odom at 20 Hz), so this is VP-internal, not a
transport gap in its inputs. On the SI rig the consequence is by design:
the selected source (DrivingReference) goes stale past the 1.0 s
supervisor timeout, SI latches SI_STOP and the vehicle brakes to a stop
until an explicit re-enable.

Reproduction:
`./deploy/run-loop.sh` stack (VPS), CARLA Town04 empty rig, VP image
`visionpilot:gpu-ros2` (`feat/vp-si-interface@3d4976e4`), SI supervisor
(vp_si_control_contract v0.1).

Evidence:
- 2026-09-24 ~22:05 UTC, idle hero: VP stopped planning for ~5 min while
  the process stayed up; only its ROS endpoints vanished. Coincided with
  a CycloneDDS probe container leaving the domain; container restart
  restored it. (`single-run` occurrence.)
- 2026-09-24 23:05:17.93–23:05:20.15 UTC, driving at ~19 m/s: `plan:`
  lines stop for ~2.2 s, then resume and burst (`23:05:22.954`, six
  lines in one ms). SI latched `SI_STOP: trajectory ... arrived 1.02 s
  ago` at `23:05:19.100`; the vehicle stopped from 19.1 m/s in ~3 s /
  ~35 m (dist 1738→1773) with the explicit stop control
  (`cmd_v=0.00 cmd_a=-1.50 brake=0.400`). Repeated stalls within the
  next minutes (another SI latch stopped the car again ~400 m later).
- The deliberate VP-container stop test (22:57) shows the same 1.0 s
  supervisor path: stop issued 22:57:24.657 → latch 22:57:25.725.
- 2026-09-25 09:44–09:46 UTC, **new rig host** (vast.ai RTX 4060 Ti):
  much worse cadence — 26 adapter arrival gaps > 0.5 s within 542 samples
  over ~90 s, largest **9.33 s**, several 1.2–3.4 s; the SI latched six
  times in ~3 minutes (`fault_id 1..6`, each `trajectory ... arrived
  ~1.0–1.1 s ago`). Inputs and resources were healthy in the same window:
  camera stream had **0** stall warnings, host load 4.35 on 23 vCPU, GPU
  38% / 4.6 of 16 GB, VP container ~110% CPU, no cgroup quota, and no VP
  error lines. The moving applied-stop gate test only succeeded because
  its preflight found one NORMAL window at 8 m/s.
- 2026-09-25 10:04–10:13 UTC, dedicated split measurement on the same
  host: an independent image-stream subscriber (domain 1, reliable QoS
  matching the bridge) received **1678 images in 180 s with a single
  0.50 s gap** — the camera/bridge/DDS feed is healthy. Over the same
  seven minutes the adapter logged 181 stalls > 0.5 s (87 of them ≥ 1.0 s,
  median 0.96 s, p90 3.80 s, max 16.86 s). During every stall window the
  GPU peaked at 52% (baseline peak 54%) — idle, not saturated — while the
  VP container kept burning 70–150% CPU and emitted its log lines in
  backlog bursts after each silence. Conclusion: the stall is **inside the
  VP process** (busy but not publishing), not camera, bridge, DDS
  transport, GPU saturation or host throttling; per-stage stopwatch
  instrumentation in the VP pipeline is the next step.
- 2026-09-25 17:15:57.508–17:15:58.850 UTC, a fresh **4 m/s** VP→SI demo
  world showed a 1.342 s gap in VP `plan:` logs and a 1.386 s gap in
  adapter `xfer` logs. SI latched `trajectory arrived 1.10 s ago` at
  17:15:58.626, before the planned deliberate source cut, and the car
  stopped in-lane (no collision). A prior fresh demo world latched the
  same way after a 1.15 s trajectory gap. These are discarded demo
  attempts, not source-cut gate measurements. The 4 m/s limit improves
  lateral behavior; it does **not** remove the source-stall fault.

**Resolved by 012:** the stalls were camera-frame starvation on the
CycloneDDS→FastDDS UDP path with 212 KB socket buffers, not VP-internal work.

Status + follow-up (historical): confirmed (≥3 stalls observed). This is the top rig
blocker: the vehicle cannot complete a long autonomous run while the
planner stalls every few minutes. Next: instrument VP's per-stage timing
(camera→planner→publish) with the existing stopwatch macros to find
whether the stall is in the image path, the planner, or the ROS publish
chain; a VP-side "no frame processed in N s" watchdog would also surface
it as a first-class fault instead of an input-staleness symptom.

## 006 — VP path estimate goes straight on a curve at speed, running the car off-lane [single-run]

On the SI rig (config 1, VP → SI_CONTROL) the car drove well for ~70 s /
1.1 km at 20–21 m/s, then left the lane on a curve and contacted the
barrier/off-road edge while still fast. VP never requested a slowdown;
this is a VP-side lane/path estimate failure, not an SI ingress fault.

Reproduction: `RIG_MODE=vp SI_MODE=si` on the fresh-world viewer run
(VPS `77.104.167.149`, 2026-09-25 12:54 UTC), VP image
`visionpilot:gpu-ros2-view` (`feat/vp-mjpeg-viewer@2ecc4146`, which only
adds the MJPEG endpoint; driving code identical to
`feat/vp-si-interface@3d4976e4`), Town04 empty rig.

Evidence:
- Ground truth (`scenario`): `lane_off` +0.25 → +0.90 → +2.15 → +2.65 in
  3 s while `v` 20.58 → 18.76 → 14.31 → 8.31 → 0.01 m/s; the car stops
  off-lane at x=383.0 y=96.7, `curve=-0.0085`.
- VP plan at that section: `kappa≈0.007`, `tyre=0.0000 rad` — i.e. VP
  believes the road is nearly straight; its speed horizon stays
  `h0≈20.5 hn≈21` up to the event (adapter xfer), so no braking intent.
- Applied control: `tire≈0.003–0.017 rad` (nearly straight) at 20+ m/s,
  i.e. the SI follower faithfully tracked VP's (straight) path.
- Coincident, but separate: a 1.1 s adapter publication gap
  (`src=94.87 → 95.97`, the finding-005 stall) made SI latch SI_STOP
  1.01 s after the last candidate; the actuator applied
  `decision=1 brake=0.400` while the car was already at 16.2 m/s and
  decelerating from the off-lane contact.

Caveat: VP's `plan:` lines carry no timestamps, so the kappa/tyre values
above are from the stopped car at the same location; the path shape
during the event is inferred from the near-zero applied steering and the
ground-truth lane loss.

Status + follow-up: `single-run`, VP-side (separate from the SI ingress
work). Next: timestamp VP's path output (`path_a/b/c`, `kappa`, `cte`)
and reproduce the same curve; a VP-side curvature/path sanity check (or
the unselected-Autoware cross-check discussed in the contract) would
catch a straight-on-curve plan before it reaches the follower.

## 007 — DrivingCommand steering sign mirrored vs the contract; VP_CONTROL steered against the error [confirmed, fixed]

In VP_CONTROL the SI passes VP's `DrivingCommand` through unchanged. Every
run on the SI rig left the lane at the first gentle curve and hit the
barrier at ~35 m (same pose each time: lane_off ≈ +2.5 m, yaw ≈ 116°,
dist 35-37 m), while SI_CONTROL drove the *same* VP perception for
kilometers. The lateral error grew in the same direction as VP's command:
the published steering had the opposite sign of the declared contract, so
the car steered against the correction.

Reproduction: `RIG_MODE=vp SI_MODE=vp` on the SI rig (Town04, empty rig),
VP image built from `feat/vp-si-interface` (default) and the experiment
image `visionpilot:gpu-ros2-neg` (same source with the publish sign
negated).

Evidence (2026-09-25, VPS `77.104.167.149`):
- Crash window, car drifting right: ground truth `lane_off` +0.07 → +0.52
  → +1.90 → +2.60 m, yaw 86.7 → 98.5 → 116.1°, speed collapse 7.4 → 0.03.
- Same window, VP command: `tire` +0.014 → -0.090 → -0.195 rad; the
  actuator (contract-correct: `carla = -tire / max_steer`, and `carla`
  positive turns the car right) applied +0.073 → +0.160 — i.e. further
  right while the car was already right of centre.
- `DrivingCommand.msg` declares `steering_tire_angle_rad (rad, positive
  left)`, matching the Autoware/SI actuator convention; the raw plan value
  published by `app/vision_pilot.cpp` was its mirror.
- Experiment image with `command.steering_tire_angle_rad =
  -applied_steering`: same rig drove **131 m centred** (lane_off +0.85 m,
  yaw 50.7°), no collision; the run ended only when the known VP planning
  stall latched the SI (`vp command arrived 1.12 s ago`).
- SI_CONTROL is unaffected: it consumes the path polynomial `a/b/c` with
  the SI's own follower, so no wall contact occurred in those runs.

Fix: fork branch `rig/vp-e2e-demo` (`491743a9`; contains `fix/vp-steering-sign`
`fb147561` plus `feat/vp-mjpeg-viewer` `2ecc4146` and the speed-HUD exposure
fix), negating only at the ROS publish boundary (`command.steering_tire_angle_rad =
-applied_steering`); the CAN `write()` path keeps its internal convention
and the message contract stays "positive left". Status: fix in place,
low-speed re-validation below (008) passed; the source-cut example is
[`docs/media/vp-control.mp4`](media/vp-control.mp4). High-speed lane keeping
remains open.

## 008 — VP_CONTROL high-speed oscillation is not an actuator steer reset

Reproduction (2026-09-25, VPS `77.104.167.149`): fresh `run-loop.sh` worlds,
`visionpilot:gpu-ros2-view` (includes 007's publish-sign fix), SI in
`VP_CONTROL`, `carla-rig-empty.json` (Town04, Lincoln mkz_2020, spawn 184).
Only VP's `speed_limit` was changed between the trials below (configs:
`deploy/config/vision_pilot.diag9.conf` and `...diag4.conf`, one-line
derivatives of the rig viewer config). The SI binary,
CARLA actuator, camera, and map stayed the same. The 4 m/s start script missed
a VP log readiness marker; SI was started separately in the still-idle fresh
world, **before** the car moved. A read-only 20 Hz CARLA probe recorded the
commanded control, actual left/right front-wheel angles, pose, and lane offset;
it never ticks the world or writes controls (`deploy/tools/passive_steering_probe.py`).

| VP speed limit | Ground-truth result (one run each, not reliability bounds) |
|---|---|
| 33.3 m/s (default) | 12.7 m/s peak; guardrail contact after ~158 m at sim t=39.7 s in the synchronized trace. Other fast runs reached ~126–178 m before failure. |
| 9.0 m/s | 9.17 m/s peak; guardrail contact at sim t=40.7 s, ~141 m from spawn. Merely reducing to 9 m/s does not fix it. |
| **4.0 m/s** | **105 s / 412 m** measured while moving, 2.72–4.40 m/s, no collision and no SI latch; CARLA lane offset −0.47 to +0.84 m. The test window ended while driving, not at a fault. |

A second fresh 4 m/s world used `carla-rig-traffic.json` (14 spawned NPCs,
ClearNoon, 0 fog): **55 s / 216 m**, 3.07–4.39 m/s, no collision or SI latch,
CARLA lane offset +0.68 to +0.83 m. The nearby-NPC count during the drive
was typically 2–3, not all 14 visible at once. The separate demo config
`deploy/config/vision_pilot.demo.conf` records this operating point; the
production/default `vision_pilot.conf` still has 33.3 m/s.

Evidence: read-only traces `/tmp/opencode/steer-trace-{1628,diag9,diag4}.csv`
and matching `vpcontrol-{1628,diag9,diag4}-{vp,scenario,si}.log`, plus
`steer-trace-traffic4.csv` and `vpcontrol-traffic4-{scenario,si}.log` (local
temporary evidence; do not commit raw recordings). The `CARLA
VehiclePhysicsControl.steering_curve` on this car is `(0 km/h, 1.0), (20,
0.9), (60, 0.8), (120, 0.7)`. During the fast trace the actual FL wheel was
0.858 × the command at >5 m/s and 0.837 × at >10 m/s, matching the interpolated
curve within ~0.001 rad; it did **not** return to zero between VP updates.
The actuator continuously applies the last SI-approved payload on each loop.

The VP plan does intermittently pass through near-zero and reverse sign. At
16:28:56 UTC the VP filtered CTE was about −1.1 m as the car crossed to the
left, and the applied tire angle was about −0.055 rad (right). At 16:28:57 the
CARLA map reported the car about +1.3 m **right** of lane center, while VP's
filtered CTE still said about −1.4 m and requested right correction. The raw
camera CTE had already reversed sign. This is evidence of material estimator
lag/mismatch during a rapid maneuver, **not** proof that CARLA ignores or
forgets a command. The physical curve attenuation contributes to understeer;
the trace does not isolate it as the sole cause of the later oscillation.

Upstream comparison: [openadkit #147](https://github.com/autowarefoundation/openadkit/issues/147)
and [vision_pilot #422](https://github.com/autowarefoundation/vision_pilot/pull/422)
run the CES prototype with `speed_limit = 4.0`, CARLA 0.10 `Town04_Opt`,
`vehicle.lincoln.mkz`, spawn 5, a 1280×720/FOV 60 camera at (1.25, 0, 1.58),
its matching `H_carla.yaml` and C matrix, and CARLA-native Ackermann messages.
Our 0.9.16 rig uses a different car, spawn, camera, homography, map variant,
and SI-approved `VehicleControl`. Their CARLA Ackermann controller eventually
normalizes steering to `VehicleControl.steer` with the maximum wheel angle; an
API swap alone would not remove the vehicle's speed-dependent steering curve.
In the lateral path, our publish-boundary negation (VP internal right-positive
to ROS left-positive) and actuator negation (ROS left-positive to CARLA
right-positive) cancel: both rigs command CARLA in VP's original sign.
Our 4 m/s result reproduces a **similar stable operating point**, not their
full CARLA 0.10/X5H/CR52 stack or one-lap gate. Do not treat the 4 m/s limit
as a high-speed VP lateral-controller fix.

## 009 — Rig H/C homography pair mismatch: fixed, and ruled out as the high-speed departure cause

VisionPilot loads `H.yaml` at startup and `homography_C_matrix.yaml` in the
preprocessing module, both from the image config directory. The image
generates C at build time from the in-tree `VisionPilot/config/H.yaml` — the
OpenLane 1920x1080 example whose own header says "MODIFY THIS MATRIX FOR
DIFFERENT INPUT CAMERA" — while `deploy/docker-compose.yaml` bind-mounts the
rig-calibrated `deploy/config/H.yaml` at run time. So AutoDrive's BEV input
(`warped` in `inference.cpp`) was produced with a calibration that did not
match the H used for `H_resized` and the lane path; AutoDrive curvature feeds
the lateral fusion (`ad_curv = drive.curvature_raw * ad_curvature_scale` in
`lateral_fusion.cpp`). Upstream PR #422 ships a C that matches its
`H_carla.yaml` (recomputed C differs by < 0.007), i.e. a matched pair is the
intended deployment shape.

Fix (commit `8077d53`): `deploy/tools/gen-homography-c.sh` regenerates C from
the rig H with the upstream script; `deploy/config/homography_C_matrix.yaml`
is that generated pair; compose mounts it next to H.

A/B (both fresh empty-rig worlds, spawn 184, VP_CONTROL, `vision_pilot.diag9.conf`,
9 m/s limit, read-only 20 Hz trace):
- old C: guardrail contact at sim t=40.7 s, ~141 m from spawn.
- new C: lane offset +0.46 → +0.76 → +1.41 → +2.47 m over sim t=34–37 s;
  guardrail contact at sim t=37.4 s, ~122 m from spawn.

Pre-crash AutoDrive raw curvature was small in both (10 s-bucket means +0.013
old / +0.020 new); the large post-crash differences (old +0.2..+0.34, new
−0.1) are the model reacting to the off-lane scene. The mismatch was real and
is fixed for correctness, but it is **not** the dominant high-speed
lane-keeping cause.

What the new trace shows instead: the raw camera CTE tracks the growing offset
(+0.48 → +1.55 m, the same sign as CARLA's +1.4 → +2.5 m), then flips to
−1.54 m within 0.23 s at sim t≈36.6; the particle-filtered CTE follows slowly
(−0.3..−0.6 m/s) and the commanded tire angle stays below 0.05 rad. The
departure begins ~2.5 s earlier (sim t≈35) with the road curve already
established (camera κ ≈ 0.005): the car's yaw rate collapses from ≈0.05 rad/s
(following the curve) to ≈0.004 rad/s (essentially straight) while the road
still requires ≈0.050 rad/s (v·κ at 9 m/s), and the controller does not
command the sustained curve angle (the steady-state bicycle angle would be
L·κ ≈ 0.016 rad). The leading candidate remains VP's lane
measurement/fusion and the lateral command under load (estimate lag and a
near-lane-edge measurement flip), not the AutoDrive warp, the actuator or the
SI.

Evidence: `/tmp/opencode/vpcontrol-cfix9-{vp,scenario,si}.log`,
`/tmp/opencode/steer-trace-cfix9.csv` (local temporary evidence).

## 010 — High-speed survival is route-dependent; the SI rejects an out-of-envelope VP accel

Same rig and same 9 m/s VP_CONTROL limit, only the spawn section differs
(fresh empty-rig worlds, `vision_pilot.diag9.conf`, read-only 20 Hz trace):
- **spawn 184** (our default): guardrail contact at sim t=37–41 s, ~120–140 m
  (009).
- **spawn 100** (the PR #422 `carla916.json` spawn): no collision at all; the
  car covered **614 m**. First |lane offset| > 1.5 m at ~478 m; max |offset|
  2.26 m; the run ended when the SI latched `vp accel out of range`: VP
  commanded **−7.513 m/s²** (cycle 547) against the SI actuation limit
  **±6.0 m/s²**; the SI refused it and applied its own stop payload
  (brake 0.400), stopping the car on-road at lane −1.85 m.

So the high-speed lane-keeping failure is strongly route-dependent, and
upstream's spawn choice is part of why their demo can hold a long run. Lane
wandering to ±2 m at 9 m/s is still not acceptable lane keeping, and the
spawn-184 curve remains a deterministic counterexample at this speed.
Separately, the −7.5 m/s² command is a VP longitudinal-control excursion; the
SI's independent envelope check turned it into a safe stop, exactly the
behavior the architecture requires in VP_CONTROL mode. Its source is a phantom
AutoDrive-only CIPO on the guardrail (011, fixed).

Evidence: `/tmp/opencode/vpcontrol-spawn100-{vp,scenario,si}.log`,
`/tmp/opencode/steer-trace-spawn100.csv` (local temporary evidence).

## 011 — Phantom CIPO braking: AutoDrive-only detections of the guardrail on curves [confirmed, fixed]

The −7.513 m/s² command of 010 was not a lead-vehicle decision. In the
camera-only fusion path (`longitudinal_fusion.cpp`, radar off) a **single**
AutoDrive frame with `flag_prob ≥ 0.40` and no AutoSpeed vehicle box confirmed
a CIPO, and the IDM term in `compute_acceleration` has no lower bound, so a
7 m "lead" at 9 m/s produced −7.5 m/s² (SI refused it, ±6.0 envelope).

Evidence:
- 010's spawn-100 run: `[Fusion] src=cam | AD=7.2 m (p=42%) | AS+H=(none) …
  Fused=7.0 m` → `plan: … accel=-7.513 … cipo=true dist=7.0 m` → next frame
  `No CIPO confirmed (AD=23%) — reset to 150 m`. Empty world (no NPCs).
- 19 pre-collision CIPO episodes across five 2026-09-25 empty-world runs
  (spawn100/diag9/diag4/1628/1613 logs in `/tmp/opencode/`): **all** AD-only
  (no AS box), 1–8 frames, p up to 79 %, 5–70 m; the −4.09 / −3.92 m/s² of 010
  were the same kind. A pure N-frame persistence filter would not remove them.
- 1 Hz frames (`frame_sampler.py`) at the phantom moments of
  `20260925T200938Z-baseline-s184-9ms` show an empty road curving away with the
  guardrail crossing the car's heading ~15–20 m ahead — where AD reported the
  CIPO (16 m). The code's own comment warns about a "static rail".

Fix (fork `fix/vp-ad-only-cipo`, `c4e17bf0`, in pin `652b72e1`): an AD-only
frame may continue a track that an in-path AutoSpeed vehicle started (close
range, where the box is lost — finding 002), but cannot start one;
`fusion.ad_only_needs_as_track=false` restores the old rule. No IDM clamp: the
SI owns the envelope.

A/B (VPS, fresh worlds, 64 MiB socket buffers, `diag9-latfast` conf, actuator
steering-curve compensation on; `visionpilot:gpu-ros2-p3` = diag + fix vs
`visionpilot:gpu-ros2-diag`):

| Run | Image | Pre-collision CIPO episodes | Lead following |
|---|---|---|---|
| spawn 100 (`…214345Z-p1comp1-latfast-s100`) | diag | 7 (all AD-only) | — |
| spawn 100 (`…215811Z-p3f-s100-p3`) | fix | **0** (19 AD-only frames ignored) | — |
| slow lead 4.09 m/s (`…215117Z-p3f-slowlead-diag`) | diag | — | closes 45→11 m, holds 11.0–12.1 m at 4.0–4.3 m/s |
| slow lead (`…214750Z-p3f-slowlead-p3`, `…215443Z-p3f-slowlead-p3`) | fix | — | same: holds 11.0–12.4 m, no contact |

Both spawn-100 runs end at the same junction wall (013); the −4.7/−5.1 m/s²
there are AS-boxed wall detections at 8.4 m, i.e. a real obstacle. An earlier
slow-lead A/B on the old lateral filter was confounded by weaving (the lead left
the in-path region; the fix changed no fusion decision in that run) and is
superseded by the table above. Separately, 003's weak braking for a slower
lead is not reproduced here: with the current pin the car matches the lead's
speed at ~11.5 m.

## 012 — VP "planning stalls" were camera-frame starvation from 212 KB UDP socket buffers [confirmed, fixed]

The source stalls of 005 are not inside VisionPilot's pipeline. VP waits for a
camera frame that has not arrived: the bridge publishes 1920×1280 BGR8 frames
(7.37 MB) at 10 Hz on CycloneDDS, VP (Jazzy, FastDDS) receives them over UDP in
reliable mode, and the rig host had the stock `net.core.rmem_max` /
`rmem_default` of 212 992 B. Fragments overflow the socket, reliable repair
takes hundreds of ms to seconds, and the SI latches on the 1.0 s freshness
watchdog.

Reproduction (VPS `77.104.167.149`, RTX 4060 Ti, 2026-09-25, fresh worlds,
`RIG_MODE=vp SI_MODE=vp`, `carla-rig-empty.json`, spawn 184,
`vision_pilot.demo.conf` 4 m/s, 600 s drive). Diagnostic image
`visionpilot:gpu-ros2-diag` = pinned `491743a9` + fork branch
`diag/vp-stage-timing`: a per-cycle `[Timing]` line (wait / preprocess / infer
per model / plan / publish / viz / render) and a watchdog that reports a loop
that has not changed stage for 500 ms, while it happens.

Evidence (`/tmp/opencode/runs/<run>/`, analysed with the adapter `xfer` arrival
times):

| Run | Socket buffers | VP cycles | adapter gaps > 0.5 s | max gap | longest `wait` | max busy | SI latch |
|---|---|---|---|---|---|---|---|
| `20260925T204517Z-p2-base-demo4-600s` (pinned image) | 212 KB | — | 42 | 2.09 s | (no timing) | (no timing) | 1, at ~1 min (`vp command arrived 1.04 s ago`) |
| `20260925T201846Z-p2-diag-demo4-600s` | 212 KB | 4522 (~7.5 Hz) | 19 | 1.01 s | 871 ms | 200 ms | 0 (one 1.01 s gap) |
| `20260925T205648Z-p2-bufs-diag-demo4-600s` | rmem 64 MiB max / 16 MiB default | **6059 (~10 Hz)** | **0** | ≤ 0.21 s | **103 ms** | 158 ms | **0**, 2402 m |

- Every adapter gap > 0.5 s in the 212 KB runs coincides with a VP `wait`
  (no new frame) interval; VP's own work never exceeded 200 ms per cycle.
- The same starvation latched both 9 m/s `diag9-latfast` runs of 013
  (`wait` 1107 ms → `vp command arrived 1.03 s ago`; 1474 ms → 1.10 s).
- With large buffers VP processed every camera frame (10 Hz instead of ~7.5 Hz).
- 005's independent CycloneDDS subscriber saw a healthy stream: a
  CycloneDDS→CycloneDDS reader is not the path VP uses; it did not rule out the
  cross-vendor UDP receive.
- Side finding: `VP_INFO` is `printf` to stdout, fully buffered under the
  container log pipe, so VP log timestamps are buffer-flush times. 005's "`plan:`
  gap" numbers are therefore unreliable; the diag image line-buffers stdout.

Fix: `deploy/setup.sh` persists `net.core.{r,w}mem_max = 64 MiB` and
`{r,w}mem_default = 16 MiB` (`/etc/sysctl.d/60-openadkit-e2e-dds.conf`);
`deploy/run-loop.sh` refuses to start when `rmem_default` < 16 MiB
(`DDS_BUFFER_CHECK=0` skips). Follow-up (not done): the camera frame is larger
than VP needs (it resizes to the network input); a smaller bridge image or a
same-vendor/shared-memory transport would cut the 74 MB/s DDS load.

## 013 — 9 m/s lane loss was lateral-filter lag; a curvature-proportional offset remains [confirmed, partly fixed]

The spawn-184 failure of 006/008–010 is a growing **weave**, not a straight
run-off. 1 Hz frames of `20260925T200938Z-baseline-s184-9ms` show the car
yawed toward the right guardrail with VP's path curving hard left ("Left Lane
Departure"), one second after the opposite. The particle-filtered CTE lagged the
raw camera CTE by ~7 cycles (~0.7 s, cross-correlation r=0.70 vs 0.42 at lag 0);
the MPC acted on stale lateral error, the oscillation grew (CTE ±1.3 m, tire
±0.15 rad, ~5 s period) until contact. The earlier "deterministic 120–140 m"
is run-to-run variable: default-filter runs today hit the guardrail at 212,
225, 274 and 284 m.

Fix (fork `fix/vp-lat-fusion-config`, `3391b28e`, in pin `652b72e1`): the
lateral noise terms are `fusion.lat.*` keys; the rig confs set
`proc_noise_cte_rate_mps = 0.60`, `proc_noise_yaw_rate_rps = 0.20`,
`meas_noise_cte_m = 0.08` (defaults 0.15 / 0.05 / 0.15). `diag4`/`diag9` keep
the old filter as the historical A/B baseline; `diag9-latfast` is diag9 plus
these keys.

A/B (VPS, fresh worlds, spawn 184, VP_CONTROL, 9 m/s, 64 MiB socket buffers,
`visionpilot:gpu-ros2-diag`; frames checked at 25/55/85/115 s — on the road,
no lane change, no barrier):

| Filter | Runs | Result |
|---|---|---|
| default (`diag9`) | `…203026Z`, `…204051Z` (212 KB buffers), `…211137Z` | guardrail at 212 / 284 / 225 m, no SI latch |
| latfast | `…210811Z`, `…211506Z`, `…213329Z`, `…213657Z`, `…214017Z` | **1076–1088 m, no contact, no SI latch**; max \|lane_off\| in the first 300 m 0.79–0.82 m; filter lag 0 cycles |

With 212 KB buffers two latfast runs were cut early by 012's starvation latch;
both fixes are needed for a long 9 m/s run.

What remains (measured on the latfast runs): the car is centred on straights
(+0.03…+0.08 m) but holds a **curvature-proportional offset** in curves
(+0.6 m at κ = −0.0056, −0.65 m at κ = +0.0030; ≈ 110 m × κ), and VP's own CTE
reports it (+0.8 m) — not a camera bias, so `fusion.cte_bias_m` would be wrong.
Two candidate causes were tested:
- Actuator under-delivery: CARLA scales the wheel angle by the car's
  speed-dependent `steering_curve` (0.869 at 9 m/s, measured 0.87). The
  actuator now divides by it (`ACTUATOR_STEER_CURVE_COMP`, default on) so the
  approved tire angle is realised. A/B on/off/on (`…213329Z`, `…213657Z`,
  `…214017Z`): identical offsets and distances — the closed loop already
  absorbed the gain. Kept as a contract correctness fix; not the offset cause.
- CTE/yaw read at x_min: VP evaluates the fitted path at the nearest waypoint
  (x_min = 8.3 m), where the lane heading differs from the vehicle's by
  κ·x_min ≈ 0.044 rad; the logged yaw sits at 0.05–0.08 rad in curves. Fork
  `fix/vp-lat-eval-at-vehicle` (`680edfd9`) adds `fusion.lat.cte_eval_x_m`
  (default −1 = unchanged; 0 = at the vehicle). A/B on one build
  (`652b72e1`): x_min `…220646Z` 1078 m, curve \|lane_off\| 0.57 m; x = 0
  `…220319Z`, `…221013Z` left the lane at 7 m and hit the guardrail at 16–21 m
  (yaw 141–160°). Extrapolating the fit 8 m back to where there is no data
  destabilises the start. **Negative; not in the pin** (branch kept as the
  record, one-off conf not committed).

So the curve offset stays open as a known limitation: it is steady (≈ 110 m × κ,
reproduced on every latfast run), bounded (≤ 0.8 m on this route at 9 m/s) and
not caused by the actuator or by a camera bias. It is not speed-driven either:
at 4 m/s the offset per curvature is larger (≈ 150 m × κ, curve |lane_off|
0.75 m) with the old and the new filter alike (`…205648Z` 0.76 m, pin
`…222103Z` 0.75 m) — so the tuned filter does not regress the 4 m/s demo, and
the offset is a static balance, not understeer. The next place to look is the
MPC cost balance in `lateral_planning.cpp` (`delta_weight` 45000 on
`delta − delta_ff` vs `cte_weight` 10·(1+100κ)), not the measurement.

MPC A/B (fork `exp/vp-mpc-cte-weight-40`, `a46fec07`, the commented-out
earlier value; image `visionpilot:gpu-ros2-w40` = pin + this change):

| cte_weight | Runs | curve \|lane_off\| | Result |
|---|---|---|---|
| 10 (pin) | 6 at 9 m/s | 0.57–0.60 m | 6/6 no contact |
| 40 | `…224617Z` 9 m/s | **0.25 m** | 1079 m, no contact |
| 40 | `…224946Z` 9 m/s | **0.18 m** | **spun out at 629–637 m** (yaw −49° → −163° in 2 s) |
| 40 | `…225317Z` 4 m/s, 180 s | **0.39 m** (pin 0.75) | 740 m, no contact |

The spin happens where the right edge line splits at a merge (90 km/h sign,
frames at 94–96 s) — the same spot where every pin run first exceeds 1 m
(~630 m) and recovers. A 4× CTE weight halves the curve offset but turns that
measurement jump into a violent correction. Not adopted; the trade-off needs a
jump-robust CTE (gating at lane splits) before a stiffer weight.

The offset belongs to VP's own lateral controller: in the re-recorded example
worlds (same curve, same VP perception, 4 m/s) VP_CONTROL held +0.74…+0.81 m
while VP→SI — the SI's follower tracking VP's path polynomial — held
−0.03…−0.04 m (`docs/media/README.md`).

Pin validation (`rig/vp-e2e-demo` `9cae16f9`, rebuilt `visionpilot:gpu-ros2`,
fresh worlds, 64 MiB buffers, compensation on): spawn 184 at 9 m/s
`…223342Z-final-s184-9ms` 1083 m, no contact or latch, curve |lane_off| 0.57 m;
slow lead `…final-slowlead-9ms` follows at 11.1–11.8 m, no contact; the same
code at `652b72e1` with default confs: 4 m/s for 600 s `…222103Z` 2404 m, no
contact, no latch, 0 adapter gaps > 0.5 s.

Known limit (not the weave; measured in 016): on spawn 100 the latfast car drove 1264–1270 m and
then hit the wall of a junction/ramp (κ = 0.0135, 30 km/h sign) after drifting
across the dashed lanes of a multi-lane curve (`…214345Z`, `…215811Z` frames).

## 014 — CARLA server stall latched the SI on odometry [single-run]

`20260925T214750Z-p3f-slowlead-p3` (VPS, fresh world, 64 MiB buffers) ended
with `SI fault: odometry arrived 0.43 s ago; latching SI_STOP`. Everything
stopped together at 21:50:33 UTC: the scenario's 1 Hz ground-truth pose log
ends at sim t=103 s, VP waited 1996 ms for a frame (`[Watchdog] … stage=wait`),
the adapter saw a 2.09 s `xfer` gap with odometry age 2178 ms. The simulator
stopped producing, and the SI stopped the car on stale odometry as designed.
One occurrence in ~25 runs; odometry age is otherwise unchanged by 012
(median ~200 ms, p99 ~300 ms with either buffer size). Not a VP or DDS fault;
collect the CARLA container log if it recurs.

## 015 — Stopped lead hit at ~9 m/s: IDM closing-speed sign, fusion velocity lag, near-field range [confirmed, open]

002 reproduces on the current pin, harder: with the default conf
(`speed_limit = 33.3`, curvature-limited) the ego reaches ~10 m/s behind the
default lead (`carla-rig.json`: 5 m/s for 15 s, then brakes to a stop) and hits
it at ~9 m/s (`COLLISION t=25.5–25.6 s … impulse=(5542,7818,617)`), then
pushes against it at throttle 0.47 with `cipo=false`.

Reproduction: `RIG_MODE=vp SI_MODE=si RIG_JSON=carla-rig.json
./deploy/tools/run-evidence.sh <label> 90`, pin `9cae16f9`, RTX 5080 VPS,
2026-09-26 (`20260926T132532Z-smoke-vp-si`, `…133825Z-base-stoplead-2`, 2/2).
Evidence for every run below: owner's archive
`~/openadkit-e2e-evidence-20260926/vps-records/`.

Four causes, each measured on these runs:

1. **Planner code bug — IDM closing speed has the wrong sign** (upstream
   `6305ea90`, 2026-08-10, changed `delta_v = -cipo_v` to `delta_v = cipo_v`).
   Both planner headers and `test_planning.cpp` define `cipo_v` as the lead's
   speed, the app passes the fused velocity *relative* to ego, and the IDM
   treats it as closing speed. At 9.4 m/s and 20 m: stopped lead **+0.52**
   m/s², lead pulling away at 14 m/s **−6.84** m/s² (fixed: −3.62 / +1.36;
   host check against `longitudinal_planning.cpp`). Steady following
   (Δv ≈ 0) is unaffected, which is why 003's slow lead is followed.
2. **Fusion — a low-flag AutoDrive distance outweighs AS+H at range.** With an
   AutoSpeed box present, AD's distance joins the particle filter at any flag.
   At p = 10–30 % it read 26–36 m while AS+H tracked the lead from 71 to 44 m
   (matching truth); AS+H noise grows with d² (16.8 m at 71 m vs 7.3 m for AD),
   so the track sat at 34–40 m for 3.5 s with ~0 closing speed.
3. **Fusion — the track velocity is unobservable.** `process_noise_dist_m =
   2.0` m per step (0.05 before upstream `09feb20a`; the comment above it still
   says keep it small) lets the distance follow the measurements without any
   velocity, and every single-frame loss of the far box resets the track to
   v = 0 (6 resets in one approach). Replaying the run's measurements through a
   port of the filter: velocity 0.0 m/s at 62 m (truth −7.1); with 0.3 m
   noise, a 6 m/s initial spread and a 0.5 s coast, −6.3 m/s.
4. **Perception — the near field has no reliable range** (not fixed here). The
   AutoSpeed box of the lead reaches the bottom of the 1024×512 frame at a
   homography range of 8.3 m (`bbox=(…,512)` from ~4 m camera-to-rear), stays
   there, and is lost at ~3 m. AS+H also reads **+2.2–2.9 m long** against
   CARLA truth at 5–30 m (steady slow-lead following, so not a timing error);
   AutoDrive at p ≥ 0.8 reads 0.5–1.7 m short and is right at ~4 m even at
   p = 0.36. The pin follows a slow lead only because the low-flag AD pulls the
   near range in (cause 2 works in its favour there).

A/B (fresh worlds, default conf + the keys named; min centre gap 4.7 m =
contact):

| Variant | Stopped lead | Slow lead (4.09 m/s) |
|---|---|---|
| pin `9cae16f9` | 2/2 hit, t = 25.5 s | no contact, min centre 10.0 m |
| 1 (`fix/vp-idm-closing-speed`) | hit, t = 30.5 s | — |
| 1+2 (`fix/vp-ad-dist-needs-flag`) | 2/2 hit, t = 28.8 / 30.5 s | — |
| 1+2+3 (`fix/vp-long-fusion-config`, keys above) | 2/2 hit, t = 29.4 / 29.5 s: brakes from ~46 m, 3.5 m/s at 17 m, then creeps in once the box clips | touch at t = 48.5 s (following at the 8.3 m floor) |

1–3 move the start of braking from ~10 m to ~46 m (centre gap); what remains is 4. Near-field
heuristics on top of that (`exp/vp-near-field-hold`: clipped box as an upper
bound, hold the lost lead, an AS+H range offset) were tried in 3 iterations
and dropped: each traded one failure for another (hold at the wrong range →
creep into contact; stationary-lead hold → speed sawtooth and a low-speed
swerve into the guardrail; hold never released when a lead was lost at 9.5 m).
They would mask a perception limit rather than fix it.

Status: **open, VP-side.** Pin unchanged. 1 is a plain code bug (draft-PR
candidate); 2 and 3 are fusion-design issues to raise upstream with this data;
4 is a model/geometry limit to report upstream (issue). Rig tooling added for
this: `carla-rig-stop-go.json` (`lead_vehicle.resume_s`: the lead drives off
again 20 s after stopping).

## 016 — Lane splits, a ramp and the end-of-highway junction break lane following [confirmed, open]

With the current pin (`9cae16f9`, SI `65b6875`, RTX 5080 VPS, 2026-09-26,
fresh worlds, `run-evidence.sh`), open-road lane keeping is sound: at 9 m/s
(`diag9-latfast`, VP_CONTROL) the car drove the whole spawn-184 highway loop
twice, ~2.5 km each, and spawn 100 for 1780 m, with no SI latch. Every contact
seen on empty roads happens at one of three places, all outside plain lane
keeping:

| Place | Runs | Result |
|---|---|---|
| spawn 184, ~615 m: the road forks, the right edge line diverges | `…150300Z-final2-demo-4ms` (4 m/s, `demo.conf`, SI_CONTROL + VP) | VP flags "Right Lane Departure" while centred; raw CTE jumps −0.9…+1.0 m, the fused CTE runs to −3 then +5.7 m, the car swerves left across the lanes and into the guardrail at 643 m (SI latches afterwards: no path fit once off the road). Passed by both 9 m/s runs today and by 013's 600 s 4 m/s run; 013's `cte_weight` 40 spin-out was here too — intermittent. |
| spawn 184, ~2520 m: the highway ends at a signalized junction with a κ ≈ 0.1 turn | `…145115Z-final2-s184-9ms-1`, `…145708Z-final2-s184-9ms-2` | 2/2: enters at 8.3–8.5 m/s, guardrail / wall at t ≈ 304–306 s. |
| spawn 100, ~1270 m: ramp, κ = 0.0135, 30 km/h sign | default conf (speed limit 33.3): `…140709Z`, `…142819Z`, `…144801Z` | 3/3 wall at ~1279 m, entering at ~14 m/s. At 9 m/s: 013 2/2 wall, today `…151353Z-final2-s100-9ms` 1/1 through — intermittent. |

Two VP-side causes are visible in the logs and frames:
- No lane-split handling: at the fork the lane model follows the diverging
  edge line (the HUD shows the path bending away while the car is centred),
  and the fused CTE follows it.
- The curve speed limit sees only the current curvature: `sqrt(mu·g/|κ|)`
  from the κ at the car. Upstream `6305ea90` removed the curvature preview
  (`t_preview`, filtered dκ) that used to slow the car before a bend, so it
  enters the ramp and the junction turn at full speed (κ = 0.1 → 4.4 m/s
  limit, reached only inside the turn).

Junctions with traffic lights are outside VP's lane-following scope. Status:
open, VP-side; report upstream (issue) with these runs. Evidence: owner's
archive `~/openadkit-e2e-evidence-20260926/vps-records/`.

## Template for new entries

## NNN — Title [single-run]

Reproduction: (rig file, image:tag, commits, command).
Evidence: (log paths + quoted lines, run date/host).
Status + follow-up: (confirm with 2nd run? fix in scope?)
