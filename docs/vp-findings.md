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

Status + follow-up: confirmed (≥3 stalls observed). This is the top rig
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
behavior the architecture requires in VP_CONTROL mode.

Evidence: `/tmp/opencode/vpcontrol-spawn100-{vp,scenario,si}.log`,
`/tmp/opencode/steer-trace-spawn100.csv` (local temporary evidence).

## Template for new entries

## NNN — Title [single-run]

Reproduction: (rig file, image:tag, commits, command).
Evidence: (log paths + quoted lines, run date/host).
Status + follow-up: (confirm with 2nd run? fix in scope?)
