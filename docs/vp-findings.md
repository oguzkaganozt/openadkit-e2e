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

## Template for new entries

## NNN — Title [single-run]

Reproduction: (rig file, image:tag, commits, command).
Evidence: (log paths + quoted lines, run date/host).
Status + follow-up: (confirm with 2nd run? fix in scope?)
