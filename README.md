# openadkit-e2e

![openadkit-e2e closed loop](docs/closed-loop.svg)

A closed-loop Open AD Kit rig in CARLA 0.9.16. **VisionPilot or Autoware
plans, the Autoware Safety Island (SI) decides, and exactly one process
drives the car.** Everything runs with Docker Compose.

## How it works

- **Planners** (VisionPilot or Autoware) run on DDS domain 1. **The SI and the
  actuator** run on domain 2. A one-way bridge carries the inputs listed in
  [`bridge-config.yaml`](deploy/config/bridge-config.yaml); nothing goes back.
- **VisionPilot and the SI talk directly.** The SI reads VisionPilot's own
  `DrivingReference` (lane + speed plan) or `DrivingCommand` (steer, speed,
  acceleration). No adapter sits in between.
- **The SI picks its mode once at startup** and never switches or falls back.
  A stale or invalid input latches a stop. Only an explicit re-enable clears
  it.
- **`carla-actuator` is the only process that drives the car.** It sends the
  approved speed, acceleration and steering to CARLA's Ackermann controller;
  stops are a fixed brake.

| `RIG_MODE` | `SI_MODE` | Who plans | Who drives |
| --- | --- | --- | --- |
| `vp` (default) | `si` (default) | VisionPilot lane + speed plan | SI follower |
| `vp` | `vp` | VisionPilot command | VisionPilot, checked by the SI |
| `autoware` | `si` | Autoware trajectory (empty scene) | SI follower |

## Quick start

You need Ubuntu x86-64 with an NVIDIA GPU and driver, Git, curl and Python 3.10.

```bash
git clone https://github.com/oguzkaganozt/openadkit-e2e.git
cd openadkit-e2e
./deploy/setup.sh                        # Docker, NVIDIA runtime, socket buffers
./deploy/tools/fetch-town04-map.sh       # only for the Autoware profile
./deploy/build.sh --dds-interface ens3   # submodules, images, SI binary, CARLA client
./deploy/run-loop.sh --gpu               # one fresh-world drive
```

Replace `ens3` with your network interface. The camera preview is at
<http://127.0.0.1:8090/>. Other modes and settings are in the
[deployment guide](deploy/README.md).

Don't skip `setup.sh`. VisionPilot gets large camera frames over UDP and
starves with default socket buffers, so `run-loop.sh` refuses to start without
the bigger limits.

## Results

| What | Result | Details |
| --- | --- | --- |
| SI stop: fault detected → brake applied in CARLA | 4–14 ms in all three modes (limit 500 ms) | [stop gate](docs/e2e2-stop-gate.md) |
| Fault handling: replay, restart, re-enable | Pass | [ingress faults](docs/si-ingress-faults.md) |
| Lane keeping on open road | ~2.5 km at 9 m/s without contact | [findings](docs/vp-findings.md) |
| 4 m/s cruise | Steady (3.97–4.00 m/s) | [findings](docs/vp-findings.md) |

Example clips for each mode: [`docs/media/`](docs/media/README.md).

## Known limitations

- **VisionPilot hits a stopped car at ~9 m/s.** It has no reliable distance
  below ~8 m. Reported upstream:
  [vision_pilot#431](https://github.com/autowarefoundation/vision_pilot/issues/431).
- **VisionPilot fails at lane splits, ramps and the end of the highway**, and
  drives slightly off-centre in curves. Reported upstream:
  [vision_pilot#432](https://github.com/autowarefoundation/vision_pilot/issues/432).
- **The Autoware profile** uses an empty scene with no other cars.
- **If the SI stops publishing,** the actuator keeps the last command.
- **CARLA needs an NVIDIA GPU.** `--cpu` only moves VisionPilot inference to
  the CPU.

## Upstream work

| Repo | PR | What |
| --- | --- | --- |
| autoware-safety-island | [#66](https://github.com/autowarefoundation/autoware-safety-island/pull/66) | The SI supervisor used here |
| vision_pilot | [#423](https://github.com/autowarefoundation/vision_pilot/pull/423) | `DrivingCommand` / `DrivingReference` messages |
| vision_pilot | [#427](https://github.com/autowarefoundation/vision_pilot/pull/427) | Braking fixes |
| vision_pilot | [#428](https://github.com/autowarefoundation/vision_pilot/pull/428) | Configurable lateral filter |
| vision_pilot | [#429](https://github.com/autowarefoundation/vision_pilot/pull/429) | MJPEG viewer, HUD fix |
| vision_pilot | [#430](https://github.com/autowarefoundation/vision_pilot/pull/430) | Docker build and log fixes |

Until these merge, the submodules are pinned to the branches below. Open work
is in [`todo.md`](todo.md).

## Repository

| Path | Contents |
| --- | --- |
| [`deploy/`](deploy/README.md) | Setup, build, Compose services, configs, rig nodes and tools |
| [`docs/`](docs/) | Stop-gate and fault evidence, VisionPilot findings, clips |
| [`safety_island_msgs/`](safety_island_msgs/msg) | `ApprovedRequest`, the SI output the actuator reads |
| [`upstream/vision_pilot`](https://github.com/oguzkaganozt/vision_pilot/tree/rig/vp-e2e-demo) | VisionPilot, pinned on `rig/vp-e2e-demo` |
| [`upstream/autoware-safety-island`](https://github.com/autowarefoundation/autoware-safety-island/tree/feat/si-supervisor-v0-1) | Safety Island, pinned on `feat/si-supervisor-v0-1` |
