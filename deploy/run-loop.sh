#!/usr/bin/env bash
# One clean SI drive: VP or Autoware planning -> SI -> CARLA.
# Compute mode: --cpu | --gpu, or COMPUTE=cpu|gpu (default: auto-detect).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT/deploy"
# RIG_MODE is the SI-selected source in SI_CONTROL. With BOTH_SOURCES=1 the
# unselected planner is also launched for an explicit ingress-isolation test.
RIG_MODE="${RIG_MODE:-vp}"
BOTH_SOURCES="${BOTH_SOURCES:-0}"
case "$RIG_MODE" in
  vp) RIG_JSON="${RIG_JSON:-carla-rig.json}" ;;
  autoware) RIG_JSON="${RIG_JSON:-carla-rig-empty.json}" ;;
  *) echo "Invalid RIG_MODE '$RIG_MODE' (expected vp or autoware)" >&2; exit 2 ;;
esac
export RIG_JSON
# One SI binary reads the mode and selected trajectory source once at startup.
# It is never changed in a running process, and no fallback is permitted.
SI_MODE="${SI_MODE:-si}"
SI_BIN="$ROOT/upstream/autoware-safety-island/build/freertos-posix/actuation_freertos"
case "$SI_MODE" in
  si|vp) ;;
  *) echo "Invalid SI_MODE '$SI_MODE' (expected si or vp)" >&2; exit 2 ;;
esac
if [[ "$RIG_MODE" == "autoware" && "$SI_MODE" != "si" ]]; then
  echo "Autoware trajectory requires SI_MODE=si (SI_CONTROL)" >&2
  exit 2
fi
if [[ "$BOTH_SOURCES" != 0 && "$BOTH_SOURCES" != 1 ]]; then
  echo "BOTH_SOURCES must be 0 or 1" >&2
  exit 2
fi
if [[ "$BOTH_SOURCES" == 1 && ( "$SI_MODE" != si || "$RIG_JSON" != carla-rig-empty.json ) ]]; then
  echo "Concurrent-source test requires SI_MODE=si and the empty CARLA rig" >&2
  exit 2
fi
export RIG_MODE SI_MODE
COMPOSE=(docker compose --env-file config.env --profile "$RIG_MODE")
if [[ "$BOTH_SOURCES" == 1 ]]; then
  COMPOSE+=(--profile vp --profile autoware)
fi

COMPUTE="${COMPUTE:-auto}"
while (($#)); do
  case "$1" in
    --cpu)
      COMPUTE="cpu"
      shift
      ;;
    --gpu)
      COMPUTE="gpu"
      shift
      ;;
    -h|--help)
      echo "Usage: RIG_MODE=vp|autoware SI_MODE=si|vp [BOTH_SOURCES=1] ./deploy/run-loop.sh [--cpu|--gpu]"
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

# shellcheck source=compute.sh
. "$ROOT/deploy/compute.sh"
resolve_compute
echo "Compute mode: $COMPUTE"
if [[ "$COMPUTE" == "cpu" ]]; then
  # Defaults for a CPU-only host; explicit exports still win.
  VISIONPILOT_IMAGE="${VISIONPILOT_IMAGE:-visionpilot:cpu-ros2}"
  VISIONPILOT_RUNTIME="${VISIONPILOT_RUNTIME:-runc}"
  VISIONPILOT_CONF="${VISIONPILOT_CONF:-vision_pilot.cpu.conf}"
  CARLA_RUNTIME="${CARLA_RUNTIME:-runc}"
  export VISIONPILOT_IMAGE VISIONPILOT_RUNTIME VISIONPILOT_CONF CARLA_RUNTIME
  if [[ "$CARLA_RUNTIME" == "runc" ]]; then
    echo "CARLA requires an NVIDIA GPU and cannot run with runtime 'runc'." >&2
    echo "Run on a GPU host, or point CARLA at one (CARLA_RUNTIME=nvidia)." >&2
    exit 1
  fi
  CARLA_WAIT_TRIES=240
else
  CARLA_WAIT_TRIES=90
fi

wait_for_log() {
  local container="$1"
  local pattern="$2"
  local description="$3"
  for _ in $(seq 1 60); do
    if docker logs --since "$started_at" "$container" 2>&1 | grep -q "$pattern"; then
      echo "$description ready"
      return 0
    fi
    sleep 1
  done
  echo "$description did not become ready" >&2
  docker logs --since "$started_at" "$container" >&2
  return 1
}

wait_for_carla() {
  local py="${CARLA_PY:-/tmp/carla-venv/bin/python}"
  echo "Waiting for CARLA RPC..."
  for _ in $(seq 1 "$CARLA_WAIT_TRIES"); do
    if [[ -x "$py" ]] && "$py" -c \
      "import carla; c = carla.Client('127.0.0.1', 2000); c.set_timeout(2); c.get_world()" \
      >/dev/null 2>&1; then
      echo "CARLA ready"
      return 0
    fi
    # Fallback probe through the bridge container only when it is actually
    # running: this function must not depend on a service that the loop has
    # not started yet (it may also have been stopped on purpose to keep the
    # world wipe client-free).
    if docker ps --format '{{.Names}}' 2>/dev/null | grep -q '^openadkit-e2e-carla-bridge$' && \
      docker exec openadkit-e2e-carla-bridge python3 -c \
      "import carla; c = carla.Client('127.0.0.1', 2000); c.set_timeout(2); c.get_world()" \
      >/dev/null 2>&1; then
      echo "CARLA ready"
      return 0
    fi
    sleep 2
  done
  echo "CARLA did not become ready" >&2
  echo "Hint: install the host CARLA wheel venv (deploy/build.sh) or set CARLA_PY to a python with the carla module." >&2
  docker logs --tail 50 openadkit-e2e-carla >&2 || true
  return 1
}

if [[ ! -x "$SI_BIN" ]]; then
  echo "SI binary missing: $SI_BIN" >&2
  echo "Build first: $ROOT/deploy/build.sh --dds-interface <nic>" >&2
  exit 1
fi
echo "SI supervision mode: $SI_MODE ($SI_BIN)"
echo "Rig candidate: $RIG_MODE ($RIG_JSON); both publishers: $BOTH_SOURCES"
if [[ "$RIG_MODE" == "autoware" || "$BOTH_SOURCES" == 1 ]]; then
  python3 - "$ROOT/deploy/config/$RIG_JSON" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as config_file:
    config = json.load(config_file)
if config.get("npc_vehicles") != [] or \
        config.get("lead_vehicle", {}).get("enabled") is not False:
    raise SystemExit("Autoware empty-scene profile requires no NPCs and no lead vehicle")
PY
  for map_file in lanelet2_map.osm pointcloud_map.pcd map_projector_info.yaml; do
    if [[ ! -s "$ROOT/deploy/maps/town04/$map_file" ]]; then
      echo "Missing Town04 $map_file; run $ROOT/deploy/tools/fetch-town04-map.sh first" >&2
      exit 1
    fi
  done
  printf '%s  %s\n' \
    '41e9149cfc74621d48a4af22b66fca90a12e51792779913106c9ac2ab73019ef' \
    "$ROOT/deploy/maps/town04/lanelet2_map.osm" | sha256sum -c --status -
  printf '%s  %s\n' \
    '9703b93441a5f5854b6e7cbb31a4d0d4043fe1080b57dfd5c784d5095f811593' \
    "$ROOT/deploy/maps/town04/pointcloud_map.pcd" | sha256sum -c --status -
fi

# Every attempt starts from zero: tear the whole project down (containers,
# not images) so no CARLA client, listener, or half-wiped world survives a
# previous run. A self-resurrected client across another client's world wipe
# leaves the 0.9.16 server pumping requests for dead streams ("Invalid
# session: no stream available with id N") until its RPC starves (observed
# 2026-09-25, twice); compose restart policies are disabled for the same
# reason. CARLA is then force-recreated below, so each attempt gets a fresh
# server rather than a wedged one.
"${COMPOSE[@]}" down --remove-orphans --timeout 10 2>/dev/null || true
# Belt and braces: remove any deterministic-name container Compose does not
# own (ad-hoc probe runs) that could still hold a CARLA client.
for name in openadkit-e2e-carla openadkit-e2e-scenario openadkit-e2e-carla-bridge \
             openadkit-e2e-adapter openadkit-e2e-si openadkit-e2e-visionpilot \
             openadkit-e2e-bridge openadkit-e2e-operation-mode \
             openadkit-e2e-autoware-planning openadkit-e2e-odom-to-tf \
             openadkit-e2e-empty-scene openadkit-e2e-carla-actuator; do
  docker rm -f "$name" >/dev/null 2>&1 || true
done
# Kill only host-launched instances of these two programs by their exact
# argv, never a broad pkill -f pattern that can match its own shell.
while read -r pid; do
  kill "$pid" 2>/dev/null || true
done < <(ps -eo pid=,args= | awk -v bin="$SI_BIN" -v scenario="$ROOT/deploy/nodes/scenario.py" '
  { pid=$1; $1=""; sub(/^ +/, ""); n=split($0, arg, " ");
    if (arg[1]==bin || (n>1 && arg[1] ~ /(^|\/)python[0-9.]*$/ && arg[2]==scenario)) print pid
  }')

"${COMPOSE[@]}" up -d --force-recreate carla
wait_for_carla
# Shared DDS/mode infrastructure (no world handles; start once).
"${COMPOSE[@]}" up -d domain-bridge operation-mode
# Fresh world FIRST: scenario load_world wipes every actor, so anything
# holding CARLA handles must (re)start after it. Always recreate (never
# restart): a world wipe must also reset VP latch/odom/fusion state and
# pick up rebuilt images, with clean per-run logs.
"${COMPOSE[@]}" up -d --no-deps --force-recreate scenario
started_at="$(date --iso-8601=seconds)"
wait_for_log openadkit-e2e-scenario "ego up" "CARLA scenario"
if python3 -c "import json,sys,os; sys.exit(0 if json.load(open('$ROOT/deploy/config/' + os.environ.get('RIG_JSON', 'carla-rig.json'))).get('lead_vehicle', {}).get('enabled') else 1)"; then
  wait_for_log openadkit-e2e-scenario "lead on-lane" "lead vehicle"
fi
"${COMPOSE[@]}" up -d --no-deps --force-recreate carla-bridge
started_at="$(date --iso-8601=seconds)"
wait_for_log openadkit-e2e-carla-bridge "camera frame #" "bridge camera"
"${COMPOSE[@]}" up -d --no-deps --force-recreate carla-actuator
started_at="$(date --iso-8601=seconds)"
wait_for_log openadkit-e2e-carla-actuator "sole CARLA control writer" "CARLA actuator"
if [[ "$RIG_MODE" == "autoware" || "$BOTH_SOURCES" == 1 ]]; then
  started_at="$(date --iso-8601=seconds)"
  "${COMPOSE[@]}" up -d --no-deps --force-recreate autoware-planning odom-to-tf empty-scene
  wait_for_log openadkit-e2e-empty-scene "empty-scene fixture active" "empty scene"
  echo "Waiting for Autoware route and nonempty trajectory (no SI until ready)..."
  if ! docker exec openadkit-e2e-autoware-planning bash -lc \
    'source /opt/ros/$ROS_DISTRO/setup.bash && source /opt/autoware/setup.bash && python3 /opt/rig-autoware/set_route.py'; then
    docker logs --tail 150 openadkit-e2e-autoware-planning >&2 || true
    exit 1
  fi
fi
if [[ "$RIG_MODE" == "vp" || "$BOTH_SOURCES" == 1 ]]; then
  "${COMPOSE[@]}" up -d --no-deps --force-recreate adapter visionpilot
  started_at="$(date --iso-8601=seconds)"
  wait_for_log openadkit-e2e-adapter "vehicle/driving_reference + /localization/kinematic_state" "adapter subscribed"
  wait_for_log openadkit-e2e-adapter "xfer #" "VP reference + adapter TrajectoryCandidate"
  wait_for_log openadkit-e2e-visionpilot "plan: tyre=" "VP planning"
fi
started_at="$(date --iso-8601=seconds)"
"${COMPOSE[@]}" up -d --no-deps --force-recreate si
wait_for_log openadkit-e2e-si "Supervision mode: $SI_MODE; trajectory source: $RIG_MODE" "SI selected ingress"
wait_for_log openadkit-e2e-carla-actuator "applied control #" "SI control"
# PREVIEW_HOST wins; otherwise auto-detect the public IP (link-local EC2-style
# metadata, then a public echo service), else fall back to local addresses.
# Set PREVIEW_AUTO=0 to skip auto-detection entirely.
preview_host="${PREVIEW_HOST:-}"
if [[ -z "$preview_host" && "${PREVIEW_AUTO:-1}" != "0" ]]; then
  preview_host="$(curl -s --max-time 2 http://169.254.169.254/latest/meta-data/public-ipv4 2>/dev/null | grep -Eo '[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+' | head -1 || true)"
fi
if [[ -z "$preview_host" && "${PREVIEW_AUTO:-1}" != "0" ]]; then
  preview_host="$(curl -s --max-time 3 https://api.ipify.org 2>/dev/null | grep -Eo '[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+' | head -1 || true)"
fi
if [[ -n "$preview_host" ]]; then
  echo "SI started. Camera preview: http://${preview_host}:8090/"
else
  echo "SI started. Camera preview:"
  hostname -I 2>/dev/null | tr ' ' '\n' | grep -v '^$' | while read -r ip; do
    echo "  http://${ip}:8090/"
  done
  echo "  http://127.0.0.1:8090/ (local)"
fi
