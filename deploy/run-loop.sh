#!/usr/bin/env bash
# One clean SI drive: start the scenario, wait for a trajectory, then start SI.
# Compute mode: --cpu | --gpu, or COMPUTE=cpu|gpu (default: auto-detect).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT/deploy"
SI_BIN="$ROOT/upstream/autoware-safety-island/build/freertos-posix/actuation_freertos"
COMPOSE=(docker compose --env-file config.env --profile vp)

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
      echo "Usage: ./deploy/run-loop.sh [--cpu|--gpu]"
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
    if docker exec openadkit-e2e-carla-bridge python3 -c \
      "import carla; c = carla.Client('127.0.0.1', 2000); c.set_timeout(2); c.get_world()" \
      >/dev/null 2>&1; then
      echo "CARLA ready"
      return 0
    fi
    sleep 2
  done
  echo "CARLA did not become ready" >&2
  docker logs --tail 50 openadkit-e2e-carla >&2 || true
  return 1
}

if [[ ! -x "$SI_BIN" ]]; then
  echo "SI binary missing: $SI_BIN" >&2
  echo "Build first: $ROOT/deploy/build.sh --dds-interface <nic>" >&2
  exit 1
fi

pkill -f "$SI_BIN" 2>/dev/null || true
pkill -f "$ROOT/deploy/nodes/scenario.py" 2>/dev/null || true

"${COMPOSE[@]}" up -d carla
wait_for_carla
# Shared DDS/mode infrastructure (no world handles; start once).
"${COMPOSE[@]}" up -d domain-bridge operation-mode
# Fresh world FIRST: scenario load_world wipes every actor, so anything
# holding CARLA handles must (re)start after it. Always recreate (never
# restart): a world wipe must also reset VP latch/odom/fusion state and
# pick up rebuilt images, with clean per-run logs.
"${COMPOSE[@]}" up -d --force-recreate scenario
started_at="$(date --iso-8601=seconds)"
wait_for_log openadkit-e2e-scenario "ego up" "CARLA scenario"
if python3 -c "import json,sys; sys.exit(0 if json.load(open('$ROOT/deploy/config/carla-rig.json')).get('lead_vehicle', {}).get('enabled') else 1)"; then
  wait_for_log openadkit-e2e-scenario "lead on-lane" "lead vehicle"
fi
"${COMPOSE[@]}" up -d --force-recreate carla-bridge
started_at="$(date --iso-8601=seconds)"
wait_for_log openadkit-e2e-carla-bridge "camera frame #" "bridge camera"
"${COMPOSE[@]}" up -d --force-recreate adapter si visionpilot
started_at="$(date --iso-8601=seconds)"
wait_for_log openadkit-e2e-adapter "vehicle/lane_path + /localization/kinematic_state" "adapter subscribed"
wait_for_log openadkit-e2e-adapter "xfer #" "VP Path + adapter Trajectory"
wait_for_log openadkit-e2e-visionpilot "plan: tyre=" "VP planning"
wait_for_log openadkit-e2e-carla-bridge "applied control #" "SI control"
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
