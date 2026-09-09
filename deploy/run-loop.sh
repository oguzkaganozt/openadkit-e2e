#!/usr/bin/env bash
# One clean SI drive: spawn ego, wait for a trajectory, then start SI.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT/deploy"
SI_BIN="$ROOT/upstream/autoware-safety-island/build/freertos-posix/actuation_freertos"
COMPOSE=(docker compose --env-file config.env --profile vp)

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
  for _ in $(seq 1 90); do
    if [[ -x "$py" ]] && "$py" -c \
      "import carla; c = carla.Client('127.0.0.1', 2000); c.set_timeout(2); c.get_world()" \
      >/dev/null 2>&1; then
      echo "CARLA ready"
      return 0
    fi
    if docker exec openadkit-e2e-plant python3 -c \
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
pkill -f "$ROOT/deploy/config_carla.py" 2>/dev/null || true

"${COMPOSE[@]}" up -d carla
wait_for_carla
"${COMPOSE[@]}" up -d --force-recreate spawn
started_at="$(date --iso-8601=seconds)"
wait_for_log openadkit-e2e-spawn "ego up" "CARLA spawn"
"${COMPOSE[@]}" up -d
"${COMPOSE[@]}" up -d --force-recreate si
started_at="$(date --iso-8601=seconds)"
"${COMPOSE[@]}" restart visionpilot path-tx path-rx adapter plant
wait_for_log openadkit-e2e-path-tx "forwarded Path #" "VP Path"
wait_for_log openadkit-e2e-path-rx "published relayed Path #" "UDP relay"
wait_for_log openadkit-e2e-adapter "published Trajectory #" "adapter Trajectory"
wait_for_log openadkit-e2e-plant "applied control #" "SI control"
echo "SI started. Camera preview: http://127.0.0.1:8090/"
