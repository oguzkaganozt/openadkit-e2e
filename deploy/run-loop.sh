#!/usr/bin/env bash
# One clean SI drive: spawn ego, wait for a trajectory, then start SI.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT/deploy"

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

docker compose --env-file config.env --profile vp up -d
for _ in $(seq 1 60); do
  if docker exec openadkit-e2e-plant python3 -c \
    "import carla; c = carla.Client('127.0.0.1', 2000); c.set_timeout(1); c.get_world()"; then
    break
  fi
  sleep 2
done
if ! docker exec openadkit-e2e-plant python3 -c \
  "import carla; c = carla.Client('127.0.0.1', 2000); c.set_timeout(1); c.get_world()"; then
  echo "CARLA did not become ready" >&2
  exit 1
fi

SI_BIN="$ROOT/upstream/autoware-safety-island/build/freertos-posix/actuation_freertos"
pkill -f "$SI_BIN" 2>/dev/null || true
pkill -f "$ROOT/deploy/config_carla.py" 2>/dev/null || true
for _ in $(seq 1 100); do
  pgrep -f "$ROOT/deploy/config_carla.py" >/dev/null || break
  sleep 0.1
done
if pgrep -f "$ROOT/deploy/config_carla.py" >/dev/null; then
  echo "Existing CARLA spawn process did not stop" >&2
  exit 1
fi

: >/tmp/e2e-spawn.log
nohup "$ROOT/deploy/run-spawn.sh" >/tmp/e2e-spawn.log 2>&1 </dev/null &
for _ in $(seq 1 40); do
  grep -q "Running" /tmp/e2e-spawn.log 2>/dev/null && break
  sleep 2
done
grep -q "Running" /tmp/e2e-spawn.log

started_at="$(date --iso-8601=seconds)"
docker compose --env-file config.env --profile vp restart visionpilot path-tx path-rx adapter plant
wait_for_log openadkit-e2e-path-tx "forwarded Path #" "VP Path"
wait_for_log openadkit-e2e-path-rx "published relayed Path #" "UDP relay"
wait_for_log openadkit-e2e-adapter "published Trajectory #" "adapter Trajectory"

nohup "$ROOT/deploy/run-si.sh" >/tmp/si.log 2>&1 </dev/null &
wait_for_log openadkit-e2e-plant "applied control #" "SI control"
echo "SI started. Camera preview: http://127.0.0.1:8090/"
echo "SI log: /tmp/si.log"
