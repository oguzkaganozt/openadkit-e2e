#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PY="${CARLA_PY:-/tmp/carla-venv/bin/python}"
export SPAWN_INDEX="${SPAWN_INDEX:-184}"
exec "$PY" \
  "$ROOT/deploy/config_carla.py" \
  --host localhost --port 2000 \
  -f "$ROOT/deploy/carla916.json"
