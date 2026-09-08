#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PY="${CARLA_PY:-/tmp/carla-venv/bin/python}"
exec "$PY" \
  "$ROOT/deploy/config_carla.py" \
  --host localhost --port 2000 \
  -f "$ROOT/upstream/vision_pilot/Simulation/CARLA/ROS2/config/carla916.json"
