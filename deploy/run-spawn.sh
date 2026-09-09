#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
pkill -f "$ROOT/deploy/config_carla.py" 2>/dev/null || true
exec docker compose --env-file "$ROOT/deploy/config.env" --file "$ROOT/deploy/docker-compose.yaml" up -d spawn
