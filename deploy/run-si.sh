#!/usr/bin/env bash
# Start the Safety Island compose service (FreeRTOS POSIX binary).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="$ROOT/upstream/autoware-safety-island/build/freertos-posix/actuation_freertos"
if [[ ! -x "$BIN" ]]; then
  echo "SI binary missing: $BIN" >&2
  echo "Build first: $ROOT/deploy/build.sh --dds-interface <nic>" >&2
  exit 1
fi
pkill -f "$BIN" 2>/dev/null || true
exec docker compose --env-file "$ROOT/deploy/config.env" --file "$ROOT/deploy/docker-compose.yaml" up -d si
