#!/usr/bin/env bash
# Run the Safety Island trajectory follower on the host (FreeRTOS POSIX).
# Compose does not start this; SI is the isolated compute.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SI="$ROOT/upstream/autoware-safety-island"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$ROOT/deploy/cyclonedds.xml"
BIN="$SI/build/freertos-posix/actuation_freertos"
if [[ ! -x "$BIN" ]]; then
  echo "SI binary missing: $BIN" >&2
  echo "Build first:" >&2
  echo "  cd $SI && ./build.sh --platform freertos-posix -d build/freertos-posix --control-output DDS_ONLY --dds-interface <nic>" >&2
  exit 1
fi

# Multiple SI processes publish conflicting control_cmd messages.
pkill -f "$BIN" 2>/dev/null || true
for _ in $(seq 1 20); do
  pgrep -f "$BIN" >/dev/null || break
  sleep 0.1
done
if pgrep -f "$BIN" >/dev/null; then
  echo "Existing SI process did not stop" >&2
  exit 1
fi

exec "$BIN"
