#!/usr/bin/env bash
# One fresh-world evidence run on the GPU rig: run-loop.sh, then a read-only
# steering/pose trace, 1 Hz frames (front, VP HUD, chase) and every container
# log, all under one directory. Rig selection comes from the environment
# exactly as for run-loop.sh (RIG_MODE, SI_MODE, RIG_JSON, VISIONPILOT_IMAGE,
# VISIONPILOT_CONF, SPAWN_INDEX, ACTUATOR_STEER_CURVE_COMP).
# Usage: ./deploy/tools/run-evidence.sh <label> [drive_seconds]
set -uo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT" || exit 1
label="${1:?expected a run label}"
drive_sec="${2:-120}"
CARLA_PY="${CARLA_PY:-/tmp/carla-venv/bin/python}"
out="${RECORD_ROOT:-/root/records}/$(date -u +%Y%m%dT%H%M%SZ)-$label"
mkdir -p "$out"

{
  env | grep -E '^(RIG_|SI_MODE|VISIONPILOT_|SPAWN_INDEX|ACTUATOR_)' | sort
  git log --oneline -1
  git submodule status
} >"$out/env.txt"

./deploy/run-loop.sh --gpu >"$out/startup.log" 2>&1
rc=$?
echo "run-loop rc=$rc at $(date -u +%T.%3N)" >>"$out/startup.log"
if ((rc != 0)); then
  tail -20 "$out/startup.log" >&2
  exit "$rc"
fi

# Start the probes after run-loop.sh: it recreates CARLA, and a client that
# connected to the old server never sees the new hero. The car only moves once
# the SI runs, so nothing of the drive is missed.
"$CARLA_PY" -u deploy/tools/passive_steering_probe.py \
  --out "$out/steer-trace.csv" --seconds "$((drive_sec + 5))" >"$out/probe.log" 2>&1 &
probe=$!
"$CARLA_PY" deploy/tools/frame_sampler.py --chase --out "$out/frames" \
  --seconds "$drive_sec" >"$out/frames.log" 2>&1 &
sampler=$!
wait "$sampler" "$probe"

for c in visionpilot scenario si carla-actuator adapter carla-bridge carla; do
  docker logs -t "openadkit-e2e-$c" >"$out/$c.log" 2>&1
done
echo "$out"
