#!/usr/bin/env bash
# Isolated CARLA world per VP cycle-regression attempt; never retry on a
# latched world, even when the probe or an unrelated publisher stalls.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
EVIDENCE_DIR="${EVIDENCE_DIR:-/tmp/si-vp-replay-evidence}"
MAX_ATTEMPTS="${MAX_ATTEMPTS:-2}"
export RIG_MODE=vp SI_MODE=si RIG_JSON=carla-rig-empty.json PREVIEW_AUTO=0
if [[ -e "$EVIDENCE_DIR" ]] && [[ -n "$(find "$EVIDENCE_DIR" -mindepth 1 -print -quit)" ]]; then
  echo "Evidence directory is not empty: $EVIDENCE_DIR" >&2
  exit 2
fi
cleanup() {
  (cd "$ROOT/deploy" && docker compose --env-file config.env --profile vp \
    down --remove-orphans --timeout 10 >/dev/null 2>&1) || true
}
trap cleanup EXIT

for attempt in $(seq 1 "$MAX_ATTEMPTS"); do
  out="$EVIDENCE_DIR/attempt-$attempt"
  mkdir -p "$out"
  echo "=== fresh VP replay attempt $attempt ==="
  result=1
  if "$ROOT/deploy/run-loop.sh" --gpu >"$out/run-loop.log" 2>&1; then
    if docker run --rm --network host --ipc host --entrypoint bash \
      -e ROS_DOMAIN_ID=2 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
      -e CYCLONEDDS_URI=file:///autoware/cyclonedds.xml \
      -v "$ROOT/deploy/tools:/opt/tools:ro" \
      -v "$ROOT/deploy/config/cyclonedds.xml:/autoware/cyclonedds.xml:ro" \
      openadkit-e2e-adapter:latest -lc \
      'source /opt/ros/humble/setup.bash && source /opt/autoware/setup.bash && python3 /opt/tools/candidate_replay_probe.py' \
      >"$out/replay.log" 2>&1; then
      result=0
    fi
  fi
  for name in si adapter visionpilot carla-actuator scenario bridge; do
    docker logs "openadkit-e2e-$name" >"$out/$name.log" 2>&1 || true
  done
  sha256sum "$ROOT/upstream/autoware-safety-island/build/freertos-posix/actuation_freertos" \
    >"$out/si-binary.sha256"
  cat "$out/replay.log" 2>/dev/null || cat "$out/run-loop.log" >&2
  if ((result == 0)) && \
    grep -qa 'SI_STOP latched: vp trajectory candidate regression' "$out/si.log" && \
    grep -qa 'GATE STOP_APPLIED' "$out/carla-actuator.log"; then
    echo "VP_REPLAY_PASS attempt=$attempt evidence=$out"
    exit 0
  fi
  echo "attempt $attempt failed; discarding this world" >&2
done
echo "VP_REPLAY_FAIL evidence=$EVIDENCE_DIR" >&2
exit 1
