#!/usr/bin/env bash
# Every stop injection starts with a newly created CARLA server, world and
# containers. Never re-enable and retry a gate on a previously stopped world.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
: "${SOURCE_CONTAINER:?SOURCE_CONTAINER is required}"
RIG_MODE="${RIG_MODE:-vp}"
SI_MODE="${SI_MODE:-si}"
BOTH_SOURCES="${BOTH_SOURCES:-0}"
RIG_JSON="${RIG_JSON:-carla-rig-empty.json}"
MAX_ATTEMPTS="${MAX_ATTEMPTS:-3}"
EVIDENCE_DIR="${EVIDENCE_DIR:-/tmp/si-gate-evidence}"
export RIG_MODE SI_MODE RIG_JSON BOTH_SOURCES PREVIEW_AUTO=0
if [[ -e "$EVIDENCE_DIR" ]] && [[ -n "$(find "$EVIDENCE_DIR" -mindepth 1 -print -quit)" ]]; then
  echo "Evidence directory is not empty: $EVIDENCE_DIR (choose a new path)" >&2
  exit 2
fi

cleanup() {
  (cd "$ROOT/deploy" && docker compose --env-file config.env --profile vp \
    --profile autoware down --remove-orphans --timeout 10 >/dev/null 2>&1) || true
}
trap cleanup EXIT

wait_for_moving() {
  local max_sec="${WAIT_READY_SEC:-60}"
  local min_speed="${MIN_SPEED_MPS:-1.0}"
  local started_at="$(date --iso-8601=seconds)"
  for ((i = 0; i < max_sec; ++i)); do
    if docker logs --since "$started_at" openadkit-e2e-si 2>&1 | grep -qa "SI_STOP latched"; then
      echo "SI latched before injection; discard this world" >&2
      return 1
    fi
    local last_control speed_line speed
    last_control="$(docker logs --tail 6 openadkit-e2e-carla-actuator 2>&1 \
      | grep -a 'applied control #' | tail -1 || true)"
    if [[ "$last_control" == *"decision=1"* ]]; then
      echo "SI_STOP was already applied before injection; discard this world" >&2
      return 1
    fi
    speed_line="$(docker logs --tail 4 openadkit-e2e-scenario 2>&1 \
      | grep -a 'INFO: pose' | tail -1 || true)"
    speed="$(grep -oE 'v=[0-9.]+' <<<"$speed_line" | cut -d= -f2 || true)"
    if [[ "$last_control" == *"decision=0"* ]] && [[ -n "$speed" ]] && \
      awk -v s="$speed" -v m="$min_speed" 'BEGIN {exit !(s >= m)}'; then
      echo "fresh NORMAL+moving preflight: $speed m/s"
      return 0
    fi
    sleep 1
  done
  echo "no fresh NORMAL+moving window within ${max_sec}s" >&2
  return 1
}

probe_candidates() {
  local remaining="${1:-}"
  docker run --rm --network host --ipc host --entrypoint bash \
    -e ROS_DOMAIN_ID=2 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI=file:///autoware/cyclonedds.xml \
    -e EXPECTED_SOURCE="$RIG_MODE" -e EXPECT_REMAINING="$remaining" \
    -e RUN_SEC="${ISOLATION_SEC:-10}" \
    -v "$ROOT/deploy/tools:/opt/tools:ro" \
    -v "$ROOT/deploy/config/cyclonedds.xml:/autoware/cyclonedds.xml:ro" \
    openadkit-e2e-adapter:latest -lc \
    'source /opt/ros/humble/setup.bash && source /opt/autoware/setup.bash && python3 /opt/tools/candidate_isolation_probe.py'
}

probe_vp_passthrough() {
  local out="$1"
  capture() {
    local side="$1" domain="$2"
    docker run --rm --network host --ipc host --entrypoint bash \
      -e ROS_DOMAIN_ID="$domain" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
      -e CYCLONEDDS_URI=file:///autoware/cyclonedds.xml \
      -v "$ROOT/deploy/tools:/opt/tools:ro" -v "$out:/evidence:rw" \
      -v "$ROOT/deploy/config/cyclonedds.xml:/autoware/cyclonedds.xml:ro" \
      openadkit-e2e-adapter:latest -lc \
      "source /opt/ros/humble/setup.bash && source /opt/autoware/setup.bash && python3 /opt/tools/vp_passthrough_check.py capture-$side --out /evidence/$side.jsonl --sec 12"
  }
  capture vp 1 >"$out/command-capture.log" 2>&1 &
  local vp_pid=$!
  capture si 2 >"$out/approval-capture.log" 2>&1 &
  local si_pid=$!
  local result=0
  wait "$vp_pid" || result=1
  wait "$si_pid" || result=1
  if ((result != 0)); then
    return 1
  fi
  MIN_MATCHED_CYCLES=10 python3 "$ROOT/deploy/tools/vp_passthrough_check.py" \
    join "$out/vp.jsonl" "$out/si.jsonl"
}

for attempt in $(seq 1 "$MAX_ATTEMPTS"); do
  out="$EVIDENCE_DIR/attempt-$attempt"
  mkdir -p "$out"
  echo "=== fresh ${RIG_MODE}/${SI_MODE} attempt ${attempt} ==="
  if "$ROOT/deploy/run-loop.sh" --gpu >"$out/run-loop.log" 2>&1; then
    if wait_for_moving >"$out/ready.log" 2>&1 && \
      { [[ "$BOTH_SOURCES" != 1 ]] || probe_candidates >"$out/isolation-before.log" 2>&1; } && \
      { [[ "$SI_MODE" != vp || "${SKIP_PASSTHROUGH:-0}" == 1 ]] || probe_vp_passthrough "$out" >"$out/passthrough.log" 2>&1; } && \
      SOURCE_CONTAINER="$SOURCE_CONTAINER" \
        MIN_SPEED_MPS="${MIN_SPEED_MPS:-1.0}" PREFLIGHT_SEC="${PREFLIGHT_SEC:-3}" \
        "$ROOT/deploy/tools/stop-gate-test.sh" >"$out/gate.log" 2>&1; then
      if [[ "$BOTH_SOURCES" == 1 ]]; then
        other=vp
        [[ "$RIG_MODE" == vp ]] && other=autoware
        if probe_candidates "$other" >"$out/isolation-after.log" 2>&1; then
          result=0
        else
          result=1
        fi
      else
        result=0
      fi
    else
      result=1
    fi
  else
    result=1
  fi
  for name in si adapter visionpilot autoware-planning carla-actuator \
              carla-bridge scenario bridge; do
    docker logs "openadkit-e2e-$name" >"$out/$name.log" 2>&1 || true
  done
  sha256sum "$ROOT/upstream/autoware-safety-island/build/freertos-posix/actuation_freertos" \
    >"$out/si-binary.sha256"
  cat "$out/ready.log" 2>/dev/null || true
  cat "$out/isolation-before.log" 2>/dev/null || true
  cat "$out/passthrough.log" 2>/dev/null || true
  cat "$out/gate.log" 2>/dev/null || cat "$out/run-loop.log" >&2
  cat "$out/isolation-after.log" 2>/dev/null || true
  if ((result == 0)); then
    echo "CLEAN_GATE_PASS attempt=$attempt evidence=$out"
    exit 0
  fi
  echo "attempt $attempt failed; discarding this world before the next attempt" >&2
done
echo "CLEAN_GATE_FAIL evidence=$EVIDENCE_DIR" >&2
exit 1
