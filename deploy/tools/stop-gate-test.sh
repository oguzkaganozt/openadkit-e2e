#!/usr/bin/env bash
# Fault-detection -> CARLA-applied-stop gate measurement (E2E #2).
#
# Preconditions: a rig run is already driving (run-loop.sh with RIG_MODE=vp
# SI_MODE=si) and the actuator log shows applied controls. This injects a
# candidate-source outage by stopping the source container, then parses:
#
#   - the SI latch line in the si container log (its own HH:MM:SS.mmm wall
#     clock) = SI fault detection;
#   - the actuator's GATE STOP_RECV / GATE STOP_APPLIED wall_ns lines, where
#     STOP_APPLIED is the first CARLA frame with brake and no throttle after
#     the stop request.
#
# Reported:
#   detection_latency_ms = SI detection - injection   (reported separately)
#   applied_gate_ms      = first applied stop frame - SI detection
#
# The gate is SI detection -> CARLA-applied stop, per the shared contract;
# it does not include the source watchdog that leads up to detection.
set -euo pipefail

SI_CONTAINER="${SI_CONTAINER:-openadkit-e2e-si}"
ACTUATOR_CONTAINER="${ACTUATOR_CONTAINER:-openadkit-e2e-carla-actuator}"
SCENARIO_CONTAINER="${SCENARIO_CONTAINER:-openadkit-e2e-scenario}"
SOURCE_CONTAINER="${SOURCE_CONTAINER:-openadkit-e2e-adapter}"
WAIT_SEC="${WAIT_SEC:-20}"
PREFLIGHT_SEC="${PREFLIGHT_SEC:-10}"
MIN_SPEED_MPS="${MIN_SPEED_MPS:-1.0}"

log_section() {
  echo
  echo "== $1 =="
}

# Preflight: the rig must be healthy and driving; a pre-existing latch would
# make the injection measure nothing new.
if docker logs --since "${PREFLIGHT_SEC}s" "$SI_CONTAINER" 2>&1 | grep -qa "SI_STOP latched"; then
  echo "FAIL: SI latched within the last ${PREFLIGHT_SEC}s; fix the rig before measuring" >&2
  exit 1
fi
# The measurement must start from a moving car: read CARLA ground-truth speed
# from the scenario telemetry, not from the actuator's periodic log lines.
speed_line="$(docker logs --since 5s "$SCENARIO_CONTAINER" 2>&1 | grep -a "INFO: pose" | tail -1)"
speed="$(grep -oE "v=[0-9.]+" <<<"$speed_line" | cut -d= -f2)"
if [[ -z "$speed" ]] || ! awk -v s="$speed" -v m="$MIN_SPEED_MPS" 'BEGIN {exit !(s >= m)}'; then
  echo "FAIL: CARLA ground speed ${speed:-unknown} < ${MIN_SPEED_MPS} m/s; wait for driving" >&2
  exit 1
fi
echo "preflight ground speed: ${speed} m/s"

inject_iso="$(date --iso-8601=seconds)"
inject_ms="$(date +%s%3N)"
echo "Injecting candidate-source outage: docker stop $SOURCE_CONTAINER"
docker stop "$SOURCE_CONTAINER" >/dev/null

find_first() {
  local container="$1"
  local pattern="$2"
  local deadline=$((SECONDS + WAIT_SEC))
  while ((SECONDS < deadline)); do
    local line
    line="$(docker logs --since "$inject_iso" "$container" 2>&1 | grep -m1 -a -- "$pattern" || true)"
    if [[ -n "$line" ]]; then
      printf '%s\n' "$line"
      return 0
    fi
    sleep 0.1
  done
  return 1
}

latch_line="$(find_first "$SI_CONTAINER" "SI_STOP latched")" || {
  echo "FAIL: no SI_STOP latch within ${WAIT_SEC}s of the injection" >&2
  exit 1
}
recv_line="$(find_first "$ACTUATOR_CONTAINER" "GATE STOP_RECV")" || {
  echo "FAIL: actuator did not receive an SI_STOP request" >&2
  exit 1
}
applied_line="$(find_first "$ACTUATOR_CONTAINER" "GATE STOP_APPLIED")" || {
  echo "FAIL: no CARLA stop frame applied after the SI_STOP" >&2
  exit 1
}

latch_clock="$(grep -oE '[0-9]{2}:[0-9]{2}:[0-9]{2}\.[0-9]{3}' <<<"$latch_line" | head -1)"
[[ -n "$latch_clock" ]] || { echo "FAIL: could not parse the SI latch clock" >&2; exit 1; }
latch_ms="$(date -d "$latch_clock" +%s%3N)"
# Guard against crossing local midnight between injection and latch.
if ((latch_ms < inject_ms)); then
  latch_ms=$((latch_ms + 86400000))
fi

recv_ns="$(grep -oE 'wall_ns=[0-9]+' <<<"$recv_line" | head -1 | cut -d= -f2)"
applied_ns="$(grep -oE 'wall_ns=[0-9]+' <<<"$applied_line" | head -1 | cut -d= -f2)"
[[ -n "$recv_ns" && -n "$applied_ns" ]] || {
  echo "FAIL: actuator gate lines lack wall_ns values" >&2
  exit 1
}

detection_ms=$((latch_ms - inject_ms))
transport_ms=$((recv_ns / 1000000 - latch_ms))
applied_gate_ms=$((applied_ns / 1000000 - latch_ms))

log_section "SI detection (source watchdog -> SI_STOP latch)"
echo "$latch_line"
log_section "actuator receipt / first applied stop frame"
echo "$recv_line"
echo "$applied_line"
log_section "result"
printf 'detection_latency_ms: %d\n' "$detection_ms"
printf 'si_to_actuator_transport_ms: %d\n' "$transport_ms"
printf 'applied_gate_ms: %d\n' "$applied_gate_ms"
if ((applied_gate_ms <= 500)); then
  echo "GATE PASS (<= 500 ms from SI detection to CARLA-applied stop)"
else
  echo "GATE FAIL (> 500 ms)" >&2
  exit 1
fi