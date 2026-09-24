#!/usr/bin/env bash
# VPS cadence measurement for SI #62: CARLA + bridge + VisionPilot only.
# No SI/adapter/domain-bridge: this measures healthy publication cadence and
# source stamps of the CARLA bridge and VP outputs on ROS domain 1.
#
# SKIP_STACK=1 probes an already running stack without touching containers.
set -euo pipefail
DEPLOY="${DEPLOY:-/root/openadkit-e2e/deploy}"
cd "$DEPLOY"
COMPOSE=(docker compose --env-file config.env --profile vp
  -f docker-compose.yaml -f /root/vps-carla-override.yaml)
AUTOWARE_IMAGE=ghcr.io/autowarefoundation/autoware:universe-20250207
MSGS_IMAGE=visionpilot-msgs-humble
RUN_SEC="${RUN_SEC:-150}"
OUT_JSON="${OUT_JSON:-/root/cadence-latest.json}"
PROBE_LOG="${PROBE_LOG:-/root/cadence-latest.log}"
PROBE_PY="${PROBE_PY:-/root/openadkit-e2e/deploy/tools/cadence_probe_vp.py}"
CARLA_PY="${CARLA_PY:-/tmp/opencode/carla-smoke/venv/bin/python}"
SKIP_STACK="${SKIP_STACK:-0}"

wait_for_log() {
  local container="$1" pattern="$2" name="$3" since="$4"
  for _ in $(seq 1 120); do
    if docker logs --since "$since" "$container" 2>&1 | grep -q "$pattern"; then
      echo "$name ready"
      return 0
    fi
    sleep 1
  done
  echo "$name did not become ready" >&2
  docker logs --since "$since" "$container" | tail -40 >&2
  return 1
}

if [[ "$SKIP_STACK" != "1" ]]; then
  echo "== stopping any previous measurement stack =="
  "${COMPOSE[@]}" down --remove-orphans >/dev/null 2>&1 || true

  echo "== carla =="
  "${COMPOSE[@]}" up -d carla
  for _ in $(seq 1 90); do
    if "$CARLA_PY" -c "import carla; c=carla.Client('127.0.0.1',2000); c.set_timeout(2); c.get_world()" >/dev/null 2>&1; then
      echo "CARLA RPC ready"; break
    fi
    sleep 2
  done

  echo "== scenario =="
  started_at="$(date --iso-8601=seconds)"
  "${COMPOSE[@]}" up -d --force-recreate scenario
  wait_for_log openadkit-e2e-scenario "ego up" "scenario" "$started_at"

  echo "== carla bridge =="
  started_at="$(date --iso-8601=seconds)"
  "${COMPOSE[@]}" up -d --force-recreate carla-bridge
  wait_for_log openadkit-e2e-carla-bridge "camera frame #" "bridge camera" "$started_at"

  echo "== visionpilot =="
  started_at="$(date --iso-8601=seconds)"
  "${COMPOSE[@]}" up -d --force-recreate visionpilot
  wait_for_log openadkit-e2e-visionpilot "plan: tyre=" "VP planning" "$started_at"
else
  echo "== using the running stack (SKIP_STACK=1) =="
fi

echo "== cadence probe (${RUN_SEC}s, Humble/CycloneDDS) =="
docker run --rm --network host --ipc host \
  -e ROS_DOMAIN_ID=1 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///autoware/cyclonedds.xml \
  -e RUN_SEC="$RUN_SEC" \
  -e OUT_JSON=/tmp/cadence_probe_vp.json \
  -v "$DEPLOY/config/cyclonedds.xml:/autoware/cyclonedds.xml:ro" \
  -v "$PROBE_PY:/opt/probe/cadence_probe_vp.py:ro" \
  --entrypoint bash "$MSGS_IMAGE" -lc \
  "source /opt/ros/humble/setup.bash && source /opt/autoware/setup.bash &&
   timeout $((RUN_SEC + 90)) python3 -u /opt/probe/cadence_probe_vp.py" | tee "$PROBE_LOG"

echo "probe output saved to $PROBE_LOG"
docker logs --since "$(date --iso-8601=seconds --date='3 minutes ago')" openadkit-e2e-visionpilot 2>&1 | tail -40 > "${PROBE_LOG}.vp-tail" || true