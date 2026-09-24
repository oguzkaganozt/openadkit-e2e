#!/usr/bin/env bash
# Adapter fault-window check: while the probe watches the adapter output,
# stop VisionPilot and confirm the adapter goes silent (no stop
# trajectories), then restart VP and confirm transcription resumes.
#
# Run on the rig host with the stack up (carla, scenario, carla-bridge,
# visionpilot, adapter):
#   bash deploy/tools/adapter-fault-test.sh
set -u
DEPLOY="${DEPLOY:-/root/openadkit-e2e/deploy}"
cd "$DEPLOY"
PROBE_IMAGE="${PROBE_IMAGE:-openadkit-e2e-adapter:latest}"
RUN_SEC="${RUN_SEC:-110}"
STOP_AT="${STOP_AT:-35}"
RESTART_AT="${RESTART_AT:-75}"

echo "=== probe start $(date +%T) (RUN_SEC=$RUN_SEC) ==="
docker run --rm --network host --ipc host \
  -e ROS_DOMAIN_ID=1 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///autoware/cyclonedds.xml \
  -e RUN_SEC="$RUN_SEC" \
  -v "$DEPLOY/config/cyclonedds.xml:/autoware/cyclonedds.xml:ro" \
  -v "$DEPLOY/tools/adapter_probe.py:/opt/probe/adapter_probe.py:ro" \
  --entrypoint bash "$PROBE_IMAGE" -lc \
  "source /opt/ros/humble/setup.bash && source /opt/autoware/setup.bash && timeout $((RUN_SEC + 60)) python3 -u /opt/probe/adapter_probe.py" &
PROBE_PID=$!
sleep "$STOP_AT"
echo "=== VP stop at $(date +%T) ==="
docker stop openadkit-e2e-visionpilot >/dev/null
sleep $((RESTART_AT - STOP_AT))
echo "=== VP restart at $(date +%T) ==="
docker start openadkit-e2e-visionpilot >/dev/null
wait "$PROBE_PID"
echo "=== probe end $(date +%T) ==="
echo "=== adapter log tail ==="
docker logs --tail 6 openadkit-e2e-adapter