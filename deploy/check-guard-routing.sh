#!/usr/bin/env bash
# Phase 2 routing audit: the bridge must never see raw follower Control.
# Run by run-loop.sh before SI starts; run it in CI the same way.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CFG="$ROOT/deploy/config/bridge-config.yaml"
fail=0
if grep -q "control/trajectory_follower/control_cmd" "$CFG"; then
  echo "REJECTED: raw follower Control is still routed to domain 1" >&2
  fail=1
fi
for t in "control/guard/control_cmd" "guard/inhibit_trajectory" "guard/state"; do
  if ! grep -q "$t" "$CFG"; then
    echo "REJECTED: $t missing from bridge-config.yaml" >&2
    fail=1
  fi
done
if ! grep -q 'control_topic", "/control/guard/control_cmd"' \
  "$ROOT/deploy/nodes/carla_bridge.py"; then
  echo "REJECTED: bridge does not default to the guard-approved topic" >&2
  fail=1
fi
if [[ "$fail" == 0 ]]; then
  echo "guard routing OK: approved Control only"
fi
exit "$fail"
