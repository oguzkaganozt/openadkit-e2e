#!/usr/bin/env bash
# Regenerate the VisionPilot preprocessing homography C from the rig's
# deploy/config/H.yaml. C and the H mounted at runtime MUST be a matched pair:
# C is normally generated at image-build time from the in-tree example H, but
# this rig bind-mounts its own calibrated H at run time. Run this tool
# whenever H.yaml changes and commit the regenerated C.
#
# Requires python3 with numpy and OpenCV (python3-opencv or pip opencv).
# Usage: ./deploy/tools/gen-homography-c.sh
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
VP="$ROOT/upstream/vision_pilot/VisionPilot"
script="$VP/scripts/find_homography_C_matrix.py"
h_yaml="$ROOT/deploy/config/H.yaml"
out="$ROOT/deploy/config/homography_C_matrix.yaml"
[[ -s "$h_yaml" ]] || { echo "Missing $h_yaml" >&2; exit 1; }
[[ -s "$script" ]] || { echo "Missing $script" >&2; exit 1; }

# The upstream script reads Path('../config/H.yaml') relative to its working
# directory; give it a private tree whose config/H.yaml is the rig's H.
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
mkdir -p "$work/config" "$work/scripts"
cp "$h_yaml" "$work/config/H.yaml"
(
  cd "$work/scripts"
  python3 "$script" --output "$out"
)
echo "Wrote $out"
