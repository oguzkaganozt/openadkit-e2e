#!/usr/bin/env bash
# Fetch and verify the pinned Town04 map for the Autoware planning
# configuration (E2E #4). Source: "CARLA Autoware Contents" (the maps
# Autoware's own autoware_carla_interface docs use), y-axis inverted, used
# with map_projector_info.yaml: projector_type: Local -- the same CARLA
# (x, -y) map convention this rig's bridge publishes.
#
# Usage: deploy/tools/fetch-town04-map.sh [target_dir]
#   default target: deploy/maps/town04
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
TARGET="${1:-$ROOT/deploy/maps/town04}"
BASE_URL="https://bitbucket.org/carla-simulator/autoware-contents/raw/master/maps"

declare -A FILES=(
  ["lanelet2_map.osm"]="vector_maps/lanelet2/Town04.osm"
  ["pointcloud_map.pcd"]="point_cloud_maps/Town04.pcd"
)
declare -A SHA256=(
  ["lanelet2_map.osm"]="41e9149cfc74621d48a4af22b66fca90a12e51792779913106c9ac2ab73019ef"
  ["pointcloud_map.pcd"]="9703b93441a5f5854b6e7cbb31a4d0d4043fe1080b57dfd5c784d5095f811593"
)

mkdir -p "$TARGET"
printf 'projector_type: Local\n' > "$TARGET/map_projector_info.yaml"

for name in "${!FILES[@]}"; do
  dest="$TARGET/$name"
  if [[ -f "$dest" ]] && \
    printf '%s  %s\n' "${SHA256[$name]}" "$dest" | sha256sum -c - --status 2>/dev/null; then
    echo "$name: checksum OK"
    continue
  fi
  echo "$name: downloading ${FILES[$name]} ..."
  tmp="$dest.download.$$"
  trap 'rm -f "$tmp"' EXIT
  curl --fail --location --retry 3 --max-time 1800 -A "Mozilla/5.0" \
    --output "$tmp" "$BASE_URL/${FILES[$name]}"
  printf '%s  %s\n' "${SHA256[$name]}" "$tmp" | sha256sum -c -
  mv "$tmp" "$dest"
  trap - EXIT
  echo "$name: installed"
done

echo "Town04 map ready at $TARGET"