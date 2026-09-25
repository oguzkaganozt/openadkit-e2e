#!/usr/bin/env bash
# Record a fresh-world E2E example on the GPU rig. Raw, synchronized VP HUD
# and CARLA camera streams are kept separately: the VP stream MUST end when
# the selected VP container is cut, while CARLA continues to show SI braking.
# The two streams can then be assembled into a labeled, compressed clip.
# Usage: VISIONPILOT_IMAGE=visionpilot:gpu-ros2-view2 \
#        ./deploy/tools/record-example.sh vp-control [drive_seconds]
# Modes: vp-control (VP command passthrough), vp-si (VP trajectory → SI),
#        autoware-si (native Autoware trajectory → SI).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
mode="${1:?expected vp-control, vp-si, or autoware-si}"
drive_sec="${2:-25}"
case "$mode" in
  vp-control)
    export RIG_MODE=vp SI_MODE=vp RIG_JSON="${RIG_JSON:-carla-rig-traffic.json}"
    source_container=openadkit-e2e-visionpilot ;;
  vp-si)
    export RIG_MODE=vp SI_MODE=si RIG_JSON="${RIG_JSON:-carla-rig-traffic.json}"
    source_container=openadkit-e2e-visionpilot ;;
  autoware-si)
    # The current Autoware empty-scene fixture does not accept NPCs. This
    # video must be labeled as the empty-scene configuration.
    export RIG_MODE=autoware SI_MODE=si RIG_JSON=carla-rig-autoware-demo.json
    source_container=openadkit-e2e-autoware-planning ;;
  *) echo "Unknown mode: $mode" >&2; exit 2 ;;
esac
if [[ ! "$drive_sec" =~ ^[0-9]+$ ]] || ((drive_sec < 10)); then
  echo "drive_seconds must be an integer >= 10" >&2
  exit 2
fi
export PREVIEW_AUTO=0
if [[ "$RIG_MODE" == vp ]]; then
  export VISIONPILOT_IMAGE="${VISIONPILOT_IMAGE:-visionpilot:gpu-ros2-view2}"
  export VISIONPILOT_CONF="${VISIONPILOT_CONF:-vision_pilot.demo.conf}"
fi

out_dir="${RECORD_ROOT:-/root/records}/$(date -u +%Y%m%dT%H%M%SZ)-$mode"
mkdir -p "$out_dir"
vp_pid=""
carla_pid=""
stop_recorders() {
  local pid
  for pid in "$vp_pid" "$carla_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
      wait "$pid" || true
    fi
  done
  vp_pid=""
  carla_pid=""
}
trap stop_recorders EXIT

# Every measurement/demo gets a fresh CARLA world. Never retry an injected
# fault in-place: SI_STOP latches and requires explicit re-enable.
./deploy/run-loop.sh --gpu >"$out_dir/startup.log" 2>&1 || {
  tail -60 "$out_dir/startup.log" >&2
  exit 1
}

latest_pose() {
  docker logs --tail 8 openadkit-e2e-scenario 2>&1 |
    grep -a 'INFO: pose' | tail -1
}
healthy=0
for _ in $(seq 1 100); do
  pose="$(latest_pose || true)"
  v="$(grep -oE 'v=[0-9.]+' <<<"$pose" | tail -1 | cut -d= -f2 || true)"
  lane="$(grep -oE 'lane_off=[-+0-9.]+' <<<"$pose" | tail -1 | cut -d= -f2 || true)"
  control="$(docker logs --tail 8 openadkit-e2e-carla-actuator 2>&1 |
    grep -a 'applied control #' | tail -1 || true)"
  if [[ -n "$v" && -n "$lane" && "$control" == *"decision=0"* ]] &&
    awk -v v="$v" -v l="$lane" 'BEGIN {exit !(v >= 1 && l >= -1.2 && l <= 1.2)}'; then
    healthy=$((healthy + 1))
    if ((healthy >= 5)); then break; fi
  else
    healthy=0
  fi
  sleep 1
done
if ((healthy < 5)); then
  echo "No healthy moving window; do not record/cut this world" >&2
  exit 1
fi

record_start_iso="$(date --iso-8601=seconds)"
record_start_ms="$(date +%s%3N)"
printf 'mode=%s\nrig=%s\nsi=%s\nscene=%s\nimage=%s\nconfig=%s\nrecord_start_ms=%s\n' \
  "$mode" "$RIG_MODE" "$SI_MODE" "$RIG_JSON" \
  "${VISIONPILOT_IMAGE:-none}" "${VISIONPILOT_CONF:-none}" "$record_start_ms" \
  >"$out_dir/metadata.env"

ffmpeg -y -hide_banner -loglevel warning -use_wallclock_as_timestamps 1 \
  -f mpjpeg -i http://127.0.0.1:8090/stream \
  -vf 'pad=ceil(iw/2)*2:ceil(ih/2)*2:0:0' \
  -c:v libx264 -pix_fmt yuv420p -preset veryfast -crf 23 -vsync vfr \
  "$out_dir/carla.mp4" >"$out_dir/carla-ffmpeg.log" 2>&1 &
carla_pid=$!
if [[ "$RIG_MODE" == vp ]]; then
  ffmpeg -y -hide_banner -loglevel warning -use_wallclock_as_timestamps 1 \
    -f mpjpeg -i http://127.0.0.1:8080/mjpeg \
    -c:v libx264 -pix_fmt yuv420p -preset veryfast -crf 23 -vsync vfr \
    "$out_dir/vp.mp4" >"$out_dir/vp-ffmpeg.log" 2>&1 &
  vp_pid=$!
fi
echo "Recording $mode: $out_dir"
sleep 2
for pid in "$carla_pid" "$vp_pid"; do
  [[ -z "$pid" ]] || kill -0 "$pid" 2>/dev/null || {
    cat "$out_dir"/*-ffmpeg.log >&2
    echo "FFmpeg exited before the recording began" >&2
    exit 1
  }
done

for ((drive_end = $(date +%s) + drive_sec; $(date +%s) < drive_end; )); do
  if docker logs --since "$record_start_iso" --tail 200 openadkit-e2e-si 2>&1 |
    grep -a 'SI_STOP latched' >/dev/null; then
    echo "SI_STOP latched organically during the drive; discard this world" >&2
    exit 1
  fi
  pose="$(latest_pose || true)"
  v="$(grep -oE 'v=[0-9.]+' <<<"$pose" | tail -1 | cut -d= -f2 || true)"
  lane="$(grep -oE 'lane_off=[-+0-9.]+' <<<"$pose" | tail -1 | cut -d= -f2 || true)"
  if [[ -z "$v" || -z "$lane" ]] ||
    ! awk -v v="$v" -v l="$lane" 'BEGIN {exit !(v >= 0.5 && l >= -1.5 && l <= 1.5)}'; then
    echo "Unhealthy drive: ${pose:-no pose}; discard this world" >&2
    exit 1
  fi
  sleep 1
done

cut_ms="$(date +%s%3N)"
printf 'cut_ms=%s\n' "$cut_ms" >>"$out_dir/metadata.env"
SOURCE_CONTAINER="$source_container" MIN_SPEED_MPS=0.5 PREFLIGHT_SEC=2 \
  WAIT_SEC=25 ./deploy/tools/stop-gate-test.sh >"$out_dir/gate.log" 2>&1 || {
  cat "$out_dir/gate.log" >&2
  exit 1
}
grep -F 'GATE PASS' "$out_dir/gate.log" >/dev/null
# Keep recording after the source is gone so CARLA shows the stopped car.
sleep 7
stop_recorders

docker logs --since "$record_start_iso" openadkit-e2e-scenario \
  >"$out_dir/scenario.log" 2>&1
docker logs --since "$record_start_iso" openadkit-e2e-si \
  >"$out_dir/si.log" 2>&1
if grep -a 'COLLISION' "$out_dir/scenario.log" >/dev/null; then
  echo "Collision during the recording; not a clean example" >&2
  exit 1
fi
grep -a 'SI_STOP latched' "$out_dir/si.log" >/dev/null
if ! grep -aE 'INFO: pose.*v=0\.0[0-4]' "$out_dir/scenario.log" >/dev/null; then
  echo "No ground-truth stop visible in scenario log" >&2
  exit 1
fi
videos=("$out_dir/carla.mp4")
if [[ "$RIG_MODE" == vp ]]; then videos+=("$out_dir/vp.mp4"); fi
for video in "${videos[@]}"; do
  [[ -s "$video" ]] || { echo "Missing recording: $video" >&2; exit 1; }
  duration="$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$video")"
  minimum=$((drive_sec - 2))
  if [[ "$video" == */carla.mp4 ]]; then minimum=$((drive_sec + 5)); fi
  awk -v d="$duration" -v m="$minimum" 'BEGIN {exit !(d >= m)}' || {
    echo "Recording too short: $video (${duration}s, need ${minimum}s)" >&2
    exit 1
  }
  echo "$video duration=${duration}s size=$(stat -c%s "$video")B"
done
echo "GOOD_EXAMPLE=$out_dir"
