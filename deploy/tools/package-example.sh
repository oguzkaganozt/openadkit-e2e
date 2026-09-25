#!/usr/bin/env bash
# Assemble one small, labeled example from record-example.sh's raw streams.
# The first segment is VP's actual rendered frame (or CARLA for Autoware);
# the second is CARLA's continuing camera after the selected source is cut.
# Raw files and gate logs stay on the rig; only the compressed MP4 is shipped.
# Usage: ./deploy/tools/package-example.sh /path/to/raw-run docs/media/vp-control.mp4
set -euo pipefail

run_dir="${1:?raw run directory required}"
output="${2:?output mp4 path required}"
metadata="$run_dir/metadata.env"
gate_log="$run_dir/gate.log"
[[ -s "$metadata" && -s "$gate_log" && -s "$run_dir/carla.mp4" ]] || {
  echo "Missing run metadata, gate log, or CARLA video" >&2; exit 1;
}
read_key() {
  sed -n "s/^${1}=\([a-zA-Z0-9._-]*\)$/\1/p" "$metadata" | head -1
}
mode="$(read_key mode)"
start_ms="$(read_key record_start_ms)"
cut_ms="$(read_key cut_ms)"
gate_ms="$(sed -n 's/^applied_gate_ms: \([0-9]*\)$/\1/p' "$gate_log" | head -1)"
[[ "$start_ms" =~ ^[0-9]+$ && "$cut_ms" =~ ^[0-9]+$ &&
   "$gate_ms" =~ ^[0-9]+$ ]] || {
  echo "Missing recording/cut timestamps or applied-stop gate" >&2; exit 1;
}
grep -F 'GATE PASS' "$gate_log" >/dev/null
((cut_ms > start_ms)) || { echo "Source cut preceded recording" >&2; exit 1; }
cut_sec="$(awk -v a="$start_ms" -v b="$cut_ms" 'BEGIN {printf "%.3f", (b-a)/1000}')"
stop_sec="$cut_sec"

font="/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
[[ -f "$font" ]] || { echo "Missing DejaVuSans font" >&2; exit 1; }
mkdir -p "$(dirname "$output")"

case "$mode" in
  vp-control)
    drive_label='VP_CONTROL | VP rendered frame | VP limit 4 m/s'
    stop_label="VP source cut | CARLA camera | SI stop gate ${gate_ms} ms"
    ;;
  vp-si)
    drive_label='VP_TO_SI | VP rendered frame | VP limit 4 m/s'
    stop_label="VP source cut | CARLA camera | SI stop gate ${gate_ms} ms"
    ;;
  autoware-si)
    drive_label='AUTOWARE_TO_SI | CARLA camera | normal driving'
    stop_label="Autoware source cut | CARLA camera | SI stop gate ${gate_ms} ms"
    ;;
  *) echo "Unknown mode: $mode" >&2; exit 2 ;;
esac

# Keep labels outside the camera frame. The still-running CARLA camera shows
# the full brake/deceleration while VP's MJPEG feed has legitimately stopped.
format_frame="fps=10,scale=1024:512:force_original_aspect_ratio=decrease,pad=1024:512:(ow-iw)/2:(oh-ih)/2:color=0x111820,pad=1024:576:0:64:color=0x111820,setsar=1"
drive_text="drawtext=fontfile=${font}:text='${drive_label}':x=22:y=21:fontsize=19:fontcolor=white"
stop_text="drawtext=fontfile=${font}:text='${stop_label}':x=22:y=21:fontsize=19:fontcolor=white"

if [[ "$mode" == autoware-si ]]; then
  filter="[0:v]trim=end=${stop_sec},setpts=PTS-STARTPTS,${format_frame},${drive_text}[drive];"
  filter+="[0:v]trim=start=${stop_sec}:duration=8,setpts=PTS-STARTPTS,${format_frame},${stop_text}[stop];"
  filter+="[drive][stop]concat=n=2:v=1:a=0,format=yuv420p[v]"
  inputs=(-i "$run_dir/carla.mp4")
else
  [[ -s "$run_dir/vp.mp4" ]] || { echo "Missing VP rendered-frame video" >&2; exit 1; }
  vp_duration="$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$run_dir/vp.mp4")"
  drive_end="$(awk -v c="$cut_sec" -v d="$vp_duration" \
    'BEGIN {printf "%.3f", c < d ? c : d}')"
  filter="[0:v]trim=end=${drive_end},setpts=PTS-STARTPTS,${format_frame},${drive_text}[drive];"
  filter+="[1:v]trim=start=${stop_sec}:duration=8,setpts=PTS-STARTPTS,${format_frame},${stop_text}[stop];"
  filter+="[drive][stop]concat=n=2:v=1:a=0,format=yuv420p[v]"
  inputs=(-i "$run_dir/vp.mp4" -i "$run_dir/carla.mp4")
fi

ffmpeg -hide_banner -loglevel warning -y "${inputs[@]}" \
  -filter_complex "$filter" -map '[v]' -an -c:v libx264 -preset medium \
  -crf 27 -movflags +faststart "$output"
ffprobe -v error -show_entries format=duration,size -of default=nw=1 "$output"
