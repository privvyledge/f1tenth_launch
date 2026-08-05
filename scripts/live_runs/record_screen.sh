#!/usr/bin/env bash
# record_screen.sh — capture RViz (or the whole screen) to an mp4 for the demo.
#
#   ./record_screen.sh sensors_rgb            # full screen until Ctrl-C
#   ./record_screen.sh mapping_2d --secs 45   # fixed length
#   ./record_screen.sh nav2 --window          # click a window to capture it
#
# Writes to $VIDEO_ROOT (on the SSD). Uses ffmpeg x11grab; requires DISPLAY.

set -uo pipefail
cd "$(dirname "$0")"
# shellcheck source=00_env.sh
source ./00_env.sh

NAME="${1:-clip}"; shift || true
SECS=""
GEOMETRY=""
FPS=25

while (( $# )); do
  case "$1" in
    --secs)   SECS="$2"; shift ;;
    --fps)    FPS="$2"; shift ;;
    --window) GEOMETRY=window ;;
    -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
  shift
done

command -v ffmpeg >/dev/null 2>&1 \
  || die "ffmpeg not installed in the container.
      Either 'apt-get install -y ffmpeg', or record from the host instead:
        ssh gosling1 has the same X display; run ffmpeg there,
      or use any desktop screen recorder against DISPLAY=:0."

[[ -n "${DISPLAY:-}" ]] || die "DISPLAY not set — X forwarding is required"

ensure_dirs
OUT="${VIDEO_ROOT}/${NAME}_$(date +%H%M%S).mp4"

# Resolve the capture region.
if [[ "$GEOMETRY" == window ]]; then
  command -v xwininfo >/dev/null 2>&1 || die "xwininfo not installed (x11-utils)"
  banner "click the window you want to capture"
  INFO="$(xwininfo)"
  W=$(grep -oP 'Width: \K[0-9]+'       <<<"$INFO")
  H=$(grep -oP 'Height: \K[0-9]+'      <<<"$INFO")
  X=$(grep -oP 'Absolute upper-left X: \K[0-9]+' <<<"$INFO")
  Y=$(grep -oP 'Absolute upper-left Y: \K[0-9]+' <<<"$INFO")
else
  read -r W H < <(xdpyinfo 2>/dev/null | awk '/dimensions:/ {split($2,a,"x"); print a[1], a[2]}')
  X=0; Y=0
fi
# x264 needs even dimensions.
W=$(( W - W % 2 )); H=$(( H - H % 2 ))
[[ -n "$W" && -n "$H" && "$W" != 0 ]] || die "could not determine capture size"

banner "recording ${W}x${H}+${X},${Y} at ${FPS} fps"
info "output: $OUT"
[[ -n "$SECS" ]] && info "duration: ${SECS}s" || info "Ctrl-C (or q) to stop"

FF=(ffmpeg -hide_banner -loglevel warning
    -f x11grab -framerate "$FPS" -video_size "${W}x${H}"
    -i "${DISPLAY}+${X},${Y}"
    -c:v libx264 -preset veryfast -crf 22 -pix_fmt yuv420p)
[[ -n "$SECS" ]] && FF+=(-t "$SECS")
FF+=("$OUT")

"${FF[@]}"

printf '\n'
if [[ -f "$OUT" ]]; then
  info "wrote $OUT ($(du -h "$OUT" | cut -f1))"
else
  err "no output file produced"
  exit 1
fi
