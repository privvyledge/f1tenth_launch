#!/usr/bin/env bash
# 52_localize_rtabmap_offline.sh — localize a recorded bag with the RTABMap
# localizer instead of AMCL, and measure what it costs.
#
#   ./52_localize_rtabmap_offline.sh --bag <path> [--db <rtabmap.db>]
#
#   --bag <path>    bag to localize (required)
#   --db  <path>    map database to localize IN (default $MAP_ROOT/rtabmap_final_nf.db)
#   --rate          playback rate (default 1.0 — CPU numbers are only meaningful
#                   at 1.0; a slower rate measures accuracy, not feasibility)
#   --out <name>    output bag name (default loc_rtabmap_<bagname>)
#   --launch-only   bring the stack up and stop, for poking at it by hand
#
# WHY THIS EXISTS
# ---------------
# 51_localize_offline.sh answers the same question with nav2_amcl and scored
# 65 mm mean / 144 mm p95 on the control bag (MAP_FRAME_DELIVERY.md). The
# RTABMap localizer was disabled on an ASSUMPTION that it is too expensive for
# the Orin Nano and had never been measured. This script measures it: the same
# bag, the same ground truth, the same scoring script, plus a CPU sample of the
# rtabmap process itself.
#
# TWO THINGS THAT MAKE IT DIFFERENT FROM THE AMCL PASS
# ----------------------------------------------------
# 1. It needs the CAMERA. RTABMap relocalizes by matching visual BoW features
#    against the map's keyframes, so colour + aligned depth + camera_info have
#    to be replayed as well as the scan. That is ~24 GB of the bag rather than
#    the ~30 MB AMCL needed, and it is why this pass is disk-heavy.
# 2. It needs NO SEED. Relocalization is global by construction, so there is no
#    initialpose to publish and no equivalent of the ~28 s pull-in AMCL pays
#    when seeded badly. That is a genuine advantage, and it is also why the
#    first seconds of the run may have NO map->odom at all.
#
# THE DATABASE IS COPIED, NEVER OPENED IN PLACE
# ---------------------------------------------
# RTABMap opens its database read-write and writes on shutdown even with
# Mem/IncrementalMemory false. maps/20260805/*.db is the provenance of the
# frozen v1 deliverable (MAP_FRAME_DELIVERY.md), so this script works on a copy
# under $SSD_ROOT/work and leaves the originals untouched.

set -uo pipefail
cd "$(dirname "$0")"
# shellcheck source=00_env.sh
source ./00_env.sh

BAG=""
DB=""
RATE=1.0
OUT=""
LAUNCH_ONLY=0

while (( $# )); do
  case "$1" in
    --bag)         BAG="$2"; shift ;;
    --db)          DB="$2"; shift ;;
    --rate)        RATE="$2"; shift ;;
    --out)         OUT="$2"; shift ;;
    --launch-only) LAUNCH_ONLY=1 ;;
    -h|--help)     sed -n '2,40p' "$0"; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
  shift
done

[[ -n "$BAG" ]] || die "no bag. Pass --bag <path>"
[[ -e "$BAG" ]] || die "bag not found: $BAG"
[[ -z "$DB" ]] && DB="${MAP_ROOT}/rtabmap_final_nf.db"
[[ -f "$DB" ]] || die "map database not found: $DB"

BAGNAME="$(basename "${BAG%/}")"
[[ -z "$OUT" ]] && OUT="loc_rtabmap_${BAGNAME}"
OUT_DIR="${BAG_ROOT}/${OUT}"
WORK_DIR="${SSD_ROOT}/work/rtabmap_loc"
WORK_DB="${WORK_DIR}/$(basename "$DB" .db)_${OUT}.db"
CPU_LOG="${OUT_DIR}.cpu.txt"

print_env
ensure_dirs
mkdir -p "$WORK_DIR"

banner "offline RTABMap localization"
printf '  %-16s %s\n' \
  "bag"       "$BAG" \
  "map db"    "$DB" \
  "work db"   "$WORK_DB" \
  "rate"      "$RATE" \
  "output"    "$OUT_DIR"

[[ -e "$OUT_DIR" ]] && die "output bag already exists: $OUT_DIR"

info "copying the map database (the original is never opened)"
cp -f "$DB" "$WORK_DB" || die "could not copy $DB"

PLAY_TOPICS=(
  "$(ns_topic tf)"
  "$(ns_topic tf_static)"
  "$(ns_topic odometry/local)"
  "$(ns_topic lidar/scan_filtered)"
  "$(ns_topic camera/color/image_raw)"
  "$(ns_topic camera/color/camera_info)"
  "$(ns_topic camera/aligned_depth_to_color/image_raw)"
  "$(ns_topic camera/aligned_depth_to_color/camera_info)"
  "$(ns_topic camera/imu/filtered)"
)

# localization_pose is what ekf_map.yaml's pose1 expects. Record both the
# namespaced-node name and the sub-namespaced one; whichever exists is what
# lands in the bag.
RECORD_TOPICS=(
  "$(ns_topic tf)"
  "$(ns_topic tf_static)"
  "$(ns_topic odometry/local)"
  "$(ns_topic localization_pose)"
  "$(ns_topic rtabmap/localization_pose)"
)

LAUNCH_PID=; PLAY_PID=; REC_PID=; CPU_PID=

cleanup() {
  local p
  for p in "$CPU_PID" "$REC_PID" "$PLAY_PID"; do
    [[ -n "$p" ]] && kill -INT "$p" 2>/dev/null
  done
  [[ -n "$REC_PID" ]] && wait "$REC_PID" 2>/dev/null
  if [[ -n "$LAUNCH_PID" ]]; then
    kill -INT "$LAUNCH_PID" 2>/dev/null
    local i
    # RTABMap defers SIGINT until it has flushed; be patient but bounded.
    for ((i = 0; i < 180; i++)); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 1; done
    kill -9 "$LAUNCH_PID" 2>/dev/null
    wait "$LAUNCH_PID" 2>/dev/null
  fi
  # Same orphan-reaping lesson as 40_build_map_offline.sh: nodes outlive a hard
  # kill of the launch parent and contaminate the next pass.
  for p in rtabmap_slam/rtabmap rtabmap_sync/rgbd_sync rtabmap_odom nav2_map_server/map_server; do
    pkill -9 -f "$p" 2>/dev/null
  done
  sleep 2
}
trap cleanup EXIT

# ------------------------------------------------------------- the stack ----
banner "launching the RTABMap localizer (no sensors, no vehicle)"
# launch_rtabmap_localizer:=True routes localization.launch.py into
# mapping/3d_mapping.launch.py with localization:=True and
# Mem/IncrementalMemory false. map_tf_publisher:=rtabmap gives it the map->odom
# edge; odom_tf_publisher:=bag leaves odom->base_link to the replayed bag.
ros2 launch f1tenth_launch localization.launch.py \
    use_namespace:=True \
    namespace:="$NS" \
    use_sim_time:=True \
    use_composition:=False \
    use_gpu:=False \
    launch_map_server:=False \
    launch_amcl:=False \
    launch_sensor_fusion:=False \
    launch_ekf_odom:=False \
    launch_ekf_map:=False \
    launch_slam_toolbox_localizer:=False \
    launch_rtabmap_localizer:=True \
    launch_particle_filter:=False \
    launch_pointcloud_odometry:=False \
    launch_rgbd_odometry:=False \
    launch_stereo_odometry:=False \
    launch_laserscan_odometry:=False \
    launch_icp_odometry:=False \
    rtabmap_database_file:="$WORK_DB" \
    map_tf_publisher:=rtabmap \
    odom_tf_publisher:=bag \
    autostart:=True \
    log_level:=warn &
LAUNCH_PID=$!
info "launch pid $LAUNCH_PID"

# The rtabmap node loads EVERY map node into working memory at startup
# (Mem/InitWMWithAllNodes true). On rtabmap_final.db that is 399 nodes and a
# 43k-word SIFT vocabulary, and it is nowhere near instant: playing the bag
# before the load finishes produces a stream of
#   Rejected loop closure N -> M: fromWords = 0 toWords = 0
# — the localizer looks like it is failing to relocalize when it is simply not
# up yet. RTAB_LOAD_WAIT is the grace period; raise it for a bigger database.
RTAB_PID=""
for ((i = 0; i < 30; i++)); do
  RTAB_PID="$(pgrep -f 'rtabmap_slam/rtabmap' | head -1)"
  [[ -n "$RTAB_PID" ]] && break
  sleep 2
done
[[ -n "$RTAB_PID" ]] || die "rtabmap node never started — check the launch output"
info "rtabmap pid $RTAB_PID"

if (( LAUNCH_ONLY )); then
  info "stack up. Play the bag yourself; Ctrl-C here when done."
  wait "$LAUNCH_PID"; exit 0
fi

# --------------------------------------------------------------- capture ----
banner "recording localizer output -> $OUT_DIR"
ros2 bag record -o "$OUT_DIR" -s "$BAG_STORAGE" --use-sim-time "${RECORD_TOPICS[@]}" &
REC_PID=$!
sleep 3

# ------------------------------------------------------------------- cpu ----
# utime+stime out of /proc/<pid>/stat, in clock ticks (100/s), sampled every
# 5 s. Reported as percent of ONE core; the Orin Nano has 6, so 600% is the
# whole machine. This is the number the "too slow for the Jetson" assumption
# was made about, and it has never been taken.
banner "sampling rtabmap CPU every 5 s -> $CPU_LOG"
(
  prev="$(awk '{print $14+$15}' "/proc/$RTAB_PID/stat" 2>/dev/null)"
  printf '# elapsed_s cpu_pct_of_one_core rss_mb\n' > "$CPU_LOG"
  t=0
  while sleep 5; do
    cur="$(awk '{print $14+$15}' "/proc/$RTAB_PID/stat" 2>/dev/null)" || break
    [[ -z "$cur" ]] && break
    rss="$(awk '{print $24*4/1024}' "/proc/$RTAB_PID/stat" 2>/dev/null)"
    t=$((t + 5))
    awk -v t="$t" -v d="$((cur - prev))" -v r="$rss" \
        'BEGIN{printf "%d %.1f %.0f\n", t, d/5.0, r}' >> "$CPU_LOG"
    prev="$cur"
  done
) &
CPU_PID=$!

# ------------------------------------------------------------ load wait ----
# Sampled RSS/CPU is already running, so the log shows the load itself: the
# node pins a core while it pulls the graph in and then goes quiet.
banner "waiting ${RTAB_LOAD_WAIT:-90} s for the map database to load into working memory"
sleep "${RTAB_LOAD_WAIT:-90}"
if ! kill -0 "$RTAB_PID" 2>/dev/null; then
  err "rtabmap died while loading the database (peak RSS $(awk 'NR>1 && $3>m {m=$3} END {print m+0}' "$CPU_LOG") MB)."
  die "on a 7 GB Orin Nano this is the OOM killer; see the notes in this script's header"
fi
info "rtabmap still up after the load (RSS $(awk 'END {print $3}' "$CPU_LOG") MB)"

# --------------------------------------------------------------- replay ----
banner "playing ${#PLAY_TOPICS[@]} topics at ${RATE}x (camera included — this is the heavy one)"
ros2 bag play "$BAG" --clock 100 --rate "$RATE" --topics "${PLAY_TOPICS[@]}" &
PLAY_PID=$!

wait "$PLAY_PID"
PLAY_PID=
banner "playback finished"
sleep 5

kill -INT "$CPU_PID" 2>/dev/null; CPU_PID=
kill -INT "$REC_PID" 2>/dev/null
wait "$REC_PID" 2>/dev/null
REC_PID=

if [[ ! -s "$OUT_DIR/metadata.yaml" ]]; then
  err "NO BAG WRITTEN to $OUT_DIR — nothing was captured."
  exit 1
fi
info "localization bag: $OUT_DIR ($(du -sh "$OUT_DIR" 2>/dev/null | cut -f1))"

banner "rtabmap CPU (percent of ONE core; 6 cores = 600%)"
awk 'NR>1 {n++; s+=$2; if ($2>m) m=$2; if ($3>r) r=$3}
     END {if (n) printf "  mean %.1f%%   peak %.1f%%   peak RSS %.0f MB   (%d samples)\n", s/n, m, r, n;
          else print "  no samples"}' "$CPU_LOG"

banner "next"
cat <<EOF
  Score it against the same ground truth AMCL was scored against:
      python3 ../analysis/check_map_frame.py "$OUT_DIR" \\
          --map "${MAP_ROOT}/rtabmap_2d_final.yaml" \\
          --truth "${MAP_ROOT}/truth_mapping_drive_170025.csv"

  AMCL's number on this control: mean 64.7 mm, p95 143.5 mm, 87.3% inside 126 mm.
EOF
