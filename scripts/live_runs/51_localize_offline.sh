#!/usr/bin/env bash
# 51_localize_offline.sh — localize a recorded bag against an existing map.
# No robot required; this is a bag replay plus a localizer.
#
#   ./51_localize_offline.sh --bag <path> [--map <map.yaml>] [--publisher amcl|ekf]
#
#   --bag <path>       bag to localize (required)
#   --map <yaml>       occupancy grid to localize against
#                      (default: $MAP_ROOT/rtabmap_2d_final.yaml)
#   --publisher        who broadcasts map->odom: amcl (default) or ekf
#   --map-frequency    ekf_map update/publish rate, Hz (default 30.0; only
#                      meaningful with --publisher ekf). localization.launch.py
#                      defaults this to 10.0, which is BELOW the 30 Hz
#                      odometry/local it is meant to smooth — at 10 Hz the
#                      filter adds its own staircase instead of removing AMCL's.
#   --init-x/-y/-yaw   seed pose of base_link IN THE MAP FRAME (see below)
#   --rate             playback rate (default 1.0 — only a handful of topics
#                      are replayed, so real time is comfortable)
#   --out <name>       output bag name (default: loc_<bagname>)
#   --keep-vslam       also replay the VSLAM odometry topics (ekf_map inputs)
#   --no-vslam         with --publisher ekf, do NOT replay them, leaving ekf_map
#                      on amcl_pose + vesc_odom only. Isolates whether VSLAM's
#                      differential odom2 is what makes the fused map->odom jump.
#
# WHY THIS EXISTS
# ---------------
# A map does not publish map->odom; a localizer does. The three drive bags
# already carry odom->base_link at 30 Hz from the live EKF, so all a localizer
# has to supply is the missing map->odom edge. Composing the two puts every run
# in one shared map frame, which is the actual deliverable (see
# MAP_BUILD_HANDOFF.md §2).
#
# THE SEED IS NOT A GUESS — AND IT IS NOT THE ORIGIN
# --------------------------------------------------
# The operator hand-placed the car on the same tile-aligned physical start pose
# for all three runs, so ONE map-frame seed serves all three:
#
#     x = +0.445  y = -0.575  yaw = -1.3931 rad  (-79.82 deg)
#
# This corrects the earlier assumption that map == odom at t=0 for
# mapping_drive_170025 and that the seed was therefore ~(0, 0, -54.5 deg).
# RTABMap built that map with the graph optimized FROM THE END, so map and odom
# coincide at the LAST keyframe, not the first. Its own optimized graph
# (rtabmap_ground_truth.py) puts the true map->odom at t=0 at
# (+0.4545, -0.5746, -25.33 deg), decaying to exactly zero at the end of the
# run. Seeding at the origin costs 0.73 m and 25 deg, and it is not free:
# measured on the control bag, AMCL needed ~28 s of driving to pull itself in,
# and every pose in that window is outside LUCIO's 126 mm bar.
#
# The per-run map->odom rotations in the older handoff (-30.96, -29.51 deg)
# inherit the same error; the corrected values are -56.3 and -54.8 deg. They
# are outputs, not inputs — seed the pose above and let AMCL produce them.
#
# CONTROL RUN — DO THIS FIRST
# ---------------------------
#   ./51_localize_offline.sh --bag .../mapping_drive_170025
#   python3 ../analysis/rtabmap_ground_truth.py .../rtabmap_final.db --out truth.csv
#   python3 ../analysis/check_map_frame.py <out> --truth truth.csv
# Do NOT score that run against identity: RTABMap optimizes the graph, so the
# true map->odom is a real transform reaching 0.76 m and 25 deg, and charging
# the localizer for it measures the wrong thing. Score against the CSV.

set -uo pipefail
cd "$(dirname "$0")"
# Local-only run: nothing off this machine needs the topics, so use the
# loopback-only CycloneDDS profile (no peer-unreachable spam). Override with
# DDS_PROFILE=static if you want to watch this from a remote RViz.
export DDS_PROFILE="${DDS_PROFILE:-lo}"
# shellcheck source=00_env.sh
source ./00_env.sh

BAG=""
MAP=""
PUBLISHER=amcl
MAP_FREQUENCY=30.0
RATE=1.0
OUT=""
# The shared physical start pose, in the map frame. See the header.
INIT_X=0.4451
INIT_Y=-0.5750
# NOTE (2026-08-24): this -1.3931 rad (-79.82 deg) came from RTABMap's optimized
# poses at t=0 and is off against RTABMap's own grid -- the true 08-05 park
# heading is at least -85.0 deg. Left as-is on purpose: a seed costs
# convergence TIME, not correctness, and changing it makes future replays
# non-comparable with the archived ones. See LUCIO_MAP_HEADING_ANSWER.md.
INIT_YAW=-1.3931
KEEP_VSLAM=0
NO_VSLAM=0
LAUNCH_ONLY=0
AMCL_PARAMS_OVERRIDE=""

while (( $# )); do
  case "$1" in
    --bag)        BAG="$2"; shift ;;
    --map)        MAP="$2"; shift ;;
    --publisher)  PUBLISHER="$2"; shift ;;
    --map-frequency) MAP_FREQUENCY="$2"; shift ;;
    --rate)       RATE="$2"; shift ;;
    --out)        OUT="$2"; shift ;;
    --init-x)     INIT_X="$2"; shift ;;
    --init-y)     INIT_Y="$2"; shift ;;
    --init-yaw)   INIT_YAW="$2"; shift ;;
    --params)     AMCL_PARAMS_OVERRIDE="$2"; shift ;;
    --keep-vslam) KEEP_VSLAM=1 ;;
    --no-vslam)   NO_VSLAM=1 ;;
    --launch-only) LAUNCH_ONLY=1 ;;
    -h|--help)    sed -n '2,50p' "$0"; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
  shift
done

[[ -n "$BAG" ]] || die "no bag. Pass --bag <path>"
[[ -e "$BAG" ]] || die "bag not found: $BAG"
[[ -z "$MAP" ]] && MAP="${MAP_ROOT}/rtabmap_2d_final.yaml"
[[ -f "$MAP" ]] || die "map not found: $MAP"

case "$PUBLISHER" in
  amcl|ekf) ;;
  *) die "--publisher must be amcl or ekf (got '$PUBLISHER')" ;;
esac

BAGNAME="$(basename "${BAG%/}")"
[[ -z "$OUT" ]] && OUT="loc_${BAGNAME}"
OUT_DIR="${BAG_ROOT}/${OUT}"

if [[ -n "$AMCL_PARAMS_OVERRIDE" ]]; then
  AMCL_PARAMS="$AMCL_PARAMS_OVERRIDE"
  [[ -f "$AMCL_PARAMS" ]] || die "params file not found: $AMCL_PARAMS"
else
  AMCL_PARAMS="${F1TENTH_WS}/src/f1tenth_launch/config/localization/localizer_amcl.yaml"
  [[ -f "$AMCL_PARAMS" ]] || \
    AMCL_PARAMS="$(ros2 pkg prefix f1tenth_launch 2>/dev/null)/share/f1tenth_launch/config/localization/localizer_amcl.yaml"
fi

print_env
ensure_dirs

banner "offline localization"
printf '  %-16s %s\n' \
  "bag"        "$BAG" \
  "map"        "$MAP" \
  "publisher"  "$PUBLISHER" \
  "map freq"   "$([[ "$PUBLISHER" == ekf ]] && echo "$MAP_FREQUENCY Hz" || echo 'n/a (amcl)')" \
  "rate"       "$RATE" \
  "seed x,y"   "$INIT_X, $INIT_Y" \
  "seed yaw"   "$INIT_YAW rad" \
  "amcl params" "$AMCL_PARAMS" \
  "output"     "$OUT_DIR"

[[ -e "$OUT_DIR" ]] && die "output bag already exists: $OUT_DIR"

# Replay only what a localizer consumes. The full bag is ~25 GB of camera data
# that nothing here subscribes to; replaying it makes the run disk-bound for no
# benefit and forces a low --rate. odometry/local is carried through so the
# derived-bag step can read source and localizer output from one place.
PLAY_TOPICS=(
  "$(ns_topic tf)"
  "$(ns_topic tf_static)"
  "$(ns_topic lidar/scan_filtered)"
  "$(ns_topic odometry/local)"
  "$(ns_topic vehicle/vesc_odom)"
)
if (( KEEP_VSLAM )) || { [[ "$PUBLISHER" == ekf ]] && (( ! NO_VSLAM )); }; then
  # ekf_map takes odom1/odom2 from VSLAM. Note the caveat in MAP_BUILD_HANDOFF:
  # slam_odometry is NOT in the map frame despite claiming frame_id odom, and
  # ekf_map.yaml treats it as an absolute global anchor.
  PLAY_TOPICS+=(
    "$(ns_topic visual_slam/tracking/odometry)"
    "$(ns_topic visual_slam/vis/slam_odometry)"
  )
fi

RECORD_TOPICS=(
  "$(ns_topic tf)"
  "$(ns_topic tf_static)"
  "$(ns_topic amcl_pose)"
  "$(ns_topic odometry/local)"
)
[[ "$PUBLISHER" == ekf ]] && RECORD_TOPICS+=("$(ns_topic odometry/global)")

LAUNCH_PID=; PLAY_PID=; REC_PID=

cleanup() {
  local p
  for p in "$REC_PID" "$PLAY_PID"; do
    [[ -n "$p" ]] && kill -INT "$p" 2>/dev/null
  done
  [[ -n "$REC_PID" ]] && wait "$REC_PID" 2>/dev/null
  if [[ -n "$LAUNCH_PID" ]]; then
    kill -INT "$LAUNCH_PID" 2>/dev/null
    local i
    for ((i = 0; i < 60; i++)); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 1; done
    kill -9 "$LAUNCH_PID" 2>/dev/null
    wait "$LAUNCH_PID" 2>/dev/null
  fi
  # Orphan reaping, same lesson as 40_build_map_offline.sh: nodes survive a hard
  # kill of the launch parent and contaminate the next pass.
  for p in nav2_amcl/amcl nav2_map_server/map_server \
           nav2_lifecycle_manager/lifecycle_manager robot_localization/ekf_node; do
    pkill -9 -f "$p" 2>/dev/null
  done
  sleep 2
}
trap cleanup EXIT

# ------------------------------------------------------------- the stack ----
banner "launching localizer (no sensors, no vehicle)"
# params_file MUST be localizer_amcl.yaml. This is the launch-config
# inheritance trap documented in CLAUDE.md: with nav2_params.yaml (which has no
# amcl: section) AMCL silently runs on library defaults, subscribes to /scan,
# and never localizes while looking perfectly healthy.
#
# use_gpu:=False keeps Isaac VSLAM out — its odometry comes from the bag.
# Every odometry source here is off: the bag already carries odom->base_link.
ros2 launch f1tenth_launch localization.launch.py \
    use_namespace:=True \
    namespace:="$NS" \
    use_sim_time:=True \
    use_composition:=False \
    use_gpu:=False \
    params_file:="$AMCL_PARAMS" \
    map_file:="$MAP" \
    launch_map_server:=True \
    launch_amcl:=True \
    launch_sensor_fusion:="$([[ "$PUBLISHER" == ekf ]] && echo True || echo False)" \
    launch_ekf_odom:=False \
    launch_ekf_map:="$([[ "$PUBLISHER" == ekf ]] && echo True || echo False)" \
    launch_slam_toolbox_localizer:=False \
    launch_rtabmap_localizer:=False \
    launch_particle_filter:=False \
    launch_pointcloud_odometry:=False \
    launch_rgbd_odometry:=False \
    launch_stereo_odometry:=False \
    launch_laserscan_odometry:=False \
    launch_icp_odometry:=False \
    map_tf_publisher:="$PUBLISHER" \
    map_frequency:="$MAP_FREQUENCY" \
    odom_tf_publisher:=bag \
    autostart:=True \
    log_level:=warn &
LAUNCH_PID=$!
info "launch pid $LAUNCH_PID"

sleep 12
wait_for_topic "$(ns_topic map)" 90 || die "map_server never published $(ns_topic map)"

if (( LAUNCH_ONLY )); then
  info "stack up. Play the bag yourself; Ctrl-C here when done."
  wait "$LAUNCH_PID"; exit 0
fi

# --------------------------------------------------------------- capture ----
banner "recording localizer output -> $OUT_DIR"
# --use-sim-time is a first-class flag here; `--ros-args -p use_sim_time:=true`
# is swallowed by -p/--polling-interval and the recorder dies on startup.
ros2 bag record -o "$OUT_DIR" -s "$BAG_STORAGE" --use-sim-time "${RECORD_TOPICS[@]}" &
REC_PID=$!
sleep 3

# --------------------------------------------------------------- replay ----
banner "playing ${#PLAY_TOPICS[@]} topics at ${RATE}x"
ros2 bag play "$BAG" --clock 100 --rate "$RATE" --topics "${PLAY_TOPICS[@]}" &
PLAY_PID=$!

# The seed cannot be published with `ros2 topic pub`: it stamps with wall time,
# which AMCL under use_sim_time rejects as a future extrapolation, silently
# leaving it on the (0,0,0) YAML default. seed_initialpose.py runs as a real
# sim-time node, waits for the replayed odom->base_link, and stamps with 0.
banner "seeding AMCL at ($INIT_X, $INIT_Y, $INIT_YAW rad)"
# Hard failure on purpose. An unseeded AMCL does not error — it localizes from
# (0,0,0) and produces a complete, plausible, wrong bag.
python3 ./seed_initialpose.py --ns "$NS" \
    --x "$INIT_X" --y "$INIT_Y" --yaw "$INIT_YAW" \
    || die "seeding failed — aborting rather than recording an unseeded run"

wait "$PLAY_PID"
PLAY_PID=
banner "playback finished"
sleep 3

kill -INT "$REC_PID" 2>/dev/null
wait "$REC_PID" 2>/dev/null
REC_PID=

if [[ ! -s "$OUT_DIR/metadata.yaml" ]]; then
  err "NO BAG WRITTEN to $OUT_DIR — nothing was captured."
  exit 1
fi
info "localization bag: $OUT_DIR ($(du -sh "$OUT_DIR" 2>/dev/null | cut -f1))"

banner "next"
cat <<EOF
  Score it (the control bag is the one that matters first):
      python3 ../analysis/check_map_frame.py "$OUT_DIR" --map "$MAP"

  Then build the derived bag for the LUCIO consumer:
      python3 ../analysis/make_map_frame_bag.py --source "$BAG" \\
          --localization "$OUT_DIR" --out ${BAG_ROOT}/mapframe_${BAGNAME}
EOF
