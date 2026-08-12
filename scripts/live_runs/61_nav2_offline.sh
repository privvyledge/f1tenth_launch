#!/usr/bin/env bash
# 61_nav2_offline.sh — exercise the whole Nav2 stack from a recorded bag.
# No robot required, and nothing can reach the actuation chain.
#
#   ./61_nav2_offline.sh --bag <path> [--map <map.yaml>] [--publisher amcl|ekf]
#
#   --bag <path>       drive bag supplying tf, scan and odometry (required)
#   --map <yaml>       occupancy grid (default: $MAP_ROOT/rtabmap_2d_final.yaml)
#   --publisher        who broadcasts map->odom: amcl (default) or ekf. Only
#                      the amcl path has been run; ekf mirrors what
#                      51_localize_offline.sh does and is unexercised here.
#   --map-frequency    ekf_map rate, Hz (default 30.0; only with --publisher ekf)
#   --goals-bag        a bag of map-frame poses to draw goals from (default:
#                      $BAG_ROOT/loc_ekflocal_<bagname>, an earlier localized
#                      run of the same drive). Goals must be places the car
#                      actually went — see goal_poses_from_bag.py.
#   --goals-csv        skip the extraction and use this stamp,x,y,yaw CSV
#   --goal-rows        comma-separated CSV row indices (default: all of a
#                      generated goals.csv)
#   --goal-timeout     seconds to watch each goal (default 45)
#   --rate             playback rate (default 1.0)
#   --out <name>       output bag name (default: nav2_<bagname>)
#
# WHY THIS EXISTS
# ---------------
# Nav2 has never been given a goal on this vehicle. A bench bringup on
# 2026-08-06 proved only that the eight servers come up. Everything downstream
# of that — planning, control, the behavior tree, the recoveries — was still
# untested, and the car was not available to test it on.
#
# A drive bag supplies every input Nav2 needs except a goal: odom->base_link at
# 30 Hz, lidar/scan_filtered for the local costmap, and odometry/local for the
# controller and the BT. Add map_server plus a localizer for map->odom and the
# planning stack runs exactly as it would on the car.
#
# WHAT THIS CAN AND CANNOT PROVE
# ------------------------------
# The bag drives the pose, so the loop is OPEN. The controller steers toward
# the path from wherever the bag has put the car, and no goal is ever reached.
# That is not a failure and it is not what is being measured. This test answers:
#
#   - does bt_navigator accept a goal at all
#   - does planner_server return a path through free space, inside its 5 s
#     max_planning_time
#   - does controller_server emit continuous, bounded velocity
#   - do the recovery behaviours fire, and only when they should
#   - does anything crash, hang the lifecycle manager, or peg a core
#
# Closed-loop goal reaching needs the vehicle and stays deferred.
#
# ISOLATE THE DOMAIN BEFORE RUNNING THIS
# --------------------------------------
# Everyone here defaults to ROS_DOMAIN_ID=0 and the offline scripts use 42. On
# 2026-08-06 a live bench stack and another agent's sim-time replay shared a
# domain in one container and contaminated each other silently. Pick an unused
# domain and keep DDS on loopback:
#
#     export ROS_DOMAIN_ID=51
#     export CYCLONEDDS_URI=file:///mnt/shared_dir/cyclonedds_offline_lo.xml
#
# and check for other stacks FROM THE HOST — a container's ps only sees its own
# PID namespace and will tell you it is alone:
#
#     ps -eo pid,etime,cmd | grep "[r]os2 launch"

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
GOALS_BAG=""
GOALS_CSV=""
GOAL_ROWS=""
GOAL_COUNT=3
GOAL_TIMEOUT=45
RATE=1.0
OUT=""
# The shared physical start pose in the map frame — see 51_localize_offline.sh,
# which derives it and explains why it is not the origin.
INIT_X=0.4451
INIT_Y=-0.5750
INIT_YAW=-1.3931

while (( $# )); do
  case "$1" in
    --bag)           BAG="$2"; shift ;;
    --map)           MAP="$2"; shift ;;
    --publisher)     PUBLISHER="$2"; shift ;;
    --map-frequency) MAP_FREQUENCY="$2"; shift ;;
    --goals-bag)     GOALS_BAG="$2"; shift ;;
    --goals-csv)     GOALS_CSV="$2"; shift ;;
    --goal-rows)     GOAL_ROWS="$2"; shift ;;
    --goal-count)    GOAL_COUNT="$2"; shift ;;
    --goal-timeout)  GOAL_TIMEOUT="$2"; shift ;;
    --rate)          RATE="$2"; shift ;;
    --out)           OUT="$2"; shift ;;
    --init-x)        INIT_X="$2"; shift ;;
    --init-y)        INIT_Y="$2"; shift ;;
    --init-yaw)      INIT_YAW="$2"; shift ;;
    -h|--help)       sed -n '2,20p' "$0"; exit 0 ;;
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
[[ -z "$OUT" ]] && OUT="nav2_${BAGNAME}"
OUT_DIR="${BAG_ROOT}/${OUT}"
if [[ -z "$GOALS_CSV" ]]; then
  [[ -z "$GOALS_BAG" ]] && GOALS_BAG="${BAG_ROOT}/loc_ekflocal_${BAGNAME}"
  [[ -e "$GOALS_BAG" ]] || die "no goal source. Pass --goals-bag (a localized
       run of this drive) or --goals-csv. Goals are drawn from poses the car
       actually occupied so a failed plan can never be blamed on a goal inside
       a wall. NOT from maps/*/truth_*.csv — that is the map->odom transform,
       not a pose."
fi

# AMCL must get its own params file. This is the launch-config inheritance trap
# in CLAUDE.md: nav2_params.yaml has no amcl: section, so AMCL would silently
# run on library defaults, subscribe to /scan, and never localize.
AMCL_PARAMS="${F1TENTH_WS}/src/f1tenth_launch/config/localization/localizer_amcl.yaml"
[[ -f "$AMCL_PARAMS" ]] || \
  AMCL_PARAMS="$(ros2 pkg prefix f1tenth_launch 2>/dev/null)/share/f1tenth_launch/config/localization/localizer_amcl.yaml"
[[ -f "$AMCL_PARAMS" ]] || die "localizer_amcl.yaml not found"

NAV2_PARAMS="${F1TENTH_WS}/src/f1tenth_launch/config/nav2_params.yaml"
[[ -f "$NAV2_PARAMS" ]] || \
  NAV2_PARAMS="$(ros2 pkg prefix f1tenth_launch 2>/dev/null)/share/f1tenth_launch/config/nav2_params.yaml"
[[ -f "$NAV2_PARAMS" ]] || die "nav2_params.yaml not found"

# The velocity smoother's output. Never cmd_vel here: twist_to_ackermann is not
# running in this test, but naming the live topic in an offline run is how an
# offline run becomes a live one by accident.
CMD_VEL_TOPIC=cmd_vel_nav2

RUN_DIR="${SSD_ROOT}/run/nav2_offline_$(date +%Y%m%d_%H%M%S)"

print_env
ensure_dirs
mkdir -p "$RUN_DIR"

# Goals, if not supplied, come out of an earlier localized run of this same
# drive: real poses, real free space, far enough apart to require planning.
if [[ -z "$GOALS_CSV" ]]; then
  GOALS_CSV="${RUN_DIR}/goals.csv"
  banner "picking $GOAL_COUNT goals from $GOALS_BAG"
  python3 ./goal_poses_from_bag.py "$GOALS_BAG" --ns "$NS" \
      --pick "$GOAL_COUNT" --out-csv "$GOALS_CSV" \
      || die "could not pick goals from $GOALS_BAG"
fi
[[ -s "$GOALS_CSV" ]] || die "empty goal list: $GOALS_CSV"

banner "offline Nav2 test"
printf '  %-16s %s\n' \
  "bag"        "$BAG" \
  "map"        "$MAP" \
  "publisher"  "$PUBLISHER" \
  "goals"      "$GOALS_CSV${GOAL_ROWS:+ rows $GOAL_ROWS}" \
  "cmd_vel"    "$(ns_topic "$CMD_VEL_TOPIC") (nothing reaches the vehicle)" \
  "nav2 params" "$NAV2_PARAMS" \
  "run dir"    "$RUN_DIR" \
  "output bag" "$OUT_DIR"

[[ -e "$OUT_DIR" ]] && die "output bag already exists: $OUT_DIR"

# Another stack on this domain invalidates everything below, and the check has
# to look outside this container's PID namespace. Best effort: if /proc is the
# host's (privileged container) this sees everything; if not, it sees what it
# can and the operator has been told to check from the host.
OTHER="$(ps -eo pid,cmd 2>/dev/null | grep '[r]os2 launch' | grep -v "$$" || true)"
if [[ -n "$OTHER" ]]; then
  warn "other ros2 launch processes visible from here:"
  printf '  %s\n' "$OTHER"
  warn "if any of them is on ROS_DOMAIN_ID=$ROS_DOMAIN_ID this run is contaminated"
fi

PLAY_TOPICS=(
  "$(ns_topic tf)"
  "$(ns_topic tf_static)"
  "$(ns_topic lidar/scan_filtered)"
  "$(ns_topic odometry/local)"
  "$(ns_topic vehicle/vesc_odom)"
)
if [[ "$PUBLISHER" == ekf ]]; then
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
  "$(ns_topic plan)"
  "$(ns_topic local_plan)"
  "$(ns_topic cmd_vel_nav)"
  "$(ns_topic "$CMD_VEL_TOPIC")"
  # cmd_vel is recorded even though this run diverts the smoother away from it:
  # nav2_behaviors publishes recovery velocity on an UN-REMAPPED cmd_vel
  # (timed_behavior.hpp creates the publisher on the relative name, and
  # upstream navigation_launch.py does not remap it either). So a spin or a
  # backup shows up here and nowhere else, and on the car it would reach
  # twist_to_ackermann. Recording it is how that gets proved rather than argued.
  "$(ns_topic cmd_vel)"
  "$(ns_topic global_costmap/costmap)"
  "$(ns_topic local_costmap/costmap)"
)
[[ "$PUBLISHER" == ekf ]] && RECORD_TOPICS+=("$(ns_topic odometry/global)")

LOC_PID=; NAV_PID=; PLAY_PID=; REC_PID=; CPU_PID=

cleanup() {
  local p i
  for p in "$CPU_PID" "$REC_PID" "$PLAY_PID"; do
    [[ -n "$p" ]] && kill -INT "$p" 2>/dev/null
  done
  [[ -n "$REC_PID" ]] && wait "$REC_PID" 2>/dev/null
  for p in "$NAV_PID" "$LOC_PID"; do
    [[ -n "$p" ]] || continue
    kill -INT "$p" 2>/dev/null
    for ((i = 0; i < 45; i++)); do kill -0 "$p" 2>/dev/null || break; sleep 1; done
    kill -9 "$p" 2>/dev/null
    wait "$p" 2>/dev/null
  done
  # Orphan reaping. `kill -9` on a ros2 launch parent leaves every child
  # publishing, which is indistinguishable from a launch file duplicating
  # subsystems (bug-116). Note two nodes evade the obvious greps: the EKFs run
  # as robot_localization/ekf_node, not as their node names.
  for p in nav2_controller/controller_server nav2_planner/planner_server \
           nav2_smoother/smoother_server nav2_behaviors/behavior_server \
           nav2_bt_navigator/bt_navigator nav2_waypoint_follower/waypoint_follower \
           nav2_velocity_smoother/velocity_smoother \
           nav2_lifecycle_manager/lifecycle_manager \
           nav2_amcl/amcl nav2_map_server/map_server \
           robot_localization/ekf_node; do
    pkill -9 -f "$p" 2>/dev/null
  done
  sleep 2
}
trap cleanup EXIT

# -------------------------------------------------------- the localizer ----
banner "launching localizer (map_server + $PUBLISHER)"
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
    log_level:=warn > "$RUN_DIR/localization.log" 2>&1 &
LOC_PID=$!
info "localization pid $LOC_PID -> $RUN_DIR/localization.log"

sleep 12
wait_for_topic "$(ns_topic map)" 90 || die "map_server never published $(ns_topic map)"

# -------------------------------------------------------------- nav2 ------
banner "launching nav2 servers"
# params_file is passed explicitly for the same inheritance reason as above,
# in the other direction: nav2 needs nav2_params.yaml, not localizer_amcl.yaml.
ros2 launch f1tenth_launch nav2_navigation.launch.py \
    namespace:="$NS" \
    use_sim_time:=True \
    use_composition:=False \
    autostart:=True \
    use_respawn:=False \
    params_file:="$NAV2_PARAMS" \
    cmd_vel_topic:="$CMD_VEL_TOPIC" \
    log_level:=info > "$RUN_DIR/nav2.log" 2>&1 &
NAV_PID=$!
info "nav2 pid $NAV_PID -> $RUN_DIR/nav2.log"

# The costmaps are the real readiness signal: they only publish once their
# lifecycle node has activated, which is what a hung lifecycle manager fails
# to do.
wait_for_topic "$(ns_topic global_costmap/costmap)" 120 || \
  die "global costmap never came up — check $RUN_DIR/nav2.log for a lifecycle hang"
wait_for_topic "$(ns_topic local_costmap/costmap)" 120 || \
  die "local costmap never came up — check $RUN_DIR/nav2.log"

banner "lifecycle nodes"
ros2 node list 2>/dev/null | grep -E \
  'controller_server|planner_server|smoother_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|map_server|amcl' \
  | sort | uniq -c | tee "$RUN_DIR/nodes.txt" | sed 's/^/  /'
warn "every count above must be 1; a 2 means a second stack is on this domain"

# ------------------------------------------------------------- capture ----
banner "recording -> $OUT_DIR"
ros2 bag record -o "$OUT_DIR" -s "$BAG_STORAGE" --use-sim-time "${RECORD_TOPICS[@]}" \
  > "$RUN_DIR/record.log" 2>&1 &
REC_PID=$!
sleep 3

# planner_server was once seen at 94 % CPU with no goal pending. Sample it
# throughout rather than taking one snapshot at the end.
#
# Two traps, both hit on 2026-08-06 before this settled:
#   - `ps -o pcpu` is the process's LIFETIME AVERAGE, not instantaneous load. A
#     server that worked hard during on_configure reads ~98 % for minutes
#     afterwards while doing nothing. top's second iteration is a real interval
#     measurement, so -n2 -d2 and parse only the second block.
#   - a container whose PID 1 is `sleep infinity` reaps nothing, so every
#     pkill'd node from a previous run stays as a <defunct> zombie carrying its
#     frozen name and pcpu. Three runs looked exactly like three concurrent
#     nav2 stacks. Zombies are dropped here by state; create the container with
#     `docker run --init` to not have them at all.
(
  while true; do
    top -b -n2 -d2 -w 512 2>/dev/null | awk -v t="$(date +%H:%M:%S)" '
      /^ *PID/ { block++; next }
      block == 2 && $8 != "Z" &&
        /planner_server|controller_serv|bt_navigator|behavior_serve|amcl|ekf_node/ \
        { printf "%s %s %s%%\n", t, $NF, $9 }'
    sleep 3
  done
) > "$RUN_DIR/cpu.log" 2>&1 &
CPU_PID=$!

# -------------------------------------------------------------- replay ----
banner "playing ${#PLAY_TOPICS[@]} topics at ${RATE}x"
ros2 bag play "$BAG" --clock 100 --rate "$RATE" --topics "${PLAY_TOPICS[@]}" \
  > "$RUN_DIR/play.log" 2>&1 &
PLAY_PID=$!

banner "seeding AMCL at ($INIT_X, $INIT_Y, $INIT_YAW rad)"
# Hard failure on purpose: an unseeded AMCL does not error, it localizes from
# (0,0,0) and every goal is then planned from the wrong place.
python3 ./seed_initialpose.py --ns "$NS" \
    --x "$INIT_X" --y "$INIT_Y" --yaw "$INIT_YAW" \
    || die "seeding failed — aborting rather than testing from an unknown pose"

# --------------------------------------------------------------- goals ----
banner "sending goals"
python3 ./nav2_goal_probe.py --ns "$NS" \
    --cmd-topic "$CMD_VEL_TOPIC" \
    --goals-csv "$GOALS_CSV" \
    ${GOAL_ROWS:+--goal-rows "$GOAL_ROWS"} \
    --goal-timeout "$GOAL_TIMEOUT" \
    --exercise-backup \
    --out-json "$RUN_DIR/probe.json" 2>&1 | tee "$RUN_DIR/probe.log"
PROBE_RC=${PIPESTATUS[0]}

# The bag may still be playing (goals are shorter than the run) or may have
# ended under the probe. Either is fine; stop cleanly.
if kill -0 "$PLAY_PID" 2>/dev/null; then
  kill -INT "$PLAY_PID" 2>/dev/null
fi
wait "$PLAY_PID" 2>/dev/null
PLAY_PID=
sleep 3

kill -INT "$CPU_PID" 2>/dev/null; CPU_PID=
kill -INT "$REC_PID" 2>/dev/null
wait "$REC_PID" 2>/dev/null
REC_PID=

# -------------------------------------------------------------- verdict ----
banner "results"
if [[ ! -s "$OUT_DIR/metadata.yaml" ]]; then
  err "NO BAG WRITTEN to $OUT_DIR — nothing was captured"
else
  info "bag: $OUT_DIR ($(du -sh "$OUT_DIR" 2>/dev/null | cut -f1))"
fi

printf '\n  --- crashes / aborts in the nav2 log ---\n'
grep -iE 'process has died|terminate|Exception|abort|\[ERROR\]' "$RUN_DIR/nav2.log" \
  | head -20 | sed 's/^/  /' || true
printf '\n  --- recoveries and BT events ---\n'
grep -iE 'recovery|Spin|BackUp|behavior|Failed to make progress|aborting' "$RUN_DIR/nav2.log" \
  | head -20 | sed 's/^/  /' || true
printf '\n  --- peak CPU per node ---\n'
awk '{gsub("%","",$3); if ($3+0 > peak[$2]) peak[$2] = $3+0}
     END {for (n in peak) printf "  %-20s %5.1f %%\n", n, peak[n]}' \
  "$RUN_DIR/cpu.log" 2>/dev/null | sort -k2 -rn || true

printf '\n  --- recovery velocity on the UN-DIVERTED cmd_vel ---\n'
# nav2_behaviors does not honour cmd_vel_topic. Anything here is a command that
# would have reached the vehicle on a live "dry" run with twist_to_ackermann up.
python3 - "$OUT_DIR" "$(ns_topic cmd_vel)" "${LIVE_RUNS_DIR}/../analysis" <<'PY' 2>/dev/null | sed 's/^/  /' || true
import sys
sys.path.insert(0, sys.argv[3])
from check_map_frame import read_bag
bag, topic = sys.argv[1], sys.argv[2]
n = nz = 0
vmax = wmax = 0.0
for _, msg, _ in read_bag(bag, {topic}):
    n += 1
    vmax = max(vmax, abs(msg.linear.x)); wmax = max(wmax, abs(msg.angular.z))
    if abs(msg.linear.x) > 1e-6 or abs(msg.angular.z) > 1e-6:
        nz += 1
print(f"{n} messages, {nz} non-zero, |vx|max {vmax:.3f} m/s, |wz|max {wmax:.3f} rad/s")
print("non-zero here means a recovery commanded motion outside the diverted topic"
      if nz else "nothing commanded on cmd_vel (no recovery fired, or none moved)")
PY

printf '\n'
[[ -s "$RUN_DIR/probe.json" ]] && info "probe summary: $RUN_DIR/probe.json"
info "logs: $RUN_DIR"
exit "${PROBE_RC:-0}"
