#!/usr/bin/env bash
# 60_nav2_test.sh — Objective 4: Nav2 tests.
#
#   ./60_nav2_test.sh --map <map.yaml> --dry-run   # Nav2 runs, car does NOT move
#   ./60_nav2_test.sh --map <map.yaml>             # Nav2 commands the vehicle
#
# --dry-run redirects the velocity smoother output to cmd_vel_nav2 instead of
# cmd_vel. Nav2 plans and publishes exactly as normal, and you can watch the
# whole pipeline in RViz, but nothing reaches the actuation chain. Always do
# the dry run first.

set -uo pipefail
cd "$(dirname "$0")"
# shellcheck source=00_env.sh
source ./00_env.sh
# shellcheck source=topic_sets.sh
source ./topic_sets.sh

MAP=""
DRY_RUN=false
MODE=both
while (( $# )); do
  case "$1" in
    --map)         MAP="$2"; shift ;;
    --dry-run)     DRY_RUN=true ;;
    --launch-only) MODE=launch ;;
    --record-only) MODE=record ;;
    -y|--yes)      export CONFIRM=yes ;;
    -h|--help)     sed -n '2,12p' "$0"; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
  shift
done

if [[ -z "$MAP" ]]; then
  MAP="$(ls -t "${SSD_ROOT}"/maps/*/*.yaml 2>/dev/null | head -1)"
  [[ -n "$MAP" ]] && warn "no --map given; using most recent: $MAP"
fi
[[ -f "$MAP" ]] || die "map not found: ${MAP:-<none>}"

CMD_VEL_TOPIC=cmd_vel
$DRY_RUN && CMD_VEL_TOPIC=cmd_vel_nav2

# Nav2 speaks Twist; the vehicle takes AckermannDriveStamped. twist_to_ackermann
# is the only bridge (cmd_vel -> drive -> mux -> command_gate -> VESC) and it
# defaults False because the MPC publishes `drive` directly. Without it a live
# Nav2 run plans, controls and commands perfectly while the car never moves.
# Off in a dry run, where the point is that nothing reaches the actuation chain.
TWIST_TO_ACKERMANN=True
$DRY_RUN && TWIST_TO_ACKERMANN=False

print_env
ensure_dirs
info "map: $MAP"
$DRY_RUN && warn "DRY RUN — velocity smoother output goes to $CMD_VEL_TOPIC, not the vehicle"

LAUNCH_PID=

start_stack() {
  if $DRY_RUN; then
    info "dry run: Nav2 cannot reach the actuation chain"
  else
    confirm_unsafe "LIVE Nav2: the planner will command the vehicle."
  fi

  banner "launching full bringup with navigation"
  ros2 launch f1tenth_launch bringup.launch.py \
      use_f1tenth_namespace:=True \
      f1tenth_namespace:="$NS" \
      slam:=False \
      use_gpu:="$USE_GPU" \
      map_file:="$MAP" \
      launch_vehicle:=True \
      launch_sensors:=True \
      launch_tfs:=True \
      launch_joystick:=True \
      launch_localization:=True \
      launch_local_localization:=True \
      launch_global_localization:=True \
      launch_map_server:=True \
      map_tf_publisher:=ekf \
      odom_tf_publisher:=ekf \
      localize_isaac_vslam_on_startup:="$VSLAM_LOCALIZE_ON_STARTUP" \
      launch_navigation:=True \
      cmd_vel_topic:="$CMD_VEL_TOPIC" \
      launch_twist_to_ackermann:="$TWIST_TO_ACKERMANN" \
      launch_2d_mapping:=False \
      launch_3d_mapping:=False \
      launch_command_gate:=True \
      command_gate_require_heartbeat:=True \
      reset_realsense:=True \
      max_speed:="$MAX_SPEED" \
      max_steering:="$MAX_STEERING" \
      log_level:=warn &
  LAUNCH_PID=$!
  info "launch pid $LAUNCH_PID"

  wait_for_topic "$(ns_topic map)"                     90 || die "map_server never published"
  wait_for_topic "$(ns_topic global_costmap/costmap)" 120 || die "global costmap never came up"
  wait_for_topic "$(ns_topic local_costmap/costmap)"  120 || die "local costmap never came up"
  sleep 5
}

stop_stack() {
  [[ -n "$LAUNCH_PID" ]] || return 0
  banner "shutting down stack"
  kill -INT "$LAUNCH_PID" 2>/dev/null
  wait "$LAUNCH_PID" 2>/dev/null
}
trap stop_stack EXIT

[[ "$MODE" != record ]] && start_stack

# ---------------------------------------------------------- nav2 checks ----
banner "Nav2 health"

# All eight servers must be present exactly once. Duplicate whole nav2 stacks
# have happened here before (two containers, two of every server).
banner "lifecycle nodes"
ros2 node list 2>/dev/null | grep -E 'controller_server|planner_server|smoother_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|map_server' \
  | sort | uniq -c | sed 's/^/  /'
warn "every line above must show count 1. A count of 2 means a duplicate stack."

# cmd_vel showed 14 publishers with mixed Twist/TwistStamped in an earlier
# session and was never explained. Surface it rather than let it bite silently.
banner "cmd_vel publishers"
ros2 topic info -v "$(ns_topic cmd_vel)" 2>/dev/null | head -30 | sed 's/^/  /'
warn "an earlier session saw 14 publishers here with mixed Twist/TwistStamped.
      More than a couple, or a type mismatch, will make the vehicle ignore Nav2."

./10_preflight.sh nav2 || warn "preflight reported problems"

banner "controller CPU"
info "watch planner_server: it was once seen at 94% CPU with no goal pending"
printf '  top -b -n1 | grep -E "planner_server|controller_server"\n\n'
top -b -n1 2>/dev/null | grep -E 'planner_server|controller_server' | sed 's/^/  /' || true

if [[ "$MODE" == launch ]]; then
  info "stack up; record with ./60_nav2_test.sh --record-only --map $MAP"
  wait "$LAUNCH_PID"; exit 0
fi

set_array nav2

banner "sending a goal"
cat <<EOF
  From RViz: use the "2D Goal Pose" tool (fixed frame must be 'map').

  From the CLI:
    ros2 action send_goal $(ns_topic navigate_to_pose) \\
      nav2_msgs/action/NavigateToPose \\
      '{pose: {header: {frame_id: "map"},
               pose: {position: {x: 1.0, y: 0.0, z: 0.0},
                      orientation: {w: 1.0}}}}'

  Watch for: a global plan on $(ns_topic plan), a local plan, and
  $(ns_topic "$CMD_VEL_TOPIC") going non-zero.
EOF
$DRY_RUN && info "dry run — expect NO wheel motion even with a valid plan"

bag_record "nav2$($DRY_RUN && echo _dryrun)" "${TOPIC_LIST[@]}"

banner "next"
$DRY_RUN && cat <<EOF
  If the dry run planned correctly, repeat live (car on the ground, hand on the
  joystick — releasing the deadman is your override):
      ./60_nav2_test.sh --map "$MAP"
EOF
