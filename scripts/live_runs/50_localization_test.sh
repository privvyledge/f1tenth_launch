#!/usr/bin/env bash
# 50_localization_test.sh — Objective 3: live localization accuracy test.
#
#   ./50_localization_test.sh --map /mnt/shared_dir/maps/<date>/<map>.yaml
#
# Drive a loop and return to the marked start pose. Every localization estimate
# is recorded, so the closure error of each can be computed afterwards with
# scripts/analysis/plot_localization.py.
#
# TF ownership for this test:
#   map -> odom        ekf_map_node   (map_tf_publisher:='ekf')
#   odom -> base_link  ekf_odom_node  (odom_tf_publisher:='ekf')
#
# AMCL runs with tf_broadcast:=False and feeds amcl_pose into the global EKF as
# pose0. The EKF fuses that with VESC/VSLAM odometry, so map->odom is smoother
# than AMCL's raw jumps. Whether AMCL broadcasts is decided by the launch file,
# not by localizer_amcl.yaml - the launch file appends tf_broadcast AFTER
# params_file, so it wins.

set -uo pipefail
cd "$(dirname "$0")"
# shellcheck source=00_env.sh
source ./00_env.sh
# shellcheck source=topic_sets.sh
source ./topic_sets.sh

MAP=""
MODE=both
while (( $# )); do
  case "$1" in
    --map)         MAP="$2"; shift ;;
    --launch-only) MODE=launch ;;
    --record-only) MODE=record ;;
    -y|--yes)      export CONFIRM=yes ;;
    -h|--help)     sed -n '2,22p' "$0"; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
  shift
done

if [[ -z "$MAP" ]]; then
  # Offer the most recent map built by 40_build_map_offline.sh.
  MAP="$(ls -t "${SSD_ROOT}"/maps/*/*.yaml 2>/dev/null | head -1)"
  [[ -n "$MAP" ]] && warn "no --map given; using most recent: $MAP"
fi
[[ -n "$MAP" ]] || die "no map. Pass --map <map.yaml> (build one with 40_build_map_offline.sh)"
[[ -f "$MAP" ]] || die "map file not found: $MAP"

print_env
ensure_dirs
info "map: $MAP"

LAUNCH_PID=

start_stack() {
  confirm_unsafe "Localization test: you will drive a loop under joystick control."

  banner "launching bringup (localization only, navigation off)"
  # launch_navigation:=False keeps Nav2 out of the way; this run is purely
  # about how well the pose estimate closes the loop.
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
      launch_navigation:=False \
      launch_2d_mapping:=False \
      launch_3d_mapping:=False \
      launch_command_gate:=True \
      command_gate_require_heartbeat:=True \
      reset_realsense:="$RESET_REALSENSE" \
      max_speed:="$MAX_SPEED" \
      max_steering:="$MAX_STEERING" \
      log_level:=warn &
  LAUNCH_PID=$!
  info "launch pid $LAUNCH_PID"

  wait_for_topic "$(ns_topic lidar/scan_filtered)" 60 || die "LiDAR never came up"
  wait_for_topic "$(ns_topic map)"                90 || die "map_server never published /map"
  wait_for_topic "$(ns_topic odometry/local)"     60 || die "local EKF never started"
  wait_for_topic "$(ns_topic amcl_pose)"         120 \
    || warn "no amcl_pose yet — AMCL usually needs an initial pose estimate"
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

# ---------------------------------------------------- initial pose help ----
banner "setting the initial pose"
cat <<EOF
  AMCL needs to know roughly where the car is before its particles converge.
  Either use RViz's "2D Pose Estimate" tool, or publish directly:

    ros2 topic pub --once $(ns_topic initialpose) \\
      geometry_msgs/msg/PoseWithCovarianceStamped \\
      '{header: {frame_id: "map"},
        pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0},
                      orientation: {w: 1.0}},
               covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0,
                            0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}'

  Then rotate the car gently in place; the particle cloud should tighten.
EOF
read -r -p "  Press Enter once the particle cloud has converged... " _

# --------------------------------------------------------- verification ----
banner "verifying localization topics and TF ownership"
./10_preflight.sh localization || {
  warn "preflight reported problems."
  read -r -p "Record anyway? [y/N] " a
  [[ "${a,,}" == y ]] || die "aborted"
}

banner "TF ownership check"
cat <<EOF
  Confirm map->odom has exactly ONE broadcaster (must be ekf_map_node):
      ros2 param get $(ns_topic amcl) tf_broadcast     # expect: false
      ros2 run tf2_tools view_frames --ros-args -r __ns:=/$NS

  Three competing map->odom broadcasters (rtabmap + ekf_map + amcl) is a real
  failure mode in this stack; that is what BUG-027 was.
EOF

if [[ "$MODE" == launch ]]; then
  info "stack up; record with ./50_localization_test.sh --record-only --map $MAP"
  wait "$LAUNCH_PID"; exit 0
fi

set_array localization

banner "the accuracy run"
cat <<'EOF'
  1. Park the car exactly on the marked start pose. Pause ~10 s so the
     recording captures a clean stationary reference.
  2. Drive a loop around the space and come back to the SAME marked pose,
     matching the original heading as closely as you can.
  3. Pause ~10 s again at the end.

  The closure error is the distance between the start and end estimates for
  each estimator. Because you physically returned to the same spot, true
  displacement is ~0, so whatever each estimator reports IS its drift.

  Ctrl-C when parked back at the start.
EOF

bag_record "localization_loop" "${TOPIC_LIST[@]}"

banner "next"
cat <<EOF
  Compute closure error and produce the comparison plot for the presentation:
      python3 ../analysis/plot_localization.py "\$(cat "$BAG_ROOT/.last_bag")" \\
          --map "$MAP" --out ../../docs/figures/localization
EOF
