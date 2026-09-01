#!/usr/bin/env bash
# 30_mapping_drive.sh — Objective 2: the teleop drive whose bag becomes the map.
#
# Identical stack to 20_sensor_bag.sh (vehicle + sensors + LOCAL localization),
# but records the topic set needed to rebuild maps offline: everything from the
# sensor set plus the VSLAM odometry streams.
#
# No SLAM runs live. Maps are built afterwards from this bag with
# 40_build_map_offline.sh, which means you can retune SLAM parameters and
# rebuild without re-driving.
#
#   ./30_mapping_drive.sh
#   ./30_mapping_drive.sh --record-only   # against an already-running stack

set -uo pipefail
cd "$(dirname "$0")"
# shellcheck source=00_env.sh
source ./00_env.sh
# shellcheck source=topic_sets.sh
source ./topic_sets.sh

MODE=both
while (( $# )); do
  case "$1" in
    --launch-only) MODE=launch ;;
    --record-only) MODE=record ;;
    -y|--yes)      export CONFIRM=yes ;;
    -h|--help)     sed -n '2,18p' "$0"; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
  shift
done

print_env
ensure_dirs
check_disk 40 || die "a mapping drive needs headroom; free space on the SSD first"

LAUNCH_PID=

start_stack() {
  confirm_unsafe "Mapping drive: you will be driving the car under joystick control."

  banner "launching teleop stack (local localization only, no SLAM)"
  # Local localization only. The EKF publishes odom->base_link, so the bag
  # carries a usable odom TF and the offline SLAM run does not have to
  # regenerate odometry. No map_server, no AMCL - there is no map yet.
  ros2 launch f1tenth_launch teleop.launch.py \
      use_f1tenth_namespace:=True \
      f1tenth_namespace:="$NS" \
      use_gpu:="$USE_GPU" \
      launch_vehicle:=True \
      launch_sensors:=True \
      launch_tfs:=True \
      launch_joystick:=True \
      launch_localization:=True \
      launch_local_localization:=True \
      launch_global_localization:=False \
      map_tf_publisher:=ekf \
      odom_tf_publisher:=ekf \
      localize_isaac_vslam_on_startup:="$VSLAM_LOCALIZE_ON_STARTUP" \
      launch_command_gate:=True \
      command_gate_require_heartbeat:=True \
      reset_realsense:="$RESET_REALSENSE" \
      publish_realsense_pointcloud:=False \
      max_speed:="$MAX_SPEED" \
      max_steering:="$MAX_STEERING" \
      log_level:=warn &
  LAUNCH_PID=$!
  info "launch pid $LAUNCH_PID"

  wait_for_topic "$(ns_topic lidar/scan_filtered)" 60 || die "LiDAR never came up"
  wait_for_topic "$(ns_topic camera/color/image_raw)" 90 || die "camera never came up"
  wait_for_topic "$(ns_topic odometry/local)" 60 || die "EKF never produced odometry/local"
  sleep 5
}

stop_stack() {
  [[ -n "$LAUNCH_PID" ]] || return 0
  banner "shutting down stack"
  stop_launch_tree "$LAUNCH_PID"
}
trap stop_stack EXIT

[[ "$MODE" != record ]] && start_stack

banner "pre-record verification"
./10_preflight.sh mapping || {
  warn "preflight reported problems."
  read -r -p "Record anyway? [y/N] " a
  [[ "${a,,}" == y ]] || die "aborted"
}

if [[ "$MODE" == launch ]]; then
  info "stack up; record with ./30_mapping_drive.sh --record-only"
  wait "$LAUNCH_PID"; exit 0
fi

set_array mapping

banner "how to drive for a good map"
cat <<'EOF'
  - Drive SLOWLY. Rotation is what breaks scan matching; take corners wide and
    avoid spinning in place.
  - Cover the full space, and drive corridors in BOTH directions.
  - Return to the exact start pose at the end and pause there ~10 s. That gives
    the offline run a loop closure and gives you a ground-truth check for the
    localization test later - mark the spot on the floor with tape.
  - Keep moving people out of the scene where you can; they smear the map.
  - 3-8 minutes is usually plenty for a lab.

  Ctrl-C when you are back at the start.
EOF

bag_record "mapping_drive" "${TOPIC_LIST[@]}"

banner "next"
cat <<EOF
  Verify the bag has a complete TF tree and full-rate sensors:
      ./90_inspect_bag.sh "\$(cat "$BAG_ROOT/.last_bag")"

  Then build maps from it (no robot needed):
      ./40_build_map_offline.sh --bag "\$(cat "$BAG_ROOT/.last_bag")" --mode all
EOF
