#!/usr/bin/env bash
# 20_sensor_bag.sh — Objective 1: sensor bag for tuning the object-detection repos.
#
# Brings up the teleop stack (vehicle + sensors + LOCAL localization only, no
# map, no AMCL) and records TF, LiDAR, RealSense, VESC and the full actuation
# chain. Drive the car and/or walk moving objects through the scene.
#
#   ./20_sensor_bag.sh                 # raw streams (recommended baseline)
#   ./20_sensor_bag.sh --pointcloud    # also publish + record the colored cloud
#   ./20_sensor_bag.sh --launch-only   # bring the stack up, record separately
#   ./20_sensor_bag.sh --record-only   # record against an already-running stack
#
# The two variants exist to profile whether the colored pointcloud drops frames.
# Record one of each, then compare with ./90_inspect_bag.sh.

set -uo pipefail
cd "$(dirname "$0")"
# shellcheck source=00_env.sh
source ./00_env.sh
# shellcheck source=topic_sets.sh
source ./topic_sets.sh

WITH_CLOUD=false
MODE=both
BAG_NAME=sensors

while (( $# )); do
  case "$1" in
    --pointcloud|--cloud) WITH_CLOUD=true; BAG_NAME=sensors_cloud ;;
    --launch-only)        MODE=launch ;;
    --record-only)        MODE=record ;;
    -y|--yes)             export CONFIRM=yes ;;
    -h|--help)            sed -n '2,20p' "$0"; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
  shift
done

print_env
ensure_dirs
check_disk 30 || die "not enough free space for an image bag"

PHASE=sensors
$WITH_CLOUD && PHASE=sensors_cloud

# ------------------------------------------------------------- launch ----
LAUNCH_PID=

start_stack() {
  confirm_unsafe "Teleop stack: the joystick deadman will command the motor."

  banner "launching teleop stack"
  # Notes on the argument choices:
  #   launch_global_localization:=False  no map yet; local EKF only, which is
  #                                     what puts odom->base_link in the bag
  #   map_tf_publisher:='ekf'           teleop defaults to 'vslam'; with no
  #                                     saved VSLAM map that is the wrong owner
  #   RESET_REALSENSE=True              D435i wedged? re-run with the reset (recovery only)
  #                                     the reset can itself wedge the device — see 00_env.sh
  #   localize_isaac_vslam_on_startup   must stay False without a saved map
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
      publish_realsense_pointcloud:="$($WITH_CLOUD && echo True || echo False)" \
      max_speed:="$MAX_SPEED" \
      max_steering:="$MAX_STEERING" \
      log_level:=warn &
  LAUNCH_PID=$!
  info "launch pid $LAUNCH_PID"

  # Sensors are staged: LiDAR at +2 s, camera at +6 s, localization at +10 s.
  banner "waiting for the stack to settle (camera is delayed ~6 s)"
  wait_for_topic "$(ns_topic lidar/scan_filtered)" 60 || die "LiDAR never came up"
  wait_for_topic "$(ns_topic camera/color/image_raw)" 90 \
    || die "camera never came up — try again with RESET_REALSENSE=True, and
            check the container has /tmp/.X11-unix mounted and DISPLAY set
            (RealSense needs a GL context)"
  wait_for_topic "$(ns_topic odometry/local)" 60 || warn "no odometry/local yet"
  sleep 5   # let static TFs settle before the recorder attaches
}

stop_stack() {
  [[ -n "$LAUNCH_PID" ]] || return 0
  banner "shutting down stack"
  kill -INT "$LAUNCH_PID" 2>/dev/null
  wait "$LAUNCH_PID" 2>/dev/null
  info "stack stopped"
}
trap stop_stack EXIT

[[ "$MODE" != record ]] && start_stack

# -------------------------------------------------------------- verify ----
# The recording rule from the testing checklist: confirm the tree is complete
# BEFORE the recorder attaches, or the bag is unusable for offline SLAM.
banner "pre-record verification"
./10_preflight.sh "$PHASE" || {
  warn "preflight reported problems."
  read -r -p "Record anyway? [y/N] " a
  [[ "${a,,}" == y ]] || die "aborted — fix the failures and rerun"
}

# ------------------------------------------------------------- record ----
if [[ "$MODE" == launch ]]; then
  banner "stack is up; recording skipped (--launch-only)"
  info "record separately with: ./20_sensor_bag.sh --record-only"
  wait "$LAUNCH_PID"
  exit 0
fi

set_array "$PHASE"

banner "what to capture"
cat <<EOF
  1. Hold the car STATIONARY for ~30 s while people/objects move through the
     scene at varying ranges. This is the segmentation/detection tuning data.
  2. Then drive slowly around the objects for another ~60 s so the detectors
     see them from multiple viewpoints and the LiDAR/camera extrinsics get
     exercised.
  3. Press joystick button 5 (autonomous handover) once during the run. It
     routes joy_teleop to the /dev/null sink so the mux's 'drive' channel can
     win. Watch whether ackermann_drive keeps flowing or the command_gate
     heartbeat closes the gate ~1 s later - that is the open question for the
     MPC test.

  Ctrl-C when done.
EOF

bag_record "$BAG_NAME" "${TOPIC_LIST[@]}"

banner "next"
cat <<EOF
  Inspect what you just recorded:
      ./90_inspect_bag.sh "\$(cat "$BAG_ROOT/.last_bag")"

  Then record the other variant and compare drop rates:
      ./20_sensor_bag.sh $($WITH_CLOUD && echo "# (plain)" || echo "--pointcloud")
EOF
