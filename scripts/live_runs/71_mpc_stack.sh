#!/usr/bin/env bash
# 71_mpc_stack.sh — bring the stack up and leave /<ns>/drive free for an
# external MPC, then stop and hold. Nothing here touches the controller.
#
#   ./71_mpc_stack.sh
#   ./71_mpc_stack.sh --map <map.yaml> --domain 42
#
# Use this when a SEPARATE agent or process owns the MPC node and runs its own
# checks. It is the bring-up half only; 70_mpc_handover.sh is the variant that
# also walks an operator through the deadman handover and records a bag.
#
# What it launches
# ----------------
#   vehicle + sensors + TFs + joystick, local EKF (odom->base_link), and AMCL
#   feeding the map EKF, which owns map->odom (map_tf_publisher:=ekf at 30 Hz —
#   smoother than AMCL's own broadcast; see LOCALIZER_FOLLOWUPS.md).
#   Nav2 and twist_to_ackermann are OFF, so the mux's 'navigation' channel
#   (topic: drive) has exactly one publisher: your MPC.
#
# Handover, in one line
# ---------------------
#   Hold R1 (SDL button 10). That silences 'teleop' by design, the joystick's
#   priority-100 claim lapses after 0.3 s, and 'drive' wins. Releasing R1 is the
#   override. Never button 5 — that is PS/power-off.
#
# BEFORE YOU RUN THIS: the container needs X11, or the RealSense dies
# -------------------------------------------------------------------
# librealsense needs a GL context. A container started from a shell with no
# DISPLAY (any ssh session) silently loses the X11 mounts: /tmp/.X11-unix comes
# up empty and /tmp/.docker.xauth is a 0-byte file from the image, while
# DISPLAY/XAUTHORITY still *look* correct in env. Every camera stream then sits
# at 0 Hz and Isaac VSLAM has no input. On the HOST, before starting it:
#
#     export DISPLAY=:0 XAUTHORITY=$HOME/.Xauthority
#     xauth nlist $DISPLAY | sed -e 's/^..../ffff/' \
#         | xauth -f /tmp/.docker.xauth nmerge -
#     chmod 644 /tmp/.docker.xauth
#
# Then verify INSIDE the container that /tmp/.X11-unix/X0 exists and
# /tmp/.docker.xauth is non-zero. Checking env alone will not catch this.

set -uo pipefail
cd "$(dirname "$0")"
# shellcheck source=00_env.sh
source ./00_env.sh

MAP=""
while (( $# )); do
  case "$1" in
    --map)     MAP="$2"; shift ;;
    --domain)  export ROS_DOMAIN_ID="$2"; shift ;;
    -y|--yes)  export CONFIRM=yes ;;
    -h|--help) sed -n '2,45p' "$0"; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
  shift
done

if [[ -z "$MAP" ]]; then
  MAP="$(ls -t "${SSD_ROOT}"/maps/*/*.yaml 2>/dev/null | head -1)"
  [[ -n "$MAP" ]] && warn "no --map given; using most recent: $MAP"
fi
[[ -f "$MAP" ]] || die "map not found: ${MAP:-<none>}"

print_env
info "map: $MAP"

# Another agent's stack on the same domain is invisible until its topics collide
# with yours. Check from the HOST — a container's ps only sees its own PIDs.
banner "domain check"
info "this run is on ROS_DOMAIN_ID=$ROS_DOMAIN_ID — the MPC must export the same"

confirm_unsafe "MPC bring-up: an external controller will be free to command the vehicle."

banner "launching bringup (localization on, Nav2 off, drive free)"
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
    map_frequency:=30.0 \
    localize_isaac_vslam_on_startup:="$VSLAM_LOCALIZE_ON_STARTUP" \
    launch_navigation:=False \
    launch_twist_to_ackermann:=False \
    launch_2d_mapping:=False \
    launch_3d_mapping:=False \
    launch_command_gate:=True \
    command_gate_require_heartbeat:=True \
    launch_throttle_interpolator_node:=False \
    reset_realsense:="$RESET_REALSENSE" \
    max_speed:="$MAX_SPEED" \
    max_steering:="$MAX_STEERING" \
    log_level:=warn &
LAUNCH_PID=$!

stop_stack() {
  banner "shutting down"
  stop_launch_tree "$LAUNCH_PID"
}
trap stop_stack EXIT

wait_for_topic "$(ns_topic map)"            90 || die "map_server never published"
wait_for_topic "$(ns_topic odometry/local)" 60 || die "local EKF never started"
sleep 10

# ------------------------------------------------------------- verification --
# Rates, not just presence: a topic that exists and publishes nothing is the
# failure mode that hides every bug on this vehicle.
banner "stack health"
rc=0
require_rate "$(ns_topic vehicle/sensors/core)"    40 || rc=1
require_rate "$(ns_topic vehicle/vesc_odom)"       40 || rc=1
require_rate "$(ns_topic vehicle/sensors/imu/raw)" 80 || rc=1
require_rate "$(ns_topic lidar/scan_filtered)"      5 || rc=1
require_rate "$(ns_topic odometry/local)"          25 || rc=1
require_rate "$(ns_topic odometry/global)"         25 || rc=1
require_rate "$(ns_topic command_gate/heartbeat)"   5 || rc=1
# Camera at 0 Hz almost always means the X11/GL problem in the header above.
require_rate "$(ns_topic camera/color/image_raw)"   20 \
  || { rc=1; err "camera dead — check /tmp/.X11-unix/X0 INSIDE the container"; }
require_rate "$(ns_topic visual_slam/tracking/odometry)" 20 || rc=1

banner "TF"
for pair in "map odom" "odom base_link" "base_link lidar" "base_link camera_link"; do
  # shellcheck disable=SC2086
  set -- $pair
  if tf_has_edge "$1" "$2"; then info "TF $1 -> $2"; else err "TF MISSING $1 -> $2"; rc=1; fi
done

# amcl_pose and map are latched/event-driven — 0 Hz is normal, so assert the
# lifecycle state instead of a rate.
banner "AMCL"
state="$(timeout 15 ros2 service call "$(ns_topic amcl)/get_state" \
         lifecycle_msgs/srv/GetState 2>/dev/null | grep -oP "label='\K[a-z]+" | tail -1)"
if [[ "$state" == active ]]; then info "amcl lifecycle: active"; else err "amcl lifecycle: ${state:-unknown}"; rc=1; fi

(( rc == 0 )) && info "stack healthy" || err "one or more checks FAILED — see above"

banner "ready for the MPC"
cat <<EOF
  The MPC process must use the SAME domain:

      export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
      export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION
      # publish ackermann_msgs/AckermannDriveStamped -> $(ns_topic drive)

  Heading caveat: $(ns_topic odometry/local) inherits whatever yaw drift the
  IMU chain has. Check it parked before trusting it:

      python3 $LIVE_RUNS_DIR/yaw_drift.py 60

  Watch the actuation chain by VALUE, never by 'ros2 topic hz' — a closed gate
  publishes zeros at full rate:

      ros2 topic echo $(ns_topic vehicle/ackermann_cmd) --field drive

  Ctrl-C here brings the stack down.
EOF

# Hold the stack up until the operator stops it; the EXIT trap does teardown.
wait "$LAUNCH_PID"
