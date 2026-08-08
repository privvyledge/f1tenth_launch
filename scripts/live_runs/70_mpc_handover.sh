#!/usr/bin/env bash
# 70_mpc_handover.sh — bring the stack up for an external controller on /drive.
#
#   ./70_mpc_handover.sh --map <map.yaml>
#   ./70_mpc_handover.sh --map <map.yaml> --no-heartbeat
#
# Your MPC publishes AckermannDriveStamped to /<ns>/drive, which is the mux's
# 'navigation' channel. Nav2 is disabled here so nothing else competes for it.
#
# How the handover works
# ----------------------
#   ackermann_mux priorities (config/vehicle/mux.yaml):
#       estop      255   topic: estop        timeout 0.5 s
#       joystick   100   topic: teleop       timeout 0.3 s
#       navigation  10   topic: drive        timeout 0.2 s   <- your MPC
#       safety       1   topic: safety       timeout 0.05 s
#
#   Holding a human deadman button ([4, 9]) makes joy_teleop publish 'teleop',
#   which outranks 'drive'. Holding the autonomous deadman (button 5) routes
#   joy_teleop to its /dev/null sink instead, so 'teleop' goes silent, the
#   joystick claim lapses after 0.3 s, and your 'drive' commands win. That
#   /dev/null topic is intentional - do not "fix" it.
#
#   Full path once you have the mux:
#       drive -> ackermann_drive -> vehicle/ackermann_cmd
#             -> vehicle/commands/{motor/speed, servo/position}
#
# The command_gate heartbeat (bug-048, fixed 2026-08-04)
# ------------------------------------------------------
#   The gate's heartbeat used to be 'teleop', which closed the gate ~1 s into
#   every button-5 hold. It now watches the dedicated command_gate/heartbeat
#   topic, which joy_teleop publishes at idle AND while buttons 4/5/9 are held
#   (heartbeat_idle/heartbeat_deadman in joy_teleop.yaml) - so the handover
#   survives with the watchdog active. The gate still closes ~1 s after the
#   joystick disconnects or joy_teleop dies. --no-heartbeat remains only as a
#   diagnostic escape hatch.

set -uo pipefail
cd "$(dirname "$0")"
# shellcheck source=00_env.sh
source ./00_env.sh
# shellcheck source=topic_sets.sh
source ./topic_sets.sh

MAP=""
REQUIRE_HEARTBEAT=True
while (( $# )); do
  case "$1" in
    --map)          MAP="$2"; shift ;;
    --no-heartbeat) REQUIRE_HEARTBEAT=False ;;
    -y|--yes)       export CONFIRM=yes ;;
    -h|--help)      sed -n '2,40p' "$0"; exit 0 ;;
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
ensure_dirs
info "map: $MAP"
[[ "$REQUIRE_HEARTBEAT" == False ]] && \
  warn "command_gate heartbeat DISABLED — the gate is permanently open.
      The mux still arbitrates, so the joystick deadman remains your override,
      but nothing will close the gate if the joystick disconnects."

confirm_unsafe "MPC handover: an external controller will command the vehicle."

banner "launching bringup (localization on, Nav2 off, /drive free)"
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
    command_gate_require_heartbeat:="$REQUIRE_HEARTBEAT" \
    launch_throttle_interpolator_node:=False \
    reset_realsense:="$RESET_REALSENSE" \
    max_speed:="$MAX_SPEED" \
    max_steering:="$MAX_STEERING" \
    log_level:=warn &
LAUNCH_PID=$!

stop_stack() {
  kill -INT "$LAUNCH_PID" 2>/dev/null
  wait "$LAUNCH_PID" 2>/dev/null
}
trap stop_stack EXIT

wait_for_topic "$(ns_topic map)"            90 || die "map_server never published"
wait_for_topic "$(ns_topic odometry/local)" 60 || die "local EKF never started"
sleep 5

# ------------------------------------------------- handover verification ----
banner "mux priority table"
ros2 param get "$(ns_topic ackermann_mux)" topics 2>/dev/null | sed 's/^/  /' \
  || warn "could not read mux params (CycloneDDS static-peer introspection is flaky)"

banner "handover test — follow along"
cat <<EOF
  Run this in another pane to watch the chain live:

      ros2 topic hz $(ns_topic teleop) \\
                    $(ns_topic drive) \\
                    $(ns_topic ackermann_drive) \\
                    $(ns_topic vehicle/ackermann_cmd)

  Then, with your MPC publishing to $(ns_topic drive):

    Step 1  Hold a HUMAN deadman (button 4 or 9).
            Expect: teleop flowing, ackermann_drive mirrors YOUR joystick.
                    The MPC is outranked.

    Step 2  Release it and hold the AUTONOMOUS deadman (button 5).
            Expect: teleop goes silent, and within ~0.3 s ackermann_drive
                    starts mirroring $(ns_topic drive) instead.

    Step 3  Keep holding button 5 for at least 3 s and ECHO THE VALUES of
            $(ns_topic vehicle/ackermann_cmd):
                ros2 topic echo $(ns_topic vehicle/ackermann_cmd) --field drive
            (hz cannot detect a closed gate - it publishes zeros at full rate)
            PASS  values keep tracking your MPC command for the whole hold
            FAIL  values go to zero at ~1 s -> the command_gate heartbeat died;
                  check command_gate/heartbeat is flowing (bug-048 regression).

    Step 4  Release everything.
            Expect: the mux falls through to 'safety' (zero speed) and the car
                    stops. This is the ultimate fallback and must work.
EOF

set_array mpc
banner "recording the handover"
info "record through all four steps so the transition is captured"
bag_record "mpc_handover" "${TOPIC_LIST[@]}"

banner "next"
cat <<EOF
  Confirm from the bag which link dropped, if any:
      ./90_inspect_bag.sh "\$(cat "$BAG_ROOT/.last_bag")"

  A healthy handover shows drive, ackermann_drive and vehicle/ackermann_cmd all
  at similar rates during step 2-3. If vehicle/ackermann_cmd flatlines while
  ackermann_drive keeps going, the command_gate is the culprit.
EOF
