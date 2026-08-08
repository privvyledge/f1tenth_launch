#!/usr/bin/env bash
# 10_preflight.sh — read-only health gate. Launches nothing, changes nothing.
#
#   ./10_preflight.sh              # hardware + environment checks only
#   ./10_preflight.sh sensors      # also verify the 'sensors' topic set is live
#   ./10_preflight.sh localization # also verify map->odom->base_link ownership
#
# Exits non-zero on the first hard failure with a stated reason. Run it before
# every phase, and again after a stack comes up.

set -uo pipefail
cd "$(dirname "$0")"
# shellcheck source=00_env.sh
source ./00_env.sh
# shellcheck source=topic_sets.sh
source ./topic_sets.sh

PHASE="${1:-}"
FAILURES=0
fail() { err "$*"; FAILURES=$((FAILURES + 1)); }

print_env

# ------------------------------------------------------- 1. environment ----
banner "1. environment"

[[ -d "$F1TENTH_WS/install" ]] \
  && info "workspace overlay present: $F1TENTH_WS/install" \
  || fail "no built workspace at $F1TENTH_WS/install — run colcon build"

if [[ -d "$SSD_ROOT" ]]; then
  check_disk 20 || FAILURES=$((FAILURES + 1))
else
  fail "$SSD_ROOT not mounted. Inside the container this should be the host's
        /mnt/f1tenth_ssd/shared_dir. Never fall back to the SD card root."
fi

if ros2 pkg prefix f1tenth_launch >/dev/null 2>&1; then
  info "f1tenth_launch package found"
else
  fail "f1tenth_launch not on the ROS package path — did you source the overlay?"
fi

info "RMW=$RMW_IMPLEMENTATION  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
case "$DDS_PROFILE" in
  static)
    warn "DDS_PROFILE=static ($CYCLONEDDS_URI).
      Static-peer configs make 'ros2 node list' intermittently blind; prefer
      'ros2 topic hz' over node introspection when diagnosing. Unreachable
      peers also flood stderr — see CYCLONEDDS_PEERS.md." ;;
  lo)
    warn "DDS_PROFILE=lo ($CYCLONEDDS_URI).
      Loopback only: NOTHING off this machine can see these topics, and it
      fails silently. Remote RViz will show an empty world with no error." ;;
  *)
    [[ -n "${CYCLONEDDS_URI:-}" ]] && warn "CYCLONEDDS_URI is set ($CYCLONEDDS_URI)
      but DDS_PROFILE=$DDS_PROFILE — the config is whatever the caller inherited." ;;
esac

# ---------------------------------------------------------- 2. hardware ----
banner "2. hardware"

# EXPECT_VESC / EXPECT_JOYSTICK let a sensor-only phase (nothing actuates, so
# neither device is needed) downgrade these from hard failures to notes. They
# default true: for every phase that can move the car, a missing VESC or
# joystick must still abort.
if [[ -e /dev/sensors/vesc ]]; then
  info "VESC present at /dev/sensors/vesc"
elif [[ "${EXPECT_VESC:-true}" != "true" ]]; then
  info "VESC absent — not required for this phase (EXPECT_VESC=false)"
else
  fail "/dev/sensors/vesc missing — the udev symlink is not in place, the VESC
        is unpowered, or /dev is not passed into the container."
fi

shopt -s nullglob
JOYS=(/dev/input/js*)
shopt -u nullglob
if (( ${#JOYS[@]} > 0 )); then
  info "joystick present: ${JOYS[*]}"
elif [[ "${EXPECT_JOYSTICK:-true}" != "true" ]]; then
  info "joystick absent — not required for this phase (EXPECT_JOYSTICK=false)"
else
  fail "no /dev/input/js* — without a joystick the command_gate heartbeat never
        arrives and the gate stays closed, so the vehicle will not move."
fi

# USB detection without lsusb, which is not installed in the container. Device
# nodes and sysfs are authoritative anyway; lsusb is only a convenience.
HAVE_LSUSB=false
command -v lsusb >/dev/null 2>&1 && HAVE_LSUSB=true

# RealSense: the UVC driver exposes several /dev/video* nodes, and the vendor
# id appears in sysfs. 8086 = Intel.
if grep -qs 8086 /sys/bus/usb/devices/*/idVendor 2>/dev/null; then
  info "Intel USB device (RealSense) present in sysfs"
elif compgen -G "/dev/video*" >/dev/null; then
  warn "video devices present ($(compgen -G '/dev/video*' | tr '\n' ' ')) but the
      Intel vendor id was not found in sysfs — verify the camera is the D435i"
else
  warn "no /dev/video* nodes and no Intel device in sysfs. If the camera was
      wedged by a container restart, relaunch with RESET_REALSENSE=True
      (it is opt-in now — the launch files default it False)."
fi

# YDLidar X4 sits behind a CP210x USB-serial bridge -> /dev/ttyUSB*.
if compgen -G "/dev/ttyUSB*" >/dev/null; then
  info "USB-serial present: $(compgen -G '/dev/ttyUSB*' | tr '\n' ' ') (YDLidar bridge)"
else
  warn "no /dev/ttyUSB* — the YDLidar USB-serial bridge is not enumerated"
fi

$HAVE_LSUSB || info "(lsusb not installed; used sysfs and device nodes instead)"

# --------------------------------------------------------- 3. GPU/VSLAM ----
banner "3. GPU / VSLAM readiness"

if [[ "${USE_GPU,,}" == "true" ]]; then
  if [[ -e /dev/nvhost-ctrl ]] || command -v nvidia-smi >/dev/null 2>&1; then
    info "NVIDIA runtime devices visible"
  else
    fail "USE_GPU=true but no NVIDIA device nodes — the container is missing
          --runtime nvidia. Re-run with USE_GPU=false to use the CPU path."
  fi

  # Isaac VSLAM aborts when told to localize into an empty map directory, and
  # bringup defaults localize_isaac_vslam_on_startup to True. Guard it.
  vslam_map="${VSLAM_MAP_PATH:-/mnt/data/maps/nvidia/vslam_map}"
  if [[ "${VSLAM_LOCALIZE_ON_STARTUP,,}" == "true" ]]; then
    if [[ -d "$vslam_map" ]] && [[ -n "$(ls -A "$vslam_map" 2>/dev/null)" ]]; then
      info "VSLAM map present at $vslam_map; localize_on_startup is safe"
    else
      fail "VSLAM_LOCALIZE_ON_STARTUP=True but $vslam_map is missing or empty.
            Isaac VSLAM is a prime suspect for the unresolved SIGABRT in this
            state. Set VSLAM_LOCALIZE_ON_STARTUP=False."
    fi
  else
    info "VSLAM localize_on_startup disabled (correct with no saved map)"
  fi
else
  warn "USE_GPU=false — Isaac VSLAM will not run. The EKF loses its odom1 input
        (visual_slam/tracking/odometry) and the demo plot loses the VSLAM trace."
fi

# --------------------------------------------- 4. live topics (optional) ----
if [[ -n "$PHASE" ]]; then
  banner "4. live topic check — phase '$PHASE'"

  if ! set_array "$PHASE"; then
    fail "unknown phase '$PHASE'"
  else
    n_up=0
    for t in "${TOPIC_LIST[@]}"; do
      rel="${t#/$NS/}"
      min="${MIN_RATE[$rel]:-}"
      # RATE_EXEMPT lets a phase declare that a topic is legitimately silent in
      # its configuration, so a rate floor defined for other phases does not
      # manufacture a failure. Presence is still checked.
      if [[ " ${RATE_EXEMPT:-} " == *" $rel "* ]]; then
        min=""
      fi
      if [[ -z "$min" ]]; then
        # No rate floor defined (latched, event-driven, or command topics):
        # presence alone is the check.
        if have_topic "$t"; then info "$(printf '%-52s present' "$t")"; n_up=$((n_up+1))
        else warn "$(printf '%-52s absent' "$t")"; fi
      else
        if require_rate "$t" "$min"; then n_up=$((n_up+1))
        else FAILURES=$((FAILURES + 1)); fi
      fi
    done
    info "$n_up/${#TOPIC_LIST[@]} topics healthy"
  fi

  # --------------------------------------------------- 5. TF ownership ----
  banner "5. TF tree"

  for edge in "odom base_link" "base_link lidar" "base_link imu_link"; do
    # shellcheck disable=SC2086
    set -- $edge
    if tf_has_edge "$1" "$2"; then info "TF $1 -> $2"
    else fail "TF edge missing: $1 -> $2"; fi
  done

  if [[ "$PHASE" == "localization" || "$PHASE" == "nav2" || "$PHASE" == "mpc" ]]; then
    if tf_has_edge map odom; then
      info "TF map -> odom"
    else
      fail "TF map -> odom missing. With map_tf_publisher:='ekf' this edge is
            owned by ekf_map_node; if it is absent the global EKF has no pose
            input (check amcl_pose is publishing) or ekf_map never started."
    fi

    banner "5b. map->odom sole ownership"
    warn "Automated publisher-per-edge counting is not reliable through /tf.
      Confirm manually that exactly ONE node broadcasts map->odom:
        ros2 run tf2_tools view_frames --ros-args -r __ns:=/$NS
      With map_tf_publisher:='ekf', ekf_map_node must be the only broadcaster
      and amcl must show tf_broadcast=false:
        ros2 param get /$NS/amcl tf_broadcast    # expect: false"
  fi
fi

# ------------------------------------------------------------- verdict ----
banner "verdict"
if (( FAILURES == 0 )); then
  info "preflight PASSED"
  exit 0
fi
err "preflight FAILED with $FAILURES problem(s) — fix before running a phase"
exit 1
