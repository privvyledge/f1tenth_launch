#!/usr/bin/env bash
# 21_detection_dataset_bag.sh — stationary perception dataset for the
# downstream detection / segmentation repos.
#
# The vehicle does not move and the VESC is not connected. You stand in front
# of it and move obstacles of varying size and range through the field of view.
# The bag carries LiDAR, RGB, stereo IR, depth, the colored pointcloud, the
# full TF tree and the odometry the consumers need to project sensor-frame
# detections into odom.
#
#   ./21_detection_dataset_bag.sh                 # 120 s, default scene
#   ./21_detection_dataset_bag.sh --duration 90   # shorter take
#   ./21_detection_dataset_bag.sh --name aisle_boxes
#   ./21_detection_dataset_bag.sh --no-vehicle    # VESC absent/unpowered
#   ./21_detection_dataset_bag.sh --launch-only   # stack up, record yourself
#   ./21_detection_dataset_bag.sh --record-only   # record an already-up stack
#
# Differences from 20_sensor_bag.sh, and why:
#   * launch_joystick:=False       nothing to drive, so no deadman is needed
#   * launch_command_gate:=False   with the gate down nothing publishes
#                                  vehicle/ackermann_cmd, so ackermann_to_vesc
#                                  has no input and the motor cannot be
#                                  commanded at all. The launch file's loud
#                                  "no actuation path" warning is the point
#                                  here, not a problem to fix.
#   * the colored pointcloud is ALWAYS on — it is a requested deliverable, not
#     the opt-in profiling extra it is in 20_sensor_bag.sh
#   * depth camera_info + inter-stream extrinsics + /robot_description are
#     recorded so a consumer repo can rebuild the geometry without this package
#
# The vehicle stack is ON by default (--no-vehicle turns it off). It contributes
# the VESC IMU and vehicle/vesc_odom, which are the EKF's imu0 and odom0. On a
# stationary run vesc_odom reports a hard zero velocity, which is genuinely
# useful: it pins the EKF against rf2o and VSLAM drift instead of letting
# odom->base_link wander while the car sits still. With --no-vehicle the EKF
# falls back to rf2o + the camera IMU + (on the GPU path) Isaac VSLAM, which is
# adequate but noisier.

set -uo pipefail
cd "$(dirname "$0")"
# shellcheck source=00_env.sh
source ./00_env.sh
# shellcheck source=topic_sets.sh
source ./topic_sets.sh

DURATION=120
MODE=both
BAG_NAME=detection
SKIP_PREFLIGHT=false
WITH_VEHICLE="${WITH_VEHICLE:-true}"

while (( $# )); do
  case "$1" in
    --duration|-d)  DURATION="${2:?--duration needs seconds}"; shift ;;
    --name|-n)      BAG_NAME="${2:?--name needs a value}"; shift ;;
    --no-vehicle)   WITH_VEHICLE=false ;;
    --with-vehicle) WITH_VEHICLE=true ;;
    --launch-only)  MODE=launch ;;
    --record-only)  MODE=record ;;
    # Only for when preflight has ALREADY passed against this exact running
    # stack. The rate checks take ~4 s per topic, which is a long time to leave
    # someone standing at the car holding a box.
    --skip-preflight) SKIP_PREFLIGHT=true ;;
    -h|--help)      sed -n '2,34p' "$0"; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
  shift
done

[[ "$DURATION" =~ ^[0-9]+$ ]] || die "--duration must be an integer number of seconds"

# Preflight expectations follow what this run actually uses. The joystick is
# never needed here; the VESC only when the vehicle stack is up. Both default
# to true in 10_preflight.sh, so every phase that CAN move the car still fails
# hard without them.
export EXPECT_VESC="$WITH_VEHICLE"
export EXPECT_JOYSTICK=false
export DETECTION_WITH_VEHICLE="$WITH_VEHICLE"

# Topics that are silent *by design* in this phase, so their rate floors must
# not be enforced. Both are consequences of there being no command path:
#   vehicle/vesc_odom      vesc_to_odom needs sensors/servo_position_command to
#                          derive the steering angle. That is published by
#                          ackermann_to_vesc, which never receives anything
#                          while command_gate is disabled. Measured 0.00 Hz on
#                          2026-08-05 with sensors/core healthy at 49.95 Hz —
#                          the VESC is fine, there is simply nothing to convert.
#   vehicle/sensors/imu/data  the filtered IMU; only appears when the IMU filter
#                          chain is running. sensors/imu/raw (100 Hz) is the one
#                          the EKF actually consumes.
export RATE_EXEMPT="vehicle/vesc_odom vehicle/sensors/imu/data"

# Absent topics that should not stop the run or prompt. Same reasoning.
OPTIONAL_ABSENT=(vehicle/sensors/imu/data)

PHASE=detection

print_env
ensure_dirs
# Colour + stereo + depth + an XYZRGB cloud is roughly 1 GB per recorded
# minute uncompressed. 40 G leaves room for several takes.
check_disk 40 || die "not enough free space for an image + pointcloud bag"

# ------------------------------------------------------------- launch ----
LAUNCH_PID=

start_stack() {
  banner "launching sensors + local localization ($($WITH_VEHICLE && echo 'vehicle ON' || echo 'no vehicle'), no joystick)"

  if $WITH_VEHICLE; then
    # Worth stating plainly rather than assuming: the VESC driver will be
    # running and the motor may be live, but there is no publisher anywhere on
    # the chain that reaches it. command_gate is the sole publisher of
    # vehicle/ackermann_cmd and it is disabled, so ackermann_to_vesc receives
    # nothing. The mux's always-on 'safety' channel stops at the gate too.
    warn "the VESC driver will be running. There is NO command path to the
      motor: command_gate is disabled, so nothing publishes
      vehicle/ackermann_cmd and ackermann_to_vesc has no input. The car
      cannot be commanded to move by this stack."
    read -r -p "  Car is clear / safe to power the VESC? [y/N] " a
    [[ "${a,,}" == y ]] || die "aborted — rerun with --no-vehicle to leave the VESC out"
  else
    info "vehicle stack off — the VESC is not touched"
  fi

  # Emitter stays at its default '0' (off): the D435i's projected dot pattern
  # would contaminate infra1/infra2, and the stereo images are part of the
  # deliverable. Depth and the colored cloud are correspondingly noisier on
  # low-texture surfaces — that is the accepted trade for this dataset.
  ros2 launch f1tenth_launch teleop.launch.py \
      use_f1tenth_namespace:=True \
      f1tenth_namespace:="$NS" \
      use_gpu:="$USE_GPU" \
      launch_vehicle:="$($WITH_VEHICLE && echo True || echo False)" \
      launch_joystick:=False \
      launch_command_gate:=False \
      launch_sensors:=True \
      launch_tfs:=True \
      launch_localization:=True \
      launch_local_localization:=True \
      launch_global_localization:=False \
      launch_icp_odometry:=False \
      map_tf_publisher:=ekf \
      odom_tf_publisher:=ekf \
      localize_isaac_vslam_on_startup:="$VSLAM_LOCALIZE_ON_STARTUP" \
      reset_realsense:="$RESET_REALSENSE" \
      publish_realsense_pointcloud:=True \
      realsense_emitter_enabled:=0 \
      log_level:=warn &
  LAUNCH_PID=$!
  info "launch pid $LAUNCH_PID"

  banner "waiting for the stack to settle (camera is delayed ~6 s)"
  wait_for_topic "$(ns_topic lidar/scan_filtered)" 60 || die "LiDAR never came up"
  wait_for_topic "$(ns_topic camera/color/image_raw)" 90 \
    || die "camera never came up — check the container has /tmp/.X11-unix
            mounted and DISPLAY set (RealSense needs a GL context)"
  wait_for_topic "$(ns_topic camera/depth/color/points)" 60 \
    || warn "no colored pointcloud yet — this is the topic the pointcloud-only
             detector needs; do not record without it"
  if $WITH_VEHICLE; then
    wait_for_topic "$(ns_topic vehicle/vesc_odom)" 45 \
      || warn "no vehicle/vesc_odom — the VESC driver did not come up. Check
               /dev/sensors/vesc exists and no other process holds ttyACM0;
               a stale launch from an earlier session is the usual culprit."
  fi
  wait_for_topic "$(ns_topic odometry/local)" 60 \
    || warn "no odometry/local — check rf2o is alive; without the vehicle stack
             the EKF is running on rf2o and the camera IMU alone"
  sleep 5   # let the latched static TFs and extrinsics settle
}

stop_stack() {
  [[ -n "$LAUNCH_PID" ]] || return 0
  banner "shutting down stack"
  stop_launch_tree "$LAUNCH_PID"
  info "stack stopped"
}
trap stop_stack EXIT

[[ "$MODE" != record ]] && start_stack

# -------------------------------------------------------------- verify ----
banner "pre-record verification"
if $SKIP_PREFLIGHT; then
  warn "preflight SKIPPED (--skip-preflight) — only valid because it already
        passed against this same running stack"
else
./10_preflight.sh "$PHASE" || {
  warn "preflight reported problems."
  read -r -p "Record anyway? [y/N] " a
  [[ "${a,,}" == y ]] || die "aborted — fix the failures and rerun"
}
fi

# Record only topics that actually exist. A name that never appears is silently
# dropped by `ros2 bag record`, which is exactly how a missing stream gets
# discovered a day later by the repo that needed it.
set_array "$PHASE"
PRESENT=(); MISSING=()
# Union of several discovery passes, for the reason documented on have_topic in
# 00_env.sh: one `ros2 topic list` under the static-peer config returns a
# partial graph, and a topic wrongly judged absent here silently leaves a
# required stream out of the dataset.
mapfile -t LIVE < <(for _ in 1 2 3; do ros2 topic list 2>/dev/null; sleep 1; done | sort -u)
for t in "${TOPIC_LIST[@]}"; do
  if printf '%s\n' "${LIVE[@]}" | grep -qxF "$t"; then PRESENT+=("$t")
  elif have_topic "$t"; then PRESENT+=("$t")   # slower per-topic confirmation
  else MISSING+=("$t"); fi
done

banner "topic manifest"
printf '  recording %d topic(s)\n' "${#PRESENT[@]}"
printf '    %s\n' "${PRESENT[@]}"
if (( ${#MISSING[@]} )); then
  warn "${#MISSING[@]} expected topic(s) are absent and will NOT be in the bag:"
  printf '    %s\n' "${MISSING[@]}"
  # The four the consumers cannot work without.
  for critical in lidar/scan_filtered camera/color/image_raw \
                  camera/aligned_depth_to_color/image_raw camera/depth/color/points; do
    for m in "${MISSING[@]}"; do
      [[ "$m" == "$(ns_topic "$critical")" ]] && \
        die "$critical is missing — that is a required deliverable, not optional"
    done
  done
  # Only stop to ask if something absent was NOT expected to be.
  UNEXPECTED=0
  for m in "${MISSING[@]}"; do
    rel="${m#/$NS/}"; known=0
    for o in "${OPTIONAL_ABSENT[@]}"; do [[ "$rel" == "$o" ]] && known=1; done
    (( known )) || UNEXPECTED=1
  done
  if (( UNEXPECTED )); then
    read -r -p "Continue without the absent topics? [y/N] " a
    [[ "${a,,}" == y ]] || die "aborted"
  else
    info "all absent topics are known-optional for this phase; continuing"
  fi
fi

if [[ "$MODE" == launch ]]; then
  banner "stack is up; recording skipped (--launch-only)"
  info "record separately with: ./21_detection_dataset_bag.sh --record-only"
  wait "$LAUNCH_PID"
  exit 0
fi

# ------------------------------------------------------------- record ----
banner "scene plan — ${DURATION}s, vehicle stationary throughout"
cat <<EOF
  Vary two things independently: obstacle SCALE and RANGE. The detectors are
  being tuned on both, so a run that only shows one box at one distance is not
  worth the disk.

  Rough thirds of the run (nothing depends on hitting these exactly — the bag
  is timestamped and the consumers will segment it themselves):

    1. LARGE objects, sweeping near -> far. Person, chair, big box. Walk them
       from ~0.5 m out to the far wall and back. This is what the LiDAR
       segmentation and the depth path both see clearly.
    2. SMALL objects. Cone, bottle, shoe. Same near -> far sweep. These are the
       ones that fall below the LiDAR's angular resolution at range and have to
       be carried by the camera.
    3. MIXED and occluded. Two or three objects at different depths, one
       partially behind another, some off to the edges of the FOV. Overlap and
       partial visibility are where the detectors actually disagree.

  Notes that make the data better:
    - Pause ~2 s at each new placement. Motion blur on a moving object is not
      what they are tuning against.
    - Keep something in the 0.12-10 m LiDAR band; the X4 sees nothing outside it.
    - The camera FOV is much narrower than the LiDAR's 360 deg. Objects behind
      the car appear in the scan and in nothing else — that asymmetry is useful
      to include deliberately, not something to avoid.
    - The IR emitter is OFF for this run, so depth on plain white or glossy
      surfaces will be holey. Prefer textured objects where you can.
EOF

printf '\n'
read -r -p "  Press Enter when you are in position and ready to start... " _
for i in 3 2 1; do printf '  starting in %d\r' "$i"; sleep 1; done
printf '  recording now      \n'

export BAG_DURATION="$DURATION"
bag_record "$BAG_NAME" "${PRESENT[@]}" || die "recording failed — nothing was captured"

BAG_PATH="$(cat "${BAG_ROOT}/.last_bag")"

# ----------------------------------------------------------- manifest ----
# The consuming repos get a directory, not this conversation. Everything they
# need to interpret the bag goes next to it.
cat > "${BAG_PATH}.README.md" <<EOF
# Detection / segmentation dataset bag

Recorded $(date -Iseconds) on \`${VEHICLE_NAME}\` (F1/10, ROS 2 Humble).
Storage: \`${BAG_STORAGE}\`, compression: \`${BAG_COMPRESSION}\`.
Duration: ~${DURATION}s.

## Conditions

- **Vehicle stationary** for the entire recording. $(
  if $WITH_VEHICLE; then
    printf 'The VESC was connected and its driver running, but there was **no command\n  path to the motor** (command_gate disabled), so the car could not move.\n  Consequence: `vehicle/vesc_odom` is recorded but contains **zero messages**.\n  `vesc_to_odom` needs `sensors/servo_position_command` to derive the steering\n  angle, and that is only published when a drive command arrives. The VESC\n  itself was healthy — `vehicle/sensors/core` (~50 Hz) and\n  `vehicle/sensors/imu/raw` (~100 Hz) are both fully populated. Use\n  `odometry/local` (EKF, 30 Hz) for pose; it is what carries `odom -> base_link`.'
  else
    printf 'The VESC was **disconnected**, so\n  there are no `vehicle/*` topics, no wheel odometry, and no VESC IMU.'
  fi)
- Obstacles of varying scale were moved through the scene by hand at ranges
  from ~0.5 m to the far wall.
- **D435i IR emitter OFF.** \`infra1\`/\`infra2\` are clean rectified stereo IR
  and are usable for stereo matching. The cost is noisier depth and a holier
  colored pointcloud on low-texture surfaces.

## Namespace

Everything is under \`/${NS}/\`. \`/tf\` and \`/tf_static\` are namespaced too
(\`/${NS}/tf\`), which is deliberate multi-robot support — remap them back to
\`/tf\` on playback if your tooling assumes the global names:

\`\`\`bash
ros2 bag play <bag> --remap /${NS}/tf:=/tf /${NS}/tf_static:=/tf_static
\`\`\`

## Topics

$(printf '  - `%s`\n' "${PRESENT[@]}")

## Frames

\`base_link\` is the rear axle. The static tree (from
\`vehicle/static_transformations.launch.py\` plus the URDF via
\`robot_state_publisher\`, both recorded on \`tf_static\` and
\`robot_description\`):

\`\`\`
base_link
├── sensor_kit_link
│   ├── lidar          (YDLidar X4, frame of lidar/scan*)
│   └── imu_link
├── base_footprint
├── front_axle / rear_axle / *_wheel
└── camera_link -> camera_{color,depth,infra1,infra2}_optical_frame  (URDF)
\`\`\`

\`odom -> base_link\` is broadcast by the local EKF (\`robot_localization\`),
fed by rf2o LiDAR odometry, the camera IMU$( [[ "${USE_GPU,,}" == "true" ]] && printf ', and Isaac VSLAM' ). There is **no \`map\` frame** in
this bag — none was requested, and no global localizer was running.

## Sensor notes that affect tuning

- **YDLidar X4**: valid range 0.12–10.0 m. Out-of-range returns are \`inf\`, not
  \`0.0\`. Nominal 12 Hz, observed ~8 Hz under load. Intensity is meaningless
  (constant ~1008.0 status bits) — the driver cannot report real intensity, so
  do not build a feature on it.
- \`lidar/scan\` is raw; \`lidar/scan_filtered\` has a tuned speckle filter
  applied. Both are recorded so you can measure what the filter costs you.
- \`aligned_depth_to_color/image_raw\` is resampled into the **colour**
  intrinsics — use it with \`color/camera_info\` for pixel-aligned RGB-D.
  \`depth/image_rect_raw\` is the native depth stream with the wider depth FOV;
  use it with \`depth/camera_info\`. They are not interchangeable.
- \`camera/depth/color/points\` is the driver's own XYZRGB cloud, in the depth
  optical frame.
EOF

printf '%s\n' "${PRESENT[@]}" > "${BAG_PATH}.topics.txt"

info "manifest written: ${BAG_PATH}.README.md"

banner "next"
cat <<EOF
  Verify what landed (message counts, per-topic rates, dropped frames):
      ./90_inspect_bag.sh "$BAG_PATH"

  Hand off to the other repos — take the bag directory and both sidecar files:
      $BAG_PATH
      ${BAG_PATH}.README.md
      ${BAG_PATH}.topics.txt

  If a stream came out thin, record another take with --name. Scenes are cheap;
  a second trip to the lab is not.
EOF
