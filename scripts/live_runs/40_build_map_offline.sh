#!/usr/bin/env bash
# 40_build_map_offline.sh — build maps from a drive bag. No robot required.
#
#   ./40_build_map_offline.sh --bag <path> --mode rtabmap      # RTABMap 2D grid + 3D
#   ./40_build_map_offline.sh --bag <path> --mode slamtoolbox  # SLAM Toolbox 2D only
#   ./40_build_map_offline.sh --bag <path> --mode both         # run both, compare
#
#   --rate 0.2      bag playback rate (default 0.2)
#   --append        keep the existing RTABMap DB instead of wiping it
#   --no-viz        skip RViz (useful over SSH without X)
#
# Why offline rather than SLAM-while-driving: you can retune and rebuild without
# re-driving, and the Jetson is not fighting SLAM for CPU during the drive.
#
# This deliberately drives mapping.launch.py DIRECTLY rather than
# bringup.launch.py slam:=True. Under slam:=True bringup includes localization
# twice (bringup.launch.py:855 and :943 -> mapping.launch.py:577 -> teleop),
# which strands duplicate rtabmap odometry nodes in the root namespace where
# they are permanently starved. mapping.launch.py already defaults
# launch_joystick/sensors/vehicle/tfs to False, which is exactly right for a
# bag replay.

set -uo pipefail
cd "$(dirname "$0")"
# shellcheck source=00_env.sh
source ./00_env.sh
# shellcheck source=topic_sets.sh
source ./topic_sets.sh

BAG=""
MODE=rtabmap
RATE=0.2
APPEND=False
VIZ=True

while (( $# )); do
  case "$1" in
    --bag)    BAG="$2"; shift ;;
    --mode)   MODE="$2"; shift ;;
    --rate)   RATE="$2"; shift ;;
    --append) APPEND=True ;;
    --no-viz) VIZ=False ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
  shift
done

# Fall back to the most recent bag recorded by these scripts.
[[ -z "$BAG" && -f "$BAG_ROOT/.last_bag" ]] && BAG="$(cat "$BAG_ROOT/.last_bag")"
[[ -n "$BAG" ]] || die "no bag given and no $BAG_ROOT/.last_bag — pass --bag <path>"
[[ -e "$BAG" ]] || die "bag not found: $BAG"

print_env
ensure_dirs

STAMP="$(date +%H%M%S)"
MAP_2D_BASE="${MAP_ROOT}/${MODE}_2d_${STAMP}"      # map_saver writes .pgm/.yaml
RTABMAP_DB="${MAP_ROOT}/rtabmap_${STAMP}.db"

banner "offline map build"
printf '  %-16s %s\n' \
  "bag"        "$BAG" \
  "mode"       "$MODE" \
  "rate"       "$RATE" \
  "2D output"  "${MAP_2D_BASE}.yaml" \
  "RTABMap DB" "$RTABMAP_DB"

# ------------------------------------------------- drain / teardown ----
# The mapper does not necessarily finish when the bag does. RTABMap runs with
# Rtabmap/DetectionRate 0 (process every frame) and Rtabmap/ImageBufferSize
# 10000, so if it cannot keep up with playback it queues frames and keeps
# working after the last message is published. The old fixed `sleep 8` gambled
# that the queue was empty; when it was not, the saved grid was whatever had
# been assembled by then, and nothing in the log said "partial".
#
# So poll the mapper's own CPU consumption and only save once it has actually
# gone quiet. utime+stime from /proc/<pid>/stat are in clock ticks (100/s);
# fewer than 20 ticks per 10 s wall (<2% CPU) three times running means it has
# stopped processing rather than merely paused between frames.
#
# At 0.2x on an Orin Nano this bag drains in ~30 s, so the wait is usually
# short. Do not confuse it with RTABMap's *shutdown* flush, which is a separate
# and much longer phase — see stop_launch below.
wait_for_mapper_idle() {
  local pid prev cur idle=0 waited=0
  local max_wait="${MAPPER_DRAIN_TIMEOUT:-1800}"
  pid="$(pgrep -f 'rtabmap_slam/rtabmap' | head -1)"
  [[ -z "$pid" ]] && pid="$(pgrep -f 'async_slam_toolbox|sync_slam_toolbox' | head -1)"
  if [[ -z "$pid" ]]; then
    warn "no mapper process found to wait on — falling back to a fixed 15 s flush"
    sleep 15; return 0
  fi
  prev="$(awk '{print $14+$15}' "/proc/$pid/stat" 2>/dev/null)"
  while (( waited < max_wait )); do
    sleep 10; waited=$((waited + 10))
    cur="$(awk '{print $14+$15}' "/proc/$pid/stat" 2>/dev/null)"
    # Process exited: nothing left to drain.
    [[ -z "$cur" ]] && { info "mapper exited after ${waited}s"; return 0; }
    if (( cur - prev < 20 )); then idle=$((idle + 1)); else idle=0; fi
    prev="$cur"
    if (( idle >= 3 )); then
      info "mapper idle after ${waited}s — backlog drained"
      return 0
    fi
    (( waited % 60 == 0 )) && info "still draining (${waited}s elapsed)"
  done
  warn "mapper still busy after ${max_wait}s — saving anyway; map may be partial"
}

# ros2 launch does not always die on a single SIGINT here: RTABMap defers the
# signal until it has drained, so the old `kill -INT` + unbounded `wait` could
# block forever. In --mode both that meant pass 2 silently never started.
# Escalate instead of trusting one signal.
stop_launch() {
  local pid="$1" i
  # Generous by design. RTABMap does its final graph optimization and writes the
  # completed map to the .db during shutdown, AFTER the SIGINT — on
  # mapping_drive_170025 that took ~10 minutes and grew the database from 94 MB
  # to 118 MB. Killing at 90 s produced a database that opens fine but never
  # received the optimized graph, which is the actual 3D deliverable. Wait it
  # out; only escalate if it is genuinely wedged.
  local grace="${LAUNCH_SHUTDOWN_GRACE:-900}"
  kill -INT "$pid" 2>/dev/null
  for ((i = 0; i < grace; i++)); do
    kill -0 "$pid" 2>/dev/null || break
    sleep 1
  done
  if kill -0 "$pid" 2>/dev/null; then
    warn "launch $pid still alive after ${grace}s — escalating"
    kill -TERM "$pid" 2>/dev/null; sleep 5
    kill -9 "$pid" 2>/dev/null
  fi
  wait "$pid" 2>/dev/null
  # Nodes are orphaned rather than reaped when the launch parent is killed
  # hard; leaving one alive contaminates the next pass (two mappers publishing
  # the same topics into one namespace).
  local p
  for p in 'rtabmap_slam/rtabmap' 'rtabmap_sync/rgbd_sync' 'slam_toolbox' 'map_saver'; do
    pkill -9 -f "$p" 2>/dev/null
  done
  sleep 3
}

# ------------------------------------------------------- run one pass ----
# $1 = human label, remaining = launch args for mapping.launch.py
run_pass() {
  local label="$1"; shift
  banner "pass: $label"

  # use_sim_time:=True everywhere, driven by the bag's --clock.
  # launch_localization:=False + external_odometry:=True: the bag already
  # carries odom->base_link from the live EKF, so nothing here should try to
  # produce or broadcast odometry.
  ros2 launch f1tenth_launch mapping.launch.py \
      use_f1tenth_namespace:=True \
      f1tenth_namespace:="$NS" \
      use_sim_time:=True \
      use_gpu:=False \
      launch_joystick:=False \
      launch_sensors:=False \
      launch_vehicle:=False \
      launch_tfs:=False \
      launch_localization:=False \
      launch_local_localization:=False \
      launch_global_localization:=False \
      external_odometry:=True \
      launch_command_gate:=False \
      launch_map_server:=False \
      launch_map_saver:=True \
      launch_visualization:="$VIZ" \
      life_long_mapping:="$APPEND" \
      default_2d_map_file:="${MAP_2D_BASE}.yaml" \
      rtabmap_database_file:="$RTABMAP_DB" \
      log_level:=warn \
      "$@" &
  local launch_pid=$!

  # Give the SLAM nodes time to come up and subscribe before the clock starts.
  sleep 12

  banner "playing bag at ${RATE}x"
  # --clock 10 publishes /clock at 10 Hz. Rates much above 0.2 overrun the
  # slam_toolbox TF message filter and silently drop scans.
  ros2 bag play "$BAG" --clock 10 --rate "$RATE"

  banner "playback finished — waiting for the mapper to drain its backlog"
  wait_for_mapper_idle

  banner "saving maps for pass '$label'"
  ./41_save_map.sh --mode "$label" --out "$MAP_2D_BASE" --no-banner || \
    warn "map save reported a problem for pass '$label'"

  stop_launch "$launch_pid"
  info "pass '$label' complete"
}

case "$MODE" in
  rtabmap)
    # RTABMap produces the 3D map AND a 2D occupancy grid (grid_prob_map),
    # which you have found cleaner than SLAM Toolbox's output.
    run_pass rtabmap launch_2d_mapping:=False launch_3d_mapping:=True
    ;;
  slamtoolbox)
    # launch_2d_mapping defaults to False, so it must be explicit or the run
    # silently produces no /map and no map_saver.
    run_pass slamtoolbox launch_2d_mapping:=True launch_3d_mapping:=False
    ;;
  both|all)
    run_pass rtabmap     launch_2d_mapping:=False launch_3d_mapping:=True
    MAP_2D_BASE="${MAP_ROOT}/slamtoolbox_2d_${STAMP}"
    run_pass slamtoolbox launch_2d_mapping:=True  launch_3d_mapping:=False
    ;;
  *) die "unknown mode '$MODE' (want: rtabmap | slamtoolbox | both)" ;;
esac

banner "results"
ls -lh "$MAP_ROOT" 2>/dev/null
cat <<EOF

  Compare the 2D grids side by side before choosing one for localization:
      eog ${MAP_ROOT}/*_2d_*.pgm     # or any image viewer

  Then run the localization test against your chosen map:
      ./50_localization_test.sh --map ${MAP_ROOT}/<chosen>.yaml
EOF
