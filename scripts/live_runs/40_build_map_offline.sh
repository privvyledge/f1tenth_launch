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

  banner "playback finished — letting the mapper flush"
  sleep 8

  banner "saving maps for pass '$label'"
  ./41_save_map.sh --mode "$label" --out "$MAP_2D_BASE" --no-banner || \
    warn "map save reported a problem for pass '$label'"

  kill -INT "$launch_pid" 2>/dev/null
  wait "$launch_pid" 2>/dev/null
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
