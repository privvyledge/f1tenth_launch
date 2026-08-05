#!/usr/bin/env bash
# 41_save_map.sh — save whatever map is currently in memory.
#
#   ./41_save_map.sh --mode rtabmap      [--out <basename>]
#   ./41_save_map.sh --mode slamtoolbox  [--out <basename>]
#   ./41_save_map.sh --mode vslam        [--out <dir>]
#   ./41_save_map.sh --mode all
#
# Consolidates the save recipes that were previously scattered across
# COMMANDs.md. map_saver_server never auto-saves; every save is on demand.
#
# Callable standalone against a live mapping session, or by
# 40_build_map_offline.sh at the end of a pass.

set -uo pipefail
cd "$(dirname "$0")"
# shellcheck source=00_env.sh
source ./00_env.sh

MODE=all
OUT=""
SHOW_BANNER=true

while (( $# )); do
  case "$1" in
    --mode)      MODE="$2"; shift ;;
    --out)       OUT="$2"; shift ;;
    --no-banner) SHOW_BANNER=false ;;
    -h|--help)   sed -n '2,14p' "$0"; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
  shift
done

ensure_dirs
STAMP="$(date +%H%M%S)"
[[ -z "$OUT" ]] && OUT="${MAP_ROOT}/map_${STAMP}"
# map_saver_cli wants a basename, not a .yaml path; strip the extension if the
# caller passed one (40_build_map_offline.sh does).
OUT="${OUT%.yaml}"

$SHOW_BANNER && print_env

# save_grid <absolute map topic> <basename>
# The map topics are latched (transient local); without
# map_subscribe_transient_local the saver waits forever on an empty queue.
save_grid() {
  local topic="$1" out="$2"
  if ! ros2 topic list 2>/dev/null | grep -qxF "$topic"; then
    warn "topic $topic not present — skipping"
    return 1
  fi
  banner "saving $topic -> ${out}.pgm / ${out}.yaml"
  if timeout 90s ros2 run nav2_map_server map_saver_cli \
        -t "$topic" -f "$out" \
        --ros-args -p map_subscribe_transient_local:=true \
                   -p save_map_timeout:=60.0; then
    [[ -f "${out}.yaml" ]] && info "wrote ${out}.yaml" && return 0
  fi
  err "failed to save $topic"
  return 1
}

RC=0

case "$MODE" in
  slamtoolbox|slam_toolbox|2d|all)
    save_grid "$(ns_topic map)" "${OUT}" || RC=1

    # The serialized pose graph is what lets slam_toolbox resume mapping or run
    # in localization mode later; the .pgm alone cannot do either.
    if ros2 service list 2>/dev/null | grep -q "$(ns_topic slam_toolbox/serialize_map)"; then
      banner "serializing slam_toolbox pose graph"
      timeout 60s ros2 service call "$(ns_topic slam_toolbox/serialize_map)" \
        slam_toolbox/srv/SerializePoseGraph "{filename: '${OUT}_posegraph'}" \
        && info "wrote ${OUT}_posegraph.*" \
        || warn "pose graph serialization failed"
    fi
    [[ "$MODE" == all ]] || exit $RC
    ;;&

  rtabmap|3d|all)
    # RTABMap's occupancy grid lives on grid_prob_map (probabilistic) with
    # grid_map as the binary fallback.
    if ros2 topic list 2>/dev/null | grep -qxF "$(ns_topic rtabmap/grid_prob_map)"; then
      save_grid "$(ns_topic rtabmap/grid_prob_map)" "${OUT}_rtabmap" || RC=1
    else
      save_grid "$(ns_topic rtabmap/grid_map)" "${OUT}_rtabmap" || RC=1
    fi

    banner "RTABMap 3D database"
    cat <<EOF
  The .db is written continuously by the rtabmap node; nothing to call here.
  Export a 3D cloud from it AFTER the mapping session ends:

      rtabmap-databaseViewer <db>
        File -> Export 3D clouds -> set voxel size 0.02-0.05 m -> .pcd

  Use the GUI, not 'rtabmap-export --cloud': its --output is a base name and it
  always appends _cloud.ply. A non-voxelized cloud will also overwhelm the
  Orin Nano when you try to display it.
EOF
    [[ "$MODE" == all ]] || exit $RC
    ;;&

  vslam|all)
    # Isaac VSLAM map saving is available ONLY through this service call:
    # localization.launch.py:915-916 hardcodes save_map/load_map to 'False',
    # so the launch-file save trigger never fires.
    if ros2 service list 2>/dev/null | grep -q "$(ns_topic visual_slam/save_map)"; then
      local_vslam_out="${OUT}_vslam"
      mkdir -p "$local_vslam_out"
      banner "saving Isaac VSLAM map -> $local_vslam_out"
      timeout 120s ros2 service call "$(ns_topic visual_slam/save_map)" \
        isaac_ros_visual_slam_interfaces/srv/FilePath \
        "{file_path: '${local_vslam_out}'}" \
        && info "VSLAM map written to $local_vslam_out" \
        || warn "VSLAM map save failed"
    else
      warn "visual_slam/save_map service not present (VSLAM not running, or USE_GPU=false)"
    fi
    ;;

  *) die "unknown mode '$MODE'" ;;
esac

banner "saved artifacts"
ls -lh "${OUT}"* 2>/dev/null || warn "nothing written under ${OUT}*"
exit $RC
