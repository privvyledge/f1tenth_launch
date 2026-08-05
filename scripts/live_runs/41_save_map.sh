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

# rtabmap_republish — force RTABMap to re-publish its occupancy grid.
#
# slam_toolbox latches /map, so a saver can attach at any time and get the last
# map. RTABMap's grid is NOT latched: it publishes on update only. While a bag
# is replaying that is invisible, because updates are constant — but once the
# mapper finishes processing it goes silent, and a saver that attaches then
# waits out its whole timeout and dies with "Failed to spin map subscription",
# having produced no .pgm. That is precisely the window an offline build saves
# in, so the more correctly you wait for the mapper to finish, the more reliably
# the save used to fail.
#
# The publish_map service makes the node re-emit map topics on demand.
# optimized=true asks for the loop-closure-corrected graph rather than raw odom
# poses; global=true asks for the whole map rather than the local window.
rtabmap_republish() {
  local svc="$(ns_topic rtabmap/publish_map)"
  ros2 service list 2>/dev/null | grep -qxF "$svc" || return 0
  timeout 45s ros2 service call "$svc" rtabmap_msgs/srv/PublishMap \
    "{graph_only: false, optimized: true, global_map: true}" >/dev/null 2>&1 \
    && info "asked rtabmap to re-publish its map" \
    || warn "publish_map call failed — save may time out"
}

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
  # $3, if given, is a function that makes the publisher emit. It has to run
  # AFTER the saver is subscribed — an un-latched publish that happens first is
  # simply missed — so start the saver in the background and trigger second.
  local trigger="${3:-}" saver_pid rc=0
  if [[ -n "$trigger" ]]; then
    timeout 90s ros2 run nav2_map_server map_saver_cli \
        -t "$topic" -f "$out" \
        --ros-args -p map_subscribe_transient_local:=true \
                   -p save_map_timeout:=60.0 &
    saver_pid=$!
    sleep 5
    "$trigger"
    wait "$saver_pid" || rc=1
  else
    timeout 90s ros2 run nav2_map_server map_saver_cli \
        -t "$topic" -f "$out" \
        --ros-args -p map_subscribe_transient_local:=true \
                   -p save_map_timeout:=60.0 || rc=1
  fi
  if (( rc == 0 )) && [[ -f "${out}.yaml" ]]; then
    info "wrote ${out}.yaml"; return 0
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
    #
    # These are NOT under a rtabmap/ prefix. The node is launched as
    # `-r __ns:=/gosling1 -r __node:=rtabmap`, and ROS 2 resolves a node's
    # relative topic names against its NAMESPACE, not its node name — so the
    # grid lands on /<ns>/grid_prob_map. Looking for /<ns>/rtabmap/grid_prob_map
    # found nothing, both branches missed, and the pass reported "topic not
    # present — skipping" while still exiting 0, so an offline build looked
    # successful with no .pgm on disk. COMMANDs.md always had this right.
    if ros2 topic list 2>/dev/null | grep -qxF "$(ns_topic grid_prob_map)"; then
      save_grid "$(ns_topic grid_prob_map)" "${OUT}_rtabmap" rtabmap_republish || RC=1
    else
      save_grid "$(ns_topic grid_map)" "${OUT}_rtabmap" rtabmap_republish || RC=1
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
