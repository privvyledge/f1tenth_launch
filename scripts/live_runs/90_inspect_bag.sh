#!/usr/bin/env bash
# 90_inspect_bag.sh — verify a recorded bag before you trust or delete it.
#
#   ./90_inspect_bag.sh                 # most recent bag
#   ./90_inspect_bag.sh <bag_path>
#   ./90_inspect_bag.sh <bag_a> <bag_b> # compare two (raw vs pointcloud)
#
# Reports message counts, actual rates, gap distribution and stalls. An "empty"
# topic — present in the bag but with zero messages — is the failure mode that
# hid the namespace bugs, so it is called out explicitly.

set -uo pipefail
cd "$(dirname "$0")"
# shellcheck source=00_env.sh
source ./00_env.sh
# shellcheck source=topic_sets.sh
source ./topic_sets.sh

BAGS=("$@")
if (( ${#BAGS[@]} == 0 )); then
  [[ -f "$BAG_ROOT/.last_bag" ]] || die "no bag given and no $BAG_ROOT/.last_bag"
  BAGS=("$(cat "$BAG_ROOT/.last_bag")")
fi

STATS="../analysis/bag_stats.py"
[[ -f "$STATS" ]] || die "missing $STATS"

# Rate floors reused from topic_sets.sh so there is one source of truth.
EXPECT_ARGS=()
for rel in lidar/scan_filtered camera/color/image_raw \
           camera/aligned_depth_to_color/image_raw camera/imu/filtered \
           vehicle/vesc_odom odometry/local; do
  min="${MIN_RATE[$rel]:-}"
  [[ -n "$min" ]] && EXPECT_ARGS+=(--expect "${rel}=${min}")
done

for bag in "${BAGS[@]}"; do
  [[ -e "$bag" ]] || { err "not found: $bag"; continue; }

  banner "ros2 bag info — $(basename "$bag")"
  ros2 bag info "$bag" 2>/dev/null | sed 's/^/  /'

  banner "size on disk"
  du -sh "$bag" 2>/dev/null | sed 's/^/  /'

  banner "rate and gap analysis"
  python3 "$STATS" "$bag" "${EXPECT_ARGS[@]}"
  rc=$?
  if (( rc == 0 )); then
    info "$(basename "$bag") looks healthy"
  else
    warn "$(basename "$bag") has problems — see the failures above"
  fi
done

# --------------------------------------------------- raw vs cloud verdict ----
if (( ${#BAGS[@]} == 2 )); then
  banner "comparison"
  cat <<'EOF'
  Judging the raw-vs-pointcloud question:

    - Compare the Hz column for camera/color/image_raw and
      aligned_depth_to_color between the two bags. If the pointcloud bag shows
      materially lower camera rates, publishing the cloud is costing you frames
      on the streams the detectors actually need.
    - Check the 'stalls' column. Stalls that appear only in the pointcloud bag
      mean the recorder is I/O bound.
    - Compare size on disk against the length of each run.

  If the cloud variant degrades the raw streams, keep publish_realsense_pointcloud
  False and regenerate the colored cloud offline from depth + colour + camera_info.
EOF
fi

banner "next"
cat <<EOF
  Keep what you need, then free space:
      ./99_prune_bags.sh
EOF
