#!/usr/bin/env bash
# 00_env.sh — shared environment and helpers for f1tenth live run scripts.
#
# Source this, do not execute it:   source "$(dirname "$0")/00_env.sh"
#
# Every value below can be overridden by exporting it before sourcing, e.g.
#   ROS_DOMAIN_ID=77 MAX_SPEED=1.0 ./20_sensor_bag.sh

# Deliberately no `set -e` here: this file is sourced, and killing the caller's
# shell on a failed probe would be hostile. Callers set their own -e.
set -o pipefail

# ---------------------------------------------------------------- paths ----
LIVE_RUNS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LIVE_RUNS_DIR

# ROS 2 workspace inside the container. Not a bind mount — it lives in the
# container filesystem.
export F1TENTH_WS="${F1TENTH_WS:-/workspaces/f1tenth}"

# Bulk storage. Inside the container /mnt/shared_dir is the host's
# /mnt/f1tenth_ssd/shared_dir (NVMe, ~900 GB). The host root filesystem is a
# 28 GB SD card that runs ~95% full — never write bags there.
export SSD_ROOT="${SSD_ROOT:-/mnt/shared_dir}"
export BAG_ROOT="${BAG_ROOT:-${SSD_ROOT}/bags/$(date +%Y%m%d)}"
export MAP_ROOT="${MAP_ROOT:-${SSD_ROOT}/maps/$(date +%Y%m%d)}"
export VIDEO_ROOT="${VIDEO_ROOT:-${SSD_ROOT}/videos/$(date +%Y%m%d)}"

# ------------------------------------------------------------- ros setup ----
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# Robot namespace. bringup resolves this from $VEHICLE_NAME then $USER, and
# raises if neither is set, so we pin it explicitly and pass it through.
export VEHICLE_NAME="${VEHICLE_NAME:-gosling1}"
export NS="${NS:-$VEHICLE_NAME}"

# --------------------------------------------------------- run behaviour ----
# Isaac VSLAM (GPU). Set USE_GPU=false to fall back to the RTABMap/rf2o CPU
# path; the EKF then loses its odom1 (visual_slam/tracking/odometry) input.
export USE_GPU="${USE_GPU:-true}"

# Joystick speed cap in m/s. Low by default so a dry run with the car on the
# ground cannot bolt. Raise deliberately per run.
export MAX_SPEED="${MAX_SPEED:-1.5}"

# 0.25 rad, not the package default of 0.34. With the current calibration
# (servo = -1.4 * angle + 0.56, servo_max 0.92) any left-hand command beyond
# (0.92 - 0.56) / 1.4 = 0.257 rad clips at the servo limit, and the driver
# logs "servo command value ... above maximum limit, clipping". A clipped
# command makes the bag lie: the recorded steering keeps rising while the
# wheels have stopped moving, which quietly corrupts any trajectory-tracking
# or system-identification fit done from it. Raise this back to 0.34 once
# steering_angle_to_servo_offset has been recalibrated toward 0.5.
export MAX_STEERING="${MAX_STEERING:-0.25}"

# Isaac VSLAM refuses to localize sanely into an empty map directory, and
# bringup defaults this to True. Keep it off unless a real map exists.
export VSLAM_LOCALIZE_ON_STARTUP="${VSLAM_LOCALIZE_ON_STARTUP:-False}"

# Recompress bags on the fly. zstd file-level compression roughly halves image
# bags at modest CPU cost; set to "none" if the Jetson is CPU-bound.
export BAG_COMPRESSION="${BAG_COMPRESSION:-none}"
export BAG_STORAGE="${BAG_STORAGE:-mcap}"

# --------------------------------------------------------------- colours ----
if [[ -t 1 ]]; then
  C_RED=$'\033[31m'; C_GRN=$'\033[32m'; C_YEL=$'\033[33m'
  C_BLU=$'\033[34m'; C_BLD=$'\033[1m';  C_OFF=$'\033[0m'
else
  C_RED=; C_GRN=; C_YEL=; C_BLU=; C_BLD=; C_OFF=
fi

banner()  { printf '\n%s=== %s ===%s\n' "$C_BLD$C_BLU" "$*" "$C_OFF"; }
info()    { printf '%s[ ok ]%s %s\n'   "$C_GRN" "$C_OFF" "$*"; }
warn()    { printf '%s[warn]%s %s\n'   "$C_YEL" "$C_OFF" "$*"; }
err()     { printf '%s[FAIL]%s %s\n'   "$C_RED" "$C_OFF" "$*" >&2; }
die()     { err "$*"; exit 1; }

# ------------------------------------------------------------ ros sourcing --
# Idempotent: safe to source this file more than once in a shell.
setup_ros() {
  if [[ -n "${_F1TENTH_ROS_SOURCED:-}" ]]; then
    return 0
  fi

  # The ROS setup scripts reference unset variables (AMENT_TRACE_SETUP_FILES
  # and friends), so `set -u` in a calling script makes them abort. Drop -u for
  # the duration of the sourcing and restore whatever the caller had.
  local had_u=0
  [[ $- == *u* ]] && { had_u=1; set +u; }

  # shellcheck disable=SC1091
  [[ -f /opt/ros/humble/setup.bash ]] && source /opt/ros/humble/setup.bash
  if [[ -f "${F1TENTH_WS}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${F1TENTH_WS}/install/setup.bash"
  else
    warn "no workspace overlay at ${F1TENTH_WS}/install/setup.bash"
  fi

  (( had_u )) && set -u
  export _F1TENTH_ROS_SOURCED=1
}

# ------------------------------------------------------------- utilities ----

# ns_topic <relative/topic> -> /<NS>/relative/topic
ns_topic() { printf '/%s/%s' "$NS" "${1#/}"; }

# have_topic <absolute topic>  — present in the graph at all?
#
# A single `ros2 topic list` is NOT trustworthy on this vehicle. Each CLI call
# spins a fresh node that has to discover the graph from scratch, and under the
# CycloneDDS static-peer config that discovery comes back partial: on
# 2026-08-05 one pass reported color/image_raw, both infra streams and the
# colored pointcloud all "missing" while every one of them was measurably
# publishing at ~30 Hz. Retry, then fall back to asking about the topic
# directly, which resolves names the list pass dropped.
#
# "Present" means SOMEONE IS PUBLISHING it, not merely that the name resolves.
# A topic with only a subscriber resolves fine and reports a Type — so on
# 2026-08-05 a dead YDLidar driver still passed this check, because the
# scan_to_scan_filter_chain was subscribed to the scan topic it never received.
# wait_for_topic then returned success and the caller went on to record a bag
# with no LiDAR in it. Require a non-zero publisher count.
have_topic() {
  local topic="$1" attempt
  for attempt in 1 2 3; do
    if ros2 topic list 2>/dev/null | grep -qxF "$topic"; then
      topic_has_publisher "$topic" && return 0
    fi
    sleep 1
  done
  topic_has_publisher "$topic"
}

# topic_has_publisher <absolute topic> — at least one live publisher?
topic_has_publisher() {
  local n
  n="$(timeout 8s ros2 topic info "$1" 2>/dev/null \
       | grep -oP 'Publisher count: \K[0-9]+' | tail -1)"
  [[ -n "$n" ]] && (( n > 0 ))
}

# topic_hz <absolute topic> [window_secs]  — echoes average Hz, or 0.
# `ros2 topic hz` never exits on its own, so it is time-boxed and parsed.
topic_hz() {
  local topic="$1" secs="${2:-4}" out
  out="$(timeout "${secs}s" ros2 topic hz "$topic" --window 30 2>/dev/null \
         | grep -oP 'average rate: \K[0-9.]+' | tail -1)"
  printf '%s' "${out:-0}"
}

# wait_for_topic <absolute topic> [timeout_secs]
wait_for_topic() {
  local topic="$1" timeout="${2:-60}" waited=0
  while (( waited < timeout )); do
    have_topic "$topic" && { info "topic up: $topic"; return 0; }
    sleep 2; waited=$((waited + 2))
  done
  err "timed out after ${timeout}s waiting for $topic"
  return 1
}

# require_rate <absolute topic> <min_hz> [window]
# A topic that exists but publishes nothing is the failure mode that hid the
# namespace bugs, so 0 Hz is a hard failure, not a pass.
require_rate() {
  local topic="$1" min="$2" window="${3:-4}" hz
  if ! have_topic "$topic"; then err "missing topic: $topic"; return 1; fi
  hz="$(topic_hz "$topic" "$window")"
  if awk -v h="$hz" -v m="$min" 'BEGIN{exit !(h+0 >= m+0)}'; then
    info "$(printf '%-52s %8.2f Hz  (>= %s)' "$topic" "$hz" "$min")"
    return 0
  fi
  err "$(printf '%-52s %8.2f Hz  (want >= %s)' "$topic" "$hz" "$min")"
  return 1
}

# tf_has_edge <parent> <child> — is this TF edge being broadcast?
#
# The remaps are load-bearing, not decoration. tf2_echo's TransformListener
# subscribes to the ABSOLUTE names /tf and /tf_static, so `-r __ns:=/$NS` alone
# moves the node into the namespace without moving its TF subscriptions —
# it then listens on a /tf nobody publishes and reports every edge missing.
# That produced three false "TF edge missing" failures on 2026-08-05 against a
# tree that was demonstrably healthy. Remap the topics themselves.
# The output is captured before grepping rather than piped into it, and that is
# the second half of the bug. This file sets `set -o pipefail`, and tf2_echo
# never exits on its own, so `timeout` always kills it with 124 — piping into
# grep therefore returned 124 for the whole pipeline even when grep HAD matched.
# Combined with the namespace problem above, every TF check in every phase
# reported failure regardless of the tree's actual state.
tf_has_edge() {
  local parent="$1" child="$2" out
  out="$(timeout 12s ros2 run tf2_ros tf2_echo --ros-args \
      -r /tf:="$(ns_topic tf)" -r /tf_static:="$(ns_topic tf_static)" \
      -p use_sim_time:=false -- "$parent" "$child" 2>&1)" || true
  [[ "$out" == *Translation* ]]
}

# tf_edge_publishers <parent> <child> — count distinct nodes publishing an
# edge, by inspecting /tf publishers. Two publishers on one edge means a
# fighting-TF bug (this is what BUG-027 looked like).
tf_publisher_count() {
  ros2 topic info "$(ns_topic tf)" 2>/dev/null \
    | grep -oP 'Publisher count: \K[0-9]+' | tail -1
}

# confirm_unsafe <message> — loud gate before anything that can move the car.
# Bypass with -y / CONFIRM=yes for scripted runs.
confirm_unsafe() {
  [[ "${CONFIRM:-}" == "yes" ]] && { warn "safety prompt bypassed (CONFIRM=yes)"; return 0; }
  printf '\n%s' "$C_RED$C_BLD"
  cat <<'EOF'
  ############################################################
  #  THIS SCRIPT CAN DRIVE THE VEHICLE                       #
  #                                                          #
  #  Before continuing, confirm ONE of:                      #
  #    - the drive battery is DISCONNECTED, or                #
  #    - the car is ELEVATED with wheels clear of the ground  #
  #                                                          #
  #  Keep the joystick in hand. Releasing the deadman button  #
  #  stops the car.                                           #
  ############################################################
EOF
  printf '%s' "$C_OFF"
  printf '  %s\n' "$1"
  read -r -p "  Type 'go' to continue: " reply
  [[ "$reply" == "go" ]] || die "aborted at safety prompt"
}

# ensure_dirs — create the run output directories on the SSD.
ensure_dirs() {
  mkdir -p "$BAG_ROOT" "$MAP_ROOT" "$VIDEO_ROOT" \
    || die "cannot create output dirs under $SSD_ROOT"
}

# check_disk <min_gb> — refuse to record onto a nearly-full volume.
check_disk() {
  local min_gb="${1:-20}" avail
  avail="$(df -BG --output=avail "$SSD_ROOT" 2>/dev/null | tail -1 | tr -dc '0-9')"
  [[ -z "$avail" ]] && { err "cannot stat $SSD_ROOT — is the SSD mounted?"; return 1; }
  if (( avail < min_gb )); then
    err "only ${avail}G free on $SSD_ROOT (want >= ${min_gb}G)"
    return 1
  fi
  info "$SSD_ROOT has ${avail}G free"
}

# bag_record <name> <topic...> — record to $BAG_ROOT/<name>_<HHMMSS>.
# Runs in the foreground; Ctrl-C stops it cleanly and the path is echoed.
bag_record() {
  local name="$1"; shift
  local out="${BAG_ROOT}/${name}_$(date +%H%M%S)"
  local args=(-o "$out" -s "$BAG_STORAGE")
  [[ "$BAG_COMPRESSION" != "none" ]] && \
    args+=(--compression-mode file --compression-format "$BAG_COMPRESSION")
  # Callers may add flags via the BAG_RECORD_EXTRA array. The +"${...}" form is
  # the one that stays empty under `set -u`.
  args+=(${BAG_RECORD_EXTRA[@]+"${BAG_RECORD_EXTRA[@]}"})

  banner "recording -> $out"
  printf '  %d topics\n\n' "$#"

  # ROS 2 Humble's `ros2 bag record` has NO record-for-N-seconds option. Its
  # -d/--max-bag-duration is a *split* interval, and passing --duration makes
  # the CLI exit immediately with "unrecognized arguments". A fixed-length take
  # therefore has to be done by timing a SIGINT ourselves. (Cost of learning
  # this the hard way: one 120 s scene recorded into nothing on 2026-08-05.)
  local rc=0
  if [[ -n "${BAG_DURATION:-}" ]]; then
    printf '  stopping automatically after %s s\n\n' "$BAG_DURATION"
    ros2 bag record "${args[@]}" "$@" &
    local rec=$! timer
    ( sleep "$BAG_DURATION"; kill -INT "$rec" 2>/dev/null ) & timer=$!
    wait "$rec"; rc=$?
    kill "$timer" 2>/dev/null
  else
    printf '  Ctrl-C to stop\n\n'
    ros2 bag record "${args[@]}" "$@"; rc=$?
  fi
  printf '\n'

  # Never claim success without looking. The recorder exiting non-zero is
  # normal here (SIGINT), so the authority is the bag itself: rosbag2 writes
  # metadata.yaml on clean shutdown, and its absence means no usable bag.
  if [[ ! -s "$out/metadata.yaml" ]]; then
    err "NO BAG WRITTEN to $out (recorder exit $rc, no metadata.yaml).
         Nothing was captured — do not treat this run as recorded."
    return 1
  fi
  info "bag written: $out ($(du -sh "$out" 2>/dev/null | cut -f1))"
  printf '%s\n' "$out" > "${BAG_ROOT}/.last_bag"
}

# print_env — one-line summary shown at the top of every run script.
print_env() {
  banner "environment"
  printf '  %-22s %s\n' \
    "namespace"       "/$NS" \
    "ROS_DOMAIN_ID"   "$ROS_DOMAIN_ID" \
    "RMW"             "$RMW_IMPLEMENTATION" \
    "workspace"       "$F1TENTH_WS" \
    "bag root"        "$BAG_ROOT" \
    "map root"        "$MAP_ROOT" \
    "use_gpu"         "$USE_GPU" \
    "max_speed"       "$MAX_SPEED m/s"
}

setup_ros
