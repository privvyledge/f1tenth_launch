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
export MAX_STEERING="${MAX_STEERING:-0.34}"

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
have_topic() {
  ros2 topic list 2>/dev/null | grep -qxF "$1"
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
tf_has_edge() {
  local parent="$1" child="$2"
  timeout 12s ros2 run tf2_ros tf2_echo --ros-args -r __ns:="/$NS" \
      -p use_sim_time:=false -- "$parent" "$child" 2>&1 \
    | grep -q 'Translation'
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

  banner "recording -> $out"
  printf '  %d topics; Ctrl-C to stop\n\n' "$#"
  ros2 bag record "${args[@]}" "$@"
  printf '\n'
  info "bag written: $out"
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
