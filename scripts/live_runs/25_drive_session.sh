#!/usr/bin/env bash
# 25_drive_session.sh — a long-lived driving session that records SEVERAL bags.
#
# The other record scripts (20/21/30) are one-shot: they launch the stack,
# record exactly one bag, and tear everything down on Ctrl-C. A run day where
# you drive a mapping lap, then loop laps, then figure-8s needs the opposite
# shape — bring the stack up ONCE and start/stop named bags against it on
# command, so every bag shares one calibration, one TF tree, and one set of
# sensor clocks.
#
# Every subcommand returns immediately, so this is drivable from a remote
# shell (or by an agent over ssh) rather than needing an interactive Ctrl-C.
#
#   ./25_drive_session.sh launch --max-speed 1.0
#   ./25_drive_session.sh start mapping_drive
#   ...drive...
#   ./25_drive_session.sh stop
#   ./25_drive_session.sh start loop_laps
#   ...drive...
#   ./25_drive_session.sh stop
#   ./25_drive_session.sh shutdown
#
# All bags use the `drive` topic set from topic_sets.sh, so bags from one
# session are interchangeable downstream.

set -uo pipefail
cd "$(dirname "$0")"
# shellcheck source=00_env.sh
source ./00_env.sh
# shellcheck source=topic_sets.sh
source ./topic_sets.sh

# --------------------------------------------------------------- state ----
# Kept on the SSD next to the bags, not in /tmp: a container restart wipes
# /tmp and would strand a running stack with no pid to kill.
SESSION_DIR="${SSD_ROOT}/run/session"
STACK_PID_F="${SESSION_DIR}/stack.pid"
STACK_LOG="${SESSION_DIR}/stack.log"
REC_PID_F="${SESSION_DIR}/recorder.pid"
REC_DIR_F="${SESSION_DIR}/recorder.dir"
REC_LOG="${SESSION_DIR}/recorder.log"
REC_TOPICS_F="${SESSION_DIR}/recorder.topics"

alive() { [[ -n "${1:-}" ]] && kill -0 "$1" 2>/dev/null; }
read_pid() { [[ -s "$1" ]] && cat "$1" || true; }

# Signal the whole process group. Both the stack and the recorder are started
# under setsid, so pgid == pid and the negative form reaches every child.
# `ros2 launch` in particular ignores a SIGINT delivered only to the parent.
kill_group() {
  local pid="$1" sig="${2:-INT}"
  alive "$pid" || return 0
  kill -"$sig" -- -"$pid" 2>/dev/null || kill -"$sig" "$pid" 2>/dev/null
}

# ------------------------------------------------------------- launch ----
cmd_launch() {
  local max_speed="$MAX_SPEED" skip_confirm=""
  while (( $# )); do
    case "$1" in
      --max-speed) max_speed="$2"; shift ;;
      -y|--yes)    skip_confirm=yes ;;
      *) die "launch: unknown option: $1" ;;
    esac
    shift
  done
  export MAX_SPEED="$max_speed"

  local old; old="$(read_pid "$STACK_PID_F")"
  alive "$old" && die "a stack is already running (pid $old). ./25_drive_session.sh shutdown first."

  mkdir -p "$SESSION_DIR"
  print_env
  ensure_dirs
  check_disk 60 || die "a multi-bag driving session needs headroom; free space on the SSD first"

  [[ "$skip_confirm" == yes ]] && export CONFIRM=yes
  confirm_unsafe "Driving session at max_speed=${max_speed} m/s. Keep the deadman in hand."

  banner "launching teleop stack (local localization only, no SLAM, no Nav2)"
  # Local localization only, exactly as 30_mapping_drive.sh does it. Maps are
  # built offline from these bags, so nothing here needs a map: no map_server,
  # no AMCL, no global EKF. The EKF owns odom->base_link, so every bag carries
  # a usable odom TF and the offline SLAM run does not regenerate odometry.
  setsid nohup ros2 launch f1tenth_launch teleop.launch.py \
      use_f1tenth_namespace:=True \
      f1tenth_namespace:="$NS" \
      use_gpu:="$USE_GPU" \
      launch_vehicle:=True \
      launch_sensors:=True \
      launch_tfs:=True \
      launch_joystick:=True \
      launch_localization:=True \
      launch_local_localization:=True \
      launch_global_localization:=False \
      map_tf_publisher:=ekf \
      odom_tf_publisher:=ekf \
      localize_isaac_vslam_on_startup:="$VSLAM_LOCALIZE_ON_STARTUP" \
      launch_command_gate:=True \
      command_gate_require_heartbeat:=True \
      reset_realsense:="$RESET_REALSENSE" \
      publish_realsense_pointcloud:=True \
      max_speed:="$max_speed" \
      max_steering:="$MAX_STEERING" \
      log_level:=warn \
      > "$STACK_LOG" 2>&1 < /dev/null &
  local pid=$!
  echo "$pid" > "$STACK_PID_F"
  info "stack pid $pid  (log: $STACK_LOG)"

  wait_for_topic "$(ns_topic lidar/scan_filtered)" 90  || die "LiDAR never came up"
  wait_for_topic "$(ns_topic camera/color/image_raw)" 120 || die "camera never came up"
  wait_for_topic "$(ns_topic odometry/local)" 90        || die "EKF never produced odometry/local"
  wait_for_topic "$(ns_topic camera/depth/color/points)" 60 || \
    warn "colored pointcloud not up — check publish_realsense_pointcloud"
  sleep 5

  banner "stack up"
  info "verify with: ./25_drive_session.sh status"
  info "then record: ./25_drive_session.sh start <bag_name>"
}

# -------------------------------------------------------------- start ----
cmd_start() {
  local name="${1:-}"
  [[ -n "$name" ]] || die "usage: 25_drive_session.sh start <bag_name>"

  local rec; rec="$(read_pid "$REC_PID_F")"
  alive "$rec" && die "a recording is already running (pid $rec, $(cat "$REC_DIR_F" 2>/dev/null)). Stop it first."

  local stack; stack="$(read_pid "$STACK_PID_F")"
  alive "$stack" || warn "no stack pid on file — recording against whatever is already publishing"

  mkdir -p "$SESSION_DIR"; ensure_dirs
  check_disk 25 || die "not enough space left to start another bag"

  # `drive` is the default because bags from one session must be
  # interchangeable downstream. Override with BAG_TOPIC_SET for a run that must
  # not record images -- recording them costs VSLAM (see TOPICS_CAMERA_IMU).
  set_array "${BAG_TOPIC_SET:-drive}"
  (( ${#TOPIC_LIST[@]} )) || die "empty topic list"
  printf '%s\n' "${TOPIC_LIST[@]}" > "$REC_TOPICS_F"

  local out="${BAG_ROOT}/${name}_$(date +%H%M%S)"
  local args=(-o "$out" -s "$BAG_STORAGE")
  [[ "$BAG_COMPRESSION" != "none" ]] && \
    args+=(--compression-mode file --compression-format "$BAG_COMPRESSION")

  banner "starting bag: $name"
  printf '  %-14s %s\n' "path" "$out" "topics" "${#TOPIC_LIST[@]}"

  setsid nohup ros2 bag record "${args[@]}" "${TOPIC_LIST[@]}" \
      > "$REC_LOG" 2>&1 < /dev/null &
  local pid=$!
  echo "$pid"  > "$REC_PID_F"
  echo "$out"  > "$REC_DIR_F"

  # Never announce "recording" without proving it. A recorder that died on a
  # bad topic name, or one writing nothing because the storage plugin failed,
  # both leave a plausible-looking directory behind. The authority is bytes on
  # disk increasing while the process is alive.
  sleep 4
  alive "$pid" || { err "recorder died immediately — see $REC_LOG"; tail -20 "$REC_LOG"; return 1; }
  [[ -d "$out" ]] || { err "recorder is running but wrote no bag directory at $out"; return 1; }

  local s1 s2
  s1="$(du -sk "$out" 2>/dev/null | cut -f1)"
  sleep 4
  s2="$(du -sk "$out" 2>/dev/null | cut -f1)"
  alive "$pid" || { err "recorder exited during verification — see $REC_LOG"; tail -20 "$REC_LOG"; return 1; }

  local grew=$(( s2 - s1 ))
  if (( grew <= 0 )); then
    err "bag is NOT growing (${s1}K -> ${s2}K over 4 s). Nothing is being captured."
    tail -20 "$REC_LOG"
    return 1
  fi

  # How many of the requested topics the recorder actually latched onto. A
  # subscription count well below the request means topics are missing from the
  # graph, which is recoverable now and not after the drive.
  local subs
  subs="$(grep -c 'Subscribed to topic' "$REC_LOG" 2>/dev/null)"

  banner "RECORDING STARTED — move the robot now"
  printf '  %-14s %s\n' \
    "bag"        "$out" \
    "pid"        "$pid" \
    "growth"     "$(( grew / 4 )) KB/s ($(( grew )) KB in 4 s)" \
    "subscribed" "${subs:-?} / ${#TOPIC_LIST[@]} requested topics"
  info "stop with: ./25_drive_session.sh stop"
}

# --------------------------------------------------------------- stop ----
cmd_stop() {
  local pid dir
  pid="$(read_pid "$REC_PID_F")"
  dir="$(cat "$REC_DIR_F" 2>/dev/null)"
  [[ -n "$dir" ]] || die "no recording on file"

  if alive "$pid"; then
    banner "stopping recorder (pid $pid)"
    kill_group "$pid" INT
    local waited=0
    while alive "$pid" && (( waited < 60 )); do sleep 1; waited=$((waited+1)); done
    alive "$pid" && { warn "recorder ignored SIGINT after ${waited}s — sending TERM"; kill_group "$pid" TERM; sleep 3; }
  else
    warn "recorder pid $pid was already gone"
  fi
  : > "$REC_PID_F"

  # rosbag2 writes metadata.yaml only on a clean shutdown. Its absence is the
  # difference between "a bag" and "a directory of unusable chunks", and is
  # what silently swallowed a 120 s take on 2026-08-05.
  if [[ ! -s "$dir/metadata.yaml" ]]; then
    err "NO metadata.yaml in $dir — this bag is NOT usable. Do not treat the run as recorded."
    tail -20 "$REC_LOG"
    return 1
  fi

  banner "bag closed: $dir"
  printf '%s\n' "$dir" > "${BAG_ROOT}/.last_bag"
  summarize_bag "$dir"
}

# summarize_bag <dir> — per-topic counts and rates, with the silent topics
# called out. Reads metadata.yaml directly; `ros2 bag info` is slower and does
# not give us the rate arithmetic.
#
# Some topics in the `drive` set are silent BY DESIGN in this configuration and
# must not be reported as failures, or every clean bag looks broken:
#   drive, cmd_vel  — no Nav2 and no MPC is running, so nothing publishes them.
#                     They are recorded so the same bag stays valid once one is.
#   estop           — only carries traffic while the estop button is held.
# Everything else silent is a real defect.
SILENT_OK="drive cmd_vel estop"

summarize_bag() {
  local dir="$1"
  MIN_RATE_KV="$(for k in "${!MIN_RATE[@]}"; do printf '%s=%s\n' "$k" "${MIN_RATE[$k]}"; done)" \
  NS="$NS" REQ_FILE="$REC_TOPICS_F" BAG_DIR="$dir" SILENT_OK="$SILENT_OK" python3 - <<'PY'
import os, sys, yaml

bag = os.environ["BAG_DIR"]
ns  = os.environ["NS"]
mins = {}
for line in os.environ.get("MIN_RATE_KV", "").splitlines():
    if "=" in line:
        k, v = line.split("=", 1)
        mins["/%s/%s" % (ns, k)] = float(v)

with open(os.path.join(bag, "metadata.yaml")) as fh:
    m = yaml.safe_load(fh)["rosbag2_bagfile_information"]

dur = m["duration"]["nanoseconds"] / 1e9
got = {t["topic_metadata"]["name"]: t["message_count"]
       for t in m["topics_with_message_count"]}

req = []
try:
    with open(os.environ["REQ_FILE"]) as fh:
        req = [l.strip() for l in fh if l.strip()]
except OSError:
    req = sorted(got)

size = 0
for root, _, files in os.walk(bag):
    for f in files:
        size += os.path.getsize(os.path.join(root, f))

print("  duration   %.1f s" % dur)
print("  size       %.2f GiB   (%.0f MB/s)" % (size / 2**30, size / 1e6 / max(dur, 1e-9)))
print("  messages   %d across %d topics\n" % (m["message_count"], len(got)))

expected_silent = set("/%s/%s" % (ns, s) for s in os.environ.get("SILENT_OK", "").split())

silent, slow, ok, quiet = [], [], [], []
for t in req:
    n = got.get(t)
    if n is None or n == 0:
        why = "NOT IN BAG" if n is None else "0 messages"
        (quiet if t in expected_silent else silent).append((t, why))
    else:
        hz = n / dur if dur else 0.0
        floor = mins.get(t)
        # Latched one-shot topics (URDF, extrinsics, tf_static) legitimately
        # carry a handful of messages; never flag them as slow.
        if floor is not None and hz < floor:
            slow.append((t, n, hz, floor))
        else:
            ok.append((t, n, hz))

for t, n, hz in sorted(ok, key=lambda r: -r[2]):
    print("  [ ok ] %-58s %7d  %7.1f Hz" % (t, n, hz))
for t, n, hz, floor in slow:
    print("  [SLOW] %-58s %7d  %7.1f Hz  (want >= %.1f)" % (t, n, hz, floor))
for t, why in quiet:
    print("  [ -- ] %-58s %s (silent by design here)" % (t, why))
for t, why in silent:
    print("  [FAIL] %-58s %s" % (t, why))

extra = sorted(set(got) - set(req))
for t in extra:
    print("  [note] %-58s recorded but not requested" % t)

print()
if silent:
    print("  %d requested topic(s) captured NOTHING — see [FAIL] above." % len(silent))
    sys.exit(1)
if slow:
    print("  %d topic(s) below their expected rate floor." % len(slow))
    sys.exit(2)
print("  all requested topics present and at rate.")
PY
}

# ------------------------------------------------------------- status ----
cmd_status() {
  local stack rec dir
  stack="$(read_pid "$STACK_PID_F")"
  rec="$(read_pid "$REC_PID_F")"
  dir="$(cat "$REC_DIR_F" 2>/dev/null)"

  banner "session status"
  if alive "$stack"; then info "stack running (pid $stack)"; else err "stack NOT running"; fi
  if alive "$rec"; then
    info "RECORDING (pid $rec) -> $dir  [$(du -sh "$dir" 2>/dev/null | cut -f1)]"
  else
    printf '  not recording%s\n' "${dir:+ (last bag: $dir)}"
  fi

  banner "live rates"
  local t
  for t in lidar/scan_filtered camera/color/image_raw camera/depth/color/points \
           vehicle/sensors/imu/raw vehicle/vesc_odom odometry/local \
           ackermann_drive vehicle/ackermann_cmd; do
    printf '  %-52s %8s Hz\n' "$(ns_topic "$t")" "$(topic_hz "$(ns_topic "$t")" 4)"
  done
}

# ----------------------------------------------------------- shutdown ----
cmd_shutdown() {
  local rec stack
  rec="$(read_pid "$REC_PID_F")"
  if alive "$rec"; then
    warn "a recording is still running — closing it first"
    cmd_stop || true
  fi
  stack="$(read_pid "$STACK_PID_F")"
  if alive "$stack"; then
    banner "stopping stack (pid $stack)"
    kill_group "$stack" INT
    local waited=0
    while alive "$stack" && (( waited < 45 )); do sleep 1; waited=$((waited+1)); done
    alive "$stack" && { warn "stack ignored SIGINT — sending TERM"; kill_group "$stack" TERM; sleep 3; }
    alive "$stack" && { warn "still alive — SIGKILL"; kill_group "$stack" KILL; }
  else
    warn "no stack running"
  fi
  : > "$STACK_PID_F"
  info "session closed. Bags are under $BAG_ROOT"
}

# -------------------------------------------------------------- main ----
setup_ros
case "${1:-}" in
  launch)   shift; cmd_launch "$@" ;;
  start)    shift; cmd_start "$@" ;;
  stop)     shift; cmd_stop "$@" ;;
  status)   shift; cmd_status "$@" ;;
  shutdown) shift; cmd_shutdown "$@" ;;
  summarize) shift; summarize_bag "${1:?usage: summarize <bag dir>}" ;;
  ""|-h|--help) sed -n '2,30p' "$0" ;;
  *) die "unknown command: $1  (launch|start|stop|status|shutdown|summarize)" ;;
esac
