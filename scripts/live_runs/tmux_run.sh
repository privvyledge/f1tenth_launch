#!/usr/bin/env bash
# tmux_run.sh — 4-pane layout for a live run phase.
#
#   ./tmux_run.sh sensors        [-- extra args passed to the phase script]
#   ./tmux_run.sh sensors_cloud
#   ./tmux_run.sh mapping
#   ./tmux_run.sh localization -- --map /mnt/shared_dir/maps/.../raslab.yaml
#   ./tmux_run.sh nav2         -- --map ... --dry-run
#   ./tmux_run.sh mpc          -- --map ...
#
# Layout:
#   +---------------------+---------------------+
#   | 0 stack + recorder  | 1 topic rates       |
#   +---------------------+---------------------+
#   | 2 TF / node health  | 3 free shell        |
#   +---------------------+---------------------+
#
# This only arranges panes. All the logic lives in the numbered scripts, so
# anything here can also be run by hand in four terminals.

set -uo pipefail
cd "$(dirname "$0")"
# shellcheck source=00_env.sh
source ./00_env.sh

command -v tmux >/dev/null 2>&1 || die "tmux not installed (apt install tmux)"

PHASE="${1:-}"
[[ -n "$PHASE" ]] || { sed -n '2,20p' "$0"; exit 1; }
shift
[[ "${1:-}" == "--" ]] && shift
EXTRA=("$@")

case "$PHASE" in
  sensors)       SCRIPT=./20_sensor_bag.sh;        MONITOR=sensors ;;
  sensors_cloud) SCRIPT=./20_sensor_bag.sh;        MONITOR=sensors_cloud
                 EXTRA=(--pointcloud "${EXTRA[@]}") ;;
  mapping)       SCRIPT=./30_mapping_drive.sh;     MONITOR=mapping ;;
  localization)  SCRIPT=./50_localization_test.sh; MONITOR=localization ;;
  nav2)          SCRIPT=./60_nav2_test.sh;         MONITOR=nav2 ;;
  mpc)           SCRIPT=./70_mpc_handover.sh;      MONITOR=mpc ;;
  *) die "unknown phase '$PHASE' (sensors|sensors_cloud|mapping|localization|nav2|mpc)" ;;
esac

SESSION="f1tenth_${PHASE}"
tmux has-session -t "$SESSION" 2>/dev/null && \
  die "session '$SESSION' already exists. Attach with: tmux attach -t $SESSION
       (or kill it: tmux kill-session -t $SESSION)"

# Environment the panes inherit. Each pane re-sources 00_env.sh anyway, but
# exporting the overrides keeps them consistent with this invocation.
ENVSET="export NS='$NS' ROS_DOMAIN_ID='$ROS_DOMAIN_ID' USE_GPU='$USE_GPU'"
ENVSET+=" MAX_SPEED='$MAX_SPEED' BAG_ROOT='$BAG_ROOT' MAP_ROOT='$MAP_ROOT'"
ENVSET+=" VSLAM_LOCALIZE_ON_STARTUP='$VSLAM_LOCALIZE_ON_STARTUP'"

banner "starting tmux session '$SESSION'"

# Pane 0 — the stack and the recorder.
tmux new-session -d -s "$SESSION" -n run -c "$LIVE_RUNS_DIR"
tmux send-keys -t "$SESSION:run.0" \
  "$ENVSET; $SCRIPT ${EXTRA[*]}" C-m

# Pane 1 — live rates on the phase's topic set. Waits for the stack first.
tmux split-window -h -t "$SESSION:run" -c "$LIVE_RUNS_DIR"
tmux send-keys -t "$SESSION:run.1" \
  "$ENVSET; source ./00_env.sh; source ./topic_sets.sh; \
   echo 'waiting for the stack...'; \
   wait_for_topic \$(ns_topic lidar/scan_filtered) 180 && \
   set_array $MONITOR && watch -n2 --no-title \
     \"ros2 topic hz \${TOPIC_LIST[*]} --window 20 2>/dev/null | tail -40\"" C-m

# Pane 2 — TF and node health, refreshed slowly.
tmux split-window -v -t "$SESSION:run.0" -c "$LIVE_RUNS_DIR"
tmux send-keys -t "$SESSION:run.2" \
  "$ENVSET; source ./00_env.sh; \
   echo 'TF / node health — refreshes every 10s'; sleep 25; \
   watch -n10 --no-title \
     \"echo '--- nodes ---'; ros2 node list 2>/dev/null | sort | uniq -c | sort -rn | head -20; \
       echo; echo '--- containers ---'; ros2 node list 2>/dev/null | grep -i container; \
       echo; echo '--- tf publishers ---'; ros2 topic info \$(ns_topic tf) 2>/dev/null\"" C-m

# Pane 3 — free shell, pre-sourced, for service calls and map saving.
tmux split-window -v -t "$SESSION:run.1" -c "$LIVE_RUNS_DIR"
tmux send-keys -t "$SESSION:run.3" \
  "$ENVSET; source ./00_env.sh; source ./topic_sets.sh; clear; \
   echo 'Free shell. Useful:'; \
   echo '  ./41_save_map.sh --mode all'; \
   echo '  ./10_preflight.sh $MONITOR'; \
   echo '  ros2 run tf2_tools view_frames --ros-args -r __ns:=/$NS'; \
   echo '  ros2 param get \$(ns_topic amcl) tf_broadcast'" C-m

tmux select-pane -t "$SESSION:run.0"
tmux attach -t "$SESSION"
