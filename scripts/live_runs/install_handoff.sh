#!/usr/bin/env bash
# install_handoff.sh — install the 2026-08-05 live-run fixes into the workspace
# of a FRESH container, without needing network access on the Jetson.
#
# Run this INSIDE the container:
#   bash /mnt/shared_dir/handoff/live_runs_20260805/install_handoff.sh
#
# Why this exists instead of `git pull`: the Jetson has no internet, and
# /workspaces/f1tenth lives in the container filesystem (it is NOT a bind
# mount), so a new container from a new image starts with a stale workspace.
# /mnt/shared_dir IS bind-mounted from the host SSD, so staging the files there
# makes them visible to any container immediately.

set -uo pipefail

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DST="${DST:-/workspaces/f1tenth/src/f1tenth_launch}"

[[ -d "$DST" ]] || { echo "[FAIL] no workspace at $DST"; exit 1; }

# md5 of each file as shipped on 2026-08-05. Verified after copying, because a
# silently truncated or stale copy is exactly the failure this script exists to
# prevent.
declare -A WANT=(
  ["scripts/live_runs/25_drive_session.sh"]=48b53e9e82938d68a895bbe613acfba1
  ["scripts/live_runs/topic_sets.sh"]=33b1c05cf18afd2c53f993cdcc693b73
  ["scripts/live_runs/00_env.sh"]=e85359d68752fd15ba845d9004815034
  ["launch/vehicle/vehicle.launch.py"]=415dd8e8c439ceec216e1e8a42c4d16b
)

DOCS=(
  "scripts/live_runs/DRIVE_SESSION_HANDOFF.md"
  "scripts/live_runs/CYCLONEDDS_PEERS.md"
  "scripts/live_runs/DATASET_HANDOFF.md"
)

STAMP="$(date +%Y%m%d_%H%M%S)"
rc=0

install_one() {
  local rel="$1" from="$SRC/$rel" to="$DST/$rel"
  [[ -f "$from" ]] || { echo "[FAIL] staged file missing: $from"; rc=1; return; }
  mkdir -p "$(dirname "$to")"
  # Keep a backup of whatever the image shipped, so a bad install is reversible.
  [[ -f "$to" ]] && cp -p "$to" "$to.bak_$STAMP"
  cp -f "$from" "$to"
}

for rel in "${!WANT[@]}"; do install_one "$rel"; done
for rel in "${DOCS[@]}";   do install_one "$rel"; done

chmod +x "$DST/scripts/live_runs/"*.sh 2>/dev/null

echo
echo "=== verifying checksums"
for rel in "${!WANT[@]}"; do
  got="$(md5sum "$DST/$rel" 2>/dev/null | cut -d' ' -f1)"
  if [[ "$got" == "${WANT[$rel]}" ]]; then
    printf '  [ ok ] %s\n' "$rel"
  else
    printf '  [FAIL] %s\n         want %s\n         got  %s\n' "$rel" "${WANT[$rel]}" "${got:-<missing>}"
    rc=1
  fi
done

echo
echo "=== confirming the two behavioural changes are live"
grep -q 'respawn_delay=2.0' "$DST/launch/vehicle/vehicle.launch.py" \
  && echo "  [ ok ] vesc_driver respawn_delay = 2.0 s (was 10.0)" \
  || { echo "  [FAIL] respawn_delay not applied"; rc=1; }
grep -q 'MAX_STEERING:-0.314' "$DST/scripts/live_runs/00_env.sh" \
  && echo "  [ ok ] MAX_STEERING default = 0.314 rad (was 0.25, derived from the old -1.4 gain)" \
  || { echo "  [FAIL] MAX_STEERING not applied"; rc=1; }
grep -q 'vehicle/sensors/imu/mag' "$DST/scripts/live_runs/topic_sets.sh" \
  && echo "  [ ok ] TOPICS_VEHICLE includes imu + imu/mag" \
  || { echo "  [FAIL] topic_sets.sh not applied"; rc=1; }

# install/ is symlinked to src/ under --symlink-install, so launch and YAML
# edits need no rebuild. Verify that assumption rather than trusting it: if the
# image was built WITHOUT --symlink-install, the respawn fix will not take
# effect and the dead-stick window stays at 10.7 s.
echo
echo "=== checking --symlink-install assumption"
INST=/workspaces/f1tenth/install/f1tenth_launch/share/f1tenth_launch/launch/vehicle/vehicle.launch.py
if [[ -L "$INST" ]]; then
  echo "  [ ok ] install/ is symlinked to src/ — no rebuild needed"
elif [[ -f "$INST" ]]; then
  echo "  [warn] install/ holds a REAL COPY, not a symlink."
  echo "         This image was not built with --symlink-install. You must rebuild:"
  echo "           colcon build --symlink-install --packages-up-to f1tenth_launch"
  rc=1
else
  echo "  [warn] $INST not found — is f1tenth_launch built in this image?"
  rc=1
fi

echo
if (( rc == 0 )); then
  echo "install OK. Next: read $DST/scripts/live_runs/DRIVE_SESSION_HANDOFF.md"
else
  echo "install FINISHED WITH PROBLEMS — do not record until they are resolved."
fi
exit $rc
