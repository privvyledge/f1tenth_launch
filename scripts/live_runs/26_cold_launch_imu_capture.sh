#!/usr/bin/env bash
# run_0826.sh <runname> [seconds]  -- run INSIDE the container.
#
# One cold launch, recorder started FIRST (that is what captured the bug-244
# onset on 2026-08-25), recording both the RAW and the FILTERED RealSense IMU so
# the same bag can answer bug-244 (does ekf_odom still run away?) and bug-248
# (does the 82 deg attitude jump originate in the sensor or in madgwick?).
#
# Preconditions, or the run is void:
#   * joystick DISCONNECTED  -> command_gate closed -> vehicle/vesc_odom silent.
#     That is the arm bug-244 diverged in. Do NOT use
#     command_gate_require_heartbeat:=False -- that opens the gate (bug: run11).
#   * car parked, wheels clear.
set -o pipefail   # NOT set -u: /opt/ros/humble/setup.bash reads an unbound AMENT_TRACE_SETUP_FILES

NAME="${1:?usage: run_0826.sh <runname> [seconds]}"
DUR="${2:-240}"
OUT=/mnt/shared_dir/claude_bringup_0826
mkdir -p "$OUT"
cd "$OUT"

source /opt/ros/humble/setup.bash
source /workspaces/f1tenth/install/setup.bash

TOPICS=(
  /camera/imu                       # RAW united RealSense IMU (unite_imu_method: 2) -- bug-248
  /camera/gyro/sample /camera/accel/sample   # raw split streams, if the driver publishes them
  /camera/imu/filtered              # madgwick output = ekf_odom imu1
  /vehicle/sensors/imu/raw          # VESC IMU = imu0
  /odometry/local /odometry/global
  /vehicle/vesc_odom /odom/rf2o /visual_slam/tracking/odometry
  /tf /tf_static /amcl_pose /initialpose
)

echo "=== $NAME: recorder first, then launch, ${DUR}s ==="
ros2 bag record -o "$OUT/$NAME" "${TOPICS[@]}" > "$OUT/bagrec_$NAME.log" 2>&1 &
REC=$!
sleep 4
if ! kill -0 $REC 2>/dev/null; then echo "recorder died -- see bagrec_$NAME.log"; exit 1; fi

ROS_LOG_DIR="$OUT/roslogs" setsid ros2 launch f1tenth_launch bringup.launch.py \
    launch_visualization:=True > "$OUT/launch_$NAME.log" 2>&1 &
LAUNCH=$!
echo "launch pid $LAUNCH, recorder pid $REC; running ${DUR}s"

sleep "$DUR"

# SIGINT the whole launch process GROUP: SIGKILL on the parent orphans every node.
kill -INT -- -"$LAUNCH" 2>/dev/null
for i in $(seq 1 30); do kill -0 $LAUNCH 2>/dev/null || break; sleep 1; done
kill -0 $LAUNCH 2>/dev/null && { echo "launch still up after 30s, TERM group"; kill -TERM -- -"$LAUNCH"; sleep 5; }

kill -INT $REC 2>/dev/null
for i in $(seq 1 20); do kill -0 $REC 2>/dev/null || break; sleep 1; done

echo "=== $NAME done. bag:"
du -sh "$OUT/$NAME" 2>/dev/null
ros2 bag info "$OUT/$NAME" 2>/dev/null | sed -n '/Topic information/,$p'
echo "=== leftover ros nodes (should be empty):"
pgrep -af 'component_container|ros2 launch|rviz2' | grep -v run_0826 || echo "  none"
