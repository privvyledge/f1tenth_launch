#!/usr/bin/env bash
# topic_sets.sh — the single source of truth for which topics each phase
# records and verifies. Sourced after 00_env.sh (needs $NS / ns_topic).
#
# Every name here was traced to the launch file that publishes it, not to
# documentation. Key derivations:
#   lidar/*        ydlidar.launch.py pushes namespace 'lidar' (not a remap)
#   vehicle/*      vehicle.launch.py pushes namespace 'vehicle'
#   odometry/local ekf_odom.launch.py remaps odometry/filtered -> odometry/local
#   odometry/global ekf_map.launch.py  remaps odometry/filtered -> odometry/global
#   ackermann chain  teleop|drive|estop|safety -> ackermann_drive
#                    -> vehicle/ackermann_cmd -> vehicle/commands/*

# --------------------------------------------------------------- helpers ----
# _ns expands a list of relative topics into absolute namespaced ones.
_ns() { local t; for t in "$@"; do printf '/%s/%s\n' "$NS" "${t#/}"; done; }

# ------------------------------------------------------------------- TF ----
TOPICS_TF=$(_ns tf tf_static)

# --------------------------------------------------------------- LiDAR ----
TOPICS_LIDAR=$(_ns \
  lidar/scan \
  lidar/scan_filtered)

# -------------------------------------------------------------- camera ----
# Raw streams only. The colored pointcloud is opt-in (TOPICS_CAMERA_CLOUD)
# because publish_realsense_pointcloud defaults False and the cloud is by far
# the largest topic on the vehicle.
TOPICS_CAMERA=$(_ns \
  camera/color/image_raw \
  camera/color/camera_info \
  camera/aligned_depth_to_color/image_raw \
  camera/aligned_depth_to_color/camera_info \
  camera/infra1/image_rect_raw \
  camera/infra1/camera_info \
  camera/infra2/image_rect_raw \
  camera/infra2/camera_info \
  camera/imu \
  camera/imu/filtered)

TOPICS_CAMERA_CLOUD=$(_ns camera/depth/color/points)

# Everything a consumer needs to re-derive geometry offline: the unaligned
# depth stream (aligned_depth_to_color is resampled into the colour intrinsics
# and loses the native depth FOV), and the latched inter-stream extrinsics the
# RealSense driver publishes once at startup. Only worth recording for the
# detection dataset; the phase bags don't need them.
TOPICS_CAMERA_CALIB=$(_ns \
  camera/depth/image_rect_raw \
  camera/depth/camera_info \
  camera/extrinsics/depth_to_color \
  camera/extrinsics/depth_to_infra1 \
  camera/extrinsics/depth_to_infra2)

# Latched URDF. Lets a downstream repo rebuild the full kinematic chain (and
# every camera_link -> optical_frame hop) without this package checked out.
TOPICS_DESCRIPTION=$(_ns robot_description)

# ------------------------------------------------------- vehicle / VESC ----
TOPICS_VEHICLE=$(_ns \
  vehicle/sensors/imu/raw \
  vehicle/sensors/imu/data \
  vehicle/sensors/core \
  vehicle/vesc_odom)

# The full actuation chain, in order. Recording all four links is what lets you
# prove after the fact where a command was dropped.
TOPICS_ACTUATOR=$(_ns \
  teleop \
  drive \
  estop \
  safety \
  ackermann_drive \
  vehicle/ackermann_cmd \
  vehicle/commands/motor/speed \
  vehicle/commands/servo/position \
  cmd_vel)

# ------------------------------------------------------------ odometry ----
TOPICS_ODOM_LOCAL=$(_ns \
  odometry/local \
  odom/rf2o)

TOPICS_ODOM_VSLAM=$(_ns \
  visual_slam/tracking/odometry \
  visual_slam/vis/slam_odometry)

# ------------------------------------------------- global localization ----
# The set specifically called out for verification after mapping.
TOPICS_GLOBAL_LOC=$(_ns \
  map \
  amcl_pose \
  particle_cloud \
  odometry/global \
  initialpose)

# ---------------------------------------------------------------- nav2 ----
TOPICS_NAV2=$(_ns \
  plan \
  local_plan \
  global_costmap/costmap \
  local_costmap/costmap \
  goal_pose \
  cmd_vel_nav \
  cmd_vel_smoothed)

# --------------------------------------------------------------- SLAM ----
TOPICS_SLAM_2D=$(_ns \
  map \
  slam_toolbox/graph_visualization)

TOPICS_RTABMAP=$(_ns \
  rtabmap/grid_map \
  rtabmap/grid_prob_map \
  rtabmap/cloud_map \
  rtabmap/mapData \
  rtabmap/localization_pose)

# ------------------------------------------------------------ bundles ----
# set_for <phase> — echoes the newline-separated topic list for a phase.
set_for() {
  case "$1" in
    sensors)
      printf '%s\n' "$TOPICS_TF" "$TOPICS_LIDAR" "$TOPICS_CAMERA" \
                    "$TOPICS_VEHICLE" "$TOPICS_ACTUATOR" "$TOPICS_ODOM_LOCAL" ;;
    sensors_cloud)
      printf '%s\n' "$TOPICS_TF" "$TOPICS_LIDAR" "$TOPICS_CAMERA" \
                    "$TOPICS_CAMERA_CLOUD" "$TOPICS_VEHICLE" \
                    "$TOPICS_ACTUATOR" "$TOPICS_ODOM_LOCAL" ;;
    detection)
      # Perception-only dataset for the downstream detection/segmentation
      # repos. Deliberately omits TOPICS_VEHICLE and TOPICS_ACTUATOR: this set
      # is recorded with the VESC disconnected, so those topics never appear
      # and listing them would only manufacture preflight failures. VSLAM odom
      # is included only on the GPU path, where it actually publishes.
      printf '%s\n' "$TOPICS_TF" "$TOPICS_LIDAR" "$TOPICS_CAMERA" \
                    "$TOPICS_CAMERA_CLOUD" "$TOPICS_CAMERA_CALIB" \
                    "$TOPICS_DESCRIPTION" "$TOPICS_ODOM_LOCAL"
      if [[ "${USE_GPU,,}" == "true" ]]; then
        printf '%s\n' "$TOPICS_ODOM_VSLAM"
      fi
      # Added only when the VESC is actually connected and the vehicle stack is
      # launched. TOPICS_ACTUATOR stays out either way: with the command_gate
      # disabled nothing publishes a drive command, so those topics would be
      # silent by construction.
      if [[ "${DETECTION_WITH_VEHICLE:-false}" == "true" ]]; then
        printf '%s\n' "$TOPICS_VEHICLE"
      fi
      ;;
    mapping)
      printf '%s\n' "$TOPICS_TF" "$TOPICS_LIDAR" "$TOPICS_CAMERA" \
                    "$TOPICS_VEHICLE" "$TOPICS_ACTUATOR" \
                    "$TOPICS_ODOM_LOCAL" "$TOPICS_ODOM_VSLAM" ;;
    localization)
      printf '%s\n' "$TOPICS_TF" "$TOPICS_LIDAR" "$TOPICS_VEHICLE" \
                    "$TOPICS_ACTUATOR" "$TOPICS_ODOM_LOCAL" \
                    "$TOPICS_ODOM_VSLAM" "$TOPICS_GLOBAL_LOC" ;;
    nav2)
      printf '%s\n' "$TOPICS_TF" "$TOPICS_LIDAR" "$TOPICS_VEHICLE" \
                    "$TOPICS_ACTUATOR" "$TOPICS_ODOM_LOCAL" \
                    "$TOPICS_GLOBAL_LOC" "$TOPICS_NAV2" ;;
    mpc)
      printf '%s\n' "$TOPICS_TF" "$TOPICS_LIDAR" "$TOPICS_VEHICLE" \
                    "$TOPICS_ACTUATOR" "$TOPICS_ODOM_LOCAL" \
                    "$TOPICS_GLOBAL_LOC" ;;
    *) err "unknown topic set: $1"; return 1 ;;
  esac
}

# set_array <phase> — populates the global array TOPIC_LIST, blank lines
# stripped, for passing to `ros2 bag record`.
set_array() {
  mapfile -t TOPIC_LIST < <(set_for "$1" | sed '/^[[:space:]]*$/d')
}

# ------------------------------------------ expected minimum rates (Hz) ----
# Used by 10_preflight.sh and the in-run monitors. Values are conservative
# floors observed on the Jetson Orin, not nominal spec rates: the YDLidar X4 is
# configured for 12 Hz but delivers ~8 Hz under USB/CPU load, and the RealSense
# colour stream runs ~36 Hz against a nominal 30.
declare -A MIN_RATE=(
  ["lidar/scan"]=5
  ["lidar/scan_filtered"]=5
  ["camera/color/image_raw"]=10
  ["camera/aligned_depth_to_color/image_raw"]=10
  ["camera/infra1/image_rect_raw"]=10
  ["camera/infra2/image_rect_raw"]=10
  ["camera/depth/image_rect_raw"]=10
  ["camera/depth/color/points"]=5
  ["camera/imu/filtered"]=50
  ["vehicle/sensors/imu/raw"]=50
  ["vehicle/vesc_odom"]=20
  ["odometry/local"]=20
  ["odom/rf2o"]=5
  ["visual_slam/tracking/odometry"]=15
  ["ackermann_drive"]=20
  ["vehicle/ackermann_cmd"]=20
  ["amcl_pose"]=0.5
  ["particle_cloud"]=0.5
  ["odometry/global"]=5
)
