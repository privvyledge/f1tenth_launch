 Mapping Run Commands

  # --- 2D SLAM Toolbox ---

  # Online (build map while driving)
  ros2 launch f1tenth_launch mapping.launch.py \
    launch_2d_mapping:=True launch_3d_mapping:=False \
    launch_localization:=True

  # Offline from rosbag
  ros2 launch f1tenth_launch mapping.launch.py \
    launch_2d_mapping:=True launch_3d_mapping:=False \
    use_sim_time:=True
  # In a second terminal:
  ros2 bag play <path/to/bag> --clock

  # --- 3D RTABMap (CPU, default) ---

  # Online, RGB-D mode (default)
  ros2 launch f1tenth_launch mapping.launch.py \
    launch_3d_mapping:=True use_gpu:=False \
    launch_localization:=True

  # Online, stereo IR mode (IR emitter must be OFF)
  ros2 launch f1tenth_launch mapping.launch.py \
    launch_3d_mapping:=True use_gpu:=False \
    launch_localization:=True
  # Pass use_stereo:=True to 3d_mapping.launch.py — needs a bringup arg or direct launch

  # Offline from rosbag, CPU
  ros2 launch f1tenth_launch mapping.launch.py \
    launch_3d_mapping:=True use_gpu:=False \
    use_sim_time:=True
  # mapping_3d_queue_size auto-set to 10000 when use_sim_time=True

  # Continue existing map (life_long_mapping)
  ros2 launch f1tenth_launch mapping.launch.py \
    launch_3d_mapping:=True use_gpu:=False \
    life_long_mapping:=True launch_localization:=True

  # --- 3D Nvblox (GPU, Jetson/RTX only) ---

  # Online, GPU
  ros2 launch f1tenth_launch mapping.launch.py \
    launch_3d_mapping:=True use_gpu:=True \
    launch_localization:=True

  # Via bringup (note: bringup defaults use_gpu:=True)
  ros2 launch f1tenth_launch bringup.launch.py \
    slam:=True launch_navigation:=False use_gpu:=True

  # --- Via bringup (unified) ---

  # 2D mapping, no navigation
  ros2 launch f1tenth_launch bringup.launch.py \
    slam:=True launch_2d_mapping:=True launch_3d_mapping:=False \
    launch_navigation:=False use_gpu:=False

  # 3D mapping, CPU
  ros2 launch f1tenth_launch bringup.launch.py \
    slam:=True launch_3d_mapping:=True \
    launch_navigation:=False use_gpu:=False

# --- Map Saving ---

  # --- SLAM Toolbox ---

  # Save occupancy grid (.pgm + .yaml) — use /gosling1/map if namespaced
  ros2 run nav2_map_server map_saver_cli \
    -t /gosling1/map \
    -f /mnt/shared_dir/maps/05292026/slam_toolbox/raslab \
    --ros-args -p map_subscribe_transient_local:=true

  # Save pose graph (.posegraph + .data) — required for slam_toolbox localization mode
  ros2 service call /gosling1/slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '/mnt/shared_dir/maps/05292026/slam_toolbox/raslab'}"

  # --- RTABMap ---

  # Save 2D occupancy grid (.pgm + .yaml) from grid_prob_map (laser-based, Grid/Sensor 0)
  ros2 run nav2_map_server map_saver_cli \
    -t /gosling1/grid_prob_map \
    -f /mnt/shared_dir/maps/05292026/rtabmap/raslab \
    --ros-args -p map_subscribe_transient_local:=true

  # Export assembled 3D pointcloud from database (run after mapping, not during)
  # --output-dir sets the directory; --output sets the filename (extension determines format)
  rtabmap-export --cloud \
    --output-dir /mnt/shared_dir/maps/05292026/rtabmap \
    --output cloud.pcd \
    /mnt/shared_dir/maps/05292026/rtabmap/rtabmap.db

  rtabmap-export --cloud \
    --output-dir /mnt/shared_dir/maps/05292026/rtabmap \
    --output cloud.ply \
    /mnt/shared_dir/maps/05292026/rtabmap/rtabmap.db

  # --- Nvblox (GPU) ---

  # Save nvblox voxel map (.nvblx binary) — call before killing the launch
  # Nvblox appends .nvblx automatically; omit the extension in the path argument.
  ros2 service call /nvblox_node/save_map nvblox_msgs/srv/FilePath \
    "{file_path: '/mnt/shared_dir/maps/05292026/nvblox/raslab'}"

  # Load a previously saved .nvblx map back into a running nvblox node
  ros2 service call /nvblox_node/load_map nvblox_msgs/srv/FilePath \
    "{file_path: '/mnt/shared_dir/maps/05292026/nvblox/raslab.nvblx'}"

  # Save 2D occupancy grid slice (.pgm + .yaml) for use with nav2_map_server
  # nvblox publishes static_map_slice as a nav_msgs/OccupancyGrid while running.
  ros2 run nav2_map_server map_saver_cli \
    -t /nvblox_node/static_map_slice \
    -f /mnt/shared_dir/maps/05292026/nvblox/raslab \
    --ros-args -p map_subscribe_transient_local:=true

  # Nav2 live integration (preferred over loading a static map):
  # Use the nvblox_nav2 costmap plugin so Nav2 consumes the live ESDF directly.
  # Add nvblox_nav2::NvbloxCostmapLayer to the costmap plugins in nav2_params.yaml
  # instead of (or alongside) the static map layer.

  # If namespaced (use_f1tenth_namespace:=True), prefix service names:
  #   /gosling1/nvblox_node/save_map, /gosling1/nvblox_node/static_map_slice, etc.