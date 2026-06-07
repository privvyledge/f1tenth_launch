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

  # --- Isaac ROS VSLAM ---
  ros2 service call /gosling1/visual_slam/save_map isaac_ros_visual_slam_interfaces/srv/FilePath "{file_path: /mnt/shared_dir/maps/05292026/vslam}" 

  # Export assembled 3D pointcloud from database (run after mapping, not during)

  # PREFERRED: rtabmap-databaseViewer GUI — confirmed to write a real .pcd with
  # correct colors that pcl_ros pcd_to_pointcloud opens directly (no conversion).
  #   rtabmap-databaseViewer /mnt/shared_dir/maps/05292026/rtabmap/rtabmap.db
  #   -> File -> Export 3D clouds... -> choose .pcd
  # IMPORTANT: enable voxelization (voxel size, e.g. 0.02-0.05 m) in the export
  # dialog. The raw cloud can be hundreds of thousands of points and overwhelm the
  # visualizer / RViz on constrained hardware (e.g. Jetson Orin Nano).

  # CLI alternative (rtabmap-export): --output is a BASE NAME (no extension).
  # It appends "_cloud.ply" and exports PLY ONLY — the extension in --output does
  # NOT select the format. "--output cloud.pcd" produces the misleading
  # "cloud.pcd_cloud.ply" (still PLY). Use a bare base name, then convert to PCD
  # (see Point Cloud Map Visualization below). Prefer the databaseViewer above.
  rtabmap-export --cloud \
    --output-dir /mnt/shared_dir/maps/05292026/rtabmap \
    --output cloud \
    /mnt/shared_dir/maps/05292026/rtabmap/rtabmap.db
  # -> writes /mnt/shared_dir/maps/05292026/rtabmap/cloud_cloud.ply

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

# --- Point Cloud Map Visualization (aesthetic only) ---

  # Republishes a static RTABMap-exported point cloud as a latched PointCloud2 for RViz.
  # NOT used for localization or planning — purely a 3D map overlay.
  # Output topic: map/pointcloud   Frame: map   (set RViz Fixed Frame to map)

  # Via teleop or bringup (recommended) — node is off by default
  ros2 launch f1tenth_launch teleop.launch.py \
    launch_pointcloud_map:=True launch_visualization:=True
  ros2 launch f1tenth_launch bringup.launch.py \
    launch_pointcloud_map:=True launch_visualization:=True
  # Override the file (defaults to data/maps/rtabmap/raslab/cloud.pcd in the package share):
  #   pointcloud_map_file:=/mnt/shared_dir/maps/05292026/rtabmap/cloud.pcd

  # Standalone publisher (no stack)
  ros2 run pcl_ros pcd_to_pointcloud --ros-args \
    -p file_name:=/mnt/shared_dir/maps/05292026/rtabmap/cloud.pcd \
    -p tf_frame:=map -p publishing_period_ms:=3000 \
    -r cloud_pcd:=map/pointcloud

  # IMPORTANT: pcd_to_pointcloud reads .PCD only. Feeding it the .ply export
  # ("[pcl::PCDReader::readHeader] No points to read" / "could not open pcd file")
  # fails because the PCD parser cannot read PLY.
  # Easiest fix: export the .pcd directly from rtabmap-databaseViewer (see RTABMap
  # export note above) — confirmed working, colors correct, no conversion needed.
  # If you only have the rtabmap-export .ply, convert it to PCD first: 

  # Convert PLY -> PCD (Open3D is installed in the image; preserves xyz + rgb + normals)
  python3 -c "import open3d as o3d; \
    pc = o3d.io.read_point_cloud('/mnt/shared_dir/maps/05292026/rtabmap/cloud_cloud.ply'); \
    o3d.io.write_point_cloud('/mnt/shared_dir/maps/05292026/rtabmap/cloud.pcd', pc)"
  # Then launch with pointcloud_map_file:=/mnt/shared_dir/maps/05292026/rtabmap/cloud.pcd
  # (If pcl-tools is installed, "pcl_ply2pcd cloud_cloud.ply cloud.pcd" also works.)

  # In RViz: add a PointCloud2 display, topic map/pointcloud, and set its QoS
  # Durability to "Transient Local" so it receives the latched cloud on connect.