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