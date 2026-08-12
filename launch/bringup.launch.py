"""
Todo:
    * make this the main entry point
        * update and test namespaces and composition
        * cleanup
        * include mapping and optionally teleop launch
"""
import os
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, SetRemap, PushRosNamespace, SetParametersFromFile, SetParameter
from launch_ros.descriptions import ParameterFile
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression, \
    EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, GroupAction, \
    OpaqueFunction, SetEnvironmentVariable, LogInfo
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml, ReplaceString


def launch_setup(context, *args, **kwargs):
    qos_str_to_rtabmap_int = {
        'SENSOR_DATA': 2,
        'SYSTEM_DEFAULT': 0,
        'DEFAULT': 1,
    }

    # Get package directories
    f1tenth_launch_dir = get_package_share_directory('f1tenth_launch')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Get launch directories
    f1tenth_launch_bringup_dir = os.path.join(f1tenth_launch_dir, 'launch')
    nav2_bringup_launch_dir = os.path.join(nav2_bringup_dir, 'launch')
    vehicle_include_dir = os.path.join(f1tenth_launch_bringup_dir, 'vehicle')
    sensor_include_dir = os.path.join(f1tenth_launch_bringup_dir, 'sensors')
    localization_include_dir = os.path.join(f1tenth_launch_bringup_dir, 'localization')

    # Setup default directories
    nav2_params_file_path = os.path.join(f1tenth_launch_dir, 'config', 'nav2_params.yaml')
    localization_params_file_path = os.path.join(f1tenth_launch_dir, 'config', 'localization', 'localizer_amcl.yaml')
    # Must come from the same RTABMap database as pointcloud_map_file below (rtabmap_final_nf.db),
    # and it is the grid the localizer_amcl.yaml initial_pose seed was measured against. The former
    # default, data/maps/raslab.yaml, was an older map with a different origin (bug-237).
    map_file_path = os.path.join(f1tenth_launch_dir, 'data', 'maps', '20260805', 'rtabmap_2d_final.yaml')
    rviz_config_path = os.path.join(f1tenth_launch_dir, 'config', 'f1tenth.rviz')
    offline_mapping_2d_param_file_path = os.path.join(f1tenth_launch_dir, "config/mapping/2d_mapping_offline.yaml")
    online_mapping_2d_param_file_path = os.path.join(f1tenth_launch_dir, "config/mapping/2d_mapping_online.yaml")
    default_2d_map_file_path = os.path.join(f1tenth_launch_dir, 'data', 'maps', '20260805', 'rtabmap_2d_final.yaml')
    rtabmap_database_file_path = os.path.join(f1tenth_launch_dir, 'data', 'maps', 'rtabmap', 'rtabmap.db')

    # Setup launch configuration variables
    f1tenth_namespace = LaunchConfiguration('f1tenth_namespace',
                                            default='')  # used to distinguish between multiple F1/10s
    use_f1tenth_namespace = LaunchConfiguration('use_f1tenth_namespace', default=False)
    namespace = LaunchConfiguration('namespace')  # todo: remove from here and nested launch files
    use_namespace = LaunchConfiguration('use_namespace', default=False)  # todo: remove from here and nested launch files
    slam = LaunchConfiguration('slam', default=False)
    map_file = LaunchConfiguration('map_file', default=map_file_path)
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    use_gpu = LaunchConfiguration('use_gpu', default=True)  # True=Nvblox (GPU), False=RTABMap (CPU); mapping.launch.py defaults to False
    params_file = LaunchConfiguration('params_file',
                                      default=nav2_params_file_path)
    autostart = LaunchConfiguration('autostart', default='True')
    use_composition = LaunchConfiguration('use_composition', default='True')
    use_respawn = LaunchConfiguration('use_respawn', default='True')
    log_level = LaunchConfiguration('log_level')

    launch_joystick = LaunchConfiguration('launch_joystick', default=True)
    launch_sensors = LaunchConfiguration('launch_sensors', default=True)
    launch_vehicle = LaunchConfiguration('launch_vehicle', default=True)
    launch_tfs = LaunchConfiguration('launch_tfs', default=True)
    launch_localization = LaunchConfiguration('launch_localization', default=True)
    launch_local_localization = LaunchConfiguration('launch_local_localization', default=True)
    launch_global_localization = LaunchConfiguration('launch_global_localization', default=True)
    # Was default=True until 2026-08-11, which silently defeated the ekf_map guard: the guard
    # (localization.launch.py) is computed as use_gpu AND localize_on_startup, so with both True
    # ekf_map fused visual_slam/vis/slam_odometry as an ABSOLUTE map anchor while the VSLAM node
    # itself ran unlocalized from its own power-up origin — 72.3 deg of map->odom error, with the
    # car, footprint, live scan and 3D cloud all rotated inside a perfectly good map and every node
    # reporting healthy (bug-232). Every documented launch path was passing :=False by hand, the
    # description below already claimed False, and localization.launch.py / mapping.launch.py
    # already default False. Set True only with a saved VSLAM map that is co-registered with the
    # nav map.
    localize_isaac_vslam_on_startup = LaunchConfiguration('localize_isaac_vslam_on_startup', default=False)
    launch_map_server = LaunchConfiguration('launch_map_server', default=True)
    # RTABMap ICP LiDAR odometry: expensive, and rf2o already covers this. Was previously hardcoded
    # 'True' at the localization include, which made it un-overridable from the command line.
    launch_icp_odometry = LaunchConfiguration('launch_icp_odometry', default='False')
    launch_navigation = LaunchConfiguration('launch_navigation', default=True)
    launch_controller_server = LaunchConfiguration('launch_controller_server', default=True)
    launch_smoother_server = LaunchConfiguration('launch_smoother_server', default=True)
    launch_planner_server = LaunchConfiguration('launch_planner_server', default=True)
    launch_behavior_server = LaunchConfiguration('launch_behavior_server', default=True)
    launch_bt_navigator = LaunchConfiguration('launch_bt_navigator', default=True)
    launch_waypoint_follower = LaunchConfiguration('launch_waypoint_follower', default=True)
    launch_velocity_smoother = LaunchConfiguration('launch_velocity_smoother', default=True)
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic', default='cmd_vel')
    odom_tf_publisher = LaunchConfiguration('odom_tf_publisher', default='ekf')
    map_tf_publisher = LaunchConfiguration('map_tf_publisher', default='ekf')
    launch_visualization = LaunchConfiguration('launch_visualization', default=False)
    rviz_config_file = LaunchConfiguration('rviz_config_file', default=rviz_config_path)
    # The cloud is only ever looked at, so follow launch_visualization by default rather than
    # republishing 1.5 MB every 3 s into a stack nobody is watching. Still overridable on its own.
    launch_pointcloud_map = LaunchConfiguration('launch_pointcloud_map', default=launch_visualization)
    pointcloud_map_file = LaunchConfiguration('pointcloud_map_file', default=os.path.join(
            f1tenth_launch_dir, 'data', 'maps', '20260805', 'cloud_voxel_0p05.pcd'))
    launch_2d_mapping = LaunchConfiguration('launch_2d_mapping', default=False)
    launch_3d_mapping = LaunchConfiguration('launch_3d_mapping', default=False)
    life_long_mapping = LaunchConfiguration('life_long_mapping', default=False)
    offline_mapping_2d_param_file = LaunchConfiguration('offline_mapping_2d_param_file',
                                                        default=offline_mapping_2d_param_file_path)
    online_mapping_2d_param_file = LaunchConfiguration('online_mapping_2d_param_file',
                                                       default=online_mapping_2d_param_file_path)
    map_2d_file = LaunchConfiguration('default_2d_map_file', default=default_2d_map_file_path)
    rtabmap_database_file = LaunchConfiguration('rtabmap_database_file', default=rtabmap_database_file_path)

    require_deadman = LaunchConfiguration('require_deadman', default='True')
    deadman_buttons = LaunchConfiguration('deadman_buttons', default="[4, 9]")
    autonomous_deadman_buttons = LaunchConfiguration('autonomous_deadman_buttons', default="[10]")  # SDL DualSense: R1=10 (PS/guide=5 — never use 5, it's the power-off button)
    steering_button = LaunchConfiguration('steering_button', default=2)
    max_speed = LaunchConfiguration('max_speed', default=5.0)
    max_steering = LaunchConfiguration('max_steering', default=0.34)
    max_acceleration = LaunchConfiguration('max_acceleration', default=2.5)
    max_steering_rate = LaunchConfiguration('max_steering_rate', default=3.2)
    vesc_poll_rate = LaunchConfiguration('vesc_poll_rate', default=50.0)
    vesc_imu_poll_rate = LaunchConfiguration('vesc_imu_poll_rate', default=100.0)
    use_imu_yaw_rate = LaunchConfiguration('use_imu_yaw_rate', default='False')
    use_closed_loop_speed = LaunchConfiguration('use_closed_loop_speed', default='False')
    speed_kp = LaunchConfiguration('speed_kp', default=0.0)
    speed_ki = LaunchConfiguration('speed_ki', default=0.0)
    speed_anti_windup = LaunchConfiguration('speed_anti_windup', default=1000.0)
    use_adaptive_ff = LaunchConfiguration('use_adaptive_ff', default='False')
    adaptive_ff_alpha = LaunchConfiguration('adaptive_ff_alpha', default=0.95)
    adaptive_ff_gain_min = LaunchConfiguration('adaptive_ff_gain_min', default=2307.0)
    adaptive_ff_gain_max = LaunchConfiguration('adaptive_ff_gain_max', default=9228.0)
    vesc_max_speed = LaunchConfiguration('vesc_max_speed', default=0.0)
    vesc_max_steering_angle = LaunchConfiguration('vesc_max_steering_angle', default=0.0)
    use_accel_ff = LaunchConfiguration('use_accel_ff', default='False')
    accel_to_erpm_gain = LaunchConfiguration('accel_to_erpm_gain', default=0.0)
    use_cmd_accel_rate_limit = LaunchConfiguration('use_cmd_accel_rate_limit', default='False')
    launch_ackermann_to_vesc_node = LaunchConfiguration('launch_ackermann_to_vesc_node', default='True')
    launch_vesc_to_odom_node = LaunchConfiguration('launch_vesc_to_odom_node', default='True')
    launch_throttle_interpolator_node = LaunchConfiguration('launch_throttle_interpolator_node', default='False')
    launch_ackermann_to_twist = LaunchConfiguration('launch_ackermann_to_twist', default='True')
    launch_command_gate = LaunchConfiguration('launch_command_gate', default=True)
    command_gate_require_heartbeat = LaunchConfiguration('command_gate_require_heartbeat', default='True')
    command_gate_require_enable = LaunchConfiguration('command_gate_require_enable', default='False')

    gravitational_acceleration = LaunchConfiguration('gravitational_acceleration', default='9.80665')

    camera_name = LaunchConfiguration('camera_name', default='camera')
    approx_sync = LaunchConfiguration('approx_sync', default='True')
    stereo_to_pointcloud = LaunchConfiguration('stereo_to_pointcloud', default='False')
    depthimage_to_pointcloud = LaunchConfiguration('depthimage_to_pointcloud', default='False')
    detect_ground_and_obstacles = LaunchConfiguration('detect_ground_and_obstacles', default='False')

    reset_realsense = LaunchConfiguration('reset_realsense', default='False')
    publish_realsense_pointcloud = LaunchConfiguration('publish_realsense_pointcloud', default='False')
    align_realsense_depth = LaunchConfiguration('align_realsense_depth', default='True')
    realsense_emitter_enabled = LaunchConfiguration('realsense_emitter_enabled', default='0')
    realsense_emitter_on_off = LaunchConfiguration('realsense_emitter_on_off', default='False')
    launch_realsense_splitter_node = LaunchConfiguration('launch_realsense_splitter_node', default=False)
    realsense_qos = LaunchConfiguration('realsense_qos', default='SENSOR_DATA')

    camera_launch_delay = LaunchConfiguration('camera_launch_delay', default='6.0')
    laserscan_launch_delay = LaunchConfiguration('laserscan_launch_delay', default='2.0')

    # Setup Remappings/renamings
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # # Create our own temporary YAML files that include substitutions
    # param_substitutions = {
    #     'use_sim_time': use_sim_time,
    #     'yaml_filename': map_file}
    #
    # # It only applies when `use_namespace` is True.
    # # '<robot_namespace>' keyword shall be replaced by 'namespace' launch argument
    # # in config file 'nav2_multirobot_params.yaml' as a default & example.
    # # User defined config file should contain '<robot_namespace>' keyword for the replacements.
    # params_file = ReplaceString(
    #         source_file=params_file,
    #         replacements={'<robot_namespace>': ('/', namespace)},
    #         condition=IfCondition(use_namespace))
    #
    # configured_params = ParameterFile(
    #         RewrittenYaml(
    #                 source_file=params_file,
    #                 root_key=namespace,
    #                 param_rewrites=param_substitutions,
    #                 convert_types=True),
    #         allow_substs=True)

    # Declare launch arguments
    stdout_linebuf_envvar = SetEnvironmentVariable(
            'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_f1tenth_namespace_cmd = DeclareLaunchArgument(
            'f1tenth_namespace',
            default_value=f1tenth_namespace,
            description='Top-level namespace to distinguish between each F1/10 (robot).')

    declare_use_f1tenth_namespace_cmd = DeclareLaunchArgument(
            'use_f1tenth_namespace',
            default_value=use_f1tenth_namespace,
            description='Whether to apply a namespace to the entire stack. '
                        'If launch argument for f1tenth_namespace is empty and this is true, '
                        'uses the $VEHICLE_NAME environment variable. '
                        'Defaults to the username if $VEHICLE_NAME doesn\'t exist and the namespace argument is empty.')

    declare_namespace_cmd = DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
            'use_namespace',
            default_value=use_namespace,
            description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
            'slam',
            default_value=slam,
            description='Whether to run SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
            'map_file',
            default_value=map_file,
            description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true')

    use_gpu_la = DeclareLaunchArgument(
            'use_gpu', default_value=use_gpu,
            description='Use GPU acceleration. Default: True')

    declare_params_file_cmd = DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
            'autostart', default_value=autostart,
            description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
            'use_composition', default_value=use_composition,
            description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
            'use_respawn', default_value=use_respawn,
            description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='log level')

    launch_joystick_arg = DeclareLaunchArgument('launch_joystick', default_value=launch_joystick,
                                                description="Launch the joystick driver, "
                                                            "teleop (speed and steering), and mux nodes.")

    launch_sensors_arg = DeclareLaunchArgument('launch_sensors', default_value=launch_sensors,
                                               description="Launch the sensors")

    launch_vehicle_arg = DeclareLaunchArgument('launch_vehicle', default_value=launch_vehicle,
                                               description="Launch the vehicle")

    launch_tfs_arg = DeclareLaunchArgument('launch_tfs', default_value=launch_tfs,
                                           description="Launch the tfs")

    launch_localization_arg = DeclareLaunchArgument('launch_localization',
                                                    default_value=launch_localization,
                                                    description="Launch the localization components.")
    launch_local_localization_arg = DeclareLaunchArgument('launch_local_localization',
                                                          default_value=launch_local_localization,
                                                          description="Launch the local localization component.")
    launch_global_localization_arg = DeclareLaunchArgument('launch_global_localization',
                                                           default_value=launch_global_localization,
                                                           description="Launch the global localization component.")

    localize_isaac_vslam_on_startup_la = DeclareLaunchArgument(
            'localize_isaac_vslam_on_startup', default_value=localize_isaac_vslam_on_startup,
            description='Attempt to localize Isaac ROS VSLAM in a previously saved map on startup. '
                        'Set True only when a valid VSLAM map exists at visual_slam_map_path. '
                        'False (default) prevents the intermittent SIGABRT crash caused by GXF '
                        'heap corruption on localization failure.')

    launch_map_server_la = DeclareLaunchArgument(
            'launch_map_server', default_value=launch_map_server,
            description='Whether to launch the map server. '
                        'Disable when building a new map (slam:=True) to prevent a stale stored map '
                        'from being published during mapping.')

    launch_icp_odometry_la = DeclareLaunchArgument(
            'launch_icp_odometry', default_value=launch_icp_odometry,
            description='Whether to launch RTABMap ICP LiDAR odometry (odom/rtabmap/icp, ekf_odom odom3). '
                        'Expensive and off by default; rf2o is the preferred LiDAR odometry source.')

    odom_tf_publisher_arg = DeclareLaunchArgument(
            'odom_tf_publisher', default_value=odom_tf_publisher,
            description='Node responsible for publishing the odom->base_link TF. '
                        'Options: ekf, vslam|stereo, rf2o, icp, rgbd, pointcloud, rtabmap.')
    map_tf_publisher_arg = DeclareLaunchArgument(
            'map_tf_publisher', default_value=map_tf_publisher,
            description='Node responsible for publishing the map->odom TF. '
                        'Options: amcl, pf, ekf, vslam, rtabmap, slam.')
    launch_navigation_arg = DeclareLaunchArgument('launch_navigation',
                                                  default_value=launch_navigation,
                                                  description="Launch the navigation.")
    launch_controller_server_la = DeclareLaunchArgument(
            'launch_controller_server', default_value=launch_controller_server,
            description='Launch the Nav2 controller server (local trajectory tracker). '
                        'Disable to substitute a custom node implementing FollowPath (e.g. MPC).')
    launch_smoother_server_la = DeclareLaunchArgument(
            'launch_smoother_server', default_value=launch_smoother_server,
            description='Launch the Nav2 smoother server (post-processes global paths).')
    launch_planner_server_la = DeclareLaunchArgument(
            'launch_planner_server', default_value=launch_planner_server,
            description='Launch the Nav2 planner server (global path planner). '
                        'Disable to substitute a custom planner.')
    launch_behavior_server_la = DeclareLaunchArgument(
            'launch_behavior_server', default_value=launch_behavior_server,
            description='Launch the Nav2 behavior server (recovery behaviors).')
    launch_bt_navigator_la = DeclareLaunchArgument(
            'launch_bt_navigator', default_value=launch_bt_navigator,
            description='Launch the Nav2 bt_navigator (behavior-tree coordinator).')
    launch_waypoint_follower_la = DeclareLaunchArgument(
            'launch_waypoint_follower', default_value=launch_waypoint_follower,
            description='Launch the Nav2 waypoint follower.')
    launch_velocity_smoother_la = DeclareLaunchArgument(
            'launch_velocity_smoother', default_value=launch_velocity_smoother,
            description='Launch the Nav2 velocity smoother (rate-limits cmd_vel).')
    cmd_vel_topic_la = DeclareLaunchArgument(
            'cmd_vel_topic', default_value=cmd_vel_topic,
            description='Final cmd_vel output topic from the Nav2 velocity smoother. '
                        'Default: cmd_vel (drives hardware). Set to e.g. cmd_vel_nav2 when '
                        'running a custom controller side-by-side for comparison.')
    launch_visualization_arg = DeclareLaunchArgument('launch_visualization',
                                                     default_value=launch_visualization,
                                                     description="Launch RViz.")
    rviz_config_arg = DeclareLaunchArgument('rviz_config_file',
                                            default_value=rviz_config_file,
                                            description="The path to the rviz configuration file.")

    launch_pointcloud_map_arg = DeclareLaunchArgument(
            'launch_pointcloud_map', default_value=launch_pointcloud_map,
            description='Publish a static RTABMap-exported point cloud (.pcd) on map/pointcloud '
                        'for RViz visualization only. Not used for localization or planning. '
                        'Defaults to whatever launch_visualization is set to.')
    pointcloud_map_file_arg = DeclareLaunchArgument(
            'pointcloud_map_file', default_value=pointcloud_map_file,
            description='Path to the .pcd point cloud map published when launch_pointcloud_map:=True. '
                        'Must be exported from the same RTABMap database as the occupancy grid in '
                        'map_file, or the two will not co-register.')

    launch_2d_mapping_arg = DeclareLaunchArgument('launch_2d_mapping',
                                                  default_value=launch_2d_mapping,
                                                  description="Enable 2D mapping.")
    launch_3d_mapping_arg = DeclareLaunchArgument('launch_3d_mapping',
                                                  default_value=launch_3d_mapping,
                                                  description="Enable 3D mapping.")

    life_long_mapping_arg = DeclareLaunchArgument('life_long_mapping',
                                                  default_value=life_long_mapping,
                                                  description="If set to True, the map will not be deleted but will "
                                                              "instead add to a preexisting map. Setting to false "
                                                              "deletes the old map and starts again.")

    offline_mapping_2d_param_file_la = DeclareLaunchArgument('offline_mapping_2d_param_file',
                                                             default_value=offline_mapping_2d_param_file,
                                                             description="Path to the config file for the "
                                                                         "offline 2D mapping node.")

    online_mapping_2d_param_file_la = DeclareLaunchArgument('online_mapping_2d_param_file',
                                                            default_value=online_mapping_2d_param_file,
                                                            description="Path to the config file for the "
                                                                        "online 2D mapping node.")
    map_2d_file_la = DeclareLaunchArgument('map_2d_file',
                                           default_value=map_2d_file,
                                           description="Path to the save the 2D map.")

    rtabmap_database_file_la = DeclareLaunchArgument('rtabmap_database_file',
                                                     default_value=rtabmap_database_file,
                                                     description="Path to the config file for the 3D mapping node.")

    require_deadman_la = DeclareLaunchArgument(
            'require_deadman',
            default_value='True',
            description='Require a deadman button (L1/LB) to be held to arm actuators. '
                        'Set False to disable the safety interlock.')

    deadman_buttons_la = DeclareLaunchArgument(
            'deadman_buttons',
            default_value=deadman_buttons,
            description='Buttons used to arm the vehicle actuators. Ignored when require_deadman:=False.')

    autonomous_deadman_buttons_la = DeclareLaunchArgument(
            'autonomous_deadman_buttons',
            default_value=autonomous_deadman_buttons,
            description='Buttons used to enable autonomous control. Ignored when require_deadman:=False.')

    steering_button_la = DeclareLaunchArgument(
            'steering_button',
            default_value=steering_button,
            description='Button used to control the steering angle. 2 for DualShock/DualSense, 3 for Logitech F710')

    max_speed_la = DeclareLaunchArgument(
            'max_speed',
            default_value=max_speed,
            description='The maximum speed in m/s.')

    max_steering_la = DeclareLaunchArgument(
            'max_steering',
            default_value=max_steering,
            description='The maximum steering angle in rads.')

    max_acceleration_la = DeclareLaunchArgument(
            'max_acceleration',
            default_value=max_acceleration,
            description='The maximum acceleration in m/s^2.')

    max_steering_rate_la = DeclareLaunchArgument(
            'max_steering_rate',
            default_value=max_steering_rate,
            description='The maximum steering rate in rads/s.')

    vesc_poll_rate_la = DeclareLaunchArgument(
            'vesc_poll_rate',
            default_value=vesc_poll_rate,
            description='The frequency at which to send/receive messages from/to the VESC.')

    vesc_imu_poll_rate_la = DeclareLaunchArgument(
            'vesc_imu_poll_rate',
            default_value=vesc_imu_poll_rate,
            description='The frequency at which to poll IMU data from the VESC.')

    use_imu_yaw_rate_la = DeclareLaunchArgument(
            'use_imu_yaw_rate',
            default_value='False',
            description='Use gyro-z from sensors/imu/raw for yaw integration in vesc_to_odom '
                        'instead of the kinematic model.')

    use_closed_loop_speed_la = DeclareLaunchArgument(
            'use_closed_loop_speed', default_value='False',
            description='Use closed-loop PID speed control in ackermann_to_vesc.')
    speed_kp_la = DeclareLaunchArgument(
            'speed_kp', default_value=speed_kp,
            description='Proportional gain for the closed-loop speed controller.')
    speed_ki_la = DeclareLaunchArgument(
            'speed_ki', default_value=speed_ki,
            description='Integral gain for the closed-loop speed controller.')
    speed_anti_windup_la = DeclareLaunchArgument(
            'speed_anti_windup', default_value=speed_anti_windup,
            description='Anti-windup clamp (ERPM) for the speed integrator.')
    use_adaptive_ff_la = DeclareLaunchArgument(
            'use_adaptive_ff', default_value='False',
            description='Enable adaptive feedforward gain in ackermann_to_vesc.')
    adaptive_ff_alpha_la = DeclareLaunchArgument(
            'adaptive_ff_alpha', default_value=adaptive_ff_alpha,
            description='Low-pass filter coefficient for the adaptive feedforward gain.')
    adaptive_ff_gain_min_la = DeclareLaunchArgument(
            'adaptive_ff_gain_min', default_value=adaptive_ff_gain_min,
            description='Minimum adaptive feedforward gain (ERPM/(m/s)).')
    adaptive_ff_gain_max_la = DeclareLaunchArgument(
            'adaptive_ff_gain_max', default_value=adaptive_ff_gain_max,
            description='Maximum adaptive feedforward gain (ERPM/(m/s)).')
    vesc_max_speed_la = DeclareLaunchArgument(
            'vesc_max_speed', default_value=vesc_max_speed,
            description='Speed clamp in ackermann_to_vesc (m/s). 0 disables the limit.')
    vesc_max_steering_angle_la = DeclareLaunchArgument(
            'vesc_max_steering_angle', default_value=vesc_max_steering_angle,
            description='Steering angle clamp in ackermann_to_vesc (rad). 0 disables the limit.')
    use_accel_ff_la = DeclareLaunchArgument(
            'use_accel_ff', default_value='False',
            description='Use acceleration feedforward in ackermann_to_vesc.')
    accel_to_erpm_gain_la = DeclareLaunchArgument(
            'accel_to_erpm_gain', default_value=accel_to_erpm_gain,
            description='Gain mapping acceleration (m/s^2) to ERPM for feedforward.')
    use_cmd_accel_rate_limit_la = DeclareLaunchArgument(
            'use_cmd_accel_rate_limit', default_value='False',
            description='Rate-limit acceleration commands before sending to the VESC.')

    declare_launch_ackermann_to_vesc_node = DeclareLaunchArgument(
            'launch_ackermann_to_vesc_node',
            default_value=launch_ackermann_to_vesc_node,
            description='Send ackermann commands to the VESC.')
    declare_launch_vesc_to_odom_node = DeclareLaunchArgument(
            'launch_vesc_to_odom_node',
            default_value=launch_vesc_to_odom_node,
            description='Publish odometry messages from the VESC.')
    declare_launch_throttle_interpolator_node = DeclareLaunchArgument(
            'launch_throttle_interpolator_node',
            default_value=launch_throttle_interpolator_node,
            description='Interpolate commands before sending to the VESC. '
                        'Set to False if using MPC (or increase limits), True otherwise')
    declare_launch_ackermann_to_twist = DeclareLaunchArgument(
            'launch_ackermann_to_twist',
            default_value=launch_ackermann_to_twist,
            description='Start ackermann_to_twist converter to republish ackermann_cmd as cmd_vel for EKF use_control.')
    launch_command_gate_arg = DeclareLaunchArgument(
            'launch_command_gate',
            default_value=launch_command_gate,
            description='Launch the command_gate safety relay between ackermann_mux and ackermann_to_vesc.')
    command_gate_require_heartbeat_la = DeclareLaunchArgument(
            'command_gate_require_heartbeat',
            default_value=command_gate_require_heartbeat,
            description='Close gate on heartbeat timeout. Set True and publish ~/heartbeat to activate watchdog.')
    command_gate_require_enable_la = DeclareLaunchArgument(
            'command_gate_require_enable',
            default_value=command_gate_require_enable,
            description='Gate starts closed; open with: '
                        'ros2 service call /command_gate/set_enabled std_srvs/srv/SetBool \'{data: true}\'')

    gravitational_acceleration_la = DeclareLaunchArgument(
            'gravitational_acceleration', default_value=gravitational_acceleration,
            description='Gravitational acceleration (m/s^2). Calibrate per robot.')

    camera_name_la = DeclareLaunchArgument(
            'camera_name', default_value=camera_name,
            description='Name of the camera. Used to remap topics.')

    approx_sync_la = DeclareLaunchArgument(
            'approx_sync', default_value=approx_sync,
            description='Synchronize topics')

    stereo_to_pointcloud_la = DeclareLaunchArgument('stereo_to_pointcloud',
                                                    default_value=stereo_to_pointcloud,
                                                    description='Whether to publish a PointCloud2 message from stereo '
                                                                'images.')

    depthimage_to_pointcloud_la = DeclareLaunchArgument('depthimage_to_pointcloud',
                                                        default_value=depthimage_to_pointcloud,
                                                        description='Whether to publish a PointCloud2 message from a '
                                                                    'depth image.')
    detect_ground_and_obstacles_la = DeclareLaunchArgument('detect_ground_and_obstacles',
                                                           default_value=detect_ground_and_obstacles,
                                                           description='Whether to use RTABmaps obstacle detector.')

    reset_realsense_la = DeclareLaunchArgument(
            'reset_realsense',
            default_value='False',
            description='Whether to reset the realsense device.')

    publish_realsense_pointcloud_la = DeclareLaunchArgument('publish_realsense_pointcloud',
                                                            default_value=publish_realsense_pointcloud,
                                                            description='Whether to publish PointClouds using '
                                                                        'librealsense SDK. Could be disabled when '
                                                                        'recording ROSBags or mapping. ')

    align_realsense_depth_la = DeclareLaunchArgument('align_realsense_depth',
                                                     default_value=align_realsense_depth,
                                                     description='Whether to align the depth to other frames')

    realsense_emitter_enabled_la = DeclareLaunchArgument(
            'realsense_emitter_enabled',
            default_value=realsense_emitter_enabled,
            description='Whether to enable the IR emitters to improve depth and pointcloud quality. '
                        'Unfortunately, this renders the stereo IR cameras unusable for mapping, '
                        'VSLAM, VIO odometry, etc. '
                        'Disable when mapping or running VIO enable if accurate pointclouds are essential.')

    realsense_emitter_on_off_la = DeclareLaunchArgument(
            'realsense_emitter_on_off',
            default_value=realsense_emitter_on_off,
            description='Whether to alternate enabling/disabling the emitters. '
                        'This can be used to simultaneously '
                        'get accurate depth maps and pointclouds (when in the on state, i.e enabled) and '
                        'have usable IR images (when in the off state)')

    launch_realsense_splitter_node_la = DeclareLaunchArgument(
            'launch_realsense_splitter_node', default_value=launch_realsense_splitter_node,
            description='Whether to launch the realsense splitter node.')

    realsense_qos_la = DeclareLaunchArgument(
            'realsense_qos', default_value=realsense_qos,
            description='The qos profile to use for the realsense camera. '
                        'Default: SENSOR_DATA. See librealsense2 documentation for more details.')

    camera_launch_delay_la = DeclareLaunchArgument(
            'camera_launch_delay', default_value=camera_launch_delay,
            description='Delay in seconds before launching the camera nodes. '
                        'Used to avoid USB bandwidth limitations, '
                        'especially startup current draw caused by booting multiple USB devices simultaneously.')

    laserscan_launch_delay_la = DeclareLaunchArgument(
            'laserscan_launch_delay', default_value=laserscan_launch_delay,
            description='Delay in seconds before launching the laserscan nodes. '
                        'Used to avoid USB bandwidth limitations, '
                        'especially startup current draw caused by booting multiple USB devices simultaneously.')

    # Add launch arguments to a list
    launch_args = [
        stdout_linebuf_envvar,
        declare_f1tenth_namespace_cmd,
        declare_use_f1tenth_namespace_cmd,
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        declare_slam_cmd,
        declare_map_yaml_cmd,
        declare_use_sim_time_cmd,
        use_gpu_la,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_use_composition_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        launch_joystick_arg,
        launch_sensors_arg,
        launch_vehicle_arg,
        launch_tfs_arg,
        launch_localization_arg,
        launch_local_localization_arg,
        launch_global_localization_arg,
        localize_isaac_vslam_on_startup_la,
        launch_map_server_la,
        launch_icp_odometry_la,
        odom_tf_publisher_arg,
        map_tf_publisher_arg,
        launch_navigation_arg,
        launch_controller_server_la, launch_smoother_server_la, launch_planner_server_la,
        launch_behavior_server_la, launch_bt_navigator_la, launch_waypoint_follower_la,
        launch_velocity_smoother_la, cmd_vel_topic_la,
        launch_visualization_arg,
        rviz_config_arg,
        launch_pointcloud_map_arg, pointcloud_map_file_arg,
        launch_2d_mapping_arg,
        launch_3d_mapping_arg,
        life_long_mapping_arg,
        offline_mapping_2d_param_file_la,
        online_mapping_2d_param_file_la,
        map_2d_file_la,
        rtabmap_database_file_la,
        require_deadman_la, deadman_buttons_la, autonomous_deadman_buttons_la, steering_button_la, max_speed_la, max_steering_la,
        max_acceleration_la, max_steering_rate_la, vesc_poll_rate_la,
        vesc_imu_poll_rate_la, use_imu_yaw_rate_la,
        use_closed_loop_speed_la, speed_kp_la, speed_ki_la, speed_anti_windup_la,
        use_adaptive_ff_la, adaptive_ff_alpha_la, adaptive_ff_gain_min_la, adaptive_ff_gain_max_la,
        vesc_max_speed_la, vesc_max_steering_angle_la,
        use_accel_ff_la, accel_to_erpm_gain_la, use_cmd_accel_rate_limit_la,
        declare_launch_ackermann_to_vesc_node, declare_launch_vesc_to_odom_node,
        declare_launch_throttle_interpolator_node, declare_launch_ackermann_to_twist,
        launch_command_gate_arg, command_gate_require_heartbeat_la, command_gate_require_enable_la,
        camera_name_la,
        approx_sync_la, stereo_to_pointcloud_la, depthimage_to_pointcloud_la,
        detect_ground_and_obstacles_la, reset_realsense_la, publish_realsense_pointcloud_la, align_realsense_depth_la,
        realsense_emitter_enabled_la, realsense_emitter_on_off_la, launch_realsense_splitter_node_la,
        realsense_qos_la,
        camera_launch_delay_la, laserscan_launch_delay_la,
        gravitational_acceleration_la,
    ]

    ''' Launch Nodes '''
    camera_name_string = camera_name.perform(context)
    realsense_qos_string = realsense_qos.perform(context)
    use_f1tenth_namespace_string = use_f1tenth_namespace.perform(context)
    f1tenth_namespace_string = f1tenth_namespace.perform(context)

    if camera_name_string != '':
        camera_name_string += '/'

    realsense_qos_int = qos_str_to_rtabmap_int.get(realsense_qos_string, 2)

    # In mapping mode there is no prior map to localize against, so the global localization stack
    # must not run: AMCL, the map EKF and the SLAM node would all broadcast map->odom, and
    # map_server would publish /map from a file alongside the SLAM node's live /map. Measured with
    # slam:=True before this gate: map->odom at 25.2 Hz from three broadcasters, and
    # `ros2 topic info -v /<ns>/map` reporting "Publisher count: 2" (rtabmap AND map_server) — so a
    # map saved during mapping could come from the stale file rather than the run.
    slam_string = slam.perform(context)
    global_localization_effective = 'False' if slam_string.lower() == 'true' else launch_global_localization
    map_server_effective = 'False' if slam_string.lower() == 'true' else launch_map_server

    # determine what namespace to use
    if use_f1tenth_namespace_string.lower() == 'true':
        if not f1tenth_namespace_string.lower().strip():
            # if launch argument is empty, use the VEHICLE_NAME environment variable.
            # Defaults to the username if $VEHICLE_NAME doesn't exist and the namespace argument is empty.
            f1tenth_namespace = EnvironmentVariable(
                    'VEHICLE_NAME',
                    default_value=EnvironmentVariable('USER', default_value='')
            )

            if f1tenth_namespace.perform(context) == '':
                raise RuntimeError(
                        'The launch argument "use_f1tenth_namespace" was set to "true" '
                        'but the launch argument "f1tenth_namespace" is empty. '
                        'Set the launch argument "f1tenth_namespace" to the namespace you want to use. '
                        'If you do not want to use a namespace, do not set the launch argument "use_f1tenth_namespace".'
                )

    # Build the absolute container name so LoadComposableNodes can find it regardless of
    # whether a namespace is active.  Without this, the (namespace, '/', container_name)
    # pattern in nav2_navigation.launch.py resolves to '//f1tenth_container' (→ absolute
    # '/f1tenth_container') and silently misses the container at '/gosling1/f1tenth_container'.
    _resolved_ns = f1tenth_namespace.perform(context)
    nav2_container_name = f'/{_resolved_ns}/f1tenth_container' if _resolved_ns else 'f1tenth_container'

    # Costmap observation-source topics use a <ns_prefix> placeholder (e.g.
    # <ns_prefix>lidar/scan_filtered) that must be resolved to an absolute path before the
    # parameter file reaches any node.  A bare relative topic like lidar/scan_filtered is
    # resolved from each costmap sub-node's own namespace
    # (/gosling1/local_costmap or /gosling1/global_costmap), which would produce
    # gosling1/local_costmap/lidar/scan_filtered instead of gosling1/lidar/scan_filtered.
    # Costmap sub-nodes are spawned internally by controller_server/planner_server and read
    # params from the container's process-wide parameter server (not from LoadComposableNodes),
    # so ReplaceString must be applied here — to the container's params file — not only in
    # nav2_navigation.launch.py.
    _ns_prefix = f'/{_resolved_ns}/' if _resolved_ns else '/'
    _params_file_with_prefix = ReplaceString(
        source_file=params_file,
        replacements={'<ns_prefix>': _ns_prefix})

    # Costmap sub-nodes (e.g. /gosling1/local_costmap/local_costmap) are created
    # programmatically by controller_server/planner_server and inherit the container's
    # process-wide params file.  They look for params at gosling1.local_costmap.* — so
    # the container must carry the namespace-prefixed file, not the raw one.
    container_nav2_params = ParameterFile(
        RewrittenYaml(
            source_file=_params_file_with_prefix,
            root_key=_resolved_ns,
            param_rewrites={'use_sim_time': use_sim_time},
            convert_types=True),
        allow_substs=True) if _resolved_ns else ParameterFile(
        _params_file_with_prefix,
        allow_substs=True)

    component_container_node = Node(
            condition=IfCondition(use_composition),
            name='f1tenth_container',  # todo: set as a launch argument
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[
                container_nav2_params,
                {
                    'autostart': autostart,
                    'use_sim_time': use_sim_time,
                    'yaml_filename': map_file,
                    'thread_num': os.cpu_count(),
                }
            ],
            arguments=[
                '--use_multi_threaded_executor',
                '--ros-args', '--log-level', log_level
            ],
            remappings=remappings,
            output='screen')

    joystick_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([vehicle_include_dir, 'joystick.launch.py'])
            ),
            condition=IfCondition(launch_joystick),
            launch_arguments={
                "require_deadman": require_deadman,
                "deadman_buttons": deadman_buttons,
                "autonomous_deadman_buttons": autonomous_deadman_buttons,
                "steering_button": steering_button,
                "max_speed": max_speed,
                "max_steering": max_steering
            }.items()
    )

    ackermann_mux_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([vehicle_include_dir, 'ackermann_mux.launch.py'])
            ),
            condition=IfCondition(launch_joystick)
    )

    command_gate_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([vehicle_include_dir, 'command_gate.launch.py'])
            ),
            condition=IfCondition(launch_command_gate),
            launch_arguments={
                'use_composition': use_composition,
                'command_gate_require_heartbeat': command_gate_require_heartbeat,
                'command_gate_require_enable': command_gate_require_enable,
            }.items()
    )

    # command_gate is the SOLE publisher of vehicle/ackermann_cmd. With it disabled the
    # ackermann_mux has no path to ackermann_to_vesc, so the vehicle will not move unless
    # something else publishes vehicle/ackermann_cmd. Warn loudly instead of failing — a
    # custom controller/MPC publishing that topic directly is a valid reason to disable it.
    # (ROS 2 launch has no LogWarn action; LogInfo with a WARNING prefix is the idiom.)
    command_gate_disabled_warning = LogInfo(
            condition=UnlessCondition(launch_command_gate),
            msg='WARNING: launch_command_gate:=False — command_gate is the only publisher of '
                'vehicle/ackermann_cmd, so the ackermann_mux now has NO path to the VESC and the '
                'vehicle will not respond to drive/teleop commands. Publish vehicle/ackermann_cmd '
                'from your own controller, or for a transparent passthrough keep '
                'launch_command_gate:=True with command_gate_require_heartbeat:=False and '
                'command_gate_require_enable:=False.')

    sensors_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([sensor_include_dir, 'sensors.launch.py'])
            ),
            condition=IfCondition(launch_sensors),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "use_namespace": use_f1tenth_namespace,
                "namespace": f1tenth_namespace,
                "camera_name": camera_name,
                "approx_sync": approx_sync,
                "stereo_to_pointcloud": stereo_to_pointcloud,
                "depthimage_to_pointcloud": depthimage_to_pointcloud,
                "detect_ground_and_obstacles": detect_ground_and_obstacles,
                'reset_realsense': reset_realsense,
                "publish_realsense_pointcloud": publish_realsense_pointcloud,
                "align_realsense_depth": align_realsense_depth,
                "realsense_emitter_enabled": realsense_emitter_enabled,
                "realsense_emitter_on_off": realsense_emitter_on_off,
                "launch_realsense_splitter_node": launch_realsense_splitter_node,
                "camera_launch_delay": camera_launch_delay,
                "laserscan_launch_delay": laserscan_launch_delay,
                "qos": realsense_qos,
                # "attach_to_shared_component_container": 'True',
                # # this launch file starts a container
                # "component_container_name": 'realsense_d435i_container',
                # # # hardcode so its separate from the others # container_name if ('true' in map(str.lower, [use_composition_string, attach_to_shared_component_container_string])) else 'sensing_container',
                # "intra_process_comms": 'True',
            }.items()
    )

    vehicle_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([vehicle_include_dir, 'vehicle.launch.py'])
            ),
            condition=IfCondition(launch_vehicle),
            launch_arguments={
                "launch_imu_filter": 'False',  # disabled since not all the vehicle controllers have IMUs.
                "launch_ackermann_to_vesc_node": launch_ackermann_to_vesc_node,
                "launch_vesc_to_odom_node": launch_vesc_to_odom_node,
                "launch_throttle_interpolator_node": launch_throttle_interpolator_node,
                "launch_ackermann_to_twist": launch_ackermann_to_twist,
                "max_acceleration": max_acceleration,
                "max_steering_rate": max_steering_rate,
                "vesc_poll_rate": vesc_poll_rate,
                "vesc_imu_poll_rate": vesc_imu_poll_rate,
                "use_imu_yaw_rate": use_imu_yaw_rate,
                "use_closed_loop_speed": use_closed_loop_speed,
                "speed_kp": speed_kp,
                "speed_ki": speed_ki,
                "speed_anti_windup": speed_anti_windup,
                "use_adaptive_ff": use_adaptive_ff,
                "adaptive_ff_alpha": adaptive_ff_alpha,
                "adaptive_ff_gain_min": adaptive_ff_gain_min,
                "adaptive_ff_gain_max": adaptive_ff_gain_max,
                "vesc_max_speed": vesc_max_speed,
                "vesc_max_steering_angle": vesc_max_steering_angle,
                "use_accel_ff": use_accel_ff,
                "accel_to_erpm_gain": accel_to_erpm_gain,
                "use_cmd_accel_rate_limit": use_cmd_accel_rate_limit,
            }.items()
    )

    tf_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([vehicle_include_dir, 'static_transformations.launch.py'])
            ),
            condition=LaunchConfigurationEquals('launch_tfs', 'True'),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'log_level': log_level,
            }.items()
    )

    localization_launch = TimerAction(
            period=10.0,
            actions=[
                GroupAction(
                        [
                            # NO PushRosNamespace here: localization.launch.py namespaces itself
                            # (its own PushRosNamespace + explicit namespace= on nodes), so pushing
                            # again yields /gosling1/gosling1/*. This push was gated on the legacy
                            # 'use_namespace' arg, which defaults False — but sensors_launch is
                            # visited BEFORE this action and passes use_namespace/namespace as
                            # launch_arguments, which leak into the shared context and silently
                            # flip this condition to True. Result: with launch_sensors:=True every
                            # localization node was double-namespaced and received no data.
                            SetRemap(src=['/tf'], dst=['tf']),
                            SetRemap(src=['/tf_static'], dst=['tf_static']),
                            IncludeLaunchDescription(
                                    PythonLaunchDescriptionSource(
                                            PathJoinSubstitution([localization_include_dir, 'localization.launch.py'])
                                    ),
                                    condition=IfCondition(launch_localization),
                                    launch_arguments={
                                        "namespace": f1tenth_namespace,
                                        "use_namespace": use_f1tenth_namespace,
                                        "use_composition": use_composition,
                                        "camera_name": camera_name,
                                        # Pass the localization param file explicitly. Otherwise the
                                        # bringup-level 'params_file' (nav2_params.yaml, whose amcl
                                        # section uses scan_topic: scan) is inherited by the include and
                                        # AMCL subscribes to the wrong scan topic, never localizing.
                                        "params_file": localization_params_file_path,
                                        "launch_sensor_fusion": 'True',
                                        "launch_map_server": map_server_effective,
                                        "launch_ekf_odom": launch_local_localization,
                                        "launch_ekf_map": global_localization_effective,
                                        "odom_tf_publisher": odom_tf_publisher,
                                        "map_tf_publisher": map_tf_publisher,
                                        "launch_slam_toolbox_localizer": 'False',
                                        "launch_rtabmap_localizer": 'False',
                                        'launch_pointcloud_odometry': 'False',
                                        'launch_rgbd_odometry': 'False',
                                        'launch_stereo_odometry': 'True',
                                        'launch_laserscan_odometry': 'True',
                                        'launch_icp_odometry': launch_icp_odometry,
                                        'launch_amcl': global_localization_effective,
                                        "map_file": map_file,
                                        "use_sim_time": use_sim_time,
                                        "use_gpu": use_gpu,
                                        "qos_rtabmap": "1",
                                        "qos_rtabmap_laserscan": "1",
                                        "qos_rtabmap_camera": str(realsense_qos_int),
                                        "qos_rtabmap_imu": str(realsense_qos_int),
                                        "qos": realsense_qos,
                                        "qos_imu": realsense_qos,
                                        "gravitational_acceleration": gravitational_acceleration,
                                        "localize_on_startup": localize_isaac_vslam_on_startup,
                                        "log_level": log_level,
                                    }.items()
                            )
                        ]
                )
            ]
    )

    visualization_launch = Node(
            condition=IfCondition(launch_visualization),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file, '--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('/goal_pose', 'goal_pose'),
                ('/initialpose', 'initialpose'),
                ('/clicked_point', 'clicked_point'),
            ]
    )

    # Visualization-only: republish a static RTABMap-exported .pcd as a latched PointCloud2.
    # Decoupled from localization/planning — purely for an aesthetic 3D map overlay in RViz.
    pointcloud_map_launch = Node(
            condition=IfCondition(launch_pointcloud_map),
            package='pcl_ros',
            executable='pcd_to_pointcloud',
            name='pointcloud_map_publisher',
            output='screen',
            parameters=[{
                'file_name': pointcloud_map_file,
                'tf_frame': 'map',
                'publishing_period_ms': 3000,
            }],
            remappings=[('cloud_pcd', 'map/pointcloud')],
            arguments=['--ros-args', '--log-level', log_level],
    )

    mapping_launch = TimerAction(
            period=15.0,
            actions=[
                GroupAction(
                        actions=[
                            # NO PushRosNamespace here — same reason as the localization include
                            # above: mapping.launch.py already pushes f1tenth_namespace in its own
                            # nodes_to_launch group, and the legacy 'use_namespace' condition is
                            # flipped True by config leaked from sensors_launch.
                            SetRemap(src=['/tf'], dst=['tf']),
                            SetRemap(src=['/tf_static'], dst=['tf_static']),
                            IncludeLaunchDescription(
                                    PythonLaunchDescriptionSource(
                                            os.path.join(f1tenth_launch_bringup_dir, 'mapping.launch.py')),
                                    condition=IfCondition(slam),
                                    launch_arguments={'namespace': namespace,
                                                      'use_namespace': use_namespace,
                                                      'use_sim_time': use_sim_time,
                                                      'use_gpu': use_gpu,
                                                      'autostart': autostart,
                                                      'use_composition': use_composition,
                                                      # bringup already created 'f1tenth_container'
                                                      # (component_container_node above). Without
                                                      # this, mapping.launch.py creates a SECOND
                                                      # container with the same name, namespace and
                                                      # argv, and LoadComposableNodes targets are
                                                      # then ambiguous (bug-022, class of bug-018).
                                                      'attach_to_shared_component_container': use_composition,
                                                      'container_name': 'f1tenth_container',
                                                      'use_respawn': use_respawn,
                                                      "launch_joystick": 'False',
                                                      "launch_sensors": 'False',
                                                      "launch_vehicle": 'False',
                                                      "launch_tfs": 'False',
                                                      # bringup launches command_gate itself (see
                                                      # command_gate_launch above), and mapping.launch.py
                                                      # delegates to teleop.launch.py which launches one
                                                      # too. Without this suppression slam:=True produced
                                                      # TWO /command_gate nodes — and unlike the other
                                                      # bug-007 duplicates, which land inert at root '/',
                                                      # both of these land at /<namespace> and both publish
                                                      # vehicle/ackermann_cmd. Two independent heartbeat
                                                      # watchdogs on the actuation topic is a safety
                                                      # hazard once the VESC is powered (bug-016).
                                                      "launch_command_gate": 'False',
                                                      "launch_localization": 'False',
                                                      "launch_local_localization": 'False',
                                                      # bringup starts the EKF itself (localization
                                                      # include above), so the two flags directly
                                                      # above mean "don't start a SECOND copy" —
                                                      # they do NOT mean odometry is absent. Say so
                                                      # explicitly, otherwise mapping stands up its
                                                      # own visual odometry and fights the EKF for
                                                      # ownership of odom->base_link.
                                                      "external_odometry": 'True',
                                                      # "odom_tf_publisher": odom_tf_publisher,
                                                      # "map_tf_publisher": map_tf_publisher,
                                                      "launch_global_localization": 'False',
                                                      "launch_visualization": 'True',
                                                      # "rviz_config_file": rviz_config_file,
                                                      "launch_2d_mapping": launch_2d_mapping,
                                                      "launch_3d_mapping": launch_3d_mapping,
                                                      "life_long_mapping": life_long_mapping,
                                                      "offline_mapping_2d_param_file": offline_mapping_2d_param_file,
                                                      "online_mapping_2d_param_file": online_mapping_2d_param_file,
                                                      "map_2d_file": map_2d_file,
                                                      "rtabmap_database_file": rtabmap_database_file,
                                                      "localize_isaac_vslam_on_startup": localize_isaac_vslam_on_startup,
                                                      "log_level": log_level,
                                                      }.items())
                        ]
                )
            ]
    )

    nav2_navigation_launch = TimerAction(
            period=15.0,
            actions=[
                GroupAction(
                        actions=[
                            PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
                            SetRemap(src=['/tf'], dst=['tf']),
                            SetRemap(src=['/tf_static'], dst=['tf_static']),
                            IncludeLaunchDescription(
                                    PythonLaunchDescriptionSource(
                                            PathJoinSubstitution(
                                                    [f1tenth_launch_bringup_dir, 'nav2_navigation.launch.py'])),
                                    condition=IfCondition(launch_navigation),
                                    launch_arguments={'namespace': _resolved_ns,
                                                      'use_sim_time': use_sim_time,
                                                      'autostart': autostart,
                                                      'params_file': params_file,
                                                      'use_composition': use_composition,
                                                      'use_respawn': use_respawn,
                                                      'container_name': nav2_container_name,
                                                      'launch_controller_server': launch_controller_server,
                                                      'launch_smoother_server': launch_smoother_server,
                                                      'launch_planner_server': launch_planner_server,
                                                      'launch_behavior_server': launch_behavior_server,
                                                      'launch_bt_navigator': launch_bt_navigator,
                                                      'launch_waypoint_follower': launch_waypoint_follower,
                                                      'launch_velocity_smoother': launch_velocity_smoother,
                                                      'cmd_vel_topic': cmd_vel_topic,
                                                      'log_level': log_level,
                                                      }.items())
                        ]
                )
            ]
    )

    # Group Actions
    vehicle_bringup_group = GroupAction(
            actions=[
                PushRosNamespace(
                        condition=IfCondition(use_namespace),
                        namespace=namespace
                ),
                SetParameter(name='use_sim_time', value=use_sim_time),

                # nodes
                joystick_launch,
                ackermann_mux_launch,
                command_gate_launch,
                command_gate_disabled_warning,
                vehicle_launch,
                tf_launch
            ]
    )

    nav2_bringup_group = GroupAction(
            # condition=IfCondition(),
            actions=[
                PushRosNamespace(
                        condition=IfCondition(use_namespace),
                        namespace=namespace
                ),
                SetParameter(name='use_sim_time', value=use_sim_time),
                SetRemap(src=['/tf'], dst=['tf']),
                SetRemap(src=['/tf_static'], dst=['tf_static']),

                # nodes
                mapping_launch,
                nav2_navigation_launch,
            ]
    )

    # Add launch arguments and nodes to the launch description
    nodes_to_launch = GroupAction(
            actions=[
                PushRosNamespace(
                        condition=IfCondition(use_f1tenth_namespace),
                        namespace=f1tenth_namespace
                ),
                SetParameter(name='use_sim_time', value=use_sim_time),
                SetRemap(src=['/tf'], dst=['tf']),
                SetRemap(src=['/tf_static'], dst=['tf_static']),
                # nodes
                component_container_node,
                visualization_launch,
                pointcloud_map_launch,
                vehicle_bringup_group,
                nav2_bringup_group,
            ]
    )  # append F1/10 namespace to all nodes
    ld = launch_args + [
                nodes_to_launch,
                # localization is placed outside nodes_to_launch for the same reason as teleop:
                # nav2_bringup_group inherits the f1tenth_namespace push from nodes_to_launch, so
                # including localization inside it would make every composable node in
                # localization.launch.py double-namespaced (e.g. gosling1/gosling1/amcl).
                # AMCL would publish /tf to gosling1/gosling1/tf instead of gosling1/tf and
                # the map→odom transform would never reach the TF tree.
                sensors_launch,
                localization_launch,
            ]
    return ld

def generate_launch_description():
    return LaunchDescription(
            [
                SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
                OpaqueFunction(function=launch_setup)
            ]
    )
