"""
Launches (optionally): 2D mapping (online or offline), 3D mapping (online or offline).
    Can also choose GPU/CPU mode.

Mapping notes:
    * RTABMap's (CPU + 3D) 2D map is more accurate and consistent than SLAM toolbox's implementation.
    * For now: using Depth + Color with RTABMap works better than stereo. Todo: test with good localization
    * Both GPU (Isaac Nvblox) and RTABMap work perfectly and they both use Color + Depth.
    * Use RTABMap for future localization support

Mapping Sensor recommendations in order of detail:
    1. RGB + (aligned) Depth + LaserScan + IMU
    2. Left Infrared (infra1) + Depth + LaserScan + IMU
    3. Left Infrared (infra1) + Right Infrared (infra2) + LaserScan + IMU

Todo:
    * Write instructions for saving 2D maps
    * Write instructions for saving RTABMaps
    * Write instructions for saving Nvblox maps
    * Write instructions for saving isaac ros vlsam maps
"""
import os
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, SetRemap, PushRosNamespace, SetParametersFromFile, SetParameter
from launch_ros.descriptions import ParameterFile
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression, \
    EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, GroupAction, OpaqueFunction, \
    SetEnvironmentVariable
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
    nvidia_isaac_launch_dir = os.path.join(f1tenth_launch_bringup_dir, 'nvidia_isaac_ros')

    # Setup default directories.
    map_file_path = os.path.join(f1tenth_launch_dir, 'data', 'maps', 'raslab.yaml')
    rviz_config_path = os.path.join(f1tenth_launch_dir, 'config', 'f1tenth.rviz')
    offline_mapping_2d_param_file_path = os.path.join(f1tenth_launch_dir, "config/mapping/2d_mapping_offline.yaml")
    online_mapping_2d_param_file_path = os.path.join(f1tenth_launch_dir, "config/mapping/2d_mapping_online.yaml")
    default_2d_map_file_path = os.path.join(f1tenth_launch_dir, 'data', 'maps', 'raslab.yaml')
    rtabmap_database_file_path = os.path.join(f1tenth_launch_dir, 'data', 'maps', 'rtabmap', 'rtabmap.db')

    # Setup launch configuration variables
    f1tenth_namespace = LaunchConfiguration('f1tenth_namespace',
                                            default='')  # used to distinguish between multiple F1/10s
    use_f1tenth_namespace = LaunchConfiguration('use_f1tenth_namespace', default=False)
    map_file = LaunchConfiguration('map_file', default=map_file_path)
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)  # set True for offline/rosbag playback
    use_gpu = LaunchConfiguration('use_gpu', default=False)  # False=RTABMap (CPU), True=Nvblox (GPU); bringup.launch.py defaults to True
    use_gpu_for_localization = LaunchConfiguration('use_gpu_for_localization', default='true')
    autostart = LaunchConfiguration('autostart', default='True')
    use_composition = LaunchConfiguration('use_composition', default='True')
    container_name = LaunchConfiguration('container_name', default='f1tenth_container')
    use_respawn = LaunchConfiguration('use_respawn', default='False')
    launch_joystick = LaunchConfiguration('launch_joystick', default=False)
    launch_sensors = LaunchConfiguration('launch_sensors', default=False)
    launch_vehicle = LaunchConfiguration('launch_vehicle', default=False)
    launch_tfs = LaunchConfiguration('launch_tfs', default=False)
    launch_localization = LaunchConfiguration('launch_localization', default=True)
    launch_local_localization = LaunchConfiguration('launch_local_localization', default=True)
    launch_global_localization = LaunchConfiguration('launch_global_localization', default=False)
    localize_isaac_vslam_on_startup = LaunchConfiguration('localize_isaac_vslam_on_startup', default=False)
    launch_map_server = LaunchConfiguration('launch_map_server', default=False)  # False: building a map, not consuming one
    launch_map_saver = LaunchConfiguration('launch_map_saver', default=True)   # True: auto-save 2D map during active mapping
    odom_tf_publisher = LaunchConfiguration('odom_tf_publisher', default='ekf')
    map_tf_publisher = LaunchConfiguration('map_tf_publisher', default='amcl')
    launch_visualization = LaunchConfiguration('launch_visualization', default='True')
    external_odometry = LaunchConfiguration('external_odometry', default=False)
    rviz_config_file = LaunchConfiguration('rviz_config_file', default=rviz_config_path)
    launch_2d_mapping = LaunchConfiguration('launch_2d_mapping', default=False)
    launch_3d_mapping = LaunchConfiguration('launch_3d_mapping', default=True)
    life_long_mapping = LaunchConfiguration('life_long_mapping', default=False)
    offline_mapping_2d_param_file = LaunchConfiguration('offline_mapping_2d_param_file',
                                                        default=offline_mapping_2d_param_file_path)
    online_mapping_2d_param_file = LaunchConfiguration('online_mapping_2d_param_file',
                                                       default=online_mapping_2d_param_file_path)
    map_2d_file = LaunchConfiguration('default_2d_map_file', default=default_2d_map_file_path)
    rtabmap_database_file = LaunchConfiguration('rtabmap_database_file', default=rtabmap_database_file_path)

    require_deadman = LaunchConfiguration('require_deadman', default='True')
    deadman_buttons = LaunchConfiguration('deadman_buttons', default="[4, 9]")
    autonomous_deadman_buttons = LaunchConfiguration('autonomous_deadman_buttons', default="[5]")
    steering_button = LaunchConfiguration('steering_button', default=2)
    max_speed = LaunchConfiguration('max_speed', default=5.0)
    max_steering = LaunchConfiguration('max_steering', default=0.34)
    max_acceleration = LaunchConfiguration('max_acceleration', default=2.5)
    max_steering_rate = LaunchConfiguration('max_steering_rate', default=3.2)
    vesc_poll_rate = LaunchConfiguration('vesc_poll_rate', default=200.0)
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

    publish_map_to_odom_tf = LaunchConfiguration('publish_map_to_odom_tf', default=True)
    log_level = LaunchConfiguration('log_level', default='info')

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
            description='Use GPU acceleration for the 3D mapper: False=RTABMap (CPU), True=Nvblox (GPU).')

    use_gpu_for_localization_la = DeclareLaunchArgument(
            'use_gpu_for_localization',
            default_value='true',
            description='Enable Isaac VSLAM as the visual odometry source in the localization stack. '
                        'Independent of use_gpu, which controls only the 3D mapper. '
                        'Set False on machines without Isaac ROS installed.')

    declare_autostart_cmd = DeclareLaunchArgument(
            'autostart', default_value=autostart,
            description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
            'use_composition', default_value=use_composition,
            description='Whether to use composed bringup')

    declare_container_name_cmd = DeclareLaunchArgument(
            'container_name', default_value=container_name,
            description='Name of the container node used for composition.')

    declare_use_respawn_cmd = DeclareLaunchArgument(
            'use_respawn', default_value=use_respawn,
            description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
            'log_level', default_value='info',
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
    external_odometry_arg = DeclareLaunchArgument(
            'external_odometry', default_value=external_odometry,
            description="Odometry is already provided by a node this launch file did NOT start "
                        "(e.g. the parent's EKF). Suppresses mapping's own visual odometry and its "
                        "odom->base_link broadcast. Set True by any parent that starts the EKF "
                        "itself and therefore passes launch_local_localization:=False.")

    localize_isaac_vslam_on_startup_la = DeclareLaunchArgument(
            'localize_isaac_vslam_on_startup', default_value=localize_isaac_vslam_on_startup,
            description='Attempt to localize Isaac ROS VSLAM in a previously saved map on startup. '
                        'Set True only when a valid VSLAM map exists at visual_slam_map_path. '
                        'False (default) prevents the intermittent SIGABRT crash caused by GXF '
                        'heap corruption on localization failure.')

    launch_map_server_la = DeclareLaunchArgument(
            'launch_map_server', default_value=launch_map_server,
            description='Whether to launch the map server. '
                        'Defaults to False in mapping mode since a stale stored map should not be published '
                        'while building a new one.')

    launch_map_saver_la = DeclareLaunchArgument(
            'launch_map_saver', default_value=launch_map_saver,
            description='Whether to launch the map_saver_server for periodic 2D map auto-saving. '
                        'Defaults to True so the map is saved automatically during active 2D mapping. '
                        'Distinct from launch_map_server (which loads a pre-built map for navigation).')

    odom_tf_publisher_arg = DeclareLaunchArgument(
            'odom_tf_publisher', default_value=odom_tf_publisher,
            description='Node responsible for publishing the odom->base_link TF. '
                        'Options: ekf, vslam|stereo, rf2o, icp, rgbd, pointcloud, rtabmap.')
    map_tf_publisher_arg = DeclareLaunchArgument(
            'map_tf_publisher', default_value=map_tf_publisher,
            description='Node responsible for publishing the map->odom TF. '
                        'Options: amcl, pf, ekf, vslam, rtabmap, slam.')
    launch_visualization_arg = DeclareLaunchArgument('launch_visualization',
                                                     default_value=launch_visualization,
                                                     description="Launch RViz.")
    rviz_config_arg = DeclareLaunchArgument('rviz_config_file',
                                            default_value=rviz_config_file,
                                            description="The path to the rviz configuration file.")

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
            description='Close gate on heartbeat timeout. Set True and publish teleop to activate watchdog.')
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

    publish_map_to_odom_tf_la = DeclareLaunchArgument(
            'publish_map_to_odom_tf',
            default_value=publish_map_to_odom_tf,
            description='Whether to publish the map to the odometry frame transformation.'
    )

    # Add launch arguments to a list
    launch_args = [
        stdout_linebuf_envvar,
        declare_f1tenth_namespace_cmd,
        declare_use_f1tenth_namespace_cmd,
        declare_map_yaml_cmd,
        declare_use_sim_time_cmd,
        use_gpu_la,
        use_gpu_for_localization_la,
        declare_autostart_cmd,
        declare_use_composition_cmd,
        declare_container_name_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        launch_joystick_arg,
        launch_sensors_arg,
        launch_vehicle_arg,
        launch_tfs_arg,
        launch_localization_arg,
        launch_local_localization_arg,
        launch_global_localization_arg,
        external_odometry_arg,
        localize_isaac_vslam_on_startup_la,
        launch_map_server_la,
        launch_map_saver_la,
        odom_tf_publisher_arg,
        map_tf_publisher_arg,
        launch_visualization_arg,
        rviz_config_arg,
        launch_2d_mapping_arg,
        launch_3d_mapping_arg,
        life_long_mapping_arg,
        offline_mapping_2d_param_file_la,
        online_mapping_2d_param_file_la,
        map_2d_file_la,
        rtabmap_database_file_la,
        require_deadman_la, deadman_buttons_la, autonomous_deadman_buttons_la, steering_button_la, max_speed_la, max_steering_la,
        max_acceleration_la, max_steering_rate_la, vesc_poll_rate_la,
        declare_launch_ackermann_to_vesc_node, declare_launch_vesc_to_odom_node,
        declare_launch_throttle_interpolator_node, declare_launch_ackermann_to_twist,
        launch_command_gate_arg, command_gate_require_heartbeat_la, command_gate_require_enable_la,
        camera_name_la,
        approx_sync_la, stereo_to_pointcloud_la, depthimage_to_pointcloud_la,
        detect_ground_and_obstacles_la, reset_realsense_la, publish_realsense_pointcloud_la, align_realsense_depth_la,
        realsense_emitter_enabled_la, realsense_emitter_on_off_la, launch_realsense_splitter_node_la,
        realsense_qos_la,
        camera_launch_delay_la, laserscan_launch_delay_la,
        publish_map_to_odom_tf_la,
        gravitational_acceleration_la,
    ]

    ''' Launch Nodes '''
    # use OpaqueFunction to correctly get the runtime LaunchConfiguration.
    #  E.g ZED ROS2 wrapper |
    #  https://robotics.stackexchange.com/a/103368 |
    #  https://answers.ros.org/question/396345/ros2-launch-file-how-to-convert-launchargument-to-string/ |
    #  https://robotics.stackexchange.com/a/104402
    camera_name_string = camera_name.perform(context)
    realsense_qos_string = realsense_qos.perform(context)
    use_f1tenth_namespace_string = use_f1tenth_namespace.perform(context)
    f1tenth_namespace_string = f1tenth_namespace.perform(context)

    use_sim_time_string = use_sim_time.perform(context)
    life_long_mapping_string = life_long_mapping.perform(context)
    realsense_splitter_enabled_string = launch_realsense_splitter_node.perform(context)

    launch_localization_string = launch_localization.perform(context)
    launch_local_localization_string = launch_local_localization.perform(context)
    launch_global_localization_string = launch_global_localization.perform(context)
    launch_visualization_str = launch_visualization.perform(context)  # rtabmap expects strings

    use_composition_string = use_composition.perform(context)
    use_gpu_string = use_gpu.perform(context)
    use_gpu_for_localization_string = use_gpu_for_localization.perform(context)

    # whether this launch file is responsible for starting up odometry.
    #
    # 'launch_localization'/'launch_local_localization' answer "should I START localization?",
    # which is NOT the same question as "does odometry EXIST?". A parent that already started the
    # EKF itself passes them as False to avoid launching a second copy — and mapping then wrongly
    # concluded that nobody was providing odometry, stood up its own visual odometry, and
    # broadcast odom->base_link in competition with the parent's EKF (measured: 32.7 Hz on that
    # edge = ekf_odom 30 Hz + rgbd_odometry 2.9 Hz). 'external_odometry' carries the second
    # question explicitly; such a parent sets it True.
    enable_odom_here = True
    if launch_localization_string.lower() == 'true' and launch_local_localization_string.lower() == 'true':
        enable_odom_here = False
    if external_odometry.perform(context).lower() == 'true':
        enable_odom_here = False

    if camera_name_string != '':
        camera_name_string += '/'

    realsense_qos_int = qos_str_to_rtabmap_int.get(realsense_qos_string, 2)

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

    component_container_node = GroupAction(
            condition=IfCondition(use_composition),
            actions=[
                # PushRosNamespace(
                #         condition=IfCondition(use_f1tenth_namespace),
                #         namespace=f1tenth_namespace),
                SetRemap(src=['/tf'], dst=['tf']),
                SetRemap(src=['/tf_static'], dst=['tf_static']),
                SetParameter(name='use_sim_time', value=use_sim_time),
                SetParameter(name='thread_num', value=os.cpu_count()),
                # number of threads to use with component_container_mt
                Node(
                        condition=IfCondition(use_composition),
                        name=container_name,
                        package='rclcpp_components',
                        # https://docs.ros.org/en/humble/Concepts/Intermediate/About-Composition.html#componentcontainer
                        # https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html#component-container-types
                        # executables: 'component_container_mt', 'component_container_isolated', 'component_container'
                        executable='component_container_isolated',
                        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
                        arguments=[
                            '--use_multi_threaded_executor',
                            '--ros-args', '--log-level', log_level
                        ],
                        output='screen'),
            ]
    )

    teleop_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([f1tenth_launch_bringup_dir, 'teleop.launch.py'])
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "use_f1tenth_namespace": use_f1tenth_namespace,
                "f1tenth_namespace": f1tenth_namespace,
                "use_gpu": use_gpu_for_localization_string,
                "launch_joystick": launch_joystick,
                "launch_sensors": launch_sensors,
                "launch_vehicle": launch_vehicle,
                "launch_tfs": launch_tfs,
                "launch_localization": launch_localization,
                "launch_local_localization": launch_local_localization,
                "launch_global_localization": launch_global_localization,
                "launch_map_server": launch_map_server,
                "odom_tf_publisher": odom_tf_publisher,
                "map_tf_publisher": map_tf_publisher,
                "launch_visualization": 'False',
                "rviz_config_file": rviz_config_file,
                "require_deadman": require_deadman,
                "deadman_buttons": deadman_buttons,
                "autonomous_deadman_buttons": autonomous_deadman_buttons,
                "steering_button": steering_button,
                "max_speed": max_speed,
                "max_steering": max_steering,
                "launch_ackermann_to_vesc_node": launch_ackermann_to_vesc_node,
                "launch_vesc_to_odom_node": launch_vesc_to_odom_node,
                "launch_throttle_interpolator_node": launch_throttle_interpolator_node,
                "launch_ackermann_to_twist": launch_ackermann_to_twist,
                "max_acceleration": max_acceleration,
                "max_steering_rate": max_steering_rate,
                "vesc_poll_rate": vesc_poll_rate,
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
                "realsense_qos": realsense_qos,
                "use_composition": use_composition,
                "attach_to_shared_component_container": use_composition,  # this launch file starts a container
                "component_container_name": container_name if use_composition_string.lower() == 'true' else 'teleop_container',
                "gravitational_acceleration": gravitational_acceleration,
                "localize_isaac_vslam_on_startup": localize_isaac_vslam_on_startup,
                "launch_command_gate": launch_command_gate,
                "command_gate_require_heartbeat": command_gate_require_heartbeat,
                "command_gate_require_enable": command_gate_require_enable,
                "log_level": log_level,
            }.items()
    )

    # todo: setup lifelong mapping
    #  (https://github.com/SteveMacenski/slam_toolbox/blob/humble/launch/lifelong_launch.py).
    #  Use the online config but change the node
    mapping_2d_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([f1tenth_launch_bringup_dir, 'mapping', '2d_mapping.launch.py'])
            ),
            condition=IfCondition(launch_2d_mapping),
            launch_arguments={
                "namespace": '', # f1tenth_namespace,
                "use_namespace": 'False', # use_f1tenth_namespace,
                "offline_mapping": use_sim_time,
                "use_sim_time": use_sim_time,
                "launch_map_server": launch_map_saver,
                "offline_mapping_param_file": offline_mapping_2d_param_file,
                "online_mapping_param_file": online_mapping_2d_param_file,
                "map_file_path": map_2d_file,
                "map_topic": 'map',
                "with_rviz": launch_visualization
                # "rviz_cfg_path_param": rviz_config_path,
            }.items()
    )

    mapping_3d_queue_size = '10000'  # offline mapping
    if use_sim_time_string.lower() == 'false':
        # online_mapping
        mapping_3d_queue_size = '50'  # online mapping

    delete_old_map = '-d '
    if life_long_mapping_string.lower() == 'true':
        delete_old_map = ' '

    left_image_topic = camera_name_string + 'infra1/image_rect_raw'
    right_image_topic = camera_name_string + 'infra2/image_rect_raw'
    depth_topic = camera_name_string + 'aligned_depth_to_color/image_raw'  # /camera/camera/depth/image_rect_raw
    depth_info_topic = camera_name_string + 'aligned_depth_to_color/camera_info'  # /camera/camera/depth/camera_info
    if realsense_splitter_enabled_string.lower() == 'true':
        left_image_topic = camera_name_string + 'realsense_splitter_node/output/infra_1'
        right_image_topic = camera_name_string + 'realsense_splitter_node/output/infra_2'
        # todo: might need to launch RTABMaps depth realignment to color node if using realsense splitter
        depth_topic = camera_name_string + 'realsense_splitter_node/output/depth'
        depth_info_topic = camera_name_string + 'depth/camera_info'

    # See (https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h#L161)
    # Eagerly evaluate use_gpu_string here inside OpaqueFunction so the condition is baked as a
    # literal string ('True'/'False') before the launch system processes the action list.
    # Using LaunchConfiguration(use_gpu) as the condition instead would be evaluated *after*
    # OpaqueFunction returns — by which time IncludeLaunchDescription's launch_arguments for
    # teleop_launch (processed earlier in the ld list) will have mutated
    # context.launch_configurations['use_gpu'] via SetLaunchConfiguration, causing the condition
    # to see the wrong value.
    gpu_enabled = 'True' if use_gpu_string.lower() == 'true' else 'False'

    # RTABMap parameter profiles.
    # Offline (use_sim_time=True / rosbag): accuracy — full ICP, bundle adjustment, large buffers.
    # Online  (use_sim_time=False / live):  speed — trimmed features, BA off, tight time budget.
    _RTABMAP_COMMON = (
        # Bug fixes applied to both modes
        '--RGBD/CreateOccupancyGrid true '           # was false: grid disabled despite Grid/* params; loop corrections never updated the occupancy map
        '--GFTT/QualityLevel 0.001 '                 # was 0.00001: 100x too permissive; admitted noise keypoints
        '--Icp/MaxRotation 0.5 '                     # was 1.6 (~91deg): gate too wide; bad ICP solutions passed
        '--Optimizer/GravitySigma 0.3 '              # was 0 (disabled): gravity constraint from IMU reduces roll/pitch drift
        # Core settings shared between modes
        '--RGBD/LoopClosureReextractFeatures true '
        '--Rtabmap/CreateIntermediateNodes true '
        '--Vis/MinInliers 15 '
        '--Vis/EstimationType 0 '
        '--RGBD/OptimizeFromGraphEnd true '
        '--Vis/MaxDepth 5 '          # reduced from 6: D435i depth noise increases beyond 5m, cuts ghost points at far edges
        '--Kp/MaxDepth 5 '
        '--RGBD/LinearUpdate 0.1 '
        '--RGBD/AngularUpdate 0.1 '
        '--Stereo/MinDisparity 0.5 '
        '--Stereo/MaxDisparity 128.0 '
        '--Stereo/OpticalFlow true '
        '--Stereo/DenseStrategy 1 '
        '--Vis/PnPFlags 0 '
        '--Vis/CorType 0 '
        '--Reg/Force3DoF true '
        '--RGBD/NeighborLinkRefining true '
        '--RGBD/ProximityBySpace true '
        '--Reg/Strategy 2 '          # VisIcp: visual features estimate initial transform (handles odometry drift), ICP refines; ICP-only (1) fails loop closure when accumulated drift > MaxCorrespondenceDistance
        '--Icp/MaxCorrespondenceDistance 0.15 '
        '--Icp/OutlierRatio 0.75 '
        '--Icp/CorrespondenceRatio 0.2 '
        '--Icp/Strategy 1 '
        '--Icp/PointToPlaneMinComplexity 0.04 '
        '--Icp/Force4DoF true '
        '--Rtabmap/DetectionRate 0 '
        '--Grid/Sensor 0 '          # LiDAR-only for 2D occupancy: more accurate than depth for ground-plane mapping
        '--Grid/RangeMax 10.0 '
        '--Grid/RangeMin 0.2 '
        '--Grid/RayTracing True '
        '--Optimizer/Strategy 2 '
        '--Optimizer/Slam2D true '
        '--Kp/DetectorStrategy 1 '
        '--Vis/FeatureType 1 '
        '--Mem/NotLinkedNodesKept false '
    )
    _RTABMAP_OFFLINE = _RTABMAP_COMMON + (
        # Accuracy mode: maximize loop closure reliability and map density; no time budget
        '--Kp/MaxFeatures 2000 '                     # up from 1000: denser 3D map and more loop closure candidates per node
        '--Mem/STMSize 100 '                          # up from 50: at 10cm spacing, 50 nodes = 5m; 100 nodes keeps ~10m in working memory for loop closure without LTM retrieval
        '--RGBD/LocalRadius 8 '                       # up from 5: wider spatial proximity search for loop candidates
        '--RGBD/ProximityPathMaxNeighbors 20 '        # up from 10: check more path neighbors during proximity detection
        '--Icp/Iterations 100 '
        '--Icp/MaxTranslation 1.0 '
        '--Rtabmap/ImageBufferSize 10000 '            # large buffer safe for bag replay bursts
        '--Grid/NoiseFilteringMinNeighbors 10 '       # up from 8: stricter occupancy noise filter
        '--Grid/NoiseFilteringRadius 0.15 '           # up from 0.1
        '--Vis/BundleAdjustment 1 '
        '--Rtabmap/TimeThr 0 '                        # no per-iteration time cap offline
        '--RGBD/OptimizeMaxError 6 '                  # up from 4: accumulated drift over 3 loops can push a valid closure above 4 Mahalanobis; 6 is more tolerant without accepting gross errors
        '--Rtabmap/LoopThr 0.05 '                     # default 0.11: more aggressive BoW retrieval so marginal visual candidates reach registration rather than being discarded early
    )
    _RTABMAP_ONLINE = _RTABMAP_COMMON + (
        # Speed mode: stay real-time on Jetson; trade accuracy for throughput
        '--Kp/MaxFeatures 500 '
        '--Mem/STMSize 30 '
        '--RGBD/LocalRadius 4 '
        '--RGBD/ProximityPathMaxNeighbors 5 '
        '--Icp/Iterations 30 '
        '--Icp/MaxTranslation 0.5 '
        '--Rtabmap/ImageBufferSize 20 '               # small buffer: prevent real-time lag
        '--Grid/NoiseFilteringMinNeighbors 8 '
        '--Grid/NoiseFilteringRadius 0.1 '
        '--Vis/BundleAdjustment 0 '                   # disabled: BA too slow online
        '--Rtabmap/TimeThr 700 '                      # 700ms per-iteration budget
        '--RGBD/OptimizeMaxError 4 '
        '--Rtabmap/LoopThr 0.11 '                     # default: standard threshold online
    )
    rtabmap_profile = _RTABMAP_OFFLINE if use_sim_time_string.lower() == 'true' else _RTABMAP_ONLINE

    mapping_3d_cpu_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([f1tenth_launch_bringup_dir, 'mapping', '3d_mapping.launch.py'])
            ),
            condition=UnlessCondition(gpu_enabled),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "namespace": f1tenth_namespace if use_f1tenth_namespace_string.lower() == 'true' else '',
                "use_stereo": 'false',  # False=use_depth + color, True=use_stereo
                "localization": 'false',
                "queue_size": mapping_3d_queue_size,
                "publish_map_tf": publish_map_to_odom_tf,
                "publish_odom_tf": str(enable_odom_here).lower(),  # set to False if some other node, e.g robot_localization EKF is publishing
                "visual_odometry": str(enable_odom_here).lower(),
                "icp_odometry": 'false',
                "wait_imu_to_init": 'true',
                "imu_topic": camera_name_string + 'imu/filtered',
                "left_image_topic": left_image_topic,
                "right_image_topic": right_image_topic,
                "left_camera_info_topic": camera_name_string + 'infra1/camera_info',
                "right_camera_info_topic": camera_name_string + 'infra2/camera_info',
                "rgb_topic": camera_name_string + 'color/image_raw',  # camera_name_string + 'color/image_raw', left_image_topic
                "depth_topic": depth_topic,  # depth_topic, camera_name_string + 'depth/image_rect_raw'
                "odom_topic": 'rtabmap/odom' if enable_odom_here else 'odometry/local',
                "camera_info_topic": camera_name_string + 'color/camera_info',  # 'color/camera_info', 'infra1/camera_info'
                "scan_topic": 'lidar/scan_filtered',
                "approx_sync": 'true',
                "rtabmap_viz_view": launch_visualization_str,
                "rviz_view": 'false',  # launch_visualization
                "database_path": rtabmap_database_file,
                "qos": "1",
                "qos_scan": "1",
                # BUG-021: rgbd_odometry publishes rtabmap/odom with its own `qos` param, which
                # upstream rtabmap.launch.py ties to qos_image (=2/BEST_EFFORT for the RealSense).
                # A RELIABLE subscription here is an incompatible-QoS match, so DDS delivers
                # nothing and rtabmap's synchroniser never fires. Must track qos_image.
                "qos_odom": "2",
                "qos_image": str(realsense_qos_int),
                "qos_camera_info": str(realsense_qos_int),
                "qos_imu": str(realsense_qos_int),
                "rtabmap_args": f'{delete_old_map}{rtabmap_profile}',
            }.items()
    )

    mapping_3d_gpu_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([nvidia_isaac_launch_dir, 'isaac_ros_nvblox.launch.py'])
            ),
            condition=IfCondition(gpu_enabled),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "use_namespace": use_f1tenth_namespace,
                "camera_name": '', # camera_name,
                "namespace": f1tenth_namespace,
                "global_frame": 'odom',  # only set to map if another node is publishing the map frame
                "launch_realsense_driver": 'False',
                "launch_realsense_splitter": 'False',
                # "publish_map_tf": 'True',
                "depth_topic": depth_topic,
                "depth_info_topic": depth_info_topic,
                "left_image_topic": left_image_topic,
                "right_image_topic": right_image_topic,
                "input_qos": realsense_qos,
                "remove_dynamic_objects": 'False',
                "remove_people": 'False',
                "launch_visual_slam": str(enable_odom_here),
                "attach_to_shared_component_container": use_composition,
                "component_container_name": container_name if use_composition_string.lower() == 'true' else 'nvblox_container',
            }.items()
    )

    mapping_3d_group = GroupAction(
            condition=IfCondition(launch_3d_mapping),
            actions=[
                # set common parameters
                SetParameter(name='use_sim_time', value=use_sim_time),
                SetRemap(src=['/tf'], dst=['tf']),
                SetRemap(src=['/tf_static'], dst=['tf_static']),

                # nodes
                mapping_3d_cpu_node,
                mapping_3d_gpu_node
            ]
    )

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
                mapping_2d_node,
            ]
    )  # append F1/10 namespace to all nodes

    ld = launch_args + [
                nodes_to_launch,
                teleop_launch,
                mapping_3d_group
            ]
    return ld


def generate_launch_description():
    return LaunchDescription(
            [
                SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
                OpaqueFunction(function=launch_setup)
            ]
    )
    