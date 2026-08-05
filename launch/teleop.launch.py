"""
Launches: vehicle driver, joystick driver, mux, and sensors
Todo:
    * Launch the container here or in bringup.launch.py or mapping.launch.py. See (https://github.com/ros-navigation/navigation2/blob/humble_main/nav2_bringup/launch/bringup_launch.py#L154)
"""
import os
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, SetRemap, PushRosNamespace, SetParametersFromFile, SetParameter
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, OpaqueFunction, SetEnvironmentVariable, LogInfo, TimerAction
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    qos_str_to_rtabmap_int = {
        'SENSOR_DATA': 2,
        'SYSTEM_DEFAULT': 0,
        'DEFAULT': 1,
    }

    # Get package directories
    f1tenth_launch_dir = get_package_share_directory('f1tenth_launch')

    # Get launch directories
    vehicle_include_dir = os.path.join(f1tenth_launch_dir, 'launch', 'vehicle')
    sensor_include_dir = os.path.join(f1tenth_launch_dir, 'launch', 'sensors')
    localization_include_dir = os.path.join(f1tenth_launch_dir, 'launch', 'localization')
    rviz_config_path = os.path.join(f1tenth_launch_dir, 'config', 'f1tenth.rviz')
    localization_params_file_path = os.path.join(f1tenth_launch_dir, 'config', 'localization', 'localizer_amcl.yaml')

    # Declare launch configuration variables
    use_composition = LaunchConfiguration('use_composition', default='True')  # True
    container_name = LaunchConfiguration('container_name', default='f1tenth_container')
    attach_to_shared_component_container = LaunchConfiguration('attach_to_shared_component_container', default='False')
    f1tenth_namespace = LaunchConfiguration('f1tenth_namespace',
                                            default='')  # used to distinguish between multiple F1/10s
    use_f1tenth_namespace = LaunchConfiguration('use_f1tenth_namespace', default=True)  # True
    use_sim_time = LaunchConfiguration('use_sim_time', default="False")
    use_gpu = LaunchConfiguration('use_gpu', default=True)
    launch_joystick = LaunchConfiguration('launch_joystick', default=True)
    launch_sensors = LaunchConfiguration('launch_sensors', default=True)
    launch_vehicle = LaunchConfiguration('launch_vehicle', default=True)
    launch_tfs = LaunchConfiguration('launch_tfs', default=True)
    launch_localization = LaunchConfiguration('launch_localization', default=True)
    launch_local_localization = LaunchConfiguration('launch_local_localization', default=True)
    launch_global_localization = LaunchConfiguration('launch_global_localization', default=False)
    # Off by default, matching localization.launch.py and bringup.launch.py. This
    # was previously hardcoded 'True' in the localization include below, so
    # RTABMap ICP odometry ran on every teleop launch and could not be turned
    # off from the command line. With it on, ekf_odom misses its update rate
    # (measured: odometry/local at 12.78 Hz with 2.44 s gaps, versus 29.70 Hz
    # and a 0.180 s max gap with it off). rf2o covers the same job far cheaper.
    launch_icp_odometry = LaunchConfiguration('launch_icp_odometry', default=False)
    localize_isaac_vslam_on_startup = LaunchConfiguration('localize_isaac_vslam_on_startup', default=True)
    launch_map_server = LaunchConfiguration('launch_map_server', default=True)
    odom_tf_publisher = LaunchConfiguration('odom_tf_publisher', default='ekf')  # ekf
    map_tf_publisher = LaunchConfiguration('map_tf_publisher', default='vslam')  # amcl
    launch_visualization = LaunchConfiguration('launch_visualization', default=False)
    rviz_config_file = LaunchConfiguration('rviz_config_file', default=rviz_config_path)
    launch_pointcloud_map = LaunchConfiguration('launch_pointcloud_map', default=False)
    pointcloud_map_file = LaunchConfiguration('pointcloud_map_file', default=os.path.join(
            f1tenth_launch_dir, 'data', 'maps', 'rtabmap', 'raslab', 'cloud.pcd'))

    require_deadman = LaunchConfiguration('require_deadman', default='True')
    deadman_buttons = LaunchConfiguration('deadman_buttons', default="[4, 9]")
    autonomous_deadman_buttons = LaunchConfiguration('autonomous_deadman_buttons', default="[10]")  # SDL DualSense: R1=10 (PS/guide=5 — never use 5, it's the power-off button)
    steering_button = LaunchConfiguration('steering_button', default=2)
    max_speed = LaunchConfiguration('max_speed', default=5.0)
    max_steering = LaunchConfiguration('max_steering', default=0.34)
    max_acceleration = LaunchConfiguration('max_acceleration', default=2.5)
    max_steering_rate = LaunchConfiguration('max_steering_rate', default=3.2)
    vesc_poll_rate = LaunchConfiguration('vesc_poll_rate', default=50.0)  # 200.0Hz
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

    map_file = LaunchConfiguration('map_file', default=os.path.join(
            get_package_share_directory('f1tenth_launch'), 'data/maps', 'raslab.yaml'))

    camera_name = LaunchConfiguration('camera_name', default='camera')
    approx_sync = LaunchConfiguration('approx_sync', default='True')
    stereo_to_pointcloud = LaunchConfiguration('stereo_to_pointcloud', default='False')
    depthimage_to_pointcloud = LaunchConfiguration('depthimage_to_pointcloud', default='False')
    detect_ground_and_obstacles = LaunchConfiguration('detect_ground_and_obstacles', default='False')

    reset_realsense = LaunchConfiguration('reset_realsense', default='False')
    publish_realsense_pointcloud = LaunchConfiguration('publish_realsense_pointcloud', default='False')  # use this until I port over my accelerated cv package
    align_realsense_depth = LaunchConfiguration('align_realsense_depth', default='True')
    realsense_emitter_enabled = LaunchConfiguration('realsense_emitter_enabled', default='0')
    realsense_emitter_on_off = LaunchConfiguration('realsense_emitter_on_off', default='False')
    launch_realsense_splitter_node = LaunchConfiguration('launch_realsense_splitter_node', default=False)
    realsense_qos = LaunchConfiguration('realsense_qos', default='SENSOR_DATA')

    camera_launch_delay = LaunchConfiguration('camera_launch_delay', default='6.0')
    laserscan_launch_delay = LaunchConfiguration('laserscan_launch_delay', default='2.0')
    log_level = LaunchConfiguration('log_level', default='info')

    # Declare launch arguments
    declare_use_composition_cmd = DeclareLaunchArgument(
            'use_composition', default_value=use_composition,
            description='Whether to use composed bringup')
    declare_container_name_cmd = DeclareLaunchArgument(
            'container_name', default_value=container_name,
            description='Name of the container node used for composition.')
    declare_attach_to_shared_component_container_cmd = DeclareLaunchArgument(
            'attach_to_shared_component_container', default_value=attach_to_shared_component_container,
            description='Whether to attach to a shared component container')
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

    use_gpu_la = DeclareLaunchArgument(
            'use_gpu', default_value=use_gpu,
            description='Use GPU acceleration. Default: True')

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
    launch_icp_odometry_arg = DeclareLaunchArgument('launch_icp_odometry',
                                                    default_value=launch_icp_odometry,
                                                    description="Launch RTABMap ICP odometry (odom/rtabmap/icp, the EKF's "
                                                                "odom3). Expensive; rf2o is the preferred LiDAR odometry.")

    localize_isaac_vslam_on_startup_la = DeclareLaunchArgument(
            'localize_isaac_vslam_on_startup', default_value=localize_isaac_vslam_on_startup,
            description='Attempt to localize Isaac ROS VSLAM in a previously saved map on startup. '
                        'Set True only when a valid VSLAM map exists at visual_slam_map_path. '
                        'False (default) prevents the intermittent SIGABRT crash caused by GXF '
                        'heap corruption on localization failure.')

    launch_map_server_la = DeclareLaunchArgument(
            'launch_map_server', default_value=launch_map_server,
            description='Whether to launch the map server. '
                        'Disable when building a new map to prevent a stale stored map from being published.')

    odom_tf_publisher_la = DeclareLaunchArgument(
            'odom_tf_publisher', default_value=odom_tf_publisher,
            description='The node responsible for publishing the odometry tf. '
                        'Options: ekf, vslam|stereo, rf2o, icp, rgbd, pointcloud, rtabmap.')

    map_tf_publisher_la = DeclareLaunchArgument(
            'map_tf_publisher', default_value=map_tf_publisher,
            description='The node responsible for publishing the map tf. '
                        'Options: pf, amcl, ekf, vslam, rtabmap, slam (for slam_toolbox).')

    launch_visualization_arg = DeclareLaunchArgument('launch_visualization',
                                                     default_value=launch_visualization,
                                                     description="Launch RViz.")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true')

    rviz_config_arg = DeclareLaunchArgument('rviz_config_file',
                                            default_value=rviz_config_file,
                                            description="The path to the rviz configuration file.")

    launch_pointcloud_map_arg = DeclareLaunchArgument(
            'launch_pointcloud_map', default_value=launch_pointcloud_map,
            description='Publish a static RTABMap-exported point cloud (.pcd) on map/pointcloud '
                        'for RViz visualization only. Not used for localization or planning.')
    pointcloud_map_file_arg = DeclareLaunchArgument(
            'pointcloud_map_file', default_value=pointcloud_map_file,
            description='Path to the .pcd point cloud map published when launch_pointcloud_map:=True.')

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
            description='Close gate on heartbeat timeout. Set True and publish teleop to activate watchdog.')
    command_gate_require_enable_la = DeclareLaunchArgument(
            'command_gate_require_enable',
            default_value=command_gate_require_enable,
            description='Gate starts closed; open with: '
                        'ros2 service call /command_gate/set_enabled std_srvs/srv/SetBool \'{data: true}\'')

    gravitational_acceleration_la = DeclareLaunchArgument(
            'gravitational_acceleration', default_value=gravitational_acceleration,
            description='Gravitational acceleration (m/s^2). Calibrate per robot.')

    map_file_la = DeclareLaunchArgument(
            'map_file', default_value=map_file,
            description='Path to the 2D map YAML file used by AMCL and the map server.')

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

    declare_log_level_cmd = DeclareLaunchArgument(
            'log_level', default_value='info',
            description='log level')

    launch_args = [
        declare_use_composition_cmd,
        declare_container_name_cmd,
        declare_attach_to_shared_component_container_cmd,
        declare_f1tenth_namespace_cmd,
        declare_use_f1tenth_namespace_cmd,
        use_gpu_la,
        launch_joystick_arg,
        launch_sensors_arg,
        launch_vehicle_arg,
        launch_tfs_arg,
        launch_localization_arg,
        launch_local_localization_arg,
        launch_icp_odometry_arg,
        launch_global_localization_arg,
        localize_isaac_vslam_on_startup_la,
        launch_map_server_la,
        odom_tf_publisher_la,
        map_tf_publisher_la,
        launch_visualization_arg,
        declare_use_sim_time_cmd,
        rviz_config_arg,
        launch_pointcloud_map_arg, pointcloud_map_file_arg,
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
        map_file_la,
        declare_log_level_cmd,
    ]

    # Launch nodes
    camera_name_string = camera_name.perform(context)
    realsense_qos_string = realsense_qos.perform(context)
    use_f1tenth_namespace_string = use_f1tenth_namespace.perform(context)
    f1tenth_namespace_string = f1tenth_namespace.perform(context)
    use_composition_string = use_composition.perform(context)
    attach_to_shared_component_container_string = attach_to_shared_component_container.perform(context)
    container_name_string = container_name.perform(context)
    odom_tf_publisher_string = odom_tf_publisher.perform(context)
    map_tf_publisher_string = map_tf_publisher.perform(context)

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
                raise NameError(
                        'The launch argument "use_f1tenth_namespace" was set to "true" '
                        'but the launch argument "f1tenth_namespace" is empty. '
                        'Set the launch argument "f1tenth_namespace" to the namespace you want to use. '
                        'If you do not want to use a namespace, do not set the launch argument "use_f1tenth_namespace".'
                )

    component_container_node = LogInfo(msg=f'Not launching container in teleop.launch.py.')
    if use_composition_string.lower() == 'true' and attach_to_shared_component_container_string.lower() == 'false':
        component_container_node = GroupAction(
                actions=[
                    SetParameter(name='thread_num', value=os.cpu_count()),  # number of threads to use with component_container_mt
                    Node(
                            condition=IfCondition(use_composition),
                            name=container_name,
                            package='rclcpp_components',
                            # executables: 'component_container_mt', 'component_container_isolated', 'component_container'
                            executable='component_container_isolated',
                            arguments=[
                                '--use_multi_threaded_executor',
                                '--ros-args', '--log-level', log_level
                            ],
                            output='screen'),
                ]
        )

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
            condition=IfCondition(launch_joystick),
            launch_arguments={
                'use_f1tenth_namespace': use_f1tenth_namespace,
                'f1tenth_namespace': f1tenth_namespace,
            }.items()
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
                "approx_sync": approx_sync,
                "use_namespace": use_f1tenth_namespace,
                "namespace": f1tenth_namespace,
                "camera_name": camera_name,
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
                "use_composition": use_composition,
                # 'sensing_container' is owned and created by realsense_d435i.launch.py, so never
                # ask the sensors include to attach to it. Passing 'True' here (the old behaviour
                # whenever use_composition was set) told every creator to stand down: teleop only
                # ever creates `container_name` ('f1tenth_container'), so nothing created
                # 'sensing_container' and LoadComposableNodes waited forever on a service that
                # never appeared — silently, with no error. The camera and the stereo/depth
                # pipeline simply never loaded (bug-015). bringup.launch.py leaves these two
                # arguments unset for the same reason; keep the two entry points identical.
                "attach_to_shared_component_container": 'False',
                "component_container_name": 'sensing_container',  # hardcoded so it stays separate from the others
                "intra_process_comms": 'True',
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
                IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                                PathJoinSubstitution([localization_include_dir, 'localization.launch.py'])
                        ),
                        condition=IfCondition(launch_localization),
                        launch_arguments={
                            "use_composition": use_composition,
                            "container_name": container_name if (use_composition_string.lower() == 'true' or attach_to_shared_component_container_string.lower() == 'true') else 'localization_container',
                            # This launch file already creates `container_name` (see the
                            # component_container_node above), so localization must ATTACH to it
                            # rather than create a second container under the same name.
                            # localization.launch.py creates one under
                            # `IfCondition(use_composition and not attach_to_shared_component_container)`,
                            # and this argument used to be inherited rather than passed — it only
                            # happened to be 'True' because the sensors include, visited earlier in
                            # `ld`, leaked it sideways. Pass it explicitly (bug-013's lesson).
                            "attach_to_shared_component_container": 'True' if (use_composition_string.lower() == 'true' or attach_to_shared_component_container_string.lower() == 'true') else 'False',
                            "namespace": f1tenth_namespace,
                            "use_namespace": use_f1tenth_namespace,
                            "camera_name": camera_name,
                            "map_file": map_file,
                            # Pass the localization param file explicitly. teleop declares no
                            # params_file of its own, so standalone it works — but when teleop is
                            # nested under a parent that sets params_file (e.g. bringup->mapping->teleop,
                            # where params_file=nav2_params.yaml), that value would leak into this
                            # include and AMCL would get nav2_params.yaml (scan_topic: scan) and never
                            # localize. Pinning localizer_amcl.yaml here matches the bringup fix.
                            "params_file": localization_params_file_path,
                            "launch_sensor_fusion": 'True' if ('ekf' in map(str.lower, [odom_tf_publisher_string, map_tf_publisher_string])) else 'False',
                            "launch_ekf_odom": launch_local_localization,
                            "launch_ekf_map": launch_global_localization,
                            "odom_tf_publisher": odom_tf_publisher,  # ekf, vslam|stereo, icp, rgbd, pointcloud
                            "map_tf_publisher": map_tf_publisher,  # amcl, ekf, vslam, rtabmap
                            "launch_slam_toolbox_localizer": 'False',
                            "launch_rtabmap_localizer": 'False',
                            'launch_pointcloud_odometry': 'False',
                            'launch_rgbd_odometry': 'False',
                            'launch_stereo_odometry': 'True',
                            'launch_laserscan_odometry': 'True',
                            'launch_icp_odometry': launch_icp_odometry,
                            'launch_amcl': 'True',  # launch_global_localization,
                            'launch_particle_filter': 'False',  # launch_global_localization
                            'launch_map_server': launch_map_server,
                            'localize_on_startup': localize_isaac_vslam_on_startup,
                            # "map_file": map_file,
                            "use_sim_time": use_sim_time,
                            "use_gpu": use_gpu,
                            "qos_rtabmap": "1",
                            "qos_rtabmap_laserscan": "1",
                            "qos_rtabmap_camera": str(realsense_qos_int),
                            "qos_rtabmap_imu": str(realsense_qos_int),
                            "qos": realsense_qos,
                            "qos_imu": realsense_qos,
                            "gravitational_acceleration": gravitational_acceleration,
                            "log_level": log_level,
                        }.items()
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
                joystick_launch,
                ackermann_mux_launch,
                command_gate_launch,
                command_gate_disabled_warning,
                vehicle_launch,
                tf_launch,
                visualization_launch,
                pointcloud_map_launch,
            ]
    )  # append F1/10 namespace to all nodes

    # return the launch description
    ld = launch_args + [
        nodes_to_launch,
        # nodes launched separate from the GroupAction due to namespace issues caused by nesting
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
