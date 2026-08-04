"""
Currently untested with "namespace". Might fail if a namespace is specified
todo:
    * setup joining containers for visual slam and occupancy grid, i.e resolve container names
    * run tests and remove todos below
todo: load nodes using composition (https://github.com/ros-planning/navigation2/blob/humble/nav2_bringup/launch/localization_launch.py#L145)
This node sets up local and global localization.
* Load map (yaml and/or posegraph)
* Launch AMCL localizer
* Launch slam_toolbox localizer
* (optional) Launch particle_filter localizer
* Launch RTABMap localizer
* Launch Kalman Filter (EKF or UKF) nodes

"""
import os
import pathlib
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction, SetEnvironmentVariable, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes, SetRemap, PushRosNamespace, SetParametersFromFile, SetParameter
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString
from ament_index_python import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError as ROS2PackageNotFoundError


def launch_setup(context, *args, **kwargs):
    # Get path to files and directories
    nav2_pkg_prefix = get_package_share_directory('nav2_bringup')
    f1tenth_launch_pkg_prefix = get_package_share_directory('f1tenth_launch')

    # Get launch directories
    nvidia_isaac_launch_dir = os.path.join(f1tenth_launch_pkg_prefix, 'launch', 'nvidia_isaac_ros')

    # Setup default directories
    localization_param_file = os.path.join(
            f1tenth_launch_pkg_prefix, 'config', 'localization/localizer_amcl.yaml')
    map_file_path = os.path.join(
            f1tenth_launch_pkg_prefix, 'data/maps', 'raslab.yaml')
    rtabmap_database_file_path = os.path.join(f1tenth_launch_pkg_prefix, 'data', 'maps', 'rtabmap', 'rtabmap.db')
    # ekf_param_file = os.path.join(
    #         f1tenth_launch_pkg_prefix, 'config', '/ekf.yaml')
    
    particle_filter_config_file = os.path.join(
            f1tenth_launch_pkg_prefix, 'config', 'localization', 'localizer_pf.yaml')

    # declare launch configurations
    namespace = LaunchConfiguration('namespace', default='')
    use_namespace = LaunchConfiguration('use_namespace', default=False)
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    use_composition = LaunchConfiguration('use_composition', default=True)
    container_name = LaunchConfiguration('container_name', default='localization_container')
    attach_to_shared_component_container = LaunchConfiguration('attach_to_shared_component_container', default='False')
    autostart = LaunchConfiguration('autostart', default=True)
    use_respawn = LaunchConfiguration('use_respawn', default=False)
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map_file', default=map_file_path)
    launch_slam_toolbox_localizer = LaunchConfiguration('launch_slam_toolbox_localizer', default=False)
    launch_sensor_fusion = LaunchConfiguration('launch_sensor_fusion', default=True)
    launch_ekf_odom = LaunchConfiguration('launch_ekf_odom', default=True)
    launch_ekf_map = LaunchConfiguration('launch_ekf_map', default=False)
    launch_particle_filter = LaunchConfiguration('launch_particle_filter', default=False)
    particle_filter_config = LaunchConfiguration('particle_filter_config', default=particle_filter_config_file)
    odom_frequency = LaunchConfiguration('odom_frequency', default=30.0)  # 50.0
    map_frequency = LaunchConfiguration('map_frequency', default=10.0)
    launch_rtabmap_localizer = LaunchConfiguration('launch_rtabmap_localizer', default=False)
    rtabmap_database_file = LaunchConfiguration('rtabmap_database_file', default=rtabmap_database_file_path)
    log_level = LaunchConfiguration('log_level')

    launch_pointcloud_odometry = LaunchConfiguration('launch_pointcloud_odometry', default='False')
    launch_rgbd_odometry = LaunchConfiguration('launch_rgbd_odometry', default='False')
    launch_stereo_odometry = LaunchConfiguration('launch_stereo_odometry', default='True')
    visual_slam_map_path = LaunchConfiguration('visual_slam_map_path', default='/mnt/data/maps/nvidia/vslam_map')
    launch_laserscan_odometry = LaunchConfiguration('launch_laserscan_odometry', default='True')
    launch_icp_odometry = LaunchConfiguration('launch_icp_odometry', default='False')  # RTABMap ICP; expensive, disabled by default in favour of rf2o
    launch_amcl = LaunchConfiguration('launch_amcl', default='True')
    launch_map_server = LaunchConfiguration('launch_map_server', default=True)
    localize_on_startup = LaunchConfiguration('localize_on_startup', default=True)
    odom_tf_publisher = LaunchConfiguration('odom_tf_publisher', default='ekf')
    map_tf_publisher = LaunchConfiguration('map_tf_publisher', default='amcl')

    base_frame = LaunchConfiguration('base_frame', default='base_link')
    odom_frame = LaunchConfiguration('odom_frame', default='odom')

    use_gpu = LaunchConfiguration('use_gpu', default='True')
    qos_rtabmap = LaunchConfiguration('qos_rtabmap', default=1)
    qos_rtabmap_camera = LaunchConfiguration('qos_rtabmap_camera', default=2)
    qos_rtabmap_imu = LaunchConfiguration('qos_rtabmap_imu', default=2)
    qos_rtabmap_laserscan = LaunchConfiguration('qos_rtabmap_laserscan', default=1)
    qos = LaunchConfiguration('qos', default='SENSOR_DATA')
    qos_imu = LaunchConfiguration('qos_imu', default='SENSOR_DATA')

    camera_name = LaunchConfiguration('camera_name', default='camera')
    scan_prefix = LaunchConfiguration('scan_prefix', default='lidar/')
    gravitational_acceleration = LaunchConfiguration('gravitational_acceleration', default='9.80665')

    # Declare default launch arguments
    stdout_linebuf_envvar = SetEnvironmentVariable(
            'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    declare_namespace_cmd = DeclareLaunchArgument(
            'namespace',
            default_value=namespace,
            description='Top-level namespace')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true')
    localization_param = DeclareLaunchArgument(
            'params_file',
            default_value=localization_param_file,
            description='Path to config file for localization nodes'
    )
    map_file_la = DeclareLaunchArgument(
            'map_file',
            default_value=map_file,
            description='Path to 2D map config file'
    )
    launch_slam_toolbox_localizer_la = DeclareLaunchArgument(
            'launch_slam_toolbox_localizer',
            default_value=launch_slam_toolbox_localizer,
            description='Whether to launch slam toolbox\'s localizer'
    )
    launch_sensor_fusion_la = DeclareLaunchArgument(
            'launch_sensor_fusion',
            default_value=launch_sensor_fusion,
            description='Whether to launch either EKF/UKF node.'
    )
    launch_ekf_odom_la = DeclareLaunchArgument(
            'launch_ekf_odom',
            default_value=launch_ekf_odom,
            description='Whether to launch the local/odom EKF/UKF node.'
    )
    launch_ekf_map_la = DeclareLaunchArgument(
            'launch_ekf_map',
            default_value=launch_ekf_map,
            description='Whether to launch the global/map EKF/UKF node.'
    )
    launch_particle_filter_la = DeclareLaunchArgument(
            'launch_particle_filter',
            default_value=launch_particle_filter,
            description='Whether to launch the particle filter node.'
    )
    particle_filter_config_la = DeclareLaunchArgument(
            'particle_filter_config',
            default_value=particle_filter_config,
            description='Path to particle filter config file'
    )
    odom_frequency_la = DeclareLaunchArgument(
            'odom_frequency',
            default_value=odom_frequency,  # 100.0
            description='Local/odom EKF/UKF node update/publish frequency.'
    )
    map_frequency_la = DeclareLaunchArgument(
            'map_frequency',
            default_value=map_frequency,  # 30.0
            description='Global/map EKF/UKF node update/publish frequency.'
    )
    launch_rtabmap_localizer_la = DeclareLaunchArgument(
            'launch_rtabmap_localizer',
            default_value=launch_rtabmap_localizer,
            description='Whether to launch the RTABMaps global localizer node.'
    )
    rtabmap_database_file_la = DeclareLaunchArgument('rtabmap_database_file',
                                                     default_value=rtabmap_database_file,
                                                     description="Path to the config file for the 3D mapping node.")
    declare_use_composition_cmd = DeclareLaunchArgument(
            'use_composition', default_value=use_composition,  # set to True
            description='Use composed bringup if True')
    declare_container_name_cmd = DeclareLaunchArgument(
            'container_name', default_value=container_name,
            description='the name of conatiner that nodes will load in if use composition')
    declare_attach_to_shared_component_container_cmd = DeclareLaunchArgument(
            'attach_to_shared_component_container', default_value=attach_to_shared_component_container,
            description='Whether to attach to a shared component container or launch a new one.')

    declare_autostart_cmd = DeclareLaunchArgument(
            'autostart', default_value=autostart,
            description='Automatically startup the nav2 stack')
    declare_use_respawn_cmd = DeclareLaunchArgument(
            'use_respawn', default_value=use_respawn,
            description='Whether to respawn if a node crashes. Applied when composition is disabled.')
    declare_log_level_cmd = DeclareLaunchArgument(
            'log_level', default_value='info',
            description='log level')

    launch_pointcloud_odometry_la = DeclareLaunchArgument(
            'launch_pointcloud_odometry', default_value=launch_pointcloud_odometry,
            description='Whether to launch point to point (or plane) odometry.')

    launch_rgbd_odometry_la = DeclareLaunchArgument(
            'launch_rgbd_odometry', default_value=launch_rgbd_odometry,
            description='Whether to launch RGB-D odometry.')

    launch_stereo_odometry_la = DeclareLaunchArgument(
            'launch_stereo_odometry', default_value=launch_stereo_odometry,
            description='Whether to launch stereo odometry. '
                        'For realsense cameras, this is probably more accurate than RGB-D')

    visual_slam_map_path_la = DeclareLaunchArgument(
            'visual_slam_map_path', default_value=visual_slam_map_path,
            description="Path to save/load the visual slam map to/from. "
                                                     "Examples: "
                                                     "/mnt/shared_dir/maps/nvidia/vslam_map, "
                                                     "/mnt/data/maps/nvidia/vslam_map, "
                                                     "/f1tenth_ws/src/f1tenth_launch/data/maps/nvidia/vslam_map, "
                                                     "/shared_dir/maps/nvidia/vslam_map")

    launch_laserscan_odometry_la = DeclareLaunchArgument(
            'launch_laserscan_odometry', default_value=launch_laserscan_odometry,
            description='Whether to launch laserscan odometry (rf2o). Controls rf2o_odometry_node.')

    launch_icp_odometry_la = DeclareLaunchArgument(
            'launch_icp_odometry', default_value=launch_icp_odometry,
            description='Whether to launch RTABMap ICP odometry (expensive). Disabled by default; rf2o is the preferred LiDAR odometry source.')

    launch_amcl_la = DeclareLaunchArgument(
            'launch_amcl', default_value=launch_amcl,
            description='Whether to launch AMCL global localizer.')

    localize_on_startup_la = DeclareLaunchArgument(
            'localize_on_startup', default_value=localize_on_startup,
            description='Attempt to localize in a previously saved VSLAM map on startup. '
                        'Forwarded to isaac_ros_visual_slam_realsense.launch.py. '
                        'Set True only when a valid VSLAM map exists at visual_slam_map_path')

    launch_map_server_la = DeclareLaunchArgument(
            'launch_map_server', default_value=launch_map_server,
            description='Whether to launch the map server. '
                        'Disable when building a new map to prevent a stale stored map from being published.')

    odom_tf_publisher_la = DeclareLaunchArgument(
            'odom_tf_publisher', default_value=odom_tf_publisher,
            description='The node responsible for publishing the odometry tf. '
                        'Options: ekf, vslam|stereo, icp, rgbd, pointcloud, rtabmap.')

    map_tf_publisher_la = DeclareLaunchArgument(
            'map_tf_publisher', default_value=map_tf_publisher,
            description='The node responsible for publishing the map tf. '
                        'Options: pf, amcl, ekf, vslam, rtabmap, slam (for slam_toolbox).')

    base_frame_la = DeclareLaunchArgument(
            'base_frame', default_value=base_frame,
            description='Robot frame, e.g base_link, sensor_kit_link.')

    odom_frame_la = DeclareLaunchArgument(
            'odom_frame', default_value=odom_frame,
            description='Odometry frame, e.g. odom')

    use_gpu_la = DeclareLaunchArgument(
            'use_gpu', default_value=use_gpu,
            description='Use GPU acceleration. Default: True')

    qos_rtabmap_la = DeclareLaunchArgument(
            'qos_rtabmap', default_value=qos_rtabmap,
            description='Specific QoS used for '
                        'most input data in RTABmap: 0=system default, 1=Reliable, 2=Best Effort.')

    qos_rtabmap_camera_la = DeclareLaunchArgument(
            'qos_rtabmap_camera', default_value=qos_rtabmap_camera,
            description='Specific QoS used for '
                        'image input data in RTABmap: 0=system default, 1=Reliable, 2=Best Effort.')

    qos_rtabmap_imu_la = DeclareLaunchArgument(
            'qos_rtabmap_imu', default_value=qos_rtabmap_imu,
            description='Specific QoS used for '
                        'IMU input data in RTABmap: 0=system default, 1=Reliable, 2=Best Effort.')

    qos_rtabmap_laserscan_la = DeclareLaunchArgument(
            'qos_rtabmap_laserscan', default_value=qos_rtabmap_laserscan,
            description='Specific QoS used for '
                        'the laserscan input data in RTABmap: 0=system default, 1=Reliable, 2=Best Effort. '
                        'Use 1: if using a laserscan filter, else use 2.')

    qos_la = DeclareLaunchArgument(
            'qos', default_value=qos,
            description='Specific QoS used for '
                        'image input data: SYSTEM_DEFAULT, DEFAULT, SENSOR_DATA')

    qos_imu_la = DeclareLaunchArgument(
            'qos_imu', default_value=qos_imu,
            description='Specific QoS used for '
                        'IMU input data: SYSTEM_DEFAULT, DEFAULT, SENSOR_DATA')

    camera_name_la = DeclareLaunchArgument(
            'camera_name', default_value=camera_name,
            description='Camera name. Default: camera')

    scan_prefix_la = DeclareLaunchArgument(
            'scan_prefix', default_value=scan_prefix,
            description='String to prefix to laserscan topic. Should match LaserScan namespace. Default: lidar/')

    gravitational_acceleration_la = DeclareLaunchArgument(
            'gravitational_acceleration', default_value=gravitational_acceleration,
            description='Gravitational acceleration (m/s^2). Calibrate per robot. Default: 9.80665')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Get strings from LaunchConfiguration
    namespace_str = namespace.perform(context)
    use_namespace_str = use_namespace.perform(context)
    camera_name_str = camera_name.perform(context)
    scan_prefix_str = scan_prefix.perform(context)
    odom_tf_publisher_str = odom_tf_publisher.perform(context)
    map_tf_publisher_str = map_tf_publisher.perform(context)
    launch_amcl_str = launch_amcl.perform(context)
    launch_map_server_str = launch_map_server.perform(context)
    map_file_str = map_file.perform(context)
    use_composition_str = use_composition.perform(context)
    use_gpu_str = use_gpu.perform(context)
    use_sim_time_str = use_sim_time.perform(context)
    container_name_str = container_name.perform(context)
    attach_to_shared_component_container_str = attach_to_shared_component_container.perform(context)

    # Build the absolute container name for LoadComposableNodes.  A relative name like
    # 'gosling1/f1tenth_container' silently misses '/gosling1/f1tenth_container' — same
    # issue that was fixed for nav2 in bringup.launch.py.
    # Update: now we will let VSLAM have its own container separate from the rest of the localization stack
    # NOTE: vslam_container_name must NOT be 'sensing_container'. That name is already created by
    # realsense_d435i.launch.py, and LoadComposableNodes resolves its target BY NAME — pointing
    # VSLAM at it made three independent creators share one name, so the RealSense driver was
    # loaded more than once, two /camera nodes fought over the same D435i
    # ('failed to claim usb interface ... RS2_USB_STATUS_BUSY') and the camera never started.
    # A dedicated container is also the b47d45d intent ('isolate VSLAM'), and matches the
    # non-composed branch below, which already used 'visual_slam_container'.
    if use_namespace_str.lower() == 'true' and namespace_str:
        absolute_container_name = f'/{namespace_str}/{container_name_str}'
        vslam_container_name = f'/{namespace_str}/visual_slam_container'
    else:
        absolute_container_name = f'/{container_name_str}'
        vslam_container_name = f'/visual_slam_container'

    lifecycle_nodes = []
    if launch_map_server_str.lower() == 'true':
        lifecycle_nodes.append('map_server')
    # if pathlib.Path(map_file_str).exists():
    #     lifecycle_nodes.append('map_server')  # could create an if-block to check if lifecycle_nodes is empty

    if launch_amcl_str.lower() == 'true':
        lifecycle_nodes.append('amcl')

    if camera_name_str != '':
        camera_name_str += '/'

    # Load Nodes
    load_nodes = GroupAction(
            condition=IfCondition(PythonExpression(['not ', use_composition])),
            actions=[
                PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
                # Set common parameters
                SetParameter(name='use_sim_time', value=use_sim_time),
                Node(
                        package='nav2_map_server',
                        executable='map_server',
                        condition=IfCondition(launch_map_server),
                        name='map_server',
                        #namespace=namespace,
                        output='screen',
                        respawn=use_respawn,
                        respawn_delay=2.0,
                        parameters=[
                            params_file,
                            {'yaml_filename': map_file},
                        ],
                        arguments=['--ros-args', '--log-level', log_level],
                        remappings=remappings),
                Node(
                        package='nav2_amcl',
                        executable='amcl',
                        condition=IfCondition(launch_amcl),
                        name='amcl',
                        #namespace=namespace,
                        output='screen',
                        respawn=use_respawn,
                        respawn_delay=2.0,
                        parameters=[
                            params_file,
                            {
                                'tf_broadcast': True if map_tf_publisher_str.lower() == 'amcl' else False,
                                # 'set_initial_pose': PythonExpression(['not ', use_gpu]),# todo: update to compare use_gpu and launch_amcl/global
                            }
                        ],
                        arguments=['--ros-args', '--log-level', log_level],
                        remappings=remappings),
                Node(
                        package='nav2_lifecycle_manager',
                        executable='lifecycle_manager',
                        # condition=IfCondition(launch_amcl),
                        name='lifecycle_manager_localization',
                        #namespace=namespace,
                        output='screen',
                        arguments=['--ros-args', '--log-level', log_level],
                        parameters=[{'use_sim_time': use_sim_time},
                                    {'autostart': autostart},
                                    {'node_names': lifecycle_nodes}])
            ]
    )

    load_composed_amcl_node = LoadComposableNodes(
            # condition=IfCondition(
            #         PythonExpression(
            #                 ["'", use_composition, "' == 'true' and '", launch_amcl, "' == 'true'"]
            #         )
            # ),
            condition=IfCondition(PythonExpression([use_composition, ' and ', launch_amcl])),  # equivalent to the above
            target_container=absolute_container_name,
            composable_node_descriptions=[
                ComposableNode(
                        package='nav2_amcl',
                        plugin='nav2_amcl::AmclNode',
                        name='amcl',
                        namespace=namespace if (use_namespace_str.lower() == 'true' and namespace_str) else '',
                        parameters=[
                            params_file,
                            {
                                'use_sim_time': use_sim_time,
                                'tf_broadcast': True if map_tf_publisher_str.lower() == 'amcl' else False,
                                # 'set_initial_pose': PythonExpression(['not ', use_gpu]),# todo: update to compare use_gpu and launch_amcl/global
                            }
                        ],
                        remappings=remappings),
            ],
    )

    load_composable_map_server = LoadComposableNodes(
            condition=IfCondition(PythonExpression([use_composition, ' and ', launch_map_server])),
            target_container=absolute_container_name,
            composable_node_descriptions=[
                ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        name='map_server',
                        namespace=namespace if (use_namespace_str.lower() == 'true' and namespace_str) else '',
                        parameters=[
                            params_file,
                            {
                                'use_sim_time': use_sim_time,
                                'yaml_filename': map_file},
                        ],
                        remappings=remappings),
            ],
    )

    load_composable_lifecycle_manager = LoadComposableNodes(
            condition=IfCondition(use_composition),
            target_container=absolute_container_name,
            composable_node_descriptions=[
                ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_localization',
                        namespace=namespace if (use_namespace_str.lower() == 'true' and namespace_str) else '',
                        parameters=[{'use_sim_time': use_sim_time,
                                     'autostart': autostart,
                                     'node_names': lifecycle_nodes}]),
            ],
    )

    slam_toolbox_localizer_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [f1tenth_launch_pkg_prefix, 'launch/localization', 'slam_localization.launch.py']
            )),
            condition=IfCondition([launch_slam_toolbox_localizer]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': os.path.join(
                        f1tenth_launch_pkg_prefix, 'config', 'localization/localizer_slam.yaml'),
                'use_namespace': use_namespace,
                'namespace': namespace if (use_namespace_str.lower() == 'true' and namespace_str) else '',
                'base_frame': base_frame,
                'odom_frame': odom_frame,
                'map_frame': 'map',
                'map_file_name': map_file.perform(context).rstrip('.yaml'),
                'map_tf_publish_period': '0.05' if map_tf_publisher_str.lower() == 'slam' else '0.0',
            }.items()
    )

    ekf_nodes = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [f1tenth_launch_pkg_prefix, 'launch/localization', 'dual_ekf.launch.py']
            )),
            condition=IfCondition(launch_sensor_fusion),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_namespace': use_namespace,
                'namespace': namespace,
                'use_ekf_odom': launch_ekf_odom,
                'use_ekf_map': launch_ekf_map,
                'odom_frequency': odom_frequency,
                'publish_odom_tf': 'True' if odom_tf_publisher_str.lower() == 'ekf' else 'False',
                'publish_map_tf': 'True' if map_tf_publisher_str.lower() == 'ekf' else 'False',
                'map_frequency': map_frequency,
                'gravitational_acceleration': gravitational_acceleration,
            }.items()
    )

    # global localization using a pre-existing map
    rtabmap_localizer_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [f1tenth_launch_pkg_prefix, 'launch/mapping', '3d_mapping.launch.py']
            )),
            condition=IfCondition(launch_rtabmap_localizer),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace': namespace if (use_namespace_str.lower() == 'true' and namespace_str) else '',
                'qos': qos_rtabmap,
                'qos_image': qos_rtabmap_camera,
                'qos_camera_info': qos_rtabmap_camera,
                'qos_imu': qos_rtabmap_imu,
                'qos_scan': qos_rtabmap_laserscan,
                'qos_odom': qos_rtabmap,
                'use_stereo': 'False',
                'localization': 'True',
                'queue_size': '5',  # 10
                'approx_sync': 'True',
                'publish_map_tf': 'True' if map_tf_publisher_str.lower() == 'rtabmap' else 'False',
                "publish_odom_tf": 'True' if odom_tf_publisher_str.lower() == 'rtabmap' else 'False',
                "visual_odometry": 'True' if odom_tf_publisher_str.lower() == 'rtabmap' else 'False',
                "icp_odometry": 'True' if odom_tf_publisher_str.lower() == 'rtabmap' else 'False',
                'wait_imu_to_init': 'True',
                'imu_topic': camera_name_str + 'imu/filtered',  # '/camera/imu/filtered', '/vehicle/sensors/imu/data'
                "left_image_topic": camera_name_str + 'infra1/image_rect_raw',
                "right_image_topic": camera_name_str + 'infra2/image_rect_raw',
                "left_camera_info_topic": camera_name_str + 'infra1/camera_info',
                "right_camera_info_topic": camera_name_str + 'infra2/camera_info',
                "rgb_topic": camera_name_str + 'color/image_raw',  # camera_name_string + 'color/image_raw', left_image_topic
                "depth_topic": camera_name_str + 'aligned_depth_to_color/image_raw',
                "odom_topic": 'odometry/local',
                "camera_info_topic": camera_name_str + 'color/camera_info',  # 'color/camera_info', 'infra1/camera_info'
                "scan_topic": scan_prefix_str + 'scan_filtered',
                'rtabmap_viz_view': 'False',
                'rviz_view': 'False',
                'database_path': rtabmap_database_file,
                'rtabmap_args': '--Mem/IncrementalMemory false '      # localization: do not grow the map
                                '--Mem/InitWMWithAllNodes true '       # load all map nodes into working memory at start
                                '--RGBD/LoopClosureReextractFeatures false '  # not needed in localization mode
                                '--RGBD/SavedLocalizationIgnored true '
                                # Feature detector — must match the detector used during mapping (SIFT=1).
                                # Using a different type here causes the BoW vocabulary lookup to fail silently.
                                '--Kp/DetectorStrategy 1 '             # was 8 (BRISK): must match mapping (SIFT=1)
                                '--Vis/FeatureType 1 '                  # was 8 (BRISK): must match mapping (SIFT=1)
                                '--Kp/MaxFeatures 500 '
                                '--Kp/MaxDepth 5 '
                                '--Vis/MinInliers 15 '
                                '--Vis/EstimationType 0 '
                                '--Vis/MaxDepth 0 '                    # 0=unlimited: use full depth range for map matching
                                '--Vis/BundleAdjustment 1 '
                                '--Vis/PnPFlags 0 '
                                '--Vis/CorType 0 '
                                '--GFTT/QualityLevel 0.001 '           # was 0.00001: noise keypoints
                                '--RGBD/LinearUpdate 0.1 '
                                '--RGBD/AngularUpdate 0.1 '
                                '--Stereo/MinDisparity 0.5 '
                                '--Stereo/MaxDisparity 128.0 '
                                '--Stereo/OpticalFlow true '
                                '--Stereo/DenseStrategy 1 '
                                '--Reg/Force3DoF true '
                                '--RGBD/NeighborLinkRefining true '
                                '--RGBD/ProximityBySpace true '
                                '--Reg/Strategy 2 '                    # was 1 (ICP-only): VisIcp — visual estimates initial transform, ICP refines; ICP-only fails when odometry drift > MaxCorrespondenceDistance
                                '--Icp/VoxelSize 0.05 '
                                '--Icp/MaxCorrespondenceDistance 0.15 '
                                '--Icp/MaxRotation 0.5 '               # was 1.6 (~91deg): too wide, accepted bad alignments
                                '--Icp/MaxTranslation 1.0 '
                                '--Icp/Strategy 1 '
                                '--Icp/OutlierRatio 0.75 '
                                '--Icp/CorrespondenceRatio 0.2 '
                                '--Icp/PointToPlaneMinComplexity 0.04 '
                                '--Icp/Iterations 50 '
                                '--Icp/Force4DoF true '
                                '--Rtabmap/DetectionRate 0 '
                                '--Rtabmap/ImageBufferSize 5 '
                                '--Grid/Sensor 0 '
                                '--Grid/RangeMax 10.0 '
                                '--Grid/RangeMin 0.2 '
                                '--Grid/NoiseFilteringMinNeighbors 8 '
                                '--Grid/NoiseFilteringRadius 0.1 '
                                '--Optimizer/Strategy 2 '
                                '--Optimizer/Slam2D true '
                                '--Optimizer/GravitySigma 0.3 '        # was 0 (disabled): gravity constraint from IMU reduces drift
                                '--RGBD/OptimizeMaxError 4 '
                                '--RGBD/LocalRadius 8 '                # was 5: wider search improves re-localization after drift
                                '--RGBD/ProximityPathMaxNeighbors 10 '
                                '--Mem/STMSize 50 ',
            }.items()
    )

    # common rtabmap parameters (to avoid having to create multiple LaunchConfigurations to cast as strings). Source: https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h
    parameters = {
        'Odom/Strategy': '0',  # 0=Frame-to-map (accurate), 1=Frame-to-Frame (faster)
        'Odom/Holonomic': 'False',
        'Odom/FilteringStrategy': '0',  # odom output filtering. 0=None, 1=KF, 2=PF
        'OdomF2M/BundleAdjustment': '1',  # 0=disabled, 1=g2o
        'Odom/GuessMotion': 'True',
        'Odom/KeyFrameThr': '0.3',  # default = 0.3. 0.6
        'Odom/ScanKeyFrameThr': '0.3',
        'Odom/ResetCountdown': '1',  # reset X frames after losing odometry
        'Rtabmap/StartNewMapOnLoopClosure': 'True',  # resets local odometry window on loop detection, preventing error accumulation in the F2M map
        'Optimizer/GravitySigma': '0.3',  # was 0 (disabled): gravity constraint from IMU reduces roll/pitch drift in visual odometry
    }

    # RGB-D odometry
    rtabmap_rgbd_odometry = Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            condition=IfCondition(launch_rgbd_odometry),
            name='rtabmap_rgbd_odom',
            # namespace=namespace_str,
            parameters=[
                parameters,
                {
                    'approx_sync': True,
                    'publish_tf': True if odom_tf_publisher_str.lower() == 'rgbd' else False,
                }
            ],
            output='log',
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[
                ('rgb/image', camera_name_str + 'color/image_raw'),
                ('rgb/camera_info', camera_name_str + 'color/camera_info'),
                ('depth/image', camera_name_str + 'aligned_depth_to_color/image_raw'),
                # ('imu', 'vehicle/sensors/imu/data'),  # imu must have orientation
                ('odom', 'odom/rtabmap/rgbd'),
                ('odom_last_frame', 'rtabmap/rgbd/points'),  # 'odom_last_frame ', 'odom_filtered_input_scan'
                *remappings
            ]
    )

    # Stereo odometry
    rtabmap_stereo_odometry = Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            condition=IfCondition(launch_stereo_odometry),
            name='rtabmap_stereo_odom',
            # namespace=namespace_str,
            parameters=[
                parameters,
                {
                    'approx_sync': False,
                    'publish_tf': True if odom_tf_publisher_str.lower() in ['vslam', 'stereo'] else False,
                }
            ],
            output='log',
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[
                ('left/image_rect', camera_name_str + 'infra1/image_rect_raw'),
                ('left/camera_info', camera_name_str + 'infra1/camera_info'),
                ('right/image_rect', camera_name_str + 'infra2/image_rect_raw'),
                ('right/camera_info', camera_name_str + 'infra2/camera_info'),
                # ('imu', 'vehicle/sensors/imu/data'),  # imu must have orientation
                ('odom', 'odom/rtabmap/stereo'),
                ('odom_last_frame', 'rtabmap/stereo/points'),  # 'odom_last_frame ', 'odom_filtered_input_scan'
                *remappings
            ]
    )

    # PointCloud Odometry (kiss-icp). Do not use with Realsense (RGB-D) PointCloud. Use Open3D instead.
    kiss_icp_node = Node(
            package="kiss_icp",
            executable="kiss_icp_node",
            condition=IfCondition(launch_pointcloud_odometry),
            name="kiss_icp_node",
            namespace=namespace if (use_namespace_str.lower() == 'true' and namespace_str) else '',
            output="screen",
            remappings=[
                ("pointcloud_topic", camera_name_str + "depth/color/points"),  # /camera/downsampled_cloud_from_depth. todo: also make the pointcloud topic dynamic
                *remappings
            ],
            parameters=[  # todo: see Open3D's realsense settings for ideas
                {
                    # ROS node configuration
                    "base_frame": "base_link",
                    "odom_frame": odom_frame,
                    "publish_odom_tf": True if odom_tf_publisher_str.lower() == 'pointcloud' else False,
                    # KISS-ICP configuration
                    "max_range": 10.0,  # todo: tune using realsense viewer
                    "min_range": 0.105,  # 0.105 or 0.28, todo: tune
                    "deskew": False,
                    "max_points_per_voxel": 20,
                    "voxel_size": 0.05,  # (optional)
                    # Adaptive threshold
                    # "fixed_threshold": 0.3,  # (optional) this disables adaptive thresholding
                    "initial_threshold": 2.0,
                    "min_motion_th": 0.1,
                    # Registration
                    "max_num_iterations": 500,  # (optional).
                    "convergence_criterion": 0.0001,  # (optional).
                    "max_num_threads": 0,  # (optional). todo: tune
                    # Fixed covariances
                    "position_covariance": 0.1,
                    "orientation_covariance": 0.1,
                    # ROS CLI arguments
                    "publish_debug_clouds": False,  # todo: use this to debug accuracy
                    "use_sim_time": use_sim_time,
                },
            ],
    )

    # LaserScan odometry
    icp_parameters = {
        'Odom/Strategy': '0',  # 0=Frame-to-map (accurate), 1=Frame-to-Frame (faster)
        'Odom/Holonomic': 'True',
        # should be False, but the ICP node occasionally fails and yields worse odometry when set to False. Either comment out or set to True
        'Odom/FilteringStrategy': '0',
        # odom output filtering. 0=None, 1=KF, 2=PF. When testing, setting to any value other than 0 leads to no computation.
        'OdomF2M/BundleAdjustment': '1',  # 0=disabled, 1=g2o
        'Odom/GuessMotion': 'True',
        'Odom/KeyFrameThr': '0.15',  # default = 0.3. 0.6
        'Odom/ScanKeyFrameThr': '0.7',  # default=0.9. Try 0.75, 0.3
        'Odom/ResetCountdown': '1',  # reset X frames after losing odometry
        'OdomF2M/ScanSubtractRadius': '0.05',
        # Could also set to the same as voxel size. reduce from 0.05 for performance boost at the cost of accuracy
        'OdomF2M/ScanMaxSize': '2000',  # reduce from 2000 for performance boost at the cost of accuracy
        'Rtabmap/StartNewMapOnLoopClosure': 'True',
        'Icp/Strategy': '1',  # 0=PointCloud Library (PCL), 1=libpointmatcher [default]
        'Icp/MaxCorrespondenceDistance': '0.1',
        'Icp/MaxTranslation': '0.4',
        # Maximum ICP translation correction accepted (meters). Note. This should be increased for a fast moving robot or low frequency LIDAR or both. Should be scaled based on max vehicle speed expected vs highest lidar frequency. For example, for a LIDAR publishing scans at 8.5 Hz, with a maximum expected robot speed of 10 m/s, max_translation = max_speed (10) * sample_time (1 / 8.5) = 10 * 0.1176470588 = 1.176470588. Default = 0.2
        'Icp/MaxRotation': '0.5',   # was 1.6 (~91deg): too wide; matches mapping.launch.py fix
        # Maximum ICP rotation correction accepted (in radians). See MaxTranslation explanation above. max_rotation = max_yaw_rate * sample_time. Default = 0.78 (45 degrees)
        'Optimizer/GravitySigma': '0.3',  # was 0 (disabled): gravity constraint from IMU reduces roll/pitch drift
        'Icp/VoxelSize': '0.05',  # increase to 0.2 for performance boost at the cost of accuracy
        'Icp/Epsilon': '0.0001',  # increase for performance boost at the cost of accuracy. Default=0.0
        'Icp/Iterations': '10',  # set to 10 for performance boost at the cost of accuracy. Default=30
        'Icp/RangeMax': '10.0',
        'Icp/Force4DoF': 'True',
        'Reg/Force3DoF': 'True',
    }
    rtabmap_icp_odometry = Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            condition=IfCondition(launch_icp_odometry),  # separate flag; disabled by default, rf2o is the active LiDAR odom source
            name='rtabmap_icp_odom',
            namespace=namespace_str,
            respawn=use_respawn,
            respawn_delay=2.0,
            output='log',  # suppress per-iteration console spam; errors still go to ~/.ros/log
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                icp_parameters,
                {
                    'qos': qos_rtabmap_laserscan,
                    'qos_imu': qos_rtabmap_imu,
                    'expected_update_rate': 10.0,
                    'wait_for_transform': 0.3,
                    'wait_imu_to_init': False,
                    'use_sim_time': use_sim_time,
                    'queue_size': 10,
                    'approx_sync': True,
                    'frame_id': base_frame,
                    'odom_frame_id': odom_frame,
                    'scan_range_min': 0.12,
                    'scan_range_max': 10.0,
                    'deskewing': False,
                    'guess_frame_id': odom_frame if odom_tf_publisher_str.lower() != 'icp' else '',  # comment out if this node is going to publish the tf, i.e if publish_odom_tf is True
                    # 'guess_min_translation': 0.05, # m
                    # 'guess_min_rotation': 0.005, # rad
                    'publish_tf': True if odom_tf_publisher_str.lower() == 'icp' else False,
                    'publish_null_when_lost': False,
                    # 'rtabmap_config_path': rtabmap_database_file_path,
                }
            ],
            remappings=[
                ('scan', scan_prefix_str + 'scan_filtered'),
                ('imu', 'vehicle/sensors/imu/raw'),  # imu must have orientation. /camera/camera/imu/filtered
                ('odom', 'odom/rtabmap/icp'),
                ('odom_last_frame', 'rtabmap/icp/points'),  # 'odom_last_frame ', 'odom_filtered_input_scan'
                *remappings
            ]
    )

    rf2o_parameters = [{
        'laser_scan_topic': scan_prefix_str + 'scan_filtered',
        'odom_topic': 'odom/rf2o',
        'publish_tf': True if odom_tf_publisher_str.lower() == 'rf2o' else False,
        'base_frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'init_pose_from_topic': '',
        'use_sim_time': use_sim_time,
        'freq': 10.0,
        'publish_covariance': True,
        'use_rf2o_twist_covariance': True,
        'pose_covariance_diagonal': [0.0025, 0.0025, 1000000.0, 1000000.0, 1000000.0, 0.0025],
        'twist_covariance_diagonal': [0.01, 0.01, 1000000.0, 1000000.0, 1000000.0, 0.01],
        'covariance_scale': 1.0,
    }]

    rf2o_odometry_node = Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            condition=IfCondition(PythonExpression(['not ', use_composition, ' and ', launch_laserscan_odometry])),
            name='rf2o_laser_odometry',
            namespace=namespace if (use_namespace_str.lower() == 'true' and namespace_str) else '',
            output='log',  # suppress per-scan console spam; errors still go to ~/.ros/log
            arguments=['--ros-args', '--log-level', 'rf2o_laser_odometry:=WARN'],
            parameters=rf2o_parameters,
            remappings=remappings,
    )

    load_rf2o_composable_node = LoadComposableNodes(
            condition=IfCondition(PythonExpression([use_composition, ' and ', launch_laserscan_odometry])),
            target_container=absolute_container_name,
            composable_node_descriptions=[
                ComposableNode(
                        package='rf2o_laser_odometry',
                        plugin='rf2o::CLaserOdometry2DNode',
                        name='rf2o_laser_odometry',
                        namespace=namespace if (use_namespace_str.lower() == 'true' and namespace_str) else '',
                        parameters=rf2o_parameters,
                        remappings=remappings,
                ),
            ],
    )

    # laser_scan_matcher_node = Node(
    #         package='ros2_laser_scan_matcher',
    #         executable='laser_scan_matcher',
    #         condition=IfCondition(launch_laserscan_odometry),
    #         name='laser_scan_matcher_node',
    #         namespace=namespace if (use_namespace_str.lower() == 'true' and namespace_str) else '',
    #         output='screen',
    #         parameters=[{
    #             'publish_odom': 'odom/laser_scan_matcher',
    #             'publish_tf': True if odom_tf_publisher_str.lower() == 'icp' else False,
    #             'laser_frame': 'lidar',
    #             'base_frame': 'base_link',
    #             'odom_frame': 'odom_laser_scan_matcher',
    #             'map_frame': 'map',
    #             'init_pose_from_topic': '',
    #             'use_sim_time': use_sim_time,
    #             'freq': 20.0}],
    #         remappings=[('scan', 'scan'),
    #                     ('odom', 'odom/laser_scan_matcher'),
    #                     *remappings]
    # )

    # RTabMap Group
    rtabmap_group = GroupAction(
            actions=[
                PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
                # Set common parameters
                SetParameter(name='use_sim_time', value=use_sim_time),
                SetParameter(name='queue_size', value='10'),
                # SetParameter(name='approx_sync', value='False'),  # True: for stereo, False: otherwise
                SetParameter(name='qos', value=qos_rtabmap_camera),
                SetParameter(name='qos_imu', value=qos_rtabmap_imu),
                SetParameter(name='frame_id', value=base_frame),
                SetParameter(name='odom_frame_id', value=odom_frame),
                # SetParameter(name='guess_frame_id', value=odom_frame),
                SetParameter(name='guess_min_translation', value='0.05'),  # m
                SetParameter(name='guess_min_rotation', value='0.005'),  # rad
                # SetParameter(name='publish_tf', value=publish_odom_tf),
                SetParameter(name='wait_for_transform', value='0.3'),
                SetParameter(name='wait_imu_to_init', value='False'),
                SetParameter(name='publish_null_when_lost', value='False'),
                # SetParameter(name='rtabmap_config_path', value=rtabmap_database_file_path),

                # Set remapping rules
                SetRemap(src='scan', dst=scan_prefix_str + 'scan_filtered'),
                # imu must have orientation. vehicle/sensors/imu/data
                SetRemap(src='imu', dst=camera_name_str + 'imu/filtered'),
                SetRemap(src=['/tf'], dst=['tf']),
                SetRemap(src=['/tf_static'], dst=['tf_static']),

                # add nodes
                rtabmap_rgbd_odometry,

                GroupAction(
                        condition=UnlessCondition(use_gpu),
                        actions=[
                            rtabmap_stereo_odometry
                        ]
                ),

                # rtabmap_icp_odometry,  # set 'wait_imu_to_init' to False
            ]
    )

    # GPU group
    visual_slam_launch_include = TimerAction(
            period=1.5,
            actions=[
                GroupAction(
                        actions=[
                            SetRemap(src=['/tf'], dst=['tf']),
                            SetRemap(src=['/tf_static'], dst=['tf_static']),
                            IncludeLaunchDescription(
                                    PythonLaunchDescriptionSource(
                                            PathJoinSubstitution(
                                                    [nvidia_isaac_launch_dir,
                                                     'isaac_ros_visual_slam_realsense.launch.py'])
                                    ),
                                    launch_arguments={
                                        'use_sim_time': use_sim_time,
                                        'namespace': namespace_str,
                                        'use_namespace': 'True' if (use_namespace_str.lower() == 'true' and namespace_str) else 'False',
                                        'camera_name': camera_name,
                                        'two_d_mode': 'True',
                                        'base_frame': base_frame,
                                        'publish_map_to_odom_tf': 'True' if map_tf_publisher_str.lower() == 'vslam' else 'False',
                                        'publish_odom_to_baselink_tf': 'True' if odom_tf_publisher_str.lower() in [
                                            'vslam', 'stereo'] else 'False',
                                        'save_map': 'False',  # todo: set to True (could save on an interval when we are in mapping mode, else False. Could be an argument)
                                        'load_map': 'False',  # todo: set to True (remap the rviz initial_pose to this). I should probably create an argument for init_pose guess
                                        'map_path': visual_slam_map_path,
                                        'launch_realsense_driver': 'False',
                                        'left_image_topic': camera_name_str + 'infra1/image_rect_raw',
                                        # '/camera/realsense_splitter_node/output/infra_1',
                                        'right_image_topic': camera_name_str + 'infra2/image_rect_raw',
                                        # '/camera/realsense_splitter_node/output/infra_2',
                                        'imu_topic': camera_name_str + 'imu/filtered',  # '/camera/camera/imu/filtered'
                                        'image_qos': qos,  # DEFAULT.
                                        'imu_qos': qos_imu,
                                        'enable_visualization_topics': 'True',
                                        'localize_on_startup': localize_on_startup,
                                        'attach_to_shared_component_container': 'False',  # use_composition, we now want vslam to be isolated from the rest
                                        'component_container_name': vslam_container_name if use_composition_str.lower() == 'true' else 'visual_slam_container',
                                    }.items()
                            )
                        ]
                )
            ]
    )

    laserscan_to_flatscan_node = ComposableNode(
            package='isaac_ros_pointcloud_utils',
            plugin='nvidia::isaac_ros::pointcloud_utils::LaserScantoFlatScanNode',
            name='laserscan_to_flatscan',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                }
            ],
            remappings=[('scan', scan_prefix_str + 'scan_filtered'),
                        ('flatscan', 'flatscan_localization'),
                        *remappings]
    )

    occupancy_grid_localizer_node = LoadComposableNodes(
            # condition=IfCondition(
            #         PythonExpression(
            #                 ["'", use_composition, "' == 'true' and '", launch_amcl, "' == 'true'"]
            #         )
            # ),
            condition=IfCondition(PythonExpression([use_composition, ' and ', launch_amcl])),  # equivalent to the above
            target_container=absolute_container_name,
            composable_node_descriptions=[
                ComposableNode(
                        package='isaac_ros_occupancy_grid_localizer',
                        plugin='nvidia::isaac_ros::occupancy_grid_localizer::OccupancyGridLocalizerNode',
                        name='occupancy_grid_localizer',
                        parameters=[
                            {
                                'use_sim_time': use_sim_time,
                                'loc_result_frame': 'map',
                                'map_yaml_path': map_file,
                            }
                        ],
                        remappings=[
                            ('localization_result', 'initialpose'),
                            *remappings
                        ]
                ),
            ],
    )

    localization_container_node = Node(
            name=container_name,
            package='rclcpp_components',
            executable='component_container_mt',
            namespace=namespace if (use_namespace_str.lower() == 'true' and namespace_str) else '',
            output='screen',
            condition=IfCondition(PythonExpression([use_composition, ' and not ', attach_to_shared_component_container])),
    )

    load_gpu_laser_composable_node = LoadComposableNodes(
            target_container=absolute_container_name,
            composable_node_descriptions=[
                laserscan_to_flatscan_node
            ]
    )

    occupancy_grid_localizer_group = GroupAction(
            actions=[
                # PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
                localization_container_node,
                load_gpu_laser_composable_node,
                occupancy_grid_localizer_node,
                # SetRemap(src=['/tf'], dst=['tf']),
                # SetRemap(src=['/tf_static'], dst=['tf_static']),
            ]
    )

    gpu_group = GroupAction(
            condition=IfCondition(use_gpu),
            actions=[
                PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
                # Set common parameters
                SetParameter(name='use_sim_time', value=use_sim_time),
                SetRemap(src=['/tf'], dst=['tf']),
                SetRemap(src=['/tf_static'], dst=['tf_static']),

                # add nodes
                # Guard the timer so it is never scheduled when stereo odometry is disabled.
                # The IfCondition inside the IncludeLaunchDescription fires inside TimerAction
                # callbacks where the launch context may not carry the runtime value of
                # launch_stereo_odometry (falls back to default 'True').  Wrapping with a
                # GroupAction whose condition is evaluated synchronously before the timer is
                # even queued avoids that race.
                GroupAction(
                        condition=IfCondition(launch_stereo_odometry),
                        actions=[visual_slam_launch_include]
                ),
                # occupancy_grid_localizer_group,  # todo: enable later after testing other components
            ]
    )

    try:
        particle_filter_package_share_dir = get_package_share_directory('particle_filter')
        particle_filter_node = Node(
                condition=IfCondition(launch_particle_filter),
                package='particle_filter',
                executable='particle_filter',
                name='particle_filter',
                output='log',  # suppress per-iteration MCL console spam; errors still go to ~/.ros/log
                namespace=namespace_str,
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    particle_filter_config,
                    {
                        # --- Runtime-computed overrides (cannot live in YAML) ---
                        'use_sim_time': use_sim_time,
                        'scan_qos_reliability': 'best_effort',
                        'odom_qos_reliability': 'best_effort',
                        # Ray backend: rmgpu requires range_libc CUDA build; fall back to pcddt on CPU
                        'range_method': 'rmgpu' if use_gpu_str.lower() == 'true' else 'pcddt',
                        'rangelib_variant': 2 if use_gpu_str.lower() == 'true' else 4,
                        # Frame IDs come from launch args so they stay here
                        'global_frame_id': 'map',
                        'base_frame_id': base_frame,
                        'odom_frame_id': odom_frame,
                        # TF ownership: only broadcast map→odom when this node is the designated publisher
                        'tf_broadcast': True if map_tf_publisher_str.lower() == 'pf' else False,
                    }
                ],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[
                    ('scan', scan_prefix_str + 'scan_filtered'),
                    ('odom', 'odometry/local'),
                    *remappings
                ]
        )
    except ROS2PackageNotFoundError as e:
        particle_filter_node = LogInfo(msg=f'Failed to launch particle filter node: {e}. Skipping...')

    # Create Launch Description and add nodes to the launch description
    ld = [
        stdout_linebuf_envvar,
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        launch_pointcloud_odometry_la,
        launch_rgbd_odometry_la,
        launch_stereo_odometry_la,
        visual_slam_map_path_la,
        launch_laserscan_odometry_la,
        launch_icp_odometry_la,
        launch_amcl_la,
        launch_map_server_la,
        odom_tf_publisher_la,
        map_tf_publisher_la,
        localization_param,
        map_file_la,
        launch_slam_toolbox_localizer_la,
        launch_sensor_fusion_la,
        launch_ekf_odom_la,
        launch_ekf_map_la,
        map_frequency_la,
        odom_frequency_la,
        launch_rtabmap_localizer_la,
        declare_use_composition_cmd,
        declare_container_name_cmd,
        declare_attach_to_shared_component_container_cmd,
        declare_autostart_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        localization_container_node,
        load_nodes,
        load_composed_amcl_node,
        load_composable_map_server,
        # Skip when empty: ROS 2 Humble converts [] to () and fails the scalar-type check in evaluate_parameter_dict.
        *([load_composable_lifecycle_manager] if lifecycle_nodes else []),
        slam_toolbox_localizer_node,
        ekf_nodes,
        rtabmap_localizer_node,
        rtabmap_database_file_la,
        base_frame_la,
        odom_frame_la,
        use_gpu_la,
        rtabmap_group,
        rtabmap_icp_odometry,  # separate from rtabmap group; controlled by launch_icp_odometry (default False)
        rf2o_odometry_node,          # cheap range-flow LiDAR odometry; feeds EKF odom2 via odom/rf2o
        load_rf2o_composable_node,   # composed path: loads rf2o into existing container when use_composition=True
        kiss_icp_node,
        gpu_group,
        qos_rtabmap_la, qos_rtabmap_camera_la, qos_rtabmap_imu_la, qos_rtabmap_laserscan_la, qos_la, qos_imu_la,
        camera_name_la,
        scan_prefix_la,
        gravitational_acceleration_la,
        localize_on_startup_la,
        launch_particle_filter_la,
        particle_filter_config_la,
        particle_filter_node,
        # laser_scan_matcher_node
    ]

    return ld


def generate_launch_description():
    return LaunchDescription(
            [
                SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
                OpaqueFunction(function=launch_setup)
            ]
    )

