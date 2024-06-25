"""
todo:
    * add default booleans to LaunchConfigurations instead of LaunchArguments to streamline
    * run tests and remove todos below
    * add isaac ros visual slam
todo: if odom nodes fail, test using the sensors base_frame instead
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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes, SetRemap, PushRosNamespace, SetParametersFromFile, SetParameter
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString
from ament_index_python import get_package_share_directory


def generate_launch_description():
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

    # declare launch configurations
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace', default=False)
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    use_composition = LaunchConfiguration('use_composition', default=False)
    container_name = LaunchConfiguration('container_name', default='nav2_container')
    container_name_full = (namespace, '/', container_name)
    autostart = LaunchConfiguration('autostart', default=True)
    use_respawn = LaunchConfiguration('use_respawn', default=False)
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map_file')
    launch_slam_toolbox_localizer = LaunchConfiguration('launch_slam_toolbox_localizer', default=False)
    launch_sensor_fusion = LaunchConfiguration('launch_sensor_fusion', default=True)
    launch_ekf_odom = LaunchConfiguration('launch_ekf_odom', default=True)
    launch_ekf_map = LaunchConfiguration('launch_ekf_map', default=True)
    odom_frequency = LaunchConfiguration('odom_frequency')
    map_frequency = LaunchConfiguration('map_frequency')
    launch_rtabmap_localizer = LaunchConfiguration('launch_rtabmap_localizer', default=False)
    rtabmap_database_file = LaunchConfiguration('rtabmap_database_file', default=rtabmap_database_file_path)
    log_level = LaunchConfiguration('log_level')

    launch_pointcloud_odometry = LaunchConfiguration('launch_pointcloud_odometry', default='False')
    launch_rgbd_odometry = LaunchConfiguration('launch_rgbd_odometry', default='False')
    launch_stereo_odometry = LaunchConfiguration('launch_stereo_odometry', default='True')
    launch_laserscan_odometry = LaunchConfiguration('launch_laserscan_odometry', default='True')
    launch_amcl = LaunchConfiguration('launch_amcl', default='True')

    base_frame = LaunchConfiguration('base_frame', default='base_link')
    odom_frame = LaunchConfiguration('odom_frame', default='odom')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf', default='False')

    use_gpu = LaunchConfiguration('use_gpu', default='True')

    # Declare default launch arguments
    stdout_linebuf_envvar = SetEnvironmentVariable(
            'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    declare_namespace_cmd = DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true')
    localization_param = DeclareLaunchArgument(
            'params_file',
            default_value=localization_param_file,
            description='Path to config file for localization nodes'
    )
    map_file_la = DeclareLaunchArgument(
            'map_file',
            default_value=map_file_path,
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
    odom_frequency_la = DeclareLaunchArgument(
            'odom_frequency',
            default_value='30.0',  # 100.0
            description='Local/odom EKF/UKF node update/publish frequency.'
    )
    map_frequency_la = DeclareLaunchArgument(
            'map_frequency',
            default_value='10.0',  # 30.0
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

    launch_laserscan_odometry_la = DeclareLaunchArgument(
            'launch_laserscan_odometry', default_value=launch_laserscan_odometry,
            description='Whether to launch laserscan odometry.')

    launch_amcl_la = DeclareLaunchArgument(
            'launch_amcl', default_value=launch_amcl,
            description='Whether to launch AMCL global localizer.')

    base_frame_la = DeclareLaunchArgument(
            'base_frame', default_value=base_frame,
            description='Robot frame, e.g base_link, sensor_kit_link.')

    odom_frame_la = DeclareLaunchArgument(
            'odom_frame', default_value=odom_frame,
            description='Odometry frame, e.g. odom')

    publish_odom_tf_la = DeclareLaunchArgument(
            'publish_odom_tf', default_value=publish_odom_tf,
            description='Default: False')

    use_gpu_la = DeclareLaunchArgument(
            'use_gpu', default_value=use_gpu,
            description='Use GPU acceleration. Default: True')

    lifecycle_nodes = ['map_server', 'amcl']
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_file,
        'set_initial_pose': PythonExpression(['not ', use_gpu]),  # todo: test
    }

    # Only it applys when `use_namespace` is True.
    # '<robot_namespace>' keyword shall be replaced by 'namespace' launch argument
    # in config file 'nav2_multirobot_params.yaml' as a default & example.
    # User defined config file should contain '<robot_namespace>' keyword for the replacements.
    params_file = ReplaceString(
            source_file=params_file,
            replacements={'<robot_namespace>': ('/', namespace)},
            condition=IfCondition(use_namespace))

    configured_params = ParameterFile(
            RewrittenYaml(
                    source_file=params_file,
                    root_key=namespace,
                    param_rewrites=param_substitutions,
                    convert_types=True),
            allow_substs=True)

    # Load Nodes
    load_nodes = GroupAction(
            condition=IfCondition(PythonExpression(['not ', use_composition])),
            actions=[
                Node(
                        package='nav2_map_server',
                        executable='map_server',
                        condition=IfCondition(launch_amcl),
                        name='map_server',
                        output='screen',
                        respawn=use_respawn,
                        respawn_delay=2.0,
                        parameters=[configured_params],
                        arguments=['--ros-args', '--log-level', log_level],
                        remappings=remappings),
                Node(
                        package='nav2_amcl',
                        executable='amcl',
                        condition=IfCondition(launch_amcl),
                        name='amcl',
                        output={'both': 'log'},
                        respawn=use_respawn,
                        respawn_delay=2.0,
                        parameters=[configured_params],
                        arguments=['--ros-args', '--log-level', log_level],
                        remappings=remappings),
                Node(
                        package='nav2_lifecycle_manager',
                        executable='lifecycle_manager',
                        condition=IfCondition(launch_amcl),
                        name='lifecycle_manager_localization',
                        output='screen',
                        arguments=['--ros-args', '--log-level', log_level],
                        parameters=[{'use_sim_time': use_sim_time},
                                    {'autostart': autostart},
                                    {'node_names': lifecycle_nodes}])
            ]
    )

    load_composable_nodes = LoadComposableNodes(
            condition=IfCondition(use_composition),
            target_container=container_name_full,
            composable_node_descriptions=[
                ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        condition=IfCondition(launch_amcl),
                        name='map_server',
                        parameters=[configured_params],
                        remappings=remappings),
                ComposableNode(
                        package='nav2_amcl',
                        plugin='nav2_amcl::AmclNode',
                        condition=IfCondition(launch_amcl),
                        name='amcl',
                        parameters=[configured_params],
                        remappings=remappings),
                ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        condition=IfCondition(launch_amcl),
                        name='lifecycle_manager_localization',
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
            }.items()
    )

    ekf_nodes = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [f1tenth_launch_pkg_prefix, 'launch/localization', 'dual_ekf.launch.py']
            )),
            condition=IfCondition(launch_sensor_fusion),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_ekf_odom': launch_ekf_odom,
                'use_ekf_map': launch_ekf_map,
                'odom_frequency': odom_frequency,
                'map_frequency': map_frequency,
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
                'use_stereo': 'False',
                'localization': 'True',
                'queue_size': '10',
                'approx_sync': 'True',
                'publish_map_tf': 'False',
                'wait_imu_to_init': 'True',
                'imu_topic': '/camera/camera/imu/filtered',  # '/camera/imu/filtered', '/vehicle/sensors/imu/data'
                "depth_topic": '/camera/camera/aligned_depth_to_color/image_raw',
                'rtabmap_viz_view': 'False',
                'rviz_view': 'False',
                'database_path': rtabmap_database_file,
                'rtabmap_args': '--Mem/IncrementalMemory false '  # false=localization, true=mapping
                                '--Mem/InitWMWithAllNodes true '  # true=localization, false=mapping
                                '--RGBD/LoopClosureReextractFeatures false '  # false=localization, true=mapping
                                '--Vis/MinInliers 15 '  # default=20
                                '--Vis/EstimationType 0 '  # 0=more accurate, 1=faster
                                '--Vis/MaxDepth 0 '
                                '--RGBD/LinearUpdate 0.001 '
                                '--RGBD/AngularUpdate 0.001 '
                                '--GFTT/QualityLevel 0.00001 '
                                '--Stereo/MinDisparity 0 '
                                '--Stereo/MaxDisparity 64 '  # default=128.0, 64 [tested]
                                '--Stereo/OpticalFlow true '  # default=false
                                '--Vis/BundleAdjustment 1 '
                                '--Vis/CorNNDR 0.6 '
                                '--Vis/CorGuessWinSize 20 '
                                '--Vis/PnPFlags 0 '
                                '--Vis/CorType 0 '  # 0=Features Matching, 1=Optical Flow
                                '--Reg/Force3DoF true '
                                '--RGBD/NeighborLinkRefining true '  # when using laserscan
                                '--RGBD/ProximityBySpace true '  # when using laserscan
                                '--Reg/Strategy 1 '  # when using laserscan. 0=Vis, 1=Icp, 2=VisIcp.
                                '--Icp/VoxelSize 0.05 '
                                '--Icp/MaxCorrespondenceDistance 0.1 '
                                '--Grid/FromDepth False '
                # set DetectionRate to 0 to use image rate. Default=1
                                '--Rtabmap/DetectionRate 30 '
                # '--RGBD/CreateOccupancyGrid false '
                # Grid/Sensor: 0=laser scan, 1=depth image(s), 2=both
                                '--Grid/Sensor 0 '
                                '--Grid/RangeMax 12.0 '  # 0=inf
                                '--Optimizer/Slam2D true '
                                '--Optimizer/GravitySigma 0',
            }.items()
    )

    # common rtabmap parameters (to avoid having to create multiple LaunchConfigurations to cast as strings)
    parameters = {
        'Odom/Strategy': '0',  # 0=Frame-to-map (accurate), 1=Frame-to-Frame (faster)
        'Odom/Holonomic': 'False',
        'Odom/FilteringStrategy': '0',  # odom output filtering. 0=None, 1=KF, 2=PF
        'OdomF2M/BundleAdjustment': '1',  # 0=disabled, 1=g2o
        'Odom/GuessMotion': 'True',
        'Odom/KeyFrameThr': '0.3',  # default = 0.3. 0.6
        'Odom/ScanKeyFrameThr': '0.3',
        'Odom/ResetCountdown': '1',  # reset X frames after losing odometry
        'Rtabmap/StartNewMapOnLoopClosure': 'True',
        'Optimizer/GravitySigma': '0',
        'Icp/VoxelSize': '0.05',
        # 'Icp/RangeMax': '12.0',
        # 'Icp/Force4DoF': 'True'
        # 'Reg/Force3DoF': 'True',  # leads to instability for now
    }

    # RGB-D odometry
    rtabmap_rgbd_odometry = Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            condition=IfCondition(launch_rgbd_odometry),
            name='rtabmap_rgbd_odom',
            namespace='rtabmap_rgbd_odom',
            parameters=[parameters],
            output='screen',
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
                # ('imu', '/vehicle/sensors/imu/data'),  # imu must have orientation
                ('odom', '/odom/rtabmap/rgbd'),
                ('odom_last_frame', '/rtabmap/rgbd/points'),  # 'odom_last_frame ', 'odom_filtered_input_scan'
            ]
    )

    # Stereo odometry
    rtabmap_stereo_odometry = Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            condition=IfCondition(launch_stereo_odometry),
            name='rtabmap_stereo_odom',
            namespace='rtabmap_stereo_odom',
            parameters=[parameters],
            output='screen',
            remappings=[
                ('left/image_rect', '/camera/camera/infra1/image_rect_raw'),
                ('left/camera_info', '/camera/camera/infra1/camera_info'),
                ('right/image_rect', '/camera/camera/infra2/image_rect_raw'),
                ('right/camera_info', '/camera/camera/infra2/camera_info'),
                # ('imu', '/vehicle/sensors/imu/data'),  # imu must have orientation
                ('odom', '/odom/rtabmap/stereo'),
                ('odom_last_frame', '/rtabmap/stereo/points'),  # 'odom_last_frame ', 'odom_filtered_input_scan'
            ]
    )

    # PointCloud Odometry (kiss-icp). todo: also make the pointcloud topic dynamic
    kiss_icp_node = Node(
            package="kiss_icp",
            executable="kiss_icp_node",
            condition=IfCondition(launch_pointcloud_odometry),
            name="kiss_icp_node",
            output="screen",
            remappings=[
                ("pointcloud_topic", "/camera/camera/depth/color/points"),  # /camera/downsampled_cloud_from_depth
            ],
            parameters=[  # todo: see Open3D's realsense settings for ideas
                {
                    # ROS node configuration
                    "base_frame": "base_link",
                    "odom_frame": odom_frame,
                    "publish_odom_tf": publish_odom_tf,
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
    rtabmap_icp_odometry = Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            condition=IfCondition(launch_laserscan_odometry),
            name='rtabmap_icp_odom',
            namespace='rtabmap_icp_odom',
            parameters=[
                parameters,
                {
                    'qos': 1,
                    'expected_update_rate': 10.0,
                    'wait_for_transform': 0.3,
                    'wait_imu_to_init': False,
                    'use_sim_time': use_sim_time,
                    'queue_size': 10,
                    'approx_sync': False,
                    'frame_id': base_frame,
                    'odom_frame_id': odom_frame,
                    # 'guess_frame_id': odom_frame,
                    'guess_min_translation': 0.05,  # m
                    'guess_min_rotation': 0.005,  # rad
                    'publish_tf': publish_odom_tf,
                    'publish_null_when_lost': True,
                    # 'rtabmap_config_path': rtabmap_database_file_path,
                }
            ],
            output='screen',
            remappings=[
                ('scan', '/lidar/scan_filtered'),
                # ('imu', '/vehicle/sensors/imu/data'),  # imu must have orientation
                ('odom', '/odom/rtabmap/icp'),
                ('odom_last_frame', '/rtabmap/icp/points'),  # 'odom_last_frame ', 'odom_filtered_input_scan'
            ]
    )

    rf2o_odometry_node = Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            condition=IfCondition(launch_laserscan_odometry),
            # name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/lidar/scan_filtered',
                'odom_topic': '/odom/rf2o',
                'publish_tf': False,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'use_sim_time': use_sim_time,
                'freq': 10.0}],
    )

    laser_scan_matcher_node = Node(
            package='ros2_laser_scan_matcher',
            executable='laser_scan_matcher',
            condition=IfCondition(launch_laserscan_odometry),
            name='laser_scan_matcher_node',
            output='screen',
            parameters=[{
                'publish_odom': '/odom/laser_scan_matcher',
                'publish_tf': False,
                'laser_frame': 'lidar',
                'base_frame': 'base_link',
                'odom_frame': 'odom_laser_scan_matcher',
                'map_frame': 'map',
                'init_pose_from_topic': '',
                'use_sim_time': use_sim_time,
                'freq': 20.0}],
            remappings=[('scan', '/lidar/scan'),
                        ('odom', '/odom/laser_scan_matcher')]
    )

    # RTabMap Group
    rtabmap_group = GroupAction(
            actions=[
                # Set common parameters
                SetParameter(name='use_sim_time', value=use_sim_time),
                SetParameter(name='queue_size', value='10'),
                SetParameter(name='approx_sync', value='False'),
                SetParameter(name='frame_id', value=base_frame),
                SetParameter(name='odom_frame_id', value=odom_frame),
                # SetParameter(name='guess_frame_id', value=odom_frame),
                SetParameter(name='guess_min_translation', value='0.05'),  # m
                SetParameter(name='guess_min_rotation', value='0.005'),  # rad
                SetParameter(name='publish_tf', value=publish_odom_tf),
                SetParameter(name='wait_for_transform', value='0.3'),
                SetParameter(name='wait_imu_to_init', value='False'),
                SetParameter(name='publish_null_when_lost', value='True'),
                # SetParameter(name='rtabmap_config_path', value=rtabmap_database_file_path),

                # Set remapping rules
                SetRemap(src='scan', dst='/lidar/scan_filtered'),
                # imu must have orientation. /vehicle/sensors/imu/data
                SetRemap(src='imu', dst='/camera/camera/imu/filtered'),

                # add nodes
                rtabmap_rgbd_odometry,
                rtabmap_stereo_odometry,
                # rtabmap_icp_odometry,  # set 'wait_imu_to_init' to False
            ]
    )

    # GPU group
    visual_slam_launch_include = TimerAction(
            period=1.5,
            actions=[
                IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                                PathJoinSubstitution(
                                        [nvidia_isaac_launch_dir, 'isaac_ros_visual_slam_realsense.launch.py'])
                        ),
                        condition=IfCondition(launch_stereo_odometry),
                        launch_arguments={
                            'use_sim_time': use_sim_time,
                            'two_d_mode': 'True',
                            'base_frame': 'base_link',
                            'publish_map_to_odom_tf': 'False',
                            'publish_odom_to_baselink_tf': publish_odom_tf,
                            'launch_realsense_driver': 'False',
                            'left_image_topic': '/camera/realsense_splitter_node/output/infra_1',
                            'right_image_topic': '/camera/realsense_splitter_node/output/infra_2',
                            'image_qos': 'DEFAULT',  # todo: as RTABMAP also takes in QoS arguments
                            'imu_qos': 'DEFAULT',
                            'attach_to_shared_component_container': 'False',
                            'component_container_name': 'visual_slam_launch_container',  # todo: add to container
                        }.items()
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
            remappings=[('scan', '/lidar/scan_filtered'),
                        ('flatscan', 'flatscan_localization')]
    )

    occupancy_grid_localizer_node = ComposableNode(
            package='isaac_ros_occupancy_grid_localizer',
            plugin='nvidia::isaac_ros::occupancy_grid_localizer::OccupancyGridLocalizerNode',
            name='occupancy_grid_localizer',
            condition=IfCondition(launch_amcl),
            parameters=[  # todo: get parameters
                {
                    'use_sim_time': use_sim_time,
                    'loc_result_frame': 'map',
                    'map_yaml_path': map_file,
                }
            ],
            remappings=[
                ('localization_result', '/initialpose')
            ]
    )

    # todo: use Node instead of ComposableNodeContainer
    occupancy_grid_localizer_container = GroupAction(
            actions=[
                PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
                Node(
                        name='occupancy_grid_localizer_container',
                        namespace='',  # namespace
                        package='rclcpp_components',
                        executable='component_container_mt',
                        output='screen',
                        condition=UnlessCondition(use_composition),
                        # composable_node_descriptions=[
                        #     occupancy_grid_localizer_node,
                        #     laserscan_to_flatscan_node
                        # ],
                )
            ]
    )

    load_gpu_laser_composable_node = LoadComposableNodes(
            target_container=container_name_full,
            composable_node_descriptions=[
                occupancy_grid_localizer_node,
                laserscan_to_flatscan_node
            ]
    )

    occupancy_grid_localizer_group = GroupAction(
            actions=[
                occupancy_grid_localizer_container,
                load_gpu_laser_composable_node
            ]
    )

    gpu_group = GroupAction(
            actions=[
                # Set common parameters
                SetParameter(name='use_sim_time', value=use_sim_time),

                # add nodes
                visual_slam_launch_include,
                occupancy_grid_localizer_group,
            ]
    )

    # Create Launch Description and add nodes to the launch description
    ld = LaunchDescription([
        stdout_linebuf_envvar,
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        launch_pointcloud_odometry_la,
        launch_rgbd_odometry_la,
        launch_stereo_odometry_la,
        launch_laserscan_odometry_la,
        launch_amcl_la,
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
        declare_autostart_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        load_nodes,
        load_composable_nodes,
        slam_toolbox_localizer_node,
        ekf_nodes,
        rtabmap_localizer_node,
        rtabmap_database_file_la,
        base_frame_la,
        odom_frame_la,
        publish_odom_tf_la,
        use_gpu_la,
        rtabmap_group,
        rtabmap_icp_odometry,
        kiss_icp_node,
        gpu_group,
        # rtabmap_icp_odometry,
        # rf2o_odometry_node,
        # laser_scan_matcher_node
    ])

    return ld
