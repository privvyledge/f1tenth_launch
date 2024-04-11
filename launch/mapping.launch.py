"""
Launches (optionally): 2D mapping (online or offline), 3D mapping (online or offline).
    Can also choose GPU/CPU mode
"""
import os
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, SetRemap, PushRosNamespace, SetParametersFromFile, SetParameter
from launch_ros.descriptions import ParameterFile
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression, \
    EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, GroupAction, \
    SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
    # Get package directories
    f1tenth_launch_dir = get_package_share_directory('f1tenth_launch')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Get launch directories
    f1tenth_launch_bringup_dir = os.path.join(f1tenth_launch_dir, 'launch')
    nav2_bringup_launch_dir = os.path.join(nav2_bringup_dir, 'launch')
    vehicle_include_dir = os.path.join(f1tenth_launch_bringup_dir, 'vehicle')
    sensor_include_dir = os.path.join(f1tenth_launch_bringup_dir, 'sensors')
    localization_include_dir = os.path.join(f1tenth_launch_bringup_dir, 'localization')

    # Setup default directories.
    map_file_path = os.path.join(f1tenth_launch_dir, 'data', 'maps', 'raslab.yaml')
    rviz_config_path = os.path.join(f1tenth_launch_dir, 'config', 'f1tenth.rviz')
    offline_mapping_2d_param_file_path = os.path.join(f1tenth_launch_dir, "config/mapping/2d_mapping_offline.yaml")
    online_mapping_2d_param_file_path = os.path.join(f1tenth_launch_dir, "config/mapping/2d_mapping_online.yaml")
    default_2d_map_file_path = os.path.join(f1tenth_launch_dir, 'data', 'maps', 'raslab.yaml')
    rtabmap_database_file_path = os.path.join(f1tenth_launch_dir, 'data', 'maps', 'rtabmap', 'rtabmap.db')

    # Setup launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace', default=False)
    map_yaml_file = LaunchConfiguration('map', default=map_file_path)
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    autostart = LaunchConfiguration('autostart', default='True')
    use_composition = LaunchConfiguration('use_composition', default='True')
    use_respawn = LaunchConfiguration('use_respawn', default='False')
    launch_joystick = LaunchConfiguration('launch_joystick', default=True)
    launch_sensors = LaunchConfiguration('launch_sensors', default=True)
    launch_vehicle = LaunchConfiguration('launch_vehicle', default=True)
    launch_tfs = LaunchConfiguration('launch_tfs', default=True)
    launch_localization = LaunchConfiguration('launch_localization', default=True)
    launch_local_localization = LaunchConfiguration('launch_local_localization', default=True)
    launch_global_localization = LaunchConfiguration('launch_global_localization', default=False)
    launch_visualization = LaunchConfiguration('launch_visualization', default='True')
    rviz_config_file = LaunchConfiguration('rviz_config_file', default=rviz_config_path)
    launch_2d_mapping = LaunchConfiguration('launch_2d_mapping', default=True)
    launch_3d_mapping = LaunchConfiguration('launch_3d_mapping', default=True)
    offline_mapping_2d_param_file = LaunchConfiguration('offline_mapping_2d_param_file',
                                                        default=offline_mapping_2d_param_file_path)
    online_mapping_2d_param_file = LaunchConfiguration('online_mapping_2d_param_file',
                                                       default=online_mapping_2d_param_file_path)
    map_2d_file = LaunchConfiguration('default_2d_map_file', default=default_2d_map_file_path)
    rtabmap_database_file = LaunchConfiguration('rtabmap_database_file', default=rtabmap_database_file_path)

    # Declare launch arguments
    stdout_linebuf_envvar = SetEnvironmentVariable(
            'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
            'use_namespace',
            default_value=use_namespace,
            description='Whether to apply a namespace to the navigation stack')

    declare_map_yaml_cmd = DeclareLaunchArgument(
            'map',
            default_value=map_yaml_file,
            description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true')

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

    # Add launch arguments to a list
    launch_args = [
        stdout_linebuf_envvar,
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        declare_map_yaml_cmd,
        declare_use_sim_time_cmd,
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
        launch_visualization_arg,
        rviz_config_arg,
        launch_2d_mapping_arg,
        launch_3d_mapping_arg,
        offline_mapping_2d_param_file_la,
        online_mapping_2d_param_file_la,
        map_2d_file_la,
        rtabmap_database_file_la
    ]

    ''' Launch Nodes '''
    use_sim_time_string = use_sim_time.perform(LaunchContext())  # todo: check data type

    teleop_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([f1tenth_launch_bringup_dir, 'teleop.launch.py'])
            ),
            launch_arguments={
                "launch_joystick": launch_joystick,
                "launch_sensors": launch_sensors,
                "launch_vehicle": launch_vehicle,
                "launch_tfs": launch_tfs,
                "launch_localization": launch_localization,
                "launch_local_localization": launch_local_localization,
                "launch_global_localization": launch_global_localization,
                "launch_visualization": 'False',
                "rviz_config_file": rviz_config_file,
            }.items()
    )

    mapping_2d_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([f1tenth_launch_bringup_dir, 'mapping', '2d_mapping.launch.py'])
            ),
            condition=IfCondition(launch_2d_mapping),
            launch_arguments={
                "offline_mapping": use_sim_time,
                "use_sim_time": use_sim_time,
                "offline_mapping_param_file": offline_mapping_2d_param_file,
                "online_mapping_param_file": online_mapping_2d_param_file,
                "map_file_path": map_2d_file,
                "map_topic": 'map',
                "with_rviz": 'True',  # launch_visualization
                # "rviz_cfg_path_param": rviz_config_path,
            }.items()
    )

    mapping_3d_queue_size = '10000'  # offline mapping
    if use_sim_time_string.lower() == 'false':
        # online_mapping
        mapping_3d_queue_size = '50'  # online mapping

    mapping_3d_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([f1tenth_launch_bringup_dir, 'mapping', '3d_mapping.launch.py'])
            ),
            condition=IfCondition(launch_3d_mapping),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "use_stereo": 'False',  # False=use_depth, True=use_stereo
                "localization": 'False',
                "queue_size": mapping_3d_queue_size,
                "publish_map_tf": 'True',
                "wait_imu_to_init": 'True',
                "imu_topic": '/camera/camera/imu/filtered',
                "depth_topic": '/camera/camera/aligned_depth_to_color/image_raw',  # /camera/camera/depth/image_rect_raw
                "approx_sync": 'True',
                "rtabmap_viz_view": 'True',  # launch_visualization
                "rviz_view": 'True',  # launch_visualization
                "database_path": rtabmap_database_file,
                "rtabmap_args": '-d '
                                '--RGBD/LoopClosureReextractFeatures true '
                                '--Vis/MinInliers 15 '
                                '--Vis/EstimationType 1 '
                                '--Vis/MaxDepth 0 '
                                '--GFTT/QualityLevel 0.00001 '
                                '--Stereo/MinDisparity 0 '
                                '--Stereo/MaxDisparity 64 '
                                # '--Vis/RoiRatios "0 0 0 .2" '
                                # "--Kp/RoiRatios '0 0 0 .2' "
                                # '--Odom/GuessMotion true '
                                '--Vis/BundleAdjustment 1 '
                                # '--OdomF2M/BundleAdjustment 1 '
                                '--Vis/CorNNDR 0.6 '
                                '--Vis/CorGuessWinSize 20 '
                                '--Vis/PnPFlags 0 '
                                # '--Odom/Strategy 1 '
                                '--Vis/CorType 1 '
                                # '--Odom/KeyFrameThr 0.6'
                                '--Reg/Force3DoF true '
                                '--Rtabmap/DetectionRate 0 '  # set to 0 to use image rate
                                '--Grid/Sensor 0 '  # set to 1 if not using laserscan, 0 otherwise
                                '--Optimizer/Slam2D true '
                                '--Optimizer/GravitySigma 0',
            }.items()
    )

    # Add launch arguments and nodes to the launch description
    ld = LaunchDescription(
            launch_args + [
                teleop_launch,
                mapping_2d_node,
                mapping_3d_node
            ]
    )
    return ld
