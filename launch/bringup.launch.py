"""
Todo:
    * Setup mapping launch
    * Setup composable nodes
    * Setup nav2_navigation individual subsystem launching
    * seetup loglevel
"""
import os
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, SetRemap, PushRosNamespace, SetParametersFromFile, SetParameter
from launch_ros.descriptions import ParameterFile
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, GroupAction, SetEnvironmentVariable, LogInfo
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

    # Setup default directories
    nav2_params_file_path = os.path.join(f1tenth_launch_dir, 'config', 'nav2_params.yaml')
    map_file_path = os.path.join(f1tenth_launch_dir, 'data', 'maps', 'raslab.yaml')
    rviz_config_path = os.path.join(f1tenth_launch_dir, 'config', 'f1tenth.rviz')
    offline_mapping_2d_param_file_path = os.path.join(f1tenth_launch_dir, "config/mapping/2d_mapping_offline.yaml")
    online_mapping_2d_param_file_path = os.path.join(f1tenth_launch_dir, "config/mapping/2d_mapping_online.yaml")
    default_2d_map_file_path = os.path.join(f1tenth_launch_dir, 'data', 'maps', 'raslab.yaml')
    rtabmap_database_file_path = os.path.join(f1tenth_launch_dir, 'data', 'maps', 'rtabmap', 'rtabmap.db')

    # Setup launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace', default=False)
    slam = LaunchConfiguration('slam', default=False)
    map_yaml_file = LaunchConfiguration('map', default=map_file_path)
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    params_file = LaunchConfiguration('params_file',
                                      default=nav2_params_file_path)
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    launch_joystick = LaunchConfiguration('launch_joystick', default=True)
    launch_sensors = LaunchConfiguration('launch_sensors', default=True)
    launch_vehicle = LaunchConfiguration('launch_vehicle', default=True)
    launch_tfs = LaunchConfiguration('launch_tfs', default=True)
    launch_localization = LaunchConfiguration('launch_localization', default=True)
    launch_local_localization = LaunchConfiguration('launch_local_localization', default=True)
    launch_global_localization = LaunchConfiguration('launch_global_localization', default=False)
    launch_navigation = LaunchConfiguration('launch_navigation', default=True)
    launch_visualization = LaunchConfiguration('launch_visualization', default=False)
    rviz_config_file = LaunchConfiguration('rviz_config_file', default=rviz_config_path)
    launch_2d_mapping = LaunchConfiguration('launch_2d_mapping', default=True)
    launch_3d_mapping = LaunchConfiguration('launch_3d_mapping', default=True)
    offline_mapping_2d_param_file = LaunchConfiguration('offline_mapping_2d_param_file',
                                                        default=offline_mapping_2d_param_file_path)
    online_mapping_2d_param_file = LaunchConfiguration('online_mapping_2d_param_file',
                                                       default=online_mapping_2d_param_file_path)
    map_2d_file = LaunchConfiguration('default_2d_map_file', default=default_2d_map_file_path)
    rtabmap_database_file = LaunchConfiguration('rtabmap_database_file', default=rtabmap_database_file_path)

    # Setup Remappings/renamings
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    # It only applies when `use_namespace` is True.
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

    declare_slam_cmd = DeclareLaunchArgument(
            'slam',
            default_value=slam,
            description='Whether to run SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
            'map',
            default_value=map_yaml_file,
            description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
            'use_composition', default_value='True',
            description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
            'use_respawn', default_value='False',
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
    launch_navigation_arg = DeclareLaunchArgument('launch_navigation',
                                                           default_value=launch_navigation,
                                                           description="Launch the navigation.")
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
        declare_slam_cmd,
        declare_map_yaml_cmd,
        declare_use_sim_time_cmd,
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
        launch_navigation_arg,
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
    component_container_node = Node(
            condition=IfCondition(use_composition),
            name='f1tenth_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen')

    joystick_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([vehicle_include_dir, 'joystick.launch.py'])
            ),
            condition=IfCondition(launch_joystick)
    )

    ackermann_mux_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([vehicle_include_dir, 'ackermann_mux.launch.py'])
            ),
            condition=IfCondition(launch_joystick)
    )

    sensors_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([sensor_include_dir, 'sensors.launch.py'])
            ),
            condition=IfCondition(launch_sensors)
    )

    vehicle_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([vehicle_include_dir, 'vehicle.launch.py'])
            ),
            condition=IfCondition(launch_vehicle),
            launch_arguments={
                "launch_imu_filter": 'True',
                "launch_ackermann_to_vesc_node": 'True',
                "launch_vesc_to_odom_node": 'True',
                "launch_throttle_interpolator_node": 'False'
            }.items()
    )

    tf_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([vehicle_include_dir, 'static_transformations.launch.py'])
            ),
            condition=LaunchConfigurationEquals('launch_tfs', 'True')
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
                            "launch_sensor_fusion": 'True',
                            "launch_ekf_odom": launch_local_localization,
                            "launch_ekf_map": launch_global_localization,
                            "launch_slam_toolbox_localizer": 'False',
                            "launch_rtabmap_localizer": 'False',
                            "map": map_yaml_file,
                            "use_sim_time": use_sim_time,
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
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': LaunchConfiguration("sim")}]
        )
    
    mapping_launch = TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(f1tenth_launch_bringup_dir, 'mapping.launch.py')),
                        condition=IfCondition(slam),
                        launch_arguments={'namespace': namespace,
                                          'use_namespace': use_namespace,
                                          'use_sim_time': use_sim_time,
                                          'autostart': autostart,
                                          'use_composition': use_composition,
                                          'use_respawn': use_respawn,
                                          "launch_joystick": 'False',
                                          "launch_sensors": 'False',
                                          "launch_vehicle": 'False',
                                          "launch_tfs": 'False',
                                          "launch_localization": 'False',
                                          "launch_local_localization": 'False',
                                          "launch_global_localization": 'False',
                                          "launch_visualization": 'True',
                                          "rviz_config_file": rviz_config_file,
                                          "launch_2d_mapping": launch_2d_mapping,
                                          "launch_3d_mapping": launch_3d_mapping,
                                          "offline_mapping_2d_param_file": offline_mapping_2d_param_file,
                                          "online_mapping_2d_param_file": online_mapping_2d_param_file,
                                          "map_2d_file": map_2d_file,
                                          "rtabmap_database_file": rtabmap_database_file,
                                          }.items())
            ]
    )

    nav2_navigation_launch = TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution([f1tenth_launch_bringup_dir, 'nav2_navigation.launch.py'])),
                        condition=IfCondition(launch_navigation),
                        launch_arguments={'namespace': namespace,
                                          'use_sim_time': use_sim_time,
                                          'autostart': autostart,
                                          'params_file': params_file,
                                          'use_composition': use_composition,
                                          'use_respawn': use_respawn,
                                          'container_name': 'f1tenth_container'}.items())
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
                joystick_launch, ackermann_mux_launch, sensors_launch, vehicle_launch, tf_launch
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

                mapping_launch,

                localization_launch,

                nav2_navigation_launch,
            ]
    )

    # Add launch arguments and nodes to the launch description
    ld = LaunchDescription(
            launch_args + [
                component_container_node, visualization_launch,
                vehicle_bringup_group, nav2_bringup_group,
            ]
    )
    return ld
