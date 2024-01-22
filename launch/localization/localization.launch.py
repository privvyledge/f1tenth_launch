"""
todo: load nodes using composition (https://github.com/ros-planning/navigation2/blob/humble/nav2_bringup/launch/localization_launch.py#L145)
This nodes sets up local and global localization.
* Load map (yaml and/or posegraph)
* Launch AMCL localizer
* Launch slam_toolbox localizer
* (optional) Launch particle_filter localizer
* todo: Get arguments for downstream components, e.g. IMU filter, LIDAR filter and other configuration variables.
* Launch Kalman Filter (EKF or UKF) nodes

"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # Get path to files and directories
    nav2_pkg_prefix = get_package_share_directory('nav2_bringup')
    f1tenth_launch_pkg_prefix = get_package_share_directory('f1tenth_launch')

    # declare launch configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_composition = LaunchConfiguration('use_composition')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    launch_slam_toolbox_localizer = LaunchConfiguration('launch_slam_toolbox_localizer')
    log_level = LaunchConfiguration('log_level')

    # Declare default launch arguments
    localization_param_file = os.path.join(
            f1tenth_launch_pkg_prefix, 'config', 'localization/localizer_amcl.yaml')
    map_file_path = os.path.join(
            f1tenth_launch_pkg_prefix, 'data/maps', 'raslab.yaml')
    # ekf_param_file = os.path.join(
    #         f1tenth_launch_pkg_prefix, 'config', '/ekf.yaml')
    #
    # imu_filter_param_file = os.path.join(
    #         f1tenth_launch_pkg_prefix, "config/imu_filter.yaml")
    # laser_filter_param_file = os.path.join(
    #         f1tenth_launch_pkg_prefix, "config/filters/laser_filter.yaml")

    # declare launch arguments
    stdout_linebuf_envvar = SetEnvironmentVariable(
            'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    declare_namespace_cmd = DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true')
    localization_param = DeclareLaunchArgument(
            'params_file',
            default_value=localization_param_file,
            description='Path to config file for localization nodes'
    )
    map_file_la = DeclareLaunchArgument(
            'map',
            default_value=map_file_path,
            description='Path to 2D map config file'
    )
    launch_slam_toolbox_localizer_la = DeclareLaunchArgument(
            'launch_slam_toolbox_localizer',
            default_value='True',
            description='Whether to launch slam toolbox\'s localizer'
    )
    declare_use_composition_cmd = DeclareLaunchArgument(
            'use_composition', default_value='False',
            description='Use composed bringup if True')
    declare_autostart_cmd = DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack')
    declare_use_respawn_cmd = DeclareLaunchArgument(
            'use_respawn', default_value='False',
            description='Whether to respawn if a node crashes. Applied when composition is disabled.')
    declare_log_level_cmd = DeclareLaunchArgument(
            'log_level', default_value='info',
            description='log level')

    lifecycle_nodes = ['map_server', 'amcl']
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

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
                        name='amcl',
                        output='screen',
                        respawn=use_respawn,
                        respawn_delay=2.0,
                        parameters=[configured_params],
                        arguments=['--ros-args', '--log-level', log_level],
                        remappings=remappings),
                Node(
                        package='nav2_lifecycle_manager',
                        executable='lifecycle_manager',
                        name='lifecycle_manager_localization',
                        output='screen',
                        arguments=['--ros-args', '--log-level', log_level],
                        parameters=[{'use_sim_time': use_sim_time},
                                    {'autostart': autostart},
                                    {'node_names': lifecycle_nodes}])
            ]
    )

    slam_toolbox_localizer_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [f1tenth_launch_pkg_prefix, 'launch/localization', 'slam_localization.launch.py']
            )),
            condition=IfCondition([launch_slam_toolbox_localizer]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                # 'params_file': '',
            }.items()
    )

    ekf_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    os.path.join(f1tenth_launch_pkg_prefix,
                                 'launch/localization/dual_ekf.launch.py')
            )
    )


    imu_filter_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    os.path.join(f1tenth_launch_pkg_prefix,
                                 'launch/filters/imu_filter.launch.py')
            )
    )

    # laser_filter_node = Node(
    #         package="laser_filters",
    #         namespace='lidar',
    #         executable="scan_to_scan_filter_chain",
    #         parameters=[laser_filter_param_file],
    #         remappings=[
    #             ('output', 'scan'),
    #             ('scan', '/lidar/scan')
    #         ]
    # )

    laser_filter_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    os.path.join(f1tenth_launch_pkg_prefix,
                                 'launch/filters/laser_filter.launch.py')
            )
    )

    # Create Launch Description and add nodes to the launch description
    ld = LaunchDescription([
        stdout_linebuf_envvar,
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        localization_param,
        map_file_la,
        launch_slam_toolbox_localizer_la,
        declare_use_composition_cmd,
        declare_autostart_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        load_nodes,
        slam_toolbox_localizer_node,
        # ekf_node,
        # imu_filter_node,
        # laser_filter_node,
    ])

    return ld
