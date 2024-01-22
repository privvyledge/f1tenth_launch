from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch_ros.actions import PushRosNamespace
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ekf = LaunchConfiguration('use_ekf')  # ekf_node, ukf_node
    frequency = LaunchConfiguration('frequency')
    node_name = LaunchConfiguration('node_name')  # ekf_filter_node, ukf_filter_node
    log_level = LaunchConfiguration('log_level')

    # declare default arguments
    f1tenth_launch_pkg_prefix = get_package_share_directory('f1tenth_launch')
    ekf_map_config_file = os.path.join(
            f1tenth_launch_pkg_prefix, "config/localization/ekf_map.yaml")

    # declare launch arguments
    stdout_linebuf_envvar = SetEnvironmentVariable(
            'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    declare_namespace_cmd = DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
            'use_namespace',
            default_value='False',
            description='Whether to apply a namespace to the navigation stack')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
            'params_file',
            default_value=ekf_map_config_file,
            description='Full path to the Kalman filter YAML file to use.')

    declare_kf_type = DeclareLaunchArgument(
            'use_ekf', default_value='True',
            description='whether to use ekf. If false uses ukf instead')

    declare_frequency_la = DeclareLaunchArgument(
            'frequency', default_value='10.0',
            description='Sensor fusion frequency')

    declare_node_name = DeclareLaunchArgument(
            'node_name', default_value='ekf_map_node',
            description='ekf_node, ukf_node')

    declare_log_level_cmd = DeclareLaunchArgument(
            'log_level', default_value='info',
            description='log level')

    # Specify actions/nodes
    kf_bringup_group = GroupAction([
        PushRosNamespace(
                condition=IfCondition(use_namespace),
                namespace=namespace),

        # Node(
        #         condition=IfCondition(use_composition),
        #         name='nav2_container',
        #         package='rclcpp_components',
        #         executable='component_container_isolated',
        #         parameters=[configured_params, {'autostart': autostart}],
        #         arguments=['--ros-args', '--log-level', log_level],
        #         remappings=remappings,
        #         output='screen'),

        Node(
                condition=IfCondition([use_ekf]),
                package='robot_localization',
                executable='ekf_node',
                name=node_name,
                output='screen',
                parameters=[
                    params_file,
                    {'frequency': frequency},
                    {'use_sim_time': use_sim_time}
                ],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[
                    ('odometry/filtered', 'odometry/global'),
                    ('accel/filtered', 'accel/global'),
                ]
        ),

        Node(
                condition=IfCondition(PythonExpression(['not ', use_ekf])),
                package='robot_localization',
                executable='ukf_node',
                name=node_name,
                output='screen',
                parameters=[
                    params_file,
                    {'frequency': frequency},
                    {'use_sim_time': use_sim_time}
                ],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[
                    ('odometry/filtered', 'odometry/global'),
                    ('accel/filtered', 'accel/global'),
                ]
        ),
    ])

    # Create the launch description
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_kf_type)
    ld.add_action(declare_frequency_la)
    ld.add_action(declare_node_name)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(kf_bringup_group)

    return ld
