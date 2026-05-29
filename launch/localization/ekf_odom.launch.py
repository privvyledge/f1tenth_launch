import os
import yaml
import pathlib

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
    f1tenth_launch_pkg_prefix = get_package_share_directory('f1tenth_launch')
    ekf_odom_config_file = os.path.join(
            f1tenth_launch_pkg_prefix, "config/localization/ekf_odom.yaml")

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    publish_tf = LaunchConfiguration('publish_tf')
    use_ekf = LaunchConfiguration('use_ekf')  # ekf_node, ukf_node
    frequency = LaunchConfiguration('frequency')
    node_name = LaunchConfiguration('node_name')  # ekf_filter_node, ukf_filter_node
    gravitational_acceleration = LaunchConfiguration('gravitational_acceleration')

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
        'use_sim_time': use_sim_time}

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
            default_value=ekf_odom_config_file,
            description='Full path to the Kalman filter YAML file to use.')

    declare_use_composition_cmd = DeclareLaunchArgument(
            'use_composition', default_value='True',
            description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
            'use_respawn', default_value='False',
            description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
            'log_level', default_value='info',
            description='log level')

    declare_publish_tf_cmd = DeclareLaunchArgument(
            'publish_tf', default_value='True',
            description='Whether or not to publish the TF')

    declare_kf_type = DeclareLaunchArgument(
            'use_ekf', default_value='True',
            description='whether to use ekf. If false uses ukf instead')

    declare_frequency_la = DeclareLaunchArgument(
            'frequency', default_value='100.0',
            description='Sensor fusion frequency')

    declare_node_name = DeclareLaunchArgument(
            'node_name', default_value='ekf_odom_node',
            description='ekf_node, ukf_node')

    declare_gravitational_acceleration = DeclareLaunchArgument(
            'gravitational_acceleration', default_value='9.80665',
            description='Gravitational acceleration (m/s^2). Calibrate per robot.')

    # Specify actions/nodes
    kf_bringup_group = GroupAction([
        PushRosNamespace(
                condition=IfCondition(use_namespace),
                namespace=namespace),
        # Set common parameters
        SetParameter(name='use_sim_time', value=use_sim_time),

        # Node(
        #         condition=IfCondition(use_composition),
        #         name='nav2_container',
        #         package='rclcpp_components',
        #         executable='component_container_isolated',
        #         parameters=[params_file, {'autostart': autostart}],
        #         arguments=['--ros-args', '--log-level', log_level],
        #         remappings=remappings,
        #         output='screen'),

        Node(
                condition=IfCondition(use_ekf),
                package='robot_localization',
                executable='ekf_node',
                name=node_name,
                output='screen',
                parameters=[
                    params_file,
                    {
                        'frequency': frequency,
                        'publish_tf': publish_tf,
                        'gravitational_acceleration': gravitational_acceleration,
                    }
                ],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[
                    ('odometry/filtered', 'odometry/local'),
                    ('accel/filtered', 'accel/local'),
                    *remappings
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
                    {
                        'frequency': frequency,
                        'publish_tf': publish_tf,
                        'gravitational_acceleration': gravitational_acceleration,
                    }
                ],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[
                    ('odometry/filtered', 'odometry/local'),
                    ('accel/filtered', 'accel/local'),
                    *remappings
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
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_publish_tf_cmd)
    ld.add_action(declare_kf_type)
    ld.add_action(declare_frequency_la)
    ld.add_action(declare_node_name)
    ld.add_action(declare_gravitational_acceleration)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(kf_bringup_group)

    return ld
