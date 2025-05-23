import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get path to files and directories
    f1tenth_launch_pkg_prefix = get_package_share_directory('f1tenth_launch')

    # declare launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    use_namespace = LaunchConfiguration('use_namespace')
    namespace = LaunchConfiguration('namespace')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    map_frame = LaunchConfiguration('map_frame')
    map_file_name = LaunchConfiguration('map_file_name')
    map_tf_publish_period = LaunchConfiguration('map_tf_publish_period')

    # Declare default launch arguments
    localization_param_file = os.path.join(
            f1tenth_launch_pkg_prefix, 'config', 'localization/localizer_slam.yaml')

    # declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true')
    localization_param = DeclareLaunchArgument(
            'params_file',
            default_value=localization_param_file,
            description='Path to config file for localization nodes'
    )

    use_namespace_arg = DeclareLaunchArgument(
            'use_namespace',
            default_value='false',
            description='Use namespace'
    )

    namespace_arg = DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the localization node'
    )

    base_frame_arg = DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Base frame'
    )

    odom_frame_arg = DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='Odom frame'
    )

    map_frame_arg = DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Map frame'
    )

    map_file_name_arg = DeclareLaunchArgument(
            'map_file_name',
            default_value='/mnt/shared_dir/maps/slam_toolbox/raslab',
            description='Map file name'
    )

    map_tf_publish_period_arg = DeclareLaunchArgument(
            'map_tf_publish_period',
            default_value='0.0',
            description='Map tf publish rate'
    )

    slam_toolbox_localizer_node = Node(
                parameters=[
                    params_file,
                    {
                        'use_sim_time': use_sim_time,
                        'base_frame': base_frame,
                        'odom_frame': odom_frame,
                        'map_frame': map_frame,
                        'map_file_name': map_file_name,
                        'transform_publish_period': map_tf_publish_period
                    },
                ],
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='slam_toolbox',
                namespace=namespace,
                output='screen',
                remappings=[
                    ('pose', 'slam_toolbox/pose'),
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/scan', 'scan'),
                    ('/scan_filtered', 'scan_filtered'),
                    ('/map', 'map'),
                    ('/map_metadata', 'map_metadata')
                            ]
        )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(localization_param)
    ld.add_action(use_namespace_arg)
    ld.add_action(namespace_arg)
    ld.add_action(base_frame_arg)
    ld.add_action(odom_frame_arg)
    ld.add_action(map_frame_arg)
    ld.add_action(map_file_name_arg)
    ld.add_action(map_tf_publish_period_arg)
    ld.add_action(slam_toolbox_localizer_node)

    return ld
