"""
Todo:
    * Use a container for YDLidar
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap, PushRosNamespace, LifecycleNode
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    f1tenth_launch_dir = get_package_share_directory('f1tenth_launch')

    # Create the launch configuration variables
    lidar_config = LaunchConfiguration('lidar_config')
    launch_filter = LaunchConfiguration('launch_filter')
    use_namespace = LaunchConfiguration('use_namespace', default=True)
    namespace = LaunchConfiguration('namespace', default='lidar')

    # Launch arguments
    lidar_config_file = os.path.join(
            f1tenth_launch_dir,
            'config/sensors',
            'ydlidar_X4.yaml')

    lidar_la = DeclareLaunchArgument('lidar_config',
                                     default_value=lidar_config_file,
                                     description='Path to the YDLIDAR parameters file to use.')

    declare_launch_filter_cmd = DeclareLaunchArgument(
            'launch_filter',
            default_value='True',
            description='Whether to launch the LIDAR filter')

    declare_use_namespace_cmd = DeclareLaunchArgument(
            'use_namespace',
            default_value=use_namespace,
            description='Whether to apply a namespace to the entire the LIDAR launch.')

    namespace_la = DeclareLaunchArgument(
            'namespace', default_value=namespace,
            description='Namespace for the nodes')

    # Create Launch Description
    ld = LaunchDescription([lidar_la, declare_launch_filter_cmd, declare_use_namespace_cmd, namespace_la])

    # Setup nodes
    lidar_node = LifecycleNode(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[lidar_config],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ],
            namespace='',  # namespace,
    )

    laserscan_filter = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [f1tenth_launch_dir, 'launch/filters', 'laser_filter.launch.py']
            )),
            condition=IfCondition([launch_filter]),
            launch_arguments={
                'namespace': '',  # namespace,
            }.items(),
    )

    lidar_nodes = GroupAction(
            actions=[
                PushRosNamespace(
                        condition=IfCondition(use_namespace),
                        namespace=namespace
                ),
                # nodes
                lidar_node,
                laserscan_filter,
            ]
    )

    ld.add_action(lidar_nodes)

    return ld

