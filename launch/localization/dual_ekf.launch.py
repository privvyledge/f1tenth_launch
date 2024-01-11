"""
Todo: include ekf_launch files instead of setting nodes
Todo: add condition to select nodes
"""

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    node_type = 'ekf_node'  # ekf_node, ukf_node
    node_name = 'ekf_filter_node'  # ekf_filter_node, ukf_filter_node

    f1tenth_launch_pkg_prefix = get_package_share_directory('f1tenth_launch')
    ekf_odom_config_file = os.path.join(
            f1tenth_launch_pkg_prefix, "config/localization/ekf_odom.yaml")
    ekf_map_config_file = os.path.join(
            f1tenth_launch_pkg_prefix, "config/localization/ekf_map.yaml")

    ld = LaunchDescription([
        Node(
                package='robot_localization',
                executable=node_type,
                name=node_name,
                output='screen',
                parameters=[ekf_odom_config_file],
                remappings=[('odometry/filtered', 'odometry/local')]
        ),
        Node(
                package='robot_localization',
                executable=node_type,
                name=node_name,
                output='screen',
                parameters=[ekf_map_config_file],
                remappings=[('odometry/filtered', 'odometry/global')]
        ),
    ])

    return ld
