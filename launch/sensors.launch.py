#!/usr/bin/env python3
"""
Todo: include realsense launch files
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    lidar_config = os.path.join(
            get_package_share_directory('f1tenth_launch'),
            'config',
            'ydlidar_X4.yaml')

    lidar_la = DeclareLaunchArgument('lidar_config',
                                     default_value=lidar_config,
                                     description='Path to the YDLIDAR parameters file to use.')

    ld = LaunchDescription([lidar_la])

    lidar_node = LifecycleNode(package='ydlidar_ros2_driver',
                               executable='ydlidar_ros2_driver_node',
                               name='ydlidar_ros2_driver_node',
                               output='screen',
                               emulate_tty=True,
                               parameters=[LaunchConfiguration('lidar_config')],
                               namespace='lidar',
                               )

    ld.add_action(lidar_node)
    return ld
