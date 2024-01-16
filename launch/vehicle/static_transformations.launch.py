#!/usr/bin/env python3
"""
Todo: add vesc imu and include rotations
Todo: switch to new style
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    lidar_static_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_baselink_to_laser',
            arguments=['0.11815', '0.0', '0.1491', '0.0', '0.0', '0.0', 'base_link', 'lidar']
    )  # YDLidar

    # camera_static_tf_node = Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='static_baselink_to_camera',
    #         arguments=['0.24115', '0.0', '0.0961', '0.0', '0.0', '0.0', 'base_link', 'camera_link']
    # )  # Realsense

    vesc_imu_static_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_baselink_to_vesc_imu',
            arguments=['0.13', '0.0', '0.043', '-1.57079633', '0.0', '3.14159265', 'base_link', 'imu_link']
    )  # VESC IMU (/sensors/imu/raw)

    base_footprint_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_baselink_to_basefootprint',
            arguments=['0.0', '0.0', '-0.033', '0.0', '0.0', '0.0', 'base_link', 'base_footprint']
    )

    ld = LaunchDescription([
        lidar_static_tf_node,
        camera_static_tf_node,
        vesc_imu_static_tf_node,
        base_footprint_tf_node,
    ])

    return ld
