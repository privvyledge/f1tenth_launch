#!/usr/bin/env python3
"""
Todo: add vesc imu and include rotations
Todo: switch to new style
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.descriptions import ParameterFile, ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    f1tenth_launch_dir = get_package_share_directory('f1tenth_launch')
    urdf_path = os.path.join(get_package_share_directory('f1tenth_launch'), 'urdf/f1tenth.urdf.xacro')

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
    # )

    # todo: pass base_link to camera_bottom_screw as an argument
    # set use_nominal_extrinsics:=True to use ideal dimensions instead of the calibrated dimensions.
    # Useful for simulations, e.g Gazebo
    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='realsense_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro ', str(urdf_path), ' ',
                                                             'use_nominal_extrinsics:=False']), value_type=str)
            }],
            output='screen'
    )  # Realsense

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
        # camera_static_tf_node,
        robot_state_publisher_node,
        vesc_imu_static_tf_node,
        base_footprint_tf_node,
    ])

    return ld
