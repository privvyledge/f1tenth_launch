from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    lidar_static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.11815', '0.0', '0.1491', '0.0', '0.0', '0.0', 'base_link', 'lidar']
    )  # YDLidar

    camera_static_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_baselink_to_camera',
            arguments=['0.24115', '0.0', '0.0961', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )  # Realsense

    ld = LaunchDescription([
        lidar_static_tf_node,
        camera_static_tf_node,
    ])

    return ld
