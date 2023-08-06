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
    ydlidar_parameter_file = LaunchConfiguration('params_file')
    ydlidar_node_name = 'ydlidar_ros2_driver_node'
    ###
    lidar_config = os.path.join(
            get_package_share_directory('f1tenth_launch'),
            'config',
            'ydlidar_X4.yaml')

    lidar_la = DeclareLaunchArgument('lidar_config',
                                     default_value=lidar_config,
                                     description='Path to the YDLIDAR parameters file to use.')

    ld = LaunchDescription([lidar_la])

    lidar_node = LifecycleNode(package='ydlidar_ros2_driver',
                               node_executable='ydlidar_ros2_driver_node',
                               node_name='ydlidar_ros2_driver_node',
                               output='screen',
                               emulate_tty=True,
                               parameters=[LaunchConfiguration('lidar_config')],
                               node_namespace='lidar',
                               )

    # todo: include Realsense ROS package
    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy',
            parameters=[LaunchConfiguration('joy_config')]
    )
    joy_teleop_node = Node(
            package='joy_teleop',
            executable='joy_teleop',
            name='joy_teleop',
            parameters=[LaunchConfiguration('joy_config')]
    )
    ackermann_to_vesc_node = Node(
            package='vesc_ackermann',
            executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node',
            parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_to_odom_node = Node(
            package='vesc_ackermann',
            executable='vesc_to_odom_node',
            name='vesc_to_odom_node',
            parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_driver_node = Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            parameters=[LaunchConfiguration('vesc_config')]
    )
    throttle_interpolator_node = Node(
            package='f1tenth_stack',
            executable='throttle_interpolator',
            name='throttle_interpolator',
            parameters=[LaunchConfiguration('vesc_config')]
    )
    urg_node = Node(
            package='urg_node',
            executable='urg_node_driver',
            name='urg_node',
            parameters=[LaunchConfiguration('sensors_config')]
    )
    ackermann_mux_node = Node(
            package='ackermann_mux',
            executable='ackermann_mux',
            name='ackermann_mux',
            parameters=[LaunchConfiguration('mux_config')],
            remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    )

    # finalize
    ld.add_action(urg_node)
    return ld
