from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    f1tenth_launch_dir = get_package_share_directory('f1tenth_launch')
    joy_teleop_config_file = os.path.join(
            f1tenth_launch_dir,
            'config/vehicle',
            'joy_teleop.yaml'
    )

    joy_config = LaunchConfiguration('joy_config')

    joy_la = DeclareLaunchArgument(
            'joy_config',
            default_value=joy_teleop_config_file,
            description='Descriptions for joy and joy_teleop configs')

    ld = LaunchDescription([joy_la])

    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy',
            respawn=True,
            respawn_delay=2.0,
            parameters=[joy_config]
    )

    joy_teleop_node = Node(
            package='joy_teleop',
            executable='joy_teleop',
            name='joy_teleop',
            parameters=[joy_config]
    )

    # add nodes to the launch description
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)

    return ld
