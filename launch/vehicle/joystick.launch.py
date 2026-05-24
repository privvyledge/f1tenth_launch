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
    joy_config_file = os.path.join(
            f1tenth_launch_dir,
            'config/vehicle',
            'joy_config.yaml'
    )
    joy_teleop_config_file = os.path.join(
            f1tenth_launch_dir,
            'config/vehicle',
            'joy_teleop.yaml'
    )

    joy_config = LaunchConfiguration('joy_config')
    joy_teleop_config = LaunchConfiguration('joy_teleop_config')
    require_deadman = LaunchConfiguration('require_deadman', default='True')
    deadman_buttons = LaunchConfiguration('deadman_buttons', default="[4, 9]")
    steering_button = LaunchConfiguration('steering_button', default=2)
    max_speed = LaunchConfiguration('max_speed', default=5.0)
    max_steering = LaunchConfiguration('max_steering', default=0.34)

    joy_la = DeclareLaunchArgument(
            'joy_config',
            default_value=joy_config_file,
            description='Path to joy node config (device_id, deadzone, autorepeat_rate)')

    joy_teleop_la = DeclareLaunchArgument(
            'joy_teleop_config',
            default_value=joy_teleop_config_file,
            description='Path to joy_teleop node config (axis mappings, deadman buttons)')

    require_deadman_la = DeclareLaunchArgument(
            'require_deadman',
            default_value='True',
            description='Require a deadman button (L1/LB) to be held to arm actuators. '
                        'Set False to disable the safety interlock.')

    deadman_buttons_la = DeclareLaunchArgument(
            'deadman_buttons',
            default_value=deadman_buttons,
            description='Buttons used to arm the vehicle actuators. Ignored when require_deadman:=False.')

    steering_button_la = DeclareLaunchArgument(
            'steering_button',
            default_value=steering_button,
            description='Button used to control the steering angle. 2 for DualShock/DualSense, 3 for Logitech F710')

    max_speed_la = DeclareLaunchArgument(
            'max_speed',
            default_value=max_speed,
            description='The maximum speed in m/s.')

    max_steering_la = DeclareLaunchArgument(
            'max_steering',
            default_value=max_steering,
            description='The maximum steering angle in rads.')

    # When require_deadman=False, pass an empty list so joy_teleop fires without any held button.
    effective_deadman_buttons = PythonExpression(
            ["'[]' if '", require_deadman, "' == 'False' else '", deadman_buttons, "'"])

    ld = LaunchDescription([joy_la, joy_teleop_la, require_deadman_la, deadman_buttons_la, steering_button_la, max_speed_la, max_steering_la])

    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy',
            respawn=True,
            respawn_delay=2.0,
            parameters=[joy_config],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
    )

    joy_teleop_node = Node(
            package='joy_teleop',
            executable='joy_teleop',
            name='joy_teleop',
            parameters=[
                joy_teleop_config,
                {
                    'human_control.deadman_buttons': effective_deadman_buttons,
                    'human_control.axis_mappings.drive-steering_angle.axis': steering_button,
                    'human_control.axis_mappings.drive-speed.scale': max_speed,  # max speed in m/s
                    'human_control.axis_mappings.drive-steering_angle.scale': max_steering,  # max steering in rads
                }
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
    )

    # add nodes to the launch description
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)

    return ld
