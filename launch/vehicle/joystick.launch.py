from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import ast
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
    # SDL mapping (Humble joy node, DualSense): L1=9, R1=10, PS/guide=5, Share=4.
    # The legacy joydev lore (L1=4, R1=5) is wrong here — [5] made the PS button the
    # autonomous deadman, which kept the car driving during the power-off long-press
    # (measured on gosling1 2026-08-05). Autonomous is R1 = 10.
    autonomous_deadman_buttons = LaunchConfiguration('autonomous_deadman_buttons', default="[10]")
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
                        'Set False to disable the safety interlock.'
                        'Note: The current upstream joy_teleop package requires deadman buttons to be passed. '
                        'Therefore, this is deactivated for now. Will make truly optional when I patch the upstream package.')

    deadman_buttons_la = DeclareLaunchArgument(
            'deadman_buttons',
            default_value=deadman_buttons,
            description='Buttons used to arm the vehicle actuators. Ignored when require_deadman:=False.')

    autonomous_deadman_buttons_la = DeclareLaunchArgument(
            'autonomous_deadman_buttons',
            default_value=autonomous_deadman_buttons,
            description='Buttons used to enable autonomous control. Ignored when require_deadman:=False.')

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

    ld = LaunchDescription([joy_la, joy_teleop_la, require_deadman_la, deadman_buttons_la, autonomous_deadman_buttons_la, steering_button_la, max_speed_la, max_steering_la])

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

    def launch_setup(context, *args, **kwargs):
        # Resolve the actual string value from the launch context
        require_deadman_str = require_deadman.perform(context)
        is_deadman_required = require_deadman_str.lower() in ['true', 't', '1', 'y', 'yes']

        # Initialize the baseline parameter dictionary
        teleop_params = {
            'human_control.axis_mappings.drive-steering_angle.axis': steering_button,
            'human_control.axis_mappings.drive-speed.scale': max_speed,  # max speed in m/s
            'human_control.axis_mappings.drive-steering_angle.scale': max_steering,  # max steering in rads
        }

        # Conditionally append the deadman parameter
        if is_deadman_required:
            teleop_params['human_control.deadman_buttons'] = deadman_buttons
            teleop_params['autonomous_control.deadman_buttons'] = autonomous_deadman_buttons
            # Keep the command_gate heartbeat alive in every driving state: its deadman
            # list must be the union of the human and autonomous deadman buttons, so a
            # button remap via launch args cannot drift from the joy_teleop.yaml value.
            heartbeat_buttons = sorted(
                set(ast.literal_eval(deadman_buttons.perform(context)))
                | set(ast.literal_eval(autonomous_deadman_buttons.perform(context))))
            teleop_params['heartbeat_deadman.deadman_buttons'] = heartbeat_buttons

        joy_teleop_node = Node(
                package='joy_teleop',
                executable='joy_teleop',
                name='joy_teleop',
                parameters=[joy_teleop_config, teleop_params],
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        )
        
        return [joy_teleop_node]

    # add nodes to the launch description
    ld.add_action(joy_node)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
