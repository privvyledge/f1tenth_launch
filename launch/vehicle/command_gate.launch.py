# todo: create a separate container for the VESCs.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    use_composition = LaunchConfiguration('use_composition').perform(context).lower() == 'true'
    config = LaunchConfiguration('command_gate_config').perform(context)
    require_heartbeat = LaunchConfiguration('command_gate_require_heartbeat').perform(context).lower() == 'true'
    require_enable = LaunchConfiguration('command_gate_require_enable').perform(context).lower() == 'true'

    params = [config, {'require_heartbeat': require_heartbeat, 'require_enable': require_enable}]

    if use_composition:
        # Use component_container (single-threaded executor per component) — CommandGateNode
        # is not thread-safe and must not be loaded into a multi-threaded executor container.
        return [
            ComposableNodeContainer(
                name='command_gate_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='command_gate',
                        plugin='command_gate::CommandGateNode',
                        name='command_gate',
                        parameters=params,
                    )
                ],
                output='screen',
            )
        ]

    return [
        Node(
            package='command_gate',
            executable='command_gate_node',
            name='command_gate',
            parameters=params,
            output='screen',
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_launch')
    default_config = os.path.join(pkg_share, 'config', 'vehicle', 'command_gate.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'command_gate_config',
            default_value=default_config,
            description='Path to command_gate parameter YAML'),
        DeclareLaunchArgument(
            'use_composition',
            default_value='True',
            description='Load as ComposableNode in a dedicated single-threaded container'),
        DeclareLaunchArgument(
            'command_gate_require_heartbeat',
            default_value='False',
            description='Close gate on heartbeat timeout; set True and publish ~/heartbeat to activate watchdog'),
        DeclareLaunchArgument(
            'command_gate_require_enable',
            default_value='False',
            description='Gate starts closed; open with /command_gate/set_enabled std_srvs/SetBool {data: true}'),
        OpaqueFunction(function=launch_setup),
    ])