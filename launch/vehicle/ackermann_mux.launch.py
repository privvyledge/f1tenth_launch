from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    mux_config = LaunchConfiguration('mux_config')
    use_f1tenth_namespace = LaunchConfiguration('use_f1tenth_namespace')
    f1tenth_namespace = LaunchConfiguration('f1tenth_namespace')

    use_f1tenth_namespace_string = use_f1tenth_namespace.perform(context)
    f1tenth_namespace_string = f1tenth_namespace.perform(context)

    # ros2 topic pub runs outside any ROS namespace context, so PushRosNamespace does
    # not apply to it. When namespacing is active we must publish to the absolute
    # namespaced topic to match what ackermann_mux subscribes to.
    if use_f1tenth_namespace_string.lower() == 'true' and f1tenth_namespace_string:
        safety_topic = f'/{f1tenth_namespace_string}/safety'
    else:
        safety_topic = 'safety'

    ackermann_mux_node = Node(
            package='ackermann_mux',
            executable='ackermann_mux',
            name='ackermann_mux',
            parameters=[mux_config],
            remappings=[('ackermann_cmd_out', 'ackermann_drive'),
                        ('ackermann_cmd', 'vehicle/ackermann_cmd'),
                        ('/tf', 'tf'),
                        ('/tf_static', 'tf_static')]
    )

    # Persistent zero-speed publisher on the safety topic (priority 1 in mux.yaml).
    # Acts as a safe fallback: if joystick BT disconnects and navigation is inactive,
    # the mux falls through to this topic and commands zero speed, stopping the car.
    # Publishes at 40 Hz so the mux's 50ms timeout is never triggered.
    safety_zero_speed_publisher = ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '--rate', '40',
                safety_topic,
                'ackermann_msgs/msg/AckermannDriveStamped',
                '{drive: {speed: 0.0, steering_angle: 0.0, steering_angle_velocity: 0.0, acceleration: 0.0, jerk: 0.0}}'
            ],
            output='log'
    )

    return [ackermann_mux_node, safety_zero_speed_publisher]


def generate_launch_description():
    mux_config_file = os.path.join(
            get_package_share_directory('f1tenth_launch'),
            'config', 'vehicle',
            'mux.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
                'mux_config',
                default_value=mux_config_file,
                description='Descriptions for ackermann mux configs'),
        DeclareLaunchArgument(
                'use_f1tenth_namespace',
                default_value='False',
                description='Whether a namespace is active (determines safety topic prefix).'),
        DeclareLaunchArgument(
                'f1tenth_namespace',
                default_value='',
                description='Namespace prefix for the safety topic when namespacing is active.'),
        OpaqueFunction(function=launch_setup)
    ])