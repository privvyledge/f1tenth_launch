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
    vesc_config_file = os.path.join(
            f1tenth_launch_dir,
            'config/vehicle',
            'vesc.yaml'
    )
    mux_config_file = os.path.join(
            get_package_share_directory('f1tenth_stack'),
            'config',
            'mux.yaml'
    )

    # Create the launch configuration variables
    vesc_config = LaunchConfiguration('vesc_config')
    joy_config = LaunchConfiguration('joy_config')
    mux_config = LaunchConfiguration('mux_config')
    launch_joystick = LaunchConfiguration('launch_joystick')
    launch_imu_filter = LaunchConfiguration('launch_imu_filter')

    joy_la = DeclareLaunchArgument(
            'joy_config',
            default_value=joy_teleop_config_file,
            description='Descriptions for joy and joy_teleop configs')
    vesc_la = DeclareLaunchArgument(
            'vesc_config',
            default_value=vesc_config_file,
            description='Descriptions for vesc configs')
    mux_la = DeclareLaunchArgument(
            'mux_config',
            default_value=mux_config_file,
            description='Descriptions for ackermann mux configs')
    declare_launch_joystick = DeclareLaunchArgument(
            'launch_joystick',
            default_value='True',
            description='Whether to start the joystick node.')
    declare_launch_imu_filter = DeclareLaunchArgument(
            'launch_imu_filter',
            default_value='True',
            description='Whether to start the joystick node.')

    ld = LaunchDescription([joy_la, vesc_la, mux_la,
                            declare_launch_joystick, declare_launch_imu_filter])

    joy_node = Node(
            condition=IfCondition([launch_joystick]),
            package='joy',
            executable='joy_node',
            name='joy',
            parameters=[joy_config]
    )
    joy_teleop_node = Node(
            package='joy_teleop',
            executable='joy_teleop',
            name='joy_teleop',
            parameters=[joy_config]
    )
    ackermann_to_vesc_node = Node(
            package='vesc_ackermann',
            executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node',
            namespace='vehicle',
            parameters=[vesc_config]
    )
    vesc_to_odom_node = Node(
            package='vesc_ackermann',
            executable='vesc_to_odom_node',
            name='vesc_to_odom_node',
            namespace='vehicle',  # autoware
            parameters=[vesc_config],
            remappings=[  # ('/odom', '/vesc/odom'),
                ('odom', 'vesc_odom'),  # autoware
            ]
    )
    vesc_driver_node = Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            namespace='vehicle',  # autoware
            parameters=[vesc_config]
    )
    throttle_interpolator_node = Node(
            package='f1tenth_stack',
            executable='throttle_interpolator',
            name='throttle_interpolator',
            parameters=[vesc_config]
    )
    ackermann_mux_node = Node(
            package='ackermann_mux',
            executable='ackermann_mux',
            name='ackermann_mux',
            parameters=[mux_config],
            remappings=[('ackermann_cmd_out', 'ackermann_drive'),
                        ('ackermann_cmd', '/vehicle/ackermann_cmd')]
    )

    twist_to_ackermann_node = Node(
            package='trajectory_following_ros2',   # todo: put package in this repository and make parameters input.
            executable='twist_to_ackermann',
            name='twist_to_ackermann_converter',
            parameters=[
                {'wheelbase': 0.256},
                {'twist_topic': '/cmd_vel'},  # /cmd_vel or /cmd_vel_smooth
                {'ackermann_cmd_topic': '/drive'},
                {'frame_id': 'base_link'},
                {'cmd_angle_instead_rotvel': False},
            ],
            remappings=[('ackermann_cmd_out', 'ackermann_drive'),
                        ('ackermann_cmd', '/vehicle/ackermann_cmd')]
    )

    imu_filter_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [f1tenth_launch_dir, 'launch/filters', 'imu_filter.launch.py']
            )),
            condition=IfCondition([launch_imu_filter]),
            launch_arguments={
                'input_topic': '/vehicle/sensors/imu/raw',
                'output_topic': '/vehicle/sensors/imu/data',
                'remove_gravity_vector': 'False',
                'node_name': 'vesc_imu_filter'
            }.items()
    )

    static_transform_publisher_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [f1tenth_launch_dir, 'launch/vehicle', 'static_transformations.launch.py']
            ))
    )

    # add nodes to the launch description
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    # ld.add_action(throttle_interpolator_node)
    ld.add_action(ackermann_mux_node)
    ld.add_action(twist_to_ackermann_node)
    ld.add_action(imu_filter_node)
    ld.add_action(static_transform_publisher_node)

    return ld
