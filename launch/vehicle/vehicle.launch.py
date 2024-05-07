"""
Todo: move joy launching (with mux and teleop_twist/ackermann) to another launch file.
"""
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, SetRemap
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    f1tenth_launch_dir = get_package_share_directory('f1tenth_launch')

    vesc_config_file = os.path.join(
            f1tenth_launch_dir,
            'config/vehicle',
            'vesc.yaml'
    )

    # Create the launch configuration variables
    vesc_config = LaunchConfiguration('vesc_config')
    launch_imu_filter = LaunchConfiguration('launch_imu_filter')
    launch_ackermann_to_vesc_node = LaunchConfiguration('launch_ackermann_to_vesc_node')
    launch_vesc_to_odom_node = LaunchConfiguration('launch_vesc_to_odom_node')
    launch_throttle_interpolator_node = LaunchConfiguration('launch_throttle_interpolator_node', default='False')

    max_acceleration = LaunchConfiguration('max_acceleration', default=2.5)
    max_steering_rate = LaunchConfiguration('max_servo_rate', default=3.2)

    vesc_la = DeclareLaunchArgument(
            'vesc_config',
            default_value=vesc_config_file,
            description='Descriptions for vesc configs')
    declare_launch_imu_filter = DeclareLaunchArgument(
            'launch_imu_filter',
            default_value='True',
            description='Whether to start the joystick node.')
    declare_launch_ackermann_to_vesc_node = DeclareLaunchArgument(
            'launch_ackermann_to_vesc_node',
            default_value='True',
            description='Send ackermann commands to the VESC.')
    declare_launch_vesc_to_odom_node = DeclareLaunchArgument(
            'launch_vesc_to_odom_node',
            default_value='True',
            description='Publish odometry messages from the VESC.')
    declare_launch_throttle_interpolator_node = DeclareLaunchArgument(
            'launch_throttle_interpolator_node',
            default_value=launch_throttle_interpolator_node,
            description='Interpolate commands before sending to the VESC. '
                        'Set to False if using MPC, True otherwise')

    max_acceleration_la = DeclareLaunchArgument(
            'max_acceleration',
            default_value=max_acceleration,
            description='The maximum acceleration in m/s^2.')

    max_steering_rate_la = DeclareLaunchArgument(
            'max_steering_rate',
            default_value=max_steering_rate,
            description='The maximum steering rate in rads/s.')

    ld = LaunchDescription([vesc_la,
                            declare_launch_imu_filter,
                            declare_launch_ackermann_to_vesc_node,
                            declare_launch_vesc_to_odom_node,
                            declare_launch_throttle_interpolator_node,
                            max_acceleration_la, max_steering_rate_la])

    # remap ackermann to vesc topic if the throttle interpolator is on
    # raw_speed_input_topic = 'commands/motor/speed'
    # raw_servo_input_topic = 'commands/servo/position'
    # raw_speed_output_topic = 'commands/motor/speed'  # when using actuation interpolation
    # raw_servo_output_topic = 'commands/servo/position'  # when using actuation interpolation
    #
    # launch_context = LaunchContext()
    # throttle_interpolator_string = launch_throttle_interpolator_node.perform(launch_context)
    #
    # if throttle_interpolator_string.lower() == 'true':
    #     raw_speed_output_topic = 'commands/motor/unsmoothed_speed'
    #     raw_servo_output_topic = 'commands/servo/unsmoothed_position'

    # ackermann_to_vesc_node = Node(
    #         condition=IfCondition(launch_ackermann_to_vesc_node),
    #         package='vesc_ackermann',
    #         executable='ackermann_to_vesc_node',
    #         name='ackermann_to_vesc_node',
    #         namespace='vehicle',
    #         parameters=[vesc_config],
    #         remappings=[(raw_speed_input_topic, raw_speed_output_topic),
    #                     (raw_servo_input_topic, raw_servo_output_topic)]
    # )

    ackermann_to_vesc_node = GroupAction(
            condition=IfCondition(launch_ackermann_to_vesc_node),
            actions=[
                Node(
                        condition=UnlessCondition(launch_throttle_interpolator_node),
                        package='vesc_ackermann',
                        executable='ackermann_to_vesc_node',
                        name='ackermann_to_vesc_node',
                        namespace='vehicle',
                        parameters=[vesc_config]
                ),

                Node(
                        condition=IfCondition(launch_throttle_interpolator_node),
                        package='vesc_ackermann',
                        executable='ackermann_to_vesc_node',
                        name='ackermann_to_vesc_node',
                        namespace='vehicle',
                        parameters=[vesc_config],
                        remappings=[('commands/motor/speed', 'commands/motor/unsmoothed_speed'),
                                    ('commands/servo/position', 'commands/servo/unsmoothed_position')]
                )
            ]
    )
    vesc_to_odom_node = Node(
            condition=IfCondition(launch_vesc_to_odom_node),
            package='vesc_ackermann',
            executable='vesc_to_odom_node',
            name='vesc_to_odom_node',
            namespace='vehicle',  # autoware
            parameters=[
                vesc_config,
                {
                    'max_acceleration': max_acceleration,
                    'max_servo_speed': max_steering_rate,
                }
            ],
            remappings=[  # ('/odom', '/vesc/odom'),
                ('odom', 'vesc_odom'),  # autoware
            ]
    )
    vesc_driver_node = Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            namespace='vehicle',  # autoware
            respawn=True,
            respawn_delay=10.0,
            parameters=[vesc_config]
    )
    throttle_interpolator_node = Node(
            condition=IfCondition(launch_throttle_interpolator_node),
            package='f1tenth_stack',
            executable='throttle_interpolator',
            name='throttle_interpolator',
            namespace='vehicle',
            output={'both': 'log'},
            parameters=[vesc_config]
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
                'node_name': 'vesc_imu_filter',
                'imu_corrector_output_topic': '/vehicle/sensors/imu/bias_removed',
                'use_madgwick_filter': 'False',
                'remove_imu_bias': 'True',
                'imu_corrector_frame': 'sensor_kit_link',  # camera_imu_optical_frame, sensor_kit_link, base_link
                'imu_corrector_node_name': 'vesc_imu_bias_removal_node',
                'use_sim_time': 'False',
            }.items()
    )

    # add nodes to the launch description
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    ld.add_action(throttle_interpolator_node)
    ld.add_action(twist_to_ackermann_node)
    ld.add_action(imu_filter_node)

    return ld
