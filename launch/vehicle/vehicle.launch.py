
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

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')
    ]

    # Create the launch configuration variables
    vesc_config = LaunchConfiguration('vesc_config')
    launch_imu_filter = LaunchConfiguration('launch_imu_filter')
    launch_ackermann_to_vesc_node = LaunchConfiguration('launch_ackermann_to_vesc_node')
    launch_vesc_to_odom_node = LaunchConfiguration('launch_vesc_to_odom_node')
    launch_throttle_interpolator_node = LaunchConfiguration('launch_throttle_interpolator_node', default='False')
    launch_twist_to_ackermann = LaunchConfiguration('launch_twist_to_ackermann', default='False')
    launch_ackermann_to_twist = LaunchConfiguration('launch_ackermann_to_twist', default='True')

    max_acceleration = LaunchConfiguration('max_acceleration', default=2.5)
    max_steering_rate = LaunchConfiguration('max_steering_rate', default=3.2)

    vesc_poll_rate = LaunchConfiguration('vesc_poll_rate', default=50.0)
    vesc_imu_poll_rate = LaunchConfiguration('vesc_imu_poll_rate', default=100.0)
    use_imu_yaw_rate = LaunchConfiguration('use_imu_yaw_rate', default='False')

    use_closed_loop_speed = LaunchConfiguration('use_closed_loop_speed', default='False')
    speed_kp = LaunchConfiguration('speed_kp', default=0.0)
    speed_ki = LaunchConfiguration('speed_ki', default=0.0)
    speed_anti_windup = LaunchConfiguration('speed_anti_windup', default=1000.0)
    use_adaptive_ff = LaunchConfiguration('use_adaptive_ff', default='False')
    adaptive_ff_alpha = LaunchConfiguration('adaptive_ff_alpha', default=0.95)
    adaptive_ff_gain_min = LaunchConfiguration('adaptive_ff_gain_min', default=2307.0)
    adaptive_ff_gain_max = LaunchConfiguration('adaptive_ff_gain_max', default=9228.0)
    vesc_max_speed = LaunchConfiguration('vesc_max_speed', default=0.0)
    vesc_max_steering_angle = LaunchConfiguration('vesc_max_steering_angle', default=0.0)
    use_accel_ff = LaunchConfiguration('use_accel_ff', default='False')
    accel_to_erpm_gain = LaunchConfiguration('accel_to_erpm_gain', default=0.0)
    use_cmd_accel_rate_limit = LaunchConfiguration('use_cmd_accel_rate_limit', default='False')

    vesc_la = DeclareLaunchArgument(
            'vesc_config',
            default_value=vesc_config_file,
            description='Descriptions for vesc configs')
    declare_launch_imu_filter = DeclareLaunchArgument(
            'launch_imu_filter',
            default_value='False',
            description='Whether to start the IMU filter node.')
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
    declare_launch_twist_to_ackermann = DeclareLaunchArgument(
            'launch_twist_to_ackermann',
            default_value='False',
            description='Start twist_to_ackermann converter (requires trajectory_following_ros2).')
    declare_launch_ackermann_to_twist = DeclareLaunchArgument(
            'launch_ackermann_to_twist',
            default_value='True',
            description='Start ackermann_to_twist converter to republish ackermann_cmd as cmd_vel for EKF use_control.')

    max_acceleration_la = DeclareLaunchArgument(
            'max_acceleration',
            default_value=max_acceleration,
            description='The maximum acceleration in m/s^2.')

    max_steering_rate_la = DeclareLaunchArgument(
            'max_steering_rate',
            default_value=max_steering_rate,
            description='The maximum steering rate in rads/s.')

    vesc_poll_rate_la = DeclareLaunchArgument(
            'vesc_poll_rate',
            default_value=vesc_poll_rate,
            description='The frequency at which to send/receive messages from/to the VESC.')

    vesc_imu_poll_rate_la = DeclareLaunchArgument(
            'vesc_imu_poll_rate',
            default_value=vesc_imu_poll_rate,
            description='The frequency at which to poll IMU data from the VESC.')

    use_imu_yaw_rate_la = DeclareLaunchArgument(
            'use_imu_yaw_rate',
            default_value='False',
            description='Use gyro-z from sensors/imu/raw for yaw integration in vesc_to_odom '
                        'instead of the kinematic model.')

    use_closed_loop_speed_la = DeclareLaunchArgument(
            'use_closed_loop_speed',
            default_value='False',
            description='Use closed-loop PID speed control in ackermann_to_vesc.')
    speed_kp_la = DeclareLaunchArgument(
            'speed_kp',
            default_value=speed_kp,
            description='Proportional gain for the closed-loop speed controller.')
    speed_ki_la = DeclareLaunchArgument(
            'speed_ki',
            default_value=speed_ki,
            description='Integral gain for the closed-loop speed controller.')
    speed_anti_windup_la = DeclareLaunchArgument(
            'speed_anti_windup',
            default_value=speed_anti_windup,
            description='Anti-windup clamp (ERPM) for the speed integrator.')
    use_adaptive_ff_la = DeclareLaunchArgument(
            'use_adaptive_ff',
            default_value='False',
            description='Enable adaptive feedforward gain in ackermann_to_vesc.')
    adaptive_ff_alpha_la = DeclareLaunchArgument(
            'adaptive_ff_alpha',
            default_value=adaptive_ff_alpha,
            description='Low-pass filter coefficient for the adaptive feedforward gain.')
    adaptive_ff_gain_min_la = DeclareLaunchArgument(
            'adaptive_ff_gain_min',
            default_value=adaptive_ff_gain_min,
            description='Minimum adaptive feedforward gain (ERPM/(m/s)).')
    adaptive_ff_gain_max_la = DeclareLaunchArgument(
            'adaptive_ff_gain_max',
            default_value=adaptive_ff_gain_max,
            description='Maximum adaptive feedforward gain (ERPM/(m/s)).')
    vesc_max_speed_la = DeclareLaunchArgument(
            'vesc_max_speed',
            default_value=vesc_max_speed,
            description='Speed clamp in ackermann_to_vesc (m/s). 0 disables the limit.')
    vesc_max_steering_angle_la = DeclareLaunchArgument(
            'vesc_max_steering_angle',
            default_value=vesc_max_steering_angle,
            description='Steering angle clamp in ackermann_to_vesc (rad). 0 disables the limit.')
    use_accel_ff_la = DeclareLaunchArgument(
            'use_accel_ff',
            default_value='False',
            description='Use acceleration feedforward in ackermann_to_vesc.')
    accel_to_erpm_gain_la = DeclareLaunchArgument(
            'accel_to_erpm_gain',
            default_value=accel_to_erpm_gain,
            description='Gain mapping acceleration (m/s^2) to ERPM for feedforward.')
    use_cmd_accel_rate_limit_la = DeclareLaunchArgument(
            'use_cmd_accel_rate_limit',
            default_value='False',
            description='Rate-limit acceleration commands before sending to the VESC.')

    ld = LaunchDescription([vesc_la,
                            declare_launch_imu_filter,
                            declare_launch_ackermann_to_vesc_node,
                            declare_launch_vesc_to_odom_node,
                            declare_launch_throttle_interpolator_node,
                            declare_launch_twist_to_ackermann,
                            declare_launch_ackermann_to_twist,
                            max_acceleration_la, max_steering_rate_la,
                            vesc_poll_rate_la,
                            vesc_imu_poll_rate_la,
                            use_imu_yaw_rate_la,
                            use_closed_loop_speed_la, speed_kp_la, speed_ki_la, speed_anti_windup_la,
                            use_adaptive_ff_la, adaptive_ff_alpha_la, adaptive_ff_gain_min_la, adaptive_ff_gain_max_la,
                            vesc_max_speed_la, vesc_max_steering_angle_la,
                            use_accel_ff_la, accel_to_erpm_gain_la, use_cmd_accel_rate_limit_la])

    ackermann_to_vesc_node = GroupAction(
            condition=IfCondition(launch_ackermann_to_vesc_node),
            actions=[
                Node(
                        condition=UnlessCondition(launch_throttle_interpolator_node),
                        package='vesc_ackermann',
                        executable='ackermann_to_vesc_node',
                        name='ackermann_to_vesc_node',
                        namespace='vehicle',
                        parameters=[
                            vesc_config,
                            {
                                'use_closed_loop_speed': use_closed_loop_speed,
                                'speed_kp': speed_kp,
                                'speed_ki': speed_ki,
                                'speed_anti_windup': speed_anti_windup,
                                'use_adaptive_ff': use_adaptive_ff,
                                'adaptive_ff_alpha': adaptive_ff_alpha,
                                'adaptive_ff_gain_min': adaptive_ff_gain_min,
                                'adaptive_ff_gain_max': adaptive_ff_gain_max,
                                'max_speed': vesc_max_speed,
                                'max_steering_angle': vesc_max_steering_angle,
                                'max_accel': max_acceleration,
                                'max_steering_rate': max_steering_rate,
                                'use_accel_ff': use_accel_ff,
                                'accel_to_erpm_gain': accel_to_erpm_gain,
                                'use_cmd_accel_rate_limit': use_cmd_accel_rate_limit,
                            }
                        ],
                        remappings=remappings
                ),

                Node(
                        condition=IfCondition(launch_throttle_interpolator_node),
                        package='vesc_ackermann',
                        executable='ackermann_to_vesc_node',
                        name='ackermann_to_vesc_node',
                        namespace='vehicle',
                        parameters=[
                            vesc_config,
                            {
                                'use_closed_loop_speed': use_closed_loop_speed,
                                'speed_kp': speed_kp,
                                'speed_ki': speed_ki,
                                'speed_anti_windup': speed_anti_windup,
                                'use_adaptive_ff': use_adaptive_ff,
                                'adaptive_ff_alpha': adaptive_ff_alpha,
                                'adaptive_ff_gain_min': adaptive_ff_gain_min,
                                'adaptive_ff_gain_max': adaptive_ff_gain_max,
                                'max_speed': vesc_max_speed,
                                'max_steering_angle': vesc_max_steering_angle,
                                'max_accel': max_acceleration,
                                'max_steering_rate': max_steering_rate,
                                'use_accel_ff': use_accel_ff,
                                'accel_to_erpm_gain': accel_to_erpm_gain,
                                'use_cmd_accel_rate_limit': use_cmd_accel_rate_limit,
                            }
                        ],
                        remappings=[('commands/motor/speed', 'commands/motor/unsmoothed_speed'),
                                    ('commands/servo/position', 'commands/servo/unsmoothed_position'),
                                    *remappings]
                )
            ]
    )
    vesc_to_odom_node = Node(
            condition=IfCondition(launch_vesc_to_odom_node),
            package='vesc_ackermann',
            executable='vesc_to_odom_node',
            name='vesc_to_odom_node',
            namespace='vehicle',
            parameters=[
                vesc_config,
                {
                    'max_acceleration': max_acceleration,
                    'max_servo_speed': max_steering_rate,
                    'use_imu_yaw_rate': use_imu_yaw_rate,
                }
            ],
            remappings=[  # ('/odom', '/vesc/odom'),
                ('odom', 'vesc_odom'),
                *remappings
            ]
    )
    vesc_driver_node = Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            namespace='vehicle',
            respawn=True,
            # 2 s, not 10 s. The driver aborts (SIGABRT, uncaught
            # std::system_error) whenever a serial write to the VESC returns
            # EIO, and respawn_delay is therefore the length of the dead-stick
            # window: commands keep flowing on every ROS topic while nothing
            # reaches the motor. Measured on gosling1 2026-08-05 at 10.686 s
            # mid-drive, which reads to the operator as the car stopping and
            # then spontaneously resuming under constant throttle. Nothing in
            # the safety chain notices — the mux and command_gate stay healthy
            # because the fault is downstream of them.
            respawn_delay=2.0,
            parameters=[
                vesc_config,
                {
                    'poll_rate': vesc_poll_rate,
                    'imu_poll_rate': vesc_imu_poll_rate,
                }
            ],
            remappings=remappings
    )
    throttle_interpolator_node = Node(
            condition=IfCondition(launch_throttle_interpolator_node),
            package='f1tenth_stack',
            executable='throttle_interpolator',
            name='throttle_interpolator',
            namespace='vehicle',
            output={'both': 'log'},
            parameters=[vesc_config],
            remappings=remappings
    )

    # todo: move to bringup/teleop or nav2_navigation.launch.py
    twist_to_ackermann_node = Node(
            condition=IfCondition(launch_twist_to_ackermann),
            package='trajectory_following_ros2',   # todo: put package in this repository and make parameters input.
            executable='twist_to_ackermann',
            name='twist_to_ackermann_converter',
            parameters=[
                {'wheelbase': 0.256},
                {'twist_topic': 'cmd_vel'},  # /cmd_vel or /cmd_vel_smooth
                {'ackermann_cmd_topic': 'drive'},
                {'frame_id': 'base_link'},
                {'cmd_angle_instead_rotvel': False},
                # The node's own symmetric saturation, in radians. Its upstream default is 0.4,
                # which exceeds this car's real LEFT lock: with steering_angle_to_servo_offset 0.56
                # and gain -1.4, servo = -1.4*0.4 + 0.56 = 0.0, below the 0.08 minimum, so the VESC
                # clips and logs "below minimum limit". 0.25 < 0.257 rad keeps both locks inside the
                # servo range. This is NOT the `max_steering` launch arg — that only scales the
                # joystick and never reaches this node.
                {'max_steering_angle': 0.25},
            ],
            remappings=[('ackermann_cmd_out', 'ackermann_drive'),
                        ('ackermann_cmd', 'vehicle/ackermann_cmd'),
                        *remappings]
    )

    # Echo of the command the VESC actually received, as a Twist, for robot_localization's
    # use_control input. It must NOT publish onto `cmd_vel`:
    #   * `cmd_vel` is the *inbound command* topic (Nav2's velocity_smoother output, and the topic
    #     twist_to_ackermann subscribes to on its way to `drive`). Putting an outbound echo there
    #     gave one topic two opposite meanings and a mixed Twist/TwistStamped type list.
    #   * With both as plain Twist on `cmd_vel`, enabling twist_to_ackermann closes a feedback loop:
    #     vehicle/ackermann_cmd -> cmd_vel -> drive -> mux -> command_gate -> vehicle/ackermann_cmd.
    #     Until 2026-08-04 only the TwistStamped/Twist type mismatch prevented that.
    # Plain Twist (not TwistStamped) because ROS 2 robot_localization only accepts unstamped control
    # input — see the stamped_control note in config/localization/ekf_odom.yaml.
    ackermann_to_twist_node = Node(
            condition=IfCondition(launch_ackermann_to_twist),
            package='f1tenth_stack',
            executable='ackermann_to_twist',
            name='ackermann_to_twist',
            parameters=[{
                'ackermann_topic': 'vehicle/ackermann_cmd',
                'frame_id': 'base_link',
                'override_header': False,
                'twist_topic': 'vehicle/cmd_vel_executed',
                'wheelbase': 0.256,
                'use_stamped_subscriber': True,
                'use_stamped_publisher': False
            }]
    )

    imu_filter_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [f1tenth_launch_dir, 'launch/filters', 'imu_filter.launch.py']
            )),
            condition=IfCondition([launch_imu_filter]),
            launch_arguments={
                'input_topic': 'vehicle/sensors/imu/raw',
                'output_topic': 'vehicle/sensors/imu/data',
                'mag_topic': 'vehicle/sensors/imu/mag',  # sensor_msgs/MagneticField from VESC 6 75 MkII
                'use_mag': 'False',  # set True once magnetometer topic is verified in live data
                'remove_gravity_vector': 'False',
                'imu_gyro_stddev': '0.07',
                'imu_accel_stddev': '0.07',
                'imu_orientation_stddev': '0.032',
                'node_name': 'vesc_imu_filter',
                'imu_corrector_output_topic': 'vehicle/sensors/imu/bias_removed',
                'use_madgwick_filter': 'True',
                # Stays 'False' even after the RealSense chain is enabled. This is
                # spec step 4, deliberately not bundled with step 2 -- their effects
                # on odometry/local are not separable after the fact, and the VESC
                # bias constant is still dated 2026-08-08 and needs re-measuring.
                # Only imu0's yaw *rate* row is in scope if this is ever enabled;
                # the VESC's orientation is computed onboard and no external offset
                # can touch it. See docs/imu_bias_removal_spec.md.
                'remove_imu_bias': 'False',
                'imu_corrector_frame': 'imu_link',  # imu_link, sensor_kit_link, base_link
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
    ld.add_action(ackermann_to_twist_node)
    ld.add_action(imu_filter_node)

    return ld
