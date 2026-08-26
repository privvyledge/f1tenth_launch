"""
Gyro bias removal followed by orientation filtering.

The bias remover is imu_processors::ImuBiasRemover from ros-perception's
imu_pipeline, released for Humble at 0.5.2 and installed from apt
(ros-humble-imu-pipeline). It replaced Autoware's imu_corrector, which was
never installed in any robot image and was switched off at every call site.

The chain, when remove_imu_bias is True:

    <input_topic> -> imu_bias_remover -> <imu_corrector_output_topic> -> madgwick -> <output_topic>

imu_filter_madgwick is a pass-through for angular_velocity -- it writes only
orientation (and linear_acceleration under remove_gravity_vector) -- so the
bias remover is the only thing in this chain that touches the gyro rate.
See docs/imu_bias_removal_spec.md for the measured constants and the method.
"""

import os
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, SetRemap, PushRosNamespace, SetParametersFromFile, SetParameter
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    f1tenth_launch_pkg_prefix = get_package_share_directory('f1tenth_launch')
    imu_filter_param_file = os.path.join(
            f1tenth_launch_pkg_prefix, "config/filters/imu_filter.yaml")
    imu_corrector_param_file_path = os.path.join(
            f1tenth_launch_pkg_prefix, "config/filters/imu_bias_remover.yaml")

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    imu_corrector_params_file = LaunchConfiguration('imu_corrector_params_file')

    # The parent frame to be used in publish_tf of the Madgwick or Complementary filter.
    # Should be set to the frame_id of the raw imu message (e.g imu_link) or base_link
    imu_frame = LaunchConfiguration('imu_frame')
    imu_frame_la = DeclareLaunchArgument(
            'imu_frame',
            default_value='base_link',
            description='Frame ID for the IMU message of the Madgwick or Complementary filter.')

    imu_corrector_frame = LaunchConfiguration('imu_corrector_frame')
    imu_corrector_frame_la = DeclareLaunchArgument(
            'imu_corrector_frame',
            default_value='camera_imu_optical_frame',
            description='Frame ID for the IMU message of the IMU corrector/bias remover. E.g base_link, sensor_kit_link')

    # Whether to publish a TF transform that represents the orientation of the IMU,
    # using the frame specified in fixed_frame as the parent frame and the frame
    # given in the input imu message as the child frame. Should be used for
    # debugging and visualization purposes only.
    publish_tf = False

    # If set to true, publish transforms from imu_frame to fixed frame instead of the other way around.
    # Should be used for debugging and visualization purposes only.
    reverse_tf = False

    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    mag_topic = LaunchConfiguration('mag_topic')
    imu_corrector_output_topic = LaunchConfiguration('imu_corrector_output_topic')
    remove_gravity_vector = LaunchConfiguration('remove_gravity_vector')
    imu_gyro_stddev = LaunchConfiguration('imu_gyro_stddev')
    imu_accel_stddev = LaunchConfiguration('imu_accel_stddev')
    imu_orientation_stddev = LaunchConfiguration('imu_orientation_stddev')
    node_name = LaunchConfiguration('node_name')
    imu_corrector_node_name = LaunchConfiguration('imu_corrector_node_name')
    imu_bias_odom_topic = LaunchConfiguration('imu_bias_odom_topic')
    use_madgwick_filter = LaunchConfiguration('use_madgwick_filter')
    imu_filter_constant_dt = LaunchConfiguration('imu_filter_constant_dt')
    use_mag = LaunchConfiguration('use_mag')
    remove_imu_bias = LaunchConfiguration('remove_imu_bias')

    declare_namespace_cmd = DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
            'use_namespace',
            default_value='false',
            description='Whether to apply a namespace to the navigation stack')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true')

    declare_imu_corrector_params_file_cmd = DeclareLaunchArgument(
            'imu_corrector_params_file',
            default_value=imu_corrector_param_file_path,
            description='Full path to the ROS2 parameters file to use for all launched nodes')

    input_topic_la = DeclareLaunchArgument(
            'input_topic',
            default_value='',
            description='Raw IMU message')

    mag_topic_la = DeclareLaunchArgument(
            'mag_topic',
            default_value='',
            description='Magnetometer topic (sensor_msgs/MagneticField). '
                        'Set use_mag:=True and provide this topic to enable magnetometer-aided yaw. '
                        'VESC 6 75 MkII publishes magnetometer data; the firmware already fuses it '
                        'internally, so host-side mag fusion is optional tuning.')

    use_mag_la = DeclareLaunchArgument(
            'use_mag',
            default_value='False',
            description='Whether to use magnetometer data for heading. Requires mag_topic to be set.')

    imu_corrector_output_topic_la = DeclareLaunchArgument(
            'imu_corrector_output_topic',
            default_value='imu/bias_removed',
            description='Output topic for the IMU corrector node')

    output_topic_la = DeclareLaunchArgument(
            'output_topic',
            default_value='',
            description='Raw IMU message')

    remove_gravity_vector_la = DeclareLaunchArgument(
            'remove_gravity_vector',
            default_value='False',
            description='Whether or not to remove the gravity vector. Only valid for the Madgwick Filter')

    imu_gyro_stddev_la = DeclareLaunchArgument(
            'imu_gyro_stddev',
            default_value='0.1',
            description='Standard deviation of the gyroscope noise. This will be squared to get the (co)variance.')

    imu_accel_stddev_la = DeclareLaunchArgument(
            'imu_accel_stddev',
            default_value='0.1',
            description='Standard deviation of the accelerometer noise. This will be squared to get the (co)variance.')

    imu_orientation_stddev_la = DeclareLaunchArgument(
            'imu_orientation_stddev',
            default_value='0.1',
            description='Standard deviation of the orientation noise. This will be squared to get the (co)variance.')

    node_name_la = DeclareLaunchArgument(
            'node_name',
            default_value='imu_filter',
            description='Whether or not to remove the gravity vector.')
    imu_corrector_node_name_la = DeclareLaunchArgument(
            'imu_corrector_node_name',
            default_value='imu_bias_removal_node',
            description='Name for the IMU bias corrector/removal node.')
    imu_bias_odom_topic_la = DeclareLaunchArgument(
            'imu_bias_odom_topic',
            default_value='vehicle/vesc_odom',
            description='Velocity source telling the bias remover when the robot is '
                        'stationary. Must be independent of the IMU being corrected and '
                        'must not fall silent while the vehicle is driven -- see '
                        'config/filters/imu_bias_remover.yaml. Do not point this at '
                        'cmd_vel (silent under teleop) or odometry/local (EKF output, '
                        'closes a loop with the IMU being corrected).')
    imu_filter_constant_dt_la = DeclareLaunchArgument(
            'imu_filter_constant_dt',
            default_value='0.0',
            description='Fixed integration timestep for the orientation filter, in seconds. '
                        '0.0 means derive dt from the message header stamps -- which is only '
                        'safe if the driver stamps every sample sanely. The RealSense driver '
                        'does not: its first 2-3 IMU samples carry header times seconds away '
                        'from the rest (measured +27, +1252 then -1153, +33 s), and one such dt '
                        'swings the filter attitude 63-167 deg in a single update while the raw '
                        'gyro reports under 0.003 deg of rotation. Set this to the nominal '
                        'sample period of the IMU on THIS chain (the RealSense runs ~200 Hz, '
                        'the VESC ~100 Hz) to make the filter immune to that. '
                        'See scripts/live_runs/BUG244_CLOSEOUT_20260826.md.')
    use_madgwick_filter_la = DeclareLaunchArgument(
            'use_madgwick_filter',
            default_value='True',
            description='Whether or not to use the Madgwick Filter. Uses the complementary filter if False.')
    remove_imu_bias_la = DeclareLaunchArgument(
            'remove_imu_bias',
            default_value='True',
            description='Whether or not to remove constant additive noise from the measurement.')

    ld = LaunchDescription([declare_namespace_cmd, declare_use_namespace_cmd, declare_use_sim_time_cmd,
                            declare_imu_corrector_params_file_cmd,
                            imu_frame_la, imu_corrector_frame_la, input_topic_la, mag_topic_la,
                            output_topic_la, imu_corrector_output_topic_la, imu_corrector_node_name_la,
                            imu_bias_odom_topic_la,
                            remove_gravity_vector_la,
                            imu_gyro_stddev_la, imu_accel_stddev_la, imu_orientation_stddev_la,
                            node_name_la, use_madgwick_filter_la, use_mag_la, remove_imu_bias_la,
                            imu_filter_constant_dt_la])

    imu_filter_with_correction_node = GroupAction(
            condition=IfCondition(remove_imu_bias),
            actions=[
                PushRosNamespace(
                        condition=IfCondition(use_namespace),
                        namespace=namespace
                ),
                SetParameter(name='use_sim_time', value=use_sim_time),
                # Only orientation_stddev survives here. base_link,
                # angular_velocity_stddev_* and acceleration_stddev were
                # imu_corrector parameters; neither imu_bias_remover nor
                # imu_filter_madgwick declares them, so ROS 2 would drop them
                # silently and the file would read as though they applied.
                SetParameter(name='orientation_stddev', value=imu_orientation_stddev),

                # SetParametersFromFile(imu_filter_param_file),
                SetRemap(src='imu/data_raw', dst=imu_corrector_output_topic),
                SetRemap(src='imu/data', dst=output_topic),
                SetRemap(
                        src='imu/mag',
                        dst=mag_topic,
                        condition=LaunchConfigurationNotEquals('mag_topic', '')
                ),
                SetRemap(src=['/tf'], dst=['tf']),
                SetRemap(src=['/tf_static'], dst=['tf_static']),

                Node(
                        package='imu_processors',
                        executable='imu_bias_remover_node',
                        name=imu_corrector_node_name,
                        # namespace=namespace,
                        output={'both': 'log'},
                        parameters=[imu_corrector_params_file],
                        remappings=[
                            ('imu', input_topic),
                            ('imu_biased', imu_corrector_output_topic),
                            # Stationarity source. `bias` is left unremapped on
                            # purpose: it is the only way to check the estimate
                            # converges to the measured constant, since a parked
                            # test reads ~0 either way.
                            ('odom', imu_bias_odom_topic),
                        ]
                ),

                Node(
                        condition=IfCondition([use_madgwick_filter]),
                        package='imu_filter_madgwick',
                        executable='imu_filter_madgwick_node',
                        name=node_name,
                        # namespace=namespace,
                        output='screen',
                        # do_bias_estimation, do_adaptive_gain, gain_acc and
                        # gain_mag are imu_complementary_filter parameters and
                        # are NOT declared by this node. Passing them here made
                        # the file read as though gyro bias estimation was on
                        # while ROS 2 dropped them as undeclared overrides.
                        # Madgwick's own drift term is `zeta`, and it only
                        # steers the internal orientation estimate -- the
                        # published angular_velocity is a pass-through either
                        # way. Bias removal happens upstream of this node.
                        parameters=[
                            {'use_mag': use_mag},
                            {'gain': 0.3},
                            # 0.0 would take dt from the header stamps; the
                            # RealSense driver's first samples are unusable.
                            {'constant_dt': imu_filter_constant_dt},
                            {'fixed_frame': imu_frame},
                            {'world_frame': "enu"},
                            {'remove_gravity_vector': remove_gravity_vector},
                            {'publish_tf': publish_tf},
                            {'reverse_tf': reverse_tf},
                        ],  # todo: use parameter file instead
                        # parameters=[imu_filter_param_file],
                        # remappings=[
                        #     ('imu/data_raw', input_topic),  # input topic: vehicle/sensors/imu/raw
                        #     ('imu/data', output_topic),  # output topic: vehicle/sensors/imu/data
                        # ]
                ),
                Node(
                        condition=IfCondition(PythonExpression(['not ', use_madgwick_filter])),
                        package='imu_complementary_filter',
                        executable='complementary_filter_node',
                        name=node_name,
                        # namespace=namespace,
                        output='screen',
                        parameters=[
                            {'do_bias_estimation': True},
                            {'do_adaptive_gain': True},
                            {'use_mag': use_mag},
                            {'gain': 0.3},
                            {'gain_acc': 0.01},
                            {'gain_mag': 0.01},
                            {'fixed_frame': imu_frame},
                            {'world_frame': "enu"},
                            {'remove_gravity_vector': remove_gravity_vector},
                            {'publish_tf': publish_tf},
                            {'reverse_tf': reverse_tf},
                        ],  # todo: use parameter file instead
                        # parameters=[imu_filter_param_file],
                        # remappings=[
                        #     ('imu/data_raw', input_topic),  # input topic: vehicle/sensors/imu/raw
                        #     ('imu/data', output_topic),  # output topic: vehicle/sensors/imu/data
                        # ]
                )
            ]
    )

    imu_filter_without_correction_node = GroupAction(
            condition=UnlessCondition(remove_imu_bias),
            actions=[
                PushRosNamespace(
                        condition=IfCondition(use_namespace),
                        namespace=namespace
                ),
                SetParameter(name='use_sim_time', value=use_sim_time),
                SetParameter(name='orientation_stddev', value=imu_orientation_stddev),

                # SetParametersFromFile(imu_filter_param_file),
                SetRemap(src='imu/data_raw', dst=input_topic),
                SetRemap(src='imu/data', dst=output_topic),
                SetRemap(
                        src='imu/mag',
                        dst=mag_topic,
                        condition=LaunchConfigurationNotEquals('mag_topic', '')
                ),
                SetRemap(src=['/tf'], dst=['tf']),
                SetRemap(src=['/tf_static'], dst=['tf_static']),
                Node(
                        condition=IfCondition([use_madgwick_filter]),
                        package='imu_filter_madgwick',
                        executable='imu_filter_madgwick_node',
                        name=node_name,
                        # namespace=namespace,
                        output='screen',
                        # do_bias_estimation, do_adaptive_gain, gain_acc and
                        # gain_mag are imu_complementary_filter parameters and
                        # are NOT declared by this node. Passing them here made
                        # the file read as though gyro bias estimation was on
                        # while ROS 2 dropped them as undeclared overrides.
                        # Madgwick's own drift term is `zeta`, and it only
                        # steers the internal orientation estimate -- the
                        # published angular_velocity is a pass-through either
                        # way. Bias removal happens upstream of this node.
                        parameters=[
                            {'use_mag': use_mag},
                            {'gain': 0.3},
                            # 0.0 would take dt from the header stamps; the
                            # RealSense driver's first samples are unusable.
                            {'constant_dt': imu_filter_constant_dt},
                            {'fixed_frame': imu_frame},
                            {'world_frame': "enu"},
                            {'remove_gravity_vector': remove_gravity_vector},
                            {'publish_tf': publish_tf},
                            {'reverse_tf': reverse_tf},
                        ],  # todo: use parameter file instead
                        # parameters=[imu_filter_param_file],
                        # remappings=[
                        #     ('imu/data_raw', input_topic),  # input topic: vehicle/sensors/imu/raw
                        #     ('imu/data', output_topic),  # output topic: vehicle/sensors/imu/data
                        # ]
                ),
                Node(
                        condition=IfCondition(PythonExpression(['not ', use_madgwick_filter])),
                        package='imu_complementary_filter',
                        executable='complementary_filter_node',
                        name=node_name,
                        # namespace=namespace,
                        output='screen',
                        parameters=[
                            {'do_bias_estimation': True},
                            {'do_adaptive_gain': True},
                            {'use_mag': use_mag},
                            {'gain': 0.3},
                            {'gain_acc': 0.01},
                            {'gain_mag': 0.01},
                            {'fixed_frame': imu_frame},
                            {'world_frame': "enu"},
                            {'remove_gravity_vector': remove_gravity_vector},
                            {'publish_tf': publish_tf},
                            {'reverse_tf': reverse_tf},
                        ],  # todo: use parameter file instead
                        # parameters=[imu_filter_param_file],
                        # remappings=[
                        #     ('imu/data_raw', input_topic),  # input topic: vehicle/sensors/imu/raw
                        #     ('imu/data', output_topic),  # output topic: vehicle/sensors/imu/data
                        # ]
                )
            ]
    )

    ld.add_action(imu_filter_with_correction_node)
    ld.add_action(imu_filter_without_correction_node)

    return ld
