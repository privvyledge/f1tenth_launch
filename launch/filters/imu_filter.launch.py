"""
Todo: make the remapped names dynamic
TOdo: add Autoware's complementary filter before Madgwick or Complemetary filter
"""

import os
from launch import LaunchDescription
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
            f1tenth_launch_pkg_prefix, "config/imu_filter.yaml")
    imu_corrector_param_file_path = os.path.join(
            f1tenth_launch_pkg_prefix, "config/imu_corrector.yaml")

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    imu_corrector_params_file = LaunchConfiguration('imu_corrector_params_file')

    # The parent frame to be used in publish_tf.
    # Should be set to the frame_id of the raw imu message (e.g imu_link) or base_link
    imu_frame = LaunchConfiguration('imu_frame')
    imu_frame_la = DeclareLaunchArgument(
            'imu_frame',
            default_value='',
            description='Frame ID for the IMU message.')

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
    remove_gravity_vector = LaunchConfiguration('remove_gravity_vector')
    node_name = LaunchConfiguration('node_name')
    use_madgwick_filter = LaunchConfiguration('use_madgwick_filter')
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
            default_value='false',
            description='Use simulation (Gazebo) clock if true')

    declare_imu_corrector_params_file_cmd = DeclareLaunchArgument(
            'imu_corrector_params_file',
            default_value=imu_corrector_param_file_path,
            description='Full path to the ROS2 parameters file to use for all launched nodes')

    input_topic_la = DeclareLaunchArgument(
            'input_topic',
            default_value='',
            description='Raw IMU message')

    output_topic_la = DeclareLaunchArgument(
            'output_topic',
            default_value='',
            description='Raw IMU message')
    remove_gravity_vector_la = DeclareLaunchArgument(
            'remove_gravity_vector',
            default_value='False',
            description='Whether or not to remove the gravity vector.')
    node_name_la = DeclareLaunchArgument(
            'node_name',
            default_value='imu_filter',
            description='Whether or not to remove the gravity vector.')
    use_madgwick_filter_la = DeclareLaunchArgument(
            'use_madgwick_filter',
            default_value='False',
            description='Whether or not to use the Madgwick Filter. Uses the complementary filter if False.')
    remove_imu_bias_la = DeclareLaunchArgument(
            'remove_imu_bias',
            default_value='True',
            description='Whether or not to remove constant additive noise from the measurement.')

    ld = LaunchDescription([declare_namespace_cmd, declare_use_namespace_cmd, declare_use_sim_time_cmd,
                            declare_imu_corrector_params_file_cmd,
                            imu_frame_la, input_topic_la, output_topic_la,
                            remove_gravity_vector_la, node_name_la, use_madgwick_filter_la, remove_imu_bias_la])

    imu_filter_with_correction_node = GroupAction(
            condition=IfCondition(remove_imu_bias),
            actions=[
                PushRosNamespace(
                        condition=IfCondition(use_namespace),
                        namespace=namespace
                ),
                SetParameter(name='use_sim_time', value=use_sim_time),
                SetParameter(name='base_link', value='camera_link'),  # todo: put this in the imu_parameter file
                # SetParametersFromFile(imu_filter_param_file),
                SetRemap(src='imu/data_raw', dst="imu/bias_removed"),
                SetRemap(src='imu/data', dst=output_topic),

                IncludeLaunchDescription(
                        XMLLaunchDescriptionSource(  # or FrontendLaunchDescriptionSource
                                launch_file_path=PathJoinSubstitution([
                                    FindPackageShare("imu_corrector"), "launch", "imu_corrector.launch.xml",
                                ]),
                        ),
                        launch_arguments={
                            "input_topic": input_topic,
                            "output_topic": "imu/bias_removed",
                            "param_file": imu_corrector_params_file,
                        }.items()
                ),
                Node(
                        condition=IfCondition([use_madgwick_filter]),
                        package='imu_filter_madgwick',
                        executable='imu_filter_madgwick_node',
                        name=node_name,
                        output='screen',
                        parameters=[
                            {'do_bias_estimation': True},
                            {'do_adaptive_gain': True},
                            {'use_mag': False},
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
                        #     ('/imu/data_raw', input_topic),  # input topic: /vehicle/sensors/imu/raw
                        #     ('/imu/data', output_topic),  # output topic: /vehicle/sensors/imu/data
                        # ]
                ),
                Node(
                        condition=IfCondition(PythonExpression(['not ', use_madgwick_filter])),
                        package='imu_complementary_filter',
                        executable='complementary_filter_node',
                        name=node_name,
                        output='screen',
                        parameters=[
                            {'do_bias_estimation': True},
                            {'do_adaptive_gain': True},
                            {'use_mag': False},
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
                        #     ('/imu/data_raw', input_topic),  # input topic: /vehicle/sensors/imu/raw
                        #     ('/imu/data', output_topic),  # output topic: /vehicle/sensors/imu/data
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
                # SetParametersFromFile(imu_filter_param_file),
                SetRemap(src='imu/data_raw', dst=input_topic),
                SetRemap(src='imu/data', dst=output_topic),
                Node(
                        condition=IfCondition([use_madgwick_filter]),
                        package='imu_filter_madgwick',
                        executable='imu_filter_madgwick_node',
                        name=node_name,
                        output='screen',
                        parameters=[
                            {'do_bias_estimation': True},
                            {'do_adaptive_gain': True},
                            {'use_mag': False},
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
                        #     ('/imu/data_raw', input_topic),  # input topic: /vehicle/sensors/imu/raw
                        #     ('/imu/data', output_topic),  # output topic: /vehicle/sensors/imu/data
                        # ]
                ),
                Node(
                        condition=IfCondition(PythonExpression(['not ', use_madgwick_filter])),
                        package='imu_complementary_filter',
                        executable='complementary_filter_node',
                        name=node_name,
                        output='screen',
                        parameters=[
                            {'do_bias_estimation': True},
                            {'do_adaptive_gain': True},
                            {'use_mag': False},
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
                        #     ('/imu/data_raw', input_topic),  # input topic: /vehicle/sensors/imu/raw
                        #     ('/imu/data', output_topic),  # output topic: /vehicle/sensors/imu/data
                        # ]
                )
            ]
    )

    # imu_madgwick_filter_node = Node(
    #             condition=IfCondition([use_madgwick_filter]),
    #             package='imu_filter_madgwick',
    #             executable='imu_filter_madgwick_node',
    #             name=node_name,
    #             output='screen',
    #             parameters=[
    #                 {'do_bias_estimation': True},
    #                 {'do_adaptive_gain': True},
    #                 {'use_mag': False},
    #                 {'gain_acc': 0.01},
    #                 {'gain_mag': 0.01},
    #                 {'fixed_frame': imu_frame},
    #                 {'world_frame': "enu"},
    #                 {'remove_gravity_vector': remove_gravity_vector},
    #                 {'publish_tf': publish_tf},
    #                 {'reverse_tf': reverse_tf},
    #             ],  # todo: use parameter file instead
    #             # parameters=[imu_filter_param_file],
    #             remappings=[
    #                 ('/imu/data_raw', input_topic),  # input topic: /vehicle/sensors/imu/raw
    #                 ('/imu/data', output_topic),  # output topic: /vehicle/sensors/imu/data
    #             ]
    #         )
    #
    # imu_complementary_filter_node = Node(
    #         condition=IfCondition(PythonExpression(['not ', use_madgwick_filter])),
    #         package='imu_complementary_filter',
    #         executable='complementary_filter_node',
    #         name=node_name,
    #         output='screen',
    #         parameters=[
    #             {'do_bias_estimation': True},
    #             {'do_adaptive_gain': True},
    #             {'use_mag': False},
    #             {'gain_acc': 0.01},
    #             {'gain_mag': 0.01},
    #             {'fixed_frame': imu_frame},
    #             {'world_frame': "enu"},
    #             {'remove_gravity_vector': remove_gravity_vector},
    #             {'publish_tf': publish_tf},
    #             {'reverse_tf': reverse_tf},
    #         ],  # todo: use parameter file instead
    #         # parameters=[imu_filter_param_file],
    #         remappings=[
    #             ('/imu/data_raw', input_topic),  # input topic: /vehicle/sensors/imu/raw
    #             ('/imu/data', output_topic),  # output topic: /vehicle/sensors/imu/data
    #         ]
    # )

    ld.add_action(imu_filter_with_correction_node)
    ld.add_action(imu_filter_without_correction_node)
    # ld.add_action(imu_madgwick_filter_node)
    # ld.add_action(imu_complementary_filter_node)

    return ld