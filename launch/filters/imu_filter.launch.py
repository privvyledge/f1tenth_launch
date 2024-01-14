"""
Todo: make the remapped names dynamic
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    f1tenth_launch_pkg_prefix = get_package_share_directory('f1tenth_launch')
    imu_filter_param_file = os.path.join(
            f1tenth_launch_pkg_prefix, "config/imu_filter.yaml")

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

    ld = LaunchDescription([imu_frame_la, input_topic_la, output_topic_la, remove_gravity_vector_la, node_name_la])

    imu_filter_node = Node(
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
                ],
                # parameters=[imu_filter_param_file],
                remappings=[
                    ('/imu/data_raw', input_topic),  # input topic: /vehicle/sensors/imu/raw
                    ('/imu/data', output_topic),  # output topic: /vehicle/sensors/imu/data
                ]
            )

    ld.add_action(imu_filter_node)

    return ld