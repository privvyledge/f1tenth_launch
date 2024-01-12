import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    f1tenth_launch_pkg_prefix = get_package_share_directory('f1tenth_launch')
    imu_filter_param_file = os.path.join(
            f1tenth_launch_pkg_prefix, "config/imu_filter.yaml")

    # The parent frame to be used in publish_tf.
    # Should be set to the frame_id of the raw imu message (e.g imu_link) or base_link
    imu_frame = "imu_link"

    # Whether to publish a TF transform that represents the orientation of the IMU,
    # using the frame specified in fixed_frame as the parent frame and the frame
    # given in the input imu message as the child frame. Should be used for
    # debugging and visualization purposes only.
    publish_tf = False

    # If set to true, publish transforms from imu_frame to fixed frame instead of the other way around.
    # Should be used for debugging and visualization purposes only.
    reverse_tf = False
    return LaunchDescription(
        [
            Node(
                package='imu_complementary_filter',
                executable='complementary_filter_node',
                name='imu_filter',
                output='screen',
                parameters=[
                    {'do_bias_estimation': True},
                    {'do_adaptive_gain': True},
                    {'use_mag': False},
                    {'gain_acc': 0.01},
                    {'gain_mag': 0.01},
                    # {'fixed_frame': imu_frame},
                    {'world_frame': "enu"},
                    # {'remove_gravity_vector': "false"},
                    {'publish_tf': publish_tf},
                    {'reverse_tf': reverse_tf},
                ],
                # parameters=[imu_filter_param_file],
                remappings=[
                    ('/imu/data_raw', '/vehicle/sensors/imu/raw'),  # input topic: /vehicle/sensors/imu/raw
                    ('/imu/data', '/vehicle/sensors/imu/data'),  # output topic: /vehicle/sensors/imu/data
                ]
            ),

    ]
    )