"""
Todo:
    * Prepare for unified use, containerization, realsense_splitting
        https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox/blob/release-2.0.0/nvblox_examples/nvblox_examples_bringup/launch/perception/vslam.launch.py |
        https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox/blob/main/nvblox_examples/nvblox_examples_bringup/launch/perception/vslam.launch.py
    * Todo: pass topics as arguments
"""

import os
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes, SetRemap, PushRosNamespace, SetParametersFromFile, SetParameter
from launch_ros.descriptions import ParameterFile, ComposableNode
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # todo: add launch argument to launch realsense  or not
    # todo: use realsense config

    # Setup launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    launch_realsense_driver = LaunchConfiguration('launch_realsense_driver', default=True)

    # Declare launch arguments
    use_sim_time_la = DeclareLaunchArgument(
            'use_sim_time', default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true')
    launch_realsense_driver_launch_arg = DeclareLaunchArgument('launch_realsense_driver',
                                                               default_value=launch_realsense_driver,
                                                               description="Whether or not to "
                                                                           "launch the Realsense Camera.")

    # Add launch arguments to a list
    launch_args = [
        use_sim_time_la,
        launch_realsense_driver_launch_arg,
    ]

    # Run nodes
    realsense_camera_node = Node(
            condition=IfCondition(launch_realsense_driver),
            name='camera',
            namespace='camera',
            package='realsense2_camera',
            executable='realsense2_camera_node',
            parameters=[{
                'enable_infra1': True,
                'enable_infra2': True,
                'enable_color': False,
                'enable_depth': False,
                'depth_module.emitter_enabled': 0,
                'depth_module.depth_profile': '640x360x60',
                'depth_module.infra_profile': '640x360x60',
                'depth_module.infra_format': "RGB8",
                'depth_module.infra1_format': "Y8",
                'depth_module.infra2_format': "Y8",
                'enable_gyro': True,
                'enable_accel': True,
                'gyro_fps': 200,
                'accel_fps': 200,
                'unite_imu_method': 2,
                'publish_tf': True,
                'tf_publish_rate': 30.0
            }]
    )

    # realsense_camera_node_composed = ComposableNode(
    #         condition=IfCondition(launch_realsense_driver),
    #         # name='camera',
    #         namespace="camera",
    #         package='realsense2_camera',
    #         plugin='realsense2_camera::RealSenseNodeFactory',
    #         extra_arguments=[{'use_intra_process_comms': LaunchConfiguration("intra_process_comms")}],
    #         parameters=[config_file])

    visual_slam_node = ComposableNode(
            name='visual_slam_node',
            package='isaac_ros_visual_slam',
            plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
            parameters=[{
                'use_sim_time': use_sim_time,
                'num_cameras': 2,  # two for a single stereo camera
                'enable_image_denoising': False,
                'rectified_images': True,
                'enable_imu_fusion': True,
                'enable_planar_mode': False,  # todo: set as a launch argument
                'enable_ground_constraint_in_odometry': False,
                'enable_ground_constraint_in_slam': False,
                'enable_localization_n_mapping': True,  # True enables Odometry+SLAM, False=Odometry only. Set to False
                'gyro_noise_density': 0.000244,  # todo: get noise parameters from config file
                'gyro_random_walk': 0.000019393,
                'accel_noise_density': 0.001862,
                'accel_random_walk': 0.003,
                'calibration_frequency': 200.0,
                'image_jitter_threshold_ms': 22.00,
                'path_max_size': 200,  # Default: 1024. For visualization purposes only
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'camera_link',  # camera_link, base_link. todo: set as launch argument
                'imu_frame': 'camera_imu_optical_frame',  # todo: set as launch argument
                'publish_map_to_odom_tf': True,
                'publish_odom_to_rig_tf': True,  # publish odom->base_link tf
                'invert_map_to_odom_tf': False,
                'invert_odom_to_rig_tf': False,
                'enable_slam_visualization': True,  # todo: set as launch argument
                'enable_landmarks_view': True,  # todo: set as launch argument
                'enable_observations_view': True,  # todo: set as launch argument
                'camera_optical_frames': [
                    'camera_infra1_optical_frame',
                    'camera_infra2_optical_frame',
                ],  # leaving an empty list, e.g '[]' will take the parameters from messages
                # 'image_qos': 'DEFAULT',
                # 'imu_qos': 'DEFAULT',
            }],
            remappings=[('visual_slam/image_0', 'camera/camera/infra1/image_rect_raw'),  # /camera/realsense_splitter_node/output/infra_1
                        ('visual_slam/camera_info_0', 'camera/camera/infra1/camera_info'),
                        ('visual_slam/image_1', 'camera/camera/infra2/image_rect_raw'),  # /camera/realsense_splitter_node/output/infra_2
                        ('visual_slam/camera_info_1', 'camera/camera/infra2/camera_info'),
                        ('visual_slam/imu', 'camera/camera/imu')]
    )

    visual_slam_launch_container = ComposableNodeContainer(
            name='visual_slam_launch_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[visual_slam_node],
            output='screen'
    )

    # Add launch arguments and nodes to the launch description
    ld = launch_args + [realsense_camera_node,
                        visual_slam_launch_container]
    return ld


def generate_launch_description():
    return LaunchDescription(
            [
                SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
                OpaqueFunction(function=launch_setup)
            ]
    )
