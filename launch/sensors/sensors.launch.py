#!/usr/bin/env python3
"""
Todo: launch realsense URDF/Xacro with robot state publisher using _d435i.urdf.xacro or test_d435i_camera.urdf.xacro (https://navigation.ros.org/setup_guides/urdf/setup_urdf.html  | https://github.com/IntelRealSense/realsense-ros/blob/ros2-development/realsense2_description/launch/view_model.launch.py)
Todo: switch to XML launch to simplify.

Steps:
    * include lidar launch file and pass launch filter argument bool to lidar launch file
    * include camera_depth launch file and pass arguments for camera_depth filter,  IMU and IMU filter
    * (optional) include depth_image_to_laserscan launch file and pass argument to filter laserscan
    * (optional) publish lidar odometry
    * (optional) publish visual odometry
"""
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LifecycleNode
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import LogInfo
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    f1tenth_launch_dir = get_package_share_directory('f1tenth_launch')
    # Parameter files
    lidar_config = os.path.join(
            get_package_share_directory('f1tenth_launch'),
            'config/sensors',
            'ydlidar_X4.yaml')

    depth_config = os.path.join(
            get_package_share_directory('f1tenth_launch'),
            'config/sensors',
            'realsense_config.yaml')
    realsense_imu_config = os.path.join(
            get_package_share_directory('f1tenth_launch'),
            'config/sensors',
            'realsense_imu_config.yaml')

    # depth_launch_path = PathJoinSubstitution(
    #         [FindPackageShare('f1tenth_launch'), 'launch', 'depth_image.launch.py']
    # )

    depth_sensor_name = 'realsense'
    use_sim_time = LaunchConfiguration('use_sim_time')
    approx_sync = LaunchConfiguration('approx_sync')
    stereo_to_pointcloud = LaunchConfiguration('stereo_to_pointcloud')
    depthimage_to_pointcloud = LaunchConfiguration('depthimage_to_pointcloud')

    # Launch Arguments
    use_sim_time_la = DeclareLaunchArgument(
            'use_sim_time', default_value='False',
            description='Use simulation (Gazebo) clock if true')
    approx_sync_la = DeclareLaunchArgument(
            'approx_sync', default_value='True',
            description='Synchronize topics')
    lidar_la = DeclareLaunchArgument('lidar_config',
                                     default_value=lidar_config,
                                     description='Path to the YDLIDAR parameters file to use.')
    depth_la = DeclareLaunchArgument('depth_config',
                                     default_value=depth_config,
                                     description='Path to the Realsense parameters file to use.')
    realsense_imu_la = DeclareLaunchArgument('realsense_imu_config',
                                             default_value=realsense_imu_config,
                                             description='Path to the Realsense IMU parameters file to use.')
    stereo_to_pointcloud_la = DeclareLaunchArgument('stereo_to_pointcloud',
                                                    default_value='False',
                                                    description='Whether to publish a PointCloud2 message from stereo images.')
    depthimage_to_pointcloud_la = DeclareLaunchArgument('depthimage_to_pointcloud',
                                                        default_value='False',
                                                        description='Whether to publish a PointCloud2 message from a depth image.')

    # Create Launch Description
    ld = LaunchDescription([use_sim_time_la, approx_sync_la, lidar_la, depth_la, stereo_to_pointcloud_la, depthimage_to_pointcloud_la])

    # Nodes
    lidar_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [f1tenth_launch_dir, 'launch/sensors', 'ydlidar.launch.py']
            ))
    )

    # #################### Begin LaserScan (or PointCloud) to Odometry
    rtabmap_icp_odometry = Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='rtabmap_icp',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': False,
                'wait_for_transform': 0.1,
                'wait_imu_to_init ': True,  # use if imu is passed
                'queue_size': 1,

            }],
            remappings=[('scan', '/lidar/scan_filtered'),
                        ('imu', '/vehicle/sensors/imu/raw'),  # imu must have orientation
                        ('odom', '/odom/rtabmap_icp'),
                        ('odom_last_frame', '/rtabmap_icp/points'),  # 'odom_last_frame ', 'odom_filtered_input_scan'
                        ]
    )

    rf2o_odometry_node = Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            # name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/lidar/scan_filtered',
                'odom_topic': '/odom/rf2o',
                'publish_tf': False,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 10.0}],
    )

    laser_scan_matcher_node = Node(
            package='ros2_laser_scan_matcher',
            executable='laser_scan_matcher',
            name='laser_scan_matcher_node',
            output='screen',
            parameters=[{
                'publish_odom': '/odom/laser_scan_matcher',
                'publish_tf': False,
                'laser_frame': 'lidar',
                'base_frame': 'base_link',
                'odom_frame': 'odom_laser_scan_matcher',
                'map_frame': 'map',
                'init_pose_from_topic': '',
                'freq': 20.0}],
            remappings=[('scan', '/lidar/scan'),
                        ('odom', '/odom/laser_scan_matcher')]
    )

    # #################### End LaserScan (or PointCloud) to Odometry

    # depth_image_node = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(depth_launch_path),
    #         condition=IfCondition(PythonExpression(['"" != "', depth_sensor_name, '"'])),
    #         launch_arguments={'sensor': depth_sensor_name}.items()
    #     )

    realsense_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [f1tenth_launch_dir, 'launch/sensors', 'realsense_d435i.launch.py']
            ))
    )

    depth_to_laserscan_node = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan_node',
            output='screen',
            parameters=[{'scan_time': 0.0333},
                        {'output_frame': "camera_link"},
                        {'range_min': 0.45},
                        {'range_max': 10.0}],
            arguments=['depth:=/camera/depth/image_rect_raw',
                       'depth_camera_info:=/camera/depth/camera_info',
                       'scan:=/scan/depth_image'])

    # ################# Depth Image to PointCloud
    depth_image_to_pointcloud_xyz_node = ComposableNodeContainer(
            condition=IfCondition([depthimage_to_pointcloud]),
            name='depth_image_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                ComposableNode(
                        package='depth_image_proc',
                        plugin='depth_image_proc::PointCloudXyzNode',
                        name='point_cloud_xyz_node',
                        remappings=[('image_rect', '/camera/depth/image_rect_raw'),  # or aligned depth
                                    ('camera_info', '/camera/depth/camera_info'),
                                    ('image', '/camera/depth/converted_image')]
                ),
            ],
            output='screen',
    )

    stereo_to_pointcloud_node = ComposableNodeContainer(
            condition=IfCondition([stereo_to_pointcloud]),
            name='stereo_image_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                        condition=IfCondition([stereo_to_pointcloud]),
                        package='stereo_image_proc',
                        plugin='stereo_image_proc::DisparityNode',
                        parameters=[{
                            'approximate_sync': 'False',
                            'use_system_default_qos': 'False',
                            'stereo_algorithm': '0',  # 0: block matching, 1: semi-global block matching
                            'prefilter_size': '9',
                            'prefilter_cap': '31',
                            'correlation_window_size': '15',
                            'min_disparity': '0',
                            'disparity_range': '64',
                            'texture_threshold': '10',
                            'speckle_size': '100',
                            'speckle_range': '4',
                            'disp12_max_diff': '0',
                            'uniqueness_ratio': '15.0',
                            'P1': '200.0',
                            'P2': '400.0',
                            'full_dp': 'False',
                            'queue_size': '1',
                        }],
                        remappings=[
                            ('left/image_rect', '/camera/infra1/image_rect_raw'),
                            ('left/camera_info', '/camera/infra1/camera_info'),
                            ('right/image_rect', '/camera/infra2/image_rect_raw'),
                            ('right/camera_info', '/camera/infra2/camera_info'),
                        ]
                ),
                ComposableNode(
                        condition=IfCondition([stereo_to_pointcloud]),
                        package='stereo_image_proc',
                        plugin='stereo_image_proc::PointCloudNode',
                        parameters=[{
                            'approximate_sync': 'False',
                            'avoid_point_cloud_padding': 'False',
                            'use_color': 'False',
                            'use_system_default_qos': 'False',
                            'queue_size': '1',
                        }],
                        remappings=[
                            ('left/camera_info', '/camera/infra1/camera_info'),
                            ('right/camera_info', '/camera/infra2/camera_info'),
                            ('left/image_rect_color', '/camera/infra1/image_rect_raw'),
                        ]
                ),
            ],
            output='screen',
    )

    stereo_and_depth_image_processing_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                        [f1tenth_launch_dir, 'launch/sensors', 'stereo_and_depth_image_processing.launch.py']
                )),
                # condition=LaunchConfigurationEquals('mapping', 'realsense'),
                # condition=IfCondition([imu_only]),
                launch_arguments={
                    'queue_size': 1,  # default: 10
                    'approx_sync': approx_sync,
                    'use_sim_time': use_sim_time,
                    #
                    # 'frame_id': 'base_link',
                    # # 'odom_frame_id': 'odom',  # if empty or commented out, uses odom topic instead
                    # 'vo_frame_id': 'odom',
                    # 'map_frame_id': 'map',
                    # 'publish_tf_map': publish_map_tf,
                    # 'publish_tf_odom': 'false',

                    # 'stereo': use_stereo,
                    # 'depth': PythonExpression(['not ', use_stereo]),
                    # 'localization': localization,
                    # 'visual_odometry': 'false',  # odometry from images, eg stereo or RGB-D
                    # 'icp_odometry': 'false',  # odometry from laserscans or PointClouds
                    # 'subscribe_scan': 'true',
                    # 'subscribe_scan_cloud': 'false',
                    #
                    # 'odom_topic': '/odometry/local',
                    # 'odom_args': '',
                    #
                    # 'imu_topic': imu_topic,
                    # 'wait_imu_to_init': wait_imu_to_init,
                    #
                    # 'stereo_namespace': '/camera',
                    # 'left_image_topic': '/camera/infra1/image_rect_raw',
                    # 'right_image_topic': '/camera/infra2/image_rect_raw',
                    # 'left_camera_info_topic': '/camera/infra1/camera_info',
                    # 'right_camera_info_topic': '/camera/infra2/camera_info',
                    #
                    # 'rgb_topic': '/camera/color/image_raw',
                    # 'depth_topic': depth_topic,
                    # 'camera_info_topic': '/camera/color/camera_info',
                    #
                    # 'scan_topic': '/lidar/scan_filtered',
                    # # 'scan_cloud_topic': '/lidar/point_cloud',
                    #
                    # 'rtabmap_viz': rtabmap_viz_view,
                    # 'rviz': rviz_view,
                    # # 'rviz_cfg': '',
                }.items()
        )

    # Add nodes to launch description
    ld.add_action(lidar_node)

    ld.add_action(rtabmap_icp_odometry)
    ld.add_action(rf2o_odometry_node)
    # ld.add_action(laser_scan_matcher_node)

    ld.add_action(realsense_node)
    # ld.add_action(realsense_imu_node)
    ld.add_action(depth_to_laserscan_node)

    ld.add_action(depth_image_to_pointcloud_xyz_node)
    ld.add_action(stereo_to_pointcloud_node)
    ld.add_action(stereo_and_depth_image_processing_node)
    return ld
