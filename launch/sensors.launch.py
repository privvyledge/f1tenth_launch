#!/usr/bin/env python3
"""
Todo: launch realsense URDF/Xacro with robot state publisher using _d435i.urdf.xacro or test_d435i_camera.urdf.xacro (https://navigation.ros.org/setup_guides/urdf/setup_urdf.html  | https://github.com/IntelRealSense/realsense-ros/blob/ros2-development/realsense2_description/launch/view_model.launch.py)
"""
from launch import LaunchDescription
from launch_ros.actions import Node
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

    # Launch Arguments
    lidar_la = DeclareLaunchArgument('lidar_config',
                                     default_value=lidar_config,
                                     description='Path to the YDLIDAR parameters file to use.')
    depth_la = DeclareLaunchArgument('depth_config',
                                     default_value=depth_config,
                                     description='Path to the Realsense parameters file to use.')
    realsense_imu_la = DeclareLaunchArgument('realsense_imu_config',
                                             default_value=realsense_imu_config,
                                             description='Path to the Realsense IMU parameters file to use.')

    # Create Launch Description
    ld = LaunchDescription([lidar_la, depth_la])

    # Nodes
    lidar_node = LifecycleNode(package='ydlidar_ros2_driver',
                               executable='ydlidar_ros2_driver_node',
                               name='ydlidar_ros2_driver_node',
                               output='screen',
                               emulate_tty=True,
                               parameters=[LaunchConfiguration('lidar_config')],
                               namespace='lidar',
                               )

    # #################### Begin LaserScan (or PointCloud) to Odometry
    rtabmap_icp_odometry = Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='icp',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': False,
                'wait_for_transform': 0.1,
                # 'wait_imu_to_init ': False, # use if imu is passed
            }],
            remappings=[('scan', '/lidar/scan'),
                        # ('imu', '/vehicle/sensors/imu/raw'),  # imu must have orientation
                        ('odom', '/odom/rtabmap_icp'),
                        ('odom_last_frame', '/rtabmap_icp/points'),  # 'odom_last_frame ', 'odom_filtered_input_scan'
                        ]
    )

    # rf2o_odometry_node = Node(
    #         package='rf2o_laser_odometry',
    #         executable='rf2o_laser_odometry_node',
    #         name='rf2o_laser_odometry',
    #         output='screen',
    #         parameters=[{
    #             'laser_scan_topic': '/lidar/scan',
    #             'odom_topic': '/odom/rf2o',
    #             'publish_tf': False,
    #             'base_frame_id': 'base_link',
    #             'odom_frame_id': 'odom',
    #             'init_pose_from_topic': '',
    #             'freq': 10.0}],
    # )

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

    # realsense_node = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(PathJoinSubstitution(
    #             [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
    #         )),
    #         condition=LaunchConfigurationEquals('sensor', 'realsense'),
    #         launch_arguments={
    #             'pointcloud.enable': 'true',
    #             'ordered_pc': 'true',
    #             'initial_reset': 'true'
    #         }.items()
    #     )

    # depth_image_node = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(depth_launch_path),
    #         condition=IfCondition(PythonExpression(['"" != "', depth_sensor_name, '"'])),
    #         launch_arguments={'sensor': depth_sensor_name}.items()
    #     )

    realsense_node = Node(
            package='realsense2_camera',
            namespace='camera',
            name='camera',
            executable='realsense2_camera_node',
            parameters=[LaunchConfiguration('depth_config')],
            output='screen',
            emulate_tty=True,
    )

    realsense_imu_node = Node(
            package='realsense2_camera',
            # namespace='sensors/camera',
            name='camera',
            executable='realsense2_camera_node',
            parameters=[LaunchConfiguration('realsense_imu_config')],
            output='screen',
            emulate_tty=True,
    )

    depth_to_laserscan_node = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan_node',
            output='screen',
            parameters=[{'scan_time': 0.0333},
                        {'output_frame': "camera_depth"},
                        {'range_min': 0.45},
                        {'range_max': 4.0}],
            arguments=['depth:=/camera/depth/image_rect_raw',
                       'depth_camera_info:=/camera/depth/camera_info',
                       'scan:=/scan'])

    # Add nodes to launch description
    ld.add_action(lidar_node)

    ld.add_action(rtabmap_icp_odometry)
    # ld.add_action(rf2o_odometry_node)
    # ld.add_action(laser_scan_matcher_node)

    # ld.add_action(realsense_node)
    # ld.add_action(realsense_imu_node)
    # ld.add_action(depth_to_laserscan_node)
    return ld
