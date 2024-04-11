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
    detect_ground_and_obstacles = LaunchConfiguration('detect_ground_and_obstacles')

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
                                                    description='Whether to publish a PointCloud2 message from stereo '
                                                                'images.')
    depthimage_to_pointcloud_la = DeclareLaunchArgument('depthimage_to_pointcloud',
                                                        default_value='False',
                                                        description='Whether to publish a PointCloud2 message from a '
                                                                    'depth image.')
    detect_ground_and_obstacles_la = DeclareLaunchArgument('detect_ground_and_obstacles',
                                                           default_value='False',
                                                           description='Whether to use RTABmaps obstacle detector.')

    # Create Launch Description
    ld = LaunchDescription([use_sim_time_la, approx_sync_la,
                            lidar_la, depth_la,
                            stereo_to_pointcloud_la, depthimage_to_pointcloud_la, detect_ground_and_obstacles_la])

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
                'use_sim_time': use_sim_time,
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
                'use_sim_time': use_sim_time,
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
                'use_sim_time': use_sim_time,
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
            parameters=[{'scan_time': 0.0333},  # 1 / (desired_frequency, i.e 30.0 Hz)
                        {'use_sim_time': use_sim_time},
                        {'output_frame': "camera_link"},  # camera_link, sensor_kit_link
                        # {'scan_height': 1},  # number of pixel rows to use. The minimum is selected
                        {'range_min': 0.01},  # 0.45
                        {'range_max': 10.0}],
            remappings=[
                ('depth', '/camera/camera/depth/image_rect_raw'),  # /camera/camera/aligned_depth_to_color/image_raw
                ('depth_camera_info', '/camera/camera/depth/camera_info'),
                ('scan', '/scan/from_depth_image')
            ],
            # arguments=['depth:=/camera/camera/depth/image_rect_raw',
            #            'depth_camera_info:=/camera/camera/depth/camera_info',
            #            'scan:=/scan/from_depth_image']
    )

    # depth and/or stereo to Pointcloud
    stereo_and_depth_image_processing_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [f1tenth_launch_dir, 'launch/sensors', 'stereo_and_depth_image_processing.launch.py']
            )),
            # condition=IfCondition([imu_only]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'approx_sync': approx_sync,
                'queue_size': '1',  # default: 10
                'depthimage_to_pointcloud': depthimage_to_pointcloud,
                'stereo_to_pointcloud': stereo_to_pointcloud,
                'left_image_topic': '/camera/camera/infra1/image_rect_raw',
                'right_image_topic': '/camera/camera/infra2/image_rect_raw',
                'rgb_image_topic': '/camera/camera/color/image_raw',
                'depth_image_topic': '/camera/camera/depth/image_rect_raw',
                'color_pointcloud': 'True',
                'use_image_proc': 'True',
                'use_rtabmap': 'False',
                'detect_ground_and_obstacles': detect_ground_and_obstacles,
                'register_depth': 'False',
                'rtabmap_depth_decimation': '2',  # 1 means no decimation,
                'rtabmap_voxel_size': '0.1',  # 0.0 means no filtering
            }.items()
    )

    # ######### RTabMap depth to pointcloud to depth. Note: use a voxelized pointcloud
    rtabmap_obstacle_and_floor_detection_node = Node(
            name='rtabmap_obstacle_and_floor_detection_node',
            condition=IfCondition(detect_ground_and_obstacles),
            package='rtabmap_util', executable='obstacles_detection', output='screen',
            parameters=[
                {'frame_id': 'base_link'},  # 'camera_link', 'base_link', 'sensor_kit_link', base_footprint
                {'queue_size': '1'},
                {'approx_sync': approx_sync},
                {'use_sim_time': use_sim_time},
                {'min_cluster_size': 20},  # Minimum size of the segmented clusters to keep. Default=20
                {'max_obstacles_height': 5.0},  # Maximum height of obstacles. Default=0.0
            ],
            remappings=[
                ('cloud', '/camera/camera/depth/color/points'),
                ('obstacles', '/camera/camera/obstacles_from_cloud'),
                ('ground', '/camera/camera/ground_from_cloud'),
                ('proj_obstacles', '/camera/camera/projected_obstacles')
            ]
    )

    # Add nodes to launch description
    ld.add_action(lidar_node)

    # ld.add_action(rtabmap_icp_odometry)
    # ld.add_action(rf2o_odometry_node)
    # # ld.add_action(laser_scan_matcher_node)

    ld.add_action(realsense_node)
    # ld.add_action(realsense_imu_node)
    ld.add_action(depth_to_laserscan_node)

    ld.add_action(stereo_and_depth_image_processing_node)
    ld.add_action(rtabmap_obstacle_and_floor_detection_node)
    return ld
