"""
Fuses stereo images, IMU, ekf_odom
"""
# Example:
#   $ ros2 launch velodyne_driver velodyne_driver_node-VLP16-launch.py
#   $ ros2 launch velodyne_pointcloud velodyne_transform_node-VLP16-launch.py
#
#   SLAM:
#   $ ros2 launch f1tenth_launch 3d_mapping.launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory


def generate_launch_description():
    f1tenth_launch_pkg_prefix = get_package_share_directory('f1tenth_launch')

    database_file = os.path.join(
            f1tenth_launch_pkg_prefix, 'data/maps/rtabmap', 'rtabmap.db')
    rviz_cfg_path = os.path.join(
            f1tenth_launch_pkg_prefix, 'rviz', 'rtabmap.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_stereo = LaunchConfiguration('use_stereo')
    localization = LaunchConfiguration('localization')
    queue_size = LaunchConfiguration('queue_size')
    approx_sync = LaunchConfiguration('approx_sync')
    publish_map_tf = LaunchConfiguration('publish_map_tf')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')
    wait_imu_to_init = LaunchConfiguration('wait_imu_to_init')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    gps_topic = LaunchConfiguration('gps_topic')
    database_path = LaunchConfiguration('database_path')
    rtabmap_args = LaunchConfiguration('rtabmap_args')
    rtabmap_viz_view = LaunchConfiguration('rtabmap_viz_view')
    rviz_view = LaunchConfiguration('rviz_view')
    rviz_cfg_path_param = LaunchConfiguration('rviz_cfg_path_param', default=rviz_cfg_path)

    # /camera/camera/depth/image_rect_raw, /camera/depth_registered/image_rect, /camera/realigned_depth_to_color/image_raw
    depth_topic = LaunchConfiguration('depth_topic')

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
                'use_stereo', default_value='False',
                description='Whether to use Stereo or RGB+D'),

        DeclareLaunchArgument(
                'use_sim_time', default_value='true',
                description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
                'localization', default_value='false',
                description=''),

        DeclareLaunchArgument(
                'queue_size', default_value='10000',  # 10000 (50) for offline (online) mapping, 10 for localization
                description=''),

        DeclareLaunchArgument(
                'publish_map_tf', default_value='True',  # True for mapping, False for localization
                description=''),

        DeclareLaunchArgument(
                'lidar_frame_id', default_value='lidar',
                description='Lidar frame'),

        DeclareLaunchArgument(
                'wait_imu_to_init', default_value='true',
                description=''),

        DeclareLaunchArgument(
                'imu_topic', default_value='/camera/camera/imu/filtered',
                description='Used with VIO approaches and for SLAM graph optimization (gravity constraints). '),

        DeclareLaunchArgument(
                'depth_topic', default_value='/camera/camera/aligned_depth_to_color/image_raw',
                description='Raw unaligned depth topic to subscribe to. E.g '
                            '"/camera/camera/aligned_depth_to_color/image_raw", '
                            '"/camera/camera/depth/image_rect_raw", '
                            '"/camera/depth_registered/image_rect", '
                            '"/camera/realigned_depth_to_color/image_raw"'),

        DeclareLaunchArgument(
                'approx_sync', default_value='True',
                description='Synchronize topics'),

        DeclareLaunchArgument(
                'rtabmap_viz_view', default_value='False',
                description='Whether to start RTABMAP viz'),

        DeclareLaunchArgument(
                'rviz_view', default_value='True',
                description='Whether to start Rviz'),

        DeclareLaunchArgument('database_path', default_value=database_file,
                              description='Where is the map saved/loaded.'),

        DeclareLaunchArgument(
                'rviz_cfg_path_param',
                default_value=rviz_cfg_path_param,
                description='Launch RVIZ2 with the specified config file'
        ),

        # todo: use parameters instead of args (http://wiki.ros.org/rtabmap_ros/Tutorials/Advanced%20Parameter%20Tuning#Change_Parameters)
        DeclareLaunchArgument('rtabmap_args', default_value='-d '
                                                            '--RGBD/LoopClosureReextractFeatures true '
                                                            '--Vis/MinInliers 15 '
                                                            '--Vis/EstimationType 1 '
                                                            '--Vis/MaxDepth 0 '
                                                            '--GFTT/QualityLevel 0.00001 '
                                                            '--Stereo/MinDisparity 0 '
                                                            '--Stereo/MaxDisparity 64 '
                                                            # '--Vis/RoiRatios "0 0 0 .2" '
                                                            # "--Kp/RoiRatios '0 0 0 .2' "
                                                            # '--Odom/GuessMotion true '
                                                            '--Vis/BundleAdjustment 1 '
                                                            # '--OdomF2M/BundleAdjustment 1 '
                                                            '--Vis/CorNNDR 0.6 '
                                                            '--Vis/CorGuessWinSize 20 '
                                                            '--Vis/PnPFlags 0 '
                                                            # '--Odom/Strategy 1 '
                                                            '--Vis/CorType 1 '
                                                            # '--Odom/KeyFrameThr 0.6'
                                                            '--Reg/Force3DoF true '
                                                            '--Rtabmap/DetectionRate 0 '  # set to 0 to use image rate
                                                            '--Optimizer/Slam2D true '
                                                            '--Optimizer/GravitySigma 0',
                              description='Can be used to pass RTAB-Map\'s parameters or other flags like'
                                          ' --udebug and --delete_db_on_start/-d'),

        # Nodes to launch.
        # https://github.com/introlab/rtabmap_ros/blob/humble-devel/rtabmap_launch/launch/rtabmap.launch.py
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                        [FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py']
                )),
                # condition=LaunchConfigurationEquals('mapping', 'realsense'),
                # condition=IfCondition([imu_only]),
                launch_arguments={
                    # 'cfg': '',
                    'args': rtabmap_args,
                    'rtabmap_args': rtabmap_args,
                    'database_path': database_path,

                    'queue_size': queue_size,  # default: 10
                    'wait_for_transform': '0.2',

                    'frame_id': 'base_link',
                    # 'odom_frame_id': 'odom',  # if empty or commented out, uses odom topic instead
                    'vo_frame_id': 'odom',
                    'map_frame_id': 'map',
                    'publish_tf_map': publish_map_tf,
                    'publish_tf_odom': 'false',

                    'stereo': use_stereo,
                    'depth': PythonExpression(['not ', use_stereo]),
                    'localization': localization,
                    'visual_odometry': 'false',  # odometry from images, eg stereo or RGB-D
                    'icp_odometry': 'false',  # odometry from laserscans or PointClouds
                    'subscribe_scan': 'true',
                    'subscribe_scan_cloud': 'false',

                    'odom_topic': '/odometry/local',
                    'odom_args': '',

                    'imu_topic': imu_topic,
                    'wait_imu_to_init': wait_imu_to_init,

                    'stereo_namespace': '/camera/camera',
                    'left_image_topic': '/camera/camera/infra1/image_rect_raw',
                    'right_image_topic': '/camera/camera/infra2/image_rect_raw',
                    'left_camera_info_topic': '/camera/camera/infra1/camera_info',
                    'right_camera_info_topic': '/camera/camera/infra2/camera_info',

                    'rgb_topic': '/camera/camera/color/image_raw',
                    'depth_topic': depth_topic,
                    'camera_info_topic': '/camera/camera/color/camera_info',

                    'scan_topic': '/lidar/scan_filtered',
                    # 'scan_cloud_topic': '/lidar/point_cloud',

                    'approx_sync': approx_sync,

                    'rtabmap_viz': rtabmap_viz_view,
                    'rviz': rviz_view,
                    'rviz_cfg': rviz_cfg_path_param,
                    'use_sim_time': use_sim_time,
                }.items()
        ),
    ])


