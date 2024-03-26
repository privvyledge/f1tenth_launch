"""
Takes in Stereo images, outputs a depth/disparity image, decimate the depth image and output a pointcloud.
Could also take in a disparity or depth image.

Uses the following packages:
    * Stereo_image_proc: http://wiki.ros.org/stereo_image_proc
    * Depth_image_proc: http://wiki.ros.org/depth_image_proc?distro=noetic | https://github.com/ros-perception/image_pipeline/tree/humble/depth_image_proc/launch
    * Rtabmap_util: http://wiki.ros.org/rtabmap_util

Performance Analysis using CPU from fastest to lowest:
    Summary, use depth nodes as they are faster (>8Hz vs <1Hz for depth), but use stereo nodes with GPU
    1. rtabmap_depth_to_pointcloud_xyz (>10 Hz). Due to voxelization
    2. rtabmap_obstacle_and_floor_detection_node (>6 Hz). Due to voxelization of rtabmap_depth_to_pointcloud_xyz
    3. depth_image_to_pointcloud_xyz_node (3 Hz)
    4. rtabmap_pointcloud_to_depth (3 Hz)
    5. depth_image_proc_registration_node (2 Hz)
    6. rtabmap_disparity_to_depth (>1 Hz). Depends on the rate at which disparity is published)
    7. stereo_to_dipsarity (>3 Hz)
    8. stereo_to_pointcloud (0.5 Hz)

Todo:
    * decimate/downsample images (color or depth first) then use stereo_image_proc
        (http://wiki.ros.org/image_proc?distro=noetic#image_proc.2Fdiamondback.image_proc.2Fcrop_decimate |
         https://www.robotandchisel.com/2020/09/01/navigation2/ |
         https://github.com/ros-perception/image_pipeline/blob/humble/image_proc/launch/image_proc.launch.py)
"""

# Using stereo image proc
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

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    approx_sync = LaunchConfiguration('approx_sync')
    queue_size = LaunchConfiguration('queue_size')
    use_stereo = LaunchConfiguration('use_stereo')
    use_depth = LaunchConfiguration('use_depth')
    detect_ground_and_obstacles = LaunchConfiguration('detect_ground_and_obstacles')
    register_depth = LaunchConfiguration('register_depth')
    stereo_to_pointcloud = LaunchConfiguration('stereo_to_pointcloud')
    depthimage_to_pointcloud = LaunchConfiguration('depthimage_to_pointcloud')
    left_image_topic = LaunchConfiguration('left_image_topic')
    right_image_topic = LaunchConfiguration('right_image_topic')
    rgb_image_topic = LaunchConfiguration('rgb_image_topic')
    depth_image_topic = LaunchConfiguration('depth_image_topic')

    # Launch Arguments
    use_sim_time_la = DeclareLaunchArgument(
                'use_sim_time', default_value='False',
                description='Use simulation (Gazebo) clock if true')
    approx_sync_la = DeclareLaunchArgument(
                'approx_sync', default_value='True',
                description='Synchronize topics')
    queue_size_la = DeclareLaunchArgument(
                'queue_size', default_value='1',
                description='')
    use_stereo_la = DeclareLaunchArgument(
                'use_stereo', default_value='False',
                description='Whether to use Stereo image')
    use_depth_la = DeclareLaunchArgument(
            'use_depth', default_value='True',
            description='Whether to use RGB+D')
    detect_ground_and_obstacles_la = DeclareLaunchArgument(
            'detect_ground_and_obstacles', default_value='True',
            description='Whether to detect and isolate the floor plane and obstacles from raw pointclouds.')
    register_depth_la = DeclareLaunchArgument(
            'register_depth', default_value='True',
            description='Whether to register depth to a different frame')
    stereo_to_pointcloud_la = DeclareLaunchArgument('stereo_to_pointcloud',
                                                    default_value='False',
                                                    description='Whether to publish a PointCloud2 message from stereo images.')
    depthimage_to_pointcloud_la = DeclareLaunchArgument('depthimage_to_pointcloud',
                                                        default_value='True',
                                                        description='Whether to publish a PointCloud2 message from a depth image.')
    left_image_topic_la = DeclareLaunchArgument(
            'left_image_topic', default_value='/camera/camera/infra1/image_rect_raw',
            description='')
    right_image_topic_la = DeclareLaunchArgument(
            'right_image_topic', default_value='/camera/camera/infra2/image_rect_raw',
            description='')
    rgb_image_topic_la = DeclareLaunchArgument(
            'rgb_image_topic', default_value='/camera/camera/color/image_raw',
            description='')
    depth_image_topic_la = DeclareLaunchArgument(
                'depth_image_topic', default_value='/camera/camera/depth/image_rect_raw',
                description='Raw unaligned depth topic to subscribe to. E.g '
                            '"/camera/depth_from_disparity"'
                            '"/camera/camera/depth/image_rect_raw", '
                            '"/camera/depth_registered/image_rect", '
                            '"/camera/realigned_depth_to_color/image_raw"')

    # Create Launch Description
    ld = LaunchDescription([use_sim_time_la, approx_sync_la, queue_size_la,
                            use_stereo_la, use_depth_la, detect_ground_and_obstacles_la, register_depth_la,
                            stereo_to_pointcloud_la, depthimage_to_pointcloud_la,
                            left_image_topic_la, right_image_topic_la,
                            rgb_image_topic_la, depth_image_topic_la])

    # Nodes
    # ########################## Stereo to disparity and pointcloud
    stereo_to_pointcloud_node = ComposableNodeContainer(
            condition=IfCondition([stereo_to_pointcloud]),
            name='stereo_image_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                        condition=IfCondition([use_stereo]),
                        package='stereo_image_proc',
                        plugin='stereo_image_proc::DisparityNode',
                        parameters=[{
                            'approximate_sync': approx_sync,
                            'use_system_default_qos': True,
                            'stereo_algorithm': 0,  # 0: block matching, 1: semi-global block matching
                            'prefilter_size': 9,
                            'prefilter_cap': 31,
                            'correlation_window_size': 15,
                            'min_disparity': 0,
                            'disparity_range': 64,
                            'texture_threshold': 10,
                            'speckle_size': 100,
                            'speckle_range': 4,
                            'disp12_max_diff': 0,
                            'uniqueness_ratio': 15.0,
                            'P1': 200.0,
                            'P2': 400.0,
                            'full_dp': False,
                            'queue_size': queue_size,
                            'use_sim_time': use_sim_time,
                        }],
                        remappings=[
                            ('left/image_rect', left_image_topic),
                            ('left/camera_info', '/camera/camera/infra1/camera_info'),
                            ('right/image_rect', right_image_topic),
                            ('right/camera_info', '/camera/camera/infra2/camera_info'),
                        ]
                ),
                ComposableNode(
                        condition=IfCondition([stereo_to_pointcloud]),
                        package='stereo_image_proc',
                        plugin='stereo_image_proc::PointCloudNode',
                        parameters=[{
                            'approximate_sync': approx_sync,
                            'avoid_point_cloud_padding': False,
                            'use_color': False,
                            'use_system_default_qos': True,
                            'queue_size': queue_size,
                            'use_sim_time': use_sim_time,
                        }],
                        remappings=[
                            ('left/camera_info', '/camera/camera/infra1/camera_info'),
                            ('right/camera_info', '/camera/camera/infra2/camera_info'),
                            ('left/image_rect_color', left_image_topic),
                            ('points', '/camera/points_from_stereo_proc'),
                        ]
                ),
            ],
            output='screen',
    )

    # ########################## Disparity to depth
    rtabmap_disparity_to_depth = Node(
                condition=IfCondition([use_stereo]),
                package='rtabmap_util', executable='disparity_to_depth', output='screen',
                parameters=[
                    {'approx_sync': approx_sync},
                    {'use_sim_time': use_sim_time},
                    {'queue_size': queue_size},
                ],
                remappings=[('disparity', '/disparity'),
                            ('depth', '/camera/depth_from_disparity'),
                            ('depth_raw', '/camera/depth_from_disparity_raw')])

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
                        parameters=[
                            {'use_sim_time': use_sim_time},
                            {'queue_size': queue_size},
                        ],
                        package='depth_image_proc',
                        plugin='depth_image_proc::PointCloudXyzNode',
                        name='point_cloud_xyz_node',
                        remappings=[('image_rect', depth_image_topic),  # or aligned depth
                                    ('camera_info', '/camera/camera/depth/camera_info'),
                                    ('points', '/camera/points_from_depth_proc')]
                ),
            ],
            output='screen',
    )

    # ######### RTabMap depth to pointcloud to depth
    rtabmap_depth_to_pointcloud_xyz = Node(
                condition=IfCondition([depthimage_to_pointcloud]),
                name='rtabmap_depth_to_pointcloud_xyz',
                package='rtabmap_util', executable='point_cloud_xyz', output='screen',
                parameters=[
                    {'decimation': 4},  # 1 to disable decimation
                    {'voxel_size': 0.2},  # (m) 0.0 to disable filtering
                    {'min_depth ': 0.2},  # (m) 0.0 to disable filtering
                    {'max_depth ': 6.0},  # (m) 0.0 to disable filtering
                    {'noise_filter_radius ': 0.2},  # (m) 0.0 to disable filtering
                    {'noise_filter_min_neighbors ': 5},  # (m) 0.0 to disable filtering. Minimum neighbors of a point to keep it
                    {'approx_sync': approx_sync},
                    {'use_sim_time': use_sim_time},
                    {'queue_size': queue_size},
                ],
                remappings=[
                    ('depth/image', depth_image_topic),
                    #('disparity/image', '/disparity'),
                    ('depth/camera_info', '/camera/camera/depth/camera_info'),
                    #('disparity/camera_info', '/camera/camera/infra1/camera_info'),
                    ('cloud', '/camera/downsampled_cloud_from_depth')])

    # ######### RTabMap depth to pointcloud to depth
    rtabmap_obstacle_and_floor_detection_node = Node(
            name='rtabmap_obstacle_and_floor_detection_node',
            condition=IfCondition([detect_ground_and_obstacles]),
            package='rtabmap_util', executable='obstacles_detection', output='screen',
            parameters=[
                {'frame_id': 'base_link'},  # 'camera_link', 'base_link'
                {'queue_size': queue_size},
                {'approx_sync': approx_sync},
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('cloud', '/camera/downsampled_cloud_from_depth'),
                ('obstacles', '/camera/obstacles_from_cloud'),
                ('ground', '/camera/ground_from_cloud')
            ])

    # to register depth to a different frame
    rtabmap_pointcloud_to_depth = Node(
                name='rtabmap_pointcloud_to_depth',
                condition=IfCondition([register_depth]),
                package='rtabmap_util', executable='pointcloud_to_depthimage', output='screen',
                parameters=[
                    {'decimation': 4,  # 2
                     'fixed_frame_id': 'camera_link',  # camera_link, base_link, odom
                     'fill_holes_size': 1},
                    {'use_sim_time': use_sim_time},
                    {'queue_size': queue_size},
                    {'approx': approx_sync}
                            ],
                remappings=[('camera_info', '/camera/camera/color/camera_info'),
                            ('cloud', '/camera/downsampled_cloud_from_depth'),
                            ('image', '/camera/realigned_depth_to_color/depthimage'),
                            ('image_raw', '/camera/realigned_depth_to_color/depthimage_raw')
                            ])

    depth_image_proc_registration_node = ComposableNodeContainer(
                condition=IfCondition([register_depth]),
                name='depth_registration_container',  # todo: add to depth_image_container
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    # Driver itself
                    ComposableNode(
                            package='depth_image_proc',
                            parameters=[
                                {'use_sim_time': use_sim_time},
                                {'queue_size': queue_size},
                            ],
                            plugin='depth_image_proc::RegisterNode',
                            name='depthimage_register_node',
                            remappings=[('depth/image_rect', depth_image_topic),
                                        ('depth/camera_info', '/camera/camera/depth/camera_info'),
                                        ('rgb/camera_info', '/camera/camera/color/camera_info'),
                                        ('depth_registered/image_rect', '/camera/depth_registered/image_rect'),
                                        ('depth_registered/camera_info', '/camera/depth_registered/camera_info')
                                        ]
                    ),
                ],
                output='screen',
        )

    # add nodes to Launch description
    ld.add_action(depth_image_to_pointcloud_xyz_node)
    ld.add_action(stereo_to_pointcloud_node)
    ld.add_action(rtabmap_disparity_to_depth)
    ld.add_action(rtabmap_depth_to_pointcloud_xyz)
    ld.add_action(rtabmap_obstacle_and_floor_detection_node)
    ld.add_action(rtabmap_pointcloud_to_depth)
    ld.add_action(depth_image_proc_registration_node)

    return ld
