#!/usr/bin/env python3
"""
Notes:
    * This launch file and inherited (children) launch files support Pushed namespaces.
    Therefore, we can add a 'sensing' namespace for all sensors here, then each node/include can have its own namespace appended.

Todo: launch realsense URDF/Xacro with robot state publisher using _d435i.urdf.xacro or test_d435i_camera.urdf.xacro (https://navigation.ros.org/setup_guides/urdf/setup_urdf.html  | https://github.com/IntelRealSense/realsense-ros/blob/ros2-development/realsense2_description/launch/view_model.launch.py)

Steps:
    * include lidar launch file and pass launch filter argument bool to lidar launch file
    * include camera_depth launch file and pass arguments for camera_depth filter,  IMU and IMU filter
    * (optional) include depth_image_to_laserscan launch file and pass argument to filter laserscan
    * (optional) publish lidar odometry
    * (optional) publish visual odometry
"""
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer, PushRosNamespace, SetRemap
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LifecycleNode
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess, TimerAction, GroupAction, \
    OpaqueFunction, SetEnvironmentVariable, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    qos_str_to_rtabmap_int = {
        'SENSOR_DATA': 2,
        'SYSTEM_DEFAULT': 0,
        'DEFAULT': 1,
    }

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

    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    use_namespace = LaunchConfiguration('use_namespace', default=False)
    namespace = LaunchConfiguration('namespace', default='sensing')
    camera_name = LaunchConfiguration('camera_name', default='camera')
    approx_sync = LaunchConfiguration('approx_sync')
    stereo_to_pointcloud = LaunchConfiguration('stereo_to_pointcloud')
    depthimage_to_pointcloud = LaunchConfiguration('depthimage_to_pointcloud')
    detect_ground_and_obstacles = LaunchConfiguration('detect_ground_and_obstacles')
    reset_realsense = LaunchConfiguration('reset_realsense')
    publish_realsense_pointcloud = LaunchConfiguration('publish_realsense_pointcloud')
    align_realsense_depth = LaunchConfiguration('align_realsense_depth')
    realsense_emitter_enabled = LaunchConfiguration('realsense_emitter_enabled')
    realsense_emitter_on_off = LaunchConfiguration('realsense_emitter_on_off')
    launch_realsense_splitter_node = LaunchConfiguration('launch_realsense_splitter_node', default=False)
    camera_launch_delay = LaunchConfiguration('camera_launch_delay', default=6.0)
    laserscan_launch_delay = LaunchConfiguration('laserscan_launch_delay', default=2.0)
    depth_to_laserscan = LaunchConfiguration('depth_to_laserscan', default='False')
    qos = LaunchConfiguration('qos', default='SENSOR_DATA')
    use_composition = LaunchConfiguration('use_composition', default=False)
    component_container_name = LaunchConfiguration('component_container_name', default='sensing_container')
    attach_to_shared_component_container = LaunchConfiguration('attach_to_shared_component_container', default=False)
    intra_process_comms = LaunchConfiguration("intra_process_comms", default=True)
    container_name_full = (namespace, '/', component_container_name)

    # Launch Arguments
    use_sim_time_la = DeclareLaunchArgument(
            'use_sim_time', default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true')
    use_namespace_la = DeclareLaunchArgument(
            'use_namespace', default_value=use_namespace,
            description='Use namespace if true')
    namespace_la = DeclareLaunchArgument(
            'namespace', default_value=namespace,
            description='Namespace for the nodes')
    camera_name_la = DeclareLaunchArgument('camera_name', default_value=camera_name,
                                           description='The name of the camera node.')
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
    depthimage_to_pointcloud_la = DeclareLaunchArgument(
            'depthimage_to_pointcloud',
            default_value='True',
            description='Whether to publish a PointCloud2 message from a depth image. '
                        'This method is recommended over stereo if depth is available.'
    )
    detect_ground_and_obstacles_la = DeclareLaunchArgument('detect_ground_and_obstacles',
                                                           default_value='False',
                                                           description='Whether to use RTABmaps obstacle detector.')

    reset_realsense_la = DeclareLaunchArgument(
            'reset_realsense',
            default_value='False',
            description='Whether to reset the realsense device.')

    publish_realsense_pointcloud_la = DeclareLaunchArgument('publish_realsense_pointcloud',
                                                            default_value='False',
                                                            description='Whether to publish PointClouds using '
                                                                        'librealsense SDK. Could be disabled when '
                                                                        'recording ROSBags or mapping. ')

    align_realsense_depth_la = DeclareLaunchArgument('align_realsense_depth',
                                           default_value='True',
                                           description='Whether to align the depth to other frames')

    realsense_emitter_enabled_la = DeclareLaunchArgument(
            'realsense_emitter_enabled',
            default_value='0',
            description='Whether to enable the IR emitters to improve depth and pointcloud quality. '
                        'Unfortunately, this renders the stereo IR cameras unusable for mapping, '
                        'VSLAM, VIO odometry, etc. '
                        'Disable when mapping or running VIO enable if accurate pointclouds are essential.')

    realsense_emitter_on_off_la = DeclareLaunchArgument(
            'realsense_emitter_on_off',
            default_value='False',
            description='Whether to alternate enabling/disabling the emitters. '
                        'This can be used to simultaneously '
                        'get accurate depth maps and pointclouds (when in the on state, i.e enabled) and '
                        'have usable IR images (when in the off state)')

    launch_realsense_splitter_node_la = DeclareLaunchArgument(
            'launch_realsense_splitter_node', default_value=launch_realsense_splitter_node,
            description='Whether to launch the realsense splitter node.')

    camera_launch_delay_la = DeclareLaunchArgument(
            'camera_launch_delay', default_value=camera_launch_delay,
            description='Delay in seconds before launching the camera nodes. '
                        'Used to avoid USB bandwidth limitations, '
                        'especially startup current draw caused by booting multiple USB devices simultaneously.')

    laserscan_launch_delay_la = DeclareLaunchArgument(
            'laserscan_launch_delay', default_value=laserscan_launch_delay,
            description='Delay in seconds before launching the laserscan nodes. '
                        'Used to avoid USB bandwidth limitations, '
                        'especially startup current draw caused by booting multiple USB devices simultaneously.')
    depth_to_laserscan_la = DeclareLaunchArgument(
            'depth_to_laserscan', default_value=depth_to_laserscan,
            description='Whether to publish a laserscan from the depth image. '
                        'Default is False because it is not used. '
                        'Costmaps can just use pointclouds instead. '
                        'Also, causes significant performance drop and CPU overhead.')

    declare_qos_cmd = DeclareLaunchArgument(
            'qos',
            default_value=qos,
            description='Quality of Service setting for the camera node. '
                        'Options: SENSOR_DATA, DEFAULT, SYSTEM_DEFAULT, SENSORS, PARAMETERS, SERVICES, ACTIONS. '
                        'Recommended: SENSOR_DATA for camera nodes or SYSTEM_DEFAULT')

    use_composition_cmd = DeclareLaunchArgument('use_composition', default_value=use_composition,
                                                description='Whether to use composition for the realsense camera.')
    component_container_name_cmd = DeclareLaunchArgument('component_container_name',
                                                         default_value=component_container_name,
                                                         description='Name of the component container to use.')
    attach_to_shared_component_container_cmd = DeclareLaunchArgument('attach_to_shared_component_container',
                                                                     default_value=attach_to_shared_component_container,
                                                                     description="Whether or not to join a container")
    intra_process_comms_arg = DeclareLaunchArgument('intra_process_comms',
                                                    default_value=intra_process_comms,
                                                    description="Whether to launch "
                                                                "components with intra process communication.")

    # Create Launch Description
    launch_args = [
        use_sim_time_la, use_namespace_la, namespace_la,
        camera_name_la,
        approx_sync_la,
        lidar_la, depth_la,
        stereo_to_pointcloud_la, depthimage_to_pointcloud_la, detect_ground_and_obstacles_la,
        reset_realsense_la, publish_realsense_pointcloud_la, align_realsense_depth_la,
        realsense_emitter_enabled_la,
        realsense_emitter_on_off_la, launch_realsense_splitter_node_la,
        camera_launch_delay_la, laserscan_launch_delay_la, depth_to_laserscan_la, declare_qos_cmd,
        use_composition_cmd, component_container_name_cmd, attach_to_shared_component_container_cmd,
        intra_process_comms_arg
    ]

    # Get camera name string from LaunchConfiguration
    camera_name_str = camera_name.perform(context)
    qos_str = qos.perform(context)
    use_namespace_str = use_namespace.perform(context)
    namespace_str = namespace.perform(context)
    if camera_name_str != '':
        camera_name_str += '/'

    qos_int = qos_str_to_rtabmap_int.get(qos_str, 2)

    # Nodes
    lidar_group = TimerAction(
            period=laserscan_launch_delay,
            actions=[
                GroupAction([
                    PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
                    SetRemap(src=['/tf'], dst=['tf']),
                    SetRemap(src=['/tf_static'], dst=['tf_static']),
                    IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(PathJoinSubstitution(
                                    [f1tenth_launch_dir, 'launch/sensors', 'ydlidar.launch.py']
                            )),
                            launch_arguments={
                                'launch_filter': 'True',
                                'use_namespace': 'True',  # use_namespace,
                                'namespace': 'lidar',  # namespace
                            }.items()
                    )
                ])
            ]
    )

    realsense_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [f1tenth_launch_dir, 'launch/sensors', 'realsense_d435i.launch.py']
            )),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_namespace': 'False',
                'namespace': namespace if (use_namespace_str.lower() == 'true' and namespace_str) else '',
                'camera_name': camera_name,
                'qos': qos,
                'reset_realsense': reset_realsense,
                'enable_pointcloud': publish_realsense_pointcloud,
                "align_depth": align_realsense_depth,
                'emitter_enabled': realsense_emitter_enabled,
                'emitter_on_off': realsense_emitter_on_off,
                'launch_realsense_splitter_node': launch_realsense_splitter_node,
                'use_composition': use_composition,
                'component_container_name': component_container_name,
                'attach_to_shared_component_container': attach_to_shared_component_container,
                'intra_process_comms': intra_process_comms,
            }.items()
    )

    depth_to_laserscan_node = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan_node',
            # namespace=None,  # namespace,
            condition=IfCondition(depth_to_laserscan),
            output='screen',
            parameters=[{'scan_time': 0.0333},  # 1 / (desired_frequency, i.e 30.0 Hz)
                        {'use_sim_time': use_sim_time},
                        {'output_frame': "camera_link"},  # camera_link, sensor_kit_link
                        # {'scan_height': 1},  # number of pixel rows to use. The minimum is selected
                        {'range_min': 0.01},  # 0.45
                        {'range_max': 10.0},
                        # {
                        #     'qos_overrides./parameter_events.publisher.depth': 5,
                        #     'qos_overrides./parameter_events.publisher.reliability': 'best_effort',
                        #     # 'qos_overrides./parameter_events.publisher.history': 'keep_last',
                        #     # 'qos_overrides./parameter_events.publisher.durability': 'volatile'
                        # }  # Sensor Data QoS. For now has no effect
                        ],
            remappings=[
                ('depth', camera_name_str + 'depth/image_rect_raw'),  # camera/camera/aligned_depth_to_color/image_raw
                ('depth_camera_info', camera_name_str + 'depth/camera_info'),
                ('scan', 'scan/from_depth_image')
            ],
    )

    # depth and/or stereo to Pointcloud
    stereo_and_depth_image_processing_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [f1tenth_launch_dir, 'launch/sensors', 'stereo_and_depth_image_processing.launch.py']
            )),
            # condition=IfCondition([imu_only]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_namespace': 'False',  # use_namespace,
                'namespace': namespace if (use_namespace_str.lower() == 'true' and namespace_str) else '',  # namespace,
                'camera_name': camera_name,
                'approx_sync': approx_sync,
                'queue_size': '1',  # default: 10
                'depthimage_to_pointcloud': depthimage_to_pointcloud,
                'stereo_to_pointcloud': stereo_to_pointcloud,
                'left_image_topic': camera_name_str + 'infra1/image_rect_raw',
                'right_image_topic': camera_name_str + 'infra2/image_rect_raw',
                'rgb_image_topic': camera_name_str + 'color/image_raw',
                'depth_image_topic': camera_name_str + 'aligned_depth_to_color/image_raw', # 'camera/camera/depth/image_rect_raw',
                'color_pointcloud': 'True',
                'use_image_proc': 'False',
                'use_rtabmap': 'True',
                'use_gpu': 'False',
                'detect_ground_and_obstacles': detect_ground_and_obstacles,
                'register_depth': 'False',
                'rtabmap_depth_decimation': '2',  # 1 means no decimation,
                'rtabmap_voxel_size': '0.1',  # 0.0 means no filtering
                'qos': str(qos_int),
                'use_system_default_qos': 'True' if qos_str == 'SYSTEM_DEFAULT' else 'False',
                'attach_to_shared_component_container': attach_to_shared_component_container,
                'component_container_name': component_container_name,
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
                ('cloud', 'camera/camera/depth/color/points'),
                ('obstacles', 'camera/camera/obstacles_from_cloud'),
                ('ground', 'camera/camera/ground_from_cloud'),
                ('proj_obstacles', 'camera/camera/projected_obstacles')
            ]
    )

    image_processing_group = TimerAction(
            period=camera_launch_delay,
            actions=[
                GroupAction([
                    PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
                    SetRemap(src=['/tf'], dst=['tf']),
                    SetRemap(src=['/tf_static'], dst=['tf_static']),
                    realsense_node,
                    depth_to_laserscan_node,
                    stereo_and_depth_image_processing_node,
                    rtabmap_obstacle_and_floor_detection_node
                ])
            ]
    )

    # Add nodes to launch description
    ld = launch_args + [
        image_processing_group,
        lidar_group,
    ]
    return ld


def generate_launch_description():
    return LaunchDescription(
            [
                SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
                OpaqueFunction(function=launch_setup)
            ]
    )
