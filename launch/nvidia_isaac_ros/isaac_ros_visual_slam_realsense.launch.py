"""

"""

import os

from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes, SetRemap, PushRosNamespace, \
    SetParametersFromFile, SetParameter
from launch_ros.descriptions import ParameterFile, ComposableNode
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression, \
    EnvironmentVariable, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, GroupAction, \
    OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # Setup launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    namespace = LaunchConfiguration('namespace', default='')
    use_namespace = LaunchConfiguration('use_namespace', default=False)
    camera_name = LaunchConfiguration('camera_name', default='camera')
    two_d_mode = LaunchConfiguration('two_d_mode', default=False)
    base_frame = LaunchConfiguration('base_frame', default='base_link')  # camera_link, base_link
    publish_map_to_odom_tf = LaunchConfiguration('publish_map_to_odom_tf', default=True)
    publish_odom_to_baselink_tf = LaunchConfiguration('publish_odom_to_baselink_tf', default=True)
    save_map = LaunchConfiguration('save_map', default=False)
    load_map = LaunchConfiguration('load_map', default=False)
    map_path = LaunchConfiguration('map_path', default='/mnt/data/maps/nvidia/vslam_map')
    launch_realsense_driver = LaunchConfiguration('launch_realsense_driver', default=False)
    left_image_topic = LaunchConfiguration('left_image_topic')
    right_image_topic = LaunchConfiguration('right_image_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    image_qos = LaunchConfiguration('image_qos', default='SENSOR_DATA')
    imu_qos = LaunchConfiguration('imu_qos', default='SENSOR_DATA')
    enable_visualization_topics = LaunchConfiguration('enable_visualization_topics', default=True)
    localize_on_startup = LaunchConfiguration('localize_on_startup', default=False)
    attach_to_shared_component_container = LaunchConfiguration('attach_to_shared_component_container',
                                                               default=False)
    component_container_name = LaunchConfiguration('component_container_name', default='visual_slam_launch_container')
    # Use component_container_name directly — callers (localization.launch.py) pre-build the
    # absolute path (e.g. /gosling1/f1tenth_container).  Concatenating namespace here would
    # double the namespace and silently miss the target container.
    container_name_full = component_container_name

    # Declare launch arguments
    use_sim_time_la = DeclareLaunchArgument(
            'use_sim_time', default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true')
    namespace_la = DeclareLaunchArgument(
            'namespace', default_value=namespace,
            description='Namespace for the nodes')
    use_namespace_la = DeclareLaunchArgument(
            'use_namespace', default_value=use_namespace,
            description='Use namespace for the nodes')
    camera_name_launch_arg = DeclareLaunchArgument('camera_name', default_value=camera_name,
                                                 description='The name of the camera node.')
    two_d_mode_arg = DeclareLaunchArgument('two_d_mode',
                                           default_value=two_d_mode,
                                           description="Whether to apply 2D constraints, e.g. for cars and WMR.")

    base_frame_arg = DeclareLaunchArgument('base_frame',
                                           default_value=base_frame,
                                           description="The base frame of the robot")

    publish_map_to_odom_tf_arg = DeclareLaunchArgument('publish_map_to_odom_tf',
                                                       default_value=publish_map_to_odom_tf,
                                                       description="Publish the dynamic map -> odom transform")

    publish_odom_to_baselink_tf_arg = DeclareLaunchArgument('publish_odom_to_baselink_tf',
                                                            default_value=publish_odom_to_baselink_tf,
                                                            description="Publish the dynamic "
                                                                        "odom -> base_link transform")

    save_map_arg = DeclareLaunchArgument('save_map',
                                         default_value=save_map,
                                         description="Save the map to a file. Localization and mapping must be true. "
                                                     "I recommend running the action separately on the CLI."
                                                     "Warning: this deletes the entire directory specified.")

    load_map_arg = DeclareLaunchArgument('load_map',
                                         default_value=load_map,
                                         description="Load the map from a file. "
                                                     "I recommend running the action separately on the CLI.")

    map_path_arg = DeclareLaunchArgument('map_path',
                                         default_value=map_path,
                                         description="Path to save/load the visual slam map to/from. "
                                                     "Examples: "
                                                     "/mnt/shared_dir/maps/nvidia/vslam_map, "
                                                     "/mnt/data/maps/nvidia/vslam_map, "
                                                     "/f1tenth_ws/src/f1tenth_launch/data/maps/nvidia/vslam_map, "
                                                     "/shared_dir/maps/nvidia/vslam_map")

    launch_realsense_driver_launch_arg = DeclareLaunchArgument('launch_realsense_driver',
                                                               default_value=launch_realsense_driver,
                                                               description="Whether or not to "
                                                                           "launch the Realsense Camera.")

    left_image_topic_la = DeclareLaunchArgument(
            'left_image_topic', default_value='infra1/image_rect_raw',
            description='/camera/camera/infra1/image_rect_raw or /camera/realsense_splitter_node/output/infra_1')

    right_image_topic_la = DeclareLaunchArgument(
            'right_image_topic', default_value='infra2/image_rect_raw',
            description='/camera/camera/infra2/image_rect_raw or /camera/realsense_splitter_node/output/infra_2')

    imu_topic_la = DeclareLaunchArgument(
            'imu_topic', default_value='imu',
            description='/camera/camera/imu or /camera/camera/imu/filtered')

    image_qos_arg = DeclareLaunchArgument('image_qos', default_value=image_qos,
                                          description="The QoS of the stereo image topics. "
                                                      "Options: SENSOR_DATA, DEFAULT, SYSTEM_DEFAULT")

    imu_qos_arg = DeclareLaunchArgument('imu_qos', default_value=imu_qos,
                                        description="The output QoS of the IMU topic "
                                                    "Options: SENSOR_DATA, DEFAULT, SYSTEM_DEFAULT")

    enable_visualization_topics_arg = DeclareLaunchArgument('enable_visualization_topics',
                                                            default_value=enable_visualization_topics,
                                                            description="Whether to publish visualization topics like"
                                                                        " the frame pointcloud, landmarks, "
                                                                        "all poses from the start, etc.")

    localize_on_startup_arg = DeclareLaunchArgument(
            'localize_on_startup',
            default_value=localize_on_startup,
            description='Attempt to localize in a previously saved VSLAM map on startup. '
                        'Set False (default) when another node (e.g. AMCL) owns the map TF — '
                        'the GXF localization-failure cleanup path has a heap corruption bug '
                        'that causes intermittent SIGABRT when localization fails.')

    attach_to_shared_component_container_arg = DeclareLaunchArgument('attach_to_shared_component_container',
                                                                     default_value=attach_to_shared_component_container,
                                                                     description="Whether or not to join a container")

    component_container_name_arg = DeclareLaunchArgument('component_container_name',
                                                         default_value=component_container_name,
                                                         description="The name of the optional container to join")

    # Add launch arguments to a list
    launch_args = [
        use_sim_time_la,
        namespace_la,
        use_namespace_la,
        two_d_mode_arg,
        base_frame_arg,
        publish_map_to_odom_tf_arg,
        publish_odom_to_baselink_tf_arg,
        save_map_arg,
        load_map_arg,
        map_path_arg,
        launch_realsense_driver_launch_arg,
        left_image_topic_la,
        right_image_topic_la,
        imu_topic_la,
        image_qos_arg,
        imu_qos_arg,
        enable_visualization_topics_arg,
        localize_on_startup_arg,
        attach_to_shared_component_container_arg,
        component_container_name_arg
    ]

    # Get string values from launch configuration
    namespace_string = namespace.perform(context)
    use_namespace_string = use_namespace.perform(context)
    camera_name_string = camera_name.perform(context)
    component_container_name_string = component_container_name.perform(context)

    if camera_name_string != '':
        camera_name_string += '/'

    # Run nodes
    realsense_camera_node = Node(
            condition=IfCondition(launch_realsense_driver),
            name=camera_name,
            namespace=namespace if use_namespace_string.lower() == 'true' else '',
            package='realsense2_camera',
            executable='realsense2_camera_node',
            parameters=[{
                'use_sim_time': use_sim_time,
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
                # 'tf_publish_rate': 30.0
            }],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
    )

    # realsense_camera_node_composed = ComposableNode(
    #         condition=IfCondition(launch_realsense_driver),
    #         # name=camera_name,
    #         namespace=namespace if use_namespace_string.lower() == 'true' else '',
    #         package='realsense2_camera',
    #         plugin='realsense2_camera::RealSenseNodeFactory',
    #         extra_arguments=[{'use_intra_process_comms': LaunchConfiguration("intra_process_comms")}],
    #         parameters=[config_file])

    # A map must actually exist before cuVSLAM is asked to localize in it. The map directory used to be
    # created unconditionally further down, which manufactured an EMPTY map and made
    # "Failed to localize in map. Error 3" inevitable on every run without a previously saved map.
    map_path_string = map_path.perform(context)
    map_exists = os.path.isdir(map_path_string) and bool(os.listdir(map_path_string))
    save_map_requested = save_map.perform(context).lower() == 'true'
    # cuVSLAM autosaves into save_map_folder_path when the node shuts down, so handing it a
    # path while save_map is False silently writes a map anyway. The next launch then has a
    # map to relocalize into, and because visual_slam/initial_pose is remapped to initialpose
    # (below), the startup pose seed doubles as a relocalization hint that finds it. Measured
    # 2026-08-25: that relocalization drives ekf_odom to ~25 m/s while the car is stationary.
    # Give cuVSLAM nowhere to save unless saving was actually asked for.
    save_map_folder_path = map_path_string if save_map_requested else ''
    localize_on_startup_effective = (
            localize_on_startup.perform(context).lower() == 'true' and map_exists
    )
    # Handing cuVSLAM a load path it was not asked to use is not inert, because
    # visual_slam/initial_pose is remapped to `initialpose` below. That topic is not
    # only RViz's "2D Pose Estimate" -- localization.launch.py publishes onto it on
    # purpose, `initialpose_seed_delay` seconds after start, to seed ekf_map and AMCL
    # from one number. cuVSLAM reads the same message as a RELOCALIZATION HINT, so a
    # load path plus that seed is enough to start relocalization even with
    # localize_on_startup False.
    #
    # Measured 2026-08-26: with a map present the node logged "Trying to localize in
    # map ... around [0.445, -0.575, 0.000]" -- the AMCL seed -- then "Failed to
    # localize in map. Error 3" on repeat, and visual_slam_container died exit -6, at
    # t+20 s exactly matching initialpose_seed_delay. ekf_odom then lost odom1 entirely
    # and a parked yaw_drift read +3.77 deg/min against a +0.04 baseline, which looks
    # like a fusion regression and is not one.
    #
    # So gate the load path on the same condition as localize_on_startup: no intent to
    # relocalize means no map to relocalize into, and the seed cannot start something
    # nobody asked for.
    load_map_folder_path = map_path_string if localize_on_startup_effective else ''

    # if a transformError occurs, it's probably due to a low realsense tf_publish_rate
    visual_slam_node = ComposableNode(
            name='visual_slam_node',
            namespace=namespace if use_namespace_string.lower() == 'true' else '',
            package='isaac_ros_visual_slam',
            plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
            extra_arguments=[{"use_intra_process_comms": False}],
            parameters=[{
                'use_sim_time': use_sim_time,
                'num_cameras': 2,  # two for a single stereo camera
                'enable_image_denoising': False,
                'enable_rectified_pose': True,
                'rectified_images': True,
                'enable_imu_fusion': True,
                'enable_planar_mode': two_d_mode,
                'enable_ground_constraint_in_odometry': two_d_mode,
                'enable_ground_constraint_in_slam': two_d_mode,
                'enable_localization_n_mapping': True,  # True enables Odometry+SLAM, False=Odometry only. Set to False
                'gyro_noise_density': 0.000244,  # todo: get noise parameters from config file
                'gyro_random_walk': 0.000019393,
                'accel_noise_density': 0.001862,
                'accel_random_walk': 0.003,
                'calibration_frequency': 200.0,
                'sync_matching_threshold_ms': 5.0,  # approximate stereo synchronization tolerance
                'image_jitter_threshold_ms': 34.00,  # 1000 / (Hz) + buffer. Default: 34.0, realsense 22.00 (for 45 Hz)
                'imu_jitter_threshold_ms': 10.0,
                'image_buffer_size': 100,  # Default: 100
                'imu_buffer_size': 50,  # Default: 50
                'path_max_size': 200,  # Default: 1024. For visualization purposes only
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': base_frame,  # camera_link, base_link.
                'imu_frame': 'camera_imu_optical_frame',
                'publish_map_to_odom_tf': publish_map_to_odom_tf,
                'publish_odom_to_rig_tf': publish_odom_to_baselink_tf,
                'publish_odom_to_base_tf': publish_odom_to_baselink_tf,
                'invert_map_to_odom_tf': False,
                'invert_odom_to_rig_tf': False,
                'invert_odom_to_base_tf': False,
                'enable_slam_visualization': enable_visualization_topics,
                'enable_landmarks_view': enable_visualization_topics,
                'enable_observations_view': enable_visualization_topics,
                'camera_optical_frames': [
                    'camera_infra1_optical_frame',
                    'camera_infra2_optical_frame',
                ],
                'image_qos': image_qos,  # 'DEFAULT', 'SENSOR_DATA'
                'imu_qos': imu_qos,  # 'DEFAULT', 'SENSOR_DATA'
                'save_map_folder_path': save_map_folder_path,
                'load_map_folder_path': load_map_folder_path,
                'localize_on_startup': localize_on_startup_effective,
            }],
            remappings=[('visual_slam/image_0', left_image_topic),
                        ('visual_slam/camera_info_0', camera_name_string + 'infra1/camera_info'),
                        ('visual_slam/image_1', right_image_topic),
                        ('visual_slam/camera_info_1', camera_name_string + 'infra2/camera_info'),
                        ('visual_slam/imu', imu_topic),
                        ('visual_slam/initial_pose', 'initialpose'),  # remap to RViz2 topic
                        ('/tf', 'tf'),
                        ('/tf_static', 'tf_static')]
    )

    # Only create the map directory when we actually intend to save into it. Creating it
    # unconditionally left an empty directory behind that later runs then tried to localize in.
    if save_map_requested and not os.path.exists(map_path_string):
        os.makedirs(map_path_string, exist_ok=True)

    # to save a map (ros2 action send_goal /visual_slam/save_map isaac_ros_visual_slam_interfaces/action/SaveMap "{map_url: /shared_dir/maps/nvidia/vslam_map}")
    # warning must be in an empty directory cause this wipes out the directory
    save_map_trigger = TimerAction(
            period=60.0,
            actions=[
                ExecuteProcess(
                        cmd=[[
                            FindExecutable(name='ros2'),
                            f' service call {namespace_string}/visual_slam/save_map isaac_ros_visual_slam_interfaces/srv/FilePath '
                            '"{file_path: ' + map_path_string + '}"'
                        ]],
                        shell=True,
                        condition=IfCondition(save_map)
                )
            ]
    )  # ros2 service call /visual_slam/save_map isaac_ros_visual_slam_interfaces/srv/FilePath "{file_path: /path/to/save/the/map}"

    # to load a saved map (ros2 action send_goal /visual_slam/load_map_and_localize isaac_ros_visual_slam_interfaces/action/LoadMapAndLocalize "{map_url: /shared_dir/maps/nvidia/vslam_map, localize_near_point: {x: x_val, y: y_val, z: z_val}}")
    # ros2 service call /visual_slam/localize_in_map isaac_ros_visual_slam_interfaces/srv/LocalizeInMap "
    #   map_folder_path: '/path/to/save/the/map'
    #   pose_hint:
    #     position:
    #       x: x-position
    #       y: y-position
    #       z: z-position
    #     orientation:
    #       x: 0.0
    #       y: 0.0
    #       z: 0.0
    #       w: 1.0"
    # Construct the YAML payload as a single string
    yaml_request = (
        f'{{map_folder_path: "{map_path_string}", '
        'pose_hint: {'
        'position: {x: 0.0, y: 0.0, z: 0.0}, '
        'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}'
        '}}'
    )
    # load_map_trigger = TimerAction(
    #         period=1.5,
    #         actions=[
    #             ExecuteProcess(
    #                     cmd=[[
    #                         FindExecutable(name='ros2'),
    #                         f' action send_goal {namespace_string}/visual_slam/load_map_and_localize '
    #                         'isaac_ros_visual_slam_interfaces/action/LoadMapAndLocalize '
    #                         '"{map_url: ' + map_path_string + ', localize_near_point: {x: 0.0, y: 0.0, z: 0.0}}"'
    #                     ]],
    #                     shell=True,
    #                     condition=IfCondition(load_map)
    #             )
    #         ]
    # )  #
    load_map_trigger = TimerAction(
            period=1.5,
            actions=[
                ExecuteProcess(
                        cmd=[[
                            FindExecutable(name='ros2'),
                            f' service call {namespace_string}/visual_slam/localize_in_map '
                            'isaac_ros_visual_slam_interfaces/srv/LocalizeInMap '
                            f'"{yaml_request}"'
                        ]],
                        shell=True,
                        condition=IfCondition(load_map)
                )
            ]
    )

    visual_slam_launch_container = ComposableNodeContainer(
            name=component_container_name_string.split('/')[-1],
            namespace=namespace if use_namespace_string.lower() == 'true' else '',
            package='rclcpp_components',
            executable='component_container_mt',  # use component_container_mt for multithreading support
            condition=UnlessCondition(attach_to_shared_component_container),
            composable_node_descriptions=[visual_slam_node],  # if using this, disable load_composable_nodes
            #parameters=[{'thread_num': 4, 'use_sim_time': use_sim_time}],
            output='screen',
            #arguments=[
            #    '--use_multi_threaded_executor',
            #],
    )

    load_composable_nodes = LoadComposableNodes(
            condition=IfCondition(attach_to_shared_component_container),
            target_container=container_name_full,
            composable_node_descriptions=[visual_slam_node]
    )

    # Add launch arguments and nodes to the launch description
    ld = launch_args + [realsense_camera_node,
                        visual_slam_launch_container,
                        load_composable_nodes,
                        save_map_trigger,
                        load_map_trigger
                        ]
    return ld


def generate_launch_description():
    return LaunchDescription(
            [
                SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
                OpaqueFunction(function=launch_setup)
            ]
    )
