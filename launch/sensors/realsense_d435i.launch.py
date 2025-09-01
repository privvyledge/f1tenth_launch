#!/usr/bin/env python3
"""
Todo:
    * remap topics to a common interface, e.g left/image_rect, left/camera_info, right/image_rect, right/camera_info
"""
from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap, PushRosNamespace, LoadComposableNodes, ComposableNodeContainer, SetParameter
from launch_ros.actions import LifecycleNode
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, GroupAction, \
    OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import LogInfo
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.descriptions import ParameterFile, ParameterValue, ComposableNode
from nav2_common.launch import RewrittenYaml, ReplaceString


def launch_setup(context, *args, **kwargs):
    f1tenth_launch_dir = get_package_share_directory('f1tenth_launch')
    nvidia_isaac_launch_dir = os.path.join(f1tenth_launch_dir, 'launch', 'nvidia_isaac_ros')

    # Launch arguments
    realsense_config = os.path.join(
            get_package_share_directory('f1tenth_launch'),
            'config/sensors',
            'realsense_config.yaml')
    realsense_preset_path = os.path.join(
            get_package_share_directory('f1tenth_launch'),
            'config/sensors/realsense_presets',
            'HighAccuracyPreset.json'
    )

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    config_file = LaunchConfiguration('config_file')
    namespace = LaunchConfiguration('namespace', default='')
    use_namespace = LaunchConfiguration('use_namespace', default='False')
    camera_name = LaunchConfiguration('camera_name', default='camera')
    qos = LaunchConfiguration('qos', default='SENSOR_DATA')
    use_respawn = LaunchConfiguration('use_respawn')
    imu_only = LaunchConfiguration('imu_only', default='False')
    unite_imu_method = LaunchConfiguration('unite_imu_method', default=2)
    launch_imu_filter = LaunchConfiguration('launch_imu_filter')
    log_level = LaunchConfiguration('log_level')
    reset_realsense = LaunchConfiguration('reset_realsense')
    enable_pointcloud = LaunchConfiguration('enable_pointcloud')
    align_depth = LaunchConfiguration('align_depth')
    emitter_enabled = LaunchConfiguration('emitter_enabled')
    emitter_on_off = LaunchConfiguration('emitter_on_off')
    launch_realsense_splitter_node = LaunchConfiguration('launch_realsense_splitter_node', default=False)
    json_file_path = LaunchConfiguration('json_file_path', default=realsense_preset_path)
    use_composition = LaunchConfiguration('use_composition', default=True)
    component_container_name = LaunchConfiguration('component_container_name', default='realsense_d435i_container')
    attach_to_shared_component_container = LaunchConfiguration('attach_to_shared_component_container', default=False)
    intra_process_comms = LaunchConfiguration("intra_process_comms", default=True)

    use_sim_time_la = DeclareLaunchArgument(
            'use_sim_time', default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true')
    declare_namespace_cmd = DeclareLaunchArgument(
            'namespace',
            default_value=namespace,
            description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
            'use_namespace',
            default_value=use_namespace,
            description='Whether to apply a namespace to the navigation stack')

    declare_camera_name_cmd = DeclareLaunchArgument(
            'camera_name',
            default_value=camera_name,
            description='Name of the camera node')

    declare_qos_cmd = DeclareLaunchArgument(
            'qos',
            default_value=qos,
            description='Quality of Service setting for the camera node. '
                        'Options: SENSOR_DATA, DEFAULT, SYSTEM_DEFAULT, SENSORS, PARAMETERS, SERVICES, ACTIONS. '
                        'Recommended: SENSOR_DATA for camera nodes or SYSTEM_DEFAULT')

    declare_use_respawn_cmd = DeclareLaunchArgument(
            'use_respawn', default_value='True',
            description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
            'log_level', default_value='info',
            description='Realsense log level')

    realsense_params_file_cmd = DeclareLaunchArgument(
            'config_file',
            default_value=realsense_config,
            description='Full path to the realsense config file to use.')

    reset_realsense_la = DeclareLaunchArgument(
            'reset_realsense',
            default_value='False',
            description='Whether to reset the realsense device.')

    enable_pointcloud_la = DeclareLaunchArgument(
            'enable_pointcloud',
            default_value='False',
            description='Whether to publish PointClouds using librealsense SDK. '
                        'Could be disabled when recording ROSBags or mapping. ')

    align_depth_la = DeclareLaunchArgument(
            'align_depth',
            default_value='True',
            description='Whether to align the depth to other frames')

    emitter_enabled_la = DeclareLaunchArgument(
            'emitter_enabled',
            default_value='0',
            description='Whether to enable the IR emitters to improve depth and pointcloud quality. '
                        'Unfortunately, this renders the stereo IR cameras unusable for mapping, '
                        'VSLAM, VIO odometry, etc. '
                        'Disable when mapping or running VIO enable if accurate pointclouds are essential.')

    emitter_on_off_la = DeclareLaunchArgument(
            'emitter_on_off',
            default_value='False',
            description='Whether to alternate enabling/disabling the emitters. '
                        'This can be used to simultaneously '
                        'get accurate depth maps and pointclouds (when in the on state, i.e enabled) and '
                        'have usable IR images (when in the off state)')

    imu_only_cmd = DeclareLaunchArgument(
            'imu_only', default_value=imu_only,
            description='Whether to only launch the IMU module of the realsense camera.')

    unite_imu_method_cmd = DeclareLaunchArgument(
            'unite_imu_method', default_value=unite_imu_method,
            description='How to unify gyro and accel values into a single IMU topic. '
                        'Options: 0: no unified imu message, 1: copy, 2: linear interpolation.')

    launch_imu_filter_cmd = DeclareLaunchArgument(
            'launch_imu_filter', default_value='True',
            description='Whether to launch IMU filters for the realsense IMU.')

    launch_realsense_splitter_node_la = DeclareLaunchArgument(
            'launch_realsense_splitter_node', default_value=launch_realsense_splitter_node,
            description='Whether to launch the realsense splitter node.')

    json_file_path_cmd = DeclareLaunchArgument(
            'json_file_path', default_value=json_file_path,  # realsense_preset_path
            description='Path to the JSON file containing the realsense preset to use. '
                        'Default="" or . realsense_preset_path'
                        'Options: HighAccuracyPreset.json (recommended), MidDensityPreset.json, HighDensityPreset.json, '
                        'DefaultPreset_D435.json')

    use_composition_cmd = DeclareLaunchArgument('use_composition', default_value=use_composition,
                                               description='Whether to use composition for the realsense camera.')
    component_container_name_cmd = DeclareLaunchArgument('component_container_name', default_value=component_container_name,
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
        use_sim_time_la, declare_namespace_cmd, declare_use_namespace_cmd, declare_camera_name_cmd, declare_qos_cmd,
        declare_use_respawn_cmd, declare_log_level_cmd,
        realsense_params_file_cmd, reset_realsense_la, enable_pointcloud_la, align_depth_la,
        emitter_enabled_la, emitter_on_off_la,
        imu_only_cmd, unite_imu_method_cmd, launch_imu_filter_cmd,
        launch_realsense_splitter_node_la, json_file_path_cmd,
        use_composition_cmd, component_container_name_cmd, attach_to_shared_component_container_cmd,
        intra_process_comms_arg
    ]

    # Get name string from the LaunchConfiguration
    camera_name_str = camera_name.perform(context)
    namespace_str = namespace.perform(context)
    use_namespace_str = use_namespace.perform(context)
    imu_only_str = imu_only.perform(context)
    use_composition_str = use_composition.perform(context)
    component_container_name_str = component_container_name.perform(context)

    if camera_name_str != '':
        camera_name_str += '/'
    else:
        camera_name_str = 'camera/'

    container_name_full = component_container_name_str
    if use_composition_str.lower() == 'true':
        container_name_full = namespace_str + '/' + component_container_name_str

    common_realsense_parameters = {
        "use_sim_time": use_sim_time,
        "initial_reset": reset_realsense,
        "gyro_qos": qos,
        "accel_qos": qos,
        "gyro_info_qos": qos,
        "accel_info_qos": qos,
        "unite_imu_method": unite_imu_method,
    }

    realsense_camera_params = {
        "pointcloud.enable": enable_pointcloud,
        "align_depth.enable": align_depth,
        "depth_module.emitter_enabled": emitter_enabled,
        "depth_module.emitter_on_off": emitter_on_off,
        "color_qos": qos,
        "color_info_qos": qos,
        "infra_qos": qos,
        "infra1_qos": qos,
        "infra2_qos": qos,
        "depth_qos": qos,
        "depth_info_qos": qos,
        "pointcloud.pointcloud_qos": qos,
        "json_file_path": json_file_path
    }

    imu_only_params = {
        # Enable Gyro and Accel
        "enable_gyro": True,
        "enable_accel": True,
        # Disable all other modules
        "enable_sync": False,
        "enable_color": False,
        "enable_depth": False,
        "enable_infra1": False,
        "enable_infra2": False,
        "enable_infra": False,
        "pointcloud.enable": False,
        "colorizer.enable": False,
        "align_depth.enable": False,
        "enable_rgbd": False,
        "enable_confidence": False,
        "enable_pose": False,
    }

    if imu_only_str.lower() == 'true':
        realsense_params = common_realsense_parameters | imu_only_params
    else:
        realsense_params = common_realsense_parameters | realsense_camera_params

    # Setup nodes
    composed_realsense_group = GroupAction(
            condition=IfCondition(use_composition),
            actions=[
                PushRosNamespace(
                        condition=IfCondition(use_namespace),
                        namespace=namespace),
                SetRemap(src=['/tf'], dst=['tf']),
                SetRemap(src=['/tf_static'], dst=['tf_static']),
                SetParameter(name='thread_num', value='4'),
                Node(
                        name=component_container_name,
                        package='rclcpp_components',
                        # executables: 'component_container_mt', 'component_container_isolated', 'component_container'
                        executable='component_container_mt',
                        output='screen',
                        condition=UnlessCondition(attach_to_shared_component_container),
                        arguments=[
                            '--use_multi_threaded_executor',  # launch each component in a separate thread with component_container_isolated
                            '--ros-args', '--log-level', 'info'
                        ],
                ),
            ]
    )

    realsense_component = LoadComposableNodes(
                        condition=IfCondition(use_composition),
                        target_container=container_name_full,
                        composable_node_descriptions=[
                            # Realsense Driver Node Factory
                            ComposableNode(
                                    name=camera_name if camera_name_str else None,  # default: 'camera',
                                    namespace=namespace if (use_namespace_str.lower() == 'true' and namespace_str) else '',
                                    package='realsense2_camera',
                                    plugin='realsense2_camera::RealSenseNodeFactory',
                                    parameters=[
                                        config_file,
                                        realsense_params,
                                    ],
                                    remappings=[
                                        ('/tf', 'tf'),
                                        ('/tf_static', 'tf_static')
                                    ],
                                    extra_arguments=[{'use_intra_process_comms': intra_process_comms}]
                            )
                        ]
                )

    realsense_node = Node(
            condition=UnlessCondition(use_composition),
            package='realsense2_camera',
            # The realsense ROS package hardcoded 'camera' as a namespace if None is passed.
            # We override that behavior by specifying '' to force an empty namespace.
            namespace=namespace if (use_namespace_str.lower() == 'true' and namespace_str) else '',
            name=camera_name if camera_name_str else None,  # default: 'camera'
            executable='realsense2_camera_node',
            parameters=[
                config_file,
                realsense_params,
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ],
            arguments=['--ros-args', '--log-level', log_level],
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            emulate_tty=True,
    )

    realsense_splitter_launch_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    PathJoinSubstitution([nvidia_isaac_launch_dir, 'realsense_splitter.launch.py'])
            ),
            condition=IfCondition(launch_realsense_splitter_node),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_namespace': use_namespace,
                'namespace': namespace,  # namespace if (use_namespace_str.lower() == 'true' and namespace_str) else '',
                'camera_name': camera_name,
                'launch_realsense_driver': 'False',
                'depth_topic': camera_name_str + 'depth/image_rect_raw',
                'input_qos': qos,
                'output_qos': qos,
                'attach_to_shared_component_container': attach_to_shared_component_container,
                'component_container_name': component_container_name,  # container_name_full
                'intra_process_comms': intra_process_comms,
            }.items()
    )

    imu_filter_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [f1tenth_launch_dir, 'launch/filters', 'imu_filter.launch.py']
            )),
            condition=IfCondition([launch_imu_filter]),
            launch_arguments={
                'input_topic': camera_name_str + 'imu',
                'output_topic': camera_name_str + 'imu/filtered',
                'use_namespace': use_namespace,
                'namespace': namespace,  # namespace if (use_namespace_str.lower() == 'true' and namespace_str) else ''
                'remove_gravity_vector': 'False',  # True
                'imu_gyro_stddev': '0.1',
                'imu_accel_stddev': '0.1',
                'imu_orientation_stddev': '0.1',
                'node_name': 'realsense_imu_filter',
                'imu_corrector_output_topic': camera_name_str + 'imu/bias_removed',
                'use_madgwick_filter': 'True',
                'remove_imu_bias': 'False',  # disabled since its not really useful and requires Autoware installation
                # camera_imu_optical_frame, sensor_kit_link, base_link
                'imu_corrector_frame': 'camera_imu_optical_frame',
                'imu_corrector_node_name': 'realsense_imu_bias_removal_node',
                'use_sim_time': use_sim_time,
            }.items()
    )

    # Create the launch description and populate
    ld = launch_args + [
        realsense_node,
        composed_realsense_group,
        realsense_component,
        realsense_splitter_launch_include,
        imu_filter_node,
    ]

    return ld


def generate_launch_description():
    return LaunchDescription(
            [
                SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
                OpaqueFunction(function=launch_setup)
            ]
    )