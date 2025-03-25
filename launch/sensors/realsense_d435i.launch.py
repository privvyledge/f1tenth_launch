#!/usr/bin/env python3
"""
Todo:
    * use a container for realsense driver and imu
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, GroupAction, \
    OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import LogInfo
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.descriptions import ParameterFile, ParameterValue
from nav2_common.launch import RewrittenYaml, ReplaceString


def launch_setup(context, *args, **kwargs):
    f1tenth_launch_dir = get_package_share_directory('f1tenth_launch')
    nvidia_isaac_launch_dir = os.path.join(f1tenth_launch_dir, 'launch', 'nvidia_isaac_ros')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    config_file = LaunchConfiguration('config_file')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    camera_name = LaunchConfiguration('camera_name', default='camera')
    qos = LaunchConfiguration('qos', default='SENSOR_DATA')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    imu_only = LaunchConfiguration('imu_only')
    unite_imu_method = LaunchConfiguration('unite_imu_method')
    launch_imu_filter = LaunchConfiguration('launch_imu_filter')
    log_level = LaunchConfiguration('log_level')
    reset_realsense = LaunchConfiguration('reset_realsense')
    enable_pointcloud = LaunchConfiguration('enable_pointcloud')
    align_depth = LaunchConfiguration('align_depth')
    emitter_enabled = LaunchConfiguration('emitter_enabled')
    emitter_on_off = LaunchConfiguration('emitter_on_off')
    launch_realsense_splitter_node = LaunchConfiguration('launch_realsense_splitter_node', default=False)

    # Create a dictionary for substitutable parameters
    param_substitutions = {
        # 'unite_imu_method': unite_imu_method,
    }

    configured_params = ParameterFile(
            RewrittenYaml(
                    source_file=config_file,
                    root_key=namespace,
                    param_rewrites=param_substitutions,
                    convert_types=True),
            allow_substs=True)

    # Launch arguments
    realsense_config = os.path.join(
            get_package_share_directory('f1tenth_launch'),
            'config/sensors',
            'realsense_config.yaml')
    realsense_imu_config = os.path.join(
            get_package_share_directory('f1tenth_launch'),
            'config/sensors',
            'realsense_imu_config.yaml')

    use_sim_time_la = DeclareLaunchArgument(
            'use_sim_time', default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true')
    declare_namespace_cmd = DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
            'use_namespace',
            default_value='false',
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

    declare_autostart_cmd = DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack')

    declare_use_respawn_cmd = DeclareLaunchArgument(
            'use_respawn', default_value='False',
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
            default_value='True',
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

    realsense_imu_la = DeclareLaunchArgument('realsense_imu_config',
                                             default_value=realsense_imu_config,
                                             description='Path to the Realsense IMU parameters file to use.')

    imu_only_cmd = DeclareLaunchArgument(
            'imu_only', default_value='False',
            description='Whether to only launch the IMU module of the realsense camera.')

    launch_imu_filter_cmd = DeclareLaunchArgument(
            'launch_imu_filter', default_value='True',
            description='Whether to launch IMU filters for the realsense IMU.')

    launch_realsense_splitter_node_la = DeclareLaunchArgument(
            'launch_realsense_splitter_node', default_value=launch_realsense_splitter_node,
            description='Whether to launch the realsense splitter node.')

    # Create Launch Description
    launch_args = [
        use_sim_time_la, declare_namespace_cmd, declare_use_namespace_cmd, declare_camera_name_cmd, declare_qos_cmd,
        declare_autostart_cmd, declare_use_respawn_cmd, declare_log_level_cmd,
        realsense_params_file_cmd, reset_realsense_la, enable_pointcloud_la, align_depth_la,
        emitter_enabled_la, emitter_on_off_la,
        realsense_imu_la, imu_only_cmd, launch_imu_filter_cmd,
        launch_realsense_splitter_node_la
    ]

    # Get name string from the LaunchConfiguration
    camera_name_str = camera_name.perform(context)
    if camera_name_str != '':
        camera_name_str += '/'

    # Setup nodes
    realsense_node = Node(
            condition=IfCondition(PythonExpression(['not ', imu_only])),
            package='realsense2_camera',
            namespace=namespace,
            name=camera_name,
            executable='realsense2_camera_node',
            parameters=[
                configured_params,
                {
                    "use_sim_time": use_sim_time,
                    "initial_reset": reset_realsense,
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
                    "pointcloud.pointcloud_qos": qos,
                    "gyro_qos": qos,
                    "accel_qos": qos,
                 },
            ],
            arguments=['--ros-args', '--log-level', log_level],
            output='screen',
            respawn=True,
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
                'namespace': namespace,
                'camera_name': camera_name,
                'launch_realsense_driver': 'False',
                'depth_topic': camera_name_str + 'depth/image_rect_raw',
                'input_qos': qos,
                'output_qos': qos,
                'attach_to_shared_component_container': 'False',
                # 'component_container_name': 'realsense_container_name'
            }.items()
    )

    realsense_imu_node = Node(
            condition=IfCondition([imu_only]),
            package='realsense2_camera',
            # namespace=namespace,
            name=camera_name,
            executable='realsense2_camera_node',
            parameters=[
                LaunchConfiguration('realsense_imu_config'),
                {
                    "color_qos": qos,
                    "color_info_qos": qos,
                    "infra_qos": qos,
                    "infra1_qos": qos,
                    "infra2_qos": qos,
                    "depth_qos": qos,
                    "pointcloud.pointcloud_qos": qos,
                    "gyro_qos": qos,
                    "accel_qos": qos,
                },
            ],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            emulate_tty=True,
    )

    imu_filter_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [f1tenth_launch_dir, 'launch/filters', 'imu_filter.launch.py']
            )),
            condition=IfCondition([launch_imu_filter]),
            launch_arguments={
                'input_topic': camera_name_str + 'imu',
                'output_topic': camera_name_str + 'imu/filtered',
                'use_namespace': 'True',
                'namespace': namespace,
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
        LogInfo(
                condition=IfCondition(LaunchConfigurationEquals('log_level', 'debug')),
                msg=PythonExpression(['"Realsense node launched with camera name: "', camera_name_str])
        ),
        LogInfo(
                condition=IfCondition(LaunchConfigurationEquals('log_level', 'debug')),
                msg=PythonExpression(['"Realsense node launched with namespace: "', namespace])
        ),
        realsense_node,
        realsense_splitter_launch_include,
        realsense_imu_node,
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