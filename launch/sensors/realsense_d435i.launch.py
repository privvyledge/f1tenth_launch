#!/usr/bin/env python3
"""
Todo: launch realsense URDF/Xacro with robot state publisher using _d435i.urdf.xacro or test_d435i_camera.urdf.xacro (https://navigation.ros.org/setup_guides/urdf/setup_urdf.html  | https://github.com/IntelRealSense/realsense-ros/blob/ros2-development/realsense2_description/launch/view_model.launch.py)
Todo: don't hardcode namespace
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
from launch_ros.descriptions import ParameterFile, ParameterValue
from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
    f1tenth_launch_dir = get_package_share_directory('f1tenth_launch')

    # Create the launch configuration variables
    config_file = LaunchConfiguration('config_file')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    imu_only = LaunchConfiguration('imu_only')
    unite_imu_method = LaunchConfiguration('unite_imu_method')
    launch_imu_filter = LaunchConfiguration('launch_imu_filter')

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

    declare_namespace_cmd = DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
            'use_namespace',
            default_value='false',
            description='Whether to apply a namespace to the navigation stack')

    declare_autostart_cmd = DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack')

    declare_use_respawn_cmd = DeclareLaunchArgument(
            'use_respawn', default_value='False',
            description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    realsense_params_file_cmd = DeclareLaunchArgument(
            'config_file',
            default_value=realsense_config,
            description='Full path to the realsense config file to use.')

    realsense_imu_la = DeclareLaunchArgument('realsense_imu_config',
                                             default_value=realsense_imu_config,
                                             description='Path to the Realsense IMU parameters file to use.')

    imu_only_cmd = DeclareLaunchArgument(
            'imu_only', default_value='False',
            description='Whether to only launch the IMU module of the realsense camera.')

    launch_imu_filter_cmd = DeclareLaunchArgument(
            'launch_imu_filter', default_value='True',
            description='Whether to launch IMU filters for the realsense IMU.')

    # Create Launch Description
    ld = LaunchDescription([declare_namespace_cmd, declare_use_namespace_cmd,
                            declare_autostart_cmd, declare_use_respawn_cmd,
                            realsense_params_file_cmd, realsense_imu_la,
                            imu_only_cmd, launch_imu_filter_cmd])

    # Setup nodes
    # realsense_node = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(PathJoinSubstitution(
    #                 [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
    #         )),
    #         condition=LaunchConfigurationEquals('sensor', 'realsense'),
    #         launch_arguments={
    #             'pointcloud.enable': 'true',
    #             'ordered_pc': 'true',
    #             'initial_reset': 'true'
    #         }.items()
    # )

    realsense_node = Node(
            condition=IfCondition(PythonExpression(['not ', imu_only])),
            package='realsense2_camera',
            namespace='camera',
            name='camera',
            executable='realsense2_camera_node',
            parameters=[configured_params, {"pointcloud.enable": True}],
            output='screen',
            emulate_tty=True,
    )

    realsense_imu_node = Node(
            condition=IfCondition([imu_only]),
            package='realsense2_camera',
            # namespace='sensors/camera',
            name='camera',
            executable='realsense2_camera_node',
            parameters=[LaunchConfiguration('realsense_imu_config')],
            output='screen',
            emulate_tty=True,
    )

    imu_filter_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [f1tenth_launch_dir, 'launch/filters', 'imu_filter.launch.py']
            )),
            condition=IfCondition([launch_imu_filter]),
            launch_arguments={
                'input_topic': '/camera/imu',
                'output_topic': '/camera/imu/filtered',
                'remove_gravity_vector': 'True',
                'node_name': 'realsense_imu_filter',
                'use_madgwick_filter': 'False',
            }.items()
    )

    ld.add_action(realsense_node)
    ld.add_action(realsense_imu_node)
    ld.add_action(imu_filter_node)

    return ld