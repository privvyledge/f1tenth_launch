"""
3D Mapping using RTABMap. Either maps using Stereo or RGBD

See:
    * https://github.com/introlab/rtabmap_ros/blob/humble-devel/rtabmap_examples/launch/
    * http://wiki.ros.org/rtabmap_ros/Tutorials/StereoHandHeldMapping#Bring-up_example_with_RealSense_cameras
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
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
    f1tenth_launch_dir = get_package_share_directory('f1tenth_launch')
    rtabmap_launch_dir = get_package_share_directory('rtabmap_examples')

    # Create the launch configuration variables
    mapping_mode = LaunchConfiguration('mapping_mode')

    # Launch arguments
    declare_mapping_mode = DeclareLaunchArgument(
            'mapping_mode',
            default_value='color',
            description='What rtabmap mode to use for the Realsense camera. color, infra, stereo. Default: color')

    ld = LaunchDescription([declare_mapping_mode])

    # Setup nodes
    '''
    Runs mapping in RGBD mode.
    ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 
    enable_sync:=true
    '''
    color_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [rtabmap_launch_dir, 'launch', 'realsense_d435i_color.launch.py']
            )),
            condition=LaunchConfigurationEquals('mapping_mode', 'color'),
    )

    '''
    Runs mapping in IR-Depth mode.
    ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 
    enable_infra1:=true enable_infra2:=true enable_sync:=true
    
    ros2 param set /camera/camera depth_module.emitter_enabled 0
    '''
    infra_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [rtabmap_launch_dir, 'launch', 'realsense_d435i_infra.launch.py']
            )),
            condition=LaunchConfigurationEquals('mapping_mode', 'infra'),
    )

    '''
    Runs mapping in Stereo-IR mode.
    ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 
    enable_infra1:=true enable_infra2:=true enable_sync:=true

    ros2 param set /camera/camera depth_module.emitter_enabled 0
    '''
    stereo_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [rtabmap_launch_dir, 'launch', 'realsense_d435i_stereo.launch.py']
            )),
            condition=LaunchConfigurationEquals('mapping_mode', 'stereo'),
    )

    ld.add_action(color_node)
    ld.add_action(infra_node)
    ld.add_action(stereo_node)

    return ld
