"""
This nodes sets up local and global localization.
* Load map (yaml or posegraph)
* Launch AMCL (or slam_toolbox)
* (Optional) Launch IMU Filter
* (Optional) Launch Laser Filter
* Launch EKF node
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # Get path to files and directories
    nav2_pkg_prefix = get_package_share_directory('nav2_bringup')

    f1tenth_launch_pkg_prefix = get_package_share_directory('f1tenth_launch')

    # Load parameter files
    localization_param_file = os.path.join(
        f1tenth_launch_pkg_prefix, 'config', '/localization/localizer_amcl.yaml')
    localization_param = DeclareLaunchArgument(
            'localization_param_file',
            default_value=localization_param_file,
            description='Path to config file for localization nodes'
    )
    ekf_param_file = os.path.join(
            f1tenth_launch_pkg_prefix, 'config', '/ekf.yaml')

    imu_filter_param_file = os.path.join(
            f1tenth_launch_pkg_prefix, "config/imu_filter.yaml")
    laser_filter_param_file = os.path.join(
            f1tenth_launch_pkg_prefix, "config/filters/laser_filter.yaml")

    map_file_path = os.path.join(
            f1tenth_launch_pkg_prefix, 'data/maps', 'raslab.yaml')
    map_file = DeclareLaunchArgument(
            'map',
            default_value=map_file_path,
            description='Path to 2D map config file'
    )

    print(LaunchConfiguration("map_file"))

    # Include launch files (run map_server and AMCL nodes).
    #  Todo: load map separately so it can be loaded without AMCL
    nav2_localization = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    os.path.join(nav2_pkg_prefix,
                                 'launch/localization_launch.py')
            ),
            launch_arguments={
                'params_file': LaunchConfiguration('localization_param_file'),
                'map': LaunchConfiguration('map')
            }.items()
    )

    # Run nodes
    ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            namespace='vehicle',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[ekf_param_file],
    )

    # imu_filter_node = Node(
    #             package='imu_filter_madgwick',
    #             node_executable='imu_filter_madgwick_node',
    #             node_name='imu_filter',
    #             output='screen',
    #             parameters=[imu_filter_param_file],
    #             remappings=[('/imu/data_raw', '/sensors/imu/raw')]
    #         )

    imu_filter_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    os.path.join(f1tenth_launch_pkg_prefix,
                                 'launch/filters/imu_filter.launch.py')
            )
    )

    # laser_filter_node = Node(
    #         package="laser_filters",
    #         namespace='lidar',
    #         executable="scan_to_scan_filter_chain",
    #         parameters=[laser_filter_param_file],
    #         remappings=[
    #             ('output', 'scan'),
    #             ('scan', '/lidar/scan')
    #         ]
    # )

    laser_filter_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    os.path.join(f1tenth_launch_pkg_prefix,
                                 'launch/filters/laser_filter.launch.py')
            )
    )

    # Create Launch Description and add nodes to the launch description
    ld = LaunchDescription([
        localization_param,
        map_file,
        nav2_localization,
        ekf_node,
        # imu_filter_node,
        laser_filter_node,
    ])

    return ld
