"""
Todo: launch joystick in a separate package, i.e vehicle_bringup
Todo: pass arguments to mapper
Todo: load from ROSBAG
Todo: refer to https://github.com/ros-planning/navigation2/blob/humble/nav2_bringup/launch/slam_launch.py
Todo: add argument to select online/offline mapping
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, LaunchConfigurationEquals

import os


def IfEqualsCondition(arg_name: str, value: str):
    return IfCondition(PythonExpression([
        '"', LaunchConfiguration(arg_name), '" == "', value, '"'
    ]))


def generate_launch_description():
    # Parameters
    lifecycle_nodes = ['map_saver']
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = True
    save_map_timeout = 2000.0
    free_thresh_default = 0.25
    occupied_thresh_default = 0.65
    offline_mapping = LaunchConfiguration('offline_mapping')
    declare_offline_mapping = DeclareLaunchArgument(
            "offline_mapping",
            default_value='True',
            description="Path to config file for mapping nodes",
    )

    # package paths
    f1tenth_launch_pkg_prefix = get_package_share_directory('f1tenth_launch')
    slam_toolbox_pkg_prefix = get_package_share_directory('slam_toolbox')

    # parameters
    use_sim_time_la = DeclareLaunchArgument(
                'use_sim_time', default_value='True',
                description='Use simulation or ROSBAG clock if true'),

    offline_mapping_param_file = os.path.join(
        f1tenth_launch_pkg_prefix, "config/mapping/2d_mapping_offline.yaml"
    )
    offline_mapping_param = DeclareLaunchArgument(
        "offline_mapping_param_file",
        default_value=offline_mapping_param_file,
        description="Path to config file for mapping nodes",
    )

    online_mapping_param_file = os.path.join(
            f1tenth_launch_pkg_prefix, "config/mapping/2d_mapping_online.yaml"
    )
    online_mapping_param = DeclareLaunchArgument(
            "online_mapping_param_file",
            default_value=online_mapping_param_file,
            description="Path to config file for mapping nodes",
    )
    map_file_name = os.path.join(f1tenth_launch_pkg_prefix, "data/raslab.yaml")
    map_file_name_la = DeclareLaunchArgument('map_file_name', default_value=map_file_name,
                                             description="location to store the map periodically.")

    # with_joy_param = DeclareLaunchArgument(
    #     'with_joy',
    #     default_value='True',
    #     description='Launch joystick_interface in addition to other nodes'
    # )

    with_rviz_param = DeclareLaunchArgument(
        'with_rviz',
        default_value='False',
        description='Launch rviz in addition to other nodes'
    )

    vehicle_interface_mode = DeclareLaunchArgument(
        'vehicle_interface',
        default_value='vesc',
        description='Launch rviz in addition to other nodes'
    )

    rviz_cfg_path = os.path.join(f1tenth_launch_pkg_prefix,
                                 'rviz', 'slam_toolbox.rviz')
    rviz_cfg_path_param = DeclareLaunchArgument(
        'rviz_cfg_path_param',
        default_value=rviz_cfg_path,
        description='Launch RVIZ2 with the specified config file'
    )

    # nodes
    # vehicle_launch_svl = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(f1tenth_launch_pkg_prefix,
    #                      'launch/f1tenth_vehicle_svl.launch.py'),
    #     ),
    #     launch_arguments={
    #         'with_joy': LaunchConfiguration('with_joy'),
    #     }.items(),
    #     condition=IfEqualsCondition("vehicle_interface", "svl")
    # )

    vehicle_launch_vesc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_launch_pkg_prefix,
                         'launch/vehicle/vehicle.launch.py'),
        ),
        condition=IfCondition(PythonExpression(['not ', offline_mapping])),
        # condition=IfEqualsCondition("vehicle_interface", "vesc")
    )

    sensor_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    os.path.join(f1tenth_launch_pkg_prefix,
                                 'launch/sensors/sensors.launch.py'),
            ),
            condition=IfCondition(PythonExpression(['not ', offline_mapping])),
    )

    ekf_odom_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    os.path.join(f1tenth_launch_pkg_prefix,
                                 'launch/localization/ekf_odom.launch.py'),
            ),
            condition=IfCondition(PythonExpression(['not ', offline_mapping])),
    )

    # slam_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(slam_toolbox_pkg_prefix,
    #                      'launch/online_async_launch.py')
    #     ),
    #     launch_arguments={
    #         'params_file': LaunchConfiguration('offline_mapping_param_file'),
    #     }.items()
    # )

    offline_slam_launch = Node(
            condition=IfCondition([offline_mapping]),
            parameters=[
                offline_mapping_param_file,
                {'use_sim_time': use_sim_time}
            ],
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'
    )

    online_slam_launch = Node(
            condition=IfCondition(PythonExpression(['not ', offline_mapping])),
            parameters=[
                online_mapping_param_file,
                {'use_sim_time': use_sim_time}
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('with_rviz')),
        arguments=['-d', LaunchConfiguration("rviz_cfg_path_param")]
    )

    # Nodes launching commands
    # the map saver node doesn't work, might need to setup yaml path for map_server. Use rosservices instead, e.g:
    # ros2 run nav2_map_server map_saver_cli -f /f1tenth/data/maps/raslab
    # ros2 service call /slam_toolbox/serialize_map slam_toolbox/SerializePoseGraph "{filename: '/f1tenth/data/maps/raslab'}"
    start_map_saver_server_cmd = Node(
            package='nav2_map_server',
            executable='map_saver_server',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[
                {'map_topic': 'map'},
                {'map_url': map_file_name},
                {'save_map_timeout': save_map_timeout},
                {'free_thresh_default': free_thresh_default},
                {'occupied_thresh_default': occupied_thresh_default},
                {'map_subscribe_transient_local': True}])

    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    return LaunchDescription([
        use_sim_time_la,
        declare_offline_mapping,
        # with_joy_param,
        with_rviz_param,
        vehicle_interface_mode,
        offline_mapping_param,
        online_mapping_param,
        map_file_name_la,
        rviz_cfg_path_param,
        # vehicle_launch_svl,
        vehicle_launch_vesc,
        sensor_launch,
        ekf_odom_launch,
        offline_slam_launch,
        online_slam_launch,
        rviz2,
        start_map_saver_server_cmd,
        start_lifecycle_manager_cmd
    ])
