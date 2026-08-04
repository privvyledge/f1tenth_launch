"""
Nav2 navigation stack — each server is individually toggleable via launch_<name>:=True/False.

Node roles:
  controller_server  – local trajectory tracker; runs a control plugin (RPP, DWB, TEB) to follow
                       the global plan. Publishes cmd_vel_nav. Disable and substitute a custom MPC
                       node that implements the nav2_msgs/action/FollowPath action interface.
  smoother_server    – post-processes global paths before handing them to the controller.
  planner_server     – global path planner (NavFn, Smac Hybrid-A*, Theta*); queries the global
                       costmap to produce a collision-free path to the goal. Disable and publish
                       your own path if using a custom planner.
  behavior_server    – recovery behaviors (spin, backup, wait, assisted teleop) invoked by
                       bt_navigator when the robot gets stuck.
  bt_navigator       – behavior-tree coordinator: drives planner → controller → recoveries.
                       Central orchestrator; disable only when replacing the entire Nav2 loop.
  waypoint_follower  – feeds a list of waypoints to bt_navigator sequentially.
  velocity_smoother  – rate-limits and smooths cmd_vel before it reaches the hardware driver.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, SetParameter
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    launch_controller_server = LaunchConfiguration('launch_controller_server')
    launch_smoother_server = LaunchConfiguration('launch_smoother_server')
    launch_planner_server = LaunchConfiguration('launch_planner_server')
    launch_behavior_server = LaunchConfiguration('launch_behavior_server')
    launch_bt_navigator = LaunchConfiguration('launch_bt_navigator')
    launch_waypoint_follower = LaunchConfiguration('launch_waypoint_follower')
    launch_velocity_smoother = LaunchConfiguration('launch_velocity_smoother')

    # Build lifecycle_nodes at runtime — lifecycle_manager must only list nodes that are
    # actually launched. Including a missing node causes the lifecycle manager to hang on
    # transition. Also guards against the ROS 2 Humble bug where an empty list crashes
    # evaluate_parameter_dict ([] is coerced to () and fails the scalar-type check).
    node_flags = [
        ('controller_server', launch_controller_server),
        ('smoother_server', launch_smoother_server),
        ('planner_server', launch_planner_server),
        ('behavior_server', launch_behavior_server),
        ('bt_navigator', launch_bt_navigator),
        ('waypoint_follower', launch_waypoint_follower),
        ('velocity_smoother', launch_velocity_smoother),
    ]
    lifecycle_nodes = [
        name for name, flag in node_flags
        if flag.perform(context).lower() == 'true'
    ]

    _ns_str = namespace.perform(context)
    node_ns = f'/{_ns_str}' if _ns_str else ''
    cmd_vel_topic_str = cmd_vel_topic.perform(context)

    # Costmap observation-source topics (e.g. lidar/scan_filtered) are resolved relative to
    # each costmap sub-node's namespace (/gosling1/local_costmap, /gosling1/global_costmap).
    # A bare relative name would become gosling1/local_costmap/lidar/scan_filtered — wrong.
    # We substitute <ns_prefix> with the absolute robot-namespace prefix so the topic resolves
    # correctly regardless of the costmap sub-node's own namespace depth:
    #   with namespace:    <ns_prefix>lidar/scan_filtered → /gosling1/lidar/scan_filtered
    #   without namespace: <ns_prefix>lidar/scan_filtered → /lidar/scan_filtered
    _ns_prefix = f'/{_ns_str}/' if _ns_str else '/'
    configured_params = ParameterFile(
            RewrittenYaml(
                    source_file=ReplaceString(
                            source_file=params_file,
                            replacements={'<ns_prefix>': _ns_prefix}),
                    root_key=namespace,
                    param_rewrites={
                        'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'map_subscribe_transient_local': map_subscribe_transient_local,
                    },
                    convert_types=True),
            allow_substs=True)

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # ── Non-composable path ──────────────────────────────────────────────────────────────────
    non_composable_actions = [
        SetParameter('use_sim_time', use_sim_time),
        # Runs a control plugin (RPP, DWB, TEB) to track the global plan; publishes cmd_vel_nav.
        # Replace with a custom node implementing nav2_msgs/action/FollowPath (e.g. an MPC node).
        Node(
                condition=IfCondition(launch_controller_server),
                namespace=node_ns,
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
        # Post-processes the global path before it reaches the controller.
        Node(
                condition=IfCondition(launch_smoother_server),
                namespace=node_ns,
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
        # Computes a collision-free global path using a costmap plugin (NavFn, Smac, Theta*).
        Node(
                condition=IfCondition(launch_planner_server),
                namespace=node_ns,
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
        # Executes recovery behaviors (spin, backup, wait) when the robot gets stuck.
        Node(
                condition=IfCondition(launch_behavior_server),
                namespace=node_ns,
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
        # Orchestrates navigation via a behavior tree: planner → controller → recoveries.
        Node(
                condition=IfCondition(launch_bt_navigator),
                namespace=node_ns,
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
        # Drives bt_navigator through a sequence of waypoints.
        Node(
                condition=IfCondition(launch_waypoint_follower),
                namespace=node_ns,
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
        # Rate-limits and smooths cmd_vel to prevent hardware acceleration spikes.
        Node(
                condition=IfCondition(launch_velocity_smoother),
                namespace=node_ns,
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [
                    ('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', cmd_vel_topic_str),
                ]),
    ]

    if lifecycle_nodes:
        non_composable_actions.append(
            Node(
                    namespace=node_ns,
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    arguments=['--ros-args', '--log-level', log_level],
                    parameters=[{'autostart': autostart, 'node_names': lifecycle_nodes}]))

    load_nodes = GroupAction(
            condition=IfCondition(PythonExpression(['not ', use_composition])),
            actions=non_composable_actions)

    # ── Composable path ──────────────────────────────────────────────────────────────────────
    # Use node_ns (absolute, '/gosling1'), not namespace (relative, 'gosling1'), exactly as the
    # non-composable Node entries above do. bringup.launch.py wraps this include in a
    # PushRosNamespace *and* passes namespace= down; a relative namespace gets the push prepended
    # to it, yielding /<ns>/<ns>/controller_server. The doubled name means nav2_params.yaml no
    # longer matches the node, controller_server configures with no critics ("No critics defined
    # for FollowPath"), throws in on_configure, and lifecycle_manager aborts the entire nav2
    # bringup. An absolute namespace is immune to PushRosNamespace, so this stays correct whether
    # nav2 is launched standalone or from a namespacing parent.
    composable_node_descriptions = []

    if launch_controller_server.perform(context).lower() == 'true':
        composable_node_descriptions.append(
            ComposableNode(
                    package='nav2_controller',
                    plugin='nav2_controller::ControllerServer',
                    name='controller_server',
                    namespace=node_ns,
                    parameters=[configured_params],
                    remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]))

    if launch_smoother_server.perform(context).lower() == 'true':
        composable_node_descriptions.append(
            ComposableNode(
                    package='nav2_smoother',
                    plugin='nav2_smoother::SmootherServer',
                    name='smoother_server',
                    namespace=node_ns,
                    parameters=[configured_params],
                    remappings=remappings))

    if launch_planner_server.perform(context).lower() == 'true':
        composable_node_descriptions.append(
            ComposableNode(
                    package='nav2_planner',
                    plugin='nav2_planner::PlannerServer',
                    name='planner_server',
                    namespace=node_ns,
                    parameters=[configured_params],
                    remappings=remappings))

    if launch_behavior_server.perform(context).lower() == 'true':
        composable_node_descriptions.append(
            ComposableNode(
                    package='nav2_behaviors',
                    plugin='behavior_server::BehaviorServer',
                    name='behavior_server',
                    namespace=node_ns,
                    parameters=[configured_params],
                    remappings=remappings))

    if launch_bt_navigator.perform(context).lower() == 'true':
        composable_node_descriptions.append(
            ComposableNode(
                    package='nav2_bt_navigator',
                    plugin='nav2_bt_navigator::BtNavigator',
                    name='bt_navigator',
                    namespace=node_ns,
                    parameters=[configured_params],
                    remappings=remappings))

    if launch_waypoint_follower.perform(context).lower() == 'true':
        composable_node_descriptions.append(
            ComposableNode(
                    package='nav2_waypoint_follower',
                    plugin='nav2_waypoint_follower::WaypointFollower',
                    name='waypoint_follower',
                    namespace=node_ns,
                    parameters=[configured_params],
                    remappings=remappings))

    if launch_velocity_smoother.perform(context).lower() == 'true':
        composable_node_descriptions.append(
            ComposableNode(
                    package='nav2_velocity_smoother',
                    plugin='nav2_velocity_smoother::VelocitySmoother',
                    name='velocity_smoother',
                    namespace=node_ns,
                    parameters=[configured_params],
                    remappings=remappings + [
                        ('cmd_vel', 'cmd_vel_nav'),
                        ('cmd_vel_smoothed', cmd_vel_topic_str),
                    ]))

    if lifecycle_nodes:
        composable_node_descriptions.append(
            ComposableNode(
                    package='nav2_lifecycle_manager',
                    plugin='nav2_lifecycle_manager::LifecycleManager',
                    name='lifecycle_manager_navigation',
                    namespace=node_ns,
                    parameters=[{'autostart': autostart, 'node_names': lifecycle_nodes}]))

    load_composable_nodes = GroupAction(
            condition=IfCondition(use_composition),
            actions=[
                SetParameter('use_sim_time', use_sim_time),
                LoadComposableNodes(
                        target_container=container_name,
                        composable_node_descriptions=composable_node_descriptions),
            ])

    return [load_nodes, load_composable_nodes]


def generate_launch_description():
    bringup_dir = get_package_share_directory('f1tenth_launch')

    ld = LaunchDescription()

    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    for arg in [
        DeclareLaunchArgument(
                'namespace', default_value='',
                description='Top-level namespace'),
        DeclareLaunchArgument(
                'use_sim_time', default_value='False',
                description='Use simulation (Gazebo) clock if True'),
        DeclareLaunchArgument(
                'autostart', default_value='True',
                description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument(
                'use_composition', default_value='False',
                description='Use composed bringup if True'),
        DeclareLaunchArgument(
                'container_name', default_value='nav2_container',
                description='Container to load composable nodes into'),
        DeclareLaunchArgument(
                'use_respawn', default_value='False',
                description='Respawn crashed nodes (non-composable path only)'),
        DeclareLaunchArgument(
                'log_level', default_value='info',
                description='Log level'),
        DeclareLaunchArgument(
                'params_file',
                default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
                description='Full path to the Nav2 parameters YAML file'),
        DeclareLaunchArgument(
                'cmd_vel_topic', default_value='cmd_vel',
                description='Final cmd_vel output topic from the velocity smoother. '
                            'Default: cmd_vel (drives hardware directly). '
                            'Set to e.g. cmd_vel_nav2 when running a custom controller '
                            'side-by-side so Nav2 output can be monitored without commanding hardware.'),
        DeclareLaunchArgument(
                'map_subscribe_transient_local', default_value='True',
                description='Set the map subscriber QoS to transient local'),
        # ── Per-node toggles ───────────────────────────────────────────────────────────────
        DeclareLaunchArgument(
                'launch_controller_server', default_value='True',
                description='Local trajectory tracker. Runs a control plugin (RPP, DWB, TEB) '
                            'to follow the global plan and publishes cmd_vel_nav. '
                            'Disable and replace with a custom node implementing the '
                            'nav2_msgs/action/FollowPath interface (e.g. an MPC controller).'),
        DeclareLaunchArgument(
                'launch_smoother_server', default_value='True',
                description='Path smoother. Post-processes global paths before the controller '
                            'tracks them. Safe to disable if your planner already outputs '
                            'smooth, controller-ready paths.'),
        DeclareLaunchArgument(
                'launch_planner_server', default_value='True',
                description='Global path planner (NavFn, Smac Hybrid-A*, Theta*). Queries the '
                            'global costmap to compute a collision-free path to the goal. '
                            'Disable and publish your own path if using a custom planner.'),
        DeclareLaunchArgument(
                'launch_behavior_server', default_value='True',
                description='Recovery behavior server. Executes spin, backup, wait, and '
                            'assisted-teleop behaviors when bt_navigator detects the robot is '
                            'stuck. Safe to disable when not using bt_navigator.'),
        DeclareLaunchArgument(
                'launch_bt_navigator', default_value='True',
                description='Behavior-tree navigator. Central coordinator that sequences '
                            'planner → controller → recoveries. Disable only when replacing '
                            'the entire Nav2 navigation loop with a custom implementation.'),
        DeclareLaunchArgument(
                'launch_waypoint_follower', default_value='True',
                description='Waypoint follower. Accepts a stamped list of waypoints and drives '
                            'bt_navigator to visit them in order. Disable when sending single '
                            'NavigateToPose goals directly without multi-waypoint sequences.'),
        DeclareLaunchArgument(
                'launch_velocity_smoother', default_value='True',
                description='Velocity smoother. Rate-limits and smooths cmd_vel before it '
                            'reaches the hardware driver to prevent acceleration spikes. '
                            'Disable if the controller or hardware driver already handles '
                            'velocity smoothing.'),
    ]:
        ld.add_action(arg)

    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld