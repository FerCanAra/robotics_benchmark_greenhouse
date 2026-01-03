# Fernando

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('mvsim_nav2_demos')

    # Parámetros principales
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_lifecycle_mgr = LaunchConfiguration('use_lifecycle_mgr')
    use_remappings = LaunchConfiguration('use_remappings')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    container_name_full = (namespace, '/', container_name)

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother'
    ]

    # Remapeos estándar
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/scan', 'scan'),
        ('/cmd_vel', 'cmd_vel'),
        ('/map', 'map'),
        ('/goal_pose', 'goal_pose')
    ]

    # Sustituciones en parámetros
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'bt_xml_filename': bt_xml_file,
        'map_subscribe_transient_local': map_subscribe_transient_local
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # Variables de entorno
    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # Declaraciones de argumentos
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (MVSim) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'bt_xml_file',
        default_value=os.path.join(bringup_dir, 'behavior_trees', 'nav2_bt_navigator/navigate_through_poses_w_replanning_and_recovery.xml'),
        description='Full path to behavior tree XML file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='Name of container that nodes will load in if composition is used')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='Log level')

    declare_use_lifecycle_mgr_cmd = DeclareLaunchArgument(
        'use_lifecycle_mgr', default_value='true',
        description='Whether to launch lifecycle manager')

    declare_use_remappings_cmd = DeclareLaunchArgument(
        'use_remappings', default_value='true',
        description='Whether to apply remappings')

    declare_map_subscribe_cmd = DeclareLaunchArgument(
        'map_subscribe_transient_local', default_value='false',
        description='Use transient local QoS for map subscription')

    # Grupo de nodos sin composición
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            # Controller server (TEB)
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings,
                arguments=['--ros-args', '--log-level', log_level]),

            # Smoother
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),

            # Planner server
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),

            # Behavior server
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),

            # BT Navigator
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),

            # Waypoint follower
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[configured_params],
                remappings=remappings),

            # Velocity smoother
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[configured_params],
                remappings=remappings +
                    [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),

            # Lifecycle manager
            Node(
                condition=IfCondition(use_lifecycle_mgr),
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': lifecycle_nodes
                }]
            ),
        ]
    )

    # Grupo de nodos compuestos
    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings +
                    [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        ],
    )

    # Lanzamiento final
    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_lifecycle_mgr_cmd)
    ld.add_action(declare_use_remappings_cmd)
    ld.add_action(declare_map_subscribe_cmd)
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
