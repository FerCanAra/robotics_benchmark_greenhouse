# Fernando - Launch corregido para ROS2 Humble

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # --- Directorios base ---
    mvsim_nav2_demo_dir = get_package_share_directory("mvsim_nav2_demos")
    launch_dir = os.path.join(mvsim_nav2_demo_dir, "launch")

    # --- Configuraciones ---
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    use_remappings = LaunchConfiguration("use_remappings")
    bt_xml_file = LaunchConfiguration("bt_xml_file")

    # --- Remapeos ---
    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
        ("/cmd_vel", "cmd_vel"),
        ("/scan", "scan"),
        ("/map", "map"),
    ]

    # --- Sustitución de parámetros ---
    param_substitutions = {"use_sim_time": use_sim_time}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    # --- Variables de entorno ---
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    # --- Argumentos declarados ---
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the navigation stack",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (MVSim/Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(mvsim_nav2_demo_dir, "params", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="Log level"
    )

    declare_use_remappings_cmd = DeclareLaunchArgument(
        "use_remappings",
        default_value="true",
        description="Apply remappings to all Nav2 nodes",
    )

    declare_bt_xml_cmd = DeclareLaunchArgument(
        "bt_xml_file",
        default_value=os.path.join(
            get_package_share_directory("nav2_bt_navigator"),
            "behavior_trees",
            "nav2_bt_navigator/navigate_through_poses_w_replanning_and_recovery.xml",
        ),
        description="Full path to Behavior Tree XML file to use",
    )

    # --- Inclusión de los lanzadores de navegación y localización ---
    start_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "mvsim_mpc_nav2_localization_launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": params_file,
            "use_lifecycle_mgr": "false",
            "use_remappings": use_remappings,
        }.items(),
    )

    # start_navigation_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_dir, "mvsim_mpc_nav2_navigation_launch.py")
    #     ),
    #     launch_arguments={
    #         "namespace": namespace,
    #         "use_sim_time": use_sim_time,
    #         "autostart": autostart,
    #         "params_file": params_file,
    #         "bt_xml_file": bt_xml_file,
    #         "use_lifecycle_mgr": "false",
    #         "use_remappings": use_remappings,
    #         "map_subscribe_transient_local": "true",
    #     }.items(),
    # )

    # --- Contenedor Nav2 con TEB incluido ---
    bringup_cmd_group = GroupAction(
        [
            Node(
                condition=IfCondition(use_composition),
                name="nav2_container",
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[configured_params, {"autostart": autostart}],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
                output="screen",
            ),
            start_localization_cmd,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "mvsim_nav2_navigation_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": "nav2_container",
                }.items(),
            ),
        ]
    )

    # --- Descripción del lanzamiento ---
    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_use_remappings_cmd)

    ld.add_action(bringup_cmd_group)

    return ld
