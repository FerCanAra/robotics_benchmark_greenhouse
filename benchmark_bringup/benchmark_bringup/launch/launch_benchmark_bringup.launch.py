# Generic ROS2 launch file with Nav2 integration
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration,)
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

mvsimDir = get_package_share_directory("mvsim")
benchmarkDir = get_package_share_directory("benchmark_bringup")

# FIX ARGUMENTS AND FILES
MVSIM_ROS2_PARAMS_FILE = os.path.join(mvsimDir, "mvsim_tutorial", "mvsim_ros2_params.yaml")
map_file = os.path.join(benchmarkDir, "maps", "greenhouse_test2.yaml")
# Additional world files for different scenarios
MVSIM_WORLD_FILE = os.path.join(benchmarkDir, "definitions/world", "greenhouse_control.world.xml")
MVSIM_WORLD_FILE_CHANGE_NO_SLOPE = os.path.join(benchmarkDir, "definitions/world", "greenhouse_control_change_no_slope.world.xml")
MVSIM_WORLD_FILE_NO_CHANGE_SLOPE = os.path.join(benchmarkDir, "definitions/world", "greenhouse_control_no_change_slope.world.xml")
MVSIM_WORLD_FILE_NO_CHANGE_NO_SLOPE = os.path.join(benchmarkDir, "definitions/world", "greenhouse_control_no_change_no_slope.world.xml")
NAV2_PARAMS_C1 = os.path.join(benchmarkDir, "params", "c1_greenhouse_params.yaml")
NAV2_PARAMS_C2 = os.path.join(benchmarkDir, "params", "c2_greenhouse_params.yaml")
NAV2_PARAMS_C3 = os.path.join(benchmarkDir, "params", "c3_greenhouse_params.yaml")
RVIZ2_FILE_C1 = os.path.join(benchmarkDir, "rviz", "greenhouse.rviz")
RVIZ2_FILE_C2 = os.path.join(benchmarkDir, "rviz", "greenhouse.rviz")
RVIZ2_FILE_C3 = os.path.join(benchmarkDir, "rviz", "greenhouse.rviz")



def generate_launch_description():

    ############################# DECLARE ARGUMENTS ##############################
    world_file_launch_arg = DeclareLaunchArgument("world_file",default_value=TextSubstitution(text=MVSIM_WORLD_FILE),description="Path to the *.world.xml file to load")
    headless_launch_arg = DeclareLaunchArgument("headless", default_value="False")
    do_fake_localization_arg = DeclareLaunchArgument("do_fake_localization", default_value="True")
    publish_tf_odom2baselink_arg = DeclareLaunchArgument("publish_tf_odom2baselink", default_value="True")
    force_publish_vehicle_namespace_arg = DeclareLaunchArgument("force_publish_vehicle_namespace", default_value="False")
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="True")
    rviz_config_file_arg = DeclareLaunchArgument("rviz_config_file", default_value=RVIZ2_FILE_C3)
    declare_params_file_cmd = DeclareLaunchArgument("params_file",default_value=NAV2_PARAMS_C3,description="Full path to the ROS2 parameters file to use for all launched nodes")
    declare_use_sim_time_cmd = DeclareLaunchArgument("use_sim_time",default_value="False",description="Use simulation (MVSim) clock if true")
    declare_autostart_cmd = DeclareLaunchArgument("autostart", default_value="True")
    declare_use_composition_cmd = DeclareLaunchArgument("use_composition", default_value="False")
    declare_use_respawn_cmd = DeclareLaunchArgument("use_respawn", default_value="False")
    declare_map_launch_arg = DeclareLaunchArgument("map", default_value=map_file, description="Full path to map yaml file to load")
    declare_slam_launch_arg = DeclareLaunchArgument("slam", default_value="False", description="Default value for SLAM")
    declare_category_arg = DeclareLaunchArgument("category", default_value="1")
    declare_payload_arg = DeclareLaunchArgument("payload", default_value="70")
    declare_terrain_slope_arg = DeclareLaunchArgument("terrain_slope", default_value="false")
    declare_change_terrain_arg = DeclareLaunchArgument("change_terrain", default_value="false")

    # -------- VALIDATION FUNCTION --------
    def validate_args(context):
        category = int(LaunchConfiguration("category").perform(context))
        payload = float(LaunchConfiguration("payload").perform(context))
        terrain_slope = LaunchConfiguration("terrain_slope").perform(context).lower()
        change_terrain = LaunchConfiguration("change_terrain").perform(context).lower()

        # Validate category
        if category not in (1, 2, 3):
            raise ValueError(
                f"[ERROR] category must be 1, 2, or 3. Received: {category}"
            )

        # Validate payload
        if not (0 <= payload <= 70):
            raise ValueError(
                f"[ERROR] payload must be between 0 and 70. Received: {payload}"
            )

        # Validate booleans
        if terrain_slope not in ("true", "false"):
            raise ValueError(
                f"[ERROR] terrain_slope must be true or false. Received: {terrain_slope}"
            )

        if change_terrain not in ("true", "false"):
            raise ValueError(
                f"[ERROR] change_terrain must be true or false. Received: {change_terrain}"
            )

        print("[OK] All custom parameters validated successfully.")

    validate_action = OpaqueFunction(function=validate_args)

    # -------- SELECT WORLD + OPTIONAL EXTRA NODES --------
    def select_nodes(context, *args, **kwargs):
        actions = []

        category = int(LaunchConfiguration("category").perform(context))
        terrain_slope = LaunchConfiguration("terrain_slope").perform(context).lower()
        change_terrain = LaunchConfiguration("change_terrain").perform(context).lower()

        if terrain_slope == "true" and change_terrain == "true":
            selected_world = MVSIM_WORLD_FILE
            # min_elevation = "-0.001"
            # max_elevation = "0.001"

        if terrain_slope == "false" and change_terrain == "true":
            selected_world = MVSIM_WORLD_FILE_CHANGE_NO_SLOPE
        
        if terrain_slope == "true" and change_terrain == "false":
            selected_world = MVSIM_WORLD_FILE_NO_CHANGE_SLOPE

        if terrain_slope == "false" and change_terrain == "false":
                selected_world = MVSIM_WORLD_FILE_NO_CHANGE_NO_SLOPE

        actions.append(SetLaunchConfiguration(name="world_file",value=selected_world))

        #############################
        # CATEGORY 1
        #  - rviz not used
        #  - 
        #############################
        if category == 1:
            selected_params_file = NAV2_PARAMS_C1
            actions.append(SetLaunchConfiguration(name="use_rviz",value="False"))
            actions.append(
                
                Node(
                    package="benchmark_bringup",
                    executable="velocity_cat1",
                    name="velocity_cat1",
                    output="screen"
                )
            )

        #############################
        # CATEGORY 2
        #############################
        elif category == 2:
            selected_params_file = NAV2_PARAMS_C2
            actions.append(
                Node(
                    package="benchmark_bringup",
                    executable="route_cat2",
                    name="route_cat2",
                    output="screen"
                )
            )

        #############################
        # CATEGORY 3
        #############################
        elif category == 3:
            selected_params_file = NAV2_PARAMS_C3
            actions.append(
                Node(
                    package="benchmark_bringup",
                    executable="route_cat3",
                    name="route_cat3",
                    output="screen"
                )
            )
        actions.append(SetLaunchConfiguration(name="params_file",value=selected_params_file))
        return actions

    selected_nodes = OpaqueFunction(function=select_nodes)

    ################################## NODES ####################################
    # mvsim node
    mvsim_node = Node(
        package="mvsim",
        executable="mvsim_node",
        name="mvsim",
        output="screen",
        parameters=[
            MVSIM_ROS2_PARAMS_FILE,
            {
                "world_file": LaunchConfiguration("world_file"),
                "headless": LaunchConfiguration("headless"),
                "do_fake_localization": LaunchConfiguration("do_fake_localization"),
                "publish_tf_odom2baselink": LaunchConfiguration("publish_tf_odom2baselink"),
                "force_publish_vehicle_namespace": LaunchConfiguration("force_publish_vehicle_namespace"),
            },
        # env={
        #     'ELEVATION_MIN': min_elevation,
        #     'ELEVATION_MAX': max_elevation,
        # }
        ],
    )

    # RViz2 node
    rviz2_node = Node(
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
    )

    # Nav2 bringup
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py"
            )
        ),
        launch_arguments={
            "slam": LaunchConfiguration("slam"),  # False - no SLAM
            "use_sim_time": LaunchConfiguration("use_sim_time"),  # False
            "params_file": LaunchConfiguration("params_file"),
            "map": LaunchConfiguration("map"),  # Cargar mapa existente
            "autostart": LaunchConfiguration("autostart"),
            "use_composition": LaunchConfiguration("use_composition"),
            "use_respawn": LaunchConfiguration("use_respawn"),
        }.items(),
    )

    ################################ LAUNCH DESCRIPTION ####################################
    ld = LaunchDescription()

    # Add common arguments
    ld.add_action(world_file_launch_arg)
    ld.add_action(headless_launch_arg)
    ld.add_action(do_fake_localization_arg)
    ld.add_action(publish_tf_odom2baselink_arg)
    ld.add_action(force_publish_vehicle_namespace_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(rviz_config_file_arg)
    ld.add_action(declare_category_arg)
    ld.add_action(declare_payload_arg)
    ld.add_action(declare_terrain_slope_arg)
    ld.add_action(declare_change_terrain_arg)

    # Validación de argumentos
    ld.add_action(validate_action)

    # Selección de mundo + nodos extra (SE EJECUTA ANTES QUE MVSIM)
    ld.add_action(selected_nodes)

    # Add Nav2 arguments
    ld.add_action(declare_map_launch_arg)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_slam_launch_arg)

    # Add nodes
    ld.add_action(mvsim_node)
    ld.add_action(rviz2_node)
    ld.add_action(bringup_cmd)
    # ld.add_action(pid_plotter_node)

    return ld
