# +-------------------------------------------------------------------------+
# |                       Benchmark control simulator                       |
# |                                                                         |
# | Copyright (C) 2025  Fernando Cañadas Aránega                            |
# | PhD Student University of Almería, Spain                                |
# | Contact: fernando.ca@ual.es                                             |
# | Distributed under 3-clause BSD License                                  |
# | See COPYING                                                             |
# | Main launch for Benchmark control simulatoring                          |
# +-------------------------------------------------------------------------+

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
    RegisterEventHandler,
    EmitEvent,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml
import yaml
import os

mvsimDir = get_package_share_directory("mvsim")
benchmarkDir = get_package_share_directory("benchmark_bringup")

# ---------------------------------------------------------------------------
#                            FIX PATHS TO FILES
# ---------------------------------------------------------------------------
MVSIM_ROS2_PARAMS_FILE = os.path.join(
    mvsimDir, "mvsim_tutorial", "mvsim_ros2_params.yaml"
)  # mvsim parameters file
MVSIM_WORLD_FILE = os.path.join(
    benchmarkDir, "definitions/world", "greenhouse_control.world.xml"
)  # default world file
MAP_FILE = os.path.join(
    benchmarkDir, "maps", "greenhouse_benchmark.yaml"
)  # Default slam map file
RVIZ2_FILE_C2 = os.path.join(
    benchmarkDir, "rviz", "c2_greenhouse.rviz"
)  # Default rviz2 config file for category 2
RVIZ2_FILE_C3 = os.path.join(
    benchmarkDir, "rviz", "c3_greenhouse.rviz"
)  # Default rviz2 config file for category 3

PID_MPC_PARAMS = os.path.join(
    benchmarkDir, "control_params", "c1_pid_params.yaml"
)  # c1 parameters file

TEB_MPC_PARAMS = os.path.join(
    benchmarkDir, "control_params", "c2_mpc_teb_params.yaml"
)  # c2 parameters file
THETA_STAR_PARAMS = os.path.join(
    benchmarkDir, "control_params", "c3_theta_star_params.yaml"
)  # c2 and c3 parameters file
# Mid-level controllers that can be benchmarked (launch argument controller:=...).
# For each one: FollowPath parameters file, and the path published by the controller
# that is used as reference of the mid-level prediction/tracking error (SAE2):
#  - predictive controllers (TEB, DWB): the trajectory they plan,
#  - path followers (RPP, MPPI in Humble): the portion of the global path they follow.
CONTROLLERS = {
    "teb": {"params": None, "local_path_topic": "/local_plan"},  # TEB file of each category
    "mppi": {"params": os.path.join(benchmarkDir, "control_params", "controller_mppi.yaml"),
             "local_path_topic": "/transformed_global_plan"},
    "rpp": {"params": os.path.join(benchmarkDir, "control_params", "controller_rpp.yaml"),
            "local_path_topic": "/received_global_plan"},
    "dwb": {"params": os.path.join(benchmarkDir, "control_params", "controller_dwb.yaml"),
            "local_path_topic": "/local_plan"},
}

# Angular limits of every mid-level controller, set from the launch arguments w_max
# [rad/s] and w_acc_max [rad/s^2] so that all of them are compared with the same limits
# (and SCI2 is normalised with the same w_max).
ANGULAR_LIMIT_KEYS = {
    "teb": {"max_vel_theta": "w", "acc_lim_theta": "a"},
    "mppi": {"wz_max": "w"},
    "dwb": {"max_vel_theta": "w", "acc_lim_theta": "a", "decel_lim_theta": "-a"},
    # RPP runs without rotate-to-heading (see controller_rpp.yaml), so its angular
    # limits are only kept for completeness; the common w_acc_max is applied by the
    # velocity smoother, as for the other controllers.
    "rpp": {"rotate_to_heading_angular_vel": "w", "max_angular_accel": "a"},
}

# Global planners of category 3 (launch argument planner:=...)
PLANNERS = {
    "theta": os.path.join(benchmarkDir, "control_params", "planner_theta.yaml"),
    "smac": os.path.join(benchmarkDir, "control_params", "planner_smac.yaml"),
    "navfn": os.path.join(benchmarkDir, "control_params", "planner_navfn.yaml"),
}

NAV2_PARAMS_C2 = os.path.join(
    benchmarkDir, "configuration", "c2_nav2_config.yaml"
)  # c2 nav2 config file
NAV2_PARAMS_C3 = os.path.join(
    benchmarkDir, "configuration", "c3_nav2_config.yaml"
)  # c3 nav2 config file


def generate_launch_description():
    # ---------------------------------------------------------------------------
    #                            DECLARE ARGUMENTS
    # ---------------------------------------------------------------------------

    # mvsim arguments
    world_file_launch_arg = DeclareLaunchArgument(
        "world_file",
        default_value=TextSubstitution(text=MVSIM_WORLD_FILE),
        description="Path to the *.world.xml file to load",
    )
    headless_launch_arg = DeclareLaunchArgument("headless", default_value="False")
    do_fake_localization_arg = DeclareLaunchArgument(
        "do_fake_localization", default_value="True"
    )
    publish_tf_odom2baselink_arg = DeclareLaunchArgument(
        "publish_tf_odom2baselink", default_value="True"
    )
    force_publish_vehicle_namespace_arg = DeclareLaunchArgument(
        "force_publish_vehicle_namespace", default_value="False"
    )
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="True")
    use_nav2_arg = DeclareLaunchArgument("use_nav2", default_value="True")
    rviz_config_file_arg = DeclareLaunchArgument(
        "rviz_config_file", default_value=RVIZ2_FILE_C2
    )
    # Nav2 arguments
    declare_slam_launch_arg = DeclareLaunchArgument(
        "slam", default_value="False", description="Default value for SLAM"
    )
    declare_map_launch_arg = DeclareLaunchArgument(
        "map", default_value=MAP_FILE, description="Full path to map yaml file to load"
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (MVSim) clock if true",
    )
    declare_autostart_cmd = DeclareLaunchArgument("autostart", default_value="True")
    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition", default_value="False"
    )
    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn", default_value="False"
    )
    # Params configuration files
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=NAV2_PARAMS_C3,
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )
    declare_params_file_teb = DeclareLaunchArgument(
        "params_file_teb",
        default_value=TEB_MPC_PARAMS,
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )
    declare_params_file_theta = DeclareLaunchArgument(
        "params_file_theta",
        default_value=THETA_STAR_PARAMS,
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )
    # Arguments for benchmark
    declare_category_arg = DeclareLaunchArgument("category", default_value="1")
    declare_payload_arg = DeclareLaunchArgument("payload", default_value="0")
    declare_terrain_slope_arg = DeclareLaunchArgument(
        "terrain_slope", default_value="false"
    )
    declare_change_terrain_arg = DeclareLaunchArgument(
        "change_terrain", default_value="false"
    )
    declare_localization_arg = DeclareLaunchArgument(
        "localization",
        default_value="fake",
        description="Localization for categories 2-3: 'fake' (MVSim ground truth) "
        "or 'amcl' (AMCL with the 2D LiDAR). Only one of them publishes map->odom.",
    )
    declare_controller_arg = DeclareLaunchArgument(
        "controller",
        default_value="teb",
        description="Mid-level controller (categories 2-3): " + ", ".join(CONTROLLERS),
    )
    declare_w_max_arg = DeclareLaunchArgument(
        "w_max",
        default_value="1.0",
        description="Max. angular velocity [rad/s] of the mid-level controller (all controllers)",
    )
    declare_w_acc_max_arg = DeclareLaunchArgument(
        "w_acc_max",
        default_value="1.0",
        description="Max. angular acceleration [rad/s^2] of the mid-level controller (all controllers). "
                    "1.0 = yaw acceleration actually achieved by the simulated robot (3.2 caused a limit cycle)",
    )
    declare_planner_arg = DeclareLaunchArgument(
        "planner",
        default_value="theta",
        description="Global planner (category 3): " + ", ".join(PLANNERS),
    )
    declare_local_path_topic_arg = DeclareLaunchArgument(
        "local_path_topic",
        default_value="",
        description="nav_msgs/Path used as mid-level reference (default: the one of the controller)",
    )
    declare_seed_arg = DeclareLaunchArgument(
        "seed",
        default_value="-1",
        description="Seed of the sensor-noise generators (-1 = random seed)",
    )
    declare_results_dir_arg = DeclareLaunchArgument(
        "results_dir",
        default_value="result",
        description="Directory where the results of each run are stored",
    )
    declare_run_name_arg = DeclareLaunchArgument(
        "run_name",
        default_value="",
        description="Name of the results folder of this run (empty = timestamp)",
    )
    declare_plot_arg = DeclareLaunchArgument(
        "plot",
        default_value="True",
        description="Show the real-time plots (set False for batch runs)",
    )
    declare_auto_shutdown_arg = DeclareLaunchArgument(
        "auto_shutdown",
        default_value="False",
        description="Shut down the whole launch when the run finishes (batch runs)",
    )

    # ---------------------------------------------------------------------------
    #                            VALIDATE ARGUMENTS
    # ---------------------------------------------------------------------------
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
        localization = LaunchConfiguration("localization").perform(context).lower()
        if localization not in ("fake", "amcl"):
            raise ValueError(
                f"[ERROR] localization must be 'fake' or 'amcl'. Received: {localization}"
            )
        if change_terrain not in ("true", "false"):
            raise ValueError(
                f"[ERROR] change_terrain must be true or false. Received: {change_terrain}"
            )

        controller = LaunchConfiguration("controller").perform(context).lower()
        if controller not in CONTROLLERS:
            raise ValueError(
                f"[ERROR] controller must be one of {list(CONTROLLERS)}. Received: {controller}"
            )

        planner = LaunchConfiguration("planner").perform(context).lower()
        if planner not in PLANNERS:
            raise ValueError(f"[ERROR] planner must be one of {list(PLANNERS)}. Received: {planner}")

        print("[OK] All custom parameters validated successfully.")

    validate_action = OpaqueFunction(function=validate_args)

    # ---------------------------------------------------------------------------
    #                         SELECTION CATEGORY NODES
    # ---------------------------------------------------------------------------
    def select_nodes(context, *args, **kwargs):
        actions = []

        category = int(LaunchConfiguration("category").perform(context))
        terrain_slope = LaunchConfiguration("terrain_slope").perform(context).lower()
        change_terrain = LaunchConfiguration("change_terrain").perform(context).lower()
        localization = LaunchConfiguration("localization").perform(context).lower()
        controller = LaunchConfiguration("controller").perform(context).lower()
        local_path_topic = (LaunchConfiguration("local_path_topic").perform(context)
                            or CONTROLLERS.get(controller, CONTROLLERS["teb"])["local_path_topic"])
        auto_shutdown = (
            LaunchConfiguration("auto_shutdown").perform(context).lower() == "true"
        )

        # Only one source publishes map->odom: MVSim (fake localization) or AMCL.
        # Category 1 does not use Nav2, so the MVSim fake localization is kept.
        use_fake = category == 1 or localization == "fake"
        actions.append(
            SetLaunchConfiguration("do_fake_localization", str(use_fake))
        )

        # MVSim publishes its logger columns as std_msgs/Float64 topics (one message
        # per log row), under the vehicle namespace only if it is forced:
        force_ns = LaunchConfiguration("force_publish_vehicle_namespace").perform(context)
        log_topic_prefix = "/AGI/log" if force_ns.lower() == "true" else "/log"

        # Motor torque saturation (the CTRL_* arguments are declared after this function)
        with open(PID_MPC_PARAMS) as f:
            max_torque_default = yaml.safe_load(f)["MotorSim"]["max_torque"]
        max_torque = float(
            LaunchConfiguration(
                "CTRL_MOTORSIM_MAX_TORQUE", default=str(max_torque_default)
            ).perform(context)
        )

        # Parameters shared by all the plotters (results logging)
        plotter_params = [
            {
                "plot": LaunchConfiguration("plot"),
                "auto_shutdown": LaunchConfiguration("auto_shutdown"),
                "results_dir": LaunchConfiguration("results_dir"),
                "run_name": LaunchConfiguration("run_name"),
                "seed": LaunchConfiguration("seed"),
                "payload": LaunchConfiguration("payload"),
                "terrain_slope": LaunchConfiguration("terrain_slope"),
                "change_terrain": LaunchConfiguration("change_terrain"),
                "localization": "fake" if use_fake else "amcl",
                "controller": controller,
                "planner": LaunchConfiguration("planner").perform(context).lower() if category == 3 else "",
                "local_path_topic": local_path_topic,
                "w_max": float(LaunchConfiguration("w_max").perform(context)),
                "max_torque": max_torque,
                "log_topic_prefix": log_topic_prefix,
            }
        ]

        def add_plotter(executable):
            plotter = Node(
                package="benchmark_bringup",
                executable=executable,
                name=executable,
                output="screen",
                parameters=plotter_params,
            )
            actions.append(plotter)
            if auto_shutdown:
                actions.append(
                    RegisterEventHandler(
                        OnProcessExit(
                            target_action=plotter,
                            on_exit=[EmitEvent(event=Shutdown(reason="run finished"))],
                        )
                    )
                )

        if terrain_slope == "true" and change_terrain == "true":
            actions.append(SetLaunchConfiguration("min_terrain", "-0.3"))
            actions.append(SetLaunchConfiguration("max_terrain", "0.3"))
            actions.append(SetLaunchConfiguration("adaptative_friction", "true"))

        elif terrain_slope == "false" and change_terrain == "true":
            actions.append(SetLaunchConfiguration("min_terrain", "-0.00001"))
            actions.append(SetLaunchConfiguration("max_terrain", "0.00001"))
            actions.append(SetLaunchConfiguration("adaptative_friction", "true"))

        elif terrain_slope == "true" and change_terrain == "false":
            actions.append(SetLaunchConfiguration("min_terrain", "-0.3"))
            actions.append(SetLaunchConfiguration("max_terrain", "0.3"))
            actions.append(SetLaunchConfiguration("adaptative_friction", "false"))

        else:  # terrain_slope == "false" and change_terrain == "false"
            actions.append(SetLaunchConfiguration("min_terrain", "-0.00001"))
            actions.append(SetLaunchConfiguration("max_terrain", "0.00001"))
            actions.append(SetLaunchConfiguration("adaptative_friction", "false"))

        #############################
        # CATEGORY 1
        #############################
        if category == 1:
            actions.append(SetLaunchConfiguration(name="use_rviz", value="False"))
            actions.append(SetLaunchConfiguration(name="use_nav2", value="False"))
            actions.append(
                Node(
                    package="benchmark_bringup",
                    executable="velocity_cat1",
                    name="velocity_cat1",
                    output="screen",
                    parameters=[{"log_topic_prefix": log_topic_prefix}],
                )
            )
            add_plotter("plotter_c1_benchmark")

        #############################
        # CATEGORY 2
        #############################
        elif category == 2:
            if use_fake:
                # No map needed for category 2 with the MVSim fake localization
                actions.append(SetLaunchConfiguration("map", ""))
            actions.append(
                Node(
                    package="benchmark_bringup",
                    executable="route_cat2",
                    name="route_cat2",
                    output="screen",
                )
            )
            add_plotter("plotter_c2_benchmark")

        #############################
        # CATEGORY 3
        #############################
        elif category == 3:
            actions.append(SetLaunchConfiguration("rviz_config_file", RVIZ2_FILE_C3))
            actions.append(
                Node(
                    package="benchmark_bringup",
                    executable="route_cat3",
                    name="route_cat3",
                    output="screen",
                )
            )
            add_plotter("plotter_c3_benchmark")

        return actions

    selected_nodes = OpaqueFunction(function=select_nodes)

    # -------------------------------------------------------------------------
    # Nav2 param rewrite
    # -------------------------------------------------------------------------
    def generate_nav2_params(context):
        """Nav2 parameters of the run: base file of the category + FollowPath of the
        selected controller (+ GridBased planner of category 3, planner:=...) + localization.
        The merged file is written to /tmp (RewrittenYaml cannot add new keys, which
        a different controller plugin needs)."""
        import tempfile

        c = int(LaunchConfiguration("category").perform(context))
        if c not in (2, 3):
            return [SetLaunchConfiguration("nav2_params", NAV2_PARAMS_C3)]
        controller = LaunchConfiguration("controller").perform(context).lower()

        base_yaml = NAV2_PARAMS_C2 if c == 2 else NAV2_PARAMS_C3
        with open(base_yaml) as f:
            params = yaml.safe_load(f)
        teb_file = TEB_MPC_PARAMS if c == 2 else THETA_STAR_PARAMS
        ctrl_file = CONTROLLERS[controller]["params"] or teb_file
        with open(ctrl_file) as f:
            ctrl = yaml.safe_load(f)["controller_server"]["ros__parameters"]["FollowPath"]

        cs = params["controller_server"]["ros__parameters"]
        if cs.get("FollowPath", {}).get("plugin") == ctrl.get("plugin"):
            cs["FollowPath"].update(ctrl)        # same plugin (TEB): override its values
        else:
            cs["FollowPath"] = dict(ctrl)        # another controller: replace the block

        # Same angular limits for every controller (and the velocity smoother)
        w_max = float(LaunchConfiguration("w_max").perform(context))
        w_acc = float(LaunchConfiguration("w_acc_max").perform(context))
        val = {"w": w_max, "a": w_acc, "-a": -w_acc}
        for key, kind in ANGULAR_LIMIT_KEYS[controller].items():
            cs["FollowPath"][key] = val[kind]
        vs = params.get("velocity_smoother", {}).get("ros__parameters")
        if vs is not None:
            vs["max_velocity"][2] = w_max
            vs["min_velocity"][2] = -w_max
            vs["max_accel"][2] = w_acc
            if "max_decel" in vs:
                vs["max_decel"][2] = -w_acc

        planner_name = ""
        if c == 3:
            planner_name = LaunchConfiguration("planner").perform(context).lower()
            with open(PLANNERS[planner_name]) as f:
                planner = yaml.safe_load(f)["planner_server"]["ros__parameters"]["GridBased"]
            ps = params["planner_server"]["ros__parameters"]
            ps.pop("planner_plugin_types", None)   # Foxy syntax, ignored by Humble
            ps.pop("planner_plugin_ids", None)
            ps["planner_plugins"] = ["GridBased"]
            ps["GridBased"] = dict(planner)

        # With the MVSim fake localization, AMCL is kept alive (Nav2 needs it) but
        # it must not publish map->odom nor /amcl_pose: disable its TF broadcast
        # and connect it to an unused scan topic.
        localization = LaunchConfiguration("localization").perform(context).lower()
        if localization == "fake" and "amcl" in params:
            params["amcl"]["ros__parameters"]["tf_broadcast"] = False
            params["amcl"]["ros__parameters"]["scan_topic"] = "/amcl_unused_scan"

        fd, path = tempfile.mkstemp(prefix=f"benchmark_nav2_c{c}_{controller}{planner_name}_", suffix=".yaml")
        with os.fdopen(fd, "w") as f:
            yaml.safe_dump(params, f, default_flow_style=False, sort_keys=False)
        print(f"[OK] Nav2 parameters (category {c}, controller {controller}"
              f"{', planner ' + planner_name if planner_name else ''}): {path}")
        return [SetLaunchConfiguration("nav2_params", path)]

    nav2_param_rewrite = OpaqueFunction(function=generate_nav2_params)

    # ---------------------------------------------------------------------------
    #                                ROS2 NODES
    # ---------------------------------------------------------------------------
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
                "publish_tf_odom2baselink": LaunchConfiguration(
                    "publish_tf_odom2baselink"
                ),
                "force_publish_vehicle_namespace": LaunchConfiguration(
                    "force_publish_vehicle_namespace"
                ),
                # Logger columns as topics (replaces the old /benchmark_params message)
                "publish_log_topics": True,
            },
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

    # acml init pose publisher, necesary for Nav2 localization
    init_pose_node = Node(
        package="benchmark_bringup",
        executable="odom_mvsim2teb",
        name="init_pose_publisher",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("category"), "' != '1'"])
        ),
    )

    # Nav2 bringup (not needed, nor required to be installed, for category 1)
    def nav2_bringup_launch_file():
        try:
            share = get_package_share_directory("nav2_bringup")
        except Exception:
            return ""
        return os.path.join(share, "launch", "bringup_launch.py")

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch_file()),
        condition=IfCondition(LaunchConfiguration("use_nav2")),
        launch_arguments={
            "slam": LaunchConfiguration("slam"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "params_file": LaunchConfiguration("nav2_params"),
            "map": LaunchConfiguration("map"),
            "autostart": LaunchConfiguration("autostart"),
            "use_composition": LaunchConfiguration("use_composition"),
            "use_respawn": LaunchConfiguration("use_respawn"),
        }.items(),
    )

    # ---------------------------------------------------------------------------
    #                         LAUNCH DESCRIPTION
    # ---------------------------------------------------------------------------
    ld = LaunchDescription()

    # Add common arguments
    ld.add_action(world_file_launch_arg)
    ld.add_action(headless_launch_arg)
    ld.add_action(do_fake_localization_arg)
    ld.add_action(publish_tf_odom2baselink_arg)
    ld.add_action(force_publish_vehicle_namespace_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(use_nav2_arg)
    ld.add_action(rviz_config_file_arg)
    ld.add_action(declare_category_arg)
    ld.add_action(declare_payload_arg)
    ld.add_action(declare_terrain_slope_arg)
    ld.add_action(declare_change_terrain_arg)
    ld.add_action(declare_localization_arg)
    ld.add_action(declare_controller_arg)
    ld.add_action(declare_local_path_topic_arg)
    ld.add_action(declare_planner_arg)
    ld.add_action(declare_w_max_arg)
    ld.add_action(declare_w_acc_max_arg)
    ld.add_action(declare_seed_arg)
    ld.add_action(declare_results_dir_arg)
    ld.add_action(declare_run_name_arg)
    ld.add_action(declare_plot_arg)
    ld.add_action(declare_auto_shutdown_arg)
    ld.add_action(DeclareLaunchArgument("num_plants_south", default_value="15"))
    ld.add_action(DeclareLaunchArgument("num_plants_north", default_value="10"))
    ld.add_action(DeclareLaunchArgument("num_workers", default_value="0"))
    ld.add_action(validate_action)
    ld.add_action(selected_nodes)

    # Add Nav2 arguments
    ld.add_action(declare_map_launch_arg)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_params_file_teb)
    ld.add_action(declare_params_file_theta)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_slam_launch_arg)

    # ===============================
    # Load PID parameters from YAML
    # ===============================
    with open(PID_MPC_PARAMS, "r") as f:
        params = yaml.safe_load(f)

    # Aplanar diccionario (PID.KP → CTRL_PID_KP)
    def flatten(prefix, d, out):
        for k, v in d.items():
            key = f"{prefix}_{k}".upper()
            if isinstance(v, dict):
                flatten(key, v, out)
            else:
                out[key] = v

    flat_params = {}
    flatten("CTRL", params, flat_params)

    # 1) Declarar argumentos LaunchConfiguration para todos
    for key, val in flat_params.items():
        ld.add_action(DeclareLaunchArgument(key, default_value=str(val)))

    # 2) Crear variables de entorno a partir de esos argumentos
    for key in flat_params.keys():
        env_name = key.replace("CTRL_PID_", "")
        ld.add_action(
            SetEnvironmentVariable(name=env_name, value=LaunchConfiguration(key))
        )

    # Set benchmark arguments as environment variables
    ld.add_action(
        SetEnvironmentVariable(name="MVSIM_RANDOM_SEED", value=LaunchConfiguration("seed"))
    )
    ld.add_action(
        SetEnvironmentVariable(
            name="PAYLOAD_MASS", value=LaunchConfiguration("payload")
        )
    )
    ld.add_action(
        SetEnvironmentVariable(name="MIN_ALT", value=LaunchConfiguration("min_terrain"))
    )
    ld.add_action(
        SetEnvironmentVariable(name="MAX_ALT", value=LaunchConfiguration("max_terrain"))
    )
    ld.add_action(
        SetEnvironmentVariable(
            name="USE_ADAPTATIVE_FRICTION",
            value=LaunchConfiguration("adaptative_friction"),
        )
    )

    # Example of PID parameters usage:
    ld.add_action(
        SetEnvironmentVariable(name="KP", value=LaunchConfiguration("CTRL_PID_KP"))
    )
    ld.add_action(
        SetEnvironmentVariable(name="KI", value=LaunchConfiguration("CTRL_PID_KI"))
    )
    ld.add_action(
        SetEnvironmentVariable(name="KD", value=LaunchConfiguration("CTRL_PID_KD"))
    )
    ld.add_action(
        SetEnvironmentVariable(name="N", value=LaunchConfiguration("CTRL_PID_N"))
    )
    ld.add_action(
        SetEnvironmentVariable(
            name="ENABLE_ANTIWINDUP",
            value=LaunchConfiguration("CTRL_PID_ENABLE_ANTIWINDUP"),
        )
    )
    ld.add_action(
        SetEnvironmentVariable(
            name="ENABLE_FEEDFORWARD",
            value=LaunchConfiguration("CTRL_FEEDFORWARD_ENABLE_FEEDFORWARD"),
        )
    )
    ld.add_action(
        SetEnvironmentVariable(
            name="FEEDFORWARD_GAIN",
            value=LaunchConfiguration("CTRL_FEEDFORWARD_FEEDFORWARD_GAIN"),
        )
    )
    ld.add_action(
        SetEnvironmentVariable(
            name="ENABLE_REFERENCE_FILTER",
            value=LaunchConfiguration("CTRL_REFERENCEFILTER_ENABLE_REFERENCE_FILTER"),
        )
    )
    ld.add_action(
        SetEnvironmentVariable(
            name="REFERENCE_FILTER_TAU",
            value=LaunchConfiguration("CTRL_REFERENCEFILTER_REFERENCE_FILTER_TAU"),
        )
    )
    ld.add_action(
        SetEnvironmentVariable(
            name="REFERENCE_FILTER_ORDER",
            value=LaunchConfiguration("CTRL_REFERENCEFILTER_REFERENCE_FILTER_ORDER"),
        )
    )
    # Greenhouse population (number of plants per corridor and of human workers)
    for arg, env in (("num_plants_south", "NUM_PLANTS_SOUTH"),
                     ("num_plants_north", "NUM_PLANTS_NORTH"),
                     ("num_workers", "NUM_WORKERS")):
        ld.add_action(SetEnvironmentVariable(name=env, value=LaunchConfiguration(arg)))
    ld.add_action(
        SetEnvironmentVariable(
            name="MAX_TORQUE", value=LaunchConfiguration("CTRL_MOTORSIM_MAX_TORQUE")
        )
    )

    # Add nodes
    ld.add_action(nav2_param_rewrite)
    ld.add_action(mvsim_node)
    ld.add_action(init_pose_node)
    ld.add_action(rviz2_node)
    ld.add_action(bringup_cmd)

    return ld
