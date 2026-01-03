# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""All-in-one launch script for MVSim + Nav2 + auto init pose from odom."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution

mvsimDir = get_package_share_directory("mvsim")
mvsimNav2DemoDir = get_package_share_directory("mvsim_nav2_demos")

MVSIM_WORLD_FILE = os.path.join(mvsimDir, "mvsim_tutorial", "demo_greenhouse.world.xml")
MVSIM_ROS2_PARAMS_FILE = os.path.join(
    mvsimDir, "mvsim_tutorial", "greenhouse_nav2_params.yaml"
)
waypoint_follower_yaml = os.path.join(
    get_package_share_directory("follow_waypoints"), "config", "follow_waypoints.yaml"
)


def generate_launch_description():

    ############################# DECLARE ARGUMENTS ##############################
    launch_dir = os.path.join(mvsimNav2DemoDir, "launch")
    world_file_launch_arg = DeclareLaunchArgument(
        "world_file", default_value=TextSubstitution(text=MVSIM_WORLD_FILE)
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            mvsimNav2DemoDir, "params", "greenhouse_nav2_params.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (MVSim) clock if true",
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

    declare_use_slam = DeclareLaunchArgument(
        "use_slam", default_value="False", description="Use SLAM."
    )

    params_file = LaunchConfiguration("params_file")  # for nav2
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_slam = LaunchConfiguration("use_slam")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")

    ############################# WAYPOINT FOLLOWER ##############################
    declare_waypoint = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[waypoint_follower_yaml],
    )

    ############################# NAV2 BRINGUP ###################################
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "mvsim_mpc_nav2_bringup_launch.py")
        ),
        launch_arguments={
            "slam": use_slam,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
            "use_composition": use_composition,
            "map": LaunchConfiguration("map"),
            "use_respawn": use_respawn,
        }.items(),
    )

    ############################# AUTO INIT POSE NODE ############################
    # Este nodo lee el primer mensaje de /odom y publica /initialpose para AMCL
    init_pose_node = Node(
        package="simulator_control",  # tu paquete
        executable="odom_mvsim2TEB",  # script Python
        name="init_pose_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    ############################# CREATE LAUNCH DESCRIPTION ######################
    ld = LaunchDescription()

    ld.add_action(world_file_launch_arg)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_use_slam)
    ld.add_action(declare_waypoint)
    ld.add_action(bringup_cmd)
    ld.add_action(init_pose_node)  # ← añade aquí el publicador de pose inicial

    return ld
