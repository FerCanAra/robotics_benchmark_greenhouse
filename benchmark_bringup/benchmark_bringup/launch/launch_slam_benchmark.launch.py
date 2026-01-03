from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    mvsim_dir = get_package_share_directory("mvsim")
    mvsim_nav2_demo_dir = get_package_share_directory("mvsim_nav2_demos")

    world_file = os.path.join(
        mvsim_dir, "mvsim_tutorial", "greenhouse_control.world.xml"
    )
    mvsim_params = os.path.join(mvsim_dir, "mvsim_tutorial", "mvsim_ros2_params.yaml")
    slam_params = os.path.join(
        mvsim_nav2_demo_dir, "params", "nav2_slam_greenhouse.yaml"
    )
    # laser_filter_params = os.path.join(
    #     get_package_share_directory('mvsim_nav2_demos'),
    #     'params',
    #     'laser_filter.yaml')
    rviz_config = os.path.join(mvsim_nav2_demo_dir, "rviz", "greenhouse_slam.rviz")

    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="False")
    declare_headless = DeclareLaunchArgument("headless", default_value="False")

    # Launch MVSim
    mvsim_node = Node(
        package="mvsim",
        executable="mvsim_node",
        output="screen",
        parameters=[
            mvsim_params,
            {"world_file": world_file, "headless": LaunchConfiguration("headless")},
        ],
    )

    # laser_filter_node = Node(
    #     package='laser_filters',
    #     executable='scan_to_scan_filter_chain',
    #     name='laser_filter',
    #     output='screen',
    #     parameters=[laser_filter_params],
    #     remappings=[
    #         ('scan', '/laser1'),
    #         ('scan_filtered', '/laser1_filtered')
    #     ]
    # )

    # Launch SLAM Toolbox
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("slam_toolbox"),
                    "launch",
                    "online_async_launch.py",
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "slam_params_file": slam_params,
        }.items(),
    )

    # Launch RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_headless,
            mvsim_node,
            # laser_filter_node,
            slam_toolbox,
            rviz,
        ]
    )
