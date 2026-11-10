#!/usr/bin/env python3
# +-------------------------------------------------------------------------+
# |                       Benchmark control simulator                       |
# |                                                                         |
# | Copyright (C) 2025  Fernando Cañadas Aránega                            |
# | PhD Student University of Almería, Spain                                |
# | Contact: fernando.ca@ual.es                                             |
# | Distributed under 3-clause BSD License                                  |
# | See COPYING                                                             |
# | Category 3: PID Control + MPC Control + Planner                         |
# +-------------------------------------------------------------------------+

# ---------------------------------------------------------------------------
#                  High Level Control Inputs - Category 3
# ---------------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator

from benchmark_bringup.categories.common import (
    StatusPublisher,
    initial_pose_from_ground_truth,
    make_pose,
    run_navigation,
)

# CHANGE YOUR OBJECTIVE POINT OF THE CATEGORY 3 HERE (x, y, yaw)
ROUTE = [
    (18.0, 17.4, 0.74),
]
MAX_TIME = 180.0  # [s] termination rule of the benchmark (skid-steer platform: in-place corrections are slow)


class Category3Navigator(Node):
    def __init__(self):
        super().__init__("route_cat3")
        self.navigator = BasicNavigator()
        self.status = StatusPublisher(self, category=3)

    def execute_route(self):
        initial_pose_from_ground_truth(self, self.navigator)
        self.get_logger().info("Waiting for Nav2...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active, executing Cat3 route...")
        poses = [make_pose(self.navigator, x, y, yaw) for x, y, yaw in ROUTE]
        state = run_navigation(self, self.navigator, self.status, poses,
                               [list(p) for p in ROUTE], MAX_TIME)
        self.get_logger().info(f"Category 3 route finished with status: {state}")


def main(args=None):
    rclpy.init(args=args)
    node = Category3Navigator()
    try:
        node.execute_route()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        node.get_logger().warn("Ctrl-C detected: canceling active Nav2 tasks...")
    finally:
        node.navigator.cancelTask()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
