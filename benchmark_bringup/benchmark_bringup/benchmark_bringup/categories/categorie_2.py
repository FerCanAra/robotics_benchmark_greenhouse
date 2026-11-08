#!/usr/bin/env python3
# +-------------------------------------------------------------------------+
# |                       Benchmark control simulator                       |
# |                                                                         |
# | Copyright (C) 2025  Fernando Cañadas Aránega                            |
# | PhD Student University of Almería, Spain                                |
# | Contact: fernando.ca@ual.es                                             |
# | Distributed under 3-clause BSD License                                  |
# | See COPYING                                                             |
# | Category 2: PID Control + MPC Control                                   |
# +-------------------------------------------------------------------------+

# ---------------------------------------------------------------------------
#                  Mid Level Control Inputs - Category 2
# ---------------------------------------------------------------------------

import math

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nav2_simple_commander.robot_navigator import BasicNavigator

from benchmark_bringup.categories.common import (
    StatusPublisher,
    initial_pose_from_odom,
    make_pose,
    make_route_path,
    run_navigation,
)

# CHANGE YOUR ROUTE HERE (x, y)
# Category 2 evaluates the local controller only: the route (start -> waypoints) is
# turned into a fixed polyline and sent straight to the controller server through
# the FollowPath action, with no global planner and no behaviour tree. Every
# controller therefore follows exactly the same reference (aisle, headland, aisle).
# (With NavigateThroughPoses the planner/BT occasionally sent the robot back to a
# corner already passed, or down another aisle.)
ROUTE = [
    (10.2, 13.7),
    (15.0, 13.7),
    (18.0, 13.7),
    (18.0, 17.4),
]
MAX_TIME = 180.0  # [s] termination rule of the benchmark (skid-steer platform: in-place corrections are slow)


class Category2Navigator(Node):
    def __init__(self):
        super().__init__("route_cat2")
        self.navigator = BasicNavigator()
        self.marker_pub = self.create_publisher(MarkerArray, "/goal_markers", 10)
        self.status = StatusPublisher(self, category=2)

    def publish_goal_markers(self, poses):
        marker_array = MarkerArray()
        for i, pose in enumerate(poses):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "goal_points"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose = pose.pose
            m.scale.x = m.scale.y = m.scale.z = 0.2
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 1.0
            marker_array.markers.append(m)
        self.marker_pub.publish(marker_array)

    def build_poses(self):
        # Orientation of each waypoint: direction of the next segment (last one:
        # direction of the previous segment). Deterministic final orientation.
        poses = []
        for i, (x, y) in enumerate(ROUTE):
            if i < len(ROUTE) - 1:
                xn, yn = ROUTE[i + 1]
                yaw = math.atan2(yn - y, xn - x)
            else:
                xp, yp = ROUTE[i - 1]
                yaw = math.atan2(y - yp, x - xp)
            poses.append(make_pose(self.navigator, x, y, yaw))
        return poses

    def execute_route(self):
        start = initial_pose_from_odom(self, self.navigator)
        self.get_logger().info("Waiting for Nav2...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active, executing route...")
        poses = self.build_poses()
        self.publish_goal_markers(poses)
        if start is None:
            start = (10.1, 3.0)
            self.get_logger().warn(f"Start pose unknown: using {start}")
        path = make_route_path(self.navigator, [start] + list(ROUTE))
        state = run_navigation(self, self.navigator, self.status, path,
                               [list(p) for p in ROUTE], MAX_TIME, follow_path=True)
        self.get_logger().info(f"Category 2 route finished with status: {state}")


def main(args=None):
    rclpy.init(args=args)
    node = Category2Navigator()
    try:
        node.execute_route()
        # keep the node alive so that the latched status remains available
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
