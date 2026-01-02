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

import rclpy
from rclpy.node import Node

import math
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler


class Category2Navigator(Node):
    def __init__(self):
        super().__init__("route_cat2")

        self.navigator = BasicNavigator()
        self.marker_pub = self.create_publisher(MarkerArray, "/goal_markers", 10)

        # CHANGE THE ROUTE HERE (x, y)
        self.route = [
            (10.2, 13.7),
            (15.0, 13.7),
            (18.0, 17.4),
        ]

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

            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.2

            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 1.0

            marker_array.markers.append(m)

        self.marker_pub.publish(marker_array)
        self.get_logger().info("Goal markers published for RViz")

    def execute_route(self):
        self.get_logger().info("Waiting for Nav2...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active, executing route...")

        poses = []

        for i, (x, y) in enumerate(self.route):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            if i < len(self.route) - 1:
                x_next, y_next = self.route[i + 1]
                yaw = math.atan2(y_next - y, x_next - x)
            else:
                x_prev, y_prev = self.route[i - 1]
                yaw = math.atan2(y - y_prev, x - x_prev)

            q = quaternion_from_euler(0.0, 0.0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            poses.append(deepcopy(pose))

        # Publish markers for visualization
        self.publish_goal_markers(poses)
        self.navigator.followWaypoints(poses)
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(
                    f"Reached waypoint " f"{feedback.current_waypoint + 1}/{len(poses)}"
                )

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Category 2 route completed successfully!")
        else:
            self.get_logger().warn(f"Route finished with status: {result}")


def main(args=None):
    rclpy.init(args=args)

    node = Category2Navigator()

    try:
        node.execute_route()
    except KeyboardInterrupt:
        node.get_logger().warn("Ctrl-C detected: canceling active Nav2 tasks...")
        node.navigator.cancelTask()
    finally:
        node.navigator.cancelTask()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
