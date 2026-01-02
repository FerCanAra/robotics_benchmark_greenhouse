#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf_transformations import quaternion_from_euler
from copy import deepcopy


class Category3Navigator(Node):
    def __init__(self):
        super().__init__("route_cat3")

        self.navigator = BasicNavigator()

        # Misma ruta que categoría 2
        self.route = [
            (10.1, 13.7, 1.54),
            (14.6, 14.2, 0.74),
            (18.2, 2.3, 0.74),
        ]

    def execute_route(self):
        self.get_logger().info("Waiting for Nav2...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active, executing Cat3 route...")

        poses = []
        for x, y, yaw in self.route:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            q = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            poses.append(deepcopy(pose))

        self.navigator.followWaypoints(poses)

        # Feedback loop
        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 10 == 0:
                self.get_logger().info(
                    f"Executing waypoint {feedback.current_waypoint}/{len(poses)}"
                )

        result = self.navigator.getResult()
        self.get_logger().info(f"Category 3 route result: {result}")


def main(args=None):
    rclpy.init(args=args)
    node = Category3Navigator()
    node.execute_route()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
