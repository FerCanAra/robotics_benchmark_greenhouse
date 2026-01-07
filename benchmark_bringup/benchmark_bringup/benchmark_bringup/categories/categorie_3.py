# +-------------------------------------------------------------------------+
# |                       Benchmark control simulator                       |
# |                                                                         |
# | Copyright (C) 2025  Fernando Cañadas Aránega                            |
# | PhD Student University of Almería, Spain                                |
# | Contact: fernando.ca@ual.es                                             |
# | Distributed under 3-clause BSD License                                  |
# | See COPYING                                                             |
# | Category 3: PID Control + MPC Control + Theta* Control                  |
# +-------------------------------------------------------------------------+

# ---------------------------------------------------------------------------
#                High Level Control Inputs - Category 3
# ---------------------------------------------------------------------------

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

        # CHANGE YOUR OBJECTIVE POINT OF THE CATEGORY 3 HERE (x, y, yaw)
        self.route = [
            (18.0, 17.4, 0.74),
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
