# +-------------------------------------------------------------------------+
# |                       Benchmark control simulator                       |
# |                                                                         |
# | Copyright (C) 2025  Fernando Cañadas Aránega                            |
# | PhD Student University of Almería, Spain                                |
# | Contact: fernando.ca@ual.es                                             |
# | Distributed under 3-clause BSD License                                  |
# | See COPYING                                                             |
# | Category 1: Cat 1 (Low Level)                                           |
# +-------------------------------------------------------------------------+

# ---------------------------------------------------------------------------
#                  Low Level Control Inputs - Category 1
# ---------------------------------------------------------------------------

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import sys


class VelocityProfileCat1(Node):
    def __init__(self):
        super().__init__("velocity_cat1")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.get_logger().info(
            "Starting test category 1: Advance 30s, turn 180 degrees, return 30s"
        )
        self.start_time = time.time()
        self.timer = self.create_timer(0.02, self.update_velocity)

    def update_velocity(self):
        t = time.time() - self.start_time
        msg = Twist()

        # ============= Speed profile =================
        if t < 5.0:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        elif t < 35.0:
            msg.linear.x = 0.5
            msg.angular.z = 0.0
        elif t < 36.0:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        elif t < 39.3:
            msg.linear.x = 0.0
            msg.angular.z = 1.5708
        elif t < 40.4:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        elif t < 49.0:
            msg.linear.x = 0.5
            msg.angular.z = 0.0
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.pub.publish(msg)
            self.timer.cancel()
            rclpy.shutdown()
            sys.exit(0)
            return
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityProfileCat1()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()
