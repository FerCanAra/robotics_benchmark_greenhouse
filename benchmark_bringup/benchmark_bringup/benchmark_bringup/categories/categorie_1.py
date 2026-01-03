#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import sys

class VelocityProfileCat1(Node):
    def __init__(self):
        super().__init__('velocity_cat1')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.start_time = time.time()

        # 50 Hz timer
        self.timer = self.create_timer(0.02, self.update_velocity)

    def update_velocity(self):
        t = time.time() - self.start_time
        msg = Twist()

        # Fase 1:
        if t < 5.0:
            msg.linear.x = 0.0

        # Fase 2: 
        elif t < 40.0:
            msg.linear.x = 0.75

        # Fase 3: parar y cerrar nodo
        else:
            msg.linear.x = 0.0
            self.pub.publish(msg)
            self.get_logger().info("Velocidad finalizada (Cat 1), cerrando nodo...")
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


if __name__ == '__main__':
    main()
 