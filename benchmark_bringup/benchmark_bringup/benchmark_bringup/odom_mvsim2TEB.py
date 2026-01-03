#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitPosePublisher(Node):
    def __init__(self):
        super().__init__('init_pose_publisher')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10)
        self.pose_published = False
        self.get_logger().info("Esperando primer /odom para publicar /initialpose...")

    def odom_callback(self, msg):
        if self.pose_published:
            return

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"   # ← AMCL espera 'map' aquí
        pose_msg.pose.pose = msg.pose.pose

        # Opcional: pequeñas covarianzas
        pose_msg.pose.covariance = [0.0]*36
        pose_msg.pose.covariance[0] = 0.25
        pose_msg.pose.covariance[7] = 0.25
        pose_msg.pose.covariance[35] = 0.0685

        self.publisher.publish(pose_msg)
        self.pose_published = True
        self.get_logger().info("Publicada pose inicial desde /odom en /initialpose")
        # Si quieres, puedes cerrar el nodo tras publicar:
        # self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = InitPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
