#!/usr/bin/env python3
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
# The velocity profile is scheduled in SIMULATED time (the Timestamp column of the
# MVSim wheel log, published as a topic), so it does not depend on the CPU load of
# the computer. t = 0 is the first log row received by this node, and it is
# announced to the plotter through the STARTED event on /benchmark/status.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from benchmark_bringup.categories.common import StatusPublisher

# CHANGE YOUR VELOCITY PROFILE HERE: (t_start [s], v [m/s], w [rad/s])
# Each command is applied from t_start until the next one.
PROFILE = [
    (0.0, 0.0, 0.0),  # rest
    (5.0, 0.5, 0.0),  # advance
    (35.0, 0.0, 0.0),  # stop
    (
        36.0,
        0.0,
        1.1,
    ),  # in-place rotation of 180 deg (command longer than pi/(pi/2)
    # to compensate for the closed-loop response and slippage)
    (39.3, 0.0, 0.0),  # stop
    (40.4, 0.5, 0.0),  # advance
    (49.0, 0.0, 0.0),  # stop (end of the profile)
]
T_PROFILE_END = 49.0


class VelocityProfileCat1(Node):
    def __init__(self):
        super().__init__("velocity_cat1")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status = StatusPublisher(self, category=1)
        self.t0_sim = None
        self.t_sim = None
        self.finished = False
        prefix = str(self.declare_parameter("log_topic_prefix", "/log").value).rstrip("/")
        self.create_subscription(Float64, f"{prefix}/wheel_1/Timestamp", self.cb_stamp, 50)
        self.create_timer(0.02, self.update_velocity)
        self.get_logger().info(
            "Category 1 test: advance 30 s, turn 180 degrees, advance (simulated time)"
        )

    def cb_stamp(self, msg: Float64):
        self.t_sim = msg.data
        if self.t0_sim is None:
            self.t0_sim = msg.data
            self.status.publish("STARTED", t0_sim=self.t0_sim, profile=PROFILE)

    @staticmethod
    def command_at(t: float):
        v, w = 0.0, 0.0
        for t_k, v_k, w_k in PROFILE:
            if t >= t_k:
                v, w = v_k, w_k
        return v, w

    def update_velocity(self):
        msg = Twist()
        if self.t0_sim is not None and self.t_sim is not None:
            t = self.t_sim - self.t0_sim
            msg.linear.x, msg.angular.z = self.command_at(t)
            if t >= T_PROFILE_END and not self.finished:
                self.finished = True
                self.status.publish("PROFILE_END", t_sim=t)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityProfileCat1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
