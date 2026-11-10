# +-------------------------------------------------------------------------+
# |                       Benchmark control simulator                       |
# |                                                                         |
# | Copyright (C) 2025  Fernando Cañadas Aránega                            |
# | PhD Student University of Almería, Spain                                |
# | Contact: fernando.ca@ual.es                                             |
# | Distributed under 3-clause BSD License                                  |
# | See COPYING                                                             |
# | Common utilities of the category scripts                                |
# +-------------------------------------------------------------------------+
"""Utilities shared by the category scripts.

The scripts publish the events of the run on /benchmark/status (JSON strings,
latched QoS) so that the plotters open and close the evaluation window at the
same instants:
    {"state": "STARTED", "t_wall": ..., "t0_sim": ..., "route": [...]}
    {"state": "SUCCEEDED" | "FAILED" | "CANCELED" | "TIMEOUT" | "FINISHED", "t_wall": ...}
"""

import json
import math
import time
from copy import deepcopy

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

STATUS_TOPIC = "/benchmark/status"


def status_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=20,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class StatusPublisher:
    def __init__(self, node, category: int):
        self.node = node
        self.category = category
        self.pub = node.create_publisher(String, STATUS_TOPIC, status_qos())

    def publish(self, state: str, **fields):
        ev = {"category": self.category, "state": state, "t_wall": time.time()}
        ev.update(fields)
        msg = String()
        msg.data = json.dumps(ev)
        self.pub.publish(msg)
        self.node.get_logger().info(f"[benchmark/status] {msg.data}")
        return ev


def make_pose(navigator, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(0.5 * yaw)
    pose.pose.orientation.w = math.cos(0.5 * yaw)
    return deepcopy(pose)


def initial_pose_from_ground_truth(node, navigator, timeout: float = 30.0):
    """Wait for the first MVSim ground-truth pose and use it as the initial pose of
    Nav2. This avoids BasicNavigator publishing a default (0, 0, 0) initial pose.
    (/odom cannot be used: the odometry frame starts at the origin, not at the
    pose of the robot in the map.)"""
    first = {}

    def cb(msg):
        if not first:
            first["msg"] = msg

    sub = node.create_subscription(Odometry, "/base_pose_ground_truth", cb, 10)
    t_start = time.time()
    while rclpy.ok() and not first and time.time() - t_start < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    if not first:
        node.get_logger().warn(
            "No ground-truth pose received: Nav2 initial pose not set by the script.")
        return
    p = first["msg"].pose.pose
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose = p
    navigator.setInitialPose(pose)
    return (p.position.x, p.position.y)


def make_route_path(navigator, points, step: float = 0.05):
    """nav_msgs/Path along the polyline `points` [(x, y), ...], sampled every `step`
    metres, each pose oriented along its segment (frame: map)."""
    from nav_msgs.msg import Path
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = navigator.get_clock().now().to_msg()
    for (x0, y0), (x1, y1) in zip(points[:-1], points[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg < 1e-6:
            continue
        yaw = math.atan2(y1 - y0, x1 - x0)
        n = max(1, int(math.ceil(seg / step)))
        for k in range(n):
            f = k / n
            path.poses.append(make_pose(navigator, x0 + f * (x1 - x0), y0 + f * (y1 - y0), yaw))
    xl, yl = points[-1]
    xp, yp = points[-2]
    path.poses.append(make_pose(navigator, xl, yl, math.atan2(yl - yp, xl - xp)))
    return path


def run_navigation(node, navigator, status: StatusPublisher, poses, route, max_time: float,
                   through_poses: bool = False, follow_path: bool = False):
    """Execute the task, publishing STARTED and the terminal event; cancels the task
    when max_time is exceeded (termination rule of the benchmark).

    through_poses=False: followWaypoints (one NavigateToPose per waypoint, the robot
    stops at each one). through_poses=True: NavigateThroughPoses (one continuous path
    through all the waypoints; only the last one is a goal to stop at)."""
    from nav2_simple_commander.robot_navigator import TaskResult

    status.publish("STARTED", route=route, max_time=max_time)
    t_start = time.time()
    if follow_path:
        # poses is a nav_msgs/Path: sent straight to the controller server
        # (FollowPath action), with no planner and no behaviour tree in between.
        navigator.followPath(poses)
    elif through_poses:
        navigator.goThroughPoses(poses)
    else:
        navigator.followWaypoints(poses)
    i = 0
    timed_out = False
    while not navigator.isTaskComplete():
        i += 1
        if time.time() - t_start > max_time:
            node.get_logger().warn(f"Maximum time ({max_time} s) exceeded: canceling the task.")
            navigator.cancelTask()
            timed_out = True
            break
        feedback = navigator.getFeedback()
        if feedback and i % 10 == 0:
            if hasattr(feedback, "current_waypoint"):
                node.get_logger().info(f"Executing waypoint {feedback.current_waypoint + 1}/{len(poses)}")
            elif hasattr(feedback, "distance_to_goal"):
                node.get_logger().info(f"Distance to goal: {feedback.distance_to_goal:.2f} m")
            elif hasattr(feedback, "number_of_poses_remaining"):
                node.get_logger().info(
                    f"Poses remaining: {feedback.number_of_poses_remaining}/{len(poses)}")
    if timed_out:
        state = "TIMEOUT"
    else:
        result = navigator.getResult()
        state = {
            TaskResult.SUCCEEDED: "SUCCEEDED",
            TaskResult.CANCELED: "CANCELED",
            TaskResult.FAILED: "FAILED",
        }.get(result, "FAILED")
    status.publish(state, duration=time.time() - t_start)
    return state
