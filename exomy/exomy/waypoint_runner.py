#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(frame_id: str, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp.sec = 0
    pose.header.stamp.nanosec = 0
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main() -> None:
    rclpy.init()
    navigator = BasicNavigator(namespace='leo')

    goals = [
        ("B", make_pose("map", -3.0, 6.5, 0.0)),
        ("C", make_pose("map", -7.0, -5.0, 0.0)),
    ]

    navigator.waitUntilNav2Active(navigator="bt_navigator", localizer="controller_server")

    for label, goal in goals:
        navigator.get_logger().info(f"Navigating to {label}: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})")
        goal.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(goal)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback is not None:
                navigator.get_logger().info(f"{label}: distance remaining {feedback.distance_remaining:.2f} m")
            time.sleep(1.0)

        result = navigator.getResult()
        if result != TaskResult.SUCCEEDED:
            navigator.get_logger().error(f"Failed to reach {label}, result={result}")
            raise SystemExit(1)

    navigator.get_logger().info("Reached B -> C successfully")
    raise SystemExit(0)


if __name__ == "__main__":
    main()
