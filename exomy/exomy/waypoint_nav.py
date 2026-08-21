#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


# 3 waypoints A, B and C defined as (label, x, y, yaw_deg)
# Several goals for each point

WAYPOINT_GROUPS = [
    # A
    [
        ("A-approach", 23.0, 15.0,  90.0), 
        ("A",          18.5, 17.5,  135.0),  # waypoint A
    ],
    # B
    [
        ("B-approach-1", 14.0, 22.0, 135.0),
        ("B-approach-2", 11.0, 20.0, 180.0),
        ("B",            10.0, 18.0, 225.0),  # waypoint B
    ],
    # C
    [
        ("C-approach",  4.0, 11.0, 225.0),   
        ("C",           6.0,  5.0, 290.0),   # waypoint C
    ],
]

MAX_RETRIES        = 5     # retries per sub-goal
ABORT_RETRY_DELAY  = 15.0   # seconds to wait after an abort before retrying
REJECT_RETRY_DELAY = 15.0   # seconds to wait after a rejection
GOAL_TIMEOUT       = 1200.0 # seconds per sub-goal


def yaw_to_quaternion(yaw_deg: float):
    """Convert yaw angle (degrees about Z) to quaternion (z, w) components."""
    r = math.radians(yaw_deg)
    return math.sin(r / 2.0), math.cos(r / 2.0)


def build_goal(x: float, y: float, yaw_deg: float,
               node: Node, frame: str = "map") -> NavigateToPose.Goal:
    """Construct a NavigateToPose.Goal at position (x, y) with heading yaw_deg."""
    goal = NavigateToPose.Goal()
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    qz, qw = yaw_to_quaternion(yaw_deg)
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    goal.pose = pose
    return goal


class WaypointNavigatorNode(Node):

    def __init__(self):
        super().__init__("waypoint_navigator_node")

        self.declare_parameter("nav_action_server",
                               "/mars_rover/navigate_to_pose")
        action_name = self.get_parameter("nav_action_server").value

        # STEP 1: Create the NavigateToPose action client.
        self._ac = ActionClient(self, NavigateToPose, action_name)
        self.get_logger().info(
            f"WaypointNavigatorNode: connecting to '{action_name}' ..."
        )

        # STEP 2: wait_for_server() — block until Nav2 bt_navigator is ready.
        if not self._ac.wait_for_server(timeout_sec=60.0):
            self.get_logger().error("Nav2 action server not available after 60 s.")
            raise RuntimeError("Nav2 not available")

        self.get_logger().info("Nav2 connected. Starting traverse.")

    def run(self) -> None:
        """Navigate through all waypoint groups in order."""
        all_ok = True

        for group in WAYPOINT_GROUPS:
            group_name = group[-1][0]  # name of the labelled endpoint
            self.get_logger().info(
                f"\n{'='*60}\n  Navigating group → {group_name}\n{'='*60}"
            )

            for label, x, y, yaw in group:
                self.get_logger().info(
                    f"  Sub-goal: {label}  ({x:.1f}, {y:.1f})"
                )
                ok = self._navigate_to(label, x, y, yaw)
                if ok:
                    self.get_logger().info(
                        f"  ✓ Reached sub-goal {label}"
                    )
                else:
                    self.get_logger().error(
                        f"  ✗ Failed sub-goal {label} after {MAX_RETRIES} "
                        f"retries.  Skipping rest of group."
                    )
                    all_ok = False
                    break   # skip remaining sub-goals in this group

        result = "ALL WAYPOINTS REACHED" if all_ok else "SOME WAYPOINTS FAILED"
        self.get_logger().info(f"\n{'='*60}\n  {result}\n{'='*60}")

    def _navigate_to(self, label: str,
                     x: float, y: float, yaw: float) -> bool:
        """Send a NavigateToPose goal and wait for the result.
        Returns True if error_code == 0 (success).
        Retries up to MAX_RETRIES times if rejection or abort."""
        for attempt in range(1, MAX_RETRIES + 1):
            self.get_logger().info(
                f"    Attempt {attempt}/{MAX_RETRIES} ..."
            )

            # STEP 3: send_goal_async + spin until accepted/rejected.
            goal_msg = build_goal(x, y, yaw, self)
            gf = self._ac.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, gf, timeout_sec=15.0)

            if not gf.done():
                self.get_logger().warn("    Goal acceptance timed out.")
                time.sleep(REJECT_RETRY_DELAY)
                continue

            gh = gf.result()
            if gh is None or not gh.accepted:
                self.get_logger().warn(
                    f"    Goal REJECTED (Nav2 may be in cleanup). "
                    f"Waiting {REJECT_RETRY_DELAY:.0f} s ..."
                )
                time.sleep(REJECT_RETRY_DELAY)
                continue

            self.get_logger().info("    Goal ACCEPTED. Driving ...")

            # STEP 4-5: get_result_async + spin until navigation finishes.
            rf = gh.get_result_async()
            rclpy.spin_until_future_complete(
                self, rf, timeout_sec=GOAL_TIMEOUT
            )

            if not rf.done():
                self.get_logger().warn(
                    f"    Timed out after {GOAL_TIMEOUT:.0f} s. Cancelling."
                )
                gh.cancel_goal_async()
                time.sleep(ABORT_RETRY_DELAY)
                continue

            # STEP 6: check error_code.
            status = rf.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                return True   # error_code == 0 → move to next sub-goal

            self.get_logger().warn(
                f"    Nav2 status={status} (non-zero). "
                f"Waiting {ABORT_RETRY_DELAY:.0f} s then retrying ..."
            )
            time.sleep(ABORT_RETRY_DELAY)

        return False


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigatorNode()
    try:
        node.run()
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
