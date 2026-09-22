"""Headless system test: send a NavigateToPose goal and verify the robot autonomously
reaches it. Exits non-zero on failure (safety/arrival criteria)."""
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node

GOAL_X, GOAL_Y = 2.5, 2.5      # map frame (== odom here); ~3.5 m diagonal from start
ARRIVAL_RADIUS = 0.5           # metres
WALL_TIMEOUT = 90.0            # seconds


class NavSmoke(Node):
    def __init__(self):
        super().__init__("nav_to_goal_smoke")
        self.pos = None
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def _odom(self, msg):
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)


def main():
    rclpy.init()
    n = NavSmoke()

    if not n.client.wait_for_server(timeout_sec=40.0):
        print("NAV_FAIL: /navigate_to_pose action server not available in 40s")
        rclpy.shutdown()
        raise SystemExit(2)

    # Wait for first odom.
    t0 = time.time()
    while n.pos is None and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.pos is None:
        print("NAV_FAIL: no /odom")
        rclpy.shutdown()
        raise SystemExit(2)
    print(f"START odom x={n.pos[0]:.2f} y={n.pos[1]:.2f}; goal=({GOAL_X},{GOAL_Y})")

    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x = float(GOAL_X)
    goal.pose.pose.position.y = float(GOAL_Y)
    goal.pose.pose.orientation.w = 1.0

    send_future = n.client.send_goal_async(goal)
    rclpy.spin_until_future_complete(n, send_future, timeout_sec=10.0)
    gh = send_future.result()
    if gh is None or not gh.accepted:
        print("NAV_FAIL: goal not accepted")
        rclpy.shutdown()
        raise SystemExit(3)
    print("goal accepted; navigating...")
    result_future = gh.get_result_async()

    best = 1e9
    t0 = time.time()
    last_print = 0.0
    while time.time() - t0 < WALL_TIMEOUT:
        rclpy.spin_once(n, timeout_sec=0.1)
        d = math.hypot(n.pos[0] - GOAL_X, n.pos[1] - GOAL_Y)
        best = min(best, d)
        if time.time() - last_print > 5.0:
            print(f"  t={time.time()-t0:5.1f}s  pos=({n.pos[0]:.2f},{n.pos[1]:.2f})  dist={d:.2f}")
            last_print = time.time()
        if d < ARRIVAL_RADIUS:
            print(f"NAV_PASS: reached goal (dist={d:.2f} m at t={time.time()-t0:.1f}s)")
            rclpy.shutdown()
            raise SystemExit(0)
        if result_future.done():
            status = result_future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED and d < ARRIVAL_RADIUS:
                print("NAV_PASS: action succeeded")
                rclpy.shutdown()
                raise SystemExit(0)

    print(f"NAV_FAIL: did not reach goal in {WALL_TIMEOUT}s (best dist={best:.2f} m)")
    rclpy.shutdown()
    raise SystemExit(4)


main()
