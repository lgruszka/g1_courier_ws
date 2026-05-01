"""Bridges our `g1_courier_msgs/NavigateToPose` action onto nav2's
`nav2_msgs/NavigateToPose`. Decouples the rest of the stack from nav2 details
(BT navigators, recovery behaviors, etc) and lets us add waypoint logging,
custom timeouts and stop-on-cancel without forking nav2.
"""
from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose as Nav2Goal
from g1_courier_msgs.action import NavigateToPose as CourierGoal


class NavigateProxy(Node):
    def __init__(self) -> None:
        super().__init__('navigate_proxy')
        self._client = ActionClient(self, Nav2Goal, 'navigate_to_pose')
        self._busy = threading.Lock()
        self._server = ActionServer(
            self, CourierGoal, '/courier/navigate_to_pose',
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )
        self.get_logger().info('navigate_proxy ready on /courier/navigate_to_pose')

    def _goal_cb(self, _request) -> GoalResponse:
        # Busy check happens in _execute via acquire(blocking=False); see ARCH §11.
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _gh) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute(self, gh: ServerGoalHandle) -> CourierGoal.Result:
        result = CourierGoal.Result()
        if not self._busy.acquire(blocking=False):
            gh.abort()
            result.success = False
            result.message = 'already busy'
            return result
        try:
            if not self._client.wait_for_server(timeout_sec=5.0):
                gh.abort()
                result.success = False
                result.message = 'nav2 navigate_to_pose not available'
                return result

            inner = Nav2Goal.Goal()
            inner.pose = gh.request.target_pose
            send_future = self._client.send_goal_async(
                inner,
                feedback_callback=lambda f: self._on_nav_feedback(gh, f),
            )
            # Poll futures - never call rclpy.spin* from inside an execute_callback;
            # the MultiThreadedExecutor in main() runs callbacks on other threads.
            while not send_future.done():
                time.sleep(0.05)
            handle = send_future.result()
            if handle is None or not handle.accepted:
                gh.abort()
                result.success = False
                result.message = 'nav2 rejected goal'
                return result

            result_future = handle.get_result_async()
            while not result_future.done():
                if gh.is_cancel_requested:
                    handle.cancel_goal_async()
                    gh.canceled()
                    result.success = False
                    result.message = 'cancelled'
                    return result
                time.sleep(0.1)

            nav_result = result_future.result()
            if nav_result is None:
                gh.abort()
                result.success = False
                result.message = 'nav2 returned no result'
                return result

            result.success = True
            result.message = f'arrived at {gh.request.waypoint_name}'
            result.final_pose = gh.request.target_pose
            gh.succeed()
            return result
        finally:
            self._busy.release()

    def _on_nav_feedback(self, gh: ServerGoalHandle, fb_msg) -> None:
        try:
            distance = float(fb_msg.feedback.distance_remaining)
        except (AttributeError, ValueError):
            distance = math.nan
        out = CourierGoal.Feedback()
        out.distance_remaining_m = distance
        out.phase = 'navigating'
        gh.publish_feedback(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigateProxy()
    # MultiThreadedExecutor is required: this node hosts both an action server
    # and an action client - single-threaded would deadlock on inner futures.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
