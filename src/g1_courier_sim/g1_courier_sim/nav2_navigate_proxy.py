"""nav2 adapter — translates our /courier/navigate_to_pose goals into
nav2's /navigate_to_pose action.

Mission BT (and behaviors.NavigateTo) keep using g1_courier_msgs.NavigateToPose
on /courier/navigate_to_pose, so the rest of the stack stays modular —
swap this node out for kinematic_nav_node or any future backend without
touching mission logic.

Forwarding semantics:
  - target_pose, timeout_s pass straight through.
  - xy_tolerance_m / yaw_tolerance_rad: nav2 BT navigator doesn't accept
    per-goal tolerances; nav2_params.yaml controls them globally
    (controller_server.goal_checker). We log a notice if non-default values
    are supplied so the operator can edit nav2_params.yaml instead.
  - waypoint_name: not used by nav2; we put it in result.message for traceability.
  - cancel: forward to nav2 via cancel_goal_async.
"""
from __future__ import annotations

import threading
import time
from typing import Optional

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from g1_courier_msgs.action import NavigateToPose as G1NavigateToPose
from nav2_msgs.action import NavigateToPose as Nav2NavigateToPose


class Nav2NavigateProxy(Node):
    def __init__(self) -> None:
        super().__init__('nav2_navigate_proxy')

        self.declare_parameter('inbound_action_name', '/courier/navigate_to_pose')
        self.declare_parameter('outbound_action_name', '/navigate_to_pose')

        self._inbound_name = str(self.get_parameter('inbound_action_name').value)
        self._outbound_name = str(self.get_parameter('outbound_action_name').value)

        self._client = ActionClient(self, Nav2NavigateToPose, self._outbound_name)

        self._busy_lock = threading.Lock()
        self._cancel_event = threading.Event()
        self._action_server = ActionServer(
            self, G1NavigateToPose, self._inbound_name,
            execute_callback=self._execute,
            goal_callback=lambda _req: GoalResponse.ACCEPT,
            cancel_callback=self._cancel,
        )
        self.get_logger().info(
            f'nav2_navigate_proxy ready '
            f'(in={self._inbound_name}, out={self._outbound_name})'
        )

    def _cancel(self, _gh) -> CancelResponse:
        self._cancel_event.set()
        return CancelResponse.ACCEPT

    def _execute(self, gh: ServerGoalHandle) -> G1NavigateToPose.Result:
        result = G1NavigateToPose.Result()
        if not self._busy_lock.acquire(blocking=False):
            gh.abort()
            result.success = False
            result.message = 'nav2_navigate_proxy already busy'
            return result

        self._cancel_event.clear()
        request = gh.request

        try:
            if not self._client.wait_for_server(timeout_sec=5.0):
                gh.abort()
                result.success = False
                result.message = 'nav2 /navigate_to_pose action server unavailable'
                return result

            # Notify operator if per-goal tolerances were supplied — nav2 takes
            # them globally from nav2_params.yaml, not from the goal message.
            if request.xy_tolerance_m > 0 or request.yaw_tolerance_rad > 0:
                self.get_logger().info(
                    f'note: nav2 ignores per-goal tolerances; got '
                    f'xy={request.xy_tolerance_m:.3f} yaw={request.yaw_tolerance_rad:.3f}, '
                    f'edit nav2_params.yaml goal_checker instead'
                )

            nav2_goal = Nav2NavigateToPose.Goal()
            nav2_goal.pose = request.target_pose
            nav2_goal.behavior_tree = ''  # use nav2 default BT

            send_future = self._client.send_goal_async(nav2_goal)
            self._spin_until_done(send_future)
            handle = send_future.result()
            if handle is None or not handle.accepted:
                gh.abort()
                result.success = False
                result.message = 'nav2 rejected goal'
                return result

            result_future = handle.get_result_async()
            deadline = (time.monotonic() + float(request.timeout_s)
                        if request.timeout_s > 0 else float('inf'))

            while not result_future.done():
                if self._cancel_event.is_set():
                    handle.cancel_goal_async()
                    gh.canceled()
                    result.success = False
                    result.message = 'cancelled'
                    return result
                if time.monotonic() >= deadline:
                    handle.cancel_goal_async()
                    gh.abort()
                    result.success = False
                    result.message = 'nav2_navigate_proxy timeout'
                    return result
                time.sleep(0.05)

            outcome = result_future.result()
            # nav2 result.error_code == 0 is "no error" (success).
            err_code = int(getattr(outcome.result, 'error_code', 0))
            if err_code == 0:
                result.success = True
                result.message = (f'nav2 done {request.waypoint_name}'
                                  if request.waypoint_name else 'nav2 done')
                gh.succeed()
            else:
                result.success = False
                result.message = f'nav2 error_code={err_code}'
                gh.abort()
            return result
        finally:
            self._busy_lock.release()

    def _spin_until_done(self, future) -> None:
        # Action server callbacks block their executor thread; spin our own
        # tiny event loop until the upstream future resolves.
        while not future.done():
            if self._cancel_event.is_set():
                return
            time.sleep(0.02)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2NavigateProxy()
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
