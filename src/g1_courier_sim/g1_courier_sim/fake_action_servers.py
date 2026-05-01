"""Phase 0 fake action servers — let the mission BT run end-to-end without
the real nav2/dock/arm stack.

Each fake accepts any goal, sleeps `duration_s`, then returns success or
failure based on the `succeed` parameter. The BT's behaviours only read
`result.success` and `result.message`, so this is enough to exercise:
  - happy-path A->B->A cycles,
  - cancel propagation (sleep loop checks `is_cancel_requested`),
  - failure branches (set `succeed:=false` from launch).

One generic class, one entry point per action — invoked as e.g.
  `ros2 run g1_courier_sim fake_navigate_proxy --ros-args -p duration_s:=3.0`
"""
from __future__ import annotations

import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from g1_courier_msgs.action import (
    DockToTable, NavigateToPose, PickBox, PlaceBox, Retreat,
)


class _FakeActionServer(Node):
    def __init__(self, node_name: str, action_type, action_name: str,
                 default_duration_s: float) -> None:
        super().__init__(node_name)
        self._action_type = action_type
        self._action_name = action_name
        self.declare_parameter('duration_s', default_duration_s)
        self.declare_parameter('succeed', True)
        self._busy = threading.Lock()
        self._server = ActionServer(
            self, action_type, action_name,
            execute_callback=self._execute,
            goal_callback=lambda _req: GoalResponse.ACCEPT,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
        )
        self.get_logger().info(f'fake action server ready on {action_name}')

    def _execute(self, gh: ServerGoalHandle):
        result = self._action_type.Result()
        if not self._busy.acquire(blocking=False):
            gh.abort()
            result.success = False
            result.message = f'{self._action_name} fake busy'
            return result
        try:
            duration = max(0.0, float(self.get_parameter('duration_s').value))
            succeed = bool(self.get_parameter('succeed').value)
            # Step-sleep so cancel can preempt promptly (ARCH §sleep rule).
            step = 0.05
            elapsed = 0.0
            while elapsed < duration:
                if gh.is_cancel_requested:
                    gh.canceled()
                    result.success = False
                    result.message = f'{self._action_name} fake cancelled'
                    return result
                time.sleep(step)
                elapsed += step
            if succeed:
                gh.succeed()
                result.success = True
                result.message = f'{self._action_name} fake done'
            else:
                gh.abort()
                result.success = False
                result.message = f'{self._action_name} fake forced-fail'
            return result
        finally:
            self._busy.release()


def _run(node_name: str, action_type, action_name: str, default_duration_s: float) -> None:
    rclpy.init()
    node = _FakeActionServer(node_name, action_type, action_name, default_duration_s)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_navigate() -> None:
    _run('fake_navigate_proxy', NavigateToPose, '/courier/navigate_to_pose', 2.0)


def main_dock() -> None:
    _run('fake_dock_action_server', DockToTable, '/dock_to_table', 2.0)


def main_pick() -> None:
    _run('fake_pick_action_server', PickBox, '/pick_box', 1.5)


def main_place() -> None:
    _run('fake_place_action_server', PlaceBox, '/place_box', 1.5)


def main_retreat() -> None:
    _run('fake_retreat_action_server', Retreat, '/retreat', 1.0)
