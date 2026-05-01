"""Open-loop retreat: drive backwards a distance at speed.

Uses a dedicated `/cmd_vel_retreat` topic so the cmd_vel arbiter can elevate it
above nav2 while retreating. Distance is integrated from elapsed time times
speed; tighten later by replacing with /odom integration if needed.
"""
from __future__ import annotations

import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node

from geometry_msgs.msg import Twist
from g1_courier_msgs.action import Retreat


class RetreatActionServer(Node):
    def __init__(self) -> None:
        super().__init__('retreat_action_server')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_retreat')
        self.declare_parameter('rate_hz', 20.0)
        self._cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10,
        )
        self._rate_hz = max(1.0, float(self.get_parameter('rate_hz').value))
        self._busy = threading.Lock()
        self._cancel = threading.Event()
        self._server = ActionServer(
            self, Retreat, '/retreat',
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )
        self.get_logger().info('retreat_action_server ready on /retreat')

    def _goal_cb(self, _r) -> GoalResponse:
        # Busy check happens in _execute via acquire(blocking=False); see ARCH §11.
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _gh) -> CancelResponse:
        self._cancel.set()
        return CancelResponse.ACCEPT

    def _execute(self, gh: ServerGoalHandle) -> Retreat.Result:
        result = Retreat.Result()
        if not self._busy.acquire(blocking=False):
            gh.abort()
            result.success = False
            result.message = 'already busy'
            return result
        self._cancel.clear()
        try:
            req = gh.request
            speed = max(0.01, abs(float(req.speed_mps)))
            target = max(0.0, float(req.distance_m))
            timeout = float(req.timeout_s) if req.timeout_s > 0 else target / speed * 2.0 + 1.0
            deadline = time.monotonic() + timeout

            cmd = Twist()
            cmd.linear.x = -speed
            traveled = 0.0
            period = 1.0 / self._rate_hz
            t0 = time.monotonic()
            while True:
                if self._cancel.is_set():
                    gh.canceled()
                    result.success = False
                    result.message = 'cancelled'
                    return result
                now = time.monotonic()
                traveled = (now - t0) * speed
                if traveled >= target:
                    break
                if now >= deadline:
                    gh.abort()
                    result.success = False
                    result.message = 'timeout'
                    result.distance_traveled_m = float(traveled)
                    return result
                self._cmd_pub.publish(cmd)
                fb = Retreat.Feedback()
                fb.distance_traveled_m = float(traveled)
                gh.publish_feedback(fb)
                time.sleep(period)

            self._cmd_pub.publish(Twist())
            result.success = True
            result.message = 'retreat complete'
            result.distance_traveled_m = float(traveled)
            gh.succeed()
            return result
        finally:
            self._cmd_pub.publish(Twist())
            self._busy.release()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RetreatActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
