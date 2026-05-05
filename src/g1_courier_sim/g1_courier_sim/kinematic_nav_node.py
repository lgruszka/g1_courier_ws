"""Kinematic nav node — sim-only nav2 replacement.

Drives the robot to a 2D pose with a per-tick P-controller in the robot
body frame. Subscribes /odom (from sim_cmd_vel_bridge_node, which is the
same kinematic integrator the mac mocap mirrors) and publishes /cmd_vel_nav.
Settles when within tolerance for `settle_samples` consecutive ticks.

Drop-in replacement for fake_navigate_proxy when we want the robot to
actually traverse A<->B in MuJoCo: mac kinematic mocap integrates the
arbiter's /cmd_vel into pelvis_anchor mocap_pos, the welded pelvis slides,
and the head_cam moves so dock APRILTAG can converge against the second
table's tag.

Real-robot path uses nav2 instead; this node is only wired in
phase1_smoke.launch.py and similar sim launches.
"""
from __future__ import annotations

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from g1_courier_msgs.action import NavigateToPose


def _quat_to_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class KinematicNavNode(Node):
    def __init__(self) -> None:
        super().__init__('kinematic_nav_node')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('action_name', '/courier/navigate_to_pose')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('settle_samples', 5)
        self.declare_parameter('xy_tolerance_m_default', 0.05)
        self.declare_parameter('yaw_tolerance_rad_default', 0.10)
        self.declare_parameter('kp_xy', 0.5)
        self.declare_parameter('kp_yaw', 1.0)
        # Caps below are local; the arbiter applies its own normal/carry
        # caps on /cmd_vel afterwards. Kept conservative so the mac mocap
        # integrator (which competes for GIL with the lowcmd handler and
        # camera renderer) physically tracks the Linux odom integrator —
        # otherwise this node declares "converged" before the robot
        # actually arrived, and place tries to drop the parcel mid-air.
        self.declare_parameter('max_vx', 0.3)
        self.declare_parameter('max_vy', 0.2)
        self.declare_parameter('max_vyaw', 0.6)

        self._rate_hz = max(1.0, float(self.get_parameter('control_rate_hz').value))
        self._period = 1.0 / self._rate_hz
        self._settle_samples = int(self.get_parameter('settle_samples').value)
        self._xy_tol_default = float(self.get_parameter('xy_tolerance_m_default').value)
        self._yaw_tol_default = float(self.get_parameter('yaw_tolerance_rad_default').value)
        self._kp_xy = float(self.get_parameter('kp_xy').value)
        self._kp_yaw = float(self.get_parameter('kp_yaw').value)
        self._max_vx = float(self.get_parameter('max_vx').value)
        self._max_vy = float(self.get_parameter('max_vy').value)
        self._max_vyaw = float(self.get_parameter('max_vyaw').value)

        self._cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10,
        )
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value),
            self._on_odom, qos_profile_sensor_data,
        )

        self._odom_lock = threading.Lock()
        self._odom: Optional[Odometry] = None

        self._busy_lock = threading.Lock()
        self._cancel_event = threading.Event()
        action_name = str(self.get_parameter('action_name').value)
        self._action_server = ActionServer(
            self, NavigateToPose, action_name,
            execute_callback=self._execute,
            goal_callback=lambda _req: GoalResponse.ACCEPT,
            cancel_callback=self._cancel,
        )
        self.get_logger().info(
            f'kinematic_nav_node ready @ {self._rate_hz:.0f} Hz on {action_name}'
        )

    # ---------- callbacks ----------

    def _on_odom(self, msg: Odometry) -> None:
        with self._odom_lock:
            self._odom = msg

    def _cancel(self, _gh) -> CancelResponse:
        self._cancel_event.set()
        return CancelResponse.ACCEPT

    # ---------- main execute ----------

    def _execute(self, gh: ServerGoalHandle) -> NavigateToPose.Result:
        result = NavigateToPose.Result()
        if not self._busy_lock.acquire(blocking=False):
            gh.abort()
            result.success = False
            result.message = 'kinematic_nav already busy'
            return result

        self._cancel_event.clear()
        request = gh.request
        target = request.target_pose.pose
        target_x = float(target.position.x)
        target_y = float(target.position.y)
        target_yaw = _quat_to_yaw(target.orientation)
        xy_tol = (float(request.xy_tolerance_m)
                  if request.xy_tolerance_m > 0 else self._xy_tol_default)
        yaw_tol = (float(request.yaw_tolerance_rad)
                   if request.yaw_tolerance_rad > 0 else self._yaw_tol_default)
        deadline = (time.monotonic() + float(request.timeout_s)
                    if request.timeout_s > 0 else float('inf'))
        within_tol_count = 0
        self.get_logger().info(
            f'goal -> target=({target_x:+.2f}, {target_y:+.2f}, '
            f'{math.degrees(target_yaw):+.1f}°), tol={xy_tol:.2f} m / '
            f'{math.degrees(yaw_tol):.1f}°'
        )
        last_log_t = time.monotonic()

        try:
            while True:
                if self._cancel_event.is_set():
                    gh.canceled()
                    result.success = False
                    result.message = 'kinematic_nav cancelled'
                    return result
                if time.monotonic() >= deadline:
                    gh.abort()
                    result.success = False
                    result.message = 'kinematic_nav timeout'
                    return result

                with self._odom_lock:
                    odom = self._odom
                if odom is None:
                    self._cmd_pub.publish(Twist())
                    time.sleep(self._period)
                    continue

                cur_x = float(odom.pose.pose.position.x)
                cur_y = float(odom.pose.pose.position.y)
                cur_yaw = _quat_to_yaw(odom.pose.pose.orientation)

                # Position error in world frame, rotated into robot body frame
                # (P-controller acts on body-frame deltas, so a rotated robot
                # still drives in the right direction).
                ex_w = target_x - cur_x
                ey_w = target_y - cur_y
                cy, sy = math.cos(cur_yaw), math.sin(cur_yaw)
                ex_b =  cy * ex_w + sy * ey_w
                ey_b = -sy * ex_w + cy * ey_w
                eyaw = _wrap(target_yaw - cur_yaw)
                xy_err = math.hypot(ex_w, ey_w)

                cmd = Twist()
                cmd.linear.x  = _clamp(self._kp_xy * ex_b,   -self._max_vx,   self._max_vx)
                cmd.linear.y  = _clamp(self._kp_xy * ey_b,   -self._max_vy,   self._max_vy)
                cmd.angular.z = _clamp(self._kp_yaw * eyaw,  -self._max_vyaw, self._max_vyaw)
                self._cmd_pub.publish(cmd)

                # Settle / convergence check.
                if xy_err <= xy_tol and abs(eyaw) <= yaw_tol:
                    within_tol_count += 1
                    phase = 'settle'
                else:
                    within_tol_count = 0
                    phase = 'driving'

                fb = NavigateToPose.Feedback()
                fb.current_pose.header.stamp = self.get_clock().now().to_msg()
                fb.current_pose.header.frame_id = odom.header.frame_id
                fb.current_pose.pose = odom.pose.pose
                fb.distance_remaining_m = float(xy_err)
                fb.phase = phase
                gh.publish_feedback(fb)

                if within_tol_count >= self._settle_samples:
                    self.get_logger().info(
                        f'converged at ({cur_x:+.2f}, {cur_y:+.2f}, '
                        f'{math.degrees(cur_yaw):+.1f}°), xy_err={xy_err:.3f} m, '
                        f'yaw_err={math.degrees(eyaw):+.2f}°'
                    )
                    result.success = True
                    result.message = 'kinematic_nav converged'
                    result.final_pose.header.stamp = self.get_clock().now().to_msg()
                    result.final_pose.header.frame_id = odom.header.frame_id
                    result.final_pose.pose = odom.pose.pose
                    gh.succeed()
                    return result

                # Status log once per second to avoid log spam.
                now = time.monotonic()
                if now - last_log_t >= 1.0:
                    self.get_logger().info(
                        f'cur=({cur_x:+.2f}, {cur_y:+.2f}, '
                        f'{math.degrees(cur_yaw):+.1f}°)  '
                        f'remaining={xy_err:.2f} m, eyaw={math.degrees(eyaw):+.1f}°'
                    )
                    last_log_t = now

                time.sleep(self._period)
        finally:
            # Always brake on exit (success, abort, cancel) so the arbiter
            # doesn't keep an old non-zero nav cmd pinned for cmd_timeout_s.
            self._cmd_pub.publish(Twist())
            self._busy_lock.release()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KinematicNavNode()
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
