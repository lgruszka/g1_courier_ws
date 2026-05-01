"""DockToTable action server.

Three independent strategies share one action interface:
  MODE_APRILTAG     - 6-DoF visual servo on a known AprilTag (precise pick).
  MODE_LIDAR_LINE   - fit a line to the table edge in the 2D scan and align
                      perpendicular at a known offset (used when carrying box).
  MODE_AMCL_ONLY    - trust the global pose from AMCL, assume nav2 finished
                      close enough; just publish a final twist=0.

The action *publishes velocity* on a dedicated topic (default `/cmd_vel_dock`).
The cmd_vel_arbiter elevates this above nav2's `/cmd_vel_nav` so the dock
gets exclusive control while servoing.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan

from g1_courier_msgs.action import DockToTable

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
    APRILTAG_OK = True
except ImportError:
    AprilTagDetectionArray = None  # type: ignore[assignment]
    APRILTAG_OK = False


# ---------- aligners ----------

@dataclass
class AlignError:
    xy_m: float
    yaw_rad: float


class AprilTagAligner:
    """Closes the loop on a tag's 6-DoF pose, expressed in `base_link`.

    Target convention: `goal_pose` is the desired robot pose in the *tag* frame.
    We compute the residual robot-frame error and emit a Twist that decreases it.
    """

    def __init__(self, *, kp_xy: float, kp_yaw: float, max_vx: float, max_vyaw: float,
                 max_vy: float, target_xy_m: float, target_yaw_rad: float) -> None:
        self.kp_xy = kp_xy
        self.kp_yaw = kp_yaw
        self.max_vx = max_vx
        self.max_vy = max_vy
        self.max_vyaw = max_vyaw
        self.target_xy = target_xy_m
        self.target_yaw = target_yaw_rad

    def step(self, dx: float, dy: float, dyaw: float) -> tuple[Twist, AlignError]:
        # dx, dy: vector from robot to desired pose in robot frame.
        # dyaw: heading error (positive = robot needs to rotate CCW).
        cmd = Twist()
        cmd.linear.x = _clamp(self.kp_xy * dx, -self.max_vx, self.max_vx)
        cmd.linear.y = _clamp(self.kp_xy * dy, -self.max_vy, self.max_vy)
        cmd.angular.z = _clamp(self.kp_yaw * dyaw, -self.max_vyaw, self.max_vyaw)
        err = AlignError(xy_m=math.hypot(dx, dy), yaw_rad=abs(dyaw))
        return cmd, err


class LidarLineAligner:
    """Line fit + perpendicular alignment to a table edge in a 2D scan.

    Algorithm:
      1. Filter scan to a forward window (configurable angle/range bounds).
      2. RANSAC fit a 2D line to the resulting points.
      3. Express the line as `a*x + b*y + c = 0` with (a, b) the unit normal
         pointing FROM the robot TOWARD the line. `c` is then the signed
         distance from the robot to the line (positive when line is in front).
      4. Forward error: `dx = c - target_distance` (positive => too far).
      5. Heading error: `yaw_err = atan2(b, a)` — angle from +x (robot
         forward) to the line normal. Zero when the line is dead ahead
         and perpendicular.
      6. We have no lateral feature on a continuous line, so dy is held at 0.
    """

    def __init__(self, *, target_distance_m: float, kp_xy: float, kp_yaw: float,
                 max_vx: float, max_vyaw: float,
                 angle_min_rad: float = -math.pi / 6,
                 angle_max_rad: float = math.pi / 6,
                 range_min_m: float = 0.3,
                 range_max_m: float = 2.5,
                 ransac_iters: int = 80,
                 ransac_inlier_thresh_m: float = 0.02,
                 min_inliers: int = 8) -> None:
        self.target_distance = target_distance_m
        self.kp_xy = kp_xy
        self.kp_yaw = kp_yaw
        self.max_vx = max_vx
        self.max_vyaw = max_vyaw
        self.angle_min = angle_min_rad
        self.angle_max = angle_max_rad
        self.range_min = range_min_m
        self.range_max = range_max_m
        self.iters = ransac_iters
        self.inlier_thresh = ransac_inlier_thresh_m
        self.min_inliers = min_inliers

    def step(self, scan: LaserScan) -> tuple[Twist, AlignError]:
        pts = self._scan_to_points(scan)
        if len(pts) < self.min_inliers:
            return Twist(), AlignError(xy_m=float('inf'), yaw_rad=float('inf'))
        line = self._ransac(pts)
        if line is None:
            return Twist(), AlignError(xy_m=float('inf'), yaw_rad=float('inf'))
        a, b, c = line
        # Make (a, b) point FROM origin TOWARD the line. In the form
        # `a*x + b*y + c = 0`, evaluating at origin gives `c`; the normal
        # `(a, b)` points in the direction where this expression increases.
        # If `c > 0`, origin is on the +normal side, so (a, b) points AWAY
        # from the line — flip. After flip, |c| is the perpendicular
        # distance from origin to the line.
        if c > 0.0:
            a, b, c = -a, -b, -c
        distance_to_line = -c
        dx = distance_to_line - self.target_distance
        yaw_err = math.atan2(b, a)
        cmd = Twist()
        cmd.linear.x = _clamp(self.kp_xy * dx, -self.max_vx, self.max_vx)
        cmd.angular.z = _clamp(-self.kp_yaw * yaw_err, -self.max_vyaw, self.max_vyaw)
        return cmd, AlignError(xy_m=abs(dx), yaw_rad=abs(yaw_err))

    def _scan_to_points(self, scan: LaserScan) -> list[tuple[float, float]]:
        pts: list[tuple[float, float]] = []
        a = scan.angle_min
        # Bound by the aligner's window AND the scan's own physical range bounds.
        lo_r = max(self.range_min, scan.range_min if scan.range_min > 0 else self.range_min)
        hi_r = min(self.range_max, scan.range_max if scan.range_max > 0 else self.range_max)
        for r in scan.ranges:
            if math.isfinite(r) and lo_r < r < hi_r and self.angle_min <= a <= self.angle_max:
                pts.append((r * math.cos(a), r * math.sin(a)))
            a += scan.angle_increment
        return pts

    def _ransac(self, pts: list[tuple[float, float]]):
        import random
        n = len(pts)
        best_inliers = 0
        best: Optional[tuple[float, float, float]] = None
        for _ in range(self.iters):
            i, j = random.sample(range(n), 2)
            x1, y1 = pts[i]
            x2, y2 = pts[j]
            dx, dy = x2 - x1, y2 - y1
            norm = math.hypot(dx, dy)
            if norm < 1e-6:
                continue
            # Line through (x1,y1) and (x2,y2). Unit normal = (-dy, dx) / norm.
            na = -dy / norm
            nb = dx / norm
            nc = -(na * x1 + nb * y1)
            inliers = sum(1 for px, py in pts if abs(na * px + nb * py + nc) < self.inlier_thresh)
            if inliers > best_inliers:
                best_inliers = inliers
                best = (na, nb, nc)
        if best is None or best_inliers < self.min_inliers:
            return None
        return best


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ---------- node ----------

class DockActionServer(Node):
    def __init__(self) -> None:
        super().__init__('dock_action_server')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel_dock')
        self.declare_parameter('apriltag_topic', '/detections')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('settle_samples', 5)
        self.declare_parameter('apriltag.kp_xy', 0.6)
        self.declare_parameter('apriltag.kp_yaw', 1.2)
        self.declare_parameter('apriltag.max_vx', 0.15)
        self.declare_parameter('apriltag.max_vy', 0.10)
        self.declare_parameter('apriltag.max_vyaw', 0.4)
        self.declare_parameter('apriltag.target_distance_m', 0.55)
        self.declare_parameter('lidar.target_distance_m', 0.55)
        self.declare_parameter('lidar.kp_xy', 0.6)
        self.declare_parameter('lidar.kp_yaw', 1.0)
        self.declare_parameter('lidar.max_vx', 0.12)
        self.declare_parameter('lidar.max_vyaw', 0.3)

        cmd_topic = str(self.get_parameter('cmd_vel_topic').value)
        self._rate_hz = max(1.0, float(self.get_parameter('control_rate_hz').value))
        self._settle_samples = int(self.get_parameter('settle_samples').value)

        self._cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        if APRILTAG_OK:
            self._tag_sub = self.create_subscription(
                AprilTagDetectionArray,
                str(self.get_parameter('apriltag_topic').value),
                self._on_tag, qos_profile_sensor_data,
            )
        else:
            self._tag_sub = None
            self.get_logger().warn('apriltag_msgs not available - APRILTAG mode disabled.')

        self._scan_sub = self.create_subscription(
            LaserScan, str(self.get_parameter('scan_topic').value),
            self._on_scan, qos_profile_sensor_data,
        )

        self._tag_lock = threading.Lock()
        self._latest_tags = None
        self._scan_lock = threading.Lock()
        self._latest_scan: Optional[LaserScan] = None

        self._busy_lock = threading.Lock()
        self._action_server = ActionServer(
            self, DockToTable, '/dock_to_table',
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
        )
        self._cancel_event = threading.Event()
        self.get_logger().info(f'dock_action_server ready (cmd on {cmd_topic})')

    # ---------- callbacks ----------

    def _on_tag(self, msg) -> None:
        with self._tag_lock:
            self._latest_tags = msg

    def _on_scan(self, msg: LaserScan) -> None:
        with self._scan_lock:
            self._latest_scan = msg

    def _goal(self, _request) -> GoalResponse:
        # Busy check happens in _execute via acquire(blocking=False); see ARCH §11.
        return GoalResponse.ACCEPT

    def _cancel(self, _goal_handle) -> CancelResponse:
        self._cancel_event.set()
        return CancelResponse.ACCEPT

    # ---------- execution ----------

    def _execute(self, goal_handle: ServerGoalHandle) -> DockToTable.Result:
        result = DockToTable.Result()
        if not self._busy_lock.acquire(blocking=False):
            goal_handle.abort()
            result.success = False
            result.message = 'already busy'
            return result

        self._cancel_event.clear()
        request = goal_handle.request
        deadline = time.monotonic() + max(0.0, request.timeout_s) if request.timeout_s > 0 else float('inf')

        try:
            mode = request.mode
            if mode == DockToTable.Goal.MODE_APRILTAG:
                self._run_apriltag(goal_handle, request, result, deadline)
            elif mode == DockToTable.Goal.MODE_LIDAR_LINE:
                self._run_lidar(goal_handle, request, result, deadline)
            elif mode == DockToTable.Goal.MODE_AMCL_ONLY:
                self._run_amcl_only(goal_handle, result)
            else:
                goal_handle.abort()
                result.success = False
                result.message = f'unknown mode: {mode}'
                return result
        finally:
            self._publish_zero()
            self._busy_lock.release()
        return result

    # ---------- APRILTAG mode ----------

    def _run_apriltag(self, goal_handle, request, result, deadline) -> None:
        if not APRILTAG_OK:
            goal_handle.abort()
            result.success = False
            result.message = 'apriltag_msgs unavailable'
            return

        aligner = AprilTagAligner(
            kp_xy=float(self.get_parameter('apriltag.kp_xy').value),
            kp_yaw=float(self.get_parameter('apriltag.kp_yaw').value),
            max_vx=float(self.get_parameter('apriltag.max_vx').value),
            max_vy=float(self.get_parameter('apriltag.max_vy').value),
            max_vyaw=float(self.get_parameter('apriltag.max_vyaw').value),
            target_xy_m=float(self.get_parameter('apriltag.target_distance_m').value),
            target_yaw_rad=0.0,
        )
        period = 1.0 / self._rate_hz
        within_tol_count = 0

        while True:
            if self._cancel_event.is_set():
                goal_handle.canceled()
                result.success = False
                result.message = 'cancelled'
                return
            if time.monotonic() >= deadline:
                goal_handle.abort()
                result.success = False
                result.message = 'timeout'
                return

            dx, dy, dyaw = self._extract_tag_residual(request.apriltag_id, request.target_pose)
            if dx is None:
                # No detection this tick; brake briefly and continue.
                self._publish_zero()
                time.sleep(period)
                continue

            cmd, err = aligner.step(dx, dy, dyaw)
            self._cmd_pub.publish(cmd)
            self._publish_feedback(goal_handle, 'servoing', err)

            if err.xy_m <= request.xy_tolerance_m and err.yaw_rad <= request.yaw_tolerance_rad:
                within_tol_count += 1
                if within_tol_count >= self._settle_samples:
                    self._publish_zero()
                    self._publish_feedback(goal_handle, 'settle', err)
                    result.success = True
                    result.message = 'apriltag dock converged'
                    result.final_xy_error_m = err.xy_m
                    result.final_yaw_error_rad = err.yaw_rad
                    goal_handle.succeed()
                    return
            else:
                within_tol_count = 0
            time.sleep(period)

    def _extract_tag_residual(self, tag_id: int, goal_pose: PoseStamped):
        """Return (dx, dy, dyaw) of robot vs goal pose in tag frame.

        TODO: full implementation needs to (a) read the tag pose from
        AprilTagDetectionArray (or a dedicated tf frame), (b) transform `goal_pose`
        from the tag frame into base_link, (c) compute the residual.

        For the starter we use the simplification that the camera frame is the
        robot frame and that `goal_pose` is given in the tag frame as
        (target_x_in_tag = -target_distance, 0, 0) facing the tag - we read
        `pose.position` from the first matching detection and treat its
        Z (forward) as distance and X (lateral) as side error.
        """
        with self._tag_lock:
            msg = self._latest_tags
        if msg is None:
            return (None, None, None)
        for det in getattr(msg, 'detections', []):
            if int(getattr(det, 'id', -1)) != int(tag_id):
                continue
            # NOTE: this assumes the apriltag node publishes pose-bearing detections.
            # If it only publishes pixel centers, switch to /tf lookups.
            try:
                pos = det.pose.pose.pose.position  # apriltag_ros standard nesting
            except AttributeError:
                return (None, None, None)
            target = goal_pose.pose.position
            # robot_to_target in tag frame:
            dx_tag = target.x - pos.x
            dy_tag = target.y - pos.y
            # treat tag z (forward in camera) as robot x; tag x (lateral) as robot -y
            dx_robot = -dx_tag      # forward correction
            dy_robot = -dy_tag      # lateral correction
            dyaw = 0.0              # TODO: full yaw from quaternion
            return (dx_robot, dy_robot, dyaw)
        return (None, None, None)

    # ---------- LIDAR_LINE mode ----------

    def _run_lidar(self, goal_handle, request, result, deadline) -> None:
        aligner = LidarLineAligner(
            target_distance_m=float(self.get_parameter('lidar.target_distance_m').value),
            kp_xy=float(self.get_parameter('lidar.kp_xy').value),
            kp_yaw=float(self.get_parameter('lidar.kp_yaw').value),
            max_vx=float(self.get_parameter('lidar.max_vx').value),
            max_vyaw=float(self.get_parameter('lidar.max_vyaw').value),
        )
        period = 1.0 / self._rate_hz
        within_tol_count = 0

        while True:
            if self._cancel_event.is_set():
                goal_handle.canceled()
                result.success = False
                result.message = 'cancelled'
                return
            if time.monotonic() >= deadline:
                goal_handle.abort()
                result.success = False
                result.message = 'timeout'
                return

            with self._scan_lock:
                scan = self._latest_scan
            if scan is None:
                # No scan yet; brake briefly and continue.
                self._publish_zero()
                time.sleep(period)
                continue

            cmd, err = aligner.step(scan)
            if not math.isfinite(err.xy_m):
                # RANSAC could not find a line this tick — wait for a better scan.
                self._publish_zero()
                self._publish_feedback(goal_handle, 'searching', err)
                time.sleep(period)
                continue

            self._cmd_pub.publish(cmd)
            self._publish_feedback(goal_handle, 'servoing', err)

            if err.xy_m <= request.xy_tolerance_m and err.yaw_rad <= request.yaw_tolerance_rad:
                within_tol_count += 1
                if within_tol_count >= self._settle_samples:
                    self._publish_zero()
                    self._publish_feedback(goal_handle, 'settle', err)
                    result.success = True
                    result.message = 'lidar_line dock converged'
                    result.final_xy_error_m = err.xy_m
                    result.final_yaw_error_rad = err.yaw_rad
                    goal_handle.succeed()
                    return
            else:
                within_tol_count = 0
            time.sleep(period)

    # ---------- AMCL_ONLY mode ----------

    def _run_amcl_only(self, goal_handle, result) -> None:
        # Trivial: assume nav2 already converged. Just publish zero and report success.
        self._publish_zero()
        result.success = True
        result.message = 'amcl_only: trusted upstream localization'
        goal_handle.succeed()

    # ---------- helpers ----------

    def _publish_zero(self) -> None:
        self._cmd_pub.publish(Twist())

    def _publish_feedback(self, goal_handle, phase: str, err: AlignError) -> None:
        fb = DockToTable.Feedback()
        fb.phase = phase
        fb.xy_error_m = float(err.xy_m if err.xy_m != float('inf') else 1e6)
        fb.yaw_error_rad = float(err.yaw_rad if err.yaw_rad != float('inf') else 1e6)
        goal_handle.publish_feedback(fb)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DockActionServer()
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
