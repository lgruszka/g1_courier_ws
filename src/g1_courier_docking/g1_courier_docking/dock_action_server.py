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
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import CameraInfo, LaserScan

from g1_courier_msgs.action import DockToTable

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
    APRILTAG_OK = True
except ImportError:
    AprilTagDetectionArray = None  # type: ignore[assignment]
    APRILTAG_OK = False

try:
    import cv2
    import numpy as np
    CV2_OK = True
except ImportError:
    CV2_OK = False


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
        # yaw_err is the angle between robot's +X (forward) and the line's
        # inward normal (a, b). When line normal points "to the right"
        # (yaw_err < 0), the robot must rotate CW (negative angular.z) to
        # face it — i.e. cmd.angular.z and yaw_err have the *same* sign so
        # the correction damps the error instead of amplifying it.
        yaw_err = math.atan2(b, a)
        cmd = Twist()
        cmd.linear.x = _clamp(self.kp_xy * dx, -self.max_vx, self.max_vx)
        cmd.angular.z = _clamp(self.kp_yaw * yaw_err, -self.max_vyaw, self.max_vyaw)
        return cmd, AlignError(xy_m=abs(dx), yaw_rad=abs(yaw_err))

    def _scan_to_points(self, scan: LaserScan) -> list[tuple[float, float]]:
        pts: list[tuple[float, float]] = []
        a = scan.angle_min
        # Bound by the aligner's window AND the scan's own physical range bounds.
        lo_r = max(self.range_min, scan.range_min if scan.range_min > 0 else self.range_min)
        hi_r = min(self.range_max, scan.range_max if scan.range_max > 0 else self.range_max)
        for r in scan.ranges:
            # Wrap unsigned angles (mac publishes 0..2π) into the signed range
            # (-π..π) that the forward window expects. Otherwise filter would
            # only match the right half [0, +π/6] and miss [-π/6, 0].
            a_signed = a if a <= math.pi else a - 2.0 * math.pi
            if math.isfinite(r) and lo_r < r < hi_r and self.angle_min <= a_signed <= self.angle_max:
                pts.append((r * math.cos(a_signed), r * math.sin(a_signed)))
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
        self.declare_parameter('apriltag.yaw_deadband_rad', 0.10)
        self.declare_parameter('apriltag.target_distance_m', 0.55)
        self.declare_parameter('apriltag.tag_size_m', 0.16)
        self.declare_parameter('camera_info_topic', '/camera_info')
        # Fallback intrinsics — used until a CameraInfo publisher appears.
        # Set fx<=0 to disable and require CameraInfo before any PnP attempt.
        self.declare_parameter('apriltag.fx', 0.0)
        self.declare_parameter('apriltag.fy', 0.0)
        self.declare_parameter('apriltag.cx', 0.0)
        self.declare_parameter('apriltag.cy', 0.0)
        self.declare_parameter('lidar.target_distance_m', 0.55)
        self.declare_parameter('lidar.kp_xy', 0.6)
        self.declare_parameter('lidar.kp_yaw', 1.0)
        self.declare_parameter('lidar.max_vx', 0.12)
        self.declare_parameter('lidar.max_vyaw', 0.3)
        # Forward window for RANSAC line fit. range_min must be < target_distance
        # so the table edge stays in the window even at convergence — otherwise
        # the aligner filters out the table when robot is closer than range_min,
        # returns inf err, robot stops short of the goal.
        self.declare_parameter('lidar.range_min_m', 0.05)
        self.declare_parameter('lidar.range_max_m', 2.5)

        cmd_topic = str(self.get_parameter('cmd_vel_topic').value)
        self._rate_hz = max(1.0, float(self.get_parameter('control_rate_hz').value))
        self._settle_samples = int(self.get_parameter('settle_samples').value)

        self._cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        self._tag_lock = threading.Lock()
        self._latest_tags = None
        self._camera_K = None

        if APRILTAG_OK:
            self._tag_sub = self.create_subscription(
                AprilTagDetectionArray,
                str(self.get_parameter('apriltag_topic').value),
                self._on_tag, qos_profile_sensor_data,
            )
            # CameraInfo is conventionally latched: the publisher publishes
            # once with TRANSIENT_LOCAL, late subscribers still receive it.
            # Match with reliable + transient_local on our side.
            cam_info_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self._cam_info_sub = self.create_subscription(
                CameraInfo,
                str(self.get_parameter('camera_info_topic').value),
                self._on_camera_info, cam_info_qos,
            )
            if not CV2_OK:
                self.get_logger().warn(
                    'cv2/numpy not available - APRILTAG mode pose extraction disabled.'
                )
            elif self._seed_intrinsics_from_params():
                self.get_logger().info(
                    'using fallback pinhole intrinsics from apriltag.{fx,fy,cx,cy}; '
                    'CameraInfo will override when received.'
                )
        else:
            self._tag_sub = None
            self._cam_info_sub = None
            self.get_logger().warn('apriltag_msgs not available - APRILTAG mode disabled.')

        self._scan_sub = self.create_subscription(
            LaserScan, str(self.get_parameter('scan_topic').value),
            self._on_scan, qos_profile_sensor_data,
        )

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

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if not CV2_OK:
            return
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        with self._tag_lock:
            self._camera_K = K

    def _seed_intrinsics_from_params(self) -> bool:
        if not CV2_OK:
            return False
        fx = float(self.get_parameter('apriltag.fx').value)
        fy = float(self.get_parameter('apriltag.fy').value)
        cx = float(self.get_parameter('apriltag.cx').value)
        cy = float(self.get_parameter('apriltag.cy').value)
        if fx <= 0.0 or fy <= 0.0:
            return False
        K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
                     dtype=np.float64)
        with self._tag_lock:
            self._camera_K = K
        return True

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
        no_det_count = 0   # consecutive ticks without a valid tag detection
        no_det_recovery_threshold = max(5, int(self._rate_hz))   # ~1 s @ 20 Hz
        # Per-call target_distance for log readability — same resolution as
        # _extract_tag_residual uses internally.
        log_target_m = (
            float(request.target_pose.pose.position.z)
            if request.target_pose.pose.position.z > 0.0
            else float(self.get_parameter('apriltag.target_distance_m').value)
        )
        log_tick = 0

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
                no_det_count += 1
                if no_det_count >= no_det_recovery_threshold:
                    # Tag lost for >1 s — back up slowly to recover FoV. Robot
                    # was likely too close (tag at FoV edge) or laterally
                    # offset (tag cropped). Backing up shrinks tag in image
                    # so corners come into view, dock loop resumes.
                    recovery_cmd = Twist()
                    recovery_cmd.linear.x = -0.05   # 5 cm/s backwards
                    self._cmd_pub.publish(recovery_cmd)
                    if no_det_count % 10 == 0:
                        self.get_logger().info(
                            f'[dock_apriltag tag={request.apriltag_id}] no detection '
                            f'for {no_det_count} ticks, recovering (vx=-0.05)')
                else:
                    self._publish_zero()
                time.sleep(period)
                continue
            no_det_count = 0   # reset on successful detection
            # Deadband on yaw — suppresses PnP/IPPE noise and 2-fold ambiguity
            # flip (±0.6 rad spurious jumps) when robot is essentially aligned.
            # Real rotations >threshold pass through unchanged.
            yaw_deadband = float(self.get_parameter('apriltag.yaw_deadband_rad').value)
            if abs(dyaw) < yaw_deadband:
                dyaw = 0.0

            cmd, err = aligner.step(dx, dy, dyaw)
            self._cmd_pub.publish(cmd)
            self._publish_feedback(goal_handle, 'servoing', err)

            # Throttled servo log — every 5 ticks (~4 Hz @ rate 20). Shows
            # current z_c, target, dx, err.xy and the published cmd so user
            # can see live why the dock does/doesn't converge.
            log_tick += 1
            if log_tick % 5 == 0:
                z_c = dx + log_target_m  # dx = z_c - target → z_c = dx + target
                self.get_logger().info(
                    f'[dock_apriltag tag={request.apriltag_id}] '
                    f'z_c={z_c:.3f} target={log_target_m:.3f} '
                    f'dx={dx:+.3f} dy={dy:+.3f} dyaw={dyaw:+.3f} '
                    f'err.xy={err.xy_m:.3f} err.yaw={err.yaw_rad:.3f} '
                    f'cmd.vx={cmd.linear.x:+.3f} cmd.vy={cmd.linear.y:+.3f} '
                    f'cmd.wz={cmd.angular.z:+.3f} '
                    f'in_tol={within_tol_count}/{self._settle_samples}'
                )

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
        """Return (dx, dy, dyaw) in robot frame, or (None, None, None) if no fix.

        Pipeline:
          1. Pull the latest detection matching `tag_id` (4 image corners).
          2. solvePnP against a known-size square in tag frame to get tvec, rvec
             of the tag in OpenCV camera optical frame (X right, Y down, Z fwd).
          3. Map to REP-103 base_link assuming the camera is rigidly mounted
             facing forward (camera Z = robot X, camera X = -robot Y).
             For the welded-pelvis sim this rigid identity holds; on the real
             robot replace with a tf2 lookup of camera_optical_frame -> base_link.
          4. dx is forward overshoot vs target_distance, dy is lateral, dyaw is
             rotation about robot Z extracted from the tag's normal direction.

        `goal_pose.pose.position.z` overrides target_distance for this call —
        used by mission BT to dock at different distances per tag (table tag
        at 0.30 m, box tag10 at 0.17 m for palm-press alignment). Lateral
        offset (dx, dy in tag frame) and orientation are still always "centred
        and facing"; multi-pose side-approach can be added later.
        """
        if not CV2_OK:
            return (None, None, None)
        with self._tag_lock:
            msg = self._latest_tags
            K = self._camera_K
        if msg is None or K is None:
            return (None, None, None)
        # Allow per-call override of target_distance via the goal's target_pose.
        # Mission BT uses this to dock at different distances for different tags
        # (e.g. 0.30 m from table tag5/7 for table-dock, 0.17 m from box
        # tag10 for box-dock). Falls back to config default when goal sends
        # an empty pose.
        if goal_pose is not None and goal_pose.pose.position.z > 0.0:
            target_distance = float(goal_pose.pose.position.z)
        else:
            target_distance = float(self.get_parameter('apriltag.target_distance_m').value)
        tag_size_m = float(self.get_parameter('apriltag.tag_size_m').value)
        s = tag_size_m / 2.0
        # OpenCV SOLVEPNP_IPPE_SQUARE requires obj_pts in TL,TR,BR,BL order
        # within a tag frame where +X is right and +Y is up. apriltag_msgs
        # publishes corners CCW starting from the tag's BL — so we reorder
        # corners[3,2,1,0] to match.
        obj_pts = np.array([
            [-s,  s, 0.0],   # TL
            [ s,  s, 0.0],   # TR
            [ s, -s, 0.0],   # BR
            [-s, -s, 0.0],   # BL
        ], dtype=np.float64)
        for det in getattr(msg, 'detections', []):
            if int(getattr(det, 'id', -1)) != int(tag_id):
                continue
            corners = getattr(det, 'corners', None)
            if corners is None or len(corners) != 4:
                return (None, None, None)
            # NOTE: previously rejected detections with corners at image edges,
            # but that created a "dead zone" — when nav left robot with lateral
            # offset (xy_goal_tolerance=0.20), tag corners at FoV edge were
            # rejected, aligner published 0, robot never moved to recover.
            # Now: trust IPPE solver even with partial cropping. PnP is robust
            # to one corner near edge (reprojection error increases but tvec
            # roughly OK). Aligner has time to servo robot back into clean view.
            img_pts = np.array([
                (float(corners[3].x), float(corners[3].y)),  # TL
                (float(corners[2].x), float(corners[2].y)),  # TR
                (float(corners[1].x), float(corners[1].y)),  # BR
                (float(corners[0].x), float(corners[0].y)),  # BL
            ], dtype=np.float64)
            # IPPE_SQUARE has a 2-fold ambiguity for planar markers (the tag's
            # normal can be flipped). Get both candidates and pick the one
            # where the tag faces back toward the camera (its local +Z lies
            # in the -Z_camera half-space, i.e. R[2,2] < 0). Without this we
            # see a 180° dyaw at the facing-on singular point.
            n_sols, rvecs, tvecs, _ = cv2.solvePnPGeneric(
                obj_pts, img_pts, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if n_sols == 0:
                return (None, None, None)
            rvec, tvec, R = None, None, None
            for cand_rvec, cand_tvec in zip(rvecs, tvecs):
                cand_R, _ = cv2.Rodrigues(cand_rvec)
                if cand_R[2, 2] < 0:
                    rvec, tvec, R = cand_rvec, cand_tvec, cand_R
                    break
            if R is None:
                # Both IPPE candidates picked the "tag faces away" branch —
                # happens at the facing-on singular point. Force-flip 180°
                # about the tag's X axis so the outward normal points back
                # toward the camera. tvec is unchanged (same physical pose).
                rvec, tvec = rvecs[0], tvecs[0]
                R, _ = cv2.Rodrigues(rvec)
                R = R @ np.diag([1.0, -1.0, -1.0])
            x_c, y_c, z_c = (float(v) for v in tvec.flatten())
            dx_robot = z_c - target_distance
            dy_robot = -x_c
            # Tag's local +Z (out-of-plane normal) in camera frame is R[:, 2].
            # When the tag faces the camera squarely, that vector points back
            # along -Z_cam, so R[2,2] ≈ -1. Yaw correction (sign convention:
            # positive = robot needs to rotate CCW) is computed so that:
            #   - tag rotated CCW relative to camera ⇒ dyaw > 0 (rotate to align)
            #   - robot drifted CCW with tag fixed   ⇒ dyaw < 0 (rotate back)
            # Both reduce to atan2(R[0,2], -R[2,2]) given R[2,2]<0 after
            # disambiguation above.
            dyaw = math.atan2(R[0, 2], -R[2, 2])
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
            range_min_m=float(self.get_parameter('lidar.range_min_m').value),
            range_max_m=float(self.get_parameter('lidar.range_max_m').value),
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
