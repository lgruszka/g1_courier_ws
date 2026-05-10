#!/usr/bin/env python3
"""Live lidar scan viewer with dock_action_server overlay.

Subscribes /scan and renders top-down view of lidar points in robot frame.
Highlights:
  - Forward window (angle_min..angle_max, range_min..range_max) used by aligner
  - RANSAC line fit (matches dock_action_server.LidarLineAligner)
  - Inlier points (red), other window points (blue), out-of-window (gray)
  - target_distance circle (where robot tries to stop)

Press 'q' quit, 's' snap to /tmp/lidar_snap.png.
"""
from __future__ import annotations

import math
import random
import threading
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


# Match dock_action_server.LidarLineAligner defaults + docking.yaml
ANGLE_MIN = -math.pi / 6     # forward window -30°
ANGLE_MAX = +math.pi / 6     # +30°
RANGE_MIN = 0.05
RANGE_MAX = 2.5
TARGET_DISTANCE = 0.30
RANSAC_ITERS = 80
INLIER_THRESH = 0.02
MIN_INLIERS = 8

# Display
WIN_W, WIN_H = 800, 800
SCALE = 250.0    # pixels per meter
ORIGIN = (WIN_W // 2, WIN_H // 2)


def world_to_pixel(x: float, y: float) -> tuple[int, int]:
    # Robot frame: +x forward, +y left.
    # Pixel frame: +px right, +py down. Show forward UP, left LEFT.
    px = int(ORIGIN[0] - y * SCALE)
    py = int(ORIGIN[1] - x * SCALE)
    return px, py


def ransac_line(pts):
    n = len(pts)
    if n < 2:
        return None, []
    best_inliers = 0
    best = None
    best_inlier_idx = []
    for _ in range(RANSAC_ITERS):
        i, j = random.sample(range(n), 2)
        x1, y1 = pts[i]
        x2, y2 = pts[j]
        dx_, dy_ = x2 - x1, y2 - y1
        norm = math.hypot(dx_, dy_)
        if norm < 1e-6:
            continue
        na = -dy_ / norm
        nb = dx_ / norm
        nc = -(na * x1 + nb * y1)
        idx = [k for k, (px, py) in enumerate(pts)
               if abs(na * px + nb * py + nc) < INLIER_THRESH]
        if len(idx) > best_inliers:
            best_inliers = len(idx)
            best = (na, nb, nc)
            best_inlier_idx = idx
    if best is None or best_inliers < MIN_INLIERS:
        return None, []
    return best, best_inlier_idx


class LidarViewer(Node):
    def __init__(self):
        super().__init__('lidar_viewer')
        self._lock = threading.Lock()
        self._scan = None
        self.create_subscription(LaserScan, '/scan',
                                 self._on_scan, qos_profile_sensor_data)
        self.get_logger().info('lidar_viewer ready, waiting for /scan')

    def _on_scan(self, msg):
        with self._lock:
            self._scan = msg

    def render(self):
        with self._lock:
            scan = self._scan
        if scan is None:
            frame = np.zeros((WIN_H, WIN_W, 3), dtype=np.uint8)
            cv2.putText(frame, 'waiting for /scan ...', (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1)
            return frame

        frame = np.full((WIN_H, WIN_W, 3), 30, dtype=np.uint8)

        # Grid every 0.5 m
        for r_m in (0.5, 1.0, 1.5, 2.0, 2.5):
            r_px = int(r_m * SCALE)
            cv2.circle(frame, ORIGIN, r_px, (60, 60, 60), 1)
            cv2.putText(frame, f'{r_m:.1f}m',
                        (ORIGIN[0] + 4, ORIGIN[1] - r_px + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (90, 90, 90), 1)

        # Forward window cone (±30°)
        for ang in (ANGLE_MIN, ANGLE_MAX):
            x_end = RANGE_MAX * math.cos(ang)
            y_end = RANGE_MAX * math.sin(ang)
            cv2.line(frame, ORIGIN, world_to_pixel(x_end, y_end),
                     (50, 100, 50), 1)

        # target_distance line (perpendicular to robot forward at +X = TARGET)
        cv2.line(frame,
                 world_to_pixel(TARGET_DISTANCE, -1.0),
                 world_to_pixel(TARGET_DISTANCE, +1.0),
                 (40, 80, 120), 1)
        cv2.putText(frame, f'target {TARGET_DISTANCE:.2f}m',
                    world_to_pixel(TARGET_DISTANCE, 1.0),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (60, 100, 160), 1)

        # Robot symbol
        cv2.circle(frame, ORIGIN, 8, (200, 200, 200), -1)
        cv2.line(frame, ORIGIN, world_to_pixel(0.15, 0.0),
                 (200, 200, 50), 2)   # +X arrow

        # Convert scan to points (in robot frame)
        a = scan.angle_min
        all_pts = []           # (x, y, in_window)
        window_pts = []        # only those used by aligner
        for r in scan.ranges:
            a_signed = a if a <= math.pi else a - 2.0 * math.pi
            in_win = False
            if math.isfinite(r) and RANGE_MIN < r < RANGE_MAX:
                if ANGLE_MIN <= a_signed <= ANGLE_MAX:
                    in_win = True
                    window_pts.append((r * math.cos(a_signed), r * math.sin(a_signed)))
                all_pts.append((r * math.cos(a_signed), r * math.sin(a_signed), in_win))
            a += scan.angle_increment

        # RANSAC on window_pts
        line, inlier_idx = ransac_line(window_pts)
        inlier_set = set(inlier_idx)

        # Plot all points
        for x, y, in_win in all_pts:
            color = (90, 90, 90) if not in_win else (180, 120, 50)
            cv2.circle(frame, world_to_pixel(x, y), 2, color, -1)

        # Plot inliers in red on top
        for k, (x, y) in enumerate(window_pts):
            if k in inlier_set:
                cv2.circle(frame, world_to_pixel(x, y), 3, (40, 40, 220), -1)

        # Draw RANSAC line if found
        if line is not None:
            na, nb, nc = line
            # line: na*x + nb*y + nc = 0
            # parameterize: pick two distant points along line
            if abs(nb) > 1e-3:
                # y = -(na*x + nc) / nb
                xs = (-2.0, 2.0)
                ys = (-(na * xs[0] + nc) / nb, -(na * xs[1] + nc) / nb)
            else:
                # vertical line: x = -nc/na
                xs = (-nc / na, -nc / na)
                ys = (-2.0, 2.0)
            cv2.line(frame, world_to_pixel(xs[0], ys[0]),
                     world_to_pixel(xs[1], ys[1]),
                     (40, 200, 40), 2)
            # Compute distance + yaw_err same as aligner
            a_, b_, c_ = na, nb, nc
            if c_ > 0:
                a_, b_, c_ = -a_, -b_, -c_
            distance_to_line = -c_
            yaw_err = math.atan2(b_, a_)
            dx = distance_to_line - TARGET_DISTANCE
            txt = (f'inliers={len(inlier_idx)}  dist={distance_to_line:.3f}m  '
                   f'dx={dx:+.3f}  yaw_err={math.degrees(yaw_err):+.1f}deg')
            cv2.putText(frame, txt, (10, WIN_H - 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 60), 1)
        else:
            cv2.putText(frame, f'RANSAC: NO LINE  (window pts={len(window_pts)})',
                        (10, WIN_H - 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 50, 220), 1)

        cv2.putText(frame,
                    f'/scan rays={len(scan.ranges)}  '
                    f'window=[{math.degrees(ANGLE_MIN):.0f}..{math.degrees(ANGLE_MAX):.0f}]deg  '
                    f'range=[{RANGE_MIN}..{RANGE_MAX}]m',
                    (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
        return frame


def main():
    rclpy.init()
    node = LidarViewer()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    cv2.namedWindow('lidar (q quit, s snap)', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('lidar (q quit, s snap)', WIN_W, WIN_H)
    try:
        while rclpy.ok():
            f = node.render()
            cv2.imshow('lidar (q quit, s snap)', f)
            k = cv2.waitKey(50) & 0xFF
            if k == ord('q'):
                break
            if k == ord('s'):
                cv2.imwrite('/tmp/lidar_snap.png', f)
                node.get_logger().info('saved /tmp/lidar_snap.png')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
