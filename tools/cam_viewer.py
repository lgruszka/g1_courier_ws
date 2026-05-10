#!/usr/bin/env python3
"""Live head_cam preview + AprilTag bbox + distance overlay.

Subscribes /head_cam/image_raw + /detections + /camera_info.
Per detection runs solvePnP, displays z_c (depth along optical axis),
full 3D distance, and lateral+vertical offset (x_c, y_c).

Press 'q' quit, 's' snapshot to /tmp/cam_snap.png.
"""
from __future__ import annotations
import threading
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, CameraInfo
from apriltag_msgs.msg import AprilTagDetectionArray

TAG_SIZE_M = {5: 0.16, 7: 0.16, 10: 0.10}

FALLBACK_K = np.array([
    [415.69, 0.0, 320.0],
    [0.0, 415.69, 240.0],
    [0.0, 0.0, 1.0],
], dtype=np.float64)


def _solve_pose(corners, tag_size_m, K):
    s = tag_size_m / 2.0
    obj_pts = np.array([
        [-s,  s, 0.0], [ s,  s, 0.0],
        [ s, -s, 0.0], [-s, -s, 0.0],
    ], dtype=np.float64)
    img_pts = np.array([
        (float(corners[3].x), float(corners[3].y)),
        (float(corners[2].x), float(corners[2].y)),
        (float(corners[1].x), float(corners[1].y)),
        (float(corners[0].x), float(corners[0].y)),
    ], dtype=np.float64)
    n, rvecs, tvecs, _ = cv2.solvePnPGeneric(
        obj_pts, img_pts, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if n == 0:
        return None
    for r, t in zip(rvecs, tvecs):
        R, _ = cv2.Rodrigues(r)
        if R[2, 2] < 0:
            return t
    return tvecs[0]


class CamViewer(Node):
    def __init__(self):
        super().__init__('cam_viewer')
        self._lock = threading.Lock()
        self._frame = None
        self._dets = []
        self._K = FALLBACK_K.copy()
        info_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, '/head_cam/image_raw',
                                 self._on_image, qos_profile_sensor_data)
        self.create_subscription(AprilTagDetectionArray, '/detections',
                                 self._on_dets, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera_info',
                                 self._on_info, info_qos)
        self.get_logger().info('cam_viewer ready')

    def _on_image(self, msg):
        if msg.encoding != 'rgb8':
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        with self._lock:
            self._frame = bgr

    def _on_dets(self, msg):
        with self._lock:
            self._dets = list(msg.detections)

    def _on_info(self, msg):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        with self._lock:
            self._K = K

    def render(self):
        with self._lock:
            if self._frame is None:
                return None
            frame = self._frame.copy()
            dets = list(self._dets)
            K = self._K.copy()
        for d in dets:
            corners = getattr(d, 'corners', None)
            if not corners or len(corners) != 4:
                continue
            pts = np.array([[int(c.x), int(c.y)] for c in corners],
                           dtype=np.int32)
            color = {5: (0, 255, 0), 7: (0, 200, 255), 10: (0, 100, 255)}.get(
                d.id, (200, 200, 200))
            cv2.polylines(frame, [pts], True, color, 2)
            cx = int(getattr(d.centre, 'x', 0))
            cy = int(getattr(d.centre, 'y', 0))
            cv2.circle(frame, (cx, cy), 4, color, -1)
            tvec = _solve_pose(corners, TAG_SIZE_M.get(d.id, 0.10), K)
            lines = [f'id={d.id} m={d.decision_margin:.0f}']
            if tvec is not None:
                xc, yc, zc = (float(v) for v in tvec.flatten())
                lines.append(f'z={zc:.3f}m')
                lines.append(f'dist={np.linalg.norm(tvec):.3f}m')
                lines.append(f'xy=({xc:+.2f},{yc:+.2f})')
            for i, line in enumerate(lines):
                cv2.putText(frame, line, (cx + 8, cy - 8 + i * 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
        return frame


def main():
    rclpy.init()
    node = CamViewer()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    cv2.namedWindow('head_cam (q quit, s snap)', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('head_cam (q quit, s snap)', 800, 600)
    try:
        while rclpy.ok():
            f = node.render()
            if f is not None:
                cv2.imshow('head_cam (q quit, s snap)', f)
            k = cv2.waitKey(30) & 0xFF
            if k == ord('q'):
                break
            if k == ord('s') and f is not None:
                cv2.imwrite('/tmp/cam_snap.png', f)
                node.get_logger().info('saved /tmp/cam_snap.png')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
