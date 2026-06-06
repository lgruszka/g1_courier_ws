import os
import sys
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image


def _inject_venv_site_packages() -> None:
    venv = os.environ.get('VIRTUAL_ENV')
    if not venv:
        return
    py_ver = f'python{sys.version_info.major}.{sys.version_info.minor}'
    site_packages = os.path.join(venv, 'lib', py_ver, 'site-packages')
    if os.path.isdir(site_packages) and site_packages not in sys.path:
        sys.path.insert(0, site_packages)


_inject_venv_site_packages()
import pyrealsense2 as rs


class D435iNode(Node):
    def __init__(self) -> None:
        super().__init__('d435i_node')

        self._format_preferences = (
            rs.format.bgr8,
            rs.format.rgb8,
            rs.format.yuyv,
            rs.format.bgra8,
            rs.format.rgba8,
        )

        # Parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('legacy_color_topic', '/camera/color/image_raw')
        self.declare_parameter('publish_legacy_color_topic', True)
        self.declare_parameter('legacy_camera_info_topic', '/camera_info')
        self.declare_parameter('publish_legacy_camera_info_topic', True)
        self.declare_parameter('publish_depth_topic', True)
        self.declare_parameter('depth_max_distance_m', 4.0)
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('wait_timeout_ms', 1000)
        # When True, depth is aligned to color frame (expensive). When False,
        # depth is published in its own frame without alignment.
        self.declare_parameter('align_depth_to_color', True)
        # Use realsense hardware timestamps for ROS Header.stamp.
        # If False (default), uses node clock at the moment of publish.
        self.declare_parameter('use_hardware_timestamp', False)

        self.requested_width = int(self.get_parameter('width').value)
        self.requested_height = int(self.get_parameter('height').value)
        self.requested_fps = int(self.get_parameter('fps').value)
        self.width = self.requested_width
        self.height = self.requested_height
        self.fps = self.requested_fps
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.legacy_color_topic = str(self.get_parameter('legacy_color_topic').value)
        self.publish_legacy_color_topic = bool(
            self.get_parameter('publish_legacy_color_topic').value
        )
        self.legacy_camera_info_topic = str(
            self.get_parameter('legacy_camera_info_topic').value
        )
        self.publish_legacy_camera_info_topic = bool(
            self.get_parameter('publish_legacy_camera_info_topic').value
        )
        self.publish_depth_topic = bool(self.get_parameter('publish_depth_topic').value)
        self.depth_max_distance_m = max(0.0, float(self.get_parameter('depth_max_distance_m').value))
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.wait_timeout_ms = int(self.get_parameter('wait_timeout_ms').value)
        self.align_depth_to_color = bool(self.get_parameter('align_depth_to_color').value)
        self.use_hardware_timestamp = bool(self.get_parameter('use_hardware_timestamp').value)

        # Publishers — sensor_data QoS = BEST_EFFORT, depth=5. Critical for cameras:
        # avoids RELIABLE buffering when a downstream subscriber stalls.
        self.image_pub = self.create_publisher(Image, self.image_topic, qos_profile_sensor_data)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, self.camera_info_topic, qos_profile_sensor_data
        )
        self.legacy_pub = None
        self.legacy_camera_info_pub = None
        self.depth_pub = None
        if self.publish_legacy_color_topic and self.legacy_color_topic != self.image_topic:
            self.legacy_pub = self.create_publisher(
                Image, self.legacy_color_topic, qos_profile_sensor_data
            )
        legacy_cam_info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        if (
            self.publish_legacy_camera_info_topic
            and self.legacy_camera_info_topic != self.camera_info_topic
        ):
            self.legacy_camera_info_pub = self.create_publisher(
                CameraInfo, self.legacy_camera_info_topic, legacy_cam_info_qos
            )
        if self.publish_depth_topic:
            self.depth_pub = self.create_publisher(
                Image, self.depth_topic, qos_profile_sensor_data
            )

        # Configure RealSense device
        self.device = self._get_first_device()
        self.stream_profile = self._select_color_profile(self.device)
        self.depth_profile = self._select_depth_profile(self.device)
        self.width = self.stream_profile.width()
        self.height = self.stream_profile.height()
        self.fps = self.stream_profile.fps()
        self.stream_format = self.stream_profile.format()
        self.color_intrinsics = self.stream_profile.get_intrinsics()
        self.depth_width = self.depth_profile.width()
        self.depth_height = self.depth_profile.height()
        self.depth_fps = self.depth_profile.fps()
        self.depth_format = self.depth_profile.format()

        self.depth_scale = self._get_depth_scale(self.device)
        self.align_to_color = rs.align(rs.stream.color) if self.align_depth_to_color else None

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_device(self.device.get_info(rs.camera_info.serial_number))
        self.config.enable_stream(
            rs.stream.color,
            self.width,
            self.height,
            self.stream_format,
            self.fps,
        )
        self.config.enable_stream(
            rs.stream.depth,
            self.depth_width,
            self.depth_height,
            self.depth_format,
            self.depth_fps,
        )

        try:
            self.pipeline.start(self.config)
        except Exception as exc:
            self.get_logger().error(
                'Failed to start RealSense D435i '
                f'with {self.width}x{self.height}@{self.fps} {self.stream_format}: {exc}'
            )
            raise

        # Capture thread — uses blocking wait_for_frames so the SDK frame
        # queue never accumulates. This is the key change vs. timer + poll.
        self._stop_event = threading.Event()
        self._capture_thread = threading.Thread(
            target=self._capture_loop, name='d435i_capture', daemon=True
        )
        self._capture_thread.start()

        self.get_logger().info(
            f'Publishing D435i color stream to {self.image_topic} '
            f'({self.width}x{self.height}@{self.fps}fps, source format={self.stream_format})'
        )
        if self.legacy_pub is not None:
            self.get_logger().info(f'Also publishing legacy color topic: {self.legacy_color_topic}')
        self.get_logger().info(f'Publishing camera info to {self.camera_info_topic}')
        if self.legacy_camera_info_pub is not None:
            self.get_logger().info(
                f'Also publishing legacy camera info topic: {self.legacy_camera_info_topic}'
            )
        if self.depth_pub is not None:
            align_str = 'aligned to color' if self.align_depth_to_color else 'native depth frame'
            self.get_logger().info(
                f'Also publishing depth stream to {self.depth_topic} '
                f'({self.depth_width}x{self.depth_height}@{self.depth_fps}fps, '
                f'scale={self.depth_scale:.6f}m/unit, {align_str})'
            )

    # ---- device / profile selection (unchanged from original) ----

    def _get_first_device(self):
        context = rs.context()
        devices = context.query_devices()
        if len(devices) == 0:
            raise RuntimeError('No RealSense device detected')
        return devices[0]

    def _select_color_profile(self, device):
        color_sensor = None
        for sensor in device.query_sensors():
            if sensor.get_info(rs.camera_info.name) == 'RGB Camera':
                color_sensor = sensor
                break

        if color_sensor is None:
            raise RuntimeError('RGB Camera sensor not found on RealSense device')

        profiles = []
        for profile in color_sensor.get_stream_profiles():
            if profile.stream_type() != rs.stream.color:
                continue
            profiles.append(profile.as_video_stream_profile())

        if not profiles:
            raise RuntimeError('No color stream profiles reported by RealSense device')

        format_rank = {fmt: index for index, fmt in enumerate(self._format_preferences)}

        def profile_score(profile):
            width_delta = abs(profile.width() - self.requested_width)
            height_delta = abs(profile.height() - self.requested_height)
            fps_delta = abs(profile.fps() - self.requested_fps)
            exact_resolution = (
                0
                if (
                    profile.width() == self.requested_width
                    and profile.height() == self.requested_height
                )
                else 1
            )
            exact_fps = 0 if profile.fps() == self.requested_fps else 1
            format_delta = format_rank.get(profile.format(), len(self._format_preferences))
            return (exact_resolution, width_delta + height_delta, exact_fps, fps_delta, format_delta)

        compatible_profiles = [p for p in profiles if p.format() in format_rank]
        if not compatible_profiles:
            raise RuntimeError('No compatible color profiles found for supported formats')

        selected = min(compatible_profiles, key=profile_score)
        if (
            selected.width() != self.requested_width
            or selected.height() != self.requested_height
            or selected.fps() != self.requested_fps
            or selected.format() != rs.format.bgr8
        ):
            self.get_logger().warn(
                f'Requested {self.requested_width}x{self.requested_height}'
                f'@{self.requested_fps} bgr8 was not selected exactly; using '
                f'{selected.width()}x{selected.height()}@{selected.fps()} '
                f'{selected.format()} instead'
            )
        return selected

    def _select_depth_profile(self, device):
        depth_sensor = None
        for sensor in device.query_sensors():
            if sensor.is_depth_sensor():
                depth_sensor = sensor
                break

        if depth_sensor is None:
            raise RuntimeError('Depth sensor not found on RealSense device')

        profiles = []
        for profile in depth_sensor.get_stream_profiles():
            if profile.stream_type() != rs.stream.depth:
                continue
            profiles.append(profile.as_video_stream_profile())

        if not profiles:
            raise RuntimeError('No depth stream profiles reported by RealSense device')

        compatible = [
            p for p in profiles if p.width() == self.width and p.height() == self.height
        ]
        if not compatible:
            compatible = profiles

        def profile_score(profile):
            res_penalty = 0 if profile.width() == self.width and profile.height() == self.height else 1
            fps_penalty = abs(profile.fps() - self.fps)
            return (res_penalty, fps_penalty)

        return min(compatible, key=profile_score)

    @staticmethod
    def _get_depth_scale(device) -> float:
        for sensor in device.query_sensors():
            if sensor.is_depth_sensor():
                return float(sensor.as_depth_sensor().get_depth_scale())
        raise RuntimeError('Failed to query RealSense depth scale')

    # ---- capture loop ----

    def _capture_loop(self) -> None:
        logger = self.get_logger()
        consecutive_errors = 0
        while not self._stop_event.is_set():
            try:
                frames = self.pipeline.wait_for_frames(self.wait_timeout_ms)
                consecutive_errors = 0
            except RuntimeError as exc:
                # Most common: 'Frame didn't arrive within X' timeout.
                # Don't spam logs — only every 10th occurrence.
                consecutive_errors += 1
                if consecutive_errors % 10 == 1 and not self._stop_event.is_set():
                    logger.warn(f'wait_for_frames issue ({consecutive_errors}x): {exc}')
                continue
            except Exception as exc:
                logger.error(f'Unexpected capture error: {exc}')
                continue

            try:
                self._process_and_publish(frames)
            except Exception as exc:
                logger.error(f'Frame publish error: {exc}')

    def _stamp_from_frame(self, frame):
        """Build Header.stamp — hardware timestamp if requested, else node clock."""
        if self.use_hardware_timestamp:
            # frame.get_timestamp() returns milliseconds since some epoch
            # (frame_metadata.TIMESTAMP_DOMAIN determines which one).
            ts_ms = float(frame.get_timestamp())
            sec = int(ts_ms // 1000)
            nanosec = int((ts_ms - sec * 1000) * 1_000_000)
            from builtin_interfaces.msg import Time
            return Time(sec=sec, nanosec=nanosec)
        return self.get_clock().now().to_msg()

    def _process_and_publish(self, frames) -> None:
        # Skip alignment entirely if nobody is consuming depth aligned to color.
        depth_subs = (
            self.depth_pub.get_subscription_count() if self.depth_pub is not None else 0
        )
        need_aligned_depth = self.align_depth_to_color and depth_subs > 0

        if need_aligned_depth and self.align_to_color is not None:
            frames = self.align_to_color.process(frames)

        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        color_subs = self.image_pub.get_subscription_count()
        if self.legacy_pub is not None:
            color_subs += self.legacy_pub.get_subscription_count()
        camera_info_subs = self.camera_info_pub.get_subscription_count()
        if self.legacy_camera_info_pub is not None:
            camera_info_subs += self.legacy_camera_info_pub.get_subscription_count()

        # Build color + CameraInfo only if somebody is consuming them.
        if color_subs > 0 or camera_info_subs > 0:
            stamp = self._stamp_from_frame(color_frame)
            camera_info = self._camera_info_msg(stamp)

            if color_subs > 0:
                image = Image()
                image.header.stamp = stamp
                image.header.frame_id = self.frame_id
                image.height = int(color_frame.get_height())
                image.width = int(color_frame.get_width())
                image.encoding = 'bgr8'
                image.is_bigendian = 0
                image.step = image.width * 3
                image.data = self._frame_to_bgr8(color_frame)

                if self.image_pub.get_subscription_count() > 0:
                    self.image_pub.publish(image)
                if self.legacy_pub is not None and self.legacy_pub.get_subscription_count() > 0:
                    self.legacy_pub.publish(image)

            if self.camera_info_pub.get_subscription_count() > 0:
                self.camera_info_pub.publish(camera_info)
            if (
                self.legacy_camera_info_pub is not None
                and self.legacy_camera_info_pub.get_subscription_count() > 0
            ):
                self.legacy_camera_info_pub.publish(camera_info)

        # Depth publish — skipped entirely if no subscribers (no copy, no NaN mask).
        if depth_subs > 0:
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                stamp = self._stamp_from_frame(depth_frame)
                self.depth_pub.publish(self._depth_frame_to_msg(depth_frame, stamp))

    def _frame_to_bgr8(self, color_frame) -> bytes:
        frame_format = color_frame.profile.format()
        width = int(color_frame.get_width())
        height = int(color_frame.get_height())
        frame_data = np.asanyarray(color_frame.get_data())

        if frame_format == rs.format.bgr8:
            bgr = frame_data.reshape((height, width, 3))
        elif frame_format == rs.format.rgb8:
            rgb = frame_data.reshape((height, width, 3))
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        elif frame_format == rs.format.rgba8:
            rgba = frame_data.reshape((height, width, 4))
            bgr = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
        elif frame_format == rs.format.bgra8:
            bgra = frame_data.reshape((height, width, 4))
            bgr = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
        elif frame_format == rs.format.yuyv:
            yuyv = frame_data.reshape((height, width, 2))
            bgr = cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)
        else:
            raise RuntimeError(f'Unsupported RealSense color format: {frame_format}')

        return bgr.tobytes()

    def _depth_frame_to_msg(self, depth_frame, stamp) -> Image:
        depth_data = np.asanyarray(depth_frame.get_data()).astype(np.float32)
        depth_m = depth_data * self.depth_scale
        if self.depth_max_distance_m > 0.0:
            invalid_mask = (depth_m <= 0.0) | (depth_m > self.depth_max_distance_m)
        else:
            invalid_mask = depth_m <= 0.0
        depth_m[invalid_mask] = np.nan

        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = self.frame_id
        image.height = int(depth_frame.get_height())
        image.width = int(depth_frame.get_width())
        image.encoding = '32FC1'
        image.is_bigendian = 0
        image.step = image.width * 4
        image.data = np.ascontiguousarray(depth_m).tobytes()
        return image

    def _camera_info_msg(self, stamp) -> CameraInfo:
        intrinsics = self.color_intrinsics
        camera_info = CameraInfo()
        camera_info.header.stamp = stamp
        camera_info.header.frame_id = self.frame_id
        camera_info.height = int(intrinsics.height)
        camera_info.width = int(intrinsics.width)
        camera_info.distortion_model = self._distortion_model_name(intrinsics.model)
        camera_info.d = [float(coeff) for coeff in intrinsics.coeffs]
        camera_info.k = [
            float(intrinsics.fx), 0.0, float(intrinsics.ppx),
            0.0, float(intrinsics.fy), float(intrinsics.ppy),
            0.0, 0.0, 1.0,
        ]
        camera_info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        camera_info.p = [
            float(intrinsics.fx), 0.0, float(intrinsics.ppx), 0.0,
            0.0, float(intrinsics.fy), float(intrinsics.ppy), 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return camera_info

    @staticmethod
    def _distortion_model_name(model) -> str:
        if model in (rs.distortion.brown_conrady, rs.distortion.inverse_brown_conrady):
            return 'plumb_bob'
        if model == rs.distortion.kannala_brandt4:
            return 'equidistant'
        if hasattr(rs.distortion, 'ftheta') and model == rs.distortion.ftheta:
            return 'ftheta'
        return 'plumb_bob'

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._capture_thread.is_alive():
            self._capture_thread.join(timeout=2.0)
        try:
            self.pipeline.stop()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = D435iNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
