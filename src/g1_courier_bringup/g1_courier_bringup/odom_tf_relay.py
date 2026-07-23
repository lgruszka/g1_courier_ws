from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


class OdomTfRelay(Node):
    """Relay Odometry pose into a dynamic TF transform."""

    def __init__(self) -> None:
        super().__init__('odom_tf_relay')

        self.declare_parameter('odom_topic', '/dog_odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('use_msg_frame_ids', False)
        self.declare_parameter('use_msg_stamp', True)
        # Gorny limit publikacji TF [Hz]; 0 = bez limitu (1:1 z odom).
        # /dog_odom na realnym G1 to ~950 Hz — TF w tym tempie kosztuje CPU
        # (zmierzone ~82%) i zatyka konsumentow (np. websocket do Foxglove
        # dropuje wtedy rzadsze TF stawow); nav wystarcza <=50 Hz.
        self.declare_parameter('max_rate', 0.0)
        # Rzut 2D dla base_footprint (REP-120): zeruje z oraz roll/pitch
        # (zostaje x, y, yaw). Firmware G1 daje w odom poze PELVIS (z~0.75,
        # z przechylami tulowia) — do nawigacji 2D ramka bazowa ma lezec
        # plasko na podlodze, inaczej AMCL klei plaszczyzne mapy do bioder.
        self.declare_parameter('flatten', False)

        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._use_msg_frame_ids = bool(self.get_parameter('use_msg_frame_ids').value)
        self._use_msg_stamp = bool(self.get_parameter('use_msg_stamp').value)
        max_rate = float(self.get_parameter('max_rate').value)
        self._min_period = (1.0 / max_rate) if max_rate > 0.0 else 0.0
        self._last_pub = 0.0
        self._flatten = bool(self.get_parameter('flatten').value)
        odom_topic = str(self.get_parameter('odom_topic').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self._sub = self.create_subscription(Odometry, odom_topic, self._on_odom, qos)

        self.get_logger().info(
            f"Relaying '{odom_topic}' into TF using fallback frames "
            f"'{self._odom_frame}' -> '{self._base_frame}'"
        )

    def _on_odom(self, msg: Odometry) -> None:
        if self._min_period > 0.0:
            now = time.monotonic()
            if now - self._last_pub < self._min_period:
                return
            self._last_pub = now

        parent = self._odom_frame
        child = self._base_frame

        if self._use_msg_frame_ids:
            msg_parent = msg.header.frame_id.strip()
            msg_child = msg.child_frame_id.strip()
            if msg_parent:
                parent = msg_parent
            if msg_child:
                child = msg_child

        if parent == child:
            self.get_logger().warn(
                f"Skipping TF publish because parent and child frame are identical ('{parent}')."
            )
            return

        t = TransformStamped()
        t.header.frame_id = parent
        t.child_frame_id = child

        if self._use_msg_stamp and (msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0):
            t.header.stamp = msg.header.stamp
        else:
            t.header.stamp = self.get_clock().now().to_msg()

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        if self._flatten:
            import math
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(yaw / 2.0)
            t.transform.rotation.w = math.cos(yaw / 2.0)
        else:
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation = msg.pose.pose.orientation

        self._tf_broadcaster.sendTransform(t)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OdomTfRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
