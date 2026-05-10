from __future__ import annotations

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

        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._use_msg_frame_ids = bool(self.get_parameter('use_msg_frame_ids').value)
        self._use_msg_stamp = bool(self.get_parameter('use_msg_stamp').value)
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
