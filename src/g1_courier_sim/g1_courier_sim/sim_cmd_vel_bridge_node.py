"""Phase 0 kinematic cmd_vel bridge.

Integrates `/cmd_vel` over time and pretends the robot moved there. No physics,
no sensors — pure dead-reckoning of a unicycle/holonomic base. Used in Phase 0
(no-sim) to drive nav2/mission BT against a fake but plausible base. In later
phases (MuJoCo, Isaac) this node is bypassed by the simulator's own odom.

Inputs:
  /cmd_vel         (geometry_msgs/Twist)  — base-frame velocity from arbiter.
  /initialpose     (geometry_msgs/PoseWithCovarianceStamped) — rviz "2D Pose Estimate".

Outputs:
  /odom            (nav_msgs/Odometry)    — integrated pose + velocity.
  TF odom -> base_link  (dynamic, every tick)
  TF map  -> odom       (static, identity by default; tunable via params)

Behavior:
  - If `/cmd_vel` stale > cmd_timeout_s, integration uses zero velocity (robot stops).
  - `/initialpose` snaps the pose to the requested point — convenient for AMCL-style
    relocalization in rviz.
"""
from __future__ import annotations

import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    """Returns (x, y, z, w) for a rotation around Z by yaw."""
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class SimCmdVelBridge(Node):
    def __init__(self) -> None:
        super().__init__('sim_cmd_vel_bridge_node')

        self.declare_parameter('update_rate_hz', 50.0)
        self.declare_parameter('cmd_timeout_s', 0.4)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        # Static map->odom transform (usually identity in Phase 0).
        self.declare_parameter('map_to_odom_x', 0.0)
        self.declare_parameter('map_to_odom_y', 0.0)
        self.declare_parameter('map_to_odom_yaw', 0.0)
        # Set False when running with a SLAM stack (slam_toolbox/AMCL) that
        # publishes its own map->odom transform; otherwise the two static
        # broadcasters fight on /tf_static and TF tree breaks.
        self.declare_parameter('publish_map_to_odom', True)

        self._rate_hz = max(1.0, float(self.get_parameter('update_rate_hz').value))
        self._dt = 1.0 / self._rate_hz
        self._timeout_ns = int(float(self.get_parameter('cmd_timeout_s').value) * 1e9)
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)

        # Pose state: 2D dead-reckoning.
        self._lock = threading.Lock()
        self._x = float(self.get_parameter('initial_x').value)
        self._y = float(self.get_parameter('initial_y').value)
        self._yaw = float(self.get_parameter('initial_yaw').value)
        self._cmd = Twist()
        self._cmd_stamp_ns = 0

        # IO.
        self._odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter('odom_topic').value), 10,
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.create_subscription(
            Twist, str(self.get_parameter('cmd_vel_topic').value), self._on_cmd_vel, 10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self._on_initialpose, 10,
        )

        if bool(self.get_parameter('publish_map_to_odom').value):
            self._publish_static_map_to_odom()
            map_status = 'static map->odom published'
        else:
            map_status = 'map->odom delegated to SLAM stack'
        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f'sim_cmd_vel_bridge ready @ {self._rate_hz:.0f} Hz, '
            f'pose=({self._x:.2f}, {self._y:.2f}, {math.degrees(self._yaw):.1f}deg), '
            f'{map_status}'
        )

    # ---------- callbacks ----------

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._lock:
            self._cmd = msg
            self._cmd_stamp_ns = self.get_clock().now().nanoseconds

    def _on_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        # Extract yaw from quaternion (assumes near-flat, z-axis rotation only).
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._x = p.position.x
            self._y = p.position.y
            self._yaw = yaw
        self.get_logger().info(
            f'initialpose -> ({self._x:.2f}, {self._y:.2f}, {math.degrees(self._yaw):.1f}deg)'
        )

    # ---------- helpers ----------

    def _publish_static_map_to_odom(self) -> None:
        x = float(self.get_parameter('map_to_odom_x').value)
        y = float(self.get_parameter('map_to_odom_y').value)
        yaw = float(self.get_parameter('map_to_odom_yaw').value)
        qx, qy, qz, qw = _yaw_to_quat(yaw)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._map_frame
        t.child_frame_id = self._odom_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._static_tf_broadcaster.sendTransform(t)

    # ---------- main loop ----------

    def _tick(self) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds

        with self._lock:
            stale = (now_ns - self._cmd_stamp_ns) > self._timeout_ns
            vx = 0.0 if stale else self._cmd.linear.x
            vy = 0.0 if stale else self._cmd.linear.y
            wz = 0.0 if stale else self._cmd.angular.z

            # Forward Euler integration in odom frame.
            cy, sy = math.cos(self._yaw), math.sin(self._yaw)
            self._x += (vx * cy - vy * sy) * self._dt
            self._y += (vx * sy + vy * cy) * self._dt
            self._yaw = math.atan2(math.sin(self._yaw + wz * self._dt),
                                   math.cos(self._yaw + wz * self._dt))
            x, y, yaw = self._x, self._y, self._yaw

        qx, qy, qz, qw = _yaw_to_quat(yaw)
        stamp = now.to_msg()

        # TF odom -> base_link (dynamic).
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id = self._base_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf)

        # /odom message.
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self._odom_pub.publish(odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimCmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
