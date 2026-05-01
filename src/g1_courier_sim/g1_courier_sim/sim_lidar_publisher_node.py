"""Phase 0/1 fixture: synthetic 2D LaserScan with a configurable table edge.

Models the front face of a single table as a line segment in the `map` frame
and ray-casts the robot's `/scan` against it. The robot's pose comes from
`/odom` (published by `sim_cmd_vel_bridge_node`), so as the robot drives the
ranges update consistently — letting `dock_to_table MODE_LIDAR_LINE`
actually converge against this fixture.

Geometry (default):
  - Table edge is a horizontal line at `table_x_m` ahead of the map origin,
    spanning `[-table_width_m/2, table_width_m/2]` in y, oriented at
    `table_yaw_rad` (rotation of the segment around its midpoint).
  - Robot starts at (0, 0, 0) and drives forward; when its x approaches
    table_x_m - target_distance, the dock controller should converge.

Topic: /scan (sensor_msgs/LaserScan)
Rate:  10 Hz (configurable)
"""
from __future__ import annotations

import math
import random
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _ray_segment_intersect(ox: float, oy: float, dx: float, dy: float,
                           ax: float, ay: float, bx: float, by: float) -> Optional[float]:
    """Return positive ray parameter t at which the ray (o + t*d) hits segment a..b,
    or None if no intersection. Direction (dx, dy) is unit-length."""
    rx, ry = bx - ax, by - ay
    denom = dx * ry - dy * rx
    if abs(denom) < 1e-9:
        return None  # parallel
    t = ((ax - ox) * ry - (ay - oy) * rx) / denom
    s = ((ax - ox) * dy - (ay - oy) * dx) / denom
    if t > 0.0 and 0.0 <= s <= 1.0:
        return t
    return None


class SimLidarPublisher(Node):
    def __init__(self) -> None:
        super().__init__('sim_lidar_publisher_node')
        self.declare_parameter('topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('n_beams', 181)            # 1 deg resolution, 180 deg fov
        self.declare_parameter('angle_min_rad', -math.pi / 2)
        self.declare_parameter('angle_max_rad', math.pi / 2)
        self.declare_parameter('range_min_m', 0.05)
        self.declare_parameter('range_max_m', 5.0)
        self.declare_parameter('noise_m', 0.005)
        # Table front segment in map frame (centre, width, orientation).
        self.declare_parameter('table_x_m', 1.0)
        self.declare_parameter('table_y_m', 0.0)
        self.declare_parameter('table_width_m', 1.2)
        self.declare_parameter('table_yaw_rad', 0.0)

        self._n = int(self.get_parameter('n_beams').value)
        self._a_min = float(self.get_parameter('angle_min_rad').value)
        self._a_max = float(self.get_parameter('angle_max_rad').value)
        self._a_inc = (self._a_max - self._a_min) / max(1, self._n - 1)
        self._r_min = float(self.get_parameter('range_min_m').value)
        self._r_max = float(self.get_parameter('range_max_m').value)
        self._noise = float(self.get_parameter('noise_m').value)
        self._frame = str(self.get_parameter('frame_id').value)

        # Robot pose, updated from /odom.
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value),
            self._on_odom, qos_profile_sensor_data,
        )
        self._pub = self.create_publisher(
            LaserScan, str(self.get_parameter('topic').value), 10,
        )
        rate = max(1.0, float(self.get_parameter('rate_hz').value))
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f'sim_lidar_publisher ready @ {rate:.0f} Hz, table at '
            f'(x={self.get_parameter("table_x_m").value}, '
            f'y={self.get_parameter("table_y_m").value}, '
            f'w={self.get_parameter("table_width_m").value}, '
            f'yaw={self.get_parameter("table_yaw_rad").value})'
        )

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._x = float(p.position.x)
        self._y = float(p.position.y)
        self._yaw = _yaw_from_quat(p.orientation)

    def _table_endpoints(self) -> tuple[float, float, float, float]:
        cx = float(self.get_parameter('table_x_m').value)
        cy = float(self.get_parameter('table_y_m').value)
        w = float(self.get_parameter('table_width_m').value)
        ty = float(self.get_parameter('table_yaw_rad').value)
        # segment along table-local Y axis, rotated by table_yaw, centered at (cx, cy)
        h = w * 0.5
        cy_, sy_ = math.cos(ty), math.sin(ty)
        ax = cx - h * sy_
        ay = cy + h * cy_
        bx = cx + h * sy_
        by = cy - h * cy_
        return ax, ay, bx, by

    def _publish(self) -> None:
        ax, ay, bx, by = self._table_endpoints()
        ranges: list[float] = []
        # Robot pose in map.
        rx, ry, ryaw = self._x, self._y, self._yaw
        for i in range(self._n):
            beam_angle = self._a_min + i * self._a_inc
            world_angle = ryaw + beam_angle
            dx, dy = math.cos(world_angle), math.sin(world_angle)
            t = _ray_segment_intersect(rx, ry, dx, dy, ax, ay, bx, by)
            if t is None or t < self._r_min or t > self._r_max:
                ranges.append(float('inf'))
            else:
                if self._noise > 0.0:
                    t += random.uniform(-self._noise, self._noise)
                ranges.append(float(t))
        msg = LaserScan()
        msg.header.frame_id = self._frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle_min = self._a_min
        msg.angle_max = self._a_max
        msg.angle_increment = self._a_inc
        msg.range_min = self._r_min
        msg.range_max = self._r_max
        msg.ranges = ranges
        msg.intensities = []
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimLidarPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
