"""Cropbox filtr chmury — usuwa bryłę niesionej paczki z LiDAR-a.

PROBLEM: gdy G1 niesie karton w rękach przed sobą, karton pojawia się w skanie
jako bliskie punkty z przodu → AMCL dopasowuje je do mapy (a nie pasują) →
robot się gubi.

ROZWIĄZANIE: wytnij z chmury wąską bryłę 3D w base_link (przód, na wysokości
chwytaka, na szerokość paczki) ZANIM pójdzie do pointcloud_to_laserscan. Bryła
WĄSKA w Y — więc krawędź stołu (szeroka) przetrwa po bokach i dokowanie
LIDAR_LINE dalej działa.

Pipeline:
  /livox/lidar → [ten węzeł] → /livox/lidar_filtered → pointcloud_to_laserscan → /scan

Filtr zachowuje WSZYSTKIE pola punktu (bajt-w-bajt) — usuwa tylko wiersze punktów
wpadające w cropbox base_link. Test inside/outside po transformacie tf2 do
base_link; republikacja w ORYGINALNYM frame (współrzędne bez zmian).

Parametry (base_link, przód=+x):
  box_x_min/max, box_y_min/max, box_z_min/max  — granice wycinanej bryły [m]
  target_frame (base_link), cloud_in, cloud_out
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import tf2_ros
from sensor_msgs.msg import PointCloud2


def _quat_to_R(x, y, z, w):
    import math
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z),   2*(y*z-x*w)],
        [2*(x*z-y*w),     2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


class ParcelCloudFilter(Node):
    def __init__(self) -> None:
        super().__init__('parcel_cloud_filter')
        self.declare_parameter('cloud_in', '/livox/lidar')
        self.declare_parameter('cloud_out', '/livox/lidar_filtered')
        self.declare_parameter('target_frame', 'base_link')
        # Domyślny cropbox: paczka 0.10–0.45 m przed, ±0.25 m wąsko (bok stołu
        # przetrwa), pełna rozsądna wysokość. Tuning: ros2 param set.
        self.declare_parameter('box_x_min', 0.10)
        self.declare_parameter('box_x_max', 0.45)
        self.declare_parameter('box_y_min', -0.25)
        self.declare_parameter('box_y_max', 0.25)
        self.declare_parameter('box_z_min', -1.0)
        self.declare_parameter('box_z_max', 1.0)

        self.target_frame = str(self.get_parameter('target_frame').value)
        self.box = (
            float(self.get_parameter('box_x_min').value),
            float(self.get_parameter('box_x_max').value),
            float(self.get_parameter('box_y_min').value),
            float(self.get_parameter('box_y_max').value),
            float(self.get_parameter('box_z_min').value),
            float(self.get_parameter('box_z_max').value),
        )
        cloud_in = str(self.get_parameter('cloud_in').value)
        cloud_out = str(self.get_parameter('cloud_out').value)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._pub = self.create_publisher(PointCloud2, cloud_out, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, cloud_in, self._on_cloud, qos_profile_sensor_data)
        self._warned = False
        self._removed_ema = 0.0
        self.get_logger().info(
            f'parcel_cloud_filter: {cloud_in} -> {cloud_out}, cropbox base_link '
            f'x[{self.box[0]:.2f},{self.box[1]:.2f}] y[{self.box[2]:.2f},{self.box[3]:.2f}] '
            f'z[{self.box[4]:.2f},{self.box[5]:.2f}]')

    def _x_off(self, msg):
        off = {f.name: f.offset for f in msg.fields}
        return off.get('x'), off.get('y'), off.get('z')

    def _on_cloud(self, msg: PointCloud2) -> None:
        n = msg.width * msg.height
        if n == 0:
            self._pub.publish(msg)
            return
        xo, yo, zo = self._x_off(msg)
        if xo is None or yo is None or zo is None:
            self._pub.publish(msg)   # nieznany layout — przepuść
            return

        # TF target_frame <- cloud frame; bez TF przepuść (nie gub całej chmury).
        try:
            tf = self._tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id, rclpy.time.Time())
        except Exception:
            if not self._warned:
                self.get_logger().warn(
                    f'brak TF {self.target_frame}<-{msg.header.frame_id} — przepuszczam '
                    f'chmurę bez filtra (do czasu aż TF będzie)')
                self._warned = True
            self._pub.publish(msg)
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        R = _quat_to_R(q.x, q.y, q.z, q.w)
        T = np.array([t.x, t.y, t.z])

        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)
        xs = raw[:, xo:xo+4].copy().view('<f4').reshape(n)
        ys = raw[:, yo:yo+4].copy().view('<f4').reshape(n)
        zs = raw[:, zo:zo+4].copy().view('<f4').reshape(n)
        pts = np.stack([xs, ys, zs], axis=1).astype(np.float64)
        finite = np.isfinite(pts).all(axis=1)

        # transform do base_link tylko do testu inside/outside
        pb = (R @ pts.T).T + T
        xmin, xmax, ymin, ymax, zmin, zmax = self.box
        inside = (
            finite &
            (pb[:, 0] >= xmin) & (pb[:, 0] <= xmax) &
            (pb[:, 1] >= ymin) & (pb[:, 1] <= ymax) &
            (pb[:, 2] >= zmin) & (pb[:, 2] <= zmax)
        )
        keep = ~inside
        n_removed = int(inside.sum())
        self._removed_ema = 0.9 * self._removed_ema + 0.1 * n_removed

        out = PointCloud2()
        out.header = msg.header                 # ten sam frame + stamp
        out.height = 1
        out.width = int(keep.sum())
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.point_step * out.width
        out.is_dense = msg.is_dense
        out.data = raw[keep].tobytes()          # oryginalne bajty zachowanych pkt
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParcelCloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
