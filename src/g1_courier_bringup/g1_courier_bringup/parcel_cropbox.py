"""Cropbox 3D — wycina bryłę niesionej paczki z chmury PRZED projekcją na scan.

Dlaczego własny węzeł zamiast pcl_ros filter_crop_box_node: na realnym G1
pcl_ros po kilku/kilkunastu wiadomościach przestawał publikować (węzeł żył,
debug bez błędów, subskrypcja istniała, źródło nadawało — potwierdzone przez
zespół także standalone z configiem passthrough). pcl_ros nie wystawia
parametrów QoS wejścia/wyjścia ani żadnej introspekcji, więc nie dało się
tego ani obejść, ani obserwować. Ten węzeł:

  1. odbiera i nadaje na qos_profile_sensor_data (best_effort) — dokładnie
     ten profil, którym pointcloud_to_laserscan czyta chmurę i który na
     robocie działa,
  2. RAPORTUJE co report_period_s: ile chmur weszło/wyszło, ile punktów
     wycięto — cisza w torze jest natychmiast widoczna i przypisywalna
     (wejście martwe vs wyjście martwe),
  3. tnie box w NATYWNEJ ramce chmury (bez TF, bez mnożenia macierzy) —
     te same nazwy parametrów co pcl_ros CropBox (min_x..max_z, negative),
     więc config/parcel_cropbox.yaml działa bez zmian.

Koszt: dostęp do x/y/z przez strided view (zero-copy), maska = 6 porównań,
jedna kopia zachowanych wierszy na wyjście. ~20k punktów @ 10 Hz => pojedyncze
% CPU (dawny parcel_cloud_filter brał ~96% przez float64 + matmul + TF).

Topiki `input`/`output` — remapy w launchu identyczne jak dla pcl_ros.
Watchdog toru skanu (scan_watchdog) obejmuje też ten proces.
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

_BOX_KEYS = ('min_x', 'max_x', 'min_y', 'max_y', 'min_z', 'max_z')


class ParcelCropbox(Node):
    def __init__(self) -> None:
        super().__init__('parcel_cropbox')
        self.declare_parameter('min_x', -1.0)
        self.declare_parameter('max_x', 1.0)
        self.declare_parameter('min_y', -1.0)
        self.declare_parameter('max_y', 1.0)
        self.declare_parameter('min_z', -1.0)
        self.declare_parameter('max_z', 1.0)
        # true => usuwa punkty WEWNĄTRZ boxa (semantyka pcl_ros CropBox
        # z negative=true, jedyna jakiej używamy).
        self.declare_parameter('negative', True)
        self.declare_parameter('report_period_s', 10.0)

        self._read_box()
        self.add_on_set_parameters_callback(self._on_set_params)

        self._pub = self.create_publisher(PointCloud2, 'output', qos_profile_sensor_data)
        self.create_subscription(PointCloud2, 'input', self._on_cloud, qos_profile_sensor_data)

        self._n_in = 0
        self._n_out = 0
        self._cut_last = 0
        self._dtype_cache: dict[tuple, np.dtype] = {}
        period = max(1.0, float(self.get_parameter('report_period_s').value))
        self.create_timer(period, self._report)
        self.get_logger().info(
            f'parcel_cropbox: box x[{self.box[0]:.2f},{self.box[1]:.2f}] '
            f'y[{self.box[2]:.2f},{self.box[3]:.2f}] z[{self.box[4]:.2f},{self.box[5]:.2f}] '
            f'negative={self._negative} (sensor_data QoS in/out)')

    def _read_box(self) -> None:
        self.box = tuple(float(self.get_parameter(k).value) for k in _BOX_KEYS)
        self._negative = bool(self.get_parameter('negative').value)

    def _on_set_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        names = {p.name for p in params}
        if names & (set(_BOX_KEYS) | {'negative'}):
            # rclpy najpierw ustawia wartości, callback woła się z nowymi —
            # ale get_parameter zwróci nowe dopiero po zaakceptowaniu, więc
            # czytamy wprost z przekazanych parametrów.
            box = list(self.box)
            for p in params:
                if p.name in _BOX_KEYS:
                    box[_BOX_KEYS.index(p.name)] = float(p.value)
                elif p.name == 'negative':
                    self._negative = bool(p.value)
            self.box = tuple(box)
            self.get_logger().info(
                f'cropbox update: x[{self.box[0]:.2f},{self.box[1]:.2f}] '
                f'y[{self.box[2]:.2f},{self.box[3]:.2f}] z[{self.box[4]:.2f},{self.box[5]:.2f}]')
        return SetParametersResult(successful=True)

    def _xyz_view(self, msg: PointCloud2, n: int):
        """Zero-copy strided view na pola x/y/z (float32)."""
        off = {f.name: f.offset for f in msg.fields}
        if 'x' not in off or 'y' not in off or 'z' not in off:
            return None
        key = (off['x'], off['y'], off['z'], msg.point_step)
        dt = self._dtype_cache.get(key)
        if dt is None:
            dt = np.dtype({'names': ['x', 'y', 'z'],
                           'formats': ['<f4', '<f4', '<f4'],
                           'offsets': [off['x'], off['y'], off['z']],
                           'itemsize': msg.point_step})
            self._dtype_cache[key] = dt
        return np.frombuffer(msg.data, dtype=dt, count=n)

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._n_in += 1
        n = msg.width * msg.height
        if n == 0:
            self._pub.publish(msg)
            self._n_out += 1
            return
        pts = self._xyz_view(msg, n)
        if pts is None:
            self._pub.publish(msg)   # nieznany layout — przepuść bez cięcia
            self._n_out += 1
            return

        xmin, xmax, ymin, ymax, zmin, zmax = self.box
        inside = ((pts['x'] >= xmin) & (pts['x'] <= xmax) &
                  (pts['y'] >= ymin) & (pts['y'] <= ymax) &
                  (pts['z'] >= zmin) & (pts['z'] <= zmax))
        keep = ~inside if self._negative else inside
        n_keep = int(keep.sum())
        self._cut_last = n - n_keep

        if n_keep == n:
            self._pub.publish(msg)   # nic nie wycięto — bez kopiowania
            self._n_out += 1
            return

        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
        out = PointCloud2()
        out.header = msg.header                  # ten sam frame + stamp
        out.height = 1
        out.width = n_keep
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.point_step * n_keep
        out.is_dense = msg.is_dense
        out.data = raw[keep].tobytes()
        self._pub.publish(out)
        self._n_out += 1

    def _report(self) -> None:
        # Stały puls w logu: pozwala odróżnić "wejście martwe" (in nie
        # rośnie) od "wyjście martwe" (in rośnie, out stoi) bez debuggera.
        self.get_logger().info(
            f'clouds in={self._n_in} out={self._n_out} cut_last={self._cut_last} pkt')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParcelCropbox()
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
