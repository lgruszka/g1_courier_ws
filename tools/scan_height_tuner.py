"""Live tuner dla pointcloud_to_laserscan min_height / max_height.

Tnie PIONOWO w base_footprint (REP-120; target_frame w yaml) — ramka lezy
plasko NA PODLODZE, wiec min/max_height to wprost METRY NAD PODLOGA
(podloga=0, blat biurka ~0.75, sufit ~2.6). Tuner sam publikuje statyczny
lancuch base_footprint->base_link->livox_frame (montaz zmierzony RANSAC,
jak w launchach), wiec dziala standalone: wystarczy zrodlo chmury.

Workflow (VM + most TCP do robota, albo maszyna widzaca chmure w DDS):
  1. zrodlo chmury zyje: bridge_rx (-> /livox/lidar) albo natywny DDS
  2. python3 tools/scan_height_tuner.py [--cloud /livox/lidar]
     (tuner sam odpala pointcloud_to_laserscan i publikuje TF)
  3. RViz: Fixed Frame base_link, LaserScan /scan (Best Effort)
     + PointCloud2 chmury dla odniesienia — np. rviz/mapping_debug.rviz
  4. Suwaki zmieniaja pas na zywo (restart p2l ~0.3 s) — patrzysz w RViz
  5. Gdy /scan = czyste sciany — klik **Save to yaml**, rsync na robota

Wymaga: PyQt5 (sudo apt install python3-pyqt5).
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from PyQt5.QtCore import Qt, pyqtSignal, QObject, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication, QDoubleSpinBox, QFormLayout, QGroupBox, QHBoxLayout,
    QLabel, QMainWindow, QMessageBox, QPushButton, QSlider, QVBoxLayout, QWidget,
)


# Zakres sliderow — wartosci w base_footprint (REP-120): Z+ pionowo w gore,
# 0 = podloga, czyli wartosci to wprost METRY NAD PODLOGA.
MIN_VAL = -0.5
MAX_VAL = 3.0
STEP = 0.05

# Ramka ciecia lezy na podlodze => przelicznik "nad podloga" = wartosc.
BASE_ABOVE_FLOOR = 0.0

# Statyczny lancuch jak w mapping/localization launch:
# base_footprint --(z)--> base_link --(z, roll, pitch)--> livox_frame.
# Defaulty = montaz nominalny (lidar do gory nogami, roll=pi); wartosci
# PER-ROBOT podawaj flagami --footprint-z/--lidar-z/--roll/--pitch
# (pomiar: RANSAC plaszczyzny podlogi z chmury).
DEFAULT_FOOTPRINT_Z = 0.75
DEFAULT_LIDAR_Z = 0.5
DEFAULT_LIDAR_ROLL = 3.14159
DEFAULT_LIDAR_PITCH = 0.0
LIDAR_FRAME = 'livox_frame'
BASE_LINK_FRAME = 'base_link'
BASE_FRAME = 'base_footprint'

# Default path do config yaml (wyliczany względem repo, nie nazwy workspace).
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
DEFAULT_YAML = os.path.join(
    REPO_ROOT, 'src', 'g1_courier_bringup', 'config', 'pointcloud_to_laserscan.yaml'
)
DEFAULT_NODE_NAME = '/pointcloud_to_laserscan'
DEFAULT_CLOUD_TOPIC = '/livox/lidar'


class ScanWatcher(QObject):
    """ROS subscriber że zlicza punkty w /scan plus inf-ratio."""
    stats_updated = pyqtSignal(float, int, int)   # hz, valid_pts, inf_pts

    def __init__(self, node: Node) -> None:
        super().__init__()
        self.node = node
        self._lock = threading.Lock()
        self._count = 0
        self._t0 = time.monotonic()
        self._last_valid = 0
        self._last_inf = 0
        from rclpy.qos import qos_profile_sensor_data
        node.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        node.create_timer(0.5, self._tick)

    def _on_scan(self, msg: LaserScan) -> None:
        import math
        valid = 0
        inf = 0
        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r):
                inf += 1
            else:
                valid += 1
        with self._lock:
            self._count += 1
            self._last_valid = valid
            self._last_inf = inf

    def _tick(self) -> None:
        now = time.monotonic()
        with self._lock:
            dt = now - self._t0
            hz = self._count / dt if dt > 0 else 0
            self._count = 0
            self._t0 = now
            valid = self._last_valid
            inf = self._last_inf
        self.stats_updated.emit(hz, valid, inf)


class TunerWindow(QMainWindow):
    def __init__(self, cloud_topic: str = DEFAULT_CLOUD_TOPIC,
                 publish_tf: bool = True,
                 footprint_z: float = DEFAULT_FOOTPRINT_Z,
                 lidar_z: float = DEFAULT_LIDAR_Z,
                 lidar_roll: float = DEFAULT_LIDAR_ROLL,
                 lidar_pitch: float = DEFAULT_LIDAR_PITCH) -> None:
        super().__init__()
        self.setWindowTitle('pointcloud_to_laserscan — height tuner (base_footprint)')
        self.resize(820, 560)
        self._pcl_proc: Optional[subprocess.Popen] = None
        self.cloud_topic = cloud_topic
        self._mount = (footprint_z, lidar_z, lidar_roll, lidar_pitch)

        rclpy.init()
        self.node = rclpy.create_node('scan_height_tuner')
        if publish_tf:
            self._publish_lidar_tf()
        self.watcher = ScanWatcher(self.node)
        self.watcher.stats_updated.connect(self._on_stats)
        # ROS w wątku.
        self._executor_thread = threading.Thread(
            target=lambda: rclpy.spin(self.node), daemon=True,
        )
        self._executor_thread.start()

        self.node_name = DEFAULT_NODE_NAME
        self._ensure_single_converter_running()
        self._restart_timer = QTimer(self)
        self._restart_timer.setSingleShot(True)
        self._restart_timer.timeout.connect(self._restart_converter_with_current_values)

        # GUI.
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        info = QLabel(
            f'<b>Node:</b> {self.node_name} &nbsp; '
            f'<b>Chmura:</b> {self.cloud_topic} &nbsp; '
            f'<b>YAML:</b> {DEFAULT_YAML}<br>'
            f'Pas ciety <b>pionowo w {BASE_FRAME}</b> (lidar do gory nogami — '
            'tuner publikuje TF montazu i p2l sam prostuje chmure). '
            f'Odniesienie: podloga = {-BASE_ABOVE_FLOOR:.2f}, '
            'blat biurka &asymp; 0.0, sufit &asymp; +1.9. '
            'Patrz w RViz na `/scan` (Best Effort). '
            'Gdy OK — klik <b>Save to yaml</b>.'
        )
        info.setTextFormat(Qt.RichText)
        info.setWordWrap(True)
        root.addWidget(info)

        # min_height slider + spinbox.
        self.min_slider, self.min_spin, self.min_floor_lbl = \
            self._make_slider_pair('min_height', 0.33)
        # max_height slider + spinbox.
        self.max_slider, self.max_spin, self.max_floor_lbl = \
            self._make_slider_pair('max_height', 2.03)

        box_min = QGroupBox('min_height [m w base_link] — DOLNE ciecie: '
                            'podnies, by wyciac podloge/meble')
        bl1 = QVBoxLayout(box_min)
        bl1.addWidget(self.min_slider)
        row_min = QHBoxLayout()
        row_min.addWidget(self.min_spin)
        row_min.addWidget(self.min_floor_lbl)
        bl1.addLayout(row_min)
        root.addWidget(box_min)

        box_max = QGroupBox('max_height [m w base_link] — GORNE ciecie: '
                            'obniz, by wyciac sufit/lampy')
        bl2 = QVBoxLayout(box_max)
        bl2.addWidget(self.max_slider)
        row_max = QHBoxLayout()
        row_max.addWidget(self.max_spin)
        row_max.addWidget(self.max_floor_lbl)
        bl2.addLayout(row_max)
        root.addWidget(box_max)

        # Quick presets (wartosci w base_link; w nawiasie metry nad podloga).
        preset_row = QHBoxLayout()
        preset_row.addWidget(QLabel('<b>Presety:</b>'))
        for name, (lo, hi) in [
            ('ściany+przeszkody 0.33..2.03', (0.33, 2.03)),
            ('ściany nad meblami 1.1..2.0', (1.1, 2.0)),
            ('pas mebli 0.3..1.1', (0.3, 1.1)),
            ('podłoga test -0.2..0.15', (-0.2, 0.15)),
            ('wszystko -0.3..2.8', (-0.3, 2.8)),
        ]:
            btn = QPushButton(name)
            btn.clicked.connect(lambda _, l=lo, h=hi: self._apply_preset(l, h))
            preset_row.addWidget(btn)
        root.addLayout(preset_row)

        # Status (Hz, valid points, inf points).
        self.lbl_status = QLabel('Hz: — | valid: — | inf: —')
        self.lbl_status.setFont(QFont('Monospace'))
        self.lbl_status.setStyleSheet('background: #222; color: #cfc; padding: 8px;')
        root.addWidget(self.lbl_status)
        self.lbl_publishers = QLabel('scan publishers: —')
        self.lbl_publishers.setFont(QFont('Monospace'))
        root.addWidget(self.lbl_publishers)

        # Action buttons.
        row = QHBoxLayout()
        self.btn_save = QPushButton('💾 Save to yaml')
        self.btn_save.setStyleSheet(
            'background-color: #2266aa; color: white; padding: 10px; font-weight: bold;'
        )
        self.btn_save.clicked.connect(self._save_yaml)
        row.addWidget(self.btn_save)

        self.btn_reset = QPushButton('Reset to yaml current')
        self.btn_reset.clicked.connect(self._reset_to_yaml)
        row.addWidget(self.btn_reset)

        root.addLayout(row)

        # Po starcie wczytaj aktualne values z yaml plus zaaplikuj na slider.
        self._load_yaml_into_sliders()
        self._pub_count_timer = QTimer(self)
        self._pub_count_timer.timeout.connect(self._update_scan_publishers_count)
        self._pub_count_timer.start(1000)

    def _publish_lidar_tf(self) -> None:
        """Statyczny lancuch base_footprint->base_link->livox_frame (jak w
        launchach) — publikowany przez wezel tunera, zeby p2l z
        target_frame=base_footprint dzialal standalone."""
        import math
        from geometry_msgs.msg import TransformStamped
        from tf2_ros import StaticTransformBroadcaster
        self._tf_bcaster = StaticTransformBroadcaster(self.node)
        now = self.node.get_clock().now().to_msg()
        footprint_z, lidar_z, lidar_roll, lidar_pitch = self._mount

        weld = TransformStamped()
        weld.header.stamp = now
        weld.header.frame_id = BASE_FRAME
        weld.child_frame_id = BASE_LINK_FRAME
        weld.transform.translation.z = footprint_z
        weld.transform.rotation.w = 1.0

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = BASE_LINK_FRAME
        t.child_frame_id = LIDAR_FRAME
        t.transform.translation.z = lidar_z
        roll, pitch, yaw = lidar_roll, lidar_pitch, 0.0
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        t.transform.rotation.w = cr * cp * cy + sr * sp * sy
        t.transform.rotation.x = sr * cp * cy - cr * sp * sy
        t.transform.rotation.y = cr * sp * cy + sr * cp * sy
        t.transform.rotation.z = cr * cp * sy - sr * sp * cy
        self._tf_bcaster.sendTransform([weld, t])

    def _kill_all_converters(self) -> None:
        try:
            subprocess.run(
                ['pkill', '-f', 'pointcloud_to_laserscan_node'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=2,
            )
        except Exception:
            pass
        time.sleep(0.3)

    def _start_converter(self, min_h: float, max_h: float) -> None:
        cmd = self._build_converter_cmd(min_h=min_h, max_h=max_h)
        try:
            self._pcl_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            time.sleep(0.6)
        except Exception as exc:
            QMessageBox.warning(
                self, 'Autostart fail',
                f'Nie udało się odpalić pointcloud_to_laserscan:\n{exc}'
            )

    def _ensure_single_converter_running(self) -> None:
        self._kill_all_converters()
        lo, hi = self._read_yaml_heights()
        self._start_converter(min_h=lo, max_h=hi)

    @staticmethod
    def _read_yaml_heights() -> tuple[float, float]:
        try:
            import yaml as y
            with open(DEFAULT_YAML) as f:
                data = y.safe_load(f)
            p = data.get('pointcloud_to_laserscan', {}).get('ros__parameters', {})
            return float(p.get('min_height', 0.33)), float(p.get('max_height', 2.03))
        except Exception:
            return 0.33, 2.03

    def _build_converter_cmd(self, min_h: float, max_h: float) -> list[str]:
        return [
            'ros2', 'run', 'pointcloud_to_laserscan', 'pointcloud_to_laserscan_node',
            '--ros-args',
            '--params-file', DEFAULT_YAML,
            '-p', f'min_height:={min_h}',
            '-p', f'max_height:={max_h}',
            # Pas MUSI byc pionowy — wymus base_link nawet na starym yaml.
            '-p', f'target_frame:={BASE_FRAME}',
            '-r', '__node:=pointcloud_to_laserscan',
            '-r', f'cloud_in:={self.cloud_topic}',
            '-r', 'scan:=/scan',
        ]

    def _restart_converter_with_current_values(self) -> None:
        min_h = float(self.min_spin.value())
        max_h = float(self.max_spin.value())
        try:
            self._kill_all_converters()
            self._start_converter(min_h=min_h, max_h=max_h)
        except Exception:
            pass

    def _make_slider_pair(self, name: str, default: float):
        slider = QSlider(Qt.Horizontal)
        steps = int((MAX_VAL - MIN_VAL) / STEP)
        slider.setRange(0, steps)
        slider.setValue(int((default - MIN_VAL) / STEP))
        slider.setTickPosition(QSlider.TicksBelow)
        slider.setTickInterval(int(0.5 / STEP))

        spin = QDoubleSpinBox()
        spin.setRange(MIN_VAL, MAX_VAL)
        spin.setSingleStep(STEP)
        spin.setDecimals(2)
        spin.setValue(default)

        # Przelicznik na metry nad podloga (etykieta obok spinboxa).
        floor_lbl = QLabel()
        floor_lbl.setFont(QFont('Monospace'))

        def update_floor(v: float) -> None:
            floor_lbl.setText(f'= {v + BASE_ABOVE_FLOOR:+.2f} m nad podłogą')

        update_floor(default)

        # Two-way binding.
        def on_slider(v):
            real = MIN_VAL + v * STEP
            spin.blockSignals(True)
            spin.setValue(real)
            spin.blockSignals(False)
            update_floor(real)
            self._apply_live(name, real)

        def on_spin(v):
            slider.blockSignals(True)
            slider.setValue(int((v - MIN_VAL) / STEP))
            slider.blockSignals(False)
            update_floor(v)
            self._apply_live(name, v)

        slider.valueChanged.connect(on_slider)
        spin.valueChanged.connect(on_spin)

        return slider, spin, floor_lbl

    def _apply_live(self, param: str, value: float) -> None:
        # W praktyce ten node nie aplikuje min/max dynamicznie, więc
        # robimy krótko opóźniony restart z nowymi parametrami.
        self._restart_timer.start(250)

    def _apply_preset(self, lo: float, hi: float) -> None:
        self.min_spin.setValue(lo)
        self.max_spin.setValue(hi)

    def _on_stats(self, hz: float, valid: int, inf: int) -> None:
        total = valid + inf
        valid_pct = (100 * valid / total) if total > 0 else 0
        self.lbl_status.setText(
            f'/scan {hz:5.1f} Hz | valid: {valid:>4} ({valid_pct:.0f}%) | inf: {inf:>4}'
        )
        # Kolor by quality:
        if hz < 1:
            self.lbl_status.setStyleSheet('background: #422; color: #fcc; padding: 8px;')
        elif valid < 10:
            self.lbl_status.setStyleSheet('background: #442; color: #ffc; padding: 8px;')
        else:
            self.lbl_status.setStyleSheet('background: #242; color: #cfc; padding: 8px;')

    def _update_scan_publishers_count(self) -> None:
        count = 0
        try:
            out = subprocess.run(
                ['ros2', 'topic', 'info', '/scan'],
                capture_output=True,
                text=True,
                timeout=1.5,
            ).stdout
            for line in out.splitlines():
                if 'Publisher count:' in line:
                    count = int(line.split(':', 1)[1].strip())
                    break
        except Exception:
            pass
        self.lbl_publishers.setText(f'scan publishers: {count}')

    def _load_yaml_into_sliders(self) -> None:
        try:
            import yaml as y
            with open(DEFAULT_YAML) as f:
                data = y.safe_load(f)
            params = data.get('pointcloud_to_laserscan', {}).get('ros__parameters', {})
            self.min_spin.setValue(float(params.get('min_height', 0.33)))
            self.max_spin.setValue(float(params.get('max_height', 2.03)))
        except Exception as exc:
            QMessageBox.warning(
                self, 'Yaml load fail', f'Nie udało się wczytać {DEFAULT_YAML}: {exc}'
            )

    def _reset_to_yaml(self) -> None:
        self._load_yaml_into_sliders()

    def _save_yaml(self) -> None:
        import yaml as y
        try:
            with open(DEFAULT_YAML) as f:
                data = y.safe_load(f)
            params = data.setdefault('pointcloud_to_laserscan', {}).setdefault(
                'ros__parameters', {},
            )
            params['min_height'] = float(self.min_spin.value())
            params['max_height'] = float(self.max_spin.value())
            # Wartosci sa strojone pionowo — pilnuj spojnej ramki w yaml.
            params['target_frame'] = BASE_FRAME
            with open(DEFAULT_YAML, 'w') as f:
                y.safe_dump(data, f, default_flow_style=False, sort_keys=False)
            QMessageBox.information(
                self, 'Saved',
                f'min_height={self.min_spin.value():.2f}, '
                f'max_height={self.max_spin.value():.2f} '
                f'(= {self.min_spin.value() + BASE_ABOVE_FLOOR:.2f}..'
                f'{self.max_spin.value() + BASE_ABOVE_FLOOR:.2f} m nad podłogą)\n\n'
                f'Zapisane do: {DEFAULT_YAML}\n'
                f'(target_frame={BASE_FRAME})\n\n'
                'PAMIĘTAJ: yaml jest aplikowany przy KOLEJNYM uruchomieniu launchu; '
                'na robocie dopiero po rsync + colcon build w kontenerze.'
            )
        except Exception as exc:
            QMessageBox.critical(self, 'Save fail', str(exc))

    def closeEvent(self, event) -> None:
        try:
            if self._pcl_proc is not None and self._pcl_proc.poll() is None:
                self._pcl_proc.terminate()
            self.node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        super().closeEvent(event)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--cloud', default=DEFAULT_CLOUD_TOPIC,
                    help='topik PointCloud2 (VM przez most: /livox/lidar; '
                         'natywnie na robocie: /utlidar/cloud_livox_mid360)')
    ap.add_argument('--no-tf', action='store_true',
                    help='nie publikuj statycznego lancucha TF '
                         '(gdy launch juz go publikuje)')
    ap.add_argument('--footprint-z', type=float, default=DEFAULT_FOOTPRINT_Z,
                    help='wysokosc base_link nad podloga [m]')
    ap.add_argument('--lidar-z', type=float, default=DEFAULT_LIDAR_Z,
                    help='wysokosc lidaru nad base_link [m]')
    ap.add_argument('--roll', type=float, default=DEFAULT_LIDAR_ROLL,
                    help='roll montazu lidaru [rad] (do gory nogami = pi)')
    ap.add_argument('--pitch', type=float, default=DEFAULT_LIDAR_PITCH,
                    help='pitch montazu lidaru [rad] (pomiar RANSAC)')
    args, qt_args = ap.parse_known_args()

    app = QApplication(sys.argv[:1] + qt_args)
    win = TunerWindow(cloud_topic=args.cloud, publish_tf=not args.no_tf,
                      footprint_z=args.footprint_z, lidar_z=args.lidar_z,
                      lidar_roll=args.roll, lidar_pitch=args.pitch)
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
