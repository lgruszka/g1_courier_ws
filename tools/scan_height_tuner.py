"""Live tuner dla pointcloud_to_laserscan min_height / max_height.

Workflow:
  1. real.launch.py odpalone (pointcloud_to_laserscan node chodzi)
  2. RViz otwarte z LaserScan display (`/scan`, Best Effort)
  3. python3 tools/scan_height_tuner.py
  4. Slider zmienia params LIVE przez `ros2 param set` — patrzysz w RViz
  5. Gdy /scan wygląda OK (pokrywa ściany mapy) — klik **Save to yaml**

Plus optional auto-suggest: jeśli mapa załadowana w map_server, skrypt zaproponuje
slice wartości na podstawie overlap heuristic.

Wymaga: PyQt5 (sudo apt install python3-pyqt5).
"""
from __future__ import annotations

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


# Defaults — kontroler na slidery. Wartości w *target_frame* z config yaml
# (typowo livox_frame, w którym Z+ to "do góry lokalnie w lidar").
MIN_VAL = -3.0
MAX_VAL = 3.0
STEP = 0.05

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
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle('pointcloud_to_laserscan — height tuner')
        self.resize(800, 500)
        self._pcl_proc: Optional[subprocess.Popen] = None

        rclpy.init()
        self.node = rclpy.create_node('scan_height_tuner')
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
            f'<b>YAML:</b> {DEFAULT_YAML}<br>'
            'Zmiany leca <b>live</b> przez <code>ros2 param set</code>. '
            'Patrz w RViz LaserScan display (`/scan`, Best Effort QoS). '
            'Gdy OK — klik <b>Save to yaml</b>.'
        )
        info.setTextFormat(Qt.RichText)
        info.setWordWrap(True)
        root.addWidget(info)

        # min_height slider + spinbox.
        self.min_slider, self.min_spin = self._make_slider_pair('min_height', -1.0)
        # max_height slider + spinbox.
        self.max_slider, self.max_spin = self._make_slider_pair('max_height', 1.0)

        box_min = QGroupBox('min_height [m] (poniżej tego punkty wycięte)')
        bl1 = QVBoxLayout(box_min)
        bl1.addWidget(self.min_slider)
        bl1.addWidget(self.min_spin)
        root.addWidget(box_min)

        box_max = QGroupBox('max_height [m] (powyżej tego punkty wycięte)')
        bl2 = QVBoxLayout(box_max)
        bl2.addWidget(self.max_slider)
        bl2.addWidget(self.max_spin)
        root.addWidget(box_max)

        # Quick presets.
        preset_row = QHBoxLayout()
        preset_row.addWidget(QLabel('<b>Quick presets:</b>'))
        for name, (lo, hi) in [
            ('podłoga -1.0..-0.5', (-1.0, -0.5)),
            ('niskie -0.3..+0.3', (-0.3, 0.3)),
            ('biurka 0.3..0.8', (0.3, 0.8)),
            ('ściany 0.8..1.5', (0.8, 1.5)),
            ('wszystko -2..+2', (-2.0, 2.0)),
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
        self._start_converter(min_h=-1.0, max_h=1.0)

    def _build_converter_cmd(self, min_h: float, max_h: float) -> list[str]:
        return [
            'ros2', 'run', 'pointcloud_to_laserscan', 'pointcloud_to_laserscan_node',
            '--ros-args',
            '--params-file', DEFAULT_YAML,
            '-p', f'min_height:={min_h}',
            '-p', f'max_height:={max_h}',
            '-r', '__node:=pointcloud_to_laserscan',
            '-r', f'cloud_in:={DEFAULT_CLOUD_TOPIC}',
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

        # Two-way binding.
        def on_slider(v):
            real = MIN_VAL + v * STEP
            spin.blockSignals(True)
            spin.setValue(real)
            spin.blockSignals(False)
            self._apply_live(name, real)

        def on_spin(v):
            slider.blockSignals(True)
            slider.setValue(int((v - MIN_VAL) / STEP))
            slider.blockSignals(False)
            self._apply_live(name, v)

        slider.valueChanged.connect(on_slider)
        spin.valueChanged.connect(on_spin)

        return slider, spin

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
            self.min_spin.setValue(float(params.get('min_height', -1.0)))
            self.max_spin.setValue(float(params.get('max_height', 1.0)))
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
            with open(DEFAULT_YAML, 'w') as f:
                y.safe_dump(data, f, default_flow_style=False, sort_keys=False)
            QMessageBox.information(
                self, 'Saved',
                f'min_height={self.min_spin.value():.2f}, '
                f'max_height={self.max_spin.value():.2f}\n\n'
                f'Zapisane do: {DEFAULT_YAML}\n\n'
                'PAMIĘTAJ: live values już chodzą (przez ros2 param set), ale yaml '
                'jest aplikowany dopiero przy KOLEJNYM uruchomieniu launchu.'
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
    app = QApplication(sys.argv)
    win = TunerWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
