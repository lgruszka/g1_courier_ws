"""Map picker — przegląd wariantów PGM z pcd_variant_grid + wybór do produkcji.

Usage:
  python3 tools/map_picker.py <scenarios_dir>

Funkcje:
- Lista wariantów po lewej (z opisem + statystykami z manifest.json)
- Preview PGM po prawej (skalowany do okna)
- Przycisk **Save as production** — kopiuje wybrany PGM/YAML jako
  ~/maps/lab.pgm + ~/maps/lab.yaml (override defaultu real.launch.py).
- Przycisk **Open in external viewer** — odpala eog/xdg-open na PGM

Wymaga: PyQt5 (sudo apt install python3-pyqt5).
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys

from PyQt5.QtCore import Qt, QSize
from PyQt5.QtGui import QPixmap, QFont
from PyQt5.QtWidgets import (
    QApplication, QFileDialog, QHBoxLayout, QLabel, QListWidget, QListWidgetItem,
    QMainWindow, QMessageBox, QPushButton, QSplitter, QVBoxLayout, QWidget,
)


class MapPicker(QMainWindow):
    def __init__(self, scenarios_dir: str) -> None:
        super().__init__()
        self.scenarios_dir = scenarios_dir
        self.manifest = self._load_manifest()
        self.current_variant = None

        self.setWindowTitle(f'Map picker — {scenarios_dir}')
        self.resize(1400, 900)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # Top bar — info o source PCD.
        info = QLabel(
            f'<b>Source PCD:</b> {self.manifest.get("source_pcd", "?")}<br>'
            f'<b>X:</b> {self._fmt_range("x_range")}  '
            f'<b>Y:</b> {self._fmt_range("y_range")}  '
            f'<b>Z:</b> {self._fmt_range("z_range")}<br>'
            f'<b>Flip-Y:</b> {self.manifest.get("flip_y", False)}  '
            f'<b>Points:</b> {self.manifest.get("total_points", "?")}'
        )
        info.setTextFormat(Qt.RichText)
        root.addWidget(info)

        # Splitter: lista po lewej, preview + buttons po prawej.
        splitter = QSplitter(Qt.Horizontal)
        root.addWidget(splitter, stretch=1)

        # Lewa strona — lista wariantów.
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.addWidget(QLabel('<b>Warianty</b> (kliknij aby zobaczyć)'))
        self.list_widget = QListWidget()
        self.list_widget.setFont(QFont('Monospace', 9))
        for v in self.manifest.get('variants', []):
            label = (
                f"{v['name']:24s}  Z[{v['z_min']:+.2f}..{v['z_max']:+.2f}] "
                f"r={v['resolution']:.3f}m m={v['min_pts_per_cell']}  "
                f"{v['cols']}×{v['rows']}  occ={v['occupied_cells']}"
            )
            item = QListWidgetItem(label)
            item.setData(Qt.UserRole, v)
            self.list_widget.addItem(item)
        self.list_widget.currentItemChanged.connect(self._on_variant_selected)
        left_layout.addWidget(self.list_widget)

        # Prawa strona — preview + buttons.
        right = QWidget()
        right_layout = QVBoxLayout(right)

        self.preview = QLabel()
        self.preview.setAlignment(Qt.AlignCenter)
        self.preview.setMinimumSize(QSize(600, 600))
        self.preview.setStyleSheet('background: #222; color: #888;')
        self.preview.setText('(wybierz wariant z listy po lewej)')
        right_layout.addWidget(self.preview, stretch=1)

        self.detail = QLabel()
        self.detail.setTextFormat(Qt.RichText)
        self.detail.setWordWrap(True)
        self.detail.setStyleSheet('background: #333; color: #ddd; padding: 6px;')
        right_layout.addWidget(self.detail)

        btn_row = QHBoxLayout()
        self.btn_external = QPushButton('Open in external viewer')
        self.btn_external.setEnabled(False)
        self.btn_external.clicked.connect(self._open_external)
        btn_row.addWidget(self.btn_external)

        self.btn_save = QPushButton('💾 Save as production map (~/maps/lab.yaml)')
        self.btn_save.setStyleSheet(
            'background-color: #2266aa; color: white; padding: 10px; '
            'font-weight: bold; font-size: 12pt;'
        )
        self.btn_save.setEnabled(False)
        self.btn_save.clicked.connect(self._save_production)
        btn_row.addWidget(self.btn_save)

        self.btn_save_as = QPushButton('💾 Save as... (custom name)')
        self.btn_save_as.setEnabled(False)
        self.btn_save_as.clicked.connect(self._save_as)
        btn_row.addWidget(self.btn_save_as)

        right_layout.addLayout(btn_row)

        splitter.addWidget(left)
        splitter.addWidget(right)
        splitter.setSizes([600, 800])

    def _load_manifest(self) -> dict:
        path = os.path.join(self.scenarios_dir, 'manifest.json')
        if not os.path.isfile(path):
            QMessageBox.critical(
                None, 'Manifest missing',
                f'Brak manifest.json w {self.scenarios_dir}.\n\n'
                'Pewnie nie odpaliłeś pcd_variant_grid.py — odpal go najpierw:\n'
                f'  python3 tools/pcd_variant_grid.py <pcd> {self.scenarios_dir} --flip-y',
            )
            sys.exit(1)
        with open(path) as f:
            return json.load(f)

    def _fmt_range(self, key: str) -> str:
        r = self.manifest.get(key, [0, 0])
        return f'[{r[0]:+.2f}, {r[1]:+.2f}]'

    def _on_variant_selected(self, current: QListWidgetItem, _previous) -> None:
        if current is None:
            return
        variant = current.data(Qt.UserRole)
        self.current_variant = variant

        pgm_path = os.path.join(self.scenarios_dir, variant['pgm'])
        pixmap = QPixmap(pgm_path)
        if pixmap.isNull():
            self.preview.setText(f'(błąd ładowania {variant["pgm"]})')
        else:
            # Skaluj zachowując aspect ratio.
            target = self.preview.size()
            scaled = pixmap.scaled(target, Qt.KeepAspectRatio,
                                   Qt.SmoothTransformation)
            self.preview.setPixmap(scaled)

        # Detal.
        size_m = (variant['cols'] * variant['resolution'],
                  variant['rows'] * variant['resolution'])
        self.detail.setText(
            f"<b>{variant['name']}</b><br>"
            f"<b>Komentarz:</b> {variant['comment']}<br>"
            f"<b>Z slice:</b> [{variant['z_min']:+.3f} .. {variant['z_max']:+.3f}] m "
            f"(grubość {variant['z_max']-variant['z_min']:.2f} m)<br>"
            f"<b>Resolution:</b> {variant['resolution']*100:.2f} cm/pixel<br>"
            f"<b>Min points/cell:</b> {variant['min_pts_per_cell']}<br>"
            f"<b>Rozmiar:</b> {variant['cols']} × {variant['rows']} px "
            f"({size_m[0]:.2f} × {size_m[1]:.2f} m world)<br>"
            f"<b>Occupied cells:</b> {variant['occupied_cells']}<br>"
            f"<b>Plik:</b> {variant['yaml']}"
        )
        self.btn_external.setEnabled(True)
        self.btn_save.setEnabled(True)
        self.btn_save_as.setEnabled(True)

    def _open_external(self) -> None:
        if self.current_variant is None:
            return
        path = os.path.join(self.scenarios_dir, self.current_variant['pgm'])
        # Pierwsze eog, fallback xdg-open
        for cmd in (['eog', path], ['xdg-open', path]):
            try:
                subprocess.Popen(cmd)
                return
            except FileNotFoundError:
                continue
        QMessageBox.warning(self, 'Viewer brak', 'eog ani xdg-open niedostępne')

    def _save_production(self) -> None:
        target_dir = os.path.expanduser('~/maps')
        self._save_to(target_dir, 'lab')

    def _save_as(self) -> None:
        target_dir = os.path.expanduser('~/maps')
        os.makedirs(target_dir, exist_ok=True)
        path, _ = QFileDialog.getSaveFileName(
            self, 'Save map as', os.path.join(target_dir, 'custom.yaml'),
            'YAML map (*.yaml)',
        )
        if not path:
            return
        target_dir = os.path.dirname(path)
        name = os.path.splitext(os.path.basename(path))[0]
        self._save_to(target_dir, name)

    def _save_to(self, target_dir: str, name: str) -> None:
        if self.current_variant is None:
            return
        os.makedirs(target_dir, exist_ok=True)
        src_pgm = os.path.join(self.scenarios_dir, self.current_variant['pgm'])
        src_yaml = os.path.join(self.scenarios_dir, self.current_variant['yaml'])
        dst_pgm = os.path.join(target_dir, f'{name}.pgm')
        dst_yaml = os.path.join(target_dir, f'{name}.yaml')

        shutil.copyfile(src_pgm, dst_pgm)
        # Update yaml żeby image wskazywało na nowy plik (relative).
        with open(src_yaml) as f:
            yaml_content = f.read()
        # zamień `image: <stary>.pgm` na `image: <name>.pgm`
        import re
        yaml_content = re.sub(r'image:\s*\S+\.pgm', f'image: {name}.pgm', yaml_content)
        with open(dst_yaml, 'w') as f:
            f.write(yaml_content)

        QMessageBox.information(
            self, 'Saved',
            f'Mapa zapisana:\n  {dst_pgm}\n  {dst_yaml}\n\n'
            f'Użyj w nav: ros2 launch g1_courier_bringup real.launch.py map:={dst_yaml}'
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenarios_dir',
                        help='Folder ze scenariuszami (musi mieć manifest.json)')
    args = parser.parse_args()

    if not os.path.isdir(args.scenarios_dir):
        sys.stderr.write(f'Folder nie istnieje: {args.scenarios_dir}\n')
        sys.exit(1)

    app = QApplication(sys.argv)
    window = MapPicker(os.path.abspath(args.scenarios_dir))
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
