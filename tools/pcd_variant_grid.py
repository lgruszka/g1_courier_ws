"""Generuj wiele wariantów mapy z jednego PCD — do dobrania najlepszego slice + res.

Multi-variant batch dla `fastlio_pcd_to_pgm.py`. Wypycha grid kombinacji:
- Z-slice (wiele pasm wysokości)
- resolution (kilka rozdzielczości)
- min_points_per_cell (rzadko/gęsto wymagana zajętość komórki)
- flip-y (mirror gdy Mid-360 jest upside-down na G1)

Wyniki w osobnych podfolderach żeby łatwo porównać w viewer obrazów.

Bez open3d — parser PCD binary natywny (numpy). Plus szybki.

Usage:
  python3 tools/pcd_variant_grid.py <input.pcd> <out_dir> [--flip-y]

Przykład:
  python3 tools/pcd_variant_grid.py g1_map.pcd ~/maps/scenarios_g1_map_v2 --flip-y
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
from PIL import Image


def parse_pcd(path: str) -> np.ndarray:
    """Return Nx3 array of (x,y,z) from a binary PCD with float32 fields."""
    with open(path, 'rb') as f:
        header = b''
        while True:
            line = f.readline()
            header += line
            if line.startswith(b'DATA'):
                break
        hdr_text = header.decode('ascii', errors='ignore')

        fields, sizes, types, counts = [], [], [], []
        npoints = 0
        data_type = 'ascii'
        for ln in hdr_text.splitlines():
            t = ln.strip()
            if t.startswith('FIELDS'):
                fields = t.split()[1:]
            elif t.startswith('SIZE'):
                sizes = [int(x) for x in t.split()[1:]]
            elif t.startswith('TYPE'):
                types = t.split()[1:]
            elif t.startswith('COUNT'):
                counts = [int(x) for x in t.split()[1:]]
            elif t.startswith('POINTS'):
                npoints = int(t.split()[1])
            elif t.startswith('DATA'):
                data_type = t.split()[1]

        if data_type != 'binary':
            raise RuntimeError(f'only binary PCD supported, got {data_type}')

        np_type = {('F', 4): 'f4', ('F', 8): 'f8',
                   ('U', 1): 'u1', ('U', 2): 'u2', ('U', 4): 'u4',
                   ('I', 1): 'i1', ('I', 2): 'i2', ('I', 4): 'i4'}
        dtype_list = []
        for name, sz, tp, cnt in zip(fields, sizes, types, counts):
            dt = np_type[(tp, sz)]
            if cnt == 1:
                dtype_list.append((name, dt))
            else:
                dtype_list.append((name, dt, cnt))
        dtype = np.dtype(dtype_list)

        raw = np.frombuffer(f.read(), dtype=dtype, count=npoints)
        xyz = np.column_stack([raw['x'], raw['y'], raw['z']]).astype(np.float32)
        return xyz


def rasterize(pts: np.ndarray, z_min: float, z_max: float,
              resolution: float, min_pts_per_cell: int):
    mask = (pts[:, 2] >= z_min) & (pts[:, 2] <= z_max)
    sel = pts[mask]
    if len(sel) == 0:
        return None, None, None, 0

    x_min, y_min = sel[:, 0].min(), sel[:, 1].min()
    x_max, y_max = sel[:, 0].max(), sel[:, 1].max()
    cols = int(np.ceil((x_max - x_min) / resolution))
    rows = int(np.ceil((y_max - y_min) / resolution))
    if cols < 1 or rows < 1:
        return None, None, None, 0

    col_idx = np.clip(((sel[:, 0] - x_min) / resolution).astype(int), 0, cols - 1)
    row_idx = np.clip(((sel[:, 1] - y_min) / resolution).astype(int), 0, rows - 1)
    count = np.zeros((rows, cols), dtype=np.int32)
    np.add.at(count, (row_idx, col_idx), 1)

    pgm = np.full((rows, cols), 205, dtype=np.uint8)
    pgm[count >= min_pts_per_cell] = 0
    pgm = np.flipud(pgm)
    return pgm, x_min, y_min, len(sel)


def write_variant(out_dir: str, base: str, pgm, x0, y0, resolution: float):
    pgm_path = os.path.join(out_dir, f'{base}.pgm')
    yaml_path = os.path.join(out_dir, f'{base}.yaml')
    Image.fromarray(pgm, mode='L').save(pgm_path)
    with open(yaml_path, 'w') as f:
        f.write(
            f'image: {base}.pgm\n'
            f'resolution: {resolution}\n'
            f'origin: [{x0:.4f}, {y0:.4f}, 0.0]\n'
            f'occupied_thresh: 0.65\n'
            f'free_thresh: 0.25\n'
            f'negate: 0\n'
            f'mode: trinary\n'
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pcd_path')
    parser.add_argument('out_dir')
    parser.add_argument('--flip-y', action='store_true',
                        help='Mid-360 upside-down — mirror Y')
    parser.add_argument('--flip-x', action='store_true')
    args = parser.parse_args()

    if not os.path.isfile(args.pcd_path):
        sys.stderr.write(f'PCD not found: {args.pcd_path}\n')
        return 1

    print(f'Loading {args.pcd_path}...')
    pts = parse_pcd(args.pcd_path)
    print(f'  {len(pts)} points')

    if args.flip_y:
        pts[:, 1] = -pts[:, 1]
        print('  applied --flip-y')
    if args.flip_x:
        pts[:, 0] = -pts[:, 0]
        print('  applied --flip-x')

    z_min_g, z_max_g = pts[:, 2].min(), pts[:, 2].max()
    print(f'  X range: [{pts[:,0].min():.2f}, {pts[:,0].max():.2f}]')
    print(f'  Y range: [{pts[:,1].min():.2f}, {pts[:,1].max():.2f}]')
    print(f'  Z range: [{z_min_g:.2f}, {z_max_g:.2f}]')

    # Z histogram (20 bins) — guidance dla wyboru slice.
    edges = np.linspace(z_min_g, z_max_g, 21)
    h, _ = np.histogram(pts[:, 2], bins=edges)
    print('\n  Z histogram (jeden # ~ 2% gęstości):')
    bar_max = max(int(h.max()), 1)
    for i in range(len(h)):
        bar = '#' * int(50 * h[i] / bar_max)
        print(f'    [{edges[i]:+.2f} .. {edges[i+1]:+.2f}]  {h[i]:>7d}  {bar}')

    os.makedirs(args.out_dir, exist_ok=True)

    # ─── Grid wariantów ───────────────────────────────────────────────
    # Każdy wariant: (name, z_min, z_max, resolution, min_pts, opis)
    #
    # Idea:
    # - 14 wariantów Z-slice (od podłogi po ~1.5 m)
    # - 4 wariantów resolution (0.025, 0.05, 0.10) na wybranych slice
    # - 3 wariantów min_pts (1, 2, 4) na flagowym slice "desks_mid"
    variants = [
        # ─── Z slice grid @ resolution 0.05 ───
        ('floor_only',      -0.50, -0.20, 0.05, 2, 'sama podłoga / nogi mebli niskie'),
        ('legs_low',        -0.40,  0.30, 0.05, 2, 'nogi stołów, krzeseł'),
        ('legs_mid',         0.00,  0.50, 0.05, 2, 'foot+leg cross-section'),
        ('desks_low',        0.30,  0.80, 0.05, 2, 'fronty biurek + krzesła'),
        ('desks_mid',        0.50,  0.85, 0.05, 2, 'klasyczny p2ls slice (Tomek)'),
        ('desks_mid_wide',   0.40,  0.95, 0.05, 2, 'szerszy biurka + monitor bases'),
        ('desks_wide',       0.30,  1.10, 0.05, 2, 'biurka + dolne monitory'),
        ('upper',            0.80,  1.50, 0.05, 2, 'monitory / ekrany'),
        ('mid_high',         0.50,  1.50, 0.05, 2, 'biurka + górne obiekty'),
        ('walls_only',       1.20,  2.00, 0.05, 2, 'głównie ściany / lampy'),
        ('full_clear',      -0.40,  1.80, 0.05, 2, 'wszystko - podłoga & sufit'),
        ('thin_05_075',      0.50,  0.75, 0.05, 2, 'wąski Tomek-tested'),
        ('thin_06_080',      0.60,  0.80, 0.05, 2, 'wąski 20cm pas biurek wysoko'),
        ('thin_015_140',     0.15,  1.40, 0.05, 2, 'szeroki alternatywa'),
        # ─── Resolution sweep na flagowym desks_mid ───
        ('desks_mid_res025', 0.50,  0.85, 0.025, 2, 'desks_mid @ 2.5cm (highres)'),
        ('desks_mid_res100', 0.50,  0.85, 0.10, 2,  'desks_mid @ 10cm (lores)'),
        # ─── min_points_per_cell sweep — sprawdz czystość vs dziury ───
        ('desks_mid_dense',  0.50,  0.85, 0.05, 4, 'desks_mid wymaga 4 pkt/cell (mniej szumu)'),
        ('desks_mid_loose',  0.50,  0.85, 0.05, 1, 'desks_mid wymaga 1 pkt/cell (więcej detalu)'),
    ]

    print(f'\nGenerating {len(variants)} variants → {args.out_dir}/')
    print('=' * 90)
    for name, z_min, z_max, resolution, min_pts, comment in variants:
        pgm, x0, y0, n = rasterize(pts, z_min, z_max, resolution, min_pts)
        if pgm is None:
            print(f'  {name:22s}  [{z_min:+.2f}..{z_max:+.2f}] r={resolution}  EMPTY')
            continue
        base = f'{name}_z{z_min:+.2f}_{z_max:+.2f}_r{resolution}'.replace('+', 'p').replace('-', 'm')
        write_variant(args.out_dir, base, pgm, x0, y0, resolution)
        rows, cols = pgm.shape
        occ = int((pgm == 0).sum())
        print(f'  {name:22s}  [{z_min:+.2f}..{z_max:+.2f}] r={resolution} '
              f'm={min_pts} → {cols:>4}×{rows:<4}px  {occ:>6} occ  ({comment})')

    print('=' * 90)
    print(f'\nBrowse {args.out_dir}/ — np. `eog *.pgm` lub `xdg-open {args.out_dir}/`')
    print('Dla nav2: `ros2 launch g1_courier_bringup real.launch.py map:={out}/<name>.yaml`')
    return 0


if __name__ == '__main__':
    sys.exit(main())
