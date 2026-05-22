"""Generuj wiele wariantów mapy z jednego PCD — do dobrania najlepszego slice + res.

Rozszerzony grid kombinacji (~30+ wariantów):
- Z-slice (16 pasm wysokości)
- 3 wartości resolution (0.025, 0.05, 0.10)
- 3 wartości min_points_per_cell (1, 2, 4)
- Flagowe kombinacje rozszerzone (resolution + density sweep)

Wyniki w jednym folderze, manifest.json z metadanymi.

Bez open3d — parser PCD binary natywny (numpy). Szybki, bez zależności C++.

Usage:
  python3 tools/pcd_variant_grid.py <input.pcd> <out_dir> [--flip-y] [--flip-x]

Przykład:
  python3 tools/pcd_variant_grid.py ~/.ros/scans.pcd ~/maps/scenarios --flip-y
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from datetime import datetime

import numpy as np
from PIL import Image
from scipy import ndimage as ndi


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
              resolution: float, min_pts_per_cell: int,
              dilate: int = 0, close: int = 0):
    """Wycina warstwę Z, rasteryzuje do grid'a, opcjonalnie dilatuje/zamyka.

    dilate=N → expand occupied cells o N pikseli (fills small gaps in walls,
    but thickens overall).
    close=N → dilate(N) then erode(N) — closes holes WITHOUT thickening overall
    (preferred dla scan matching gdy ściany mają luki ale grubość ma znaczenie).
    """
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

    occupied = count >= min_pts_per_cell

    # Morphological post-processing — pomaga gdy chmura jest rzadka i ściany
    # wychodzą "kropkowane" zamiast ciągłej linii. AMCL/scan-match potrzebuje
    # solidnych konturów.
    if close > 0:
        # binary_closing = dilation -> erosion, fills gaps but keeps thickness
        occupied = ndi.binary_closing(occupied, iterations=close)
    if dilate > 0:
        occupied = ndi.binary_dilation(occupied, iterations=dilate)

    pgm = np.full((rows, cols), 205, dtype=np.uint8)
    pgm[occupied] = 0
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


def build_variants_grid(default_res: float = 0.05,
                        default_dilate: int = 0,
                        default_close: int = 0) -> list[tuple]:
    """Zwraca listę (name, z_min, z_max, resolution, min_pts, dilate, close, opis).

    `default_res/dilate/close` aplikowane do wszystkich narrow/wide/go2/density/shift
    wariantów. Resolution sweep i morpho sweep zawsze mają stałe wartości żeby
    porównanie było możliwe niezależnie od overrides.
    """
    # Wąskie pasma 0.20-0.35 m wysokości — żeby uchwycić konkretne struktury.
    narrow = [
        ('thin_below_lidar',  -0.30, -0.10, 'punkty tuż nad lidarem (sufit jeśli upside-down mount)'),
        ('thin_lidar_level',  -0.10,  0.10, 'pasmo wokół poziomu lidaru'),
        ('thin_just_above',    0.10,  0.30, 'tuż pod lidarem (po flip-y dla upside-down)'),
        ('thin_low_25',        0.25,  0.50, 'wąski niski pas'),
        ('thin_mid_50',        0.50,  0.75, 'klasyczny Tomek-tested pasem biurek'),
        ('thin_mid_60',        0.60,  0.80, '20cm pas wyższy biurek'),
        ('thin_high_80',       0.80,  1.05, 'wyżej — siedziska / monitory'),
    ]
    # Szerokie pasma 0.5-1.5 m wysokości — ogólny "shape" sceny.
    wide = [
        ('wide_floor_band',   -0.50,  0.20, 'pas wokół sufitu (upside-down camera_init Z- to fizyczne up)'),
        ('wide_low',           0.00,  0.50, 'low band'),
        ('wide_desks',         0.30,  0.85, 'biurka + bases'),
        ('wide_mid_full',      0.30,  1.20, 'biurka + dolne monitory'),
        ('wide_high',          0.80,  1.60, 'górne meble + ściany'),
        ('wide_full',         -0.50,  1.80, 'cały sensowny zakres (bez sufitu i pod podłogą)'),
    ]
    # Bardzo specyficzne dla Go2 (lidar niżej niż na G1, ~0.4 m offset).
    go2_specific = [
        ('go2_walls_only',     1.00,  1.80, 'tylko ściany — Go2 lidar widzi mniej geometrii niż G1'),
        ('go2_floor_obstacles', 0.10,  0.45, 'nogi mebli + niskie obiekty z perspektywy Go2'),
    ]

    variants = []
    D, C = default_dilate, default_close
    # Wszystkie wariantów Z @ globalny default_res/dilate/close (CLI), min_pts 2
    for name, z_min, z_max, comment in narrow + wide + go2_specific:
        variants.append((name, z_min, z_max, default_res, 2, D, C, comment))

    # Resolution sweep na flagowym slice (desks_mid-like).
    # Stałe wartości: 0.02 (bardzo drobne, mebla nogi widoczne), 0.025, 0.05 (standard), 0.10
    FLAG_ZMIN, FLAG_ZMAX = 0.30, 0.85
    for res in (0.02, 0.025, 0.05, 0.10):
        variants.append((
            f'flag_res{int(res*1000):03d}', FLAG_ZMIN, FLAG_ZMAX, res, 2, D, C,
            f'flagowy slice 0.30-0.85 @ {res*100:.1f} cm/px'
        ))

    # Density sweep na flagowym slice — używa default_res
    for mpts in (1, 2, 4, 8):
        variants.append((
            f'flag_density_m{mpts}', FLAG_ZMIN, FLAG_ZMAX, default_res, mpts, D, C,
            f'flagowy slice 0.30-0.85, min_pts={mpts} (większy = mniej szumu)'
        ))

    # Wide-net na podłogę (po flip-y dla upside-down) — używa default_res
    for shift in (0.0, 0.2, 0.4):
        zmin, zmax = 0.50 + shift, 0.90 + shift
        variants.append((
            f'sweep_shifted_z{zmin:.2f}',
            zmin, zmax, default_res, 2, D, C,
            f'shift slice {shift:+.2f} m vs flag'
        ))

    # Morpho sweep na flagship slice — fix dla rzadkich ścian (kropkowane → solidne)
    # Dilate = expand occupied o N px (pogrubia ścianę).
    # Close = dilate+erode (wypełnia luki bez pogrubiania).
    for d in (1, 2, 3):
        variants.append((
            f'flag_dilate{d}', FLAG_ZMIN, FLAG_ZMAX, default_res, 2, d, 0,
            f'flagowy slice + dilate {d}px (zapełnia luki w rzadkich ścianach)'
        ))
    for c in (1, 2, 3):
        variants.append((
            f'flag_close{c}', FLAG_ZMIN, FLAG_ZMAX, default_res, 2, 0, c,
            f'flagowy slice + close {c}px (luki ZAMKNIĘTE bez pogrubienia)'
        ))

    return variants


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pcd_path')
    parser.add_argument('out_dir')
    parser.add_argument('--flip-y', action='store_true',
                        help='Mid-360 upside-down — mirror Y')
    parser.add_argument('--flip-x', action='store_true')
    parser.add_argument('--resolution', type=float, default=0.05,
                        help='Globalny grid res w m/px dla wszystkich slice variants '
                             '(default 0.05 = standard nav2). Użyj 0.02-0.03 jeśli '
                             '/scan nie pasuje do mapy bo piksele za grube. Resolution '
                             'sweep na flagowym slice zawsze ma 0.02/0.025/0.05/0.10 '
                             'do porównania niezależnie od tego.')
    parser.add_argument('--dilate', type=int, default=0,
                        help='Po rasteryzacji rozszerz każdą occupied komórkę o N '
                             'pikseli (binary_dilation). Pomaga gdy chmura jest rzadka '
                             'i ściany są przerywane — AMCL/scan-match potrzebuje '
                             'ciągłych konturów. 1-2 zwykle wystarczy. Per scenarios '
                             'sweep również generuje morpho-warianty niezależnie od tej '
                             'flagi.')
    parser.add_argument('--close', type=int, default=0,
                        help='binary_closing (dilate→erode) — wypełnia luki BEZ '
                             'pogrubiania ścian. Lepsze dla scan-match niż --dilate '
                             'gdy grubość ścian ma znaczenie.')
    parser.add_argument('--quiet', action='store_true')
    args = parser.parse_args()

    if args.resolution <= 0 or args.resolution > 1.0:
        sys.stderr.write(f'--resolution {args.resolution} poza sensownym zakresem (0.005-1.0)\n')
        return 1

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

    z_min_g, z_max_g = float(pts[:, 2].min()), float(pts[:, 2].max())
    print(f'  X range: [{pts[:,0].min():.2f}, {pts[:,0].max():.2f}]')
    print(f'  Y range: [{pts[:,1].min():.2f}, {pts[:,1].max():.2f}]')
    print(f'  Z range: [{z_min_g:.2f}, {z_max_g:.2f}]')

    if not args.quiet:
        edges = np.linspace(z_min_g, z_max_g, 21)
        h, _ = np.histogram(pts[:, 2], bins=edges)
        print('\n  Z histogram (one # ≈ 2% gęstości):')
        bar_max = max(int(h.max()), 1)
        for i in range(len(h)):
            bar = '#' * int(50 * h[i] / bar_max)
            print(f'    [{edges[i]:+.2f} .. {edges[i+1]:+.2f}]  {h[i]:>7d}  {bar}')

    os.makedirs(args.out_dir, exist_ok=True)
    variants = build_variants_grid(default_res=args.resolution,
                                    default_dilate=args.dilate,
                                    default_close=args.close)
    print(f'\n  default resolution: {args.resolution} m/px ({args.resolution*100:.1f} cm/px)')
    if args.dilate: print(f'  default dilate: {args.dilate}px')
    if args.close: print(f'  default close: {args.close}px')

    print(f'\nGenerating {len(variants)} variants → {args.out_dir}/')
    print('=' * 100)
    manifest = []
    for name, z_min, z_max, resolution, min_pts, dilate, close, comment in variants:
        pgm, x0, y0, n = rasterize(pts, z_min, z_max, resolution, min_pts, dilate, close)
        if pgm is None:
            print(f'  {name:24s}  [{z_min:+.2f}..{z_max:+.2f}] r={resolution}  EMPTY')
            continue
        safe_name = name.replace('+', 'p').replace('-', 'm')
        morpho_tag = ''
        if dilate > 0: morpho_tag += f'_d{dilate}'
        if close > 0: morpho_tag += f'_c{close}'
        base = f'{safe_name}_r{resolution}_m{min_pts}{morpho_tag}'
        write_variant(args.out_dir, base, pgm, x0, y0, resolution)
        rows, cols = pgm.shape
        occ = int((pgm == 0).sum())
        morpho_str = (f' d{dilate}' if dilate else '') + (f' c{close}' if close else '')
        print(f'  {name:24s}  [{z_min:+.2f}..{z_max:+.2f}] r={resolution} '
              f'm={min_pts}{morpho_str} → {cols:>4}×{rows:<4}px  {occ:>6} occ  ({comment})')
        manifest.append({
            'name': name, 'base': base,
            'z_min': z_min, 'z_max': z_max,
            'resolution': resolution, 'min_pts_per_cell': min_pts,
            'dilate': dilate, 'close': close,
            'comment': comment, 'cols': cols, 'rows': rows,
            'occupied_cells': occ, 'source_points': n,
            'pgm': f'{base}.pgm', 'yaml': f'{base}.yaml',
        })

    # Manifest JSON dla map_picker.py
    manifest_path = os.path.join(args.out_dir, 'manifest.json')
    with open(manifest_path, 'w') as f:
        json.dump({
            'source_pcd': os.path.abspath(args.pcd_path),
            'flip_y': args.flip_y, 'flip_x': args.flip_x,
            'generated_at': datetime.utcnow().isoformat() + 'Z',
            'total_points': len(pts),
            'x_range': [float(pts[:, 0].min()), float(pts[:, 0].max())],
            'y_range': [float(pts[:, 1].min()), float(pts[:, 1].max())],
            'z_range': [z_min_g, z_max_g],
            'variants': manifest,
        }, f, indent=2)

    print('=' * 100)
    print(f'\nManifest: {manifest_path}')
    print(f'Browse: python3 tools/map_picker.py {args.out_dir}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
