#!/usr/bin/env python3
"""Convert FAST-LIO output .pcd → 2D nav2 map (.pgm + .yaml).

After mapping with FAST-LIO and saving via `ros2 service call /map_save
std_srvs/srv/Trigger {}`, you get a 3D point cloud (g1_map.pcd). nav2
expects a 2D occupancy grid (.pgm + .yaml). This script bridges them.

Two-stage pipeline:
  1. Filter floor + ceiling (keep only points in a Z slice).
  2. Rasterize XY projection onto a 2D grid, output as PGM + YAML.

Usage:
  python3 tools/fastlio_pcd_to_pgm.py g1_map.pcd \\
      --out-dir ~/maps --name lab_fastlio \\
      --resolution 0.05 --z-min -0.4 --z-max 1.5

Output:
  ~/maps/lab_fastlio.pgm
  ~/maps/lab_fastlio.yaml

Requires:
  pip install "open3d>=0.17,<0.19" "numpy==1.26.4" pillow
"""
from __future__ import annotations

import argparse
import os
import sys

try:
    import numpy as np
    import open3d as o3d
    from PIL import Image
except ImportError as exc:
    sys.stderr.write(
        f'Missing dep: {exc}\n'
        'Install: pip install "open3d>=0.17,<0.19" "numpy==1.26.4" pillow\n'
    )
    sys.exit(1)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pcd_path', help='Input .pcd file from FAST-LIO')
    parser.add_argument('--out-dir', default=os.path.expanduser('~/maps'),
                        help='Output directory (default: ~/maps)')
    parser.add_argument('--name', default='lab_fastlio',
                        help='Output basename (default: lab_fastlio)')
    parser.add_argument('--resolution', type=float, default=0.05,
                        help='Map resolution in meters/pixel (default: 0.05)')
    parser.add_argument('--z-min', type=float, default=-0.4,
                        help='Minimum Z [m] — cuts floor (default: -0.4)')
    parser.add_argument('--z-max', type=float, default=1.5,
                        help='Maximum Z [m] — cuts ceiling (default: 1.5)')
    parser.add_argument('--occupied-thresh', type=float, default=0.65,
                        help='nav2 yaml occupied_thresh (default: 0.65)')
    parser.add_argument('--free-thresh', type=float, default=0.25,
                        help='nav2 yaml free_thresh (default: 0.25)')
    parser.add_argument('--min-points-per-cell', type=int, default=2,
                        help='Cell occupied if >= N points (default: 2). '
                             'Higher = less noise but more holes.')
    parser.add_argument('--flip-y', action='store_true',
                        help='Mirror map along Y axis. Use gdy Mid-360 na G1 jest '
                             'upside-down (FAST-LIO buduje chmurę w lustrzanym '
                             'układzie). Sprawdź /livox/imu — gravity z=-9.81 → '
                             'upside down.')
    parser.add_argument('--flip-x', action='store_true',
                        help='Mirror map along X axis. Rzadkie — przy nietypowym '
                             'mountingu Mid-360.')
    args = parser.parse_args()

    if not os.path.isfile(args.pcd_path):
        sys.stderr.write(f'PCD not found: {args.pcd_path}\n')
        return 1

    os.makedirs(args.out_dir, exist_ok=True)
    out_pgm = os.path.join(args.out_dir, f'{args.name}.pgm')
    out_yaml = os.path.join(args.out_dir, f'{args.name}.yaml')

    print(f'Loading {args.pcd_path}...')
    pcd = o3d.io.read_point_cloud(args.pcd_path)
    pts = np.asarray(pcd.points)
    print(f'  {len(pts)} points')

    # Mirror axis fix — Mid-360 mount orientation.
    if args.flip_y:
        pts = pts.copy()
        pts[:, 1] = -pts[:, 1]
        print('  applied --flip-y (Y axis mirrored)')
    if args.flip_x:
        pts = pts.copy()
        pts[:, 0] = -pts[:, 0]
        print('  applied --flip-x (X axis mirrored)')

    # Z slice — cut floor and ceiling.
    mask = (pts[:, 2] >= args.z_min) & (pts[:, 2] <= args.z_max)
    pts = pts[mask]
    print(f'  {len(pts)} points after Z slice [{args.z_min}, {args.z_max}]')
    if len(pts) == 0:
        sys.stderr.write('No points after Z slice. Adjust --z-min / --z-max.\n')
        return 1

    # XY bounds + grid size.
    x_min, y_min = pts[:, 0].min(), pts[:, 1].min()
    x_max, y_max = pts[:, 0].max(), pts[:, 1].max()
    width_m = x_max - x_min
    height_m = y_max - y_min
    cols = int(np.ceil(width_m / args.resolution))
    rows = int(np.ceil(height_m / args.resolution))
    print(f'  XY bounds: ({x_min:.2f}, {y_min:.2f}) → ({x_max:.2f}, {y_max:.2f})')
    print(f'  Grid: {cols} × {rows} cells @ {args.resolution} m/px')

    # Rasterize: count points per cell.
    col_idx = ((pts[:, 0] - x_min) / args.resolution).astype(int)
    row_idx = ((pts[:, 1] - y_min) / args.resolution).astype(int)
    col_idx = np.clip(col_idx, 0, cols - 1)
    row_idx = np.clip(row_idx, 0, rows - 1)
    count = np.zeros((rows, cols), dtype=np.int32)
    np.add.at(count, (row_idx, col_idx), 1)

    # PGM convention (nav2):
    #   255 = free (white)
    #   0   = occupied (black)
    #   ~205 = unknown (mid gray)
    # In ROS map_server: occupied_thresh=0.65 means cell value <= (1-0.65)*255 = 89
    #   counts as occupied; value >= (1-0.25)*255 = 191 counts as free.
    pgm = np.full((rows, cols), 205, dtype=np.uint8)   # all unknown
    pgm[count >= args.min_points_per_cell] = 0           # occupied
    # Free space: cells with neighbors that are occupied but cell itself empty.
    # Simple heuristic: a cell is "free" if it's empty but within bounding box
    # of any occupied cell. To keep it simple here we just leave non-occupied
    # cells as unknown (205); AMCL will treat them as obstacles to be safe.
    # If you want explicit free space, run a flood-fill from a known free seed,
    # or use ray-tracing from robot path — overkill for first iteration.

    # PGM origin in nav2: bottom-left corner of image.
    # We have rows indexed [0..rows-1] from bottom (y_min) to top (y_max).
    # PIL/numpy save assumes row 0 is TOP. Flip vertically.
    pgm = np.flipud(pgm)

    Image.fromarray(pgm, mode='L').save(out_pgm)
    print(f'Saved: {out_pgm}')

    # YAML: nav2 map_server format.
    yaml_text = (
        f'image: {args.name}.pgm\n'
        f'resolution: {args.resolution}\n'
        f'origin: [{x_min:.4f}, {y_min:.4f}, 0.0]\n'
        f'occupied_thresh: {args.occupied_thresh}\n'
        f'free_thresh: {args.free_thresh}\n'
        f'negate: 0\n'
        f'mode: trinary\n'
    )
    with open(out_yaml, 'w') as f:
        f.write(yaml_text)
    print(f'Saved: {out_yaml}')

    print()
    print('Test:')
    print(f'  ros2 launch g1_courier_bringup real.launch.py map:={out_yaml}')

    return 0


if __name__ == '__main__':
    sys.exit(main())
