#!/usr/bin/env python3
"""Compare the 2D occupancy grid and the 3D point cloud of a map set, offline.

Reads the .pgm/.yaml pair and the .pcd; needs no robot. Two modes:

  --overlay   render both, as published, to a PNG. USE THIS FIRST.
  (default)   sweep yaw/shift, scoring by median distance from an occupied grid cell
              to the nearest projected cloud cell.

READ THIS BEFORE TRUSTING THE SWEEP. The median-nearest-neighbour score is BIASED and
on the 20260805 map set it reports a confident, entirely spurious optimum at +31 deg
(median 0.018 m, against 0.051 m at zero). The cause: the cloud is much denser than the
grid and does not cover the grid's full footprint, so a rotation that packs grid cells
into the cloud's dense annulus scores well without any structural agreement at all. The
overlay shows the two maps tracing the same ring with no rotation, which is what the
2026-08-10 co-registration measurement found independently.

The trap is worse than a wrong number: +31 deg happened to match a rotation the operator
believed they were seeing, so the score would have "confirmed" it. Render the overlay and
look before reporting any alignment figure from this file.

Note also that this lab is NOT rectangular -- it is a closed ring -- so wall-orientation
histograms and any other method assuming dominant perpendicular walls do not apply here.
"""
import argparse
import re
import struct
import sys

import numpy as np
from scipy.spatial import cKDTree
import yaml


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    if not data.startswith(b'P5'):
        raise ValueError('only binary P5 PGM is supported: %s' % path)
    # header: magic, width, height, maxval -- comments start with '#'
    tokens, i = [], 2
    while len(tokens) < 3:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while data[i:i + 1] not in (b'\n', b''):
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        tokens.append(int(data[i:j]))
        i = j
    i += 1  # single whitespace after maxval
    w, h, maxval = tokens
    img = np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w)
    return img, maxval


def grid_occupied_xy(pgm_path, yaml_path):
    meta = yaml.safe_load(open(yaml_path))
    img, maxval = read_pgm(pgm_path)
    h, w = img.shape
    res = float(meta['resolution'])
    ox, oy, _ = (list(meta['origin']) + [0, 0, 0])[:3]
    occ_thresh = float(meta.get('occupied_thresh', 0.65))
    negate = int(meta.get('negate', 0))
    p = img.astype(np.float64) / maxval
    # nav2 map_server trinary: occupancy = 1 - p for negate=0
    occ = p if negate else (1.0 - p)
    rows, cols = np.nonzero(occ >= occ_thresh)
    # row 0 of the image is the TOP row = max y
    x = ox + (cols + 0.5) * res
    y = oy + (h - rows - 0.5) * res
    return np.column_stack([x, y]), meta


def read_pcd_xyz(path):
    with open(path, 'rb') as f:
        raw = f.read()
    hdr_end = raw.index(b'DATA')
    nl = raw.index(b'\n', hdr_end)
    header = raw[:nl].decode('ascii', 'replace')
    body = raw[nl + 1:]
    fields = re.search(r'^FIELDS (.*)$', header, re.M).group(1).split()
    sizes = [int(v) for v in re.search(r'^SIZE (.*)$', header, re.M).group(1).split()]
    counts_m = re.search(r'^COUNT (.*)$', header, re.M)
    counts = [int(v) for v in counts_m.group(1).split()] if counts_m else [1] * len(fields)
    npts = int(re.search(r'^POINTS (\d+)$', header, re.M).group(1))
    fmt = re.search(r'^DATA (\S+)$', header, re.M).group(1)
    if fmt != 'binary':
        raise ValueError('only DATA binary is supported (got %s)' % fmt)
    stride = sum(s * c for s, c in zip(sizes, counts))
    off, offsets = 0, {}
    for name, s, c in zip(fields, sizes, counts):
        offsets[name] = off
        off += s * c
    arr = np.frombuffer(body[:npts * stride], dtype=np.uint8).reshape(npts, stride)
    out = []
    for name in ('x', 'y', 'z'):
        o = offsets[name]
        out.append(arr[:, o:o + 4].copy().view(np.float32).ravel().astype(np.float64))
    xyz = np.column_stack(out)
    return xyz[np.isfinite(xyz).all(axis=1)]


def score(grid_xy, cloud_tree_pts, yaw, dx, dy, cx, cy):
    c, s = np.cos(yaw), np.sin(yaw)
    p = grid_xy - np.array([cx, cy])
    r = np.column_stack([c * p[:, 0] - s * p[:, 1], s * p[:, 0] + c * p[:, 1]])
    r += np.array([cx + dx, cy + dy])
    d, _ = cloud_tree_pts.query(r, k=1)
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pgm', default='data/maps/20260805/rtabmap_2d_final.pgm')
    ap.add_argument('--yaml', dest='yml', default='data/maps/20260805/rtabmap_2d_final.yaml')
    ap.add_argument('--pcd', default='data/maps/20260805/cloud_voxel_0p05.pcd')
    ap.add_argument('--zmin', type=float, default=None, help='keep cloud points above this z')
    ap.add_argument('--zmax', type=float, default=None, help='keep cloud points below this z')
    ap.add_argument('--yaw-range', type=float, default=45.0, help='+/- degrees to sweep')
    ap.add_argument('--yaw-step', type=float, default=1.0)
    ap.add_argument('--shift', type=float, default=0.0, help='+/- metres of x/y shift to sweep')
    ap.add_argument('--shift-step', type=float, default=0.1)
    ap.add_argument('--overlay', metavar='PNG',
                    help='render grid and cloud as published to this PNG and exit '
                         '(do this before running the sweep)')
    args = ap.parse_args()

    grid_xy, meta = grid_occupied_xy(args.pgm, args.yml)
    cloud = read_pcd_xyz(args.pcd)
    print('grid: %d occupied cells, res %.3f, origin %s'
          % (len(grid_xy), meta['resolution'], meta['origin']))
    print('grid extent  x [%.2f, %.2f]  y [%.2f, %.2f]'
          % (grid_xy[:, 0].min(), grid_xy[:, 0].max(),
             grid_xy[:, 1].min(), grid_xy[:, 1].max()))
    print('cloud: %d points' % len(cloud))
    print('cloud extent x [%.2f, %.2f]  y [%.2f, %.2f]  z [%.2f, %.2f]'
          % (cloud[:, 0].min(), cloud[:, 0].max(), cloud[:, 1].min(),
             cloud[:, 1].max(), cloud[:, 2].min(), cloud[:, 2].max()))

    sel = np.ones(len(cloud), dtype=bool)
    if args.zmin is not None:
        sel &= cloud[:, 2] >= args.zmin
    if args.zmax is not None:
        sel &= cloud[:, 2] <= args.zmax
    cl = cloud[sel]
    print('cloud after z filter: %d points' % len(cl))
    if len(cl) == 0:
        sys.exit('no cloud points survive the z filter')

    if args.overlay:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 9))
        ax.scatter(cl[:, 0], cl[:, 1], s=0.5, c='tab:red', alpha=0.3, linewidths=0,
                   label='cloud %s (n=%d)' % (args.pcd.split('/')[-1], len(cl)))
        ax.scatter(grid_xy[:, 0], grid_xy[:, 1], s=3.5, c='k', marker='s',
                   label='grid %s (n=%d)' % (args.pgm.split('/')[-1], len(grid_xy)))
        ax.plot(0, 0, marker='+', ms=14, mew=2, c='tab:green', label='map origin')
        ax.set_aspect('equal')
        ax.grid(alpha=0.25)
        ax.legend(loc='upper left', fontsize=8)
        ax.set_xlabel('x [m], map frame')
        ax.set_ylabel('y [m], map frame')
        ax.set_title('grid vs cloud, as published, no alignment applied')
        fig.tight_layout()
        fig.savefig(args.overlay, dpi=110)
        print('wrote %s' % args.overlay)
        return

    tree = cKDTree(cl[:, :2])
    cx, cy = grid_xy[:, 0].mean(), grid_xy[:, 1].mean()
    print('rotation centre (grid centroid): (%.3f, %.3f)' % (cx, cy))

    yaws = np.arange(-args.yaw_range, args.yaw_range + 1e-9, args.yaw_step)
    if args.shift > 0:
        shifts = np.arange(-args.shift, args.shift + 1e-9, args.shift_step)
    else:
        shifts = np.array([0.0])

    best = None
    rows = []
    for yaw_deg in yaws:
        yaw = np.radians(yaw_deg)
        for dx in shifts:
            for dy in shifts:
                d = score(grid_xy, tree, yaw, dx, dy, cx, cy)
                med = float(np.median(d))
                frac = float((d <= meta['resolution']).mean())
                rows.append((yaw_deg, dx, dy, med, frac))
                if best is None or med < best[3]:
                    best = (yaw_deg, dx, dy, med, frac)

    print('\nyaw sweep at best shift (median nearest-cloud distance, and fraction within 1 cell):')
    by_yaw = {}
    for yaw_deg, dx, dy, med, frac in rows:
        if yaw_deg not in by_yaw or med < by_yaw[yaw_deg][0]:
            by_yaw[yaw_deg] = (med, frac, dx, dy)
    for yaw_deg in yaws:
        med, frac, dx, dy = by_yaw[yaw_deg]
        mark = '  <== best' if abs(yaw_deg - best[0]) < 1e-9 else ''
        if abs(yaw_deg) % 5 < 1e-9 or mark:
            print('  yaw %+7.2f deg  dx %+5.2f dy %+5.2f  median %.3f m  within-1-cell %.1f %%%s'
                  % (yaw_deg, dx, dy, med, 100 * frac, mark))

    print('\nBEST: yaw %+.2f deg, shift (%+.2f, %+.2f), median %.3f m, within 1 cell %.1f %%'
          % (best[0], best[1], best[2], best[3], 100 * best[4]))
    zero = by_yaw[0.0] if 0.0 in by_yaw else None
    if zero is not None:
        print('ZERO: yaw  +0.00 deg, shift (%+.2f, %+.2f), median %.3f m, within 1 cell %.1f %%'
              % (zero[2], zero[3], zero[0], 100 * zero[1]))


if __name__ == '__main__':
    main()
