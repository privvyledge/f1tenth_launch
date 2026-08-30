#!/usr/bin/env python3
"""Symmetric 2D registration of the occupancy grid against the 3D cloud.

Why a second script: map_cloud_align.py scores one direction only -- for each
occupied grid cell, the distance to the nearest cloud point. The cloud is far
denser than the grid and does not cover its footprint, so ANY pose that packs
grid cells into the dense annulus scores well. That metric reports a confident
+31 deg optimum that is entirely spurious, and it reports "aligned" at zero
shift for the same reason: it cannot tell alignment from density.

This one rasterizes both to binary images at the grid resolution and scores a
symmetric F1 over cell coverage in BOTH directions, so a rotation that smears
cloud cells outside the grid is penalised. Translation is searched exhaustively
by FFT cross-correlation at every yaw.
"""
import argparse
import numpy as np
from scipy.signal import fftconvolve
from scipy.ndimage import binary_dilation
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from map_cloud_align import grid_occupied_xy, read_pcd_xyz


def rasterize(xy, res, lo, shape):
    ij = np.floor((xy - lo) / res).astype(int)
    ok = (ij[:, 0] >= 0) & (ij[:, 0] < shape[1]) & (ij[:, 1] >= 0) & (ij[:, 1] < shape[0])
    img = np.zeros(shape, dtype=bool)
    img[ij[ok, 1], ij[ok, 0]] = True
    return img


def f1(a, b, tol_cells):
    """symmetric coverage: fraction of a covered by dilated b, and vice versa."""
    st = np.ones((2 * tol_cells + 1,) * 2, dtype=bool)
    ad, bd = binary_dilation(a, st), binary_dilation(b, st)
    na, nb = a.sum(), b.sum()
    if na == 0 or nb == 0:
        return 0.0, 0.0, 0.0
    pa = (a & bd).sum() / na          # grid cells explained by cloud
    pb = (b & ad).sum() / nb          # cloud cells explained by grid
    return (0.0 if pa + pb == 0 else 2 * pa * pb / (pa + pb)), pa, pb


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pgm', default='data/maps/20260805/rtabmap_2d_final.pgm')
    ap.add_argument('--yaml', dest='yml', default='data/maps/20260805/rtabmap_2d_final.yaml')
    ap.add_argument('--pcd', default='data/maps/20260805/cloud_voxel_0p05.pcd')
    ap.add_argument('--zmin', type=float, default=0.10)
    ap.add_argument('--zmax', type=float, default=1.20)
    ap.add_argument('--res', type=float, default=0.05)
    ap.add_argument('--tol-cells', type=int, default=1)
    ap.add_argument('--yaw-step', type=float, default=1.0)
    ap.add_argument('--yaw-min', type=float, default=0.0)
    ap.add_argument('--yaw-max', type=float, default=360.0)
    args = ap.parse_args()

    grid, _meta = grid_occupied_xy(args.pgm, args.yml)
    cl = read_pcd_xyz(args.pcd)
    band = cl[(cl[:, 2] >= args.zmin) & (cl[:, 2] <= args.zmax)][:, :2]
    print('grid occupied cells : %d' % len(grid))
    print('cloud pts in z band : %d of %d  (z %.2f..%.2f)'
          % (len(band), len(cl), args.zmin, args.zmax))

    cx, cy = grid[:, 0].mean(), grid[:, 1].mean()
    pad = 3.0
    lo = np.minimum(grid.min(0), band.min(0)) - pad
    hi = np.maximum(grid.max(0), band.max(0)) + pad
    shape = tuple((np.ceil((hi - lo) / args.res)).astype(int)[::-1])
    G = rasterize(grid, args.res, lo, shape)

    rows = []
    for deg in np.arange(args.yaw_min, args.yaw_max, args.yaw_step):
        th = np.radians(deg)
        c, s = np.cos(th), np.sin(th)
        p = band - [cx, cy]
        r = np.column_stack([c * p[:, 0] - s * p[:, 1], s * p[:, 0] + c * p[:, 1]]) + [cx, cy]
        C = rasterize(r, args.res, lo, shape)
        # best translation by cross-correlation of the two binary images
        corr = fftconvolve(G.astype(np.float32), C[::-1, ::-1].astype(np.float32), mode='same')
        k = np.unravel_index(np.argmax(corr), corr.shape)
        dy = k[0] - shape[0] // 2
        dx = k[1] - shape[1] // 2
        Cs = np.zeros_like(C)
        ys, xs = np.nonzero(C)
        ys2, xs2 = ys + dy, xs + dx
        m = (ys2 >= 0) & (ys2 < shape[0]) & (xs2 >= 0) & (xs2 < shape[1])
        Cs[ys2[m], xs2[m]] = True
        sc, pa, pb = f1(G, Cs, args.tol_cells)
        rows.append((deg, sc, pa, pb, dx * args.res, dy * args.res))

    rows.sort(key=lambda r: -r[1])
    print('\n  yaw     F1     grid->cloud  cloud->grid   best shift (m)')
    for r in rows[:8]:
        print('  %6.1f  %.4f    %.3f        %.3f       (%+.2f, %+.2f)'
              % (r[0], r[1], r[2], r[3], r[4], r[5]))
    zs = [r for r in rows if abs(r[0]) < 1e-9]
    best = rows[0]
    if zs:
        z = zs[0]
        print('\n  zero yaw: F1 %.4f  grid->cloud %.3f  cloud->grid %.3f  shift (%+.2f, %+.2f)'
              % (z[1], z[2], z[3], z[4], z[5]))
        print('  best yaw: %.2f deg  (F1 %.4f)   delta over zero: %+.4f'
              % (best[0], best[1], best[1] - z[1]))
    else:
        print('\n  best yaw: %.2f deg  (F1 %.4f)  shift (%+.2f, %+.2f)'
              % (best[0], best[1], best[4], best[5]))


if __name__ == '__main__':
    main()
