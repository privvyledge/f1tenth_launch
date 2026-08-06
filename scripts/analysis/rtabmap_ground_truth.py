#!/usr/bin/env python3
"""Extract the true map->odom of a mapping run from its RTABMap database.

    python3 rtabmap_ground_truth.py <rtabmap.db> [--out truth.csv]

WHY THIS IS NEEDED
------------------
It is tempting to score a localizer on the mapping bag by assuming its true
map->odom is identity, since RTABMap starts map and odom coincident. That is
only true at t=0. RTABMap then *optimizes* the pose graph — that is the whole
point of the loop closures — so from the first correction onward the true
map->odom is a real, growing transform. Scoring against identity charges the
localizer for drift it correctly removed.

The database has both halves per keyframe:

    Node.pose        the odometry pose that keyframe was captured at
    Admin.opt_poses  the same keyframes after graph optimization, in `map`

so the honest ground truth is simply

    map->odom(t_i) = opt_pose_i  o  odom_pose_i^-1

Both are stored as RTABMap `Transform`s: 12 float32 in row-major 3x4.
"""

import argparse
import math
import sqlite3
import struct
import zlib

import numpy as np


def raw(blob):
    """RTABMap zlib-compresses the Admin blobs but not Node.pose. Sniff it."""
    if blob is None:
        return None
    b = bytes(blob)
    try:
        return zlib.decompress(b)
    except zlib.error:
        return b


def unpack_transforms(blob, n):
    vals = np.array(struct.unpack("<%df" % (12 * n), blob[: 48 * n]), dtype=np.float64)
    return vals.reshape(n, 3, 4)


def as_2d(m):
    """(x, y, yaw) from a 3x4 rigid transform."""
    return m[0, 3], m[1, 3], math.atan2(m[1, 0], m[0, 0])


def compose_inv(a, b):
    """a o b^-1, both 3x4, returned as (x, y, yaw) in 2D."""
    Ra, ta = a[:, :3], a[:, 3]
    Rb, tb = b[:, :3], b[:, 3]
    R = Ra @ Rb.T
    t = ta - R @ tb
    return t[0], t[1], math.atan2(R[1, 0], R[0, 0])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("db")
    ap.add_argument("--out")
    args = ap.parse_args()

    con = sqlite3.connect(args.db)
    ids_blob, poses_blob = con.execute(
        "SELECT opt_ids, opt_poses FROM Admin").fetchone()
    if not ids_blob or not poses_blob:
        raise SystemExit("this database has no optimized graph (Admin.opt_poses "
                         "is empty) — it was never closed out properly")

    ids_blob, poses_blob = raw(ids_blob), raw(poses_blob)
    n = len(ids_blob) // 4
    if len(poses_blob) < 48 * n:
        raise SystemExit(f"opt_poses holds {len(poses_blob)} bytes, too few for "
                         f"{n} poses — unexpected encoding")
    opt_ids = np.array(struct.unpack("<%di" % n, ids_blob[: 4 * n]))
    opt = unpack_transforms(poses_blob, n)
    print(f"optimized graph: {n} poses")

    nodes = {}
    for nid, stamp, pose in con.execute(
            "SELECT id, stamp, pose FROM Node WHERE pose IS NOT NULL"):
        nodes[nid] = (stamp, unpack_transforms(raw(pose), 1)[0])

    rows = []
    for i, nid in enumerate(opt_ids):
        if nid not in nodes:
            continue
        stamp, odom = nodes[nid]
        x, y, yaw = compose_inv(opt[i], odom)
        rows.append((stamp, x, y, yaw))
    rows.sort()
    arr = np.array(rows)
    if not len(arr):
        raise SystemExit("optimized ids do not match any Node rows")

    d = np.hypot(arr[:, 1], arr[:, 2])
    print(f"matched {len(arr)} keyframes over {arr[-1,0]-arr[0,0]:.1f} s")
    print("true map->odom (this is the correction RTABMap applied, "
          "NOT localizer error):")
    print(f"  translation  mean {d.mean()*1000:7.1f} mm  "
          f"p95 {np.percentile(d,95)*1000:7.1f} mm  max {d.max()*1000:7.1f} mm")
    yy = np.abs(arr[:, 3])
    print(f"  yaw          mean {math.degrees(yy.mean()):7.2f} deg  "
          f"max {math.degrees(yy.max()):7.2f} deg")
    print(f"  final        x={arr[-1,1]:+.3f} y={arr[-1,2]:+.3f} "
          f"yaw={math.degrees(arr[-1,3]):+.2f} deg")

    if args.out:
        with open(args.out, "w") as f:
            f.write("stamp,x,y,yaw\n")
            for r in arr:
                f.write("%.9f,%.6f,%.6f,%.6f\n" % tuple(r))
        print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
