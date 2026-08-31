#!/usr/bin/env python3
"""Re-express an rtabmap-export cloud in the frame of the saved 2D grid.

    python3 cloud_to_grid_frame.py <rtabmap.db> <cloud_in.ply|.pcd> --out fixed.pcd

WHY THIS IS NEEDED
------------------
`rtabmap-export` and the `rtabmap` ROS node do not anchor the optimized pose
graph at the same node, so their two `map` frames differ by a rigid transform:

  * `rtabmap-export` re-roots the graph at the FIRST node, so its clouds come
    out in the frame the run STARTED in (the odometry frame).
  * the ROS node publishes `grid_prob_map` from `Admin.opt_poses`, which is
    anchored so the LATEST pose agrees with odometry. `map_saver` captures the
    grid in THAT frame, and that is the frame AMCL and Nav2 then localize in.

The offset between them is exactly the yaw the graph optimization removed over
the run.  For `rtabmap_final_nf.db` (2026-08-05) that is 25.33 deg, which is
why the shipped cloud and grid did not overlay.  See bug-265.

The transform is read out of a database, not fitted to the two maps: it is
`opt_pose o odom_pose^-1` at the first keyframe -- the same quantity
`rtabmap_ground_truth.py` writes as row 1 of its truth CSV.

CAVEAT, and it is the reason for `--transform-db`: only a database that was
closed out cleanly carries `Admin.opt_poses`.  Of the 2026-08-05 builds only
`rtabmap_final.db` does; `rtabmap_final_nf.db` -- the one the shipped artifacts
come from -- does not, which is also why `rtabmap-export --opt 2` silently falls
back to a full re-optimization on it.  Both are builds of the same bag and their
graphs agree, so the transform from `rtabmap_final.db` aligns the `_nf` cloud to
the `_nf` grid with a residual of 0.00 deg and 0.00 m.  State which database the
transform came from whenever you use one that is not the cloud's own.

Every PCD field is carried through untouched -- `rgb` above all. The first
version of this script wrote `FIELDS x y z`, which renders as a uniform grey
cloud in RViz with no error anywhere (bug-266).

Verify the result with `map_cloud_align2.py`: a correctly transformed cloud
peaks at yaw 0 with a zero shift. Then check the header still says
`FIELDS x y z rgb` before shipping it.
"""
import argparse
import math
import struct
import re
import sqlite3

import numpy as np

from rtabmap_ground_truth import raw, unpack_transforms, compose_inv


def db_map_from_export(db):
    """(x, y, yaw) carrying export-frame points into the published map frame."""
    con = sqlite3.connect(db)
    ids_blob, poses_blob = con.execute(
        "SELECT opt_ids, opt_poses FROM Admin").fetchone()
    if not ids_blob or not poses_blob:
        raise SystemExit(
            "no optimized graph in this database (Admin.opt_poses is empty) -- "
            "it was never closed out.  Pass --transform-db pointing at a sibling "
            "build of the same bag that was, and say so in the write-up.")
    ids_blob, poses_blob = raw(ids_blob), raw(poses_blob)
    n = len(ids_blob) // 4
    opt_ids = np.array(struct.unpack("<%di" % n, ids_blob[: 4 * n]))
    opt = unpack_transforms(poses_blob, n)
    nodes = {}
    for nid, stamp, pose in con.execute(
            "SELECT id, stamp, pose FROM Node WHERE pose IS NOT NULL"):
        nodes[nid] = (stamp, unpack_transforms(raw(pose), 1)[0])
    rows = []
    for i, nid in enumerate(opt_ids):
        if nid in nodes:
            rows.append((nodes[nid][0], i))
    if not rows:
        raise SystemExit("optimized ids do not match any Node rows")
    rows.sort()
    i = rows[0][1]
    nid = opt_ids[i]
    return compose_inv(opt[i], nodes[nid][1])


def read_pcd_records(path):
    """Return (header_text, npts, stride, uint8 record array, field offsets).

    Every field is carried through untouched -- `rgb` above all.  The first
    version of this script wrote `FIELDS x y z` and silently dropped colour,
    which renders as a uniform grey cloud in RViz (bug-266).
    """
    with open(path, "rb") as f:
        buf = f.read()
    hdr_end = buf.index(b"DATA")
    nl = buf.index(b"\n", hdr_end)
    header = buf[:nl].decode("ascii", "replace")
    body = buf[nl + 1:]
    if re.search(r"^DATA (\S+)$", header, re.M).group(1) != "binary":
        raise SystemExit("only DATA binary PCDs are supported (got ascii)")
    fields = re.search(r"^FIELDS (.*)$", header, re.M).group(1).split()
    sizes = [int(v) for v in re.search(r"^SIZE (.*)$", header, re.M).group(1).split()]
    cm = re.search(r"^COUNT (.*)$", header, re.M)
    counts = [int(v) for v in cm.group(1).split()] if cm else [1] * len(fields)
    npts = int(re.search(r"^POINTS (\d+)$", header, re.M).group(1))
    stride = sum(sz * c for sz, c in zip(sizes, counts))
    off, offsets = 0, {}
    for name, sz, c in zip(fields, sizes, counts):
        offsets[name] = off
        off += sz * c
    rec = np.frombuffer(body[: npts * stride], dtype=np.uint8).reshape(npts, stride).copy()
    for name in ("x", "y", "z"):
        if name not in offsets:
            raise SystemExit("PCD has no '%s' field" % name)
    return header, npts, stride, rec, offsets, fields


def get_xyz(rec, offsets):
    return np.column_stack([
        rec[:, offsets[n]:offsets[n] + 4].copy().view("<f4").ravel().astype(np.float64)
        for n in ("x", "y", "z")])


def set_xy(rec, offsets, x, y):
    for name, vals in (("x", x), ("y", y)):
        o = offsets[name]
        rec[:, o:o + 4] = vals.astype("<f4").view(np.uint8).reshape(-1, 4)


def write_pcd_records(path, header, rec):
    n = len(rec)
    hdr = re.sub(r"^WIDTH .*$", "WIDTH %d" % n, header, flags=re.M)
    hdr = re.sub(r"^HEIGHT .*$", "HEIGHT 1", hdr, flags=re.M)
    hdr = re.sub(r"^POINTS .*$", "POINTS %d" % n, hdr, flags=re.M)
    with open(path, "wb") as f:
        f.write(hdr.encode("ascii") + b"\n")
        f.write(rec.tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("db")
    ap.add_argument("cloud")
    ap.add_argument("--out", required=True)
    ap.add_argument("--transform-db",
                    help="read the frame offset from this database instead of "
                         "the cloud's own (needed when the cloud's database was "
                         "never closed out and so has no Admin.opt_poses)")
    ap.add_argument("--voxel", type=float, default=0.0,
                    help="voxel size in m applied after the transform (0 = off)")
    args = ap.parse_args()

    tdb = args.transform_db or args.db
    tdb = args.transform_db or args.db
    x, y, yaw = db_map_from_export(tdb)
    print("map <- export  x=%+.4f y=%+.4f yaw=%+.3f deg   (from %s)"
          % (x, y, math.degrees(yaw), tdb))

    header, npts, stride, rec, offsets, fields = read_pcd_records(args.cloud)
    print("points in %d   fields %s" % (npts, " ".join(fields)))
    p = get_xyz(rec, offsets)
    c, s_ = math.cos(yaw), math.sin(yaw)
    nx = c * p[:, 0] - s_ * p[:, 1] + x
    ny = s_ * p[:, 0] + c * p[:, 1] + y
    set_xy(rec, offsets, nx, ny)

    if args.voxel > 0:
        key = np.floor(np.column_stack([nx, ny, p[:, 2]]) / args.voxel).astype(np.int64)
        _, idx = np.unique(key, axis=0, return_index=True)
        rec = rec[np.sort(idx)]
        print("points out %d after %.3f m voxel" % (len(rec), args.voxel))

    write_pcd_records(args.out, header, rec)
    print("wrote %s   fields preserved: %s" % (args.out, " ".join(fields)))


if __name__ == "__main__":
    main()
