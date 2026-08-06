#!/usr/bin/env python3
"""Score an offline localization run produced by 51_localize_offline.sh.

    python3 check_map_frame.py <loc_bag> [--map map.yaml] [--control]

What it answers, in the order that matters:

1. Did a map->odom edge actually get broadcast, and how steady is it?
   A localizer that never received its initial pose still broadcasts a
   perfectly steady identity, so steadiness alone proves nothing.
2. --control: how far from identity is it? For ``mapping_drive_170025`` the
   true map->odom IS identity by construction, so any deviation is this
   pipeline's own error on this map. That number is the ceiling on what can be
   promised for the other two runs.
3. Where does the composed map->base_link path sit on the map? A path that
   leaves free space is localized wrong regardless of what its covariance says.

The trinary PGM convention is load-bearing: occupied 0, free 254, unknown 205.
Treating ">=200 is free" folds unknown into free and makes a wrong path look
plausible.
"""

import argparse
import math
import os
import sys

import numpy as np
import yaml

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


FREE, UNKNOWN, OCCUPIED = 254, 205, 0


def yaw_of(q):
    """Yaw from a quaternion, flattened to 2D (two_d_mode everywhere here)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def read_bag(path, wanted=None):
    """Yield (topic, msg, t_ns). Storage id is sniffed from metadata.yaml."""
    meta = yaml.safe_load(open(os.path.join(path, "metadata.yaml")))
    info = meta["rosbag2_bagfile_information"]
    storage = info.get("storage_identifier", "mcap")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id=storage),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if wanted is not None:
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(wanted)))
    while reader.has_next():
        topic, data, t = reader.read_next()
        yield topic, deserialize_message(data, get_message(types[topic])), t


def collect(bag, ns):
    """Pull map->odom, odom->base_link and amcl_pose out of a localization bag."""
    tf_topic = f"/{ns}/tf"
    odom_topic = f"/{ns}/odometry/local"
    amcl_topic = f"/{ns}/amcl_pose"

    map_odom, odom_base, amcl = [], [], []
    for topic, msg, _ in read_bag(bag, {tf_topic, odom_topic, amcl_topic}):
        if topic == tf_topic:
            for tr in msg.transforms:
                stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                p, c = tr.header.frame_id.lstrip("/"), tr.child_frame_id.lstrip("/")
                rec = (stamp, tr.transform.translation.x,
                       tr.transform.translation.y, yaw_of(tr.transform.rotation))
                if (p, c) == ("map", "odom"):
                    map_odom.append(rec)
                elif (p, c) == ("odom", "base_link"):
                    odom_base.append(rec)
        elif topic == odom_topic:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            odom_base.append((stamp, msg.pose.pose.position.x,
                              msg.pose.pose.position.y, yaw_of(msg.pose.pose.orientation)))
        elif topic == amcl_topic:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            cov = np.asarray(msg.pose.covariance).reshape(6, 6)
            amcl.append((stamp, msg.pose.pose.position.x, msg.pose.pose.position.y,
                         yaw_of(msg.pose.pose.orientation),
                         cov[0, 0], cov[1, 1], cov[5, 5]))

    dedup = {}
    for r in odom_base:
        dedup[round(r[0], 6)] = r
    odom_base = [dedup[k] for k in sorted(dedup)]
    return (np.array(map_odom), np.array(odom_base), np.array(amcl))


def compose(mo, ob):
    """map->base_link at every odom->base_link stamp.

    map->odom is piecewise constant between localizer updates, so a
    zero-order hold (last value at or before the stamp) is the correct
    interpolation, not a linear blend.
    """
    idx = np.searchsorted(mo[:, 0], ob[:, 0], side="right") - 1
    idx = np.clip(idx, 0, len(mo) - 1)
    dx, dy, dth = mo[idx, 1], mo[idx, 2], mo[idx, 3]
    c, s = np.cos(dth), np.sin(dth)
    x = dx + c * ob[:, 1] - s * ob[:, 2]
    y = dy + s * ob[:, 1] + c * ob[:, 2]
    th = np.arctan2(np.sin(dth + ob[:, 3]), np.cos(dth + ob[:, 3]))
    return np.column_stack([ob[:, 0], x, y, th])


def load_map(yaml_path):
    cfg = yaml.safe_load(open(yaml_path))
    pgm = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), cfg["image"])
    with open(pgm, "rb") as f:
        assert f.readline().strip() == b"P5", "expected a binary P5 PGM"
        dims = f.readline()
        while dims.startswith(b"#"):
            dims = f.readline()
        w, h = (int(v) for v in dims.split())
        f.readline()  # maxval
        img = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    return img, float(cfg["resolution"]), cfg["origin"]


def classify(path, img, res, origin):
    h, w = img.shape
    px = np.floor((path[:, 1] - origin[0]) / res).astype(int)
    py = h - 1 - np.floor((path[:, 2] - origin[1]) / res).astype(int)
    inside = (px >= 0) & (px < w) & (py >= 0) & (py < h)
    vals = np.full(len(path), -1, dtype=int)
    vals[inside] = img[py[inside], px[inside]]
    n = len(path)
    return {
        "off_map": int((~inside).sum()) / n,
        "free": int((vals == FREE).sum()) / n,
        "unknown": int((vals == UNKNOWN).sum()) / n,
        "occupied": int((vals == OCCUPIED).sum()) / n,
        "other": int(((vals != FREE) & (vals != UNKNOWN) & (vals != OCCUPIED)
                      & (vals != -1)).sum()) / n,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--map")
    ap.add_argument("--ns", default=os.environ.get("NS", "gosling1"))
    ap.add_argument("--control", action="store_true",
                    help="ASSUME the true map->odom is identity. Almost certainly "
                         "wrong — see --truth — kept only to reproduce the old number")
    ap.add_argument("--truth",
                    help="CSV from rtabmap_ground_truth.py: the real map->odom(t) "
                         "for the mapping bag, against which localizer error is "
                         "actually measurable")
    ap.add_argument("--skip", type=float, default=0.0, metavar="SEC",
                    help="Drop the first SEC seconds of map->odom before scoring. "
                         "For comparing a SEEDED localizer against an UNSEEDED one: "
                         "nav2_amcl accepts an /initialpose seed and starts "
                         "converged, whereas ekf_map has no seed path here and "
                         "starts from its own zero state, so it spends ~30 s "
                         "slewing in from the origin. Averaging that transient in "
                         "compares startup behaviour, not tracking quality. Report "
                         "the unskipped number too — the transient is real.")
    args = ap.parse_args()

    mo, ob, amcl = collect(args.bag, args.ns)

    if args.skip > 0 and len(mo):
        keep = (mo[:, 0] - mo[0, 0]) >= args.skip
        dropped = len(mo) - int(keep.sum())
        mo = mo[keep]
        print(f"  [--skip {args.skip:g}s] dropped {dropped} of "
              f"{dropped + len(mo)} map->odom samples")
        if len(mo) == 0:
            sys.exit("  --skip dropped every sample")

    print(f"== {os.path.basename(args.bag.rstrip('/'))}")
    if len(ob) == 0:
        sys.exit("no odom->base_link in this bag — nothing to compose")
    span = ob[-1, 0] - ob[0, 0]
    print(f"  odom->base_link  {len(ob):6d} poses  {span:6.1f} s  "
          f"{len(ob)/span:5.1f} Hz")

    if len(mo) == 0:
        sys.exit("  NO map->odom BROADCAST — the localizer never owned the edge")
    mspan = mo[-1, 0] - mo[0, 0]
    print(f"  map->odom        {len(mo):6d} tfs    {mspan:6.1f} s  "
          f"{len(mo)/max(mspan, 1e-9):5.1f} Hz")

    trans = np.hypot(mo[:, 1], mo[:, 2])
    print(f"  map->odom x      {mo[:,1].min():+7.3f} .. {mo[:,1].max():+7.3f} m")
    print(f"  map->odom y      {mo[:,2].min():+7.3f} .. {mo[:,2].max():+7.3f} m")
    print(f"  map->odom yaw    {math.degrees(mo[:,3].min()):+7.2f} .. "
          f"{math.degrees(mo[:,3].max()):+7.2f} deg")
    print(f"  final            x={mo[-1,1]:+.3f} y={mo[-1,2]:+.3f} "
          f"yaw={math.degrees(mo[-1,3]):+.2f} deg")

    # Movement of the correction itself. A localizer that is tracking well
    # nudges map->odom; one that is lost slews it.
    step = np.hypot(np.diff(mo[:, 1]), np.diff(mo[:, 2]))
    if len(step):
        print(f"  correction step  max {step.max()*1000:6.1f} mm   "
              f"p95 {np.percentile(step,95)*1000:6.1f} mm")

    if args.truth:
        # RTABMap's own optimized graph, i.e. the transform the mapper itself
        # applied at each keyframe. Sampling it with a zero-order hold matches
        # how map->odom is actually consumed.
        gt = np.loadtxt(args.truth, delimiter=",", skiprows=1)
        j = np.clip(np.searchsorted(gt[:, 0], mo[:, 0], side="right") - 1,
                    0, len(gt) - 1)
        ex = mo[:, 1] - gt[j, 1]
        ey = mo[:, 2] - gt[j, 2]
        eth = np.array([abs(wrap(a - b)) for a, b in zip(mo[:, 3], gt[j, 3])])
        e = np.hypot(ex, ey)
        print("\n  -- vs RTABMap's optimized graph (real ground truth) --")
        print(f"  translation err  mean {e.mean()*1000:7.1f} mm   "
              f"p95 {np.percentile(e,95)*1000:7.1f} mm   max {e.max()*1000:7.1f} mm")
        print(f"  yaw err          mean {math.degrees(eth.mean()):7.2f} deg  "
              f"p95 {math.degrees(np.percentile(eth,95)):7.2f} deg  "
              f"max {math.degrees(eth.max()):7.2f} deg")
        bar = 0.126
        print(f"  vs LUCIO's 126 mm bar: "
              f"{100*(e < bar).mean():.1f} % of samples inside it")

        # A single mean hides the shape that matters here. Localization error on
        # a seeded run is not stationary: it is large until the filter pulls in
        # from the seed, then small. Reporting only the mean makes a converged
        # run look mediocre and an unconverged one look acceptable.
        rel = mo[:, 0] - mo[0, 0]
        print("    window      n   mean    p95   yaw")
        for lo in range(0, int(rel.max()) + 1, 15):
            m = (rel >= lo) & (rel < lo + 15)
            if m.sum():
                print(f"    {lo:3d}-{lo+15:3d} s {m.sum():5d} "
                      f"{e[m].mean()*1000:6.1f} {np.percentile(e[m],95)*1000:6.1f} mm "
                      f"{math.degrees(eth[m].mean()):5.2f} deg")

    if args.control:
        print("\n  -- control: ASSUMING identity (see --truth; this is not the "
              "true map->odom for a mapping bag) --")
        print(f"  translation err  mean {trans.mean()*1000:7.1f} mm   "
              f"p95 {np.percentile(trans,95)*1000:7.1f} mm   "
              f"max {trans.max()*1000:7.1f} mm")
        yerr = np.abs(np.vectorize(wrap)(mo[:, 3]))
        print(f"  yaw err          mean {math.degrees(yerr.mean()):7.2f} deg  "
              f"p95 {math.degrees(np.percentile(yerr,95)):7.2f} deg  "
              f"max {math.degrees(yerr.max()):7.2f} deg")

    if len(amcl):
        sx, sy, st = np.sqrt(amcl[:, 4]), np.sqrt(amcl[:, 5]), np.sqrt(amcl[:, 6])
        print(f"\n  amcl_pose        {len(amcl)} msgs")
        print(f"  converged sigma  x {sx[-1]*1000:6.1f} mm  y {sy[-1]*1000:6.1f} mm  "
              f"yaw {math.degrees(st[-1]):5.2f} deg   (final)")
        print(f"  sigma_x          min {sx.min()*1000:6.1f} mm  "
              f"max {sx.max()*1000:6.1f} mm")

    path = compose(mo, ob)
    # The three runs started from one hand-placed physical pose, so the first
    # map-frame pose is a cross-run consistency check that needs no ground
    # truth: all three should agree here to within the placement accuracy.
    print(f"\n  first map pose   x={path[0,1]:+.3f} y={path[0,2]:+.3f} "
          f"yaw={math.degrees(path[0,3]):+.2f} deg   "
          f"(shared start pose is +0.445, -0.575, -79.82 deg)")
    print(f"  map-frame path   x {path[:,1].min():+6.2f} .. {path[:,1].max():+6.2f}   "
          f"y {path[:,2].min():+6.2f} .. {path[:,2].max():+6.2f} m")
    d = np.hypot(np.diff(path[:, 1]), np.diff(path[:, 2])).sum()
    closure = math.hypot(path[-1, 1] - path[0, 1], path[-1, 2] - path[0, 2])
    print(f"  path length      {d:6.2f} m      start->end {closure:5.2f} m")

    if args.map:
        img, res, origin = load_map(args.map)
        occ = classify(path, img, res, origin)
        print(f"\n  vs {os.path.basename(args.map)}  ({img.shape[1]}x{img.shape[0]} "
              f"@ {res} m, origin {origin[:2]})")
        for k in ("free", "unknown", "occupied", "other", "off_map"):
            print(f"  {k:14s}   {occ[k]*100:6.2f} %")
        if occ["occupied"] + occ["off_map"] > 0.02:
            print("  ** the path spends >2% of its time inside walls or off the map;"
                  " this localization is not trustworthy")


if __name__ == "__main__":
    main()
