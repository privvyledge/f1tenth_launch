#!/usr/bin/env python3
"""heading_from_scan.py — measure the map-frame heading of a parked car from
LiDAR geometry alone, independent of any localizer seed.

WHY THIS EXISTS
---------------
Three different yaw values claimed the same physical parking spot:

    -79.80 deg   the AMCL initial_pose seed, until 2026-08-11
    -86.5  deg   live AMCL settle, measured 2026-08-06
    -92.08 deg   waypoint 0 of the figure-8 route CSV

They cannot settle each other, because all three are *estimator outputs* and so
inherit the same possible bias. This script does not ask any estimator. It scores
the raw scan against the occupancy grid and reports the heading that best
explains the walls — a geometric measurement, not a filter state.

SETTLED. Five samples over two cold launches (2026-08-10, and three recordings on
2026-08-11 with the car rolled off the spot and back on between them) put the spot
at -84.5 deg, total spread 1.03 deg, on a single unambiguous peak.
config/localization/localizer_amcl.yaml initial_pose.yaw was changed -1.3928 ->
-1.4748 on 2026-08-11. The candidates below are kept as-is on purpose: they are
the historical claims this script exists to score, and re-runs should keep showing
how each one fares. See DEMO_RUNBOOK_20260810.md section 5b.

USAGE
-----
    # 1. park on the spot, record ~10 s of scan (the `mpc` topic set does NOT
    #    include scan, so existing drive bags will not work):
    ros2 bag record -o heading_check /<ns>/lidar/scan_filtered

    # 2. sweep:
    ./heading_from_scan.py --bag heading_check --map <map>.yaml
    ./heading_from_scan.py --live --topic /<ns>/lidar/scan_filtered --map <map>.yaml

    # tighter/wider search, or a different centre:
    ./heading_from_scan.py --bag b --map m.yaml --yaw-center -86 --yaw-range 25 --yaw-step 0.1
    # also search position (recommended: a wrong x/y biases the yaw estimate):
    ./heading_from_scan.py --bag b --map m.yaml --xy-range 0.15 --xy-step 0.03

WHAT IT REPORTS
---------------
The best-scoring (x, y, yaw), the score profile around it, and an explicit
side-by-side score for the three candidate headings above, so the comparison
that motivated the script is answered directly.

READ THE CAVEATS AT THE BOTTOM OF THE OUTPUT. A rectangular room can be
genuinely ambiguous to a planar LiDAR; this script tells you when that is the
case instead of hiding it behind a single number.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

# Headings under dispute, in degrees. Reported explicitly in the summary.
CANDIDATES = {
    "old config seed (pre 08-11)": -79.80,
    "config initial_pose": -84.50,
    "AMCL settle 2026-08-06": -86.50,
    "figure-8 waypoint 0": -92.08,
}


# --------------------------------------------------------------------- map ---
def read_pgm(path: str) -> np.ndarray:
    """Minimal binary (P5) and ASCII (P2) PGM reader. Avoids a PIL/cv2 dep."""
    with open(path, "rb") as f:
        data = f.read()

    # Header tokens are whitespace separated, with '#' comments to end of line.
    tokens, i = [], 0
    while len(tokens) < 4:
        while i < len(data) and data[i : i + 1].isspace():
            i += 1
        if data[i : i + 1] == b"#":
            while i < len(data) and data[i] != 0x0A:
                i += 1
            continue
        j = i
        while j < len(data) and not data[j : j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j

    magic, width, height, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    i += 1  # single whitespace byte after maxval

    if magic == b"P5":
        dtype = np.uint8 if maxval < 256 else ">u2"
        img = np.frombuffer(data, dtype=dtype, count=width * height, offset=i)
    elif magic == b"P2":
        img = np.array(data[i:].split()[: width * height], dtype=np.int64)
    else:
        raise ValueError(f"{path}: unsupported PGM magic {magic!r} (want P5 or P2)")

    return img.reshape(height, width).astype(np.float64) / float(maxval)


class OccupancyMap:
    """ROS map_server occupancy grid, with a likelihood field for scoring."""

    def __init__(self, yaml_path: str, sigma_hit: float):
        import yaml

        with open(yaml_path) as f:
            meta = yaml.safe_load(f)

        img_path = meta["image"]
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)

        norm = read_pgm(img_path)  # 0.0 (black) .. 1.0 (white)
        self.resolution = float(meta["resolution"])
        self.origin = [float(v) for v in meta["origin"]]
        occupied_thresh = float(meta.get("occupied_thresh", 0.65))
        negate = int(meta.get("negate", 0))

        # map_server: p_occ = 1 - normalised_value, inverted when negate=1.
        p_occ = norm if negate else (1.0 - norm)
        self.occupied = p_occ > occupied_thresh
        self.height, self.width = self.occupied.shape

        if not self.occupied.any():
            raise SystemExit(
                f"no occupied cells in {img_path} above occupied_thresh="
                f"{occupied_thresh} (negate={negate}) — wrong map or threshold"
            )

        self.likelihood, self.exact = self._build_likelihood(sigma_hit)

    def _build_likelihood(self, sigma_hit: float):
        """Gaussian likelihood of distance-to-nearest-obstacle.

        Exact when SciPy is available; otherwise a 1-cell-dilation fallback that
        is coarser but needs no extra dependency. Which one ran is reported.
        """
        try:
            from scipy.ndimage import distance_transform_edt
        except ImportError:
            occ = self.occupied
            near = occ.copy()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                near |= np.roll(occ, (dr, dc), axis=(0, 1))
            lik = np.where(occ, 1.0, np.where(near, 0.5, 0.0))
            return lik, False

        dist_cells = distance_transform_edt(~self.occupied)
        dist_m = dist_cells * self.resolution
        return np.exp(-(dist_m**2) / (2.0 * sigma_hit**2)), True

    def score_points(self, xs: np.ndarray, ys: np.ndarray) -> float:
        """Mean likelihood of world-frame points. Off-map points score 0."""
        cols = np.floor((xs - self.origin[0]) / self.resolution).astype(np.int64)
        rows = self.height - 1 - np.floor((ys - self.origin[1]) / self.resolution).astype(np.int64)
        inside = (rows >= 0) & (rows < self.height) & (cols >= 0) & (cols < self.width)
        if not inside.any():
            return 0.0
        total = self.likelihood[rows[inside], cols[inside]].sum()
        return float(total / xs.size)  # off-map endpoints count as misses


# -------------------------------------------------------------------- scan ---
def scan_to_local(msg, range_min: float, range_max: float, stride: int):
    """LaserScan -> (N,2) endpoints in the lidar frame, invalid returns dropped."""
    ranges = np.asarray(msg.ranges, dtype=np.float64)
    angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment

    lo = max(range_min, float(msg.range_min))
    hi = min(range_max, float(msg.range_max))
    keep = np.isfinite(ranges) & (ranges > lo) & (ranges < hi)
    ranges, angles = ranges[keep][::stride], angles[keep][::stride]

    return np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))


def scans_from_bag(bag: str, topic: str | None, limit: int):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    scan_topics = [n for n, ty in types.items() if ty == "sensor_msgs/msg/LaserScan"]
    if topic is None:
        if not scan_topics:
            raise SystemExit(f"{bag}: no sensor_msgs/msg/LaserScan topic found")
        if len(scan_topics) > 1:
            raise SystemExit(f"{bag}: several scan topics {scan_topics}; pass --topic")
        topic = scan_topics[0]
    elif topic not in types:
        raise SystemExit(f"{bag}: topic {topic} not present (have {scan_topics})")

    out = []
    while reader.has_next() and len(out) < limit:
        name, payload, _ = reader.read_next()
        if name == topic:
            out.append(deserialize_message(payload, LaserScan))
    if not out:
        raise SystemExit(f"{bag}: no messages on {topic}")
    print(f"read {len(out)} scan(s) from {topic}")
    return out


def scans_live(topic: str, limit: int, timeout: float):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = Node("heading_from_scan")
    out: list = []
    qos = QoSProfile(depth=10)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT  # sensor data
    node.create_subscription(LaserScan, topic, out.append, qos)

    import time

    start = time.time()
    while len(out) < limit and time.time() - start < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    if not out:
        raise SystemExit(f"no LaserScan on {topic} within {timeout:.0f}s")
    print(f"captured {len(out)} scan(s) from {topic}")
    return out


# ------------------------------------------------------------------ search ---
def score_pose(occ_map, pts_local, x, y, yaw_deg, lidar_dx, lidar_dy, lidar_yaw_deg):
    """Score base_link pose (x, y, yaw) by projecting the scan into the map."""
    yaw = math.radians(yaw_deg)
    # lidar origin in map = base_link + R(yaw) * (dx, dy)
    lx = x + lidar_dx * math.cos(yaw) - lidar_dy * math.sin(yaw)
    ly = y + lidar_dx * math.sin(yaw) + lidar_dy * math.cos(yaw)

    total = yaw + math.radians(lidar_yaw_deg)
    c, s = math.cos(total), math.sin(total)
    xs = lx + pts_local[:, 0] * c - pts_local[:, 1] * s
    ys = ly + pts_local[:, 0] * s + pts_local[:, 1] * c
    return occ_map.score_points(xs, ys)


def main() -> int:
    p = argparse.ArgumentParser(
        description="Measure map-frame heading from scan-to-map geometry.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--bag", help="rosbag2 directory containing a LaserScan topic")
    src.add_argument("--live", action="store_true", help="subscribe instead of reading a bag")
    p.add_argument("--map", required=True, help="map yaml (map_server format)")
    p.add_argument("--topic", default=None, help="scan topic (required for --live)")
    p.add_argument("--scans", type=int, default=20, help="scans to average (default 20)")
    p.add_argument("--live-timeout", type=float, default=20.0)

    p.add_argument("--x", type=float, default=0.445, help="prior base_link x (default 0.445)")
    p.add_argument("--y", type=float, default=-0.575, help="prior base_link y (default -0.575)")
    p.add_argument("--yaw-center", type=float, default=-86.0,
                   help="centre of the yaw sweep in deg (default -86, between the candidates)")
    p.add_argument("--yaw-range", type=float, default=20.0, help="+/- deg (default 20)")
    p.add_argument("--yaw-step", type=float, default=0.25, help="deg (default 0.25)")
    p.add_argument("--xy-range", type=float, default=0.0,
                   help="+/- m position search (default 0 = yaw only)")
    p.add_argument("--xy-step", type=float, default=0.03)

    p.add_argument("--lidar-dx", type=float, default=0.084,
                   help="base_link->lidar x in m (default 0.084, measured on this vehicle)")
    p.add_argument("--lidar-dy", type=float, default=0.0)
    p.add_argument("--lidar-yaw", type=float, default=0.0, help="base_link->lidar yaw in deg")

    p.add_argument("--sigma-hit", type=float, default=0.10, help="likelihood field std in m")
    p.add_argument("--range-min", type=float, default=0.15)
    p.add_argument("--range-max", type=float, default=10.0, help="sensor spec max (default 10.0)")
    p.add_argument("--stride", type=int, default=1, help="use every Nth beam")
    args = p.parse_args()

    if args.live and not args.topic:
        p.error("--live requires --topic")

    occ_map = OccupancyMap(args.map, args.sigma_hit)
    print(f"map: {args.map}  {occ_map.width}x{occ_map.height} @ {occ_map.resolution} m/px  "
          f"occupied={int(occ_map.occupied.sum())} cells  "
          f"likelihood={'exact (scipy)' if occ_map.exact else 'coarse (no scipy)'}")

    msgs = (scans_live(args.topic, args.scans, args.live_timeout) if args.live
            else scans_from_bag(args.bag, args.topic, args.scans))
    pts = np.vstack([scan_to_local(m, args.range_min, args.range_max, args.stride) for m in msgs])
    print(f"{pts.shape[0]} endpoints after filtering\n")

    yaws = np.arange(args.yaw_center - args.yaw_range,
                     args.yaw_center + args.yaw_range + 1e-9, args.yaw_step)
    if args.xy_range > 0:
        offs = np.arange(-args.xy_range, args.xy_range + 1e-9, args.xy_step)
    else:
        offs = np.array([0.0])

    best = (-1.0, None)
    yaw_profile = np.zeros(yaws.size)
    for iy, yaw in enumerate(yaws):
        local_best = -1.0
        for dx in offs:
            for dy in offs:
                s = score_pose(occ_map, pts, args.x + dx, args.y + dy, yaw,
                               args.lidar_dx, args.lidar_dy, args.lidar_yaw)
                local_best = max(local_best, s)
                if s > best[0]:
                    best = (s, (args.x + dx, args.y + dy, yaw))
        yaw_profile[iy] = local_best

    bs, (bx, by, byaw) = best
    print(f"BEST  x={bx:+.3f}  y={by:+.3f}  yaw={byaw:+.2f} deg   score={bs:.4f}")
    print(f"      (searched {yaws.size} yaws x {offs.size**2} positions)\n")

    print("candidate headings, scored at the best position:")
    rows = []
    for name, yaw_deg in CANDIDATES.items():
        s = score_pose(occ_map, pts, bx, by, yaw_deg,
                       args.lidar_dx, args.lidar_dy, args.lidar_yaw)
        rows.append((s, name, yaw_deg))
    for s, name, yaw_deg in sorted(rows, reverse=True):
        rel = 100.0 * s / bs if bs > 0 else 0.0
        print(f"  {name:26s} {yaw_deg:+7.2f} deg   score={s:.4f}  ({rel:5.1f}% of best, "
              f"{abs(yaw_deg - byaw):4.2f} deg from best)")

    # Secondary maxima are the ambiguity test: a rectangular room can fit a wall
    # set at several headings, and that is the thing worth knowing.
    print("\nyaw profile (peak-normalised):")
    peak = yaw_profile.max()
    for iy in range(0, yaws.size, max(1, yaws.size // 40)):
        bar = "#" * int(round(40 * yaw_profile[iy] / peak)) if peak > 0 else ""
        print(f"  {yaws[iy]:+7.2f}  {yaw_profile[iy]/peak:5.3f} {bar}")

    strong = []
    for iy in range(1, yaws.size - 1):
        if (yaw_profile[iy] >= yaw_profile[iy - 1]
                and yaw_profile[iy] >= yaw_profile[iy + 1]
                and yaw_profile[iy] > 0.9 * peak):
            strong.append(yaws[iy])
    merged: list[float] = []
    for y in strong:
        if not merged or abs(y - merged[-1]) > 2.0:
            merged.append(y)

    print("\n--- how to read this ---")
    if len(merged) > 1:
        print(f"AMBIGUOUS: {len(merged)} peaks within 10% of the best, at "
              f"{', '.join(f'{y:+.2f}' for y in merged)} deg.")
        print("A planar LiDAR cannot separate these from this spot, so scan matching")
        print("alone cannot pick the true heading here — and any localizer will be")
        print("equally free to latch onto the wrong one. That is a finding, not a")
        print("failed measurement: it means the recurring ~90 deg heading errors are")
        print("explained by map/room symmetry rather than by a frame-convention bug.")
    else:
        print("Single clear peak: the heading above is the one the walls support.")
        print("Compare it with the candidate table. A candidate scoring well below")
        print("the best is not merely a different convention — it does not fit the map.")
    print("Caveats: this assumes the prior x/y is close (use --xy-range to relax it),")
    print("that the car sat still while recording, and that the map matches the room")
    print("as it is today. Furniture moved since the map was built shows up as noise.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
