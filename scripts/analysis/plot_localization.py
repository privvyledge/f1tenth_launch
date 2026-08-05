#!/usr/bin/env python3
"""Compare every localization estimator over one loop-closure drive.

Produces the "why sensor fusion is necessary" figure: each estimator's
trajectory overlaid, plus its closure error. Because the drive physically
returns to the start pose, true net displacement is ~0 — so whatever an
estimator reports at the end *is* its accumulated drift.

    python3 plot_localization.py <bag> [--map <map.yaml>] [--out <dir>]

Outputs into --out (default docs/figures/localization/):
    trajectories.png    all estimators overlaid, on the map if one is given
    closure_error.png   final position/yaw error per estimator
    yaw_drift.png       heading over time
    summary.md          the numbers, ready to paste into a slide
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    sys.exit("ROS 2 Python packages not importable — source the overlay first")

try:
    import matplotlib
    matplotlib.use("Agg")          # headless: this runs over SSH on the Jetson
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("matplotlib not installed (pip install matplotlib)")


# Topic suffix -> (label, colour). Matched by suffix so the /<ns>/ prefix does
# not have to be known ahead of time.
ESTIMATORS = [
    ("vehicle/vesc_odom",             "VESC wheel odom",   "#8c564b"),
    ("odom/rf2o",                     "rf2o LiDAR odom",   "#ff7f0e"),
    ("visual_slam/tracking/odometry", "Isaac VSLAM (VO)",  "#9467bd"),
    ("visual_slam/vis/slam_odometry", "Isaac VSLAM (SLAM)","#c5b0d5"),
    ("amcl_pose",                     "AMCL",              "#2ca02c"),
    ("odometry/local",                "EKF local (odom)",  "#1f77b4"),
    ("odometry/global",               "EKF global (map)",  "#d62728"),
]


def yaw_from_quat(q) -> float:
    """Yaw from a geometry_msgs Quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def read_tracks(bag: str) -> dict[str, dict]:
    """Extract {label: {t, x, y, yaw}} for every estimator found in the bag."""
    reader = rosbag2_py.SequentialReader()

    storage_id = ""
    path = Path(bag)
    if path.is_dir():
        if list(path.glob("*.mcap")):
            storage_id = "mcap"
        elif list(path.glob("*.db3")):
            storage_id = "sqlite3"

    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )

    typemap = {t.name: t.type for t in reader.get_all_topics_and_types()}

    # Map each bag topic onto an estimator, by suffix.
    wanted: dict[str, tuple[str, str]] = {}
    for topic in typemap:
        for suffix, label, colour in ESTIMATORS:
            if topic.endswith("/" + suffix) or topic == "/" + suffix:
                wanted[topic] = (label, colour)
                break

    if not wanted:
        sys.exit(f"no known localization topics in {bag}\n"
                 f"topics present:\n  " + "\n  ".join(sorted(typemap)))

    tracks = {label: {"t": [], "x": [], "y": [], "yaw": [], "colour": colour}
              for label, colour in wanted.values()}

    msg_classes = {t: get_message(typemap[t]) for t in wanted}

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in wanted:
            continue
        label, _ = wanted[topic]
        msg = deserialize_message(data, msg_classes[topic])

        # Odometry and PoseWithCovarianceStamped nest the pose differently.
        pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose

        tracks[label]["t"].append(t_ns / 1e9)
        tracks[label]["x"].append(pose.position.x)
        tracks[label]["y"].append(pose.position.y)
        tracks[label]["yaw"].append(yaw_from_quat(pose.orientation))

    # Normalise time to the start of the run.
    t0 = min((tr["t"][0] for tr in tracks.values() if tr["t"]), default=0.0)
    for tr in tracks.values():
        tr["t"] = [t - t0 for t in tr["t"]]

    return {k: v for k, v in tracks.items() if v["t"]}


def load_map(map_yaml: str):
    """Return (image_array, extent) for overlaying trajectories, or None."""
    try:
        import yaml
        import numpy as np
        from PIL import Image
    except ImportError:
        print("  (skipping map underlay: needs pyyaml, numpy, pillow)")
        return None

    meta = yaml.safe_load(Path(map_yaml).read_text())
    img_path = Path(map_yaml).parent / meta["image"]
    if not img_path.exists():
        print(f"  (skipping map underlay: {img_path} not found)")
        return None

    img = np.array(Image.open(img_path).convert("L"))
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    h, w = img.shape
    # imshow extent is (left, right, bottom, top) in world coordinates.
    return img, (ox, ox + w * res, oy, oy + h * res)


def plot_trajectories(tracks, map_data, out: Path):
    fig, ax = plt.subplots(figsize=(10, 9))

    if map_data:
        img, extent = map_data
        ax.imshow(img, cmap="gray", extent=extent, origin="lower",
                  alpha=0.55, zorder=0)

    for label, tr in tracks.items():
        ax.plot(tr["x"], tr["y"], label=label, color=tr["colour"],
                linewidth=1.6, zorder=2)
        ax.plot(tr["x"][0], tr["y"][0], "o", color=tr["colour"],
                markersize=7, markeredgecolor="k", markeredgewidth=0.6, zorder=3)
        ax.plot(tr["x"][-1], tr["y"][-1], "X", color=tr["colour"],
                markersize=10, markeredgecolor="k", markeredgewidth=0.6, zorder=3)

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Localization estimators over one loop\n"
                 "circle = start, X = end (true displacement ≈ 0)")
    ax.legend(loc="best", fontsize=9)
    ax.grid(alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")
    fig.tight_layout()
    fig.savefig(out / "trajectories.png", dpi=150)
    plt.close(fig)


def closure_errors(tracks) -> dict[str, tuple[float, float]]:
    """{label: (position_error_m, yaw_error_deg)} between first and last sample."""
    out = {}
    for label, tr in tracks.items():
        dx = tr["x"][-1] - tr["x"][0]
        dy = tr["y"][-1] - tr["y"][0]
        dyaw = math.degrees(
            math.atan2(math.sin(tr["yaw"][-1] - tr["yaw"][0]),
                       math.cos(tr["yaw"][-1] - tr["yaw"][0])))
        out[label] = (math.hypot(dx, dy), dyaw)
    return out


def plot_closure(errors, out: Path):
    labels = list(errors)
    pos = [errors[k][0] for k in labels]
    yaw = [abs(errors[k][1]) for k in labels]

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 5))
    a1.barh(labels, pos, color="#1f77b4")
    a1.set_xlabel("closure error [m]")
    a1.set_title("Position drift over the loop")
    a1.grid(axis="x", alpha=0.3)
    for i, v in enumerate(pos):
        a1.text(v, i, f" {v:.3f}", va="center", fontsize=9)

    a2.barh(labels, yaw, color="#d62728")
    a2.set_xlabel("|yaw error| [deg]")
    a2.set_title("Heading drift over the loop")
    a2.grid(axis="x", alpha=0.3)
    for i, v in enumerate(yaw):
        a2.text(v, i, f" {v:.2f}", va="center", fontsize=9)

    fig.suptitle("Lower is better — the car physically returned to its start pose")
    fig.tight_layout()
    fig.savefig(out / "closure_error.png", dpi=150)
    plt.close(fig)


def plot_yaw(tracks, out: Path):
    fig, ax = plt.subplots(figsize=(11, 5))
    for label, tr in tracks.items():
        ax.plot(tr["t"], [math.degrees(y) for y in tr["yaw"]],
                label=label, color=tr["colour"], linewidth=1.3)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("yaw [deg]")
    ax.set_title("Heading estimate over the run")
    ax.legend(loc="best", fontsize=9)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "yaw_drift.png", dpi=150)
    plt.close(fig)


def write_summary(bag, tracks, errors, out: Path):
    lines = [
        "# Localization comparison",
        "",
        f"Bag: `{bag}`",
        "",
        "The vehicle physically returned to its start pose, so true net",
        "displacement is approximately zero. Each estimator's reported",
        "displacement is therefore its accumulated drift.",
        "",
        "| Estimator | Samples | Rate (Hz) | Position drift (m) | Yaw drift (deg) |",
        "|---|---:|---:|---:|---:|",
    ]
    for label in sorted(errors, key=lambda k: errors[k][0]):
        tr = tracks[label]
        span = tr["t"][-1] - tr["t"][0]
        hz = (len(tr["t"]) - 1) / span if span > 0 else 0.0
        pos_e, yaw_e = errors[label]
        lines.append(f"| {label} | {len(tr['t'])} | {hz:.1f} | "
                     f"{pos_e:.3f} | {yaw_e:+.2f} |")

    best = min(errors, key=lambda k: errors[k][0])
    lines += ["", f"Lowest closure error: **{best}** "
                  f"({errors[best][0]:.3f} m, {errors[best][1]:+.2f}°).", ""]

    if "EKF global (map)" in errors and "VESC wheel odom" in errors:
        ekf = errors["EKF global (map)"][0]
        raw = errors["VESC wheel odom"][0]
        if ekf > 0:
            lines.append(f"Fusion improves on raw wheel odometry by "
                         f"{raw / ekf:.1f}x on this run.")
    (out / "summary.md").write_text("\n".join(lines) + "\n")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--map", help="map.yaml to underlay behind the trajectories")
    ap.add_argument("--out", default="docs/figures/localization",
                    help="output directory (created if missing)")
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    print(f"reading {args.bag} ...")
    tracks = read_tracks(args.bag)
    print(f"found {len(tracks)} estimators:")
    for label, tr in tracks.items():
        print(f"  {label:<22} {len(tr['t']):>6} samples")

    map_data = load_map(args.map) if args.map else None

    errors = closure_errors(tracks)
    plot_trajectories(tracks, map_data, out)
    plot_closure(errors, out)
    plot_yaw(tracks, out)
    write_summary(args.bag, tracks, errors, out)

    print(f"\nclosure error (position, yaw):")
    for label in sorted(errors, key=lambda k: errors[k][0]):
        p, y = errors[label]
        print(f"  {label:<22} {p:>7.3f} m   {y:>+7.2f} deg")

    print(f"\nwrote trajectories.png, closure_error.png, yaw_drift.png, "
          f"summary.md -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
