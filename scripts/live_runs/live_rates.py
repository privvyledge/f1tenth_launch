#!/usr/bin/env python3
"""Measure achieved topic rates on a running stack, without perturbing them.

Why this exists rather than `ros2 topic hz`: subscribing to an image stream
costs real bandwidth. Under composition the in-stack consumers are intra-process
and pay nothing, but an external subscriber forces serialisation onto every
frame -- and starving the camera's USB thread by observing it is exactly the
failure this platform already hit once when a recorder pulled the image topics.

So image streams are measured through their paired `camera_info` instead. The
RealSense driver publishes one `camera_info` per frame on the same thread, so
the rate is identical while the bytes are negligible. That 1:1 is checkable in
any recording that carries both -- e.g. colour 5875 images / 5875 infos and
depth 5939 / 5939 in the 2026-09-01 drive bag.

Counts use message *arrival* time at this node, matching bag_stats.py's use of
the rosbag2 receive stamp, so live and recorded numbers are comparable.

Usage:
    python3 live_rates.py [--duration 30] [--ns /gosling1] [--json out.json]
    python3 live_rates.py --stress-images      # add image load, keep the light meter
    python3 live_rates.py --direct-images      # meter the images themselves (costly)

`--stress-images` is the controlled version of the starvation experiment: it
opens the heavy image subscriptions as *load* while still measuring through
camera_info, so a rate that falls is the publisher being starved rather than
this process failing to keep up with its own subscription. Run it against a
plain run to see which streams survive external image consumers.

ONE STREAM BREAKS THAT SYMMETRY, and any chart drawn from this output has to
say so. `camera/depth/color/points` has no camera_info twin, so there is no
light proxy for it: it is metered by subscribing to the cloud itself. It is
therefore its own load, in EVERY run including the baseline -- unlike the five
image streams, which cost nothing unless --stress-images is passed. Two
consequences:

  * the cloud's own rate is not an unperturbed measurement;
  * a "baseline" taken while the cloud is being published is not image-free.

The cloud is listed under load in the report for that reason, and it is not
double-subscribed under --stress-images: the meter is already pulling every
byte, so a second subscription would only add deserialisation cost without
making the experiment cleaner.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message

# Image streams and the camera_info published alongside each, 1:1.
CAMERA_INFO_PROXY = {
    "camera/color/image_raw": "camera/color/camera_info",
    "camera/depth/image_rect_raw": "camera/depth/camera_info",
    "camera/infra1/image_rect_raw": "camera/infra1/camera_info",
    "camera/infra2/image_rect_raw": "camera/infra2/camera_info",
    "camera/aligned_depth_to_color/image_raw":
        "camera/aligned_depth_to_color/camera_info",
}

# Topics with no light proxy: metered directly, hence always self-loading.
# See the docstring -- this asymmetry has to reach any chart built from the
# output, so it is carried through the report and the JSON rather than being
# left as a comment here.
DIRECT_HEAVY = {"camera/depth/color/points"}

# The operational set: what each subsystem is expected to deliver, and the
# configured rate it is quoted at. None means "no single configured target".
DEFAULT_TOPICS: list[tuple[str, float | None]] = [
    ("camera/color/image_raw", 30.0),
    ("camera/depth/image_rect_raw", 30.0),
    ("camera/infra1/image_rect_raw", 30.0),
    ("camera/infra2/image_rect_raw", 30.0),
    ("camera/aligned_depth_to_color/image_raw", 30.0),
    ("camera/depth/color/points", 30.0),
    ("camera/imu", 200.0),
    ("camera/imu/filtered", 200.0),
    ("lidar/scan", 12.0),
    ("lidar/scan_filtered", 12.0),
    ("vehicle/sensors/imu/raw", 100.0),
    ("vehicle/sensors/core", 50.0),
    ("vehicle/vesc_odom", 50.0),
    ("odom/rf2o", None),
    ("visual_slam/tracking/odometry", 30.0),
    ("odometry/local", 30.0),
    ("tf", None),
    ("safety", 40.0),
    ("command_gate/heartbeat", None),
    ("ackermann_drive", None),
    ("vehicle/ackermann_cmd", None),
]


class RateProbe(Node):
    def __init__(self, topics: dict[str, str], duration: float,
                 stress: dict[str, str] | None = None):
        super().__init__("live_rates_probe")
        # Best-effort matches a best-effort publisher and is also compatible
        # with a reliable one, so one profile covers sensor and non-sensor
        # topics alike.
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.stamps: dict[str, list[float]] = {t: [] for t in topics}
        self.subs = []
        for topic, type_str in topics.items():
            try:
                msg_type = get_message(type_str)
            except Exception as exc:  # unknown type: skip, do not abort the run
                self.get_logger().warn(f"skipping {topic}: {exc}")
                continue
            self.subs.append(self.create_subscription(
                msg_type, topic,
                lambda _m, t=topic: self.stamps[t].append(time.monotonic()),
                qos))
        # Load-only subscriptions: their messages are counted but never used,
        # and their rates are reported separately from the meter's.
        self.stress_counts: dict[str, int] = {}
        for topic, type_str in (stress or {}).items():
            try:
                msg_type = get_message(type_str)
            except Exception as exc:
                self.get_logger().warn(f"skipping stress {topic}: {exc}")
                continue
            self.stress_counts[topic] = 0
            self.subs.append(self.create_subscription(
                msg_type, topic,
                lambda _m, t=topic: self.stress_counts.__setitem__(
                    t, self.stress_counts[t] + 1),
                qos))
        self.deadline = time.monotonic() + duration


def summarize(stamps: list[float], window_s: float) -> dict:
    n = len(stamps)
    if n < 2:
        return {"count": n, "hz": 0.0, "gap_p50_ms": 0.0, "gap_max_ms": 0.0}
    gaps = sorted((b - a) * 1e3 for a, b in zip(stamps, stamps[1:]))
    return {
        "count": n,
        # Divide by the observation window, not by the observed span: a stream
        # that dies halfway through must read as half rate, not full rate over
        # a shorter span.
        "hz": n / window_s,
        "gap_p50_ms": gaps[len(gaps) // 2],
        "gap_max_ms": gaps[-1],
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=30.0,
                    help="observation window in seconds (default 30)")
    ap.add_argument("--ns", default="", help="namespace prefix, e.g. /gosling1")
    ap.add_argument("--stress-images", action="store_true",
                    help="also subscribe to every image stream as load, while "
                         "still metering via camera_info. Reproduces the "
                         "bandwidth pressure a recorder applies.")
    ap.add_argument("--direct-images", action="store_true",
                    help="subscribe to image topics directly instead of their "
                         "camera_info proxy. Costs bandwidth; can perturb the "
                         "very rates being measured.")
    ap.add_argument("--skip-topics", default="", metavar="A,B",
                    help="comma-separated logical topics to leave entirely "
                         "unsubscribed. The point is camera/depth/color/points: "
                         "it has no light proxy, so metering it means pulling "
                         "it, and skipping it is the only way to observe a "
                         "stream that is being PUBLISHED but not CONSUMED.")
    ap.add_argument("--json", metavar="PATH", help="also write results as JSON")
    args = ap.parse_args()

    ns = args.ns.rstrip("/")
    rclpy.init()
    probe_ns = Node("live_rates_discovery")
    # Discovery is not instantaneous: querying immediately after the node is
    # created routinely returns an empty graph, which then looks exactly like a
    # wrong namespace or domain. Spin until the graph stops growing.
    available: dict[str, list[str]] = {}
    settle_deadline = time.monotonic() + 15.0
    stable_since = None
    while time.monotonic() < settle_deadline:
        rclpy.spin_once(probe_ns, timeout_sec=0.2)
        now = dict(probe_ns.get_topic_names_and_types())
        if now and len(now) == len(available):
            if stable_since is None:
                stable_since = time.monotonic()
            elif time.monotonic() - stable_since > 1.5:
                break
        else:
            stable_since = None
        available = now

    wanted: dict[str, str] = {}          # resolved topic -> type
    measured_via: dict[str, str] = {}    # logical name -> resolved topic
    targets: dict[str, float | None] = {}
    skip = {t.strip() for t in args.skip_topics.split(",") if t.strip()}
    for logical, target in DEFAULT_TOPICS:
        if logical in skip:
            measured_via[logical] = ""
            targets[logical] = target
            continue
        actual = logical
        if not args.direct_images and logical in CAMERA_INFO_PROXY:
            actual = CAMERA_INFO_PROXY[logical]
        full = f"{ns}/{actual}"
        types = available.get(full)
        if not types:
            measured_via[logical] = ""      # not advertised
            targets[logical] = target
            continue
        wanted[full] = types[0]
        measured_via[logical] = full
        targets[logical] = target
    probe_ns.destroy_node()

    if not wanted:
        print(f"error: none of the expected topics are advertised under "
              f"{ns or '/'} after discovery settled ({len(available)} topics "
              f"visible in total) — wrong namespace or ROS_DOMAIN_ID?",
              file=sys.stderr)
        rclpy.shutdown()
        return 2

    stress: dict[str, str] = {}
    if args.stress_images:
        for image_topic in CAMERA_INFO_PROXY:
            full = f"{ns}/{image_topic}"
            if full in available:
                stress[full] = available[full][0]
        if not stress:
            print("warning: --stress-images found no image topics to load",
                  file=sys.stderr)

    # Streams that are load whether or not --stress-images was passed, because
    # they are metered directly. Reported so the load picture is complete.
    self_loading = [logical for logical in DIRECT_HEAVY
                    if measured_via.get(logical)]

    node = RateProbe(wanted, args.duration, stress)
    started = time.monotonic()
    while rclpy.ok() and time.monotonic() < node.deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    window = time.monotonic() - started

    results = {}
    for logical, full in measured_via.items():
        if not full:
            results[logical] = {"count": 0, "hz": 0.0, "gap_p50_ms": 0.0,
                                "gap_max_ms": 0.0, "advertised": False,
                                "measured_via": None,
                                "configured_hz": targets[logical]}
            continue
        r = summarize(node.stamps.get(full, []), window)
        r["advertised"] = True
        r["measured_via"] = full
        r["configured_hz"] = targets[logical]
        results[logical] = r

    node.destroy_node()
    rclpy.shutdown()

    mode = "images metered directly" if args.direct_images \
        else "image rates via camera_info"
    if args.stress_images:
        mode += f"; {len(node.stress_counts)} image streams subscribed as LOAD"
    print(f"\nobserved {window:.1f} s  ({mode})\n")
    header = (f"{'topic':<44}{'cfg':>7}{'meas':>9}{'% of cfg':>10}"
              f"{'p50 ms':>9}{'max ms':>9}")
    print(header)
    print("-" * len(header))
    for logical in [t for t, _ in DEFAULT_TOPICS]:
        r = results[logical]
        cfg = r["configured_hz"]
        if not r["advertised"]:
            why = "skipped" if logical in skip else "not advertised"
            print(f"{logical:<44}{'-' if cfg is None else f'{cfg:.0f}':>7}"
                  f"{why:>15}")
            continue
        pct = f"{100 * r['hz'] / cfg:.0f}%" if cfg else "-"
        print(f"{logical:<44}{'-' if cfg is None else f'{cfg:.0f}':>7}"
              f"{r['hz']:>9.2f}{pct:>10}"
              f"{r['gap_p50_ms']:>9.1f}{r['gap_max_ms']:>9.1f}")

    if node.stress_counts or self_loading:
        print("\nload subscriptions (bytes actually pulled):")
        for t in sorted(node.stress_counts):
            print(f"  {t:<52}{node.stress_counts[t] / window:>8.2f} Hz")
        for logical in sorted(self_loading):
            print(f"  {measured_via[logical]:<52}"
                  f"{results[logical]['hz']:>8.2f} Hz   "
                  f"(metered directly -- load in every run, baseline included)")

    if args.json:
        with open(args.json, "w") as fh:
            json.dump({"window_s": window,
                       "stress_images": args.stress_images,
                       # Metered directly, so load even in a baseline run.
                       "self_loading_topics": sorted(self_loading),
                       "skipped_topics": sorted(skip),
                       "stress_hz": {t: c / window
                                     for t, c in node.stress_counts.items()},
                       "direct_images": args.direct_images,
                       "namespace": ns,
                       "topics": results}, fh, indent=2)
        print(f"\nwrote {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
