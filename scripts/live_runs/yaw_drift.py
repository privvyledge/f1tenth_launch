#!/usr/bin/env python3
"""Measure stationary yaw drift and z-gyro bias across every fusion input.

    python3 yaw_drift.py [seconds]        # default 60, car PARKED

Reports, per source, the yaw travelled over the window and the implied drift
rate, then the mean z-gyro of each IMU. A gyro with a non-zero stationary mean
walks yaw at exactly that rate, so the two halves of the report should agree
for whichever source is at fault — that is what makes this diagnostic and not
just descriptive.

Why it exists (measured on gosling1, 2026-08-06, parked): the VESC IMU's
quaternion drifted -13.99 deg/min and dragged odometry/local with it at -13.94,
from a stable +0.00434 rad/s z-gyro bias, while Isaac VSLAM managed +0.07.
vehicle/sensors/imu/mag was publishing 100 Hz of identical zeros, so nothing in
the odom frame had an absolute yaw reference. Re-run this after any change to
the imu0/imu1 blocks of config/localization/ekf_odom.yaml.

Namespace comes from $VEHICLE_NAME (default gosling1). Sources that never
publish print '--' rather than being silently dropped, so a topic that is
missing cannot be mistaken for one that is steady.
"""
import math
import os
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

NS = "/" + os.environ.get("VEHICLE_NAME", "gosling1")
DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0

ODOM = [
    ("odometry/local",                "EKF local (odom)"),
    ("odometry/global",               "EKF global (map)"),
    ("vehicle/vesc_odom",             "VESC wheel odom"),
    ("odom/rf2o",                     "rf2o LiDAR odom"),
    ("visual_slam/tracking/odometry", "Isaac VSLAM (VO)"),
]
IMUS = [
    ("vehicle/sensors/imu/raw", "VESC IMU"),
    ("camera/imu/filtered",     "RealSense IMU"),
]


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def unwrap(series):
    """Total signed yaw travelled, immune to +-pi wrapping."""
    total = 0.0
    for (_, a), (_, b) in zip(series, series[1:]):
        d = b - a
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        total += d
    return total


class Probe(Node):
    def __init__(self):
        super().__init__("yaw_drift_probe")
        # Sensor streams are BEST_EFFORT; subscribing RELIABLE to them silently
        # receives nothing (the VESC odom publisher will warn about exactly this
        # if the depths disagree).
        sensor = QoSProfile(depth=50,
                            reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)
        self.yaws = {}
        self.gyro = {}

        for topic, label in ODOM:
            self.yaws[label] = []
            # BEST_EFFORT here too, not a bare depth: vehicle/vesc_odom is
            # published BEST_EFFORT, so a RELIABLE subscriber is an incompatible
            # match and reports n=0 — which reads as "topic silent" rather than
            # "never subscribed". A BEST_EFFORT subscriber accepts both kinds of
            # publisher, so the EKF outputs still arrive.
            self.create_subscription(
                Odometry, f"{NS}/{topic}",
                lambda m, l=label: self.yaws[l].append(
                    (self._t(m), yaw(m.pose.pose.orientation))), sensor)

        self.yaws["AMCL"] = []
        self.create_subscription(
            PoseWithCovarianceStamped, f"{NS}/amcl_pose",
            lambda m: self.yaws["AMCL"].append(
                (self._t(m), yaw(m.pose.pose.orientation))), 10)

        for topic, label in IMUS:
            self.gyro[label] = []
            self.yaws[label + " (orientation)"] = []
            self.create_subscription(
                Imu, f"{NS}/{topic}",
                lambda m, l=label: (
                    self.gyro[l].append(m.angular_velocity.z),
                    self.yaws[l + " (orientation)"].append(
                        (self._t(m), yaw(m.orientation)))), sensor)

    @staticmethod
    def _t(m):
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def main():
    rclpy.init()
    node = Probe()
    node.get_logger().info(
        f"sampling {DURATION:.0f}s on {NS} — keep the car still")
    end = node.get_clock().now().nanoseconds * 1e-9 + DURATION
    while rclpy.ok() and node.get_clock().now().nanoseconds * 1e-9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)

    print("\n=== stationary yaw drift ===")
    print(f"{'source':34s} {'n':>6s} {'span_s':>8s} {'drift_deg':>10s} {'deg/min':>9s}")
    for label, s in node.yaws.items():
        if len(s) < 2:
            print(f"{label:34s} {len(s):6d}       --         --        --")
            continue
        span = s[-1][0] - s[0][0]
        d = math.degrees(unwrap(s))
        rate = d / (span / 60.0) if span > 0 else float("nan")
        print(f"{label:34s} {len(s):6d} {span:8.1f} {d:10.2f} {rate:9.2f}")

    print("\n=== z-gyro bias while stationary ===")
    print(f"{'imu':34s} {'n':>6s} {'mean_rad_s':>11s} {'std':>9s} {'deg/min':>9s}")
    for label, g in node.gyro.items():
        if not g:
            print(f"{label:34s} {len(g):6d}          --        --        --")
            continue
        mean = sum(g) / len(g)
        var = sum((x - mean) ** 2 for x in g) / len(g)
        print(f"{label:34s} {len(g):6d} {mean:11.6f} {var ** 0.5:9.5f} "
              f"{math.degrees(mean) * 60:9.2f}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
