#!/usr/bin/env python3
"""Offline verification of imu_processors::ImuBiasRemover on real RealSense IMU.

Drives the node's velocity source synthetically so all three questions can be
answered with the car parked:

  A stationary  -> bias.z must converge to the bag's own stationary z-gyro mean
  B moving      -> imu_biased.z must equal raw.z - bias.z, sample for sample
  C stationary  -> accumulation resumes
  D source dies -> the node has no staleness timeout, so it should stay pinned
                   in the zeroing branch. This measures the hazard, it does not
                   assume it.

The gyro data is real (replayed from a bag). Only the velocity source is
synthetic -- which is faithful, because the node's sole use of odom is a
threshold test on twist.
"""
import csv
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped

# The phase clock starts at the FIRST IMU MESSAGE, not at node start: the bag
# carries ~23 s of lead-in (the recorder starts before the launch and the camera
# needs ~9.5 s to stream), which otherwise swallows the whole A window and
# leaves B subtracting a bias of exactly zero -- a degenerate pass.
# (name, end_time_s, publish_odom, moving)
PHASES = [
    ("A_stationary",   25.0, True,  False),
    ("B_moving",       40.0, True,  True),
    ("C_stationary",   50.0, True,  False),
    ("D_source_dead",  70.0, False, False),   # stops publishing while flag is True
]
WALL_TIMEOUT = 240.0


class Driver(Node):
    def __init__(self, out_csv):
        super().__init__("bias_test_driver")
        best = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        rel = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)

        # Republish the bag's IMU as RELIABLE: the node subscribes with
        # SystemDefaultsQoS (RELIABLE), which a BEST_EFFORT bag publisher
        # cannot satisfy. Subscribing BEST_EFFORT accepts either.
        self.pub_imu = self.create_publisher(Imu, "/test/imu", rel)
        self.pub_odom = self.create_publisher(Odometry, "/test/odom", rel)
        self.create_subscription(Imu, "/camera/imu", self.on_raw, best)
        self.create_subscription(Imu, "/test/imu_biased", self.on_out, rel)
        self.create_subscription(Vector3Stamped, "/test/bias", self.on_bias, rel)

        self.raw = {}         # stamp_ns -> raw z
        self.rows = []
        self.bias_z = 0.0
        self.n_raw = self.n_out = self.n_bias = 0
        self.t_wall0 = self.get_clock().now().nanoseconds / 1e9
        self.t_first = None   # set on the first IMU message
        self.done = False
        self.create_timer(0.02, self.tick_odom)   # 50 Hz, like vesc_odom
        self.out_csv = out_csv

    def wall(self):
        return self.get_clock().now().nanoseconds / 1e9 - self.t_wall0

    def elapsed(self):
        if self.t_first is None:
            return 0.0
        return self.get_clock().now().nanoseconds / 1e9 - self.t_first

    def phase(self):
        t = self.elapsed()
        for name, end, pub, moving in PHASES:
            if t < end:
                return name, pub, moving
        return None, False, False

    def tick_odom(self):
        name, pub, moving = self.phase()
        if name is None or self.wall() > WALL_TIMEOUT:
            self.done = True
            return
        if not pub:
            return                      # phase D: source is dead
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "odom"
        m.twist.twist.linear.x = 1.0 if moving else 0.0
        self.pub_odom.publish(m)

    def on_raw(self, msg):
        if self.t_first is None:
            self.t_first = self.get_clock().now().nanoseconds / 1e9
            print("first IMU at wall %.1f s -- phase clock starts now"
                  % self.wall(), flush=True)
        self.n_raw += 1
        key = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self.raw[key] = msg.angular_velocity.z
        if len(self.raw) > 20000:
            for k in list(self.raw)[:5000]:
                del self.raw[k]
        self.pub_imu.publish(msg)       # forward to the node under test

    def on_bias(self, msg):
        self.n_bias += 1
        self.bias_z = msg.vector.z

    def on_out(self, msg):
        self.n_out += 1
        key = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if key not in self.raw:
            return
        name, _, moving = self.phase()
        self.rows.append((round(self.elapsed(), 4), name or "END", int(moving),
                          self.raw[key], msg.angular_velocity.z, self.bias_z))

    def write(self):
        with open(self.out_csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "phase", "moving", "raw_z", "out_z", "bias_z"])
            w.writerows(self.rows)
        print("raw=%d out=%d bias=%d rows=%d -> %s"
              % (self.n_raw, self.n_out, self.n_bias, len(self.rows), self.out_csv))


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "/mnt/shared_dir/bias_test.csv"
    rclpy.init()
    n = Driver(out)
    try:
        while rclpy.ok() and not n.done:
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    n.write()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
