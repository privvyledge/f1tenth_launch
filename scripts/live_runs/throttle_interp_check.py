#!/usr/bin/env python3
"""Does the throttle interpolator pass steering through?

Records the four points of the steering command chain while the operator sweeps the
stick, then reports whether each stage actually moved. Self-synchronizing: the sweep is
detected from the data, so nothing has to be timed against a clock.

  ackermann_drive               mux output           (AckermannDriveStamped)
  vehicle/ackermann_cmd         command_gate output  (AckermannDriveStamped)
  .../commands/servo/unsmoothed_position   ackermann_to_vesc output = interpolator IN
  .../commands/servo/position              interpolator OUT -> VESC

A stage whose range collapses to ~0 while the stage above it moved is where steering is lost.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64

MOVED = 0.02  # servo units / rad -- below this a stage is "flat"


class Rec(Node):
    def __init__(self, ns, duration):
        super().__init__('throttle_interp_check')
        self.duration = duration
        self.t0 = time.time()
        self.series = {}
        qos = QoSProfile(depth=200)
        qos.reliability = ReliabilityPolicy.RELIABLE

        def ack(topic, label):
            self.series[label] = []
            self.create_subscription(
                AckermannDriveStamped, topic,
                lambda m, l=label: self.series[l].append(
                    (time.time() - self.t0, m.drive.steering_angle)), qos)

        def f64(topic, label):
            self.series[label] = []
            self.create_subscription(
                Float64, topic,
                lambda m, l=label: self.series[l].append(
                    (time.time() - self.t0, m.data)), qos)

        p = ('/' + ns.strip('/') + '/') if ns.strip('/') else '/'
        ack(p + 'ackermann_drive', 'mux ackermann_drive [rad]')
        ack(p + 'vehicle/ackermann_cmd', 'gate vehicle/ackermann_cmd [rad]')
        f64(p + 'vehicle/commands/servo/unsmoothed_position', 'interp IN  unsmoothed_position')
        f64(p + 'vehicle/commands/servo/position', 'interp OUT servo/position')


def report(series):
    print('\n%-42s %6s %9s %9s %9s %9s' % ('stage', 'n', 'min', 'max', 'range', 'verdict'))
    print('-' * 92)
    ranges = {}
    for label, pts in series.items():
        if not pts:
            print('%-42s %6d %9s %9s %9s %9s' % (label, 0, '-', '-', '-', 'SILENT'))
            ranges[label] = None
            continue
        vals = [v for _, v in pts]
        rng = max(vals) - min(vals)
        ranges[label] = rng
        print('%-42s %6d %9.4f %9.4f %9.4f %9s'
              % (label, len(pts), min(vals), max(vals), rng,
                 'moved' if rng > MOVED else 'FLAT'))
    return ranges


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ns', default='gosling1')
    ap.add_argument('--duration', type=float, default=30.0)
    args = ap.parse_args()

    rclpy.init()
    node = Rec(args.ns, args.duration)
    print('Recording %.0f s on namespace "%s".' % (args.duration, args.ns))
    print('>>> HOLD THE DEADMAN AND SWEEP THE STEERING STICK FULL LEFT <-> FULL RIGHT '
          'A FEW TIMES NOW <<<', flush=True)
    end = time.time() + args.duration
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    ranges = report(node.series)

    labels = list(node.series.keys())
    print()
    upstream_moved = None
    for label in labels:
        r = ranges[label]
        if r is None:
            print('LOST AT: %s is silent (no messages at all).' % label)
            break
        if r > MOVED:
            upstream_moved = label
        elif upstream_moved is not None:
            print('LOST AT: "%s" is flat while "%s" moved.' % (label, upstream_moved))
            break
    else:
        if all((ranges[l] or 0) > MOVED for l in labels):
            print('PASS: steering moves at every stage, including the interpolator output.')
        elif upstream_moved is None:
            print('INCONCLUSIVE: nothing moved anywhere -- was the deadman held and the '
                  'stick swept? Re-run.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
