#!/usr/bin/env python3
"""Interactive servo stepper for the Stage 0 bench sweep.

Publishes a held value on ``<ns>/vehicle/commands/servo/position`` so the
operator can walk the steering linkage across its travel and measure each front
wheel independently.  It talks to ``vesc_driver`` directly and therefore
bypasses ``ackermann_to_vesc`` entirely: the gain and offset in ``vesc.yaml``
have no effect here.  What *does* still apply is the driver's own
``servo_min``/``servo_max`` clamp, so run ``vesc_driver_node`` with those
widened if the sweep needs to explore past them.

Nothing else may be publishing on the topic while this runs -- with the full
stack up, ``ackermann_mux``'s always-on ``safety`` channel keeps
``ackermann_to_vesc`` emitting the centre value at 40 Hz and the two publishers
fight.  Bring the stack down first.

The motor is never commanded; only the servo channel is written.

Usage:
    ros2 run ...  # no -- this is a plain script
    python3 bench_servo_sweep.py --ns /gosling1 --start 0.56 --step 0.02

Keys (press enter after each):
    ]  or  d     step up by --step
    [  or  a     step down by --step
    <number>     jump to an absolute servo value
    c            return to --start (centre)
    q            return to centre, then quit
"""

import argparse
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class ServoSweeper(Node):
    def __init__(self, topic, value, rate):
        super().__init__('bench_servo_sweep')
        self._pub = self.create_publisher(Float64, topic, 10)
        self._value = value
        self._lock = threading.Lock()
        self.create_timer(1.0 / rate, self._tick)

    def _tick(self):
        with self._lock:
            self._pub.publish(Float64(data=self._value))

    def set(self, value):
        with self._lock:
            self._value = value


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ns', default='/gosling1',
                    help='robot namespace (default: /gosling1)')
    ap.add_argument('--topic', default=None,
                    help='override the full servo topic name')
    ap.add_argument('--start', type=float, default=0.56,
                    help='starting/centre servo value (default: 0.56)')
    ap.add_argument('--step', type=float, default=0.02,
                    help='step size (default: 0.02)')
    ap.add_argument('--min', dest='vmin', type=float, default=0.04,
                    help='script-side lower guard (default: 0.04)')
    ap.add_argument('--max', dest='vmax', type=float, default=0.96,
                    help='script-side upper guard (default: 0.96)')
    ap.add_argument('--rate', type=float, default=20.0,
                    help='publish rate in Hz (default: 20)')
    args = ap.parse_args()

    topic = args.topic or f'{args.ns.rstrip("/")}/vehicle/commands/servo/position'

    rclpy.init()
    node = ServoSweeper(topic, args.start, args.rate)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    value = args.start
    print(f'publishing on {topic} at {args.rate:g} Hz')
    print(f'guards [{args.vmin}, {args.vmax}]  step {args.step}  centre {args.start}')
    print('keys: ] / d = up, [ / a = down, <number> = jump, c = centre, q = quit')
    print(f'\nservo = {value:.4f}')

    try:
        for line in sys.stdin:
            cmd = line.strip()
            if not cmd:
                continue
            if cmd in ('q', 'quit'):
                break
            elif cmd in (']', 'd'):
                target = value + args.step
            elif cmd in ('[', 'a'):
                target = value - args.step
            elif cmd == 'c':
                target = args.start
            else:
                try:
                    target = float(cmd)
                except ValueError:
                    print(f'  ? unrecognised: {cmd!r}')
                    continue

            if not (args.vmin <= target <= args.vmax):
                print(f'  REFUSED {target:.4f} -- outside guards '
                      f'[{args.vmin}, {args.vmax}]. Raise them deliberately.')
                continue

            value = target
            node.set(value)
            print(f'servo = {value:.4f}')
    except KeyboardInterrupt:
        pass
    finally:
        print(f'\nreturning to centre {args.start:.4f}')
        node.set(args.start)
        # let the timer publish the centre value a few times before teardown
        import time
        time.sleep(0.5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
