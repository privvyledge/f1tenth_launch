"""Watch /odometry/local and /particle_cloud together and report whether AMCL's
motion gate opened. Self-synchronising: the push is detected from the odometry,
so the operator does not have to time anything.

Prints a running line, then a summary. Values, not just rates -- a closed gate
and a silent topic look identical to `ros2 topic hz`.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from nav2_msgs.msg import ParticleCloud

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 150.0


def spread(particles):
    """RMS distance of the particles from their own mean, in metres."""
    n = len(particles)
    if n < 2:
        return 0.0
    mx = sum(p.pose.position.x for p in particles) / n
    my = sum(p.pose.position.y for p in particles) / n
    return math.sqrt(sum((p.pose.position.x - mx) ** 2 +
                         (p.pose.position.y - my) ** 2 for p in particles) / n)


class Watch(Node):
    def __init__(self):
        super().__init__('pcloud_watch')
        qos = QoSProfile(depth=20, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.t0 = time.time()
        self.odom_n = 0
        self.path = 0.0
        self.last_xy = None
        self.pc_n = 0
        self.pc_first_t = None
        self.pc_last_t = None
        self.pc_first_spread = None
        self.pc_last_spread = None
        self.pc_count = 0
        self.create_subscription(Odometry, '/odometry/local', self.on_odom, qos)
        self.create_subscription(ParticleCloud, '/particle_cloud', self.on_pc, qos)
        self.create_timer(5.0, self.tick)

    def on_odom(self, msg):
        self.odom_n += 1
        p = msg.pose.pose.position
        if self.last_xy is not None:
            self.path += math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
        self.last_xy = (p.x, p.y)

    def on_pc(self, msg):
        t = time.time() - self.t0
        s = spread(msg.particles)
        self.pc_n += 1
        self.pc_count = len(msg.particles)
        if self.pc_first_t is None:
            self.pc_first_t, self.pc_first_spread = t, s
        self.pc_last_t, self.pc_last_spread = t, s

    def tick(self):
        el = time.time() - self.t0
        print(f'  t={el:6.1f}s  odom path={self.path:6.3f} m  '
              f'/particle_cloud msgs={self.pc_n}', flush=True)
        if el >= DURATION:
            raise SystemExit


def main():
    rclpy.init()
    n = Watch()
    print(f'watching for {DURATION:.0f} s -- push the car whenever you are ready', flush=True)
    try:
        rclpy.spin(n)
    except SystemExit:
        pass
    print('\n=== SUMMARY ===')
    print(f'odometry/local     : {n.odom_n} msgs, path travelled {n.path:.3f} m')
    if n.pc_n == 0:
        print('/particle_cloud    : 0 messages -- motion gate never opened')
    else:
        print(f'/particle_cloud    : {n.pc_n} msgs, {n.pc_count} particles')
        print(f'  first publish at : t={n.pc_first_t:.1f} s')
        print(f'  last publish at  : t={n.pc_last_t:.1f} s')
        print(f'  spread first/last: {n.pc_first_spread:.3f} -> {n.pc_last_spread:.3f} m')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
