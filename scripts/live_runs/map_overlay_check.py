#!/usr/bin/env python3
"""Which cloud and which grid is the running stack actually publishing?

Answers the RViz "the 2D and 3D maps look rotated relative to each other" question from
the wire instead of from a screenshot. Subscribes to the live /map and /map/pointcloud,
reports each one's extent, and matches the cloud against the candidate .pcd files on disk
by point count and bounding box -- so a stale cloud paired with a current grid is named,
not inferred. Also reports map->base_link for the parking-spot check.

Both topics are latched (TRANSIENT_LOCAL), so this works any time after startup.
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

# One candidate on purpose. The Jan-2024 'data/maps/rtabmap/raslab/cloud.pcd'
# used to sit here as a fallback: it is a DIFFERENT map in a DIFFERENT frame, so
# falling back to it silently compares the live grid against an unrelated cloud
# and every geometric check still "passes". The raslab grid was deleted in
# bug-237; that .pcd survives only as an untracked leftover. If the file below is
# missing, fail loudly instead.
#
# Note this cloud is the bug-265-corrected one (grid frame). Any cloud exported
# straight out of rtabmap-export is in the run's START frame and is yawed against
# the grid -- run scripts/analysis/cloud_to_grid_frame.py first.
CANDIDATES = [
    'data/maps/20260805/cloud_voxel_0p05.pcd',
]


class Check(Node):
    def __init__(self, ns):
        super().__init__('map_overlay_check')
        self.grid = None
        self.cloud = None
        # /map is latched by map_server: transient_local + reliable, and a volatile
        # subscriber would miss the one-shot publish.
        grid_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST)
        grid_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        grid_qos.reliability = ReliabilityPolicy.RELIABLE
        # /map/pointcloud is pcd_to_pointcloud, which publishes VOLATILE and repeats
        # every 3 s. Asking for TRANSIENT_LOCAL here is *incompatible*, so the
        # subscription forms and then delivers nothing -- same trap as bug-261.
        cloud_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST)
        cloud_qos.durability = DurabilityPolicy.VOLATILE
        cloud_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        p = ('/' + ns.strip('/') + '/') if ns.strip('/') else '/'
        self.create_subscription(OccupancyGrid, p + 'map', self._grid, grid_qos)
        self.create_subscription(PointCloud2, p + 'map/pointcloud', self._cloud, cloud_qos)
        self.prefix = p

    def _grid(self, m):
        self.grid = m

    def _cloud(self, m):
        self.cloud = m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ns', default='')
    ap.add_argument('--pkg-share', default='.',
                    help='directory holding data/maps/... for the candidate comparison')
    ap.add_argument('--timeout', type=float, default=30.0)
    args = ap.parse_args()

    rclpy.init()
    node = Check(args.ns)
    end = time.time() + args.timeout
    while rclpy.ok() and time.time() < end and not (node.grid and node.cloud):
        rclpy.spin_once(node, timeout_sec=0.2)

    print('=' * 78)
    if node.grid is None:
        print('%smap            : NO MESSAGE in %.0f s' % (node.prefix, args.timeout))
    else:
        g = node.grid
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        w, h, r = g.info.width, g.info.height, g.info.resolution
        data = np.asarray(g.data, dtype=np.int16).reshape(h, w)
        rows, cols = np.nonzero(data >= 65)
        print('%smap            : %dx%d @ %.3f m, origin (%.3f, %.3f), frame %s'
              % (node.prefix, w, h, r, ox, oy, g.header.frame_id))
        if len(rows):
            print('%s                 %d occupied cells, extent x [%.2f, %.2f]  y [%.2f, %.2f]'
                  % (' ' * len(node.prefix), len(rows),
                     ox + cols.min() * r, ox + cols.max() * r,
                     oy + rows.min() * r, oy + rows.max() * r))

    if node.cloud is None:
        print('%smap/pointcloud : NO MESSAGE in %.0f s  '
              '(launch_pointcloud_map follows launch_visualization -- is RViz up?)'
              % (node.prefix, args.timeout))
    else:
        # read_points returns a *structured* array in Humble, whose elements are
        # 0-d voids -- p[:3] raises IndexError. Take the fields by name instead.
        raw = point_cloud2.read_points(
            node.cloud, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = np.column_stack(
            [np.asarray(raw['x']), np.asarray(raw['y']), np.asarray(raw['z'])]
        ).astype(np.float64)
        print('%smap/pointcloud : %d points, frame %s'
              % (node.prefix, len(pts), node.cloud.header.frame_id))
        print('%s                 extent x [%.2f, %.2f]  y [%.2f, %.2f]  z [%.2f, %.2f]'
              % (' ' * len(node.prefix), pts[:, 0].min(), pts[:, 0].max(),
                 pts[:, 1].min(), pts[:, 1].max(), pts[:, 2].min(), pts[:, 2].max()))
        print('\n  matching against the candidate .pcd files on disk:')
        sys.path.insert(0, os.path.join(args.pkg_share, 'scripts', 'analysis'))
        try:
            from map_io import read_pcd_xyz
        except ImportError:
            print('    (map_io.py not importable from --pkg-share; skipping)')
            return
        for rel in CANDIDATES:
            f = os.path.join(args.pkg_share, rel)
            if not os.path.exists(f):
                print('    %-46s  not present' % rel)
                continue
            c = read_pcd_xyz(f)
            same_n = (len(c) == len(pts))
            dbox = max(abs(c[:, 0].min() - pts[:, 0].min()), abs(c[:, 0].max() - pts[:, 0].max()),
                       abs(c[:, 1].min() - pts[:, 1].min()), abs(c[:, 1].max() - pts[:, 1].max()))
            verdict = 'THIS ONE' if (same_n and dbox < 0.01) else ''
            print('    %-46s  n=%6d  bbox delta %7.3f m  %s' % (rel, len(c), dbox, verdict))
    print('=' * 78)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
