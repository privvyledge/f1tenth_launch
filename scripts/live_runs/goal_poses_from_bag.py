#!/usr/bin/env python3
"""Pick Nav2 goals from map-frame poses the car actually occupied.

    python3 goal_poses_from_bag.py <localization_or_mapframe_bag> \
        --pick 3 --out-csv goals.csv

A goal has to be somewhere the planner can reach through free space. Poses from
a real drive are that by construction, which keeps a failed plan attributable
to Nav2 instead of to a goal chosen inside a wall.

DO NOT USE maps/*/truth_<bag>.csv FOR THIS. It is the map->odom TRANSFORM over
time, not a robot pose. On mapping_drive_170025 it stays inside a 0.3 m ball
around the origin, so every "goal" drawn from it lands on top of the robot and
Nav2 reports success without planning anything.

Two input shapes are accepted:
  - a derived map-frame bag (make_map_frame_bag.py) carrying <ns>/pose_map
  - a localization bag from 51_localize_offline.sh / 61_nav2_offline.sh, from
    which map->base_link is composed exactly as check_map_frame.py does it
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'analysis'))
from check_map_frame import collect, compose, read_bag, yaw_of  # noqa: E402


def poses_from_pose_map(bag, ns):
    """<ns>/pose_map is nav_msgs/Odometry already in the map frame."""
    topic = f'/{ns}/pose_map'
    out = []
    try:
        for _, msg, _ in read_bag(bag, {topic}):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            out.append((stamp, msg.pose.pose.position.x, msg.pose.pose.position.y,
                        yaw_of(msg.pose.pose.orientation)))
    except Exception:
        return None
    return np.array(out) if out else None


def pick(path, n, min_sep, skip):
    """n poses, spread along the run, each at least min_sep from the previous
    pick and from the start. A drive that returns to its start (all three of
    these bags do) otherwise yields three goals on top of each other."""
    t0 = path[0, 0]
    cand = path[path[:, 0] - t0 >= skip]
    if len(cand) == 0:
        cand = path
    start = path[0, 1:3]

    chosen = []
    for row in cand:
        if np.hypot(*(row[1:3] - start)) < min_sep:
            continue
        if any(np.hypot(*(row[1:3] - c[1:3])) < min_sep for c in chosen):
            continue
        chosen.append(row)

    if len(chosen) <= n:
        return chosen
    # Spread the picks over the run rather than taking the first n, which
    # would cluster them in the first few seconds of driving.
    idx = np.linspace(0, len(chosen) - 1, n).round().astype(int)
    return [chosen[i] for i in idx]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--ns', default=os.environ.get('NS', 'gosling1'))
    ap.add_argument('--pick', type=int, default=3)
    ap.add_argument('--min-sep', type=float, default=1.5,
                    help='metres between goals and from the start pose')
    ap.add_argument('--skip', type=float, default=10.0,
                    help='ignore the first N seconds (localizer still settling)')
    ap.add_argument('--out-csv', default='')
    args = ap.parse_args()

    path = poses_from_pose_map(args.bag, args.ns)
    if path is None:
        mo, ob, _ = collect(args.bag, args.ns)
        if len(mo) == 0 or len(ob) == 0:
            raise SystemExit(f'{args.bag}: no pose_map, and no map->odom + '
                             f'odom->base_link to compose one from')
        path = compose(mo, ob)

    goals = pick(path, args.pick, args.min_sep, args.skip)
    if not goals:
        raise SystemExit(f'no pose in {args.bag} is {args.min_sep} m from the '
                         f'start — lower --min-sep')

    lines = ['stamp,x,y,yaw']
    for g in goals:
        lines.append(f'{g[0]:.9f},{g[1]:.6f},{g[2]:.6f},{g[3]:.6f}')
    text = '\n'.join(lines)

    if args.out_csv:
        with open(args.out_csv, 'w') as fh:
            fh.write(text + '\n')
    print(text)
    print(f'\n# {len(goals)} goals from {len(path)} poses spanning '
          f'{path[-1, 0] - path[0, 0]:.1f} s', file=sys.stderr)
    for g in goals:
        print(f'#   ({g[1]:+.2f}, {g[2]:+.2f}) '
              f'{math.hypot(g[1] - path[0, 1], g[2] - path[0, 2]):.2f} m from start',
          file=sys.stderr)


if __name__ == '__main__':
    main()
