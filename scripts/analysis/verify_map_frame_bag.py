#!/usr/bin/env python3
"""Verify a derived bag from make_map_frame_bag.py before it is handed over.

    python3 verify_map_frame_bag.py --source <orig> --derived <mapframe> [--map m.yaml]

Checks the things the consumer cannot check for us, and that would be silent
failures on our side:

1. **Stamps are byte-exact.** Every `pose_map` header stamp must equal, sec and
   nanosec, the `odometry/local` stamp it came from. The cross-machine camera
   merge is on header stamp, so a re-stamp is the one defect that would destroy
   the deliverable while every plot still looked right.
2. **The two delivered shapes agree.** `pose_map` should equal the composition
   of the delivered `map->odom` with the source `odom->base_link`. If they
   disagree, one of them is wrong and the consumer picks a coin.
3. **Rate over the whole run** clears their >=20 Hz requirement.
4. **Ordering and monotonicity** — a bag that is written out of order replays
   in a way no one expects.
"""

import argparse
import math
import os

import numpy as np
import yaml

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read(path, topics):
    meta = yaml.safe_load(open(os.path.join(path, "metadata.yaml")))
    info = meta["rosbag2_bagfile_information"]
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path,
                                     storage_id=info.get("storage_identifier", "mcap")),
           rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while r.has_next():
        topic, data, t = r.read_next()
        yield topic, deserialize_message(data, get_message(types[topic])), t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", required=True)
    ap.add_argument("--derived", required=True)
    ap.add_argument("--ns", default=os.environ.get("NS", "gosling1"))
    args = ap.parse_args()
    ns = args.ns
    ok = True

    src = [(m.header.stamp.sec, m.header.stamp.nanosec,
            m.pose.pose.position.x, m.pose.pose.position.y,
            yaw_of(m.pose.pose.orientation))
           for _, m, _ in read(args.source, [f"/{ns}/odometry/local"])]

    der, mo = [], []
    for topic, m, t in read(args.derived, [f"/{ns}/pose_map", f"/{ns}/tf"]):
        if topic.endswith("pose_map"):
            der.append((m.header.stamp.sec, m.header.stamp.nanosec,
                        m.pose.pose.position.x, m.pose.pose.position.y,
                        yaw_of(m.pose.pose.orientation), m.header.frame_id,
                        m.child_frame_id, t))
        else:
            for tr in m.transforms:
                mo.append((tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9,
                           tr.transform.translation.x, tr.transform.translation.y,
                           yaw_of(tr.transform.rotation),
                           tr.header.frame_id.lstrip("/"),
                           tr.child_frame_id.lstrip("/")))

    print(f"source odometry/local : {len(src)}")
    print(f"derived pose_map      : {len(der)}")
    print(f"derived map->odom tf  : {len(mo)}")

    bad_edge = [e for e in mo if (e[4], e[5]) != ("map", "odom")]
    if bad_edge:
        print(f"  FAIL: {len(bad_edge)} tf entries are not map->odom"); ok = False
    else:
        print("  ok: every tf entry is exactly map->odom, nothing else leaked in")

    frames = {(d[5], d[6]) for d in der}
    if frames != {("map", "base_link")}:
        print(f"  FAIL: pose_map frames are {frames}"); ok = False
    else:
        print("  ok: pose_map is map -> base_link (base_link == rear axle)")

    # 1. byte-exact stamps
    n = min(len(src), len(der))
    mism = sum(1 for i in range(n) if src[i][0] != der[i][0] or src[i][1] != der[i][1])
    if mism or len(src) != len(der):
        print(f"  FAIL: {mism} stamp mismatches; counts {len(src)} vs {len(der)}")
        ok = False
    else:
        print("  ok: all pose_map stamps identical (sec AND nanosec) to the source")

    # 2. the two shapes agree
    st = np.array([d[0] + d[1] * 1e-9 for d in der])
    if mo:
        mo.sort()
        mt = np.array([e[0] for e in mo])
        j = np.clip(np.searchsorted(mt, st, side="right") - 1, 0, len(mt) - 1)
        dx = np.array([mo[k][1] for k in j]); dy = np.array([mo[k][2] for k in j])
        dth = np.array([mo[k][3] for k in j])
        sx = np.array([s[2] for s in src[:n]]); sy = np.array([s[3] for s in src[:n]])
        c, s_ = np.cos(dth[:n]), np.sin(dth[:n])
        ex = dx[:n] + c * sx - s_ * sy - np.array([d[2] for d in der[:n]])
        ey = dy[:n] + s_ * sx + c * sy - np.array([d[3] for d in der[:n]])
        d = np.hypot(ex, ey)
        if d.max() > 1e-6:
            print(f"  FAIL: pose_map and tf disagree by up to {d.max()*1000:.3f} mm")
            ok = False
        else:
            print(f"  ok: pose_map == tf o odom->base_link (max {d.max()*1e9:.1f} nm)")

    # 3. rate, 4. ordering
    dt = np.diff(st)
    print(f"  rate: {len(st)/(st[-1]-st[0]):.2f} Hz mean, "
          f"max gap {dt.max()*1000:.1f} ms  (requirement: >= 20 Hz)")
    if (len(st) / (st[-1] - st[0])) < 20:
        print("  FAIL: below the 20 Hz requirement"); ok = False
    if (dt <= 0).any():
        print(f"  FAIL: {(dt<=0).sum()} non-monotonic stamps"); ok = False
    else:
        print("  ok: stamps strictly increasing")

    wt = np.array([d[7] for d in der])
    if (np.diff(wt) < 0).any():
        print("  FAIL: bag write timestamps out of order"); ok = False
    else:
        print("  ok: bag timestamps in order")

    print("VERDICT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
