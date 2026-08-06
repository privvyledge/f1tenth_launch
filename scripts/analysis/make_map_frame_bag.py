#!/usr/bin/env python3
"""Build the derived, map-frame bag that goes to the LUCIO consumer.

    python3 make_map_frame_bag.py --source <orig_bag> \
        --localization <loc_bag> --out <derived_bag>

It writes BOTH shapes their request allows, so they can use whichever fits:

  /<ns>/tf         TFMessage carrying ONLY map->odom. Played alongside the
                   original bag it completes the chain to the original
                   odom->base_link, which stays theirs, untouched.
  /<ns>/pose_map   nav_msgs/Odometry, frame_id `map`, child_frame_id
                   `base_link`, one message per source `odometry/local`
                   message — already composed, if they would rather not run tf2.

THE STAMPS ARE THE POINT. The cross-machine merge with the camera bags is on
header stamp, with chrony holding velox1 to 0.4-1.6 us. So every `pose_map`
message carries the header stamp of the `odometry/local` message it was derived
from, copied field by field, and each is written at the source bag's own receive
timestamp. Nothing here is re-stamped, re-sampled or re-ordered.

`base_link` IS THE REAR AXLE on this vehicle (`base_link->rear_axle` is the
identity transform). Not `base_footprint`, 33 mm below in z, and not
`front_axle`, 256 mm ahead in x. A ~0.13 m frame mistake here is invisible to
every check on our side, so it is stated in the emitted README too.
"""

import argparse
import math
import os

import numpy as np
import yaml

from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def open_reader(path):
    meta = yaml.safe_load(open(os.path.join(path, "metadata.yaml")))
    info = meta["rosbag2_bagfile_information"]
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path,
                                  storage_id=info.get("storage_identifier", "mcap")),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, types


def read_map_odom(loc_bag, ns):
    """map->odom from the localization bag, as (stamp_s, x, y, yaw, TransformStamped)."""
    reader, types = open_reader(loc_bag)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[f"/{ns}/tf"]))
    out = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, get_message(types[topic]))
        for tr in msg.transforms:
            if (tr.header.frame_id.lstrip("/"), tr.child_frame_id.lstrip("/")) \
                    == ("map", "odom"):
                s = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                out.append((s, tr.transform.translation.x, tr.transform.translation.y,
                            yaw_of(tr.transform.rotation), tr))
    out.sort(key=lambda r: r[0])
    return out


def read_amcl_cov(loc_bag, ns):
    reader, types = open_reader(loc_bag)
    try:
        reader.set_filter(rosbag2_py.StorageFilter(topics=[f"/{ns}/amcl_pose"]))
    except Exception:
        return np.empty((0, 4))
    rows = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != f"/{ns}/amcl_pose":
            continue
        m = deserialize_message(data, get_message(types[topic]))
        c = np.asarray(m.pose.covariance).reshape(6, 6)
        rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                     c[0, 0], c[1, 1], c[5, 5]))
    return np.array(rows) if rows else np.empty((0, 4))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", required=True, help="original drive bag")
    ap.add_argument("--localization", required=True, help="bag from 51_localize_offline.sh")
    ap.add_argument("--out", required=True)
    ap.add_argument("--ns", default=os.environ.get("NS", "gosling1"))
    ap.add_argument("--storage", default="mcap")
    args = ap.parse_args()

    if os.path.exists(args.out):
        raise SystemExit(f"refusing to overwrite {args.out}")

    ns = args.ns
    mo = read_map_odom(args.localization, ns)
    if not mo:
        raise SystemExit("no map->odom in the localization bag — localize first")
    cov = read_amcl_cov(args.localization, ns)
    mo_t = np.array([r[0] for r in mo])
    print(f"map->odom: {len(mo)} transforms, "
          f"{mo_t[0]:.3f} .. {mo_t[-1]:.3f} ({mo_t[-1]-mo_t[0]:.1f} s)")

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=args.out, storage_id=args.storage),
                rosbag2_py.ConverterOptions("", ""))
    for name, typ in ((f"/{ns}/tf", "tf2_msgs/msg/TFMessage"),
                      (f"/{ns}/pose_map", "nav_msgs/msg/Odometry")):
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=name, type=typ, serialization_format="cdr"))

    # --- map->odom, verbatim from the localizer -----------------------------
    for s, _, _, _, tr in mo:
        m = TFMessage()
        m.transforms = [tr]
        writer.write(f"/{ns}/tf", serialize_message(m), int(round(s * 1e9)))

    # --- composed map->base_link at every source odometry/local stamp -------
    reader, types = open_reader(args.source)
    odom_topic = f"/{ns}/odometry/local"
    reader.set_filter(rosbag2_py.StorageFilter(topics=[odom_topic]))
    n = 0
    stale = 0
    while reader.has_next():
        topic, data, t_recv = reader.read_next()
        src = deserialize_message(data, get_message(types[topic]))
        s = src.header.stamp.sec + src.header.stamp.nanosec * 1e-9

        # Zero-order hold: map->odom is piecewise constant between localizer
        # updates, so the correct value at stamp s is the last one published at
        # or before s. Interpolating between two corrections would invent a
        # motion the localizer never reported.
        i = int(np.searchsorted(mo_t, s, side="right")) - 1
        if i < 0:
            stale += 1
            i = 0
        _, dx, dy, dth, _ = mo[i]

        c, sn = math.cos(dth), math.sin(dth)
        px, py = src.pose.pose.position.x, src.pose.pose.position.y

        out = Odometry()
        out.header.stamp = src.header.stamp      # byte-exact, not recomputed
        out.header.frame_id = "map"
        out.child_frame_id = "base_link"         # == rear axle on this vehicle
        out.pose.pose.position.x = dx + c * px - sn * py
        out.pose.pose.position.y = dy + sn * px + c * py
        out.pose.pose.position.z = 0.0
        th = math.atan2(math.sin(dth + yaw_of(src.pose.pose.orientation)),
                        math.cos(dth + yaw_of(src.pose.pose.orientation)))
        out.pose.pose.orientation.z = math.sin(th / 2.0)
        out.pose.pose.orientation.w = math.cos(th / 2.0)
        # Twist is expressed in child_frame_id (base_link), so a map-frame
        # rotation leaves it unchanged — copy it through rather than rotating it.
        out.twist = src.twist

        pc = list(src.pose.covariance)
        if len(cov):
            j = int(np.searchsorted(cov[:, 0], s, side="right")) - 1
            if j >= 0:
                pc[0], pc[7], pc[35] = cov[j, 1], cov[j, 2], cov[j, 3]
        out.pose.covariance = pc

        writer.write(f"/{ns}/pose_map", serialize_message(out), t_recv)
        n += 1

    del writer
    print(f"pose_map: {n} messages written"
          + (f"  ({stale} before the first map->odom, held at the first value)"
             if stale else ""))

    readme = args.out.rstrip("/") + ".README.md"
    with open(readme, "w") as f:
        f.write(f"""# {os.path.basename(args.out.rstrip('/'))}

Derived from `{os.path.basename(args.source.rstrip('/'))}` by localizing it
against the shared map. It contains no sensor data — play it **alongside** the
original bag.

## Topics

| topic | type | what |
|---|---|---|
| `/{ns}/tf` | `tf2_msgs/msg/TFMessage` | **only** `map` -> `odom`, {len(mo)} transforms |
| `/{ns}/pose_map` | `nav_msgs/msg/Odometry` | `map` -> `base_link`, already composed, {n} messages |

Use whichever suits you. `/{ns}/tf` completes the chain to the original bag's
`odom` -> `base_link`, which is untouched. `/{ns}/pose_map` is the same pose
already composed, if you would rather not run tf2.

## The frame is the REAR AXLE

`child_frame_id` is `base_link`, and on this vehicle `base_link` is the **rear
axle** — `base_link` -> `rear_axle` is the identity transform. It is **not**
`base_footprint` (33 mm below in z) and **not** `front_axle` (256 mm ahead in
x). Getting this wrong is a ~0.13 m error that no check on either side would
catch, which is why it is written down here.

## Stamps

Every `pose_map` header stamp is copied field-for-field from the
`odometry/local` message it was derived from, and written at that message's own
bag timestamp. Nothing was re-stamped, re-sampled or re-ordered, so a merge on
header stamp against the camera bags behaves exactly as it does against the
original.

## Rate

`pose_map` follows `odometry/local`: 30 Hz across the whole run.
""")
    print(f"README: {readme}")


if __name__ == "__main__":
    main()
