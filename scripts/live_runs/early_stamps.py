#!/usr/bin/env python3
"""Are the RealSense driver's first-sample timestamp anomalies (bug-248 root
cause) still present, and did the attitude stay clean anyway?

Companion to imu_jump_check.py, not a duplicate of it. That tool finds the
LARGEST ATTITUDE STEPS and reports the header dt at each; this one goes the
other way, looking at the START of the stream where the pre-fix jump always
landed, and asks whether the provocation is still arriving.

That distinction is what makes a post-fix run evidence rather than an absence.
On run18_constdt the driver still emitted a +865.55 s header discontinuity and
the attitude moved 0.0121 deg through it -- madgwick absorbing the provocation,
which "the jump did not recur" on its own would not have shown.

Usage: early_stamps.py <bag_dir>
"""
import sys, sqlite3, math
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

bag = sys.argv[1]
db = sorted(__import__("glob").glob(bag + "/*.db3"))[0]
con = sqlite3.connect(db)
tops = {r[0]: (r[1], r[2]) for r in con.execute("select id,name,type from topics")}

def read(name, limit=None):
    tid = [k for k, v in tops.items() if v[0] == name]
    if not tid: return []
    cls = get_message(tops[tid[0]][1])
    q = "select timestamp,data from messages where topic_id=? order by timestamp"
    if limit: q += " limit %d" % limit
    return [(t, deserialize_message(bytes(d), cls)) for t, d in con.execute(q, (tid[0],))]

def hdr(m): return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

for topic in ("/camera/imu", "/camera/imu/filtered"):
    msgs = read(topic, 12)
    if not msgs:
        print("%s: none" % topic); continue
    print("=== %s: first 10 header deltas ===" % topic)
    h0 = hdr(msgs[0][1])
    for i in range(1, min(10, len(msgs))):
        dt = hdr(msgs[i][1]) - hdr(msgs[i-1][1])
        flag = "  <-- ANOMALY" if abs(dt) > 0.05 else ""
        print("   %2d  hdr_dt %+12.6f s%s" % (i, dt, flag))
    print()

# attitude over the first 5 s of filtered, where the pre-fix jump always landed
f = read("/camera/imu/filtered")
if f:
    t0 = f[0][0]
    win = [(t, m) for t, m in f if (t - t0) / 1e9 <= 5.0]
    def yaw(q):
        return math.degrees(math.atan2(2*(q.w*q.z + q.x*q.y),
                                       1 - 2*(q.y*q.y + q.z*q.z)))
    ys = [yaw(m.orientation) for _, m in win]
    steps = [abs(ys[i] - ys[i-1]) for i in range(1, len(ys))]
    steps = [s if s < 180 else 360 - s for s in steps]
    print("first 5 s of filtered: n=%d  max consecutive yaw step %.4f deg" % (len(ys), max(steps) if steps else 0))
    print("   (pre-fix bug-248 put a 63-167 deg step in this window on 4 of 4 launches)")
