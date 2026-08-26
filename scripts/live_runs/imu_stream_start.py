#!/usr/bin/env python3
"""bug-248: the first second of the camera IMU chain, raw and filtered side by side."""
import sys, math
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = sys.argv[1]
N = int(sys.argv[2]) if len(sys.argv) > 2 else 16
r = SequentialReader(); r.open(StorageOptions(uri=BAG, storage_id='sqlite3'), ConverterOptions('',''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
cache = {}
def de(tp, d):
    if tp not in cache: cache[tp] = get_message(types[tp])
    return deserialize_message(d, cache[tp])
RAW, FILT = '/camera/imu', '/camera/imu/filtered'
raw, filt = [], []
while r.has_next():
    tp, d, t = r.read_next()
    if tp == RAW and len(raw) < N * 20: raw.append((t*1e-9, de(tp, d)))
    elif tp == FILT and len(filt) < N * 20: filt.append((t*1e-9, de(tp, d)))
    if len(raw) >= N*20 and len(filt) >= N*20: break

def stamp(m): return m.header.stamp.sec + m.header.stamp.nanosec*1e-9
def rpy(q):
    sinp = 2*(q.w*q.y - q.z*q.x)
    return (math.degrees(math.atan2(2*(q.w*q.x + q.y*q.z), 1-2*(q.x**2+q.y**2))),
            math.degrees(math.asin(max(-1, min(1, sinp)))),
            math.degrees(math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2+q.z**2))))

print(f"=== first {N} /camera/imu/filtered (madgwick OUT) ===")
print(f"{'recv+':>8} {'hdr_dt':>14} {'roll':>9} {'pitch':>9} {'yaw':>9}   qw")
t0 = filt[0][0]; prev = None
for t, m in filt[:N]:
    s = stamp(m); d = '' if prev is None else f"{s-prev:+.4f}"
    prev = s
    rr, pp, yy = rpy(m.orientation)
    print(f"{t-t0:8.4f} {d:>14} {rr:9.3f} {pp:9.3f} {yy:9.3f}   {m.orientation.w:.5f}")

print(f"\n=== first {N} /camera/imu (RAW, driver out) ===")
print(f"{'recv+':>8} {'hdr_dt':>14} {'wx':>9} {'wy':>9} {'wz':>9} {'ax':>8} {'ay':>8} {'az':>8}")
tr0 = raw[0][0]; prev = None
for t, m in raw[:N]:
    s = stamp(m); d = '' if prev is None else f"{s-prev:+.4f}"
    prev = s
    w, a = m.angular_velocity, m.linear_acceleration
    print(f"{t-tr0:8.4f} {d:>14} {w.x:9.5f} {w.y:9.5f} {w.z:9.5f} {a.x:8.3f} {a.y:8.3f} {a.z:8.3f}")

# how long do the raw header stamps stay pathological?
st = [stamp(m) for _, m in raw]
bad = [i for i, (a, b) in enumerate(zip(st, st[1:])) if abs(b-a) > 1.0]
print(f"\nraw header-stamp steps >1 s: {len(bad)} of {len(st)-1} samples examined; "
      f"indices {bad[:10]}")
if bad:
    print(f"  last one at raw sample #{bad[-1]+1}, recv +{raw[bad[-1]+1][0]-tr0:.3f}s "
          f"-> the driver's stamps settle after that")
