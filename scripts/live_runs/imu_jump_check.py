#!/usr/bin/env python3
"""bug-248: is the 82 deg attitude jump in the SENSOR stream or in madgwick?

Compares /camera/imu (raw, from the RealSense driver, unite_imu_method:2) against
/camera/imu/filtered (madgwick output = ekf_odom's imu1) over the same window.

The discriminator: madgwick's orientation can only move as fast as the rates it is
fed, plus its accel-based correction. So for the interval spanning the largest
filtered attitude step, compare
    (a) the step actually taken by the filtered orientation, against
    (b) the rotation the RAW gyro reports over that same interval.
If (a) >> (b), the discontinuity is manufactured downstream of the sensor rates.
The raw accel direction is reported too, because that is madgwick's other input
and the only thing that can swing its attitude faster than the gyro.
"""
import sys, math
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = sys.argv[1]
RAW, FILT = '/camera/imu', '/camera/imu/filtered'

r = SequentialReader()
r.open(StorageOptions(uri=BAG, storage_id='sqlite3'), ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
cache = {}
def de(tp, d):
    if tp not in cache: cache[tp] = get_message(types[tp])
    return deserialize_message(d, cache[tp])

want = [RAW, FILT, '/odometry/local']
data = {t: [] for t in want}
while r.has_next():
    tp, d, trec = r.read_next()
    if tp in data: data[tp].append((trec * 1e-9, de(tp, d)))

for t in want:
    print(f"{t:26s} {len(data[t]):7d} msgs")
if not data[FILT]:
    sys.exit("no /camera/imu/filtered in this bag")
if not data[RAW]:
    print(f"\n!! {RAW} NOT IN THIS BAG -- it cannot answer bug-248 (this is exactly\n"
          f"   how ekfdiag_run12_control came up short). Re-record with it.\n")

def stamp(m): return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
def qang(a, b):
    """angle of the rotation taking quaternion a to b, in degrees"""
    d = abs(a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z)
    return math.degrees(2.0 * math.acos(min(1.0, d)))

t_bag = min(v[0][0] for v in data.values() if v)
t_launch = data[FILT][0][0]
print(f"\nbag starts {t_bag:.3f}; first /camera/imu/filtered at +{t_launch-t_bag:.2f}s "
      f"(times below are relative to the FIRST FILTERED SAMPLE unless said otherwise)")
if data[RAW]:
    print(f"first raw /camera/imu at +{data[RAW][0][0]-t_bag:.2f}s from bag start "
          f"({data[RAW][0][0]-t_launch:+.3f}s vs first filtered)")

# ---- 1. attitude steps in the FILTERED stream
steps = []
for (ta, ma), (tb, mb) in zip(data[FILT], data[FILT][1:]):
    steps.append((qang(ma.orientation, mb.orientation), tb, stamp(mb) - stamp(ma), tb - ta))
steps.sort(reverse=True)
print(f"\n=== /camera/imu/filtered: {len(steps)+1} samples, "
      f"{data[FILT][-1][0]-t_launch:.1f}s span")
print("--- 8 largest consecutive attitude steps ---")
print(f"{'deg':>10} {'t+ (s)':>10} {'hdr_dt':>14} {'recv_dt':>10}")
for a, t, dt, rdt in steps[:8]:
    print(f"{a:10.3f} {t-t_launch:10.2f} {dt:14.4f} {rdt:10.4f}")

big, t_big, dt_big, rdt_big = steps[0]
ratio = big / steps[1][0] if len(steps) > 1 and steps[1][0] > 0 else float('inf')
print(f"\nlargest step {big:.2f} deg at t+{t_big-t_launch:.2f}s "
      f"(= bag-relative t+{t_big-t_bag:.2f}s), "
      f"{ratio:.0f}x the next largest ({steps[1][0]:.2f} deg)")
JUMP = big > 10.0
print("VERDICT: a discontinuous attitude jump IS present in this launch"
      if JUMP else
      "VERDICT: NO large attitude jump in this launch (largest step is small)")

# ---- 2. the same interval in the RAW stream
if data[RAW] and JUMP:
    lo, hi = t_big - rdt_big - 1e-6, t_big + 1e-6
    span = [(t, m) for t, m in data[RAW] if lo <= t <= hi]
    print(f"\n=== {RAW} across the jump interval ({rdt_big*1e3:.1f} ms recv, {len(span)} raw samples) ===")
    if span:
        rot = 0.0
        prev = None
        for t, m in span:
            w = m.angular_velocity
            mag = math.sqrt(w.x**2 + w.y**2 + w.z**2)
            if prev is not None:
                rot += mag * (t - prev)
            prev = t
        print(f"  rotation implied by raw gyro over the interval: {math.degrees(rot):.3f} deg")
        print(f"  filtered orientation moved:                     {big:.3f} deg")
        if math.degrees(rot) * 5 < big:
            print("  -> the raw RATES DO NOT SUPPORT the jump; it is manufactured "
                  "downstream (madgwick or its accel correction), not in the gyro.")
        else:
            print("  -> the raw rates could account for the jump; suspect the sensor stream.")
        def adir(m):
            a = m.linear_acceleration
            n = math.sqrt(a.x**2 + a.y**2 + a.z**2) or 1.0
            return (a.x/n, a.y/n, a.z/n), n
        (d0, n0), (d1, n1) = adir(span[0][1]), adir(span[-1][1])
        dot = max(-1.0, min(1.0, sum(x*y for x, y in zip(d0, d1))))
        print(f"  raw accel direction change over the interval: {math.degrees(math.acos(dot)):.3f} deg"
              f"  (|a| {n0:.3f} -> {n1:.3f} m/s^2)")

    # arrival gaps either side -- the run-12 note flagged a 33 ms gap before the jump
    near = [(t, m) for t, m in data[RAW] if t_big - 2.0 <= t <= t_big + 2.0]
    if len(near) > 2:
        gaps = [(b[0]-a[0], a[0]) for a, b in zip(near, near[1:])]
        gmax, gt = max(gaps)
        print(f"  raw arrival gaps in +-2 s: median {sorted(g[0] for g in gaps)[len(gaps)//2]*1e3:.2f} ms, "
              f"max {gmax*1e3:.2f} ms at t+{gt-t_launch:.2f}s")
    nearf = [(t, m) for t, m in data[FILT] if t_big - 2.0 <= t <= t_big + 2.0]
    if len(nearf) > 2:
        gaps = [(b[0]-a[0], a[0]) for a, b in zip(nearf, nearf[1:])]
        gmax, gt = max(gaps)
        print(f"  filtered arrival gaps in +-2 s: median "
              f"{sorted(g[0] for g in gaps)[len(gaps)//2]*1e3:.2f} ms, "
              f"max {gmax*1e3:.2f} ms at t+{gt-t_launch:.2f}s")

# ---- 3. did ekf_odom run away? (bug-244 acceptance for this launch)
loc = data['/odometry/local']
if loc:
    vmax, tmax = max((abs(m.twist.twist.linear.x) + abs(m.twist.twist.linear.y), t)
                     for t, m in loc)
    t30 = [(t, m) for t, m in loc if t - loc[0][0] >= 30.0]
    print(f"\n=== bug-244 acceptance ===")
    print(f"  /odometry/local max |v| = {vmax:.6f} m/s at t+{tmax-loc[0][0]:.1f}s")
    if t30:
        t, m = t30[0]
        p, v = m.pose.pose.position, m.twist.twist.linear
        print(f"  at t+30s: |v|={abs(v.x)+abs(v.y):.3e} m/s  pose=({p.x:.4f}, {p.y:.4f})")
    print("  PASS -- no runaway" if vmax < 0.5 else "  FAIL -- DIVERGED")
