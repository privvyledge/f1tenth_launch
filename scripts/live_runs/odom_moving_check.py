#!/usr/bin/env python3
"""Validate odometry/local against its own inputs while the car is MOVING.

    python3 odom_moving_check.py <bag> [--tape 4.00] [--tape-yaw 0]

yaw_drift.py answers the parked question: does the estimate stay still when the
car does. It cannot answer the moving one. A scale error in
speed_to_erpm_gain, a wheelbase error in the vesc_odom yaw rate, or an rf2o
that quietly stops updating under motion all look perfect parked and only
appear once the car travels. As of 2026-08-10 nothing in this repo had ever
measured them: every odometry number on record was taken stationary.

What it reports, restricted to the samples where the car was actually moving:

  per source   path length, net displacement, net yaw, speed, rate, worst gap
  scale        each source's path against odometry/local, and against a tape
               measurement if one is given
  heading      how far odometry/local's yaw drifted from rf2o and from VSLAM
  health       whether the EKF held its rate under load

Two numbers per source because they fail differently. Net displacement
(start->end straight line) is robust but only meaningful on a drive that does
not return to its start. Path length (integrated step size) works on any shape
but is biased HIGH by position noise, since every jitter step adds. On a
straight leg they should agree; a path much larger than net on a straight leg
is noise, not distance.

Ground truth comes from the floor, not from another estimator. --tape is the
tape-measured straight-line distance between the start and end marks; only it
can distinguish "all four sources agree" from "all four are wrong by the same
scale factor", which is exactly what a mis-calibrated speed_to_erpm_gain
produces (measured 3687-4022 across six bags against a configured 3750, so
~7 % is the width of what is already known, not an error).

Bag: any set that carries TOPICS_ODOM_LOCAL + TOPICS_VEHICLE, e.g. `set_array
mpc` from topic_sets.sh. VSLAM only appears on the GPU path.
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'analysis'))
from check_map_frame import read_bag, wrap, yaw_of  # noqa: E402

# (topic, label). Order is the report order. odometry/local first: it is the
# subject, and every ratio below is taken against it.
SOURCES = [
    ('odometry/local',                'EKF local (odom)'),
    ('vehicle/vesc_odom',             'VESC wheel odom'),
    ('odom/rf2o',                     'rf2o LiDAR odom'),
    ('visual_slam/tracking/odometry', 'Isaac VSLAM (VO)'),
]
REF = 'odometry/local'

# Established on gosling1 2026-08-04 with launch_icp_odometry off: 29.70 Hz,
# 0.180 s worst gap. With ICP on the same stack managed 12.78 Hz and 2.44 s.
# These are the floors that separate those two cases, not spec numbers.
MIN_EKF_RATE = 25.0
MAX_EKF_GAP = 0.5

# A source reporting less than this fraction of the reference path, while still
# publishing, is frozen rather than merely inaccurate. Loose on purpose: rf2o
# and VO legitimately disagree with the EKF by tens of percent, never by 20x.
FROZEN_FRACTION = 0.05


def read_sources(bag, ns):
    """{topic_suffix: [(t, x, y, yaw, vx, wz), ...]} in bag order."""
    wanted = {f'/{ns}/{s}' if ns else f'/{s}': s for s, _ in SOURCES}
    out = {s: [] for s, _ in SOURCES}
    for topic, msg, _ in read_bag(bag, set(wanted)):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        out[wanted[topic]].append((
            stamp,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_of(msg.pose.pose.orientation),
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z,
        ))
    return {k: sorted(v) for k, v in out.items()}


def moving_intervals(samples, thresh, pad):
    """[(t_start, t_end)] where |vx| exceeded thresh, each padded outward.

    Built from ONE source (odometry/local) and then applied to all of them, so
    every source is scored over the identical stretch of wall time. Scoring
    each source over its own moving window would compare different drives.

    The pad exists because the speed a source reports lags the motion it is
    reporting; without it the first and last few centimetres of every leg fall
    outside the window and net displacement reads short.
    """
    spans, start = [], None
    for i, s in enumerate(samples):
        fast = abs(s[4]) > thresh
        if fast and start is None:
            start = s[0]
        elif not fast and start is not None:
            spans.append((start - pad, samples[i - 1][0] + pad))
            start = None
    if start is not None:
        spans.append((start - pad, samples[-1][0] + pad))

    # Merge spans the padding has made overlap, so a stop-and-go drive is not
    # counted twice across the boundary.
    merged = []
    for span in spans:
        if merged and span[0] <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], span[1]))
        else:
            merged.append(span)
    return merged


def select(samples, spans):
    return [s for s in samples if any(a <= s[0] <= b for a, b in spans)]


def summarize(samples, moving):
    """Health over the whole bag, geometry over the moving samples only."""
    if not samples:
        return None
    span = samples[-1][0] - samples[0][0]
    gaps = [b[0] - a[0] for a, b in zip(samples, samples[1:])]
    row = {
        'n': len(samples),
        'hz': len(samples) / span if span > 0 else float('nan'),
        'gap': max(gaps) if gaps else float('nan'),
        'n_move': len(moving),
    }
    if len(moving) < 2:
        return row

    path = sum(math.hypot(b[1] - a[1], b[2] - a[2])
               for a, b in zip(moving, moving[1:]))
    dyaw = sum(wrap(b[3] - a[3]) for a, b in zip(moving, moving[1:]))
    row.update({
        'path': path,
        'net': math.hypot(moving[-1][1] - moving[0][1],
                          moving[-1][2] - moving[0][2]),
        'dyaw': math.degrees(dyaw),
        'v_mean': sum(abs(s[4]) for s in moving) / len(moving),
        'v_max': max(abs(s[4]) for s in moving),
        'secs': moving[-1][0] - moving[0][0],
    })
    return row


def fmt(row, key, spec='7.3f'):
    if row is None or key not in row:
        return f'{"--":>7}'
    return f'{row[key]:{spec}}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--ns', default=os.environ.get('NS', 'gosling1'),
                    help="robot namespace; '' for an un-namespaced stack")
    ap.add_argument('--tape', type=float,
                    help='tape-measured start->end distance, metres')
    ap.add_argument('--tape-yaw', type=float,
                    help='tape-measured heading change, degrees (+ = left)')
    ap.add_argument('--moving-speed', type=float, default=0.15,
                    help='|vx| above which the car counts as moving (m/s). '
                         'Ground breakaway measured 0.20-0.26 m/s, so 0.15 '
                         'sits below the slowest real motion and above noise.')
    ap.add_argument('--pad', type=float, default=0.3,
                    help='seconds of padding on each moving interval')
    args = ap.parse_args()

    ns = args.ns.strip('/')
    data = read_sources(args.bag, ns)

    ref = data[REF]
    if len(ref) < 2:
        print(f'{REF} is empty in this bag (namespace {ns or "<root>"}).')
        print('Nothing can be validated without it. Check --ns first: a wrong')
        print('prefix reads exactly like a dead stack.')
        return 1

    spans = moving_intervals(ref, args.moving_speed, args.pad)
    moving_secs = sum(b - a for a, b in spans)
    if not spans:
        print(f'The car never exceeded {args.moving_speed} m/s in this bag.')
        print('This is a parked bag; yaw_drift.py is the tool for those.')
        return 1

    rows = {}
    for topic, _ in SOURCES:
        rows[topic] = summarize(data[topic], select(data[topic], spans))

    print(f'\nbag        {args.bag}')
    print(f'namespace  {ns or "<root>"}')
    print(f'moving     {moving_secs:.1f} s over {len(spans)} interval(s), '
          f'threshold {args.moving_speed} m/s\n')

    print(f'{"source":<22}{"path":>8}{"net":>8}{"dyaw":>8}'
          f'{"v_mean":>8}{"v_max":>8}{"Hz":>7}{"gap":>7}{"n_mov":>7}')
    print(f'{"":22}{"m":>8}{"m":>8}{"deg":>8}{"m/s":>8}{"m/s":>8}'
          f'{"":>7}{"s":>7}{"":>7}')
    print('-' * 83)
    for topic, label in SOURCES:
        r = rows[topic]
        print(f'{label:<22}{fmt(r, "path")}{fmt(r, "net")}'
              f'{fmt(r, "dyaw", "8.2f")}{fmt(r, "v_mean")}{fmt(r, "v_max")}'
              f'{fmt(r, "hz", "7.2f")}{fmt(r, "gap", "7.3f")}'
              f'{fmt(r, "n_move", "7d")}')

    # ---------------------------------------------------------------- scale --
    print('\nscale, against odometry/local')
    base = rows[REF]
    for topic, label in SOURCES:
        if topic == REF:
            continue
        r = rows[topic]
        if r is None or 'path' not in r or base['path'] <= 0:
            print(f'  {label:<22} --   (no data over the moving window)')
            continue
        line = f'  {label:<22} path x{r["path"] / base["path"]:.3f}'
        # net ratio is undefined on a drive that returns to its start
        if base['net'] > 0.05:
            line += f'   net x{r["net"] / base["net"]:.3f}'
        print(line)

    # ------------------------------------------------------------- vs tape --
    if args.tape is not None:
        print(f'\nscale, against the tape ({args.tape:.3f} m)')
        for topic, label in SOURCES:
            r = rows[topic]
            if r is None or 'net' not in r:
                print(f'  {label:<22} --')
                continue
            err = 100.0 * (r['net'] - args.tape) / args.tape
            print(f'  {label:<22} net {r["net"]:6.3f} m   {err:+6.1f} %')
        print('  A common error across ALL sources is a speed/geometry')
        print('  calibration fault, not a fusion fault. speed_to_erpm_gain is')
        print('  only known to ~7 %, so anything inside that band is expected.')
    else:
        print('\nNo --tape given, so this run can only show the sources')
        print('agreeing with each other. Agreement is not accuracy: a wrong')
        print('speed_to_erpm_gain moves vesc_odom and the EKF together.')

    # ---------------------------------------------------------- heading --
    print('\nheading, odometry/local minus each independent source')
    if 'dyaw' in base:
        for topic, label in SOURCES:
            if topic == REF:
                continue
            r = rows[topic]
            if r is None or 'dyaw' not in r:
                print(f'  {label:<22} --')
                continue
            d = base['dyaw'] - r['dyaw']
            per_min = d / (base['secs'] / 60.0) if base.get('secs') else float('nan')
            print(f'  {label:<22} {d:+7.2f} deg over the window '
                  f'({per_min:+.1f} deg/min)')
    if args.tape_yaw is not None and 'dyaw' in base:
        print(f'  {"vs tape":<22} {base["dyaw"] - args.tape_yaw:+7.2f} deg '
              f'(tape {args.tape_yaw:+.1f})')

    # ----------------------------------------------------------- health --
    print('\nhealth')
    ok = True
    if base['hz'] < MIN_EKF_RATE:
        print(f'  FAIL  odometry/local ran at {base["hz"]:.2f} Hz '
              f'(floor {MIN_EKF_RATE}). Under load the EKF is missing its '
              f'update rate; check whether ICP odometry got switched on.')
        ok = False
    if base['gap'] > MAX_EKF_GAP:
        print(f'  FAIL  worst odometry/local gap {base["gap"]:.3f} s '
              f'(limit {MAX_EKF_GAP}). A controller sees this as a frozen pose.')
        ok = False
    for topic, label in SOURCES:
        r = rows[topic]
        if r is None:
            print(f'  note  {label} never published '
                  f'(expected if VSLAM is down or on the CPU path)')
        elif r['n_move'] < 2:
            print(f'  FAIL  {label} published, but not while the car moved. '
                  f'This is the failure yaw_drift.py cannot see.')
            ok = False
        elif (base.get('path', 0.0) > 0.5
                and r.get('path', 0.0) < FROZEN_FRACTION * base['path']):
            # Publishing at full rate while reporting no motion. The rf2o
            # zero-velocity gate added 2026-08-09 fails exactly this way if it
            # latches on: the EKF then fuses a confident, wrong "we are
            # stationary" the whole time the car is driving.
            print(f'  FAIL  {label} kept publishing but reported '
                  f'{r["path"]:.3f} m while the car travelled '
                  f'{base["path"]:.3f} m. Frozen source, not a quiet one.')
            ok = False
    if ok:
        print('  odometry/local held its rate through the drive.')

    print('\nThresholds above are the measured history of this vehicle, not')
    print('spec. The scale and heading numbers have no acceptance band yet --')
    print('this is the first moving measurement, so it sets the baseline.\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
