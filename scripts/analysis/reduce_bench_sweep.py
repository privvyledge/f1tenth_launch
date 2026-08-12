#!/usr/bin/env python3
"""Reduce Stage 0 bench-sweep tape readings into gain, offset, travel and toe.

Measurement method (see scripts/live_runs/BENCH_SWEEP_SHEET.md): a straightedge
is taped flat to the outer face of each wheel, horizontal, protruding fore and
aft.  Two marks on it a known distance ``d`` apart are measured perpendicular to
a baseline laid along the car's **left** side.  The angle of that wheel relative
to the baseline is ``atan((rear - front) / d)`` -- positive means the wheel
points left, matching REP-103.

The baseline does not need to be accurately parallel to the car.  A ``rear`` row
measures the two (fixed) rear wheels the same way and defines zero; every front
angle is taken as a difference against it, which cancels any baseline
misalignment.

Input CSV columns:
    label,fl_front,fl_rear,fr_front,fr_rear

``label`` is either the literal string ``rear`` (exactly one such row, whose
columns are then read as rear-left front/rear and rear-right front/rear) or a
servo value.  All offsets in millimetres.  Lines starting with ``#`` and blank
lines are ignored.

Usage:
    python3 reduce_bench_sweep.py readings.csv --d 250 --wheelbase 0.256
"""

import argparse
import csv
import math
import sys

# The in-motion Stage 2 result these bench numbers are meant to be compared
# against: mean of mapping_drive_170025 and figure8_172338, the two bags that
# turn both directions and never touch the servo clamp.
INMOTION_A = -0.87355   # rad per servo unit
INMOTION_S0 = 0.56005   # servo value at zero road-wheel angle


def angle_from_offsets(front_mm, rear_mm, d_mm):
    """Wheel angle relative to the baseline, radians, positive = left."""
    return math.atan((rear_mm - front_mm) / d_mm)


def bicycle_angle(delta_l, delta_r, wheelbase):
    """Single-track equivalent of two road-wheel angles.

    Exact Ackermann relation: both wheels share a turn centre, so the
    single-track angle is the one whose radius is the mean of the two wheel
    radii.  Falls back to the plain mean near zero, where the radii diverge.
    """
    if abs(delta_l) < 1e-6 or abs(delta_r) < 1e-6:
        return 0.5 * (delta_l + delta_r)
    if (delta_l > 0) != (delta_r > 0):
        # wheels point opposite ways (pure toe, no net steer)
        return 0.5 * (delta_l + delta_r)
    r_l = wheelbase / math.tan(delta_l)
    r_r = wheelbase / math.tan(delta_r)
    return math.atan(wheelbase / (0.5 * (r_l + r_r)))


def linfit(xs, ys):
    """Least-squares y = m*x + c.  Returns (m, c, rms_residual, n)."""
    n = len(xs)
    if n < 2:
        return None
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return None
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    m = sxy / sxx
    c = my - m * mx
    rms = math.sqrt(sum((y - (m * x + c)) ** 2 for x, y in zip(xs, ys)) / n)
    return m, c, rms, n


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv', help='readings CSV')
    ap.add_argument('--d', type=float, required=True,
                    help='distance between the two marks on the straightedge, mm')
    ap.add_argument('--wheelbase', type=float, default=0.256,
                    help='measured wheelbase in metres (default: 0.256)')
    args = ap.parse_args()

    rear = None
    rows = []
    with open(args.csv) as fh:
        for raw in fh:
            raw = raw.strip()
            if not raw or raw.startswith('#'):
                continue
            parts = next(csv.reader([raw]))
            if len(parts) < 5:
                print(f'skipping short row: {raw}', file=sys.stderr)
                continue
            label = parts[0].strip()
            vals = [float(p) for p in parts[1:5]]
            if label.lower() == 'rear':
                rear = vals
            else:
                rows.append((float(label), vals))

    if rear is None:
        sys.exit('no "rear" reference row found -- it defines zero')
    if not rows:
        sys.exit('no servo rows found')

    rows.sort(key=lambda r: r[0])

    ref = 0.5 * (angle_from_offsets(rear[0], rear[1], args.d)
                 + angle_from_offsets(rear[2], rear[3], args.d))
    rear_spread = math.degrees(abs(angle_from_offsets(rear[0], rear[1], args.d)
                                   - angle_from_offsets(rear[2], rear[3], args.d)))

    print(f'baseline reference from rear wheels: {math.degrees(ref):+.2f} deg '
          f'(L/R disagree by {rear_spread:.2f} deg)')
    if rear_spread > 1.0:
        print('  WARNING: rear wheels disagree by more than 1 deg -- check the '
              'straightedge seating or the rear axle before trusting anything below.')
    print(f'wheelbase {args.wheelbase:.4f} m, straightedge d = {args.d:g} mm\n')

    print(f'{"servo":>7} {"dFL deg":>9} {"dFR deg":>9} {"d_eff deg":>10} '
          f'{"d_eff rad":>10} {"toe deg":>9}')
    print('-' * 60)

    servos, deltas = [], []
    table = []
    for servo, vals in rows:
        d_fl = angle_from_offsets(vals[0], vals[1], args.d) - ref
        d_fr = angle_from_offsets(vals[2], vals[3], args.d) - ref
        d_eff = bicycle_angle(d_fl, d_fr, args.wheelbase)
        toe = d_fl - d_fr
        table.append((servo, d_fl, d_fr, d_eff, toe))
        servos.append(servo)
        deltas.append(d_eff)
        print(f'{servo:7.4f} {math.degrees(d_fl):9.2f} {math.degrees(d_fr):9.2f} '
              f'{math.degrees(d_eff):10.2f} {d_eff:10.4f} {math.degrees(toe):9.2f}')

    fit = linfit(servos, deltas)
    if fit is None:
        sys.exit('\nnot enough distinct servo values to fit')
    a, c, rms, n = fit
    s0 = -c / a

    print('\n--- overall fit  delta = a * (servo - s0) ---')
    print(f'a  = {a:+.4f} rad per servo unit   ({n} points, residual RMS '
          f'{math.degrees(rms):.2f} deg)')
    print(f's0 = {s0:.4f}   (servo at zero road-wheel angle)')
    print(f'=> steering_angle_to_servo_gain   = {1.0 / a:+.4f}')
    print(f'=> steering_angle_to_servo_offset = {s0:.4f}')

    print(f'\nin-motion Stage 2 for comparison: a = {INMOTION_A:+.4f}, '
          f's0 = {INMOTION_S0:.4f}')
    print(f'  bench/in-motion slope ratio {a / INMOTION_A:.4f} '
          f'({100 * (a / INMOTION_A - 1):+.1f} %)')
    print(f'  offset difference {s0 - INMOTION_S0:+.4f} servo units')

    left = [(s, d) for s, d in zip(servos, deltas) if d > 0.02]
    right = [(s, d) for s, d in zip(servos, deltas) if d < -0.02]
    for name, subset in (('left', left), ('right', right)):
        f = linfit([s for s, _ in subset], [d for _, d in subset]) if len(subset) >= 2 else None
        if f:
            print(f'  {name:>5} slope a = {f[0]:+.4f} rad/servo ({f[3]} points)')
        else:
            print(f'  {name:>5} slope: too few points')
    if len(left) >= 2 and len(right) >= 2:
        al = linfit([s for s, _ in left], [d for _, d in left])[0]
        ar = linfit([s for s, _ in right], [d for _, d in right])[0]
        print(f'  asymmetry {100 * (al / ar - 1):+.1f} % '
              f'(positive = more rad per servo unit to the left)')

    lo, hi = table[0], table[-1]
    print('\n--- travel ---')
    print(f'lowest servo tested  {lo[0]:.4f} -> {math.degrees(lo[3]):+.2f} deg '
          f'({lo[3]:+.4f} rad)   [low servo = LEFT]')
    print(f'highest servo tested {hi[0]:.4f} -> {math.degrees(hi[3]):+.2f} deg '
          f'({hi[3]:+.4f} rad)   [high servo = RIGHT]')
    print(f'mechanical centre of tested travel = {0.5 * (lo[0] + hi[0]):.4f} servo')
    print(f'zero-steer sits {s0 - 0.5 * (lo[0] + hi[0]):+.4f} servo units off that centre')
    sym = min(abs(math.degrees(lo[3])), abs(math.degrees(hi[3])))
    print(f'symmetric usable half-range = {sym:.2f} deg '
          f'({math.radians(sym):.4f} rad)  <- candidate max_steering')

    toes = [t for _, _, _, _, t in table]
    near = min(table, key=lambda r: abs(r[0] - s0))
    print('\n--- toe ---')
    print(f'at the servo value nearest zero-steer ({near[0]:.4f}): '
          f'{math.degrees(near[4]):+.2f} deg '
          f'({"toe-out" if near[4] > 0 else "toe-in"})')
    print(f'across the sweep: min {math.degrees(min(toes)):+.2f} deg, '
          f'max {math.degrees(max(toes)):+.2f} deg')


if __name__ == '__main__':
    main()
