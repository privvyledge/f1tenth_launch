#!/usr/bin/env python3
"""Score the imu_bias_remover offline test written by bias_test_driver.py.

Usage: bias_test_analyze.py <csv> [guard_seconds]

Four questions, all answerable with the car parked:

  1. does the bias estimate converge to the gyro's real stationary mean?
  2. while moving, is the output exactly raw - bias?
  3. while stationary, is the output pinned to zero?
  4. if the velocity source dies while reading "stopped", does the node stay
     pinned in the zeroing branch forever? (the staleness hazard, spec section 3)

GUARD BAND. The driver labels each output row by the phase at RECEIVE time, but
the node picked its branch when it processed the message, and the synthetic odom
flips at 50 Hz against a 200 Hz IMU. So a couple of samples either side of a
transition carry the wrong label. Checks 2 and 3 are scored with those excluded
and the raw placement is printed underneath, so a real mid-phase defect is still
visible rather than being masked by the guard.
"""
import csv
import sys

EXPECT_BIAS = -0.00214          # rad/s, RealSense z-gyro, measured 2026-08-09
BOUNDS = [25.0, 40.0, 50.0]     # must match PHASES in bias_test_driver.py

csv_path = sys.argv[1]
GUARD = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5

rows = []
with open(csv_path) as f:
    for r in csv.DictReader(f):
        rows.append((float(r["t"]), r["phase"], int(r["moving"]),
                     float(r["raw_z"]), float(r["out_z"]), float(r["bias_z"])))
if not rows:
    print("NO ROWS -- the node published nothing. Check remaps and QoS.")
    sys.exit(1)


def near_boundary(t):
    return any(abs(t - b) <= GUARD for b in BOUNDS)


def sel(p, guarded=False):
    return [r for r in rows if r[1] == p and (not guarded or not near_boundary(r[0]))]


print("samples: %d   span: %.1f s   guard band: +/-%.2f s around %s\n"
      % (len(rows), rows[-1][0] - rows[0][0], GUARD, BOUNDS))

for p in ("A_stationary", "B_moving", "C_stationary", "D_source_dead"):
    s = sel(p)
    if not s:
        print("%-14s  (no samples)" % p)
        continue
    raw = [r[3] for r in s]
    out = [r[4] for r in s]
    print("%-14s n=%-6d raw_z mean %+.6f   out_z mean %+.6f   bias_z end %+.6f"
          % (p, len(s), sum(raw) / len(raw), sum(out) / len(out), s[-1][5]))
print()

# --- 1: does the estimate converge to the measured constant? ---
a = sel("A_stationary")
if a:
    tail = a[int(len(a) * 0.75):]                 # last quarter of the window
    bias_end = sum(r[5] for r in tail) / len(tail)
    raw_mean = sum(r[3] for r in a) / len(a)
    print("CHECK 1  bias converges to the bag's own stationary z-gyro mean")
    print("         bias.z (settled)      %+.6f rad/s" % bias_end)
    print("         raw z-gyro mean       %+.6f rad/s   (this bag's own truth)" % raw_mean)
    print("         measured constant     %+.6f rad/s   (spec 2026-08-09)" % EXPECT_BIAS)
    print("         |bias - raw_mean|     %.6f  -> %s"
          % (abs(bias_end - raw_mean), "PASS" if abs(bias_end - raw_mean) < 2e-4 else "FAIL"))
    print()

# --- 2: is the moving output exactly raw - bias? ---
b = sel("B_moving", guarded=True)
if b:
    err = [abs(r[4] - (r[3] - r[5])) for r in b]
    print("CHECK 2  moving output == raw - bias, sample for sample")
    print("         n=%d  max residual %.3e  mean %.3e  -> %s"
          % (len(b), max(err), sum(err) / len(err), "PASS" if max(err) < 1e-9 else "FAIL"))
    print()

# --- 3: stationary output is hard zero ---
for p in ("A_stationary", "C_stationary"):
    s = sel(p, guarded=True)
    if s:
        mx = max(abs(r[4]) for r in s)
        print("CHECK 3  %-14s max|out_z| %.3e -> %s"
              % (p, mx, "PASS" if mx == 0.0 else "FAIL"))
print()

# --- 4: the staleness hazard ---
d = sel("D_source_dead")
if d:
    mx = max(abs(r[4]) for r in d)
    rawmx = max(abs(r[3]) for r in d)
    print("CHECK 4  velocity source stopped while it read 'stopped'")
    print("         n=%d  max|out_z| %.3e   (raw gyro still live: max|raw_z| %.6f)"
          % (len(d), mx, rawmx))
    if mx == 0.0:
        print("         -> HAZARD CONFIRMED: the rate stays pinned at 0 with no source.")
        print("            The node never leaves the zeroing branch, so a vesc_odom")
        print("            death while parked zeroes the gyro indefinitely, including")
        print("            once the car moves. Needs an external watchdog.")
    else:
        print("         -> node left the zeroing branch on its own; re-read the source.")
print()

# Raw placement of anything the guard band excluded, so it is never hidden.
zb = [r for r in rows if r[1] == "B_moving" and r[4] == 0.0]
nc = [r for r in rows if r[1] == "C_stationary" and r[4] != 0.0]
print("excluded-by-guard placement (expect a few samples within ~1 odom tick of a bound):")
print("  B_moving rows with out==0     : %d%s" % (len(zb),
      ("   t %.2f .. %.2f" % (zb[0][0], zb[-1][0])) if zb else ""))
print("  C_stationary rows with out!=0 : %d%s" % (len(nc),
      ("   t %.2f .. %.2f" % (nc[0][0], nc[-1][0])) if nc else ""))
