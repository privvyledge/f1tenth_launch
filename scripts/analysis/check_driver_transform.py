#!/usr/bin/env python3
"""Check that ackermann_to_vesc applied the static affine map during a recording.

System identification for this vehicle regresses against the *driver outputs*
(`vehicle/commands/motor/speed`, `vehicle/commands/servo/position`) rather than
against `vehicle/ackermann_cmd`, because the driver's rate limiter sits between
them. That choice is only sound if nothing else unmodelled sits between command
and actuator. This script tests that directly:

    servo ?= clamp(gain_s * steering + offset_s, servo_min, servo_max)
    erpm  ?= gain_v * speed + offset_v

The constants default to what the driver was *configured with* at record time
(`config/vehicle/vesc.yaml`), which is the question here -- not what is correct.

Samples are classified rather than merely scored:

  match      residual within tolerance -- the affine map held
  clamped    the affine result fell outside [servo_min, servo_max] and the
             observed value sits at the corresponding bound
  transient  the command changed faster than the driver's configured rate limit
             over the joining interval, so the limiter is expected to disagree
  mismatch   none of the above -- unexplained, and the thing worth reporting

`std_msgs/Float64` carries no header, so the join is on bag receive timestamp
(nearest ackermann_cmd within --join-tol-ms). At ~115 Hz on both sides that is
a near 1:1 pairing.

Usage:
    python3 check_driver_transform.py <bag> [<bag> ...] [--ns /gosling1] [--json]
"""

from __future__ import annotations

import argparse
import json
import sys
from bisect import bisect_left
from pathlib import Path

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:  # pragma: no cover - depends on the ROS environment
    sys.exit("rosbag2_py/rclpy not importable -- source the ROS 2 overlay first")


# Defaults mirror config/vehicle/vesc.yaml as configured at record time, and
# launch/vehicle/vehicle.launch.py for the rate limits.
DEF_SPEED_GAIN = 3750.0
DEF_SPEED_OFFSET = 0.0
DEF_STEER_GAIN = -1.4
DEF_STEER_OFFSET = 0.56
DEF_SERVO_MIN = 0.08
DEF_SERVO_MAX = 0.92
DEF_MAX_ACCEL = 2.5      # m/s^2
DEF_MAX_STEER_RATE = 3.2  # rad/s


def storage_id_for(path: str) -> str:
    p = Path(path)
    if p.is_dir():
        if list(p.glob("*.mcap")):
            return "mcap"
        if list(p.glob("*.db3")):
            return "sqlite3"
    return ""


def read_topics(bag: str, wanted: set[str]) -> dict[str, list[tuple[int, object]]]:
    """Return {topic: [(recv_ns, msg), ...]} for the requested topics only."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id=storage_id_for(bag)),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = wanted - set(types)
    if missing:
        raise SystemExit(f"{bag}: missing topics {sorted(missing)}")

    classes = {name: get_message(types[name]) for name in wanted}
    out: dict[str, list[tuple[int, object]]] = {name: [] for name in wanted}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic in wanted:
            out[topic].append((t_ns, deserialize_message(data, classes[topic])))
    for name in out:
        out[name].sort(key=lambda kv: kv[0])
    return out


def best_match(
    stamps: list[int], values: list[float], t: int, window_ns: int,
    predict, observed: float,
) -> tuple[int, float] | None:
    """Index of the command in [t-window, t+window] whose affine image is closest
    to `observed`, plus that residual.

    A nearest-*timestamp* join is not good enough here. Commands and driver
    outputs both run at ~115 Hz with independent recorder jitter, so the nearest
    stamp is routinely the neighbouring message -- which shows up as residuals
    quantised to one command step (a fixed +/-29.7 erpm, +/-0.00377 servo) and
    looks like a transform error when it is a pairing error. Searching the small
    window for the best-fitting command removes that artifact; the price is that
    the chosen pairing must be reported (see `offset_ms` in the output) so a
    systematically non-zero or wide offset stays visible rather than absorbed.
    """
    if not stamps:
        return None
    lo = bisect_left(stamps, t - window_ns)
    hi = bisect_left(stamps, t + window_ns)
    if lo >= hi:
        # Nothing inside the window; fall back to the single closest stamp so
        # the sample is scored rather than silently dropped.
        i = bisect_left(stamps, t)
        i = min(max(i, 0), len(stamps) - 1)
        if abs(stamps[i] - t) > window_ns:
            return None
        lo, hi = i, i + 1
    best_i, best_r = lo, observed - predict(values[lo])
    for i in range(lo + 1, hi):
        r = observed - predict(values[i])
        if abs(r) < abs(best_r):
            best_i, best_r = i, r
    return best_i, best_r


def percentile(xs: list[float], q: float) -> float:
    if not xs:
        return 0.0
    xs = sorted(xs)
    k = (len(xs) - 1) * q
    lo, hi = int(k), min(int(k) + 1, len(xs) - 1)
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def classify_channel(
    cmd_t: list[int],
    cmd_v: list[float],
    out_t: list[int],
    out_v: list[float],
    gain: float,
    offset: float,
    tol: float,
    join_tol_ns: int,
    rate_limit: float | None,
    bounds: tuple[float, float] | None,
) -> dict:
    """Join one driver output against its command and classify every sample."""
    counts = {"match": 0, "beyond_bounds": 0, "transient": 0,
              "mismatch": 0, "unjoined": 0}
    residuals: list[float] = []
    mismatch_res: list[float] = []
    offsets_ms: list[float] = []
    examples: list[dict] = []

    def predict(c: float) -> float:
        return gain * c + offset

    for t, observed in zip(out_t, out_v):
        m = best_match(cmd_t, cmd_v, t, join_tol_ns, predict, observed)
        if m is None:
            counts["unjoined"] += 1
            continue
        i, residual = m
        offsets_ms.append((t - cmd_t[i]) / 1e6)
        residuals.append(residual)

        if abs(residual) <= tol:
            # The affine map held exactly. Note separately whether this sample
            # was outside the servo bounds -- `ackermann_to_vesc` does not clamp
            # (the clamp lives downstream in `vesc_driver`), so an out-of-bounds
            # value here is expected and is not a mismatch. It matters only
            # because it marks the samples the actuator did not actually follow.
            if bounds is not None and not (bounds[0] <= observed <= bounds[1]):
                counts["beyond_bounds"] += 1
            else:
                counts["match"] += 1
            continue

        # Transient: the *command* moved faster than the driver's configured
        # rate limit across this interval, so the limiter is expected to lag.
        if rate_limit is not None and i > 0:
            dt = (cmd_t[i] - cmd_t[i - 1]) / 1e9
            if dt > 0 and abs(cmd_v[i] - cmd_v[i - 1]) / dt > rate_limit:
                counts["transient"] += 1
                continue

        counts["mismatch"] += 1
        mismatch_res.append(residual)
        if len(examples) < 5:
            examples.append(
                {
                    "t_s": round(t / 1e9, 3),
                    "cmd": round(cmd_v[i], 5),
                    "predicted": round(predict(cmd_v[i]), 5),
                    "observed": round(observed, 5),
                    "residual": round(residual, 5),
                }
            )

    total = sum(counts.values())
    explained = counts["match"] + counts["beyond_bounds"] + counts["transient"]
    abs_res = [abs(r) for r in residuals]
    return {
        "counts": counts,
        "total": total,
        "explained_frac": explained / total if total else 0.0,
        "match_frac": (counts["match"] + counts["beyond_bounds"]) / total if total else 0.0,
        "resid_p50": percentile(abs_res, 0.50),
        "resid_p99": percentile(abs_res, 0.99),
        "resid_max": max(abs_res) if abs_res else 0.0,
        "mismatch_resid_p50": percentile([abs(r) for r in mismatch_res], 0.50),
        "offset_ms_p50": percentile(offsets_ms, 0.50),
        "offset_ms_p99": percentile(offsets_ms, 0.99),
        "examples": examples,
    }


def analyse(bag: str, ns: str, args: argparse.Namespace) -> dict:
    t_cmd = f"{ns}/vehicle/ackermann_cmd"
    t_erpm = f"{ns}/vehicle/commands/motor/speed"
    t_servo = f"{ns}/vehicle/commands/servo/position"

    data = read_topics(bag, {t_cmd, t_erpm, t_servo})
    cmd_t = [t for t, _ in data[t_cmd]]
    speed_v = [m.drive.speed for _, m in data[t_cmd]]
    steer_v = [m.drive.steering_angle for _, m in data[t_cmd]]

    join_tol_ns = int(args.join_tol_ms * 1e6)

    speed_res = classify_channel(
        cmd_t, speed_v,
        [t for t, _ in data[t_erpm]], [m.data for _, m in data[t_erpm]],
        args.speed_gain, args.speed_offset,
        tol=args.erpm_tol,
        join_tol_ns=join_tol_ns,
        rate_limit=args.max_accel * args.speed_gain if args.max_accel > 0 else None,
        bounds=None,
    )
    steer_res = classify_channel(
        cmd_t, steer_v,
        [t for t, _ in data[t_servo]], [m.data for _, m in data[t_servo]],
        args.steer_gain, args.steer_offset,
        tol=args.servo_tol,
        join_tol_ns=join_tol_ns,
        rate_limit=(args.max_steering_rate * abs(args.steer_gain)
                    if args.max_steering_rate > 0 else None),
        bounds=(args.servo_min, args.servo_max),
    )

    # How often the affine result would have needed clamping at all -- says
    # whether the recorded runs actually exercised the servo bounds.
    raw = [args.steer_gain * s + args.steer_offset for s in steer_v]
    out_of_bounds = sum(1 for r in raw if r < args.servo_min or r > args.servo_max)

    return {
        "bag": Path(bag).name,
        "n_cmd": len(cmd_t),
        "speed": speed_res,
        "steer": steer_res,
        "steer_cmd_range": [round(min(steer_v), 4), round(max(steer_v), 4)],
        "speed_cmd_range": [round(min(speed_v), 4), round(max(speed_v), 4)],
        "affine_out_of_servo_bounds_frac": out_of_bounds / len(raw) if raw else 0.0,
    }


def report(res: dict) -> None:
    print(f"\n=== {res['bag']}  ({res['n_cmd']} ackermann_cmd msgs)")
    print(f"    steering command range {res['steer_cmd_range']} rad, "
          f"speed command range {res['speed_cmd_range']} m/s")
    print(f"    affine servo result outside bounds on "
          f"{res['affine_out_of_servo_bounds_frac']*100:.2f}% of commands")
    for label, key, unit in (("speed -> erpm ", "speed", "erpm"),
                             ("steer -> servo", "steer", "servo")):
        r = res[key]
        c = r["counts"]
        print(f"    {label}: {r['match_frac']*100:6.2f}% match, "
              f"{r['explained_frac']*100:6.2f}% explained "
              f"(match {c['match']}, beyond_bounds {c['beyond_bounds']}, "
              f"transient {c['transient']}, mismatch {c['mismatch']}, "
              f"unjoined {c['unjoined']})")
        print(f"        |residual| p50 {r['resid_p50']:.5g} "
              f"p99 {r['resid_p99']:.5g} max {r['resid_max']:.5g} {unit}"
              f"   join offset p50 {r['offset_ms_p50']:.2f} ms "
              f"p99 {r['offset_ms_p99']:.2f} ms")
        for ex in r["examples"]:
            print(f"        mismatch @ {ex['t_s']}s: cmd {ex['cmd']} -> "
                  f"predicted {ex['predicted']}, observed {ex['observed']} "
                  f"(residual {ex['residual']})")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--ns", default="/gosling1", help="topic namespace prefix")
    ap.add_argument("--speed-gain", type=float, default=DEF_SPEED_GAIN)
    ap.add_argument("--speed-offset", type=float, default=DEF_SPEED_OFFSET)
    ap.add_argument("--steer-gain", type=float, default=DEF_STEER_GAIN)
    ap.add_argument("--steer-offset", type=float, default=DEF_STEER_OFFSET)
    ap.add_argument("--servo-min", type=float, default=DEF_SERVO_MIN)
    ap.add_argument("--servo-max", type=float, default=DEF_SERVO_MAX)
    ap.add_argument("--max-accel", type=float, default=DEF_MAX_ACCEL,
                    help="driver accel limit in m/s^2; 0 disables transient class")
    ap.add_argument("--max-steering-rate", type=float, default=DEF_MAX_STEER_RATE,
                    help="driver steering rate limit in rad/s; 0 disables")
    ap.add_argument("--erpm-tol", type=float, default=1.0,
                    help="erpm residual counted as a match")
    ap.add_argument("--servo-tol", type=float, default=1e-4,
                    help="servo-unit residual counted as a match")
    ap.add_argument("--join-tol-ms", type=float, default=10.0)
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    results = [analyse(b, args.ns.rstrip("/"), args) for b in args.bags]
    if args.json:
        print(json.dumps(results, indent=2))
    else:
        for r in results:
            report(r)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
