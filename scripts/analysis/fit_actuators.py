#!/usr/bin/env python3
"""Identify this vehicle's steering map, speed map, and actuation lag from bags.

Nothing in `config/vehicle/vesc.yaml` was measured on this car -- the constants
were carried over from a different chassis. This fits them from recorded data.

What it fits, and against what:

  steering   servo position -> road-wheel angle, jointly with actuation lag,
             scored against **TF-corrected gyro-z** from `vehicle/sensors/imu/raw`
             (99 Hz, negated for the roll-180 deg `imu_link` mount) through the
             kinematic bicycle model  psi_dot = v * tan(a*(servo - s0)) / L.
             Fitting in *servo units* rather than in commanded-angle units means
             the result does not inherit the old gain/offset at all: `a` is
             rad per servo unit, so `steering_angle_to_servo_gain = 1/a` and
             `steering_angle_to_servo_offset = s0` come out directly.

  speed      measured ERPM from `vehicle/sensors/core` against an
             **ERPM-independent** ground speed (VSLAM tracking odometry, with
             `odom/rf2o` as the confirming second reference). `vehicle/vesc_odom`
             is never used: its speed is computed from ERPM and its yaw rate from
             the servo command, so it is circular on both channels.

  lag        a (pure delay, first-order time constant) grid, scored by the
             steering residual -- variable projection, so `(a, s0)` are solved in
             closed form at every grid point and only the lag is searched. The
             transport component is measured separately and directly by
             cross-correlating commanded against measured ERPM.

Samples are excluded for three reasons, each reported: servo saturation beyond
the mechanical bounds, VESC drive dropout (commanded ERPM held high while
measured ERPM collapses), and a settling window at the start of the bag.

Usage:
    python3 fit_actuators.py <bag> [<bag> ...] [--ns /gosling1] [--wheelbase 0.256]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

try:
    import numpy as np
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:  # pragma: no cover - depends on the ROS environment
    sys.exit("rosbag2_py/rclpy/numpy not importable -- source the ROS 2 overlay first")


# Inherited constants, used only to convert *back* to the commanded angle for
# reporting and for the dropout detector. Nothing fitted here depends on them.
CFG_SPEED_GAIN = 3750.0
CFG_STEER_GAIN = -1.4
CFG_STEER_OFFSET = 0.56
CFG_SERVO_MIN = 0.08
CFG_SERVO_MAX = 0.92


# --------------------------------------------------------------------------
# bag reading
# --------------------------------------------------------------------------

def storage_id_for(path: str) -> str:
    p = Path(path)
    if p.is_dir():
        if list(p.glob("*.mcap")):
            return "mcap"
        if list(p.glob("*.db3")):
            return "sqlite3"
    return ""


def stamp_ns(msg) -> int | None:
    h = getattr(msg, "header", None)
    if h is None:
        return None
    return int(h.stamp.sec) * 1_000_000_000 + int(h.stamp.nanosec)


def read_series(bag: str, extractors: dict[str, callable]) -> dict[str, tuple]:
    """Read the requested topics once, applying a per-topic extractor.

    `extractors` maps topic name -> fn(msg) -> tuple of floats. Returns
    {topic: (t_seconds ndarray, values ndarray of shape (n, k))}. Timestamps are
    header stamps where the message has a header (so cross-machine merges and
    the lag search key on sensor time, not on recorder time) and bag receive
    time otherwise -- `std_msgs/Float64` has no header.
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id=storage_id_for(bag)),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = set(extractors)
    missing = wanted - set(types)
    classes = {n: get_message(types[n]) for n in wanted if n in types}

    acc: dict[str, list] = {n: [] for n in wanted if n in types}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in acc:
            continue
        msg = deserialize_message(data, classes[topic])
        t = stamp_ns(msg)
        acc[topic].append(((t if t else t_ns) / 1e9, extractors[topic](msg)))

    out = {}
    for name, rows in acc.items():
        rows.sort(key=lambda kv: kv[0])
        if not rows:
            continue
        out[name] = (np.array([r[0] for r in rows]),
                     np.array([r[1] for r in rows], dtype=float))
    for name in missing:
        out[name] = None
    return out


def resample(t_src: np.ndarray, v_src: np.ndarray, t_dst: np.ndarray) -> np.ndarray:
    """Linear interpolation of a (possibly multi-column) series onto new times."""
    if v_src.ndim == 1:
        return np.interp(t_dst, t_src, v_src)
    return np.column_stack([np.interp(t_dst, t_src, v_src[:, i])
                            for i in range(v_src.shape[1])])


# --------------------------------------------------------------------------
# exclusion masks
# --------------------------------------------------------------------------

def dropout_mask(t_core: np.ndarray, erpm_meas: np.ndarray,
                 t_cmd: np.ndarray, erpm_cmd: np.ndarray,
                 min_cmd: float, ratio: float, margin_s: float,
                 min_run: int) -> np.ndarray:
    """True where the VESC was commanded to drive but measurably was not.

    Detection is by **ERPM divergence** -- commanded stays high while measured
    collapses -- deliberately not by zero motor current. Zero current is common
    (a few percent of a bag, including while the car is moving) and on its own
    proves nothing; divergence is the actual failure signature. A speed fit
    spanning one of these windows sees a large command producing no motion and
    would both bias the gain and manufacture a fake deadband.
    """
    cmd_on_core = np.interp(t_core, t_cmd, erpm_cmd)
    bad = (np.abs(cmd_on_core) > min_cmd) & \
          (np.abs(erpm_meas) < ratio * np.abs(cmd_on_core))

    # Require a run of consecutive samples so single-sample noise does not
    # register, then widen by a margin on each side to catch the collapse and
    # recovery ramps.
    mask = np.zeros_like(bad)
    i = 0
    while i < len(bad):
        if bad[i]:
            j = i
            while j < len(bad) and bad[j]:
                j += 1
            if j - i >= min_run:
                lo = t_core[i] - margin_s
                hi = t_core[j - 1] + margin_s
                mask |= (t_core >= lo) & (t_core <= hi)
            i = j
        else:
            i += 1
    return mask


def windows_from_mask(t: np.ndarray, mask: np.ndarray) -> list[tuple[float, float]]:
    out, i = [], 0
    while i < len(mask):
        if mask[i]:
            j = i
            while j < len(mask) and mask[j]:
                j += 1
            out.append((float(t[i]), float(t[j - 1])))
            i = j
        else:
            i += 1
    return out


# --------------------------------------------------------------------------
# steering fit: variable projection over a lag grid
# --------------------------------------------------------------------------

def apply_lag(t_uniform: np.ndarray, v: np.ndarray, delay_s: float,
              tau_s: float) -> np.ndarray:
    """Pure delay then a first-order lag, on a uniformly sampled signal."""
    dt = t_uniform[1] - t_uniform[0]
    n_shift = int(round(delay_s / dt))
    if n_shift > 0:
        v = np.concatenate([np.full(n_shift, v[0]), v[:-n_shift]])
    if tau_s > 0:
        alpha = dt / (tau_s + dt)
        out = np.empty_like(v)
        acc = v[0]
        for i, x in enumerate(v):
            acc += alpha * (x - acc)
            out[i] = acc
        return out
    return v


def solve_steering(servo: np.ndarray, v: np.ndarray, psi_dot: np.ndarray,
                   wheelbase: float, iters: int = 8) -> tuple[float, float, float]:
    """Solve psi_dot = v * tan(a*(servo - s0)) / L for (a, s0); return with RMS.

    Initialised from the small-angle form, which is linear in (a, a*s0) and so
    has a closed-form least-squares solution, then refined on the exact `tan`
    model by Gauss-Newton. At the angles this car reaches (<=0.34 rad) `tan`
    departs from identity by ~4 %, which is small enough to initialise from but
    too large to leave unmodelled in the final gain.
    """
    # Small-angle init:  psi_dot ~= (v/L) * a * servo - (v/L) * a * s0
    A = np.column_stack([v * servo / wheelbase, -v / wheelbase])
    theta, *_ = np.linalg.lstsq(A, psi_dot, rcond=None)
    a = float(theta[0])
    s0 = float(theta[1] / a) if abs(a) > 1e-12 else 0.0

    for _ in range(iters):
        u = a * (servo - s0)
        tan_u = np.tan(u)
        sec2 = 1.0 + tan_u ** 2
        pred = v * tan_u / wheelbase
        r = psi_dot - pred
        # d(pred)/da and d(pred)/ds0
        J = np.column_stack([
            v * sec2 * (servo - s0) / wheelbase,
            v * sec2 * (-a) / wheelbase,
        ])
        try:
            step, *_ = np.linalg.lstsq(J, r, rcond=None)
        except np.linalg.LinAlgError:
            break
        a += float(step[0])
        s0 += float(step[1])

    pred = v * np.tan(a * (servo - s0)) / wheelbase
    rms = float(np.sqrt(np.mean((psi_dot - pred) ** 2)))
    return a, s0, rms


def fit_steering_with_lag(t_servo, servo, t_v, v, t_psi, psi_dot, wheelbase,
                          keep_fn, delays, taus, grid_hz=200.0):
    """Search the (delay, tau) grid; at each point solve (a, s0) in closed form."""
    t0 = max(t_servo[0], t_v[0], t_psi[0])
    t1 = min(t_servo[-1], t_v[-1], t_psi[-1])
    tu = np.arange(t0, t1, 1.0 / grid_hz)
    servo_u = np.interp(tu, t_servo, servo)

    # Score on the yaw-rate samples inside the common span.
    sel = (t_psi >= t0) & (t_psi <= t1)
    t_s, psi_s = t_psi[sel], psi_dot[sel]
    v_s = np.interp(t_s, t_v, v)
    keep = keep_fn(t_s)

    best = None
    for d in delays:
        for tau in taus:
            lagged = apply_lag(tu, servo_u, d, tau)
            servo_s = np.interp(t_s, tu, lagged)
            m = keep & np.isfinite(servo_s)
            if m.sum() < 200:
                continue
            a, s0, rms = solve_steering(servo_s[m], v_s[m], psi_s[m], wheelbase)
            if best is None or rms < best["rms"]:
                best = {"delay_s": float(d), "tau_s": float(tau), "a": a,
                        "s0": s0, "rms": rms, "n": int(m.sum())}
    return best, (t_s, psi_s, v_s, keep, tu, servo_u)


# --------------------------------------------------------------------------
# transport delay, measured directly
# --------------------------------------------------------------------------

def erpm_transport_delay(t_cmd, erpm_cmd, t_meas, erpm_meas, keep_mask,
                         max_lag_s=0.4, grid_hz=200.0) -> dict:
    """Cross-correlate commanded against measured ERPM.

    LUCIO could not do this -- their bags carried no `vehicle/sensors/core`, so
    they attributed only 25 of their fitted 80 ms transport delay to known relay
    hops and left the rest unexplained. `sensors/core` at ~49.6 Hz resolves it
    directly, which says how much of the remainder is real.
    """
    t0 = max(t_cmd[0], t_meas[0])
    t1 = min(t_cmd[-1], t_meas[-1])
    tu = np.arange(t0, t1, 1.0 / grid_hz)
    c = np.interp(tu, t_cmd, erpm_cmd)
    m = np.interp(tu, t_meas, erpm_meas)
    good = keep_mask(tu)
    c = np.where(good, c, 0.0)
    m = np.where(good, m, 0.0)

    # Correlate the *derivative*, not the level. Commanded and measured ERPM
    # share a slow envelope that is near-identical at every lag, which leaves the
    # correlation curve almost flat (peak 0.9894 against 0.9873 at zero lag --
    # a peak that carries no information). Differencing suppresses the shared
    # envelope and scores the transients, which are the only part of the signal
    # that can locate a delay at all.
    c = np.diff(c, prepend=c[0])
    m = np.diff(m, prepend=m[0])
    c = c - c.mean()
    m = m - m.mean()
    if np.std(c) < 1e-9 or np.std(m) < 1e-9:
        return {"delay_s": float("nan"), "peak_corr": float("nan")}

    n_max = int(max_lag_s * grid_hz)
    lags = np.arange(0, n_max + 1)
    scores = []
    for k in lags:
        a = c[: len(c) - k] if k else c
        b = m[k:] if k else m
        denom = np.linalg.norm(a) * np.linalg.norm(b)
        scores.append(float(np.dot(a, b) / denom) if denom > 0 else 0.0)
    scores = np.array(scores)
    k_best = int(np.argmax(scores))
    # A bare "best lag" hides whether the peak is sharp or the curve is flat.
    # Report the profile so a delay that is really just noise is visible as one.
    profile = {}
    for ms in (0, 20, 40, 60, 80, 100, 150, 200):
        k = int(ms * grid_hz / 1000.0)
        if k < len(scores):
            profile[f"{ms}ms"] = round(float(scores[k]), 4)
    return {"delay_s": k_best / grid_hz, "peak_corr": float(scores[k_best]),
            "corr_at_zero": float(scores[0]), "profile": profile,
            "core_sample_period_ms": 1000.0 * float(np.median(np.diff(t_meas)))}


# --------------------------------------------------------------------------
# main analysis
# --------------------------------------------------------------------------

def analyse(bag: str, ns: str, args) -> dict:
    T_SERVO = f"{ns}/vehicle/commands/servo/position"
    T_ERPM_CMD = f"{ns}/vehicle/commands/motor/speed"
    T_CORE = f"{ns}/vehicle/sensors/core"
    T_IMU = f"{ns}/vehicle/sensors/imu/raw"
    T_VSLAM = f"{ns}/visual_slam/tracking/odometry"
    T_RF2O = f"{ns}/odom/rf2o"
    T_LOCAL = f"{ns}/odometry/local"

    series = read_series(bag, {
        T_SERVO: lambda m: (m.data,),
        T_ERPM_CMD: lambda m: (m.data,),
        T_CORE: lambda m: (m.state.speed, m.state.current_motor,
                           m.state.duty_cycle, m.state.voltage_input),
        T_IMU: lambda m: (m.angular_velocity.z,),
        T_VSLAM: lambda m: (m.twist.twist.linear.x,),
        T_RF2O: lambda m: (m.twist.twist.linear.x,),
        T_LOCAL: lambda m: (m.twist.twist.linear.x, m.twist.twist.angular.z),
    })

    for required in (T_SERVO, T_ERPM_CMD, T_CORE, T_IMU):
        if series.get(required) is None:
            raise SystemExit(f"{bag}: required topic {required} absent")

    t_servo, servo = series[T_SERVO][0], series[T_SERVO][1][:, 0]
    t_ecmd, erpm_cmd = series[T_ERPM_CMD][0], series[T_ERPM_CMD][1][:, 0]
    t_core, core = series[T_CORE]
    erpm_meas, i_motor, duty, v_in = core[:, 0], core[:, 1], core[:, 2], core[:, 3]

    # Yaw rate. The VESC IMU is mounted with roll 180 deg (see
    # static_transformations.launch.py), so raw gyro-z in `imu_link` is
    # correctly *opposite* in sign to base_link yaw rate. Negating here is
    # applying the TF, not correcting a sensor bug.
    t_imu, gz = series[T_IMU][0], -series[T_IMU][1][:, 0]

    # That negation is the single assumption every left/right conclusion below
    # rests on, and LUCIO flagged this exact sign as a possible live bug, so it
    # is checked against a source that does not depend on reading the TF:
    # `odometry/local` is the EKF output in base_link under REP-103, where
    # positive yaw rate is counter-clockwise. If the negation is right, the two
    # correlate positively at ~unit scale.
    gz_check = None
    if series.get(T_LOCAL) is not None:
        t_loc, loc = series[T_LOCAL]
        wz = loc[:, 1]
        common = (t_imu >= t_loc[0]) & (t_imu <= t_loc[-1])
        if common.sum() > 200:
            wz_i = np.interp(t_imu[common], t_loc, wz)
            g = gz[common]
            moving = np.abs(wz_i) > 0.05
            if moving.sum() > 200:
                gz_check = {
                    "corr_vs_odometry_local": float(
                        np.corrcoef(g[moving], wz_i[moving])[0, 1]),
                    "scale": float(np.dot(g[moving], wz_i[moving]) /
                                   np.dot(g[moving], g[moving])),
                    "n": int(moving.sum()),
                }

    # Gyro bias from the quietest second of the bag, estimated where the car is
    # commanded to stand still. Left unremoved it leaks straight into s0.
    still = np.abs(np.interp(t_imu, t_core, erpm_meas)) < 50.0
    gz_bias = float(np.median(gz[still])) if still.sum() > 50 else 0.0
    gz = gz - gz_bias

    # --- exclusions -------------------------------------------------------
    t_start = t_core[0] + args.settle_s

    drop = dropout_mask(t_core, erpm_meas, t_ecmd, erpm_cmd,
                        min_cmd=args.dropout_min_erpm, ratio=args.dropout_ratio,
                        margin_s=args.dropout_margin_s, min_run=args.dropout_min_run)
    drop_windows = windows_from_mask(t_core, drop)

    at_min = servo <= args.servo_min + 1e-9
    at_max = servo >= args.servo_max - 1e-9
    sat = at_min | at_max
    sat_windows = windows_from_mask(t_servo, sat)

    # Which servo bound is which turn direction is settled from the measured
    # yaw rate rather than assumed: take the median gyro-z while pinned at each
    # bound. Positive is a left (counter-clockwise) turn.
    def yaw_at(mask: np.ndarray) -> float:
        if mask.sum() < 10:
            return float("nan")
        g = np.interp(t_servo[mask], t_imu, gz)
        return float(np.median(g))

    sat_sense = {"yaw_at_servo_min": yaw_at(at_min), "yaw_at_servo_max": yaw_at(at_max)}

    def keep_at(t: np.ndarray) -> np.ndarray:
        ok = t >= t_start
        for lo, hi in drop_windows:
            ok &= ~((t >= lo) & (t <= hi))
        for lo, hi in sat_windows:
            ok &= ~((t >= lo - args.sat_margin_s) & (t <= hi + args.sat_margin_s))
        return ok

    def keep_no_sat(t: np.ndarray) -> np.ndarray:
        ok = t >= t_start
        for lo, hi in drop_windows:
            ok &= ~((t >= lo) & (t <= hi))
        return ok

    # --- ground-speed references (never ERPM-derived) ---------------------
    refs = {}
    if series.get(T_VSLAM) is not None:
        refs["vslam"] = (series[T_VSLAM][0], series[T_VSLAM][1][:, 0])
    if series.get(T_RF2O) is not None:
        refs["rf2o"] = (series[T_RF2O][0], series[T_RF2O][1][:, 0])
    if not refs:
        raise SystemExit(f"{bag}: no ERPM-independent speed reference present")

    # --- speed ------------------------------------------------------------
    # Fitted before the steering search because the steering model carries `v`
    # as a known regressor: a speed reference wrong by a factor scales `a` by
    # its inverse. The reference is therefore chosen on measured quality, not
    # assumed. VSLAM was the intended primary, but on these bags its tracking
    # odometry covers only part of the recording and its speed channel fits
    # ERPM at R2 0.01-0.76 against rf2o's 0.96, so rf2o wins on the data.
    speed_fits = {}
    for name, (t_r, v_r) in refs.items():
        sel = keep_no_sat(t_core)
        vv = np.interp(t_core, t_r, v_r)
        # Coverage: interpolation happily extrapolates flat beyond a reference's
        # span, which is exactly how a short VSLAM segment silently poisons a
        # whole-bag fit. Score only inside the reference's own span.
        inside = (t_core >= t_r[0]) & (t_core <= t_r[-1])
        m = sel & inside & (np.abs(vv) > args.speed_min_ref)
        if m.sum() < 100:
            continue
        A = np.column_stack([vv[m], np.ones(m.sum())])
        theta, *_ = np.linalg.lstsq(A, erpm_meas[m], rcond=None)
        pred = A @ theta
        resid = erpm_meas[m] - pred
        ss = np.sum((erpm_meas[m] - erpm_meas[m].mean()) ** 2)
        speed_fits[name] = {
            "speed_to_erpm_gain": float(theta[0]),
            "speed_to_erpm_offset": float(theta[1]),
            "rms_erpm": float(np.sqrt(np.mean(resid ** 2))),
            "r2": float(1.0 - np.sum(resid ** 2) / ss) if ss > 0 else float("nan"),
            "n": int(m.sum()),
            "coverage_frac": float(np.mean(inside)),
        }
        # Same fit with dropouts left in, so the contamination is quantified
        # rather than asserted.
        m2 = (t_core >= t_start) & inside & (np.abs(vv) > args.speed_min_ref)
        if m2.sum() > 100:
            A2 = np.column_stack([vv[m2], np.ones(m2.sum())])
            th2, *_ = np.linalg.lstsq(A2, erpm_meas[m2], rcond=None)
            speed_fits[name]["gain_with_dropouts"] = float(th2[0])

    usable = {n: f for n, f in speed_fits.items()
              if f["r2"] >= args.min_ref_r2 and f["coverage_frac"] >= args.min_ref_coverage}
    if not usable:
        raise SystemExit(
            f"{bag}: no speed reference met R2 >= {args.min_ref_r2} and coverage "
            f">= {args.min_ref_coverage}; got "
            + ", ".join(f"{n}(R2={f['r2']:.3f},cov={f['coverage_frac']:.2f})"
                        for n, f in speed_fits.items()))
    primary = args.speed_ref if args.speed_ref in usable else \
        max(usable, key=lambda n: usable[n]["r2"])

    # --- steering + lag ---------------------------------------------------
    delays = np.arange(0.0, args.max_delay_s + 1e-9, args.delay_step_s)
    taus = np.arange(0.0, args.max_tau_s + 1e-9, args.tau_step_s)
    t_v, v_ref = refs[primary]

    # Holdout scoring: with the lag and (a, s0) pinned from another bag, the
    # "search" collapses to a single evaluation, which is exactly a prediction
    # on held-out data. Reported as `held_out: true` so a scored run is never
    # mistaken for a fitted one.
    held_out = args.fix_a is not None
    if held_out:
        delays, taus = [args.fix_delay], [args.fix_tau]

    steer_best, ctx = fit_steering_with_lag(
        t_servo, servo, t_v, v_ref, t_imu, gz, args.wheelbase, keep_at, delays, taus)
    if steer_best is None:
        raise SystemExit(f"{bag}: too few usable samples for the steering fit")

    if held_out:
        t_s0_, psi_s0_, v_s0_, keep0_, tu0_, servo_u0_ = ctx
        lagged0_ = apply_lag(tu0_, servo_u0_, args.fix_delay, args.fix_tau)
        servo_h = np.interp(t_s0_, tu0_, lagged0_)[keep0_]
        pred_h = (v_s0_[keep0_] * np.tan(args.fix_a * (servo_h - args.fix_s0))
                  / args.wheelbase)
        rms_h = float(np.sqrt(np.mean((psi_s0_[keep0_] - pred_h) ** 2)))
        steer_best = {"delay_s": args.fix_delay, "tau_s": args.fix_tau,
                      "a": args.fix_a, "s0": args.fix_s0, "rms": rms_h,
                      "n": int(keep0_.sum()), "held_out": True,
                      "rms_if_refitted": steer_best["rms"],
                      "a_if_refitted": steer_best["a"]}

    # Same fit with no lag at all, to show what the lag search actually bought.
    zero_lag, _ = fit_steering_with_lag(
        t_servo, servo, t_v, v_ref, t_imu, gz, args.wheelbase, keep_at, [0.0], [0.0])

    # Left/right split -- toe and linkage geometry make asymmetry plausible.
    t_s, psi_s, v_s, keep, tu, servo_u = ctx
    lagged = apply_lag(tu, servo_u, steer_best["delay_s"], steer_best["tau_s"])
    servo_s = np.interp(t_s, tu, lagged)
    s0 = steer_best["s0"]
    # A side with too few samples is reported as skipped rather than omitted --
    # silence would read as "no asymmetry" when it means "this bag never turned
    # that way". `loop3x` is three laps in one direction and genuinely cannot
    # answer the asymmetry question; `figure8` is the bag that can.
    # Label sides by the sign of the *implied steering angle*, not by the servo
    # direction. `a` is negative, so a servo value above s0 is a negative
    # (right-hand, clockwise) steering angle -- labelling by servo direction
    # gets left and right exactly backwards.
    delta_s = steer_best["a"] * (servo_s - s0)
    sides = {}
    for name, sel in (("left", delta_s > args.side_deadzone * abs(steer_best["a"])),
                      ("right", delta_s < -args.side_deadzone * abs(steer_best["a"]))):
        m = keep & sel
        if m.sum() < args.min_side_samples:
            sides[name] = {"skipped": True, "n": int(m.sum())}
            continue
        a_i, s0_i, rms_i = solve_steering(servo_s[m], v_s[m], psi_s[m],
                                          args.wheelbase)
        sides[name] = {"a": a_i, "s0": s0_i, "rms": rms_i, "n": int(m.sum())}

    # Wheelbase sensitivity: `a` scales with L, so report the alternative
    # rather than pretending the measured value is settled.
    alt_L = 0.25 if abs(args.wheelbase - 0.256) < 1e-6 else 0.256
    a_alt, s0_alt, _ = solve_steering(servo_s[keep], v_s[keep], psi_s[keep], alt_L)

    # --- transport delay --------------------------------------------------
    transport = erpm_transport_delay(t_ecmd, erpm_cmd, t_core, erpm_meas, keep_no_sat)

    n_core_dropped = int(drop.sum())
    n_sat = int(sat.sum())
    return {
        "bag": Path(bag).name,
        "duration_s": round(float(t_core[-1] - t_core[0]), 2),
        "wheelbase_m": args.wheelbase,
        "gyro_bias_rad_s": gz_bias,
        "gyro_sign_check": gz_check,
        "exclusions": {
            "settle_s": args.settle_s,
            "dropout_windows": len(drop_windows),
            "dropout_core_frac": n_core_dropped / len(t_core),
            "dropout_longest_s": max((hi - lo for lo, hi in drop_windows), default=0.0),
            "servo_saturated_frac": n_sat / len(servo),
            "servo_at_min_frac": float(np.mean(at_min)),
            "servo_at_max_frac": float(np.mean(at_max)),
            **sat_sense,
        },
        "steering": {
            "primary_speed_ref": primary,
            "best": steer_best,
            "zero_lag": zero_lag,
            "sides": sides,
            "wheelbase_sensitivity": {"L": alt_L, "a": a_alt, "s0": s0_alt},
            "implied": {
                "steering_angle_to_servo_gain": 1.0 / steer_best["a"],
                "steering_angle_to_servo_offset": steer_best["s0"],
                "k_vs_configured": steer_best["a"] / (1.0 / CFG_STEER_GAIN),
            },
        },
        "speed": speed_fits,
        "transport_delay": transport,
        "electrical": {
            "zero_current_frac": float(np.mean(np.abs(i_motor) < 1e-6)),
            "voltage_input_min": float(np.min(v_in)),
            "voltage_input_max": float(np.max(v_in)),
        },
    }


def report(r: dict) -> None:
    e = r["exclusions"]
    s = r["steering"]
    b = s["best"]
    print(f"\n=== {r['bag']}   {r['duration_s']} s   L = {r['wheelbase_m']} m")
    print(f"  gyro-z bias removed: {r['gyro_bias_rad_s']:+.5f} rad/s")
    gc = r.get("gyro_sign_check")
    if gc:
        print(f"  gyro-z sign check vs odometry/local: corr {gc['corr_vs_odometry_local']:+.4f}, "
              f"scale {gc['scale']:+.3f} (n={gc['n']}) -- positive corr confirms the "
              f"roll-180 negation")
    print(f"  excluded: first {e['settle_s']} s; "
          f"{e['dropout_windows']} dropout windows "
          f"({e['dropout_core_frac']*100:.2f}% of core, longest "
          f"{e['dropout_longest_s']:.2f} s); "
          f"servo saturated {e['servo_saturated_frac']*100:.2f}% "
          f"(at servo_min {e['servo_at_min_frac']*100:.2f}%, "
          f"at servo_max {e['servo_at_max_frac']*100:.2f}%)")
    print(f"    bound sense from measured yaw: servo_min -> "
          f"{e['yaw_at_servo_min']:+.3f} rad/s, servo_max -> "
          f"{e['yaw_at_servo_max']:+.3f} rad/s  (positive = left/CCW)")

    print(f"  STEERING  (yaw ref: gyro-z, speed ref: {s['primary_speed_ref']})")
    if b.get("held_out"):
        print(f"    HELD OUT: parameters pinned from another bag, not fitted here")
        print(f"    prediction RMS {b['rms']:.5f} rad/s vs "
              f"{b['rms_if_refitted']:.5f} if refitted on this bag "
              f"(a would have been {b['a_if_refitted']:.4f})")
    print(f"    best lag: delay {b['delay_s']*1000:5.0f} ms, "
          f"tau {b['tau_s']*1000:5.0f} ms   RMS {b['rms']:.5f} rad/s  (n={b['n']})")
    z = s["zero_lag"]
    print(f"    no-lag baseline RMS {z['rms']:.5f} rad/s "
          f"({(1 - b['rms']/z['rms'])*100:+.1f}% from modelling lag)")
    print(f"    a  = {b['a']:.4f} rad per servo unit    s0 = {b['s0']:.4f} servo")
    imp = s["implied"]
    print(f"    -> steering_angle_to_servo_gain   {imp['steering_angle_to_servo_gain']:+.4f} "
          f"(configured {CFG_STEER_GAIN})")
    print(f"    -> steering_angle_to_servo_offset {imp['steering_angle_to_servo_offset']:.4f} "
          f"(configured {CFG_STEER_OFFSET})")
    print(f"    -> k vs configured (achieved/commanded) {imp['k_vs_configured']:.4f}")
    for side, d in s["sides"].items():
        if d.get("skipped"):
            print(f"    {side:5s}: skipped, only {d['n']} samples this side")
        else:
            print(f"    {side:5s}: a = {d['a']:.4f}  s0 = {d['s0']:.4f}  "
                  f"RMS {d['rms']:.5f}  (n={d['n']})")
    w = s["wheelbase_sensitivity"]
    print(f"    at L = {w['L']} m instead: a = {w['a']:.4f}, s0 = {w['s0']:.4f}")

    print("  SPEED  (measured ERPM vs ERPM-independent ground speed)")
    for name, f in r["speed"].items():
        mark = "*" if name == r["steering"]["primary_speed_ref"] else " "
        print(f"   {mark}{name:6s}: speed_to_erpm_gain {f['speed_to_erpm_gain']:9.1f}  "
              f"offset {f['speed_to_erpm_offset']:+8.1f}  "
              f"R2 {f['r2']:.4f}  cov {f['coverage_frac']*100:5.1f}%  "
              f"RMS {f['rms_erpm']:.0f} erpm  (n={f['n']})"
              + (f"   [with dropouts: {f['gain_with_dropouts']:.1f}]"
                 if "gain_with_dropouts" in f else ""))
    td = r["transport_delay"]
    print(f"  TRANSPORT DELAY  commanded ERPM -> measured ERPM: "
          f"{td['delay_s']*1000:.0f} ms  (peak corr {td['peak_corr']:.4f}; "
          f"sensors/core samples every {td['core_sample_period_ms']:.1f} ms, "
          f"which bounds the resolution)")
    print("    corr vs lag: " + "  ".join(f"{k}={v}" for k, v in td["profile"].items()))
    el = r["electrical"]
    print(f"  electrical: zero motor current on {el['zero_current_frac']*100:.2f}% "
          f"of samples; voltage_input {el['voltage_input_min']:.1f}-"
          f"{el['voltage_input_max']:.1f} V")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--ns", default="/gosling1")
    ap.add_argument("--wheelbase", type=float, default=0.256,
                    help="metres, rear axle to front axle")
    ap.add_argument("--servo-min", type=float, default=CFG_SERVO_MIN)
    ap.add_argument("--servo-max", type=float, default=CFG_SERVO_MAX)
    ap.add_argument("--settle-s", type=float, default=5.0)
    ap.add_argument("--sat-margin-s", type=float, default=0.05)
    ap.add_argument("--side-deadzone", type=float, default=0.02,
                    help="servo units either side of s0 excluded from the L/R split")
    ap.add_argument("--speed-min-ref", type=float, default=0.15,
                    help="m/s below which the ground-speed reference is not trusted")
    ap.add_argument("--speed-ref", default=None, choices=["rf2o", "vslam"],
                    help="force the speed reference; default picks the best "
                         "one that clears --min-ref-r2 and --min-ref-coverage")
    ap.add_argument("--min-ref-r2", type=float, default=0.8)
    ap.add_argument("--min-ref-coverage", type=float, default=0.8)
    ap.add_argument("--min-side-samples", type=int, default=200)
    ap.add_argument("--fix-a", type=float, default=None,
                    help="score with this steering slope instead of fitting it; "
                         "with --fix-s0/--fix-delay/--fix-tau this is a holdout "
                         "prediction on the given bags")
    ap.add_argument("--fix-s0", type=float, default=0.0)
    ap.add_argument("--fix-delay", type=float, default=0.0)
    ap.add_argument("--fix-tau", type=float, default=0.0)
    ap.add_argument("--dropout-min-erpm", type=float, default=800.0)
    ap.add_argument("--dropout-ratio", type=float, default=0.3)
    ap.add_argument("--dropout-margin-s", type=float, default=0.2)
    ap.add_argument("--dropout-min-run", type=int, default=3)
    ap.add_argument("--max-delay-s", type=float, default=0.30)
    ap.add_argument("--delay-step-s", type=float, default=0.01)
    ap.add_argument("--max-tau-s", type=float, default=0.20)
    ap.add_argument("--tau-step-s", type=float, default=0.02)
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
