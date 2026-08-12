# Notes: gyro z-bias correction for the VESC and RealSense IMUs

> **Superseded on 2026-08-09 by `docs/imu_bias_removal_spec.md`. Read that first.**
>
> The **measured constants in this document are still good** — the RealSense
> figure was reconfirmed twice more on 2026-08-09 at −0.002149 and −0.002131
> rad/s, holding within 1.2 % across four measurements.
>
> **Proposed change A below cannot work as written.** `imu_corrector` is an
> Autoware node that is not installed in any image on this robot, and both call
> sites hardcode `remove_imu_bias:='False'`. `config/filters/imu_corrector.yaml`
> is dead config: editing `angular_velocity_offset_z` there changes nothing, and
> a re-measurement would correctly show no effect. The replacement plan is
> `imu_processors/imu_bias_remover` from `ros-perception/imu_pipeline` — see the
> spec.
>
> Also settled on 2026-08-09: `imu_filter_madgwick` **passes `angular_velocity`
> through unchanged**, verified by subscribing to its input and output at once
> (identical means, identical sample counts). Nothing in the current chain
> touches the gyro rate.

**Status: measured, written down, NOT applied.** Nothing in this document has been
changed in the running configuration. `odometry/local` currently drifts −0.03 to
+0.16 °/min parked, which is healthy, and none of this should be applied without a
parked re-measurement afterwards. Recorded here so the numbers don't have to be
re-derived, and so an untested fusion change doesn't ride along with a working run.

---

## The measured constants

Stationary z-gyro bias, from `scripts/live_runs/yaw_drift.py 60`, gosling1, car
parked and powered:

| IMU | date | mean z-gyro (rad/s) | sample std | implied yaw rate |
|---|---|---|---|---|
| VESC | 2026-08-06 | 0.00434 | — | +14.9 °/min |
| VESC | 2026-08-08 A | 0.004532 | 0.00132 | +15.58 °/min |
| VESC | 2026-08-08 B | 0.004679 | 0.00153 | +16.09 °/min |
| RealSense | 2026-08-08 A | −0.002144 | 0.01193 | −7.37 °/min |
| RealSense | 2026-08-08 B | −0.002157 | 0.01049 | −7.42 °/min |

Two things stand out:

- **Both biases are repeatable.** VESC spans ±4 % across three measurements on two
  days; RealSense agrees to **0.6 %** between runs. Repeatability is what makes a
  static offset a legitimate fix rather than a curve-fit to one run.
- **The RealSense is far noisier per sample** (std 0.010–0.012 vs the VESC's
  0.0013, ~8×) while having the *more* stable mean. Noise and bias are separate
  problems; the offset addresses only the mean.

These numbers are the actual deliverable of this document. Whatever node applies
them — Autoware's `imu_corrector` now, something else later — the correction is
just "subtract this from `angular_velocity.z`".

## Where each IMU currently stands

**`imu0` = `vehicle/sensors/imu/raw` (VESC)** — `ekf_odom.yaml:176`. Yaw is
disabled and *all three* angular-rate entries are `false`. So the VESC contributes
roll/pitch only, and its −15.8 °/min orientation drift is correctly kept out of the
filter. This is the state established on 2026-08-06 and it is working.

**`imu1` = `camera/imu/filtered` (RealSense)** — `ekf_odom.yaml:222`. Orientation
is off, but the rate row (`ekf_odom.yaml:226`) is `[true, true, true]` — **`vyaw`
is enabled and being fused right now, uncorrected**, carrying the −0.00215 rad/s
bias above. The in-file note at line 231 acknowledges this and records that
VSLAM + rf2o were measured to hold it. That is true, and it is also the fragility:
the bias is only harmless while the sources that outvote it are alive.

**`imu_corrector.yaml`** currently holds offsets of ~1e-5 rad/s on all three axes —
placeholders, two orders of magnitude below either measured bias, with a comment
pointing at `data/calibration/realsense`. It is wired for the RealSense chain via
`launch/filters/imu_filter.launch.py`, not for the VESC.

## Proposed change A — correct the RealSense bias (the one that acts today)

Highest value for the least new machinery, because the fusion path already exists
and only the constant is wrong.

Set `angular_velocity_offset_z: -0.00215` in `config/filters/imu_corrector.yaml`
(sign per that node's convention — verify which way it applies before trusting it;
getting the sign backwards doubles the bias instead of removing it, and looks
superficially like the fix "not working"). Leave x/y alone; neither was measured
and `two_d_mode` ignores roll/pitch anyway.

Expected effect: `imu1`'s standing contribution to yaw goes from −7.4 °/min to
≈0, which mostly matters when VSLAM is down.

## Proposed change B — recruit the VESC gyro as a third yaw-rate source

The VESC's *orientation* is unfixable from outside: the quaternion is computed
onboard by Madgwick with no magnetometer (`vehicle/sensors/imu/mag` is 100 Hz of
identical zeros), so no external offset can touch it. That is what the
`ekf_odom.yaml:187` warning means, and it stays true.

The **raw z-gyro is a different signal** and is not covered by that warning. It is
the cleanest gyro on the vehicle (std 0.0013, ~8× quieter than the RealSense) and
it runs at 100 Hz. Bias-corrected by −0.00468 rad/s and fed in as `vyaw` only, it
would be a yaw-rate source independent of both the camera and the LiDAR — which is
exactly the redundancy missing when Isaac VSLAM aborts (roughly 1 launch in 3, no
respawn).

This needs a corrector instance on the VESC chain, which does not exist today —
`imu_filter.launch.py` only wires the RealSense one. It is the larger of the two
changes and should follow A, not accompany it.

**Do not** re-enable `imu0`'s orientation yaw as part of this. Only the rate row.

## On dropping the Autoware dependency

`imu_corrector` and `gyro_bias_estimator` are Autoware nodes and are slated to go
in a few months. That does not devalue this work, but it should shape it:

- Keep the measured constants in **this repo's** config, not in an Autoware-shaped
  file, so the replacement inherits them by reading a number rather than by
  reproducing a node.
- The correction is arithmetic — subtract a constant from one field of a
  `sensor_msgs/Imu`. Any replacement (a few lines in an existing filter, or the
  EKF itself if it ever grows a bias term) reimplements it trivially.
- Note one incompatibility if `gyro_bias_estimator` is used in the meantime:
  `config/filters/gyro_bias_estimator.yaml` sets `gyro_bias_threshold: 0.003`
  rad/s, and the **VESC's measured 0.00468 rad/s exceeds it**. The estimator would
  report the VESC as faulty rather than correcting it. Raise the threshold or skip
  the estimator for that chain.

## How to verify, whenever this is picked up

1. Baseline first: `scripts/live_runs/yaw_drift.py 60`, car parked, **twice**. One
   run cannot separate a bias from a walk that happened to land small — this is the
   same trap that made rf2o look like a calibration error until the sign flipped.
2. Apply one change at a time. A and B together are not separable after the fact.
3. Re-measure parked, twice again. Watch `EKF local (odom)` — it is currently
   −0.03 to +0.16 °/min and **must not get worse**. Improving an input while
   degrading the fused output means the change fought a source that was already
   compensating.
4. Then a moving run, since a stationary test says nothing about scale error.
5. If every source suddenly reports the same large drift, the car moved — that is
   not a fusion regression.

Related: `docs/rf2o_zero_velocity_brief.md` covers the other half of the heading
story (rf2o's stationary random walk), which no bias correction addresses.
