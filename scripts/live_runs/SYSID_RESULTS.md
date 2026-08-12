# Steering, speed, and lag identification — offline results

Stages 1 and 2 of the actuator system-identification exercise, run entirely on
archived bags. Stage 3 is **partially** applied: the steering gain only.
Stage 0 (the bench sweep) still needs the car.

## Bags

Two families exist on gosling1 and they are **not** the same recordings.

`/mnt/f1tenth_ssd/shared_dir/bags/20260805/` — the full-length 2026-08-05
human-driven session. These are the primary set, and almost certainly what the
LUCIO briefing was fitted against:

| bag | duration | servo saturation | usable steering samples |
|---|---|---|---|
| `mapping_drive_170025` | 146.8 s | **0 %** | 14153 (L 2100 / R 6276) |
| `figure8_172338` | 155.1 s | **0 %** | 14985 (L 2586 / R 2188) |
| `loop_laps_173558` | 103.0 s | **0 %** | 9746 (L 323 / R 4639) |
| `mapping_drive_145639` | 171.5 s | — | fit declined, see below |

`/mnt/f1tenth_ssd/shared_dir/rosbags/*_with_localization_and_pointcloud` —
shorter excerpts, and **worse data**: 17–20 % of their samples sit pinned at the
right-hand servo clamp, and their battery is a volt lower (11.8–12.3 V against
12.5–12.8 V):

| bag | duration | servo saturation | usable steering samples |
|---|---|---|---|
| `figure8_with_localization_and_pointcloud` | 64.6 s | 20.2 % | 4379 (L 1845 / R 826) |
| `loop3x_with_localization_and_pointcloud` | 78.3 s | 17.1 % | 5520 (L 109 / R 2984) |

**Conclusions below come from the first family.** The second is reported because
it was analysed first and because the disagreement between them is itself
informative.

## Tools

`scripts/analysis/check_driver_transform.py` (Stage 1) and
`scripts/analysis/fit_actuators.py` (Stage 2), both `--ns /gosling1`. Run inside
the container with **both** overlays sourced — `vesc_msgs` is only in the second:

```
source /opt/ros/humble/setup.bash && source /workspaces/f1tenth/install/setup.bash
python3 fit_actuators.py /mnt/shared_dir/bags/20260805/figure8_172338
```

Bags are at `/mnt/shared_dir/...` *inside* the container, not at the host path.

---

## Stage 1 — the driver applied exactly the affine map it was configured with

| channel | figure8 (short) | loop3x (short) |
|---|---|---|
| `speed` → `commands/motor/speed` | 99.99 % exact | 99.92 % exact |
| `steering_angle` → `commands/servo/position` | 99.95 % exact | 99.73 % exact |

Residual p50 is **exactly 0** on both channels. The few non-matching samples are
all a single command step (`±29.709` erpm, `±0.00377` servo) — a join artifact
between two ~115 Hz streams, not a driver deviation.

Three consequences, two of which change the plan:

- **`ackermann_to_vesc` does not clamp.** 20.2 % / 17.1 % of published
  `commands/servo/position` values were *outside* `[0.08, 0.92]` and the driver
  published them unclamped — the clamp really is downstream in `vesc_driver`.
- **The rate limiter never engaged** (one sample per bag). The plan's central
  caution — that `ackermann_cmd` is not what the actuator received — is true in
  principle but did not bite: the two are interchangeable on this data.
- Therefore the 2026-08-06 `stackside_*` bag, which lacks `commands/*`, **can**
  be reconstructed from `ackermann_cmd` if needed.

## Stage 2 — steering

Fitted in **servo units** against TF-corrected gyro-z through
`ψ̇ = v·tan(a·(servo − s₀))/L`, so nothing is inherited from the old constants.
`L = 0.256 m`. Speed reference chosen per bag on measured R² and coverage.

| bag | `a` (rad/servo) | ⇒ gain `1/a` | `s₀` | `k` | delay | τ | RMS |
|---|---|---|---|---|---|---|---|
| **`mapping_drive_170025`** | −0.8783 | **−1.1386** | **0.5591** | 1.2296 | 60 ms | 40 ms | 0.0333 |
| **`figure8_172338`** | −0.8688 | **−1.1510** | **0.5610** | 1.2163 | 60 ms | 40 ms | 0.0358 |
| `loop_laps_173558` | −0.8044 | −1.2431 | 0.5517 | 1.1262 | 70 ms | 20 ms | 0.0301 |
| `figure8_…_pointcloud` | −0.8408 | −1.1894 | 0.5419 | 1.1771 | 60 ms | 40 ms | 0.0600 |
| `loop3x_…_pointcloud` | −0.8709 | −1.1482 | 0.5487 | 1.2193 | 50 ms | 40 ms | 0.0449 |

Configured before this work: gain **−1.4**, offset **0.56**.

### Applied

```
steering_angle_to_servo_gain:   -1.1448    (was -1.4)
steering_angle_to_servo_offset:  0.5600    (unchanged)
```

Averaged over the two bags that both **turn both directions** and **never touch
the servo clamp** — `mapping_drive_170025` and `figure8_172338`. They agree to
1.1 %. `loop_laps_173558` is excluded as an input because it is effectively
one-sided (323 left samples against 4639 right); it is the spread bound.

- **The gain was wrong by ~18–23 %.** `k` runs 1.216–1.230 on the two clean
  two-directional bags. The LUCIO side independently got **1.177** from
  different code, a different yaw reference (localizer pose vs gyro), and a
  different regressor (commanded angle vs servo output).
- **The offset was right all along.** Measured 0.5591 / 0.5610 against the
  configured 0.5600. This corrects an earlier conclusion in this document: the
  short bags gave 0.5419 and that value was briefly applied. It was biased by
  those bags spending 17–20 % of their samples pinned at the right-hand clamp.
  The clean bags do not saturate at all, and they land on 0.56.
- **Modelling the lag matters**: it cuts steering residual 47–65 % versus none.
- **Cross-bag holdout passes** (on the short pair): pinning loop3x's
  `(a, s₀, delay, τ)` and predicting figure8 gives RMS 0.0690 rad/s against
  0.0601 refitted — 15 % worse, on a different trajectory shape.
- **Wheelbase sensitivity**: `a` scales with `L`. At 0.25 m instead of 0.256 m,
  `figure8_172338`'s `a` moves −0.8688 → −0.8495 (2.2 %); `s₀` is unaffected.

### Left/right asymmetry

| bag | left `a` (n) | right `a` (n) | asymmetry |
|---|---|---|---|
| `mapping_drive_170025` | −0.9114 (2100) | −0.8618 (6276) | 5.8 % |
| `figure8_172338` | −0.8709 (2586) | −0.8248 (2188) | 5.6 % |
| `figure8_…_pointcloud` | −0.7957 (1845) | −0.8325 (826) | −4.4 % |

The two clean bags agree: **~5.7 % more rad-per-servo-unit to the left**. The
short saturated bag disagrees in *sign*, which is what 20 % right-clamp
saturation does to a two-sided fit — another reason to prefer the long bags.

Stage 0's bench sweep is what decomposes this into linkage geometry, toe, and
tire slip. It cannot be done from bags.

### Which servo bound is which direction — measured

From the short bags (the only ones that ever reach a bound), taking median
measured yaw rate while pinned there:

| bound | measured yaw | direction |
|---|---|---|
| `servo_max` = 0.92 | −0.862 / −0.823 rad/s | **right** (clockwise) |
| `servo_min` = 0.08 | never reached in any of the six bags | — |

So **`servo_max` is the right-hand lock, and right is the limited direction.**
With the applied calibration the range is **+24.0° left / −18.0° right**, and
`max_steering:=0.34` (19.5°) still clips right (servo 0.9492) but no longer
clips left (servo 0.1708).

The package's architecture notes previously stated this range as "[−0.257 rad
left, +0.343 rad right]" with "full-left … clips at 0.92". Both had left and
right transposed — under REP-103 positive `steering_angle` is *left*, and the
negative gain maps positive angles to the *low* servo end. Now corrected.

### The gyro sign question LUCIO raised is answered

`imu_link` is mounted with roll 180°, so raw `gz` is correctly opposite in sign
to base_link yaw rate. Rather than rest the left/right result on reading the TF,
the negation was checked against `odometry/local` (EKF output in base_link,
REP-103):

| bag | correlation | scale |
|---|---|---|
| `figure8_172338` | **+0.996** | 0.987 |
| `mapping_drive_170025` | **+0.989** | 0.971 |
| `loop_laps_173558` | **+0.979** | 0.974 |

Positive at unit scale on all three — which also **reproduces LUCIO's |scale|
1.00**, from the opposite side of the sign. The negation is right, and this is a
**consumer-side bug, not a sensor bug**: anyone reading `gz` without applying
the TF gets it backwards. Our EKF is unaffected (`ekf_odom.yaml` fuses no VESC
angular rates).

(The short bags gave a much weaker +0.65 / +0.21 at scale 1.5–1.8. That was an
artifact of their length and saturation, not a real disagreement.)

## Stage 2 — speed

Measured ERPM from `sensors/core` against ERPM-independent ground speed:

| bag | ref | gain | offset | R² |
|---|---|---|---|---|
| `figure8_172338` | rf2o | 4022.1 | +32.5 | 0.987 |
| `figure8_172338` | vslam | 3830.1 | +29.1 | 0.986 |
| `loop_laps_173558` | vslam | 3807.9 | +19.9 | 0.988 |
| `loop_laps_173558` | rf2o | 3967.9 | +90.1 | 0.964 |
| `mapping_drive_170025` | rf2o | 3757.8 | +158.9 | 0.947 |
| `figure8_…_pointcloud` | rf2o | 3838.2 | +181.2 | 0.964 |
| `loop3x_…_pointcloud` | rf2o | 3687.2 | +249.4 | 0.961 |

**`speed_to_erpm_gain: 3750.0` is left unchanged.** The measured spread is
3687–4022, and 3750 sits at its low end — within ~7 % of the best-conditioned
estimates. That is corroboration, not confirmation to three digits: the two
references disagree by 5 % on the *same* bag (`figure8_172338`: 4022 vs 3830),
which bounds how well this can be resolved from incidental driving. The Stage 4b
ERPM staircase measures it deliberately and should settle it.

Note this does **not** support the MPC side's reported 8–19 % speed-gain error;
see the open question below. The LUCIO side measured the same thing
independently on 2026-08-08 and puts the honest gain at **~3950** — inside this
range, in its upper half. Still not a reason to change the config before the
staircase; see the resolution below.

Two methodology notes:

- **VSLAM is not reliably usable as the speed reference.** On
  `figure8_…_pointcloud` its tracking odometry covers ~10 s of a 65 s bag
  (299 messages) and fits ERPM at R² 0.012; the first run of this analysis
  extrapolated it flat and produced a **sign-flipped** steering gain from 477
  one-sided samples. On the long bags it is fine (R² 0.986–0.988). The script
  now selects on measured R² and coverage and refuses to fit if neither clears
  the bar.
- **Drive dropouts were excluded but were not contaminating anything.** Zero
  windows in `mapping_drive_170025` and `figure8_172338`, one in
  `loop_laps_173558` (0.45 %, longest 0.44 s). In the short bags, including them
  moves the gain by 0.4 % and 0.14 %. The exclusion is right in principle and
  immaterial in practice on this data — which is the measurement the plan asked
  for, rather than an assumption.

## Stage 2 — transport delay: partially resolved

Cross-correlating commanded against measured ERPM, on the **differenced**
signals (raw levels share a slow envelope that leaves the correlation flat).

| bag | best lag | corr @0 ms | @20 ms | verdict |
|---|---|---|---|---|
| `loop_laps_173558` | 0 ms | **0.557** | 0.101 | sharp — resolved |
| `loop3x_…_pointcloud` | 0 ms | **0.570** | 0.018 | sharp — resolved |
| `figure8_172338` | 15 ms | 0.171 | 0.200 | too flat to resolve |
| `figure8_…_pointcloud` | 0 ms | 0.121 | 0.092 | weak |
| `mapping_drive_170025` | 145 ms | 0.078 | 0.059 | noise, not a delay |

**Where the correlation is strong enough to resolve anything, the delay is
0 ms — below the 20 ms `sensors/core` sampling period.** Where it is weak, the
"best lag" is noise and should not be read as a measurement; the 145 ms on
`mapping_drive_170025` at correlation 0.078 is exactly that.

So: the throttle path contributes **< 20 ms**, established on two bags, and
unresolved on three others. LUCIO could not run this test at all — their bags
carried no `sensors/core` — and attributed only 25 ms of their fitted 80 ms
transport delay to known relay hops. This says the remainder is not in the motor
command path. A deliberate step input (Stage 4b) would settle it on every bag
rather than on the two that happened to have sharp enough transients.

## `mapping_drive_145639` — the car never moved

The script refused this bag, and re-running it with the quality thresholds set
to zero still refused: fewer than 100 usable samples existed at all. Direct
inspection says why —

| signal | over the full 171.5 s |
|---|---|
| `commands/motor/speed` | −3750 … +3750 erpm (i.e. ±1.0 m/s commanded) |
| `sensors/core` measured ERPM | median **0**, max 2512 |
| `odom/rf2o` forward speed | −0.015 … **+0.014 m/s** |
| `visual_slam` forward speed | −0.005 … **+0.004 m/s** |

The motor turned, but **the vehicle did not move** — two independent ground
references agree to within 1.5 cm/s across nearly three minutes. That is the
signature of a run with the wheels off the ground, or an attempt aborted before
the car was set down. It carries no ground-speed information and cannot
contribute to any fit here.

Worth noting as a *positive* result for the tooling: this is the second time the
reference-quality gate caught a bag that would otherwise have silently poisoned
a fit. It is **excluded with a reason**, not silently dropped.

## Deadband table, converted to ERPM

The plan requires these in ERPM, since they were measured as commanded `speed`
and would go stale if the speed gain changed. At the 3750 erpm/(m/s) they were
commanded through:

| source | rig | breakaway | dropout |
|---|---|---|---|
| MPC ladder, forward | stands | 0.18 m/s → **675 erpm** | 0.12 m/s → **450 erpm** |
| MPC ladder, reverse | stands | 0.15 m/s → **562 erpm** | 0.15 m/s → **562 erpm** |
| dropout-recovery, where drive resumes | ground | 0.20–0.26 m/s → **750–975 erpm** | — |
| earlier deadband table | ground | ~0.2 m/s → **~750 erpm** | — |

Nothing moves below 0.06 m/s → **225 erpm**, either direction.

**Stage 3b item 1 — confirmed.** In the on-robot checkout
(`~/f1tenth_ws/src/f1tenth_system/vesc/vesc_ackermann/src/ackermann_to_vesc.cpp:71`):

```cpp
erpm_msg.data = speed_to_erpm_gain_ * cmd->drive.speed + speed_to_erpm_offset_;
```

Applied **unconditionally** — no sign-aware or dead-at-zero path anywhere in the
file. A nonzero `speed_to_erpm_offset` makes a commanded 0.0 m/s produce
breakaway ERPM and the car creeps whenever asked to stop. It must **not** be
used as deadband compensation. (Checked against the robot's working copy, not
`privvyledge/vesc@humble-devel` HEAD — re-confirm upstream before any brief.)

**Stage 3b item 2 — confirmed arithmetically.** The MPC's quoted ceiling from
rest, `0 + 3.0 × 0.05 = 0.15 m/s`, is **562 erpm**, below the 750–975 erpm
ground breakaway. Item 3 — checking that 3.0/0.05 against the MPC's actual
configuration rather than second-hand — is **not done**.

**The "10–19 % faster than commanded" claim is dead, and it was never LUCIO's**
(resolved 2026-08-08 by `RESPONSE_f1tenth_sysid_round2.md`). It reached this
work second-hand from the MPC side, and an earlier draft of `LUCIO_REPLY.md`
put the question to LUCIO — that was a misattribution. LUCIO cannot find the
claim in any of their documents and declines to defend it; the only speed
statement they have made says the opposite. **Do not carry it as a LUCIO
finding.**

They measured it anyway, on the same three long bags, six references (two
ERPM-derived, four not), and it fails three independent ways:

1. **The 0.08–0.21 m/s band does not exist in this data** — 0.5–1.2 % of
   command samples, 0.8–1.7 s equivalent per run, all of it the stick in
   transit. Consistent with the 0.20–0.26 m/s ground breakaway measured here:
   the band sits almost entirely below the speed at which this car moves at all.
2. **The sign is opposite.** Every ERPM-independent reference (`pose_map`,
   two camera pixel→world tracks, `odometry/local`) reads **3–11 % slower** than
   commanded, on every run, at every speed with data.
3. **The ERPM signals cannot produce it either** — `sensors/core` and
   `vesc_odom` track the command to 0.4 %, which is expected: the VESC closes
   its loop in ERPM and `ackermann_to_vesc` is a transparent affine map, so
   measuring speed there is very nearly measuring the command back.

Useful by-product for the gain: if true ground speed is ~5 % below `erpm/3750`,
the honest gain is `3750 / 0.95 ≈ 3950`, inside the 3687–4022 measured here.
Note their `odometry/local` reads 4.4–6.2 % slow *despite* fusing ERPM, because
rf2o and VSLAM are ground-referenced and pull it down — a second,
differently-constructed ERPM-independent vote for the same direction. Both
methods are soft in magnitude, so **the config still does not move before the
Stage 4b staircase**; expect it to move up rather than down when it does.

---

## Settled, and not

Settled by these bags:

- steering gain wrong by ~18–23 %; applied **−1.1448**, was −1.4
- steering offset **0.56 is correct** — measured 0.5591 / 0.5610
- speed gain **3687–4022**, so 3750 stands, but only to ~7 %
- actuation lag ≈ **60 ms delay + 40 ms first-order**
- throttle transport delay **< 20 ms** (on the two bags that can resolve it)
- left/right asymmetry ≈ **5.7 %, more authority left**
- `gz` sign is a consumer-side TF issue, not a sensor bug — corr +0.98…+0.996
  at unit scale against `odometry/local`
- `servo_max` is the **right** lock; right is the limited direction

Not settleable from archived data:

- **the wheelbase** — 0.25 vs 0.256 unmeasured; `a` scales with it
- **the real mechanical servo travel** — `[0.08, 0.92]` is still inherited, and
  `servo_min` was never reached in **any** of six bags, so there is no evidence
  whatsoever about the left bound
- **toe**, and how the 5.7 % asymmetry splits into geometry vs slip
- **ground breakaway as a deliberate measurement** rather than inferred from
  where drive resumes after a dropout
- **the speed gain to better than ~7 %**, and the transport delay on bags
  without sharp throttle transients — both need Stage 4b's structured excitation

## The acceptance test is `k ≈ 0.96`, not `k → 1.0`

Agreed with LUCIO 2026-08-08 and it changes how Stage 4a is read. Their fit
gave `k = 1.177`; the applied correction was `1.4 / 1.1448 = 1.223`. If both
fits are sound their post-recalibration number lands at
`1.177 / 1.223 = 0.962`, **not** 1.000. Accepted band is **0.95–1.02**.

A systematic 0.96 is the residual reference/wheelbase difference between the
two fits, not a failed recalibration — **do not chase the last 4 %.** Any
wheelbase error is common-mode in that ratio and cancels.

It does **not** cancel in the vehicle, which is why the wheelbase is now the
top open item on both sides: both fits go through `ψ̇ = v·tan(δ)/L`, so a wrong
`L` biases the calibrated gain and the delivered physical angle is off by the
same 2.2 %. Carried through, at `L = 0.25 m` the `figure8_172338` `k` of 1.2163
becomes ~1.190 against LUCIO's 1.177 — the disagreement drops from 3.4 % to
1.1 %, inside both spreads. It is a tape measure.

On lag, LUCIO withdrew their 160 ms (80 ms transport + 80 ms first-order): it
was fitted against `pose_map` yaw rate, an EKF state carrying its own group
delay, and they are re-fitting against gyro. The 60 ms + 40 ms here was gyro
throughout, and the < 20 ms throttle figure was on differenced ERPM, so neither
carries that delay. Treat any LUCIO 160 ms in a document older than 2026-08-08
as superseded.
