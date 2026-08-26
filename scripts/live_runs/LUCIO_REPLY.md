# Reply to the LUCIO briefing — actuator identification on gosling1

Drafted 2026-08-07. Everything here comes from `SYSID_RESULTS.md` in this
directory, which carries the full method and the per-bag numbers.

> **Before sending:** this was written against a summary of your asks rather
> than against the briefing document itself, so the section numbers we were
> given (§6, §9) are not cross-referenced here. Check that every question you
> raised is actually answered below, and add anything we missed.

## Short version

| your question | our answer | confidence |
|---|---|---|
| is the steering gain wrong? | **yes, by 18–23 %** — now `−1.1448`, was `−1.4` | high, two independent bags agreeing to 1.1 % |
| can you reproduce `k = 1.177`? | **yes** — we get 1.216–1.230 | high |
| is the `gz` sign a sensor bug? | **no — consumer-side TF** | high, verified against `odometry/local` |
| left/right asymmetry? | **~5.7 %, more authority left** | medium |
| does the throttle path explain your 80 ms? | **no — it contributes < 20 ms** | medium, resolvable on 2 of 5 bags |
| measured wheelbase? | **pending** — bench measurement not yet taken | — |
| post-recalibration steering limit? | **applied 2026-08-26** — `max_steering` is now 0.314 rad. *This row read "applied" when the reply was drafted 2026-08-08; the change had been decided but had not landed in the repo, and did not until 2026-08-26 (§6).* | high, derived from the calibrated gain |

## 1. The steering gain was wrong, and it is the headline

`steering_angle_to_servo_gain` was **−1.4** and is now **−1.1448**. The old
value was inherited from a different vehicle and had never been measured on this
car.

We fitted in **servo units** against TF-corrected gyro-z through
`ψ̇ = v·tan(a·(servo − s₀))/L` with `L = 0.256 m`, so nothing is inherited from
the old constants:

| bag | `a` (rad/servo) | ⇒ gain `1/a` | `s₀` | `k` |
|---|---|---|---|---|
| `mapping_drive_170025` (146.8 s) | −0.8783 | −1.1386 | 0.5591 | 1.2296 |
| `figure8_172338` (155.1 s) | −0.8688 | −1.1510 | 0.5610 | 1.2163 |
| `loop_laps_173558` (103.0 s) | −0.8044 | −1.2431 | 0.5517 | 1.1262 |

The applied value is the mean of the first two — the only bags that turn **both**
directions *and* never touch the servo clamp. They agree to 1.1 %.
`loop_laps_173558` is excluded as an input (323 left samples against 4639 right)
and used only as a spread bound.

**The offset was already correct.** Measured 0.5591 / 0.5610 against the
configured 0.5600, so it is unchanged.

Modelling the actuation lag matters: it cuts the steering residual 47–65 %
versus no lag. Best fit is **60 ms pure delay + 40 ms first-order**.

## 2. Your `k = 1.177` is reproduced

We get **1.216–1.230** on the two clean bags. That is independent agreement from
different code, a different yaw reference (we used TF-corrected gyro, you used
localizer pose), and a different regressor (we fitted servo output, you fitted
commanded angle). Both sides say the same thing: the car was over-steering by
about a fifth.

If you want to close the remaining 3–4 %, the likely cause is the wheelbase —
see §6.

## 3. The `gz` sign is a consumer-side TF issue, not a sensor bug

`imu_link` is mounted **roll 180°**, so raw `gz` from the VESC IMU is correctly
opposite in sign to base_link yaw rate. Rather than rest this on reading the TF,
we checked the negation against `odometry/local` (EKF output, base_link,
REP-103):

| bag | correlation | scale |
|---|---|---|
| `figure8_172338` | **+0.996** | 0.987 |
| `mapping_drive_170025` | **+0.989** | 0.971 |
| `loop_laps_173558` | **+0.979** | 0.974 |

Positive at unit scale on all three. This also **reproduces your |scale| ≈ 1.00**
from the opposite side of the sign — you and we are measuring the same thing and
disagreeing only about which frame it is expressed in.

**Action is on the consumer side:** anyone reading `gz` off the VESC IMU without
applying the `imu_link` → `base_link` transform gets it backwards. Our EKF is
unaffected because `ekf_odom.yaml` fuses no VESC angular rates.

One caveat worth knowing: two shorter excerpt bags gave a much weaker +0.65 /
+0.21 at scale 1.5–1.8. That was an artifact of their length and of servo
saturation, not a real disagreement — see §7.

## 4. Left/right asymmetry: ~5.7 %, more authority to the left

| bag | left `a` (n) | right `a` (n) | asymmetry |
|---|---|---|---|
| `mapping_drive_170025` | −0.9114 (2100) | −0.8618 (6276) | 5.8 % |
| `figure8_172338` | −0.8709 (2586) | −0.8248 (2188) | 5.6 % |

The limited direction is **right**. `servo_max = 0.92` is the right-hand lock —
median measured yaw rate while pinned there is −0.86 rad/s, i.e. clockwise.
`servo_min` has never been reached on this car in any of six archived bags, so
the left bound is empirically unknown.

With the new calibration the no-clip range is **+24.0° left / −18.0° right**.

We cannot yet say how much of the 5.7 % is linkage geometry, how much is toe,
and how much is tire slip — that decomposition needs a static bench measurement,
which is the pending work in §6.

## 5. The throttle path is not where your 80 ms lives

Cross-correlating commanded against measured ERPM on the **differenced** signals
(raw levels share a slow envelope that leaves the correlation flat):

| bag | best lag | corr @0 ms | @20 ms | verdict |
|---|---|---|---|---|
| `loop_laps_173558` | 0 ms | **0.557** | 0.101 | sharp — resolved |
| `loop3x_…_pointcloud` | 0 ms | **0.570** | 0.018 | sharp — resolved |
| `figure8_172338` | 15 ms | 0.171 | 0.200 | too flat to resolve |
| `figure8_…_pointcloud` | 0 ms | 0.121 | 0.092 | weak |
| `mapping_drive_170025` | 145 ms | 0.078 | 0.059 | noise, not a delay |

**Where the correlation is strong enough to resolve anything, the delay is 0 ms**
— below the 20 ms `sensors/core` sampling period. Where it is weak the "best
lag" is noise and should not be read as a measurement; the 145 ms at correlation
0.078 is exactly that.

So the throttle path contributes **< 20 ms**. You attributed 25 ms of your fitted
80 ms to known relay hops; this says the remaining ~55 ms is not in the motor
command path. We could not do better because we were fitting incidental driving —
a deliberate step input would settle it on every bag rather than on the two that
happened to have sharp transients. That is queued as Stage 4b here.

You could not run this test at all because your bags carried no `sensors/core`.
If you want to repeat it yourself, that topic is the thing to add to your
recording set.

## 6. The steering limit is set; the wheelbase is still owed

- **Post-recalibration steering limit — done (applied 2026-08-26).** `max_steering` is
  now **0.314 rad**, the largest symmetric command that never clips under
  `servo = −1.1448 · angle + 0.56` on `[0.08, 0.92]`: right lands at servo
  0.9195, just inside the bound, and left at 0.2005, well clear of it. The
  earlier 0.34 rad still clipped right (servo 0.9492 → 0.92); the 0.25 rad
  before that was derived from the old inherited −1.4 gain.

  **Correction to what we told you on 2026-08-08.** This item was reported as
  done at that time. It was not: all four launch entry points still carried
  0.34 rad, and the string `0.314` appeared nowhere in the repo. The decision
  had been taken but the edit never landed. It is applied now, at
  `bringup`/`teleop`/`mapping`/`joystick` — and, additionally, the Nav2 path's
  own saturation in `twist_to_ackermann` was raised 0.25 → 0.314 in the same
  change, so all three steering limits now agree with the measured mechanics.
  **Nothing on your side changes**: your ego-MPC publishes `drive` directly and
  goes through none of these, and its ±0.314 rad `delta_bound` was and remains
  the correct number. This is our bookkeeping error, not a wrong constraint on
  yours. What *would* oblige telling you is re-measuring the mechanical limit —
  recentring the servo horn or re-running the gain calibration — since that
  would make your box constraint stale.

  Whether to recentre the servo horn — which is what would buy back the unused
  left travel out to +0.419 rad — is a separate bench decision and is still open.
- **Measured wheelbase — still owed.** Needs a static bench session with the
  wheels off the ground, attempted twice and not completed for procedural
  reasons at the car, not technical ones. Config says 0.256 m and every
  consumer now agrees on it, but it has not been measured on this car; 0.25 m
  circulated previously. The fitted slope scales with it: at 0.25 m,
  `figure8_172338`'s `a` moves −0.8688 → −0.8495, a 2.2 % shift. `s₀` is
  unaffected. This is plausibly most of the residual gap between your 1.177 and
  our 1.216–1.230.

Everything above stands independently of the wheelbase except for that 2.2 %
sensitivity.

## 7. Method notes you should know before reusing any of this

- **Two families of bags exist on gosling1 and they are not the same
  recordings.** The full-length 2026-08-05 session (`bags/20260805/`) has 0 %
  servo saturation and 12.5–12.8 V. The shorter
  `rosbags/*_with_localization_and_pointcloud` excerpts spend **17–20 %** of
  their samples pinned at the right-hand servo clamp at 11.8–12.3 V. That bias
  is not cosmetic: it moved the fitted offset by 0.018 servo units and **flipped
  the sign** of the measured left/right asymmetry. An earlier pass of this work
  used only the short bags and applied a wrong offset as a result. **All
  conclusions above come from the long bags.**
- **VSLAM is not reliably usable as a speed reference.** On one short bag its
  tracking odometry covers ~10 s of 65 s; extrapolating it flat produced a
  *sign-flipped* steering gain that looked entirely plausible. On the long bags
  it is fine (R² 0.986–0.988). Our fitter now selects the reference on measured
  R² and coverage.
- `mapping_drive_145639` **is not a drive** — the motor turned but two
  independent ground references agree the car moved < 1.5 cm/s across 171 s.
  Excluded.
- The driver is transparent: `ackermann_to_vesc` applies exactly the affine map
  it is configured with (99.9 % of samples exact, residual median exactly 0), and
  it does **not** clamp — the clamp is downstream in `vesc_driver`. The rate
  limiter never engaged. So commanded and actuated are interchangeable on this
  data.

## 8. Two things we would like back from you

1. **Confirm the `gz` fix on your side.** If your pipeline reads the VESC IMU
   directly, it needs the `imu_link` → `base_link` transform applied. We would
   like to see your |scale| ≈ 1.00 come back with the correct sign rather than
   the correct magnitude.
2. **Which signal produced the "car moves 10–19 % faster than commanded" result**
   in the 0.08–0.21 m/s band? That is evidence about `speed_to_erpm_gain` only if
   "measured" came from an ERPM-independent source. If it came from `vesc_odom`
   or `sensors/core`, both are ERPM-derived, and the finding is instead that the
   VESC's own speed loop overshoots at low command — a different problem with a
   different owner. Our measurement of the gain is **3687–4022** across six bags,
   straddling the configured 3750, which does not support an 8–19 % gain error.

## 9. What is not settled, on our side

- `speed_to_erpm_gain` to better than ~7 %. Two references disagree 5 % on the
  *same* bag, which bounds what incidental driving can resolve. A deliberate
  ERPM staircase is queued.
- The real mechanical servo travel. `[0.08, 0.92]` is inherited from another car
  and `servo_min` has never been reached here.
- Toe, and how the 5.7 % asymmetry splits into geometry versus slip.
- Ground breakaway as a deliberate measurement rather than inferred from where
  drive resumes after a dropout (currently 0.20–0.26 m/s, i.e. 750–975 erpm).
