# Handoff — actuator sysid: Stages 1–3 done, Stage 0 and the LUCIO reply remain

Written 2026-08-07. Continues the steering/throttle system-identification
exercise; the staged plan it follows (Stages 0-5) is held outside this repo and
remains the authoritative spec. This document says only what changed, what the
numbers are, and what is left.

## What is done

- **Stage 1** (driver transparency) — passed, `scripts/analysis/check_driver_transform.py`.
- **Stage 2** (offline sysid) — done, `scripts/analysis/fit_actuators.py`.
- **Stage 3** (apply) — **partially applied**: the steering gain only.
- Results, method, and every number: **`SYSID_RESULTS.md`**, alongside this file.

### What was changed in the repo

| file | change |
|---|---|
| `config/vehicle/vesc.yaml` | `steering_angle_to_servo_gain` −1.4 → **−1.1448**. `steering_angle_to_servo_offset` **left at 0.56** — measured 0.5591/0.5610, it was already right. `speed_to_erpm_gain` left at 3750 (measured 3687–4022, good to ~7 %). `servo_min`/`servo_max` untouched and still inherited. |
| `scripts/analysis/` | two new scripts (above). |

The package's architecture notes were also corrected: they described the servo
bounds with left and right transposed.

**Not yet done on the robot.** The change is committed to the working tree only;
gosling1 gets `f1tenth_launch` from a staged tarball, not from git, so it still
runs the old −1.4. Stage the new `vesc.yaml` and **grep the deployed file for
`-1.1448` before trusting any run** — a stale tarball silently reverting
committed config has bitten this project before.

## The one number that matters

The steering gain was over-steering by ~18–23 %. Applied value is the mean of
the two bags that turn **both** directions and **never touch the servo clamp**:
`mapping_drive_170025` (a = −0.8783, s₀ = 0.5591) and `figure8_172338`
(a = −0.8688, s₀ = 0.5610), which agree to 1.1 %. The LUCIO side independently
got `k = 1.177` from different code, a different yaw reference, and a different
regressor.

**Read the bag section of SYSID_RESULTS.md before adding data.** There are two
families of bags and they are not the same recordings. The full-length
2026-08-05 session in `bags/20260805/` is the good one — 0 % servo saturation,
0–1 dropout windows, 12.5–12.8 V. The shorter
`rosbags/*_with_localization_and_pointcloud` excerpts spend 17–20 % of their
samples pinned at the right-hand clamp, and that bias is not cosmetic: it moved
the fitted offset by 0.018 servo units and **flipped the sign of the measured
left/right asymmetry**. An earlier pass of this work used only the short bags and
applied a wrong offset (0.5419) as a result.

## Stage 0 — the bench sweep (next session, at the car)

Still the backbone of the exercise, and nothing here substitutes for it. Three
practical notes from the operator that change the plan's Stage 0 text:

**1. There is no protractor.** The plan says "straightedge or phone
inclinometer" — an *inclinometer* is the wrong instrument: steering angle is
rotation about a **vertical** axis, so a level/inclinometer reads nothing. Two
options that do work:

- **Phone compass**, held flat against the wheel's outer face. Read the heading
  at zero-steer as the reference and take **differences**, never absolute
  headings — there is a motor, a LiPo, and steel hardware inches away, and the
  local field is not north. Take every reading from the same phone position.
- **Geometric, and probably better**: lay a straightedge along the wheel face and
  measure its perpendicular offset from a fixed longitudinal reference line at
  two points a known distance `d` apart; the angle is `atan(Δ/d)`. No magnetic
  bias, no calibration, and it resolves small angles better than a phone compass.
  This is also the only method that measures **toe** cleanly, which is a main
  reason Stage 0 exists.

**2. Reaching the mechanical stops is gated by `servo_min`/`servo_max`, not by
the gain and offset.** The operator's instinct that "the current values might be
saturating too early" is right about the symptom and one level off on the cause.
The sweep publishes **directly** on `vehicle/commands/servo/position` with the
stack down, which bypasses `ackermann_to_vesc` and therefore bypasses the gain
and offset entirely. What still clamps is `vesc_driver`, using
`servo_min: 0.08` / `servo_max: 0.92` from `vesc.yaml`. To explore past those,
widen **those two** for the sweep — and widen them **one small step at a time**,
because they are inherited from another car and a servo driven past its stop
stalls and cooks.

**3. `servo_min` has never been reached on this car.** Across all six archived
bags, `servo_min` was never touched; in the two that reach a bound at all, 100 %
of the saturation was at `servo_max` (the right lock). So there is no evidence
whatsoever about the left bound, and the sweep must not assume the two ends are
symmetric.

Also for Stage 0: tape-measure the wheelbase (0.25 vs 0.256 is still unsettled,
and the fitted slope scales with it), and take the **deferred servo-horn
re-centring decision**. Re-centring invalidates the archived bags for
calibration — if it happens, everything above becomes a methodology dry-run and
all numbers must come from fresh data.

## Verify before anything else on the car

The plan's Stage 3 check, now that the gain has actually changed:

1. Straight line at servo centre — the offset is unchanged at 0.56, so this is
   a regression check rather than a new-value check.
2. Fixed-radius circles **both** directions — commanded radius vs measured. The
   right-hand one matters most: right is the limited direction and
   `max_steering:=0.34` still clips it (servo 0.9492 → 0.92). Left no longer
   clips (servo 0.1708, was 0.084 and marginal).

`max_steering` was deliberately **not** changed — 0.314 rad is the symmetric
no-clip value, but that is the Stage 0 asymmetry decision, not a free edit.
Calibrated range is +24.0° left / −18.0° right.

## Still open

- **Stage 3b decision gate.** Item 1 is **confirmed**: `ackermann_to_vesc.cpp:71`
  applies `erpm = gain*speed + offset` unconditionally with no dead-at-zero path,
  so `speed_to_erpm_offset` cannot be deadband compensation. Item 2 is confirmed
  *arithmetically* (562 erpm ceiling from rest vs 750–975 erpm ground breakaway)
  but **item 3 is not done** — nobody has checked the MPC's actual `max_accel`
  and control period against the second-hand 3.0/0.05. Item 4 (VESC firmware vs
  driver code) is untouched. Do not write the VESC brief before all four hold,
  and **ask the operator first** — the plan is explicit that this is a stop point.
- **The reply to LUCIO.** Enough is now known to answer most of their §6/§9:
  the `gz` sign (roll-180 mount, consumer must apply the TF — verified against
  `odometry/local`, not just by reading the TF), `k = 1.177` reproduced,
  measured left/right asymmetry ~5.7 % (more authority left), and that the
  throttle path contributes **< 20 ms** of their 80 ms transport delay. Their
  |scale| ≈ 1.00 for `gz` is reproduced here at 0.97–0.99 against
  `odometry/local`. Two answers still need Stage 0: the measured wheelbase, and
  the post-recalibration steering limit.
- **Stages 4 and 5** — re-record (4a confirmation set, 4b excitation set incl.
  the new `26_sysid_drive.sh`), then the final fit. `k → 1.0` on 4a is the
  acceptance test for the whole exercise.

## Traps worth not rediscovering

- Run the analysis scripts with **both** overlays sourced —
  `/opt/ros/humble/setup.bash` *and* `/workspaces/f1tenth/install/setup.bash`;
  `vesc_msgs` is only in the second, and the failure is a bare `ModuleNotFoundError`.
- Bags live at `/mnt/shared_dir/rosbags/` **inside** the container, not at the
  host path `/mnt/f1tenth_ssd/shared_dir/rosbags/`. The rosbag2 error for a
  wrong path is a misleading "Could not load/open plugin with storage id 'mcap'".
- **Do not assume VSLAM is a usable speed reference.** On the short figure8 its
  tracking odometry covers ~10 s of a 65 s bag; `np.interp` extrapolated it flat
  and the fit returned a *sign-flipped* gain that looked plausible. (On the long
  bags it is fine, R² 0.986–0.988.) `fit_actuators.py` now selects on measured R²
  and coverage, but the general lesson is broader than this script.
- **`mapping_drive_145639` is not a drive.** The motor turned but the car never
  moved — rf2o and VSLAM both cap at ~0.01 m/s across 171 s. Wheels off the
  ground, or an aborted attempt. The fitter refuses it; that is correct.
- The four full-length bags are 17–28 GB each and take several minutes apiece to
  scan. Run them in the background, not on a foreground timeout.
- `pf_sweep_claude_0807` and `mpc_claude_0806` are other agents' containers.
  Analysis here ran read-only inside `jetson_container_20260807_085244` and
  started no ROS nodes, so no domain collision — keep it that way, or pick a
  non-colliding `ROS_DOMAIN_ID` checked from the **host**.
