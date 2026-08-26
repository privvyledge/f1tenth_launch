# Spec: gyro bias removal for the f1tenth IMU chains

**Audience:** whoever builds this. It is written to be handed to a fresh agent or
person with no prior context on this vehicle.

**Status:** specification only. Nothing here is built. The measured constants in
§2 are real; everything else is a proposal.

**Why this exists:** `docs/imu_yaw_bias_notes.md` measured a repeatable z-gyro
bias on both IMUs and proposed correcting it with Autoware's `imu_corrector`.
That plan is dead — see §1. This document replaces it.

---

## 1. What was found, and why the original plan does not work

`f1tenth_launch` wires an Autoware node called `imu_corrector` into
`launch/filters/imu_filter.launch.py`, configured by
`config/filters/imu_corrector.yaml`. Three facts, all verified on 2026-08-09:

1. **The node is not installed anywhere.** Checked the full `install/` tree of
   `privvyledge/f1tenth:humble-devel-08052026` and of the newer
   `privvyledge/f1tenth:humble-latest`. No `imu_corrector` package in either.
2. **It is switched off at every call site anyway.** Both parents hardcode
   `'remove_imu_bias': 'False'` with the comment *"disabled since its not really
   useful and requires Autoware installation"* —
   `launch/sensors/realsense_d435i.launch.py:366` and
   `launch/vehicle/vehicle.launch.py:769`.
3. Therefore **`config/filters/imu_corrector.yaml` is dead config.** Editing
   `angular_velocity_offset_z` in it changes nothing at runtime. Anyone who
   "applies the fix" there and re-measures will correctly observe no change and
   may conclude the bias was never real.

The chain that *does* run is `imu_filter_madgwick`, and it does not help:

- Upstream `imu_filter_madgwick` (CCNYRoboticsLab/imu_tools, `humble`) copies the
  input message with `ImuMsg imu_msg = *imu_msg_raw;` and then writes **only**
  `imu_msg.orientation` (plus `linear_acceleration` when
  `remove_gravity_vector` is set). **`angular_velocity` is never assigned, so it
  passes through byte-for-byte.**
- The node has **no `do_bias_estimation` parameter** — that belongs to
  `imu_complementary_filter`. `launch/filters/imu_filter.launch.py` passes one
  parameter block to *both* node types, so `do_bias_estimation`,
  `do_adaptive_gain`, `gain_acc` and `gain_mag` are undeclared overrides on the
  madgwick node and are silently ignored by ROS 2. **This is a latent trap, not
  just clutter**: the launch file reads as though bias estimation is enabled.

  Confirmed at runtime on gosling1, 2026-08-09 — `ros2 param list
  /realsense_imu_filter` on the live node returns `constant_dt, declination,
  fixed_frame, gain, mag_bias_{x,y,z}, orientation_stddev, publish_debug_topics,
  publish_tf, remove_gravity_vector, reverse_tf, stateless, use_mag,
  use_sim_time, world_frame, yaw_offset, zeta` and none of the four.
- Madgwick's actual gyro-drift term is `zeta`, which is not set and in any case
  only steers the internal orientation estimate — it never touches the published
  rate. The running node confirms it: `Gyro drift bias set to 0.000000`.

**Verified on the vehicle, not just read from source.** Subscribing to the input
and the output simultaneously for 60 s, car stationary and elevated on a stand
(gosling1, 2026-08-09, single clean bring-up, no duplicate nodes, both topics at
199–200 Hz):

| window | `camera/imu` z-gyro mean | `camera/imu/filtered` z-gyro mean | n (raw / filtered) |
|---|---|---|---|
| A | −0.002149 rad/s | −0.002159 rad/s | 11618 / 11619 |
| B | −0.002131 rad/s | −0.002134 rad/s | 12002 / 11999 |

Same sample count, same mean to the fourth decimal, on every axis. The madgwick
node is a **pass-through for `angular_velocity`**, and the bias reaches the EKF
intact.

> Measure this on a clean bring-up or not at all. An earlier attempt the same day
> read `camera/imu/filtered` at 553.9 Hz against a 187.6 Hz input because three
> orphaned `realsense_imu_filter` nodes from `kill -9`'d launches were still
> publishing. Check `ros2 node list | sort | uniq -d` first (bug-186).

**Consequence:** `camera/imu/filtered` carries the raw, uncorrected RealSense
gyro rate, and `ekf_odom.yaml` fuses its `vyaw` (`imu1_config` row 4 is
`[true, true, true]`). The bias in §2 is being fused into the vehicle's heading
today.

## 2. The measured constants

Stationary z-gyro bias, car parked and powered, from
`scripts/live_runs/yaw_drift.py 60` on gosling1:

| IMU | date | mean z-gyro (rad/s) | sample std | implied yaw rate |
|---|---|---|---|---|
| VESC | 2026-08-06 | 0.00434 | — | +14.9 °/min |
| VESC | 2026-08-08 A | 0.004532 | 0.00132 | +15.58 °/min |
| VESC | 2026-08-08 B | 0.004679 | 0.00153 | +16.09 °/min |
| RealSense | 2026-08-08 A | −0.002144 | 0.01193 | −7.37 °/min |
| RealSense | 2026-08-08 B | −0.002157 | 0.01049 | −7.42 °/min |
| RealSense | 2026-08-09 A | −0.002149 | 0.01490 | −7.39 °/min |
| RealSense | 2026-08-09 B | −0.002131 | 0.01403 | −7.33 °/min |

Both biases repeat. The RealSense figure now spans **four measurements across two
days, two container instances and a system-clock correction, and holds within
1.2 %** — −0.002131 to −0.002157 rad/s. Use **−0.00214 rad/s** as the constant.
That repeatability is what makes a correction legitimate rather than a fit to one
run. The RealSense is ~8× noisier per sample than the VESC while having the *more*
stable mean; noise and bias are separate problems and this work addresses only the
mean.

The **VESC row has not been re-confirmed since 2026-08-08** — the VESC is powered
from the drive pack, and it was disconnected for the 2026-08-09 session, so
`vehicle/sensors/imu/raw` produced nothing. Re-measure before acting on §5 step 4.

## 3. Recommended approach: `imu_pipeline`

Use **`imu_processors/imu_bias_remover`** from
[`ros-perception/imu_pipeline`](https://github.com/ros-perception/imu_pipeline).

**It is released for Humble and installs from apt — no fork, no `.repos` entry,
no source build:**

```bash
sudo apt install ros-humble-imu-pipeline
```

Verified 2026-08-09: `imu_processors` is released into Humble at **0.5.2**, and
the 0.5.2 tag's `imu_processors/CMakeLists.txt` builds
`add_library(imu_bias_remover SHARED src/imu_bias_remover.cpp)` and registers
`imu_bias_remover_node`. The node in 0.5.2 matches the description below — it was
checked against the release tag, not the development branch.

This replaces an Autoware dependency the project is already committed to dropping
with a `ros-perception` one, which is the same ecosystem as `imu_filter_madgwick`
that the chain already uses.

**It is a composable component**, registered as `imu_processors::ImuBiasRemover`
via `rclcpp_components_register_node`. It can therefore load into the existing
`sensing_container` under `use_composition:=True` rather than running as a
separate process — which is what the rest of the sensor chain does.

> **The apt install must happen at image build time.** The Jetson has no
> internet, so `apt install` on gosling1 will fail. This belongs in the Dockerfile
> owned by the separate workspace-build repo. Do not plan on installing it on the
> robot.

### What the node actually does

Read this carefully, because **it is not a static offset subtractor** and the
difference matters:

- **Subscribes:** `imu` (`sensor_msgs/Imu`), plus optionally `cmd_vel`
  (`Twist`/`TwistStamped`) and `odom` (`nav_msgs/Odometry`).
- **Publishes:** `imu_biased` (`sensor_msgs/Imu`) and `bias`
  (`geometry_msgs/Vector3Stamped`).
- **Parameters:** `use_cmd_vel` (false), `use_odom` (false), `use_stamped`
  (false), `accumulator_alpha` (0.01), `cmd_vel_threshold` (0.001),
  `odom_threshold` (0.001).
- **Algorithm:** when the velocity source says the robot is stationary, it
  accumulates the gyro reading into an exponential moving average
  (`alpha * meas + (1 - alpha) * avg`); when moving, it subtracts that running
  average. It modifies **only** `angular_velocity.{x,y,z}`.

Two consequences worth designing around:

1. **It zeroes the rate while stationary.** That is a bonus here: it addresses
   the same standing-still heading walk that `docs/rf2o_zero_velocity_brief.md`
   describes from the other direction. It also means a stationary drift test will
   read ≈0 by construction and therefore **cannot validate the moving-case
   correction** — see §6.
2. **It needs a velocity source, and the obvious choices are wrong.**

### Which velocity source to use — the one real design decision

| candidate | verdict |
|---|---|
| `cmd_vel` | **No.** On this vehicle `cmd_vel` is inbound-only (Nav2 → `twist_to_ackermann`) and is silent whenever Nav2 is not driving. Silence would read as "stationary" and the node would accumulate bias while the car is driven by teleop or MPC. |
| `odometry/local` | **No.** That is the EKF output, which consumes the very IMU being corrected. Closing that loop makes the bias estimate a function of its own correction. |
| `vehicle/vesc_odom` | **Yes.** Wheel/ERPM-derived, independent of every IMU, and already published whenever the VESC is alive. Use `use_odom: true` with `odom` remapped here. |

Note the failure mode of the chosen source: `vesc_odom` reads zero when the
wheels are off the ground or the car is pushed with the motor unpowered. Neither
happens in a normal run, but both happen on a test stand — which is exactly the
condition this was specified under.

### The one dangerous edge in this node

The stationary test is `if (twist_is_zero_ || odom_is_zero_)` — an **OR**, and
**neither flag has a staleness timeout**. Both initialise to `false`, so startup
is safe, and with `use_cmd_vel: false` the `twist_is_zero_` term never fires. But
each flag holds its last value indefinitely:

> **If the velocity source stops publishing while the flag is `true`, the node
> zeroes `angular_velocity` forever — including while the car is driving.**

On this vehicle that is a live risk, not a theoretical one: the VESC driver
aborts on serial EIO and the car goes dead-stick while every command topic still
looks healthy. If `vesc_odom` dies while the car is parked, the flag is stuck at
`true`, and when the VESC recovers the gyro rate stays pinned at zero.

Mitigate explicitly — do not assume upstream handles it. Either watchdog
`vehicle/vesc_odom` and stop trusting `imu_biased` when it goes stale, or gate on
liveness at the EKF. Whatever is chosen, the verification in §6 must include
killing the odom source mid-run and confirming the gyro rate does not stick at
zero.

## 4. Where it goes: nowhere new

**No new repository is needed, and nothing should be forked or vendored.**
`imu_pipeline` is upstream, maintained, and released for Humble, so it is an apt
dependency like any other:

- add `ros-humble-imu-pipeline` to the image's package install list, in the
  **separate workspace-build repo** that owns the Dockerfile — not here;
- `f1tenth_launch`'s only change is launch wiring plus a config file, which is
  exactly what a pure launch-configuration package should contain.

This is the whole integration. `f1tenth_launch` gains no custom node and no new
build dependency of its own.

Write a new repo only if §3's node proves unsuitable during §6 verification —
most plausibly if the zero-velocity coupling in §3 turns out to be unwanted and a
plain static offset is preferred. If so, the thing to write is ~40 lines
(subtract a configured constant from `angular_velocity.z`, republish) and it
should be its own small repo, installed and imported, not inlined here.

## 5. The work, in order

Each step is separately verifiable. **Do not combine steps 2 and 4** — their
effects are not separable after the fact.

**Step 0 — baseline.** `scripts/live_runs/yaw_drift.py 60`, car parked, **twice**.
One run cannot distinguish a bias from a random walk that happened to land small.
Record `odometry/local`; it is currently −0.03 to +0.16 °/min and that number is
the thing that must not get worse.

**Step 1 — get `imu_pipeline` into the image.** Add `ros-humble-imu-pipeline` to
the Dockerfile's apt list in the workspace-build repo, rebuild, confirm
`ros2 pkg executables imu_processors` lists `imu_bias_remover_node`.

Two traps here, both previously paid for on this project: the Jetson has **no
internet**, so this cannot be an `apt install` on the robot; and `/workspaces` is
a **container layer, not a bind mount**, so anything installed by hand into a
running container is lost on restart and the stack silently reverts to the image's
contents. Verify by `docker kill` + restart before believing it stuck.

**Step 2 — wire the RealSense chain.** This is the one that acts today.
Insert the node between the RealSense IMU and `imu_filter_madgwick`:

```
camera/imu  →  imu_bias_remover  →  camera/imu/bias_removed  →  madgwick  →  camera/imu/filtered
```

`launch/filters/imu_filter.launch.py` already has this shape — the
`remove_imu_bias` branch, the `imu_corrector_output_topic` argument
(`camera/imu/bias_removed`) and the `SetRemap(src='imu/data_raw', ...)` wiring all
exist and were built for exactly this. **Reuse that structure**; replace the
`Node(package='imu_corrector', ...)` action with the `imu_processors` node and
flip `remove_imu_bias` to `True` at `realsense_d435i.launch.py:366`.

While in that file, fix the latent trap from §1: the complementary-filter-only
parameters must not be passed to the madgwick node.

**Step 3 — verify step 2** per §6. Stop here and report if it regresses.

**Step 4 — the VESC chain (optional, larger, do not bundle).** The VESC's
*orientation* is unfixable from outside: the quaternion is computed onboard by
Madgwick with no magnetometer (`vehicle/sensors/imu/mag` is 100 Hz of identical
zeros), so no external offset can touch it. `ekf_odom.yaml:187` says so and it
stays true.

The **raw z-gyro is a different signal** and is not covered by that warning. It
is the cleanest gyro on the vehicle (std 0.0013, ~8× quieter than the RealSense)
at 100 Hz. Bias-corrected and fed in as `vyaw` only, it is a yaw-rate source
independent of both camera and LiDAR — the redundancy that is missing when Isaac
VSLAM aborts, which it does roughly 1 launch in 3 with no respawn.

`vehicle.launch.py:769` has the identical wiring already present and disabled.

> **Do not re-enable `imu0`'s orientation yaw as part of this.** Only the rate
> row. Every `imu0` angular-rate entry in `ekf_odom.yaml` is currently `false`;
> step 4 changes the yaw-rate entry only.

**Step 5 — do not touch `imu1`'s `vyaw` fusion.** It is currently enabled and
uncorrected, and the operator has successful test runs on that configuration
(decision, 2026-08-09). Correcting the input is in scope; changing which inputs
are fused is not.

## 6. How to verify — and the trap specific to this node

Standard protocol, from `docs/imu_yaw_bias_notes.md`:

1. Baseline parked, twice (step 0).
2. One change at a time.
3. Re-measure parked, twice. `odometry/local` must not get worse than −0.03 to
   +0.16 °/min. Improving an input while degrading the fused output means the
   change fought a source that was already compensating.
4. Then a moving run — a stationary test says nothing about scale error.
5. If every source suddenly reports the same large drift, the car moved. That is
   not a fusion regression.

**The trap:** `imu_bias_remover` zeroes the rate when its velocity source reads
stationary. A parked re-measurement will therefore show ≈0 **whether or not the
correction is working**, because the zeroing path is active and the subtraction
path is not. Step 3 above is necessary but **not sufficient**.

**The second trap:** kill `vehicle/vesc_odom` while the car is parked, then drive.
Per §3 the stationary flag has no staleness timeout, so a source that dies while
reading "stopped" pins `angular_velocity` at zero indefinitely. This test must
pass before the node is trusted on the ground.

The measurement that actually tests the correction is a **moving** one: drive a
closed loop returning to the start pose and compare final heading against truth.
Alternatively, log the node's `bias` output topic and check that the accumulated
estimate converges to the §2 constant — if it settles near −0.00215 rad/s for the
RealSense, the estimator agrees with the static measurement and the mechanism is
confirmed independent of any drive test.

## 7. Related context

- `docs/imu_yaw_bias_notes.md` — the measurements and the original (now void)
  Autoware plan.
- `docs/rf2o_zero_velocity_brief.md` — rf2o's stationary random walk, the other
  half of the heading story. No bias correction addresses it.
- `config/localization/ekf_odom.yaml` — `imu0` at line 176, `imu1` at 222, and the
  in-file note at 231 recording that `imu1`'s uncorrected `vyaw` is held by VSLAM
  and rf2o. That note is accurate and describes the fragility this work removes:
  the bias is harmless only while the sources that outvote it are alive.
- `config/filters/gyro_bias_estimator.yaml` — sets `gyro_bias_threshold: 0.003`
  rad/s, which the **VESC's measured 0.00468 exceeds**. That estimator would
  report the VESC as faulty rather than correct it. Raise the threshold or skip
  it for that chain.

---

## 8. Status, 2026-08-09 evening — built and wired, correction NOT yet measured

**Built.** `imu_processors` 0.5.2 is compiled into the container from staged
source (`/mnt/f1tenth_ssd/shared_dir/rf2o_zv_0809/imu_pipeline_0.5.2.tar.gz`) and
baked into `privvyledge/f1tenth:humble-devel-08092026`. Both
`imu_bias_remover_node` and the `imu_processors::ImuBiasRemover` component
register. The apt package is still the preferred route — see
`docs/build_repo_requirements.md`.

**Wired.** `launch/filters/imu_filter.launch.py` now instantiates the
`imu_processors` node in place of Autoware's `imu_corrector`, with `imu` /
`imu_biased` / `odom` remapped and `bias` left on the root so it can be recorded.
`config/filters/imu_bias_remover.yaml` is new. The complementary-filter-only
parameters were removed from both madgwick blocks (§1's latent trap).

**Proven to pass data.** With `remove_imu_bias` temporarily forced True:
`camera/imu` 200.6 Hz → `camera/imu/bias_removed` 200.3 → `camera/imu/filtered`
200.1, `bias` publishing.

**But the correction is unmeasured, and the test that ran cannot measure it.**
The vehicle stack was not launched, so `vehicle/vesc_odom` never published,
`odom_is_zero_` stayed `false` from its initialiser, and the node never entered
its accumulate state — `bias` held exactly `0.0` and the node was a pure
pass-through. That is the safe failure direction, and it is worth knowing that a
missing velocity source degrades to "do nothing" rather than to "zero the gyro".
It is **not** evidence that the correction works.

`remove_imu_bias` is therefore still `'False'` at both call sites.

### The next test should be offline, not on the car

`bags/20260805/figure8_172338` contains **`camera/imu` (31010 msgs)** and
**`vehicle/vesc_odom` (7751)** over a 155 s drive with stationary periods at each
end. That is everything this node consumes: it takes two topics in, publishes two
out, and touches neither TF nor the scan pipeline. Replaying those two topics
through it answers the question with no car, no battery and no risk:

- during the stationary periods `bias.z` should converge toward **−0.00214 rad/s**
  (the §2 constant). If it settles there, the estimator agrees with the static
  measurement and the mechanism is confirmed;
- while moving, `camera/imu/bias_removed`'s z-gyro should equal the raw z-gyro
  minus that accumulated value, and nothing else should change.

Note this is a much easier replay than the rf2o one that failed on 2026-08-09 —
that node needs TF and scan matching, this one needs neither.

### If it is instead enabled on a live run, two rules

1. **Not during autonomous driving.** The `twist_is_zero_ || odom_is_zero_` test
   still has no staleness timeout (§3). On this vehicle the VESC driver aborts on
   serial EIO while the command topics still look healthy; if `vesc_odom` dies
   while parked, the gyro rate is pinned at zero indefinitely — including
   mid-drive, feeding a silently wrong yaw rate into the EKF during an MPC or
   Nav2 run. Kill `vesc_odom` while parked and then drive, per §6, before
   trusting it on the ground.
2. **Not on the same bring-up as a sysid Stage 4a bag.** `k → 1.0` is the
   acceptance test for the whole sysid exercise and must be measured against a
   fusion configuration that is not simultaneously changing.
