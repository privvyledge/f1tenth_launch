# `imu_bias_remover` — offline verification, 2026-08-26

**Result: the node behaves exactly as `docs/imu_bias_removal_spec.md` describes,
and the staleness hazard is real and now measured rather than inferred.**

All four questions were answered with the car **parked, on a warm container, no
drive battery**. The spec and the previous handoff both said the moving half was
owed to a drive session. It was not: see "Why this did not need a drive".

Tooling: `bias_test_driver.py`, `bias_test_analyze.py`, `run_bias_test.sh`.
Raw data: `biastest_0826/bias_test.csv` on gosling1 (13 978 paired samples).

---

## 1 · What was actually run

Real RealSense gyro replayed from `claude_bringup_0826/run18_constdt`
(43 358 `/camera/imu` messages, today's camera, post-`constant_dt`), against a
**synthetic** `nav_msgs/Odometry` velocity source under the harness's control:

| phase | t (s) | odom | what it tests |
|---|---|---|---|
| `A_stationary` | 0–25 | published, twist 0 | does `bias` converge to the real gyro mean |
| `B_moving` | 25–40 | published, `linear.x = 1.0` | is the output exactly `raw − bias` |
| `C_stationary` | 40–50 | published, twist 0 | does accumulation resume |
| `D_source_dead` | 50–70 | **not published at all** | the staleness hazard |

The phase clock starts at the **first IMU message**, not at node start. The bag
carries ~23 s of lead-in (the recorder starts before the launch, and the camera
needs ~9.5 s to stream). A first attempt keyed off node start lost the whole `A`
window, so `B` subtracted a bias of exactly `0.0` and "passed" degenerately.
That run is kept as `bias_test_run1_FLAWED_no_phaseA.csv` — **a green `B` with
`bias_z = 0.0` in the CSV is the tell.**

## 2 · Results

```
CHECK 1  bias.z settled   -0.002223 rad/s
         raw z-gyro mean  -0.002192 rad/s   (this bag's own truth)
         measured constant -0.002140 rad/s  (spec section 2, 2026-08-09)
         |bias - raw_mean|  0.000031        PASS

CHECK 2  moving output == raw - bias:  n=2792  max residual 0.000e+00   PASS
CHECK 3  stationary output pinned to 0: A and C both 0.000e+00          PASS
CHECK 4  source dead: n=3996  max|out_z| 0.000e+00  while the raw gyro
         is still live at max|raw_z| 0.036621 rad/s        HAZARD CONFIRMED
```

**Check 1 is the discriminating one.** The estimator, given only real gyro data
and a "stopped" flag, independently reproduces the statically measured bias to
**3.1e-05 rad/s** against the bag's own mean, and lands within 4 % of the
2026-08-09 constant. That is the observation the handoff asked for, and it is
the only stationary observation that can distinguish a working correction from a
node that is merely zeroing the rate.

**Check 2 is bit-exact**, not approximately right: `max residual 0.000e+00` over
2792 samples.

### On the guard band — do not read it as fudging a failure

Scored without it, checks 2 and 3 fail. The failures are **exactly 3 samples
each, inside 10 ms of a phase boundary** (`25.00–25.01`, `40.00–40.01`) — one
tick of the 50 Hz synthetic odom against a 200 Hz IMU. They are the harness
labelling rows by *receive* time while the node picked its branch earlier. The
analyzer excludes ±0.5 s around each boundary **and prints the excluded rows'
placement underneath**, so a genuine mid-phase defect could not hide behind it.
Mid-phase, the residual is exactly zero.

## 3 · The hazard, now measured

`imu_bias_remover`'s stationary test is `twist_is_zero_ || odom_is_zero_` and
**neither flag has a staleness timeout** — confirmed by reading the shipping
source, not the docs. Phase D stops publishing the velocity source while the
flag reads `true`. The node then published `angular_velocity.z == 0.0` for
**3996 consecutive samples**, while the real gyro underneath was reading up to
0.037 rad/s. It never left the zeroing branch and had no way to.

On this vehicle that is not theoretical: the VESC driver aborts on serial EIO
and goes dead-stick while every command topic still looks healthy. If
`vehicle/vesc_odom` dies while the car is parked, the corrected gyro is pinned
at zero **from then on, including once the car moves** — feeding a confidently
wrong yaw rate into `ekf_odom`.

**So `remove_imu_bias` must not be flipped to `True` without a watchdog on
`vehicle/vesc_odom`.** That is a design decision, not a measurement, and it is
open.

## 4 · Why this did not need a drive

The node's only use of the velocity source is a threshold test on twist, and its
only action is arithmetic on `angular_velocity`. "Moving", to this node, means
nothing more than *odom twist above `odom_threshold`* — so a synthetic source
exercises the moving branch exactly as a real one would, with the gyro data
still real. What a synthetic source **cannot** tell you is whether the
correction improves real heading accuracy over a driven leg; that still belongs
to the moving-odometry check.

## 5 · Version, corrected

The previous handoff said the available version is **0.4.1** and that every
behavioural claim had to be re-derived against it. That is true of the Humble
apt binary (`0.4.1-1jammy.20260804.210108`, confirmed today with working
internet) but **not of this robot**: `privvyledge/f1tenth:humble-devel-08092026`
carries **0.5.2 built from source** at `/workspaces/f1tenth/install/`, which is
the version the spec was written against. Everything in spec section 3 was
re-checked directly against that shipping source and holds: topics `imu` →
`imu_biased` plus `bias`; parameters `use_odom` / `use_cmd_vel` / `use_stamped` /
`accumulator_alpha` / `odom_threshold` / `cmd_vel_threshold`; EMA accumulate-and-
zero while stationary, subtract while moving.

**Switching to apt would be a downgrade** (0.4.1 < 0.5.2). Spec section 5 step 1
and `docs/build_repo_requirements.md` should ask the image build to keep
building 0.5.2 from source, not `apt install ros-humble-imu-pipeline`.

## 6 · What is still open

- **The watchdog decision** (section 3). Nothing should be enabled before it.
- **`remove_imu_bias` is still `'False'`** at both call sites
  (`realsense_d435i.launch.py:373`, `vehicle.launch.py:377`) — unchanged.
- **The live parked wiring test** (spec section 6 step 3): bring the stack up
  with the node in the chain and confirm `odometry/local` does not regress from
  its +0.04 / +0.01 / +0.17 deg/min band. Stationary, but it needs a source edit
  to flip the flag, so it was not done unilaterally.
- **Whether the correction improves driven heading** — the moving-odometry check.
- **The VESC chain is spec step 4 and stays unbundled.**
