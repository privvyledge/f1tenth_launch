# Prompt for the next chat — IMU bias removal, launch validation, rf2o retest

> **STATUS 2026-08-26 — the bias-removal half of this document is DONE except for
> one decision.** The node is built, wired and now verified offline: the estimate
> reproduces the measured −0.00214 rad/s to 3.1e-05 and the subtraction is
> bit-exact. What remains is the **watchdog decision** for the staleness hazard,
> which is confirmed rather than theoretical (bug-251). Read
> `scripts/live_runs/BIAS_REMOVER_OFFLINE_20260826.md` before this file.
>
> **Three claims below are stale and will cost you time:**
> - *"The Jetson has no internet, so this cannot be `apt install` on the robot."*
>   Retired 2026-08-24 — it is online. But it is also moot: see the next point.
> - *"`imu_processors` is released for Humble at 0.5.2."* No. The apt binary is
>   **0.4.1**; 0.5.2 is the **source tag**, and the image already carries it built
>   from source. Going to apt would be a downgrade.
> - *"Root filesystem is at 96 %, ~1.1 GB free."* Retired — gosling1 was reflashed
>   2026-08-24 and `/mnt/shared_dir` has ~775 G free. Still log to the SSD.
>
> The rf2o and launch-validation halves of this document are unaffected.

Paste this into a fresh session. This resumes the 2026-08-09 evening session and
is **separate from `NEXT_CHAT_PROMPT.md`**, which owns the actuator
system-identification track. See §"How this relates" at the bottom.

---

Read `docs/imu_bias_removal_spec.md` first, then `docs/imu_yaw_bias_notes.md`
(superseded header only) in the f1tenth_launch repo, branch `perf/config-tuning`.
Then pick up the IMU gyro-bias work.

## What was settled on 2026-08-09, so it is not re-derived

**The Autoware plan is dead and the replacement is chosen.** `imu_corrector` is
not installed in any robot image and both call sites hardcode
`remove_imu_bias:='False'` (`realsense_d435i.launch.py:366`,
`vehicle.launch.py:769`), so `config/filters/imu_corrector.yaml` is dead config —
editing it changes nothing. `imu_filter_madgwick` does not fill the gap: it is a
**pass-through for `angular_velocity`**, verified on the vehicle by subscribing to
its input and output at once (identical means, identical sample counts). Nothing
in the running chain touches the gyro rate.

**The constant is solid.** RealSense z-gyro bias is **−0.00214 rad/s** (−7.4
°/min), holding within 1.2 % across four 60 s stationary measurements over two
days, two container instances and a system-clock correction. The **VESC** row is
still dated 2026-08-08 (0.00453/0.00468 rad/s) because the VESC was unpowered on
2026-08-09 — re-measure before acting on it.

**The replacement installs from apt.** `sudo apt install ros-humble-imu-pipeline`
— `imu_processors` is released for Humble at 0.5.2 and that tag builds
`imu_bias_remover`. No fork, no `.repos`, no source build. It is a composable
component, `imu_processors::ImuBiasRemover`.

**Operator decisions already made — do not reopen:**
- `imu1` (`camera/imu/filtered`) `vyaw` fusion stays **enabled and as-is**. There
  are successful test runs on that configuration. Correcting the *input* is in
  scope; changing which inputs are fused is not.
- No new repository. The apt package goes in the image's apt list in the separate
  workspace-build repo.

## The work

`docs/imu_bias_removal_spec.md` §5 has the ordered steps. In short:

1. Add `ros-humble-imu-pipeline` to the **image build** — the Jetson has no
   internet, so this cannot be `apt install` on the robot, and `/workspaces` is a
   container layer, so hand-installs vanish on restart.
2. Wire the RealSense chain: `camera/imu` → bias remover → `camera/imu/bias_removed`
   → madgwick → `camera/imu/filtered`. `launch/filters/imu_filter.launch.py`
   already has exactly this shape in its `remove_imu_bias` branch — reuse it,
   swap the node, flip the flag at `realsense_d435i.launch.py:366`.
3. While in that file, fix the latent trap: `do_bias_estimation`,
   `do_adaptive_gain`, `gain_acc` and `gain_mag` are complementary-filter
   parameters being passed to the madgwick node, where ROS 2 silently drops them
   as undeclared overrides. The launch file reads as though bias estimation is on.
4. Verify per spec §6. **Stop and report if `odometry/local` regresses.**
5. The VESC chain (spec §5 step 4) is a **separate, later** change. Do not bundle.

### Two hazards specific to this node — both in spec §3 and §6

- Its velocity source must be **`vehicle/vesc_odom`**. Not `cmd_vel` (inbound-only
  here, silent under teleop, so silence reads as "stationary" and it accumulates
  bias while driving). Not `odometry/local` (EKF output — closes a loop with the
  IMU being corrected).
- The stationary test is `twist_is_zero_ || odom_is_zero_` with **no staleness
  timeout**. A velocity source that dies while reading "stopped" pins
  `angular_velocity` at zero **forever, including while driving**. This is live on
  this vehicle — the VESC driver aborts on serial EIO and goes dead-stick while
  the command topics still look healthy. Verification must include killing
  `vesc_odom` while parked and then driving.
- A parked re-measurement reads ≈0 **whether or not the correction works**,
  because the zeroing path is active and the subtraction path is not. Only a
  moving test, or logging the node's `bias` topic and checking it converges to
  −0.00214, actually discriminates.

## Also open from this session

**rf2o retest — DONE 2026-08-09 evening, stationary half passed.** Commit
`664e0fc` on `privvyledge/rf2o_laser_odometry` branch `ros2`. `odom/rf2o`
stationary drift went **+3.08 → +0.02 and +0.40 °/min** across two parked 60 s
runs, with `odometry/local` at +0.08 / −0.15 (no regression). Full result and the
three implementation properties that were checked directly are in
`docs/rf2o_zero_velocity_brief.md`. **The moving case is still untested** — it needs the
moving-odometry check (`odom_moving_check.py`, procedure in
`DEMO_RUNBOOK_20260810.md` §3), which needs battery power and a driven leg.

Two things this cost that are worth not re-paying: the image's rf2o is one commit
behind and `/workspaces` is a container layer, so the patch must be re-applied
and rebuilt after **every** container restart (`prep_container.sh` does it); and
`yaw_drift.py` hardcoded a `/gosling1` prefix, so on an unnamespaced bring-up it
reported 0 samples on every row — which reads as a dead stack rather than a wrong
prefix. Fixed; set `F1TENTH_NS=''` for an unnamespaced stack.

Baseline to compare against, measured 2026-08-09, car elevated, VESC unpowered so
the EKF ran on rf2o + camera IMU only — the weakest input set it ever sees:

| topic | stationary drift |
|---|---|
| `odom/rf2o` | **+3.08 °/min** (prior runs: −5.58, +4.60 — a random walk) |
| `odometry/local` | **−0.30 °/min** |

Both `odom2` rejection thresholds confirmed as **3.0** in the live node via
`ros2 param get`. Note what this does *not* show: these gates exist for
driving-time timestamp spikes and a stationary car cannot produce those. The
operator planned a drive for later on 2026-08-09 — check whether that produced a
bag before re-running anything stationary.

**Launch validation, done and clean.** Sensors + localization bring-up: 37 nodes,
no duplicates, `/camera/imu` 197–200 Hz, `/lidar/scan_filtered` 8.5 Hz,
`/odom/rf2o` 9.1 Hz, `/odometry/local` 30.0 Hz. Vehicle and navigation were not
launched (VESC unpowered), so `teleop.launch.py`, the command_gate heartbeat path
and Nav2 are **still untested this session**.

## Robot state and traps paid for on 2026-08-09

- Left clean: no containers, no ROS processes. Clock corrected by the operator.
- **Use `ROS_DOMAIN_ID=7`.** Other agents use 0 and 42. Coworkers have a
  docker-compose stack (`docker-ros2-1`, `roscore`, `docker-bridge-1`) set to
  **start on boot** which auto-launches teleop on domain 0. That is theirs; leave
  it alone.
- **Every container restart reverts `f1tenth_launch` config to the image**, which
  still carries `steering_angle_to_servo_gain: -1.4` and `wheelbase: .25`.
  Re-stage `/mnt/f1tenth_ssd/shared_dir/imu_claude_0809/cfg_0809.tar.gz` into
  `/workspaces/f1tenth/src/f1tenth_launch` and **grep the values** before
  trusting any run.
- **Start the container with `~/bolus_ws/f1tenth_launch.sh`**, never a hand-built
  `docker run`. For unattended use, `/mnt/f1tenth_ssd/shared_dir/start_imu_claude.sh`
  wraps it in `script` to supply the pty that `docker run -it` needs.
- **The camera silently fails in a headless-launched container**:
  `Authorization required` → `Could not open OpenGL window`. LiDAR still works so
  the stack looks half-healthy. Fix on the host as the desktop user:
  `xhost +local:` and `xhost +SI:localuser:root`. Not the xauth cookie file.
- **Never `pkill` inside the container to tear down a launch.** It orphans the
  children, they keep publishing, and a relaunch stacks a second node set on top
  — 85 nodes, and the orphaned RealSense still holds the USB device so the new
  one never streams. An early measurement read `camera/imu/filtered` at 553.9 Hz
  against a 187.6 Hz input for exactly this reason. Restart the container, and
  check `ros2 node list | sort | uniq -d` before trusting any number.
- **Check `date` on gosling1 after any reboot.** No RTC hold, no internet for NTP;
  it boots at 1969-12-31. Correcting it mid-session jumps the clock 56 years
  forward and silently empties any measurement using a wall-clock deadline.
- Root filesystem is at **96 %, ~1.1 GB free**. Log to `/mnt/f1tenth_ssd`, never
  to the Jetson home directory.

Scripts from this session live in `/mnt/f1tenth_ssd/shared_dir/imu_claude_0809/`:
`imu_bias_measure.py` (paired raw/filtered gyro stats), `yaw_drift_pair.py`
(rf2o vs fused heading drift), `session.sh` (one bring-up, all measurements).

## Bugs logged

`bug-183` (imu_corrector absent / dead config), `bug-184` (madgwick silently drops
four params), `bug-185` (X11 auth → camera fails), `bug-186` (pkill orphans →
duplicate nodes inflate measurements), `bug-187` (Jetson clock at epoch).

## How this relates to `NEXT_CHAT_PROMPT.md`

They are **independent tracks that share one robot and one drive session**.

`NEXT_CHAT_PROMPT.md` owns actuator system identification: the Stage 0 bench servo
sweep, the LUCIO correspondence, and Stages 4a/4b/5. Its acceptance test is the
steering gain `k → 1.0`.

This document owns the heading/fusion chain: gyro bias, the EKF's IMU inputs, and
the rf2o gates.

The couplings are:

1. **Both need the same drive.** Stage 4a is a confirmation drive set; the rf2o
   gates and any bias correction also need a moving test. One session can serve
   both if the bags record the union of topics. Coordinate rather than driving
   twice.
2. **Do not let a fusion change ride along with a Stage 4a bag.** `k → 1.0` is
   the acceptance test for the whole sysid exercise and it must be measured
   against a fusion configuration that is not simultaneously changing. Sequence:
   Stage 4a first on today's fusion, then the bias work.
3. **The stale-image problem hits both.** The calibrated `-1.1448` and `0.256`
   only ever lived in a container filesystem and were lost on the 2026-08-09
   reboot. Any sysid result is void if the config was not re-staged and grepped.
4. One item in `NEXT_CHAT_PROMPT.md`'s "Also open" list is partly resolved: it
   reports that the robot container sets `CYCLONEDDS_URI` but not
   `RMW_IMPLEMENTATION` (bug-166). The coworkers' newer `humble-latest` image
   **does** set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. The operator image
   `humble-devel-08052026` was not re-checked on 2026-08-09 — verify before
   assuming either way.
