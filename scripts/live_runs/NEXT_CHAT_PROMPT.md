# Next session — START HERE

**Updated 2026-08-07 12:25 EDT.** Read this first block, then the
2026-08-07 09:40 block, then the 2026-08-06 21:35 block. Everything from
"## The job this session: the first LIVE Nav2 goal" onward is older and partly
superseded.

## Status after the 2026-08-07 midday session (MPC bench bring-up ×3)

Brought the car up three times for a separate MPC-owning agent, answered a
drivetrain question from its bags, then shut down. **No tracked code changed
this session** — all of it was operations plus `.wolf/` (git-ignored).

### Robot state as left

- **Powered ON.** All three containers left running (warm):
  `jetson_container_20260807_085244` (the one to use),
  `mpc_claude_0806`, `pf_sweep_claude_0807`.
- **Stack is DOWN**; zero ROS processes on the host. `xhost -local:` reverted.
- Relaunch is one command — the HEAD config is already staged in that container:
  ```bash
  ssh gosling1 'DISPLAY=:0 XAUTHORITY=$HOME/.Xauthority xhost +local:'
  ssh gosling1 'docker exec -d jetson_container_20260807_085244 /mnt/shared_dir/start_mpc_stack.sh'
  ```
  Domain **42**, map `/mnt/shared_dir/maps/20260805/rtabmap_2d_final.yaml`,
  Nav2 and `twist_to_ackermann` off so `/gosling1/drive` is free for the MPC.

### Three traps that cost time — do not re-derive them

1. **Always `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` before any ROS CLI.**
   Probe scripts that omitted it fell back to FastRTPS and reported TF and the
   entire node list as EMPTY on a perfectly healthy stack. That produced a false
   "TF is dead" conclusion.
2. **The camera-0 Hz X11 failure is a cookie for display `:10`**, not a missing
   mount. `/tmp/.docker.xauth` was bind-mounted and `/tmp/.X11-unix/X0` present,
   but the cookie belonged to the SSH-forwarded `localhost:10.0` of the shell
   that created the container. You cannot repair it on a running container: the
   mount is read-only (EACCES from inside), `docker cp` says "device or resource
   busy", and host-side `xauth nmerge` creates a **new inode** the bind mount
   never sees (host 53 B, container still 54 B). Use `xhost +local:` on the host
   and launch with `XAUTHORITY` **unset**; revert with `xhost -local:`.
3. **Teardown needs the process group, from inside the container.** `kill -INT`
   on the launch PID alone does nothing, and `use_respawn:=True` resurrects
   nodes until the launch process itself exits. Sequence: `kill -INT -$PGID`,
   then TERM/KILL the launch and its wrapper script, then sweep leftover
   `joy_node` / `component_container` / `ydlidar` PIDs. Host `ssh` cannot signal
   them at all — they run as root inside the container.

### bug-147 — Isaac VSLAM aborts and does NOT respawn (open)

`visual_slam_container` died with **SIGABRT (exit −6)** on 1 of 3 launches,
~30 s after `cuVSLAM tracker was successfully initialized`. It did **not**
respawn. Camera color and both infra streams stayed at 30 Hz through the abort,
so it had input; the crash is internal.

**It fails silently from every angle you would normally check:**
`ros2 topic list` still shows `visual_slam/tracking/odometry` as a stale DDS
entry (0 Hz), and `odometry/local` keeps publishing a healthy **30.00 Hz** on
the remaining EKF inputs. With `odom1` gone and `imu0` yaw disabled, heading
falls back to **rf2o alone**. Assert VSLAM by *rate* plus `ps`, never presence:

```bash
ps -eo etime,cmd | grep "[v]isual_slam_container"
grep -c "visual_slam_container.*process has died" <launch log>
```

A restart cleared it both times.

### Drivetrain finding delivered to the MPC agent (scripts on the SSD)

From the 2026-08-06 stand-side bags (`run/stackside_20260806_202339` = loop,
`run/stackside_20260806_203218_figure8`). Scripts saved at
`/mnt/shared_dir/{vesc_stops,vesc_stops2,erpm,zeroruns}.py`.

Of the five sub-second stops: **3 are drive dropout, 1 is genuine brake
overshoot, 1 is a commanded decel.** The discriminating trace is commanded vs
actual ERPM (`cmd_erpm = 3750 × cmd_speed` vs `state.speed`), not current — in
the dropouts the command stays at 2500→1200 ERPM while actual collapses to zero
with `current_motor` at exactly 0.00 A. **Not a current limit** (current falls
to zero rather than pinning; `voltage_input` flat 12.00–12.40 V whole bag).
**Not brake overshoot** (command positive, duty positive-decaying).

Caveat that nearly caused an overclaim: exact-zero `current_motor` is *not*
unique to stops (3.8 % of the loop bag, and ~0.5 s zero-runs recur where no stop
happens). The ERPM divergence is the evidence; it does not depend on trusting
the current reading.

**Reusable number:** drive resumes at **cmd 0.20–0.26 m/s (753–962 ERPM), on
the ground** — a measured ground breakaway, just under the 0.25–0.30 m/s
extrapolated from stand tests.

## The job for the next session

Three operator decisions from 2026-08-07 12:24 EDT, to implement:

1. **CycloneDDS peers — comment out `192.168.2.193` (gosling3) ONLY.** Do not
   delete it, and **leave `192.168.2.194` (DigitalStorm) active** even though it
   is currently down. File is
   `/mnt/f1tenth_ssd/shared_dir/cyclonedds_config_static.xml`, owned by the
   build repo. **Set expectations honestly: this will NOT silence the spam.**
   `.194` is the address actually flooding the logs, and it stays in by
   instruction — so expect reduced, not eliminated, noise.
   See `CYCLONEDDS_PEERS.md`.
2. **Use `/mnt/shared_dir/cyclonedds_offline_lo.xml` for local-only runs**
   (approved). It is lo-only with a single `localhost` peer and zero peer noise;
   containers are on host networking so the MPC process and the stack still see
   each other. **Cost:** anything not in the list is silently invisible — remote
   RViz from DigitalStorm sees nothing, with no error. Local-only runs only.
3. **Do NOT touch the `lo` interface entry** (approved). It took VSLAM frame
   stalls from 0.1373/s to 0.0294/s.

Then: **investigate bug-147.** The suggested experiment is ~5 launches on the
current static config vs ~5 on `cyclonedds_offline_lo.xml`, counting
`visual_slam_container.*process has died` per log. At a ~1-in-3 abort rate that
is roughly the minimum per arm to see a difference worth believing. If the rate
is unchanged, DDS config is ruled out and the suspect is cuVSLAM itself.

**One correction to carry forward:** the earlier framing that
`Delta ... [34.1 ms] is above threshold [34.0 ms]` warnings indicate frame
jitter is weak. A 30 fps camera has a nominal period of **33.33 ms** against a
**34.0 ms** threshold — ~2 % headroom — so those fire on trivial jitter and are
near baseline. The single **219 ms** delta is the only genuinely anomalous one.
Do not justify a DDS change as a VSLAM fix; justify it as log hygiene (this
noise once buried a real YDLidar failure in a 381,976-line log).

## Status after the 2026-08-07 morning session (desk work only, no driving)

The car drove under Nav2 on 2026-08-06 and `testing_checklist.md` §9 is ticked.
This session did no driving; it closed out the `twist_to_ackermann` follow-up
and one config inconsistency. **Committed and pushed on `perf/config-tuning`.**

### bug-140 is fully closed — upstream, in the launch file, and on the robot

- **Upstream:** commit `4770ecc` on `privvyledge/trajectory_following_ros2`,
  branch `refactor/unify-backends`, is **pushed**. It replaces
  `atan2(WHEELBASE, radius)` with `atan(WHEELBASE / radius)` and turns the
  hardcoded ±0.4 rad saturation into a validated `max_steering_angle`
  parameter (upstream default still 0.4), with sign/saturation regression tests.
- **This repo:** `launch/vehicle/vehicle.launch.py` now passes
  `max_steering_angle: 0.25`, which closes the "second, separate defect" noted
  further down. 0.4 rad drives servo to `-1.4*0.4 + 0.56 = 0.0`, under the 0.08
  minimum, so the VESC clips at full left; 0.25 < the 0.257 rad left lock.
- **On the robot:** the container running that morning
  (`jetson_container_20260807_085244`) had **silently reverted to the buggy
  `atan2`** — the 2026-08-06 in-container edit and its `.bak` were both gone.
  `/workspaces` is a container layer, **not** a bind mount, and the Jetson
  cannot resolve github.com, so the commit cannot reach the car by `git fetch`.
  Patched it there as commit `121806d` and verified the running node:
  `angular.z = -1.0` (right) → `steering_angle -0.25` (was `+0.4`, full left),
  `angular.z = +0.3` → `+0.15241` = `atan(0.256 / 1.6667)` exactly.

**Every fresh container needs this re-applied** until the image carries it (an
image concern, owned by the build repo — the usual workflow copies the repo over
anyway):

```bash
/mnt/shared_dir/apply_twist_fix.sh <container-name>   # idempotent, ~2 s
# patch: /mnt/shared_dir/handoff/0001-twist-to-ackermann-fix.patch
```

Python + `--symlink-install`, so no rebuild — but restart the node. Check state
with `grep -n 'math.atan(self.WHEELBASE'`.

### wheelbase unified to 0.256 m — UNVERIFIED ON HARDWARE, watch it

`config/vehicle/vesc.yaml` said `.25` ("about 25cm") while every other consumer
used the measured **0.256**: the `base_link`→`front_axle` and both front-wheel
static TFs, `twist_to_ackermann`, `ackermann_to_twist`, and the `min_turning_r`
derivation in `nav2_params.yaml`. It is now 0.256 everywhere.

This is a **live behaviour change that has not been driven**: `vesc_odom`
computes `omega = v*tan(delta)/L`, so its kinematic yaw rate was ~2.4 % high
relative to the frames it is fused into. `vehicle/vesc_odom` is `odom0` in
`ekf_odom.yaml`, and VESC IMU yaw is disabled (bug-129), so this source carries
real weight. **On the next drive, re-run `scripts/live_runs/yaw_drift.py`
(60 s, parked) and re-score `check_map_frame.py` against a known bag** before
reading anything else as a regression. If it looks worse, this one-line change
is the first suspect.

### The robot's `f1tenth_launch` must be re-staged — this is a real trap

The container gets `f1tenth_launch` from a staged tarball, and the image ships
an older copy. A container staged from the **old** tarball will run the patched
`twist_to_ackermann` at its **upstream default of 0.4 rad**, not 0.25 — sign
correct, but back to clipping the servo at full left. `handoff/head_1912.tar.gz`
was already known stale (it silently reverts the AMCL seed) and is now stale by
two more commits.

**A fresh tarball is staged: `/mnt/shared_dir/handoff/head_0940.tar.gz`**, built
from the pushed HEAD of `perf/config-tuning` (`cec7e1d`) and verified on the
robot to contain the 0.25 limit, the 0.256 wheelbase and the AMCL seed.
**`warmstart.sh` has been repointed at it** (old copy kept as
`warmstart.sh.bak_20260807`); it was staging `head_1912.tar.gz`, so a one-shot
warm start would otherwise have silently undone all three. Verify after staging
anyway — do not assume:

```bash
grep -n "max_steering_angle" src/f1tenth_launch/launch/vehicle/vehicle.launch.py   # -> 0.25
grep -n "wheelbase"          src/f1tenth_launch/config/vehicle/vesc.yaml           # -> 0.256
grep -n "initial_pose" -A4   src/f1tenth_launch/config/localization/localizer_amcl.yaml  # -> 0.445 / -0.575
```

## Next session, in order

1. **Presentation video** from `nav2live_20260806_214427_firstgoal` — this is
   the highest-value remaining item and needs no car. Replay the bag and screen-
   record RViz with `/mnt/shared_dir/nav2_live.rviz`. **Add `lookahead_point`
   and `plan_smoothed` to that config first** — RPP publishes those;
   `local_plan` is DWB-only and stays empty. The bag has map + costmaps + plan +
   goal + the full actuation chain.
2. **Prove the goal formally SUCCEEDS.** Launch with `log_level:=info` (edit
   `/mnt/shared_dir/nav2_launch.sh`, which hardcodes `warn`) or send goals via
   `nav2_goal_probe.py`, which captures the action result code. This closes the
   open 0.38 m-vs-0.25 m tolerance question — it is a question about the last
   13 cm, not about whether the stack drives.
3. **First drive after this session's config changes**: re-verify the wheelbase
   change per the section above *before* tuning anything else.
4. Then the operator's asks in section A/B below — CPU profiling (no data
   exists yet) and the one-variable-at-a-time controller progression.
5. Only then MPC and the `steering_angle_to_servo_offset` 0.56 → 0.5
   recalibration (weekend of 2026-08-08/09).

## RESULT: the car drove autonomously under Nav2. Checklist section 9 is TICKED.

**2026-08-06 21:47 EDT — first successful autonomous Nav2 drive.** From
(0.445, -0.575) to (-1.668, -2.166) in the map frame: **2.628 m travelled,
-44.8 deg net yaw (correct, rightward), stopping 0.38 m from the goal.** The
global plan shrank monotonically as the car followed it (43 -> 35 -> 28 -> 21
-> 14 -> 9 -> 6 poses, plan start tracking the robot the whole way), speed held
at the configured 0.5 m/s cap, and the car decelerated smoothly to a stop
rather than being cut off. Bag:
`/mnt/shared_dir/run/nav2live_20260806_214427_firstgoal` (closed cleanly,
35 MB) — **this is the bag to build the presentation video from.**

One loose end: final distance 0.38 m vs the 0.25 m `xy_goal_tolerance`, so it
is not certain the goal formally SUCCEEDED rather than ending some other way.
Confirm next run with `log_level:=info` or `nav2_goal_probe.py`, which captures
the action result code. This is now a question about the last 13 cm, not about
whether the stack drives.

## The bug that caused the first failed attempt (FIXED, verified)

The first live Nav2 goal was attempted and **the blocker is found and proven**.
Bag: `/mnt/shared_dir/run/nav2live_20260806_212315_firstgoal` (sqlite3, closed
cleanly, 76 MB, has map + costmaps + plan + goal + full actuation chain).
Analyzer: `/mnt/shared_dir/analyze_nav2.py` (also in scratchpad) —
`python3 analyze_nav2.py <bag> sqlite3` reprints the whole timeline.

**Everything except one function worked.** Goal accepted, valid 46-pose plan to
the goal, replanning at 1 Hz, `cmd_vel` bounded to 0.5 m/s for 5.45 s, the R1
handover textbook-perfect (R1 at t=244.94 -> mux handover at t=245.24, exactly
the 0.3 s joystick timeout), gate opened on command, no duplicate nodes, AMCL
within 11 mm of truth.

### bug-140 — `twist_to_ackermann` inverts and saturates every right turn

`trajectory_following_ros2/twist_to_ackermann_drive.py`, in
`yaw_rate_to_steering_angle()`:

```python
radius = longitudinal_speed / desired_yaw_rate
steering_angle = math.atan2(self.WHEELBASE, radius)   # <-- WRONG
```

`WHEELBASE` is always positive. A right turn gives `yaw_rate < 0` hence
`radius < 0`, so `atan2(+L, -R)` lands in the **second quadrant** and returns
+1.6..+3.1 rad instead of a small negative angle. The node's own +/-0.4 rad
clamp then pins it to exactly **+0.4 = full left lock**. Right turns become
hard left; left turns happen to be correct.

Proof in the bag: `/cmd_vel` `angular.z` spanned **[-1.1754, -0.1000]** (all
right), `/drive` `steering_angle` was **constant +0.4000 for all 110 messages**,
the log prints `Saturating steering_angle.` every cycle, and the VESC logged
`servo command value (-0.000000) below minimum limit (0.080000), clipping`
(servo = -1.4*0.4 + 0.56 = 0.0 -> clipped to 0.08 = hard left). The car yawed
**+42 deg left** while the plan went right.

**FIXED 2026-08-06 21:44 and verified on hardware the same run:**

```python
steering_angle = math.atan(self.WHEELBASE / radius)   # sign-correct everywhere
```

The clamp was also narrowed from +/-0.4 to +/-0.25 rad so neither lock clips
the servo. **The edit exists ONLY inside the container** at
`/workspaces/f1tenth/src/trajectory_following_ros2` (original kept as
`twist_to_ackermann_drive.py.bak`). `--symlink-install` means no rebuild was
needed — but it is **lost on the next image pull unless upstreamed to the
`trajectory_following_ros2` repo.** That is the single most important
follow-up, and it belongs to whoever owns that repo.

A/B proof, same goal both times: BEFORE `/drive` steering was constant **+0.4**
for all 110 messages (car yawed +42 deg LEFT, 0.55 s of motion); AFTER it spans
**[-0.25, +0.25]** and tracks the path (car yawed -44.8 deg RIGHT, drove
2.628 m).

**Second, separate defect in the same node:** the +/-0.4 rad saturation is
hardcoded and exceeds the car's real left-lock limit of 0.257 rad (from
`steering_angle_to_servo_offset: 0.56`). It is **not** affected by the
`max_steering` launch arg — that only scales the joystick. It should be a
parameter sourced from `vesc.yaml`. Until then even a sign-correct left command
can drive the servo past its stop.

### Resolved: the 5.45 s stop was a consequence, not a separate bug

`cmd_vel` ran t=240.34..245.79 then stopped dead; the planner stopped
replanning at the same moment, so the whole BT terminated. **No WARN or ERROR
was logged by any Nav2 server** — so it was not an abort, a collision, the
progress checker (`movement_time_allowance` is 100 s) or goal-reached
(tolerance 0.25 m, car was ~3 m away). It is invisible because the stack runs
`log_level:=warn` and SUCCEEDED/CANCELED log at INFO.

With the steering fixed, the very next run commanded for **7.6 s** and drove
the full path to the goal, so the 5.45 s cutoff was the robot driving off its
own path under inverted steering — not an independent defect. No further
investigation needed.

## Traps that cost time tonight — do not repeat

- **`handoff/head_1912.tar.gz` is STALE.** It does *not* contain the AMCL seed
  fix, contrary to what the older handoff below claims: extracting it resets
  `localizer_amcl.yaml` `initial_pose` to (0,0,0). Regenerate it from real HEAD
  before trusting it. Workaround used: publish `/initialpose` directly with
  **`stamp: {sec: 0, nanosec: 0}`** (tf2 reads 0 as "latest available", so it
  cannot extrapolate). Do NOT use `seed_initialpose.py` live — it hardcodes
  `use_sim_time: True` for bag replay.
- **The Jetson and the VESC are on SEPARATE batteries.** `voltage_input` on
  `vehicle/sensors/core` is the **drive pack only** and stayed 12.1-12.3 V all
  night. The Jetson has its own battery, no ROS topic reports it, and it is
  what died mid-run at 21:28 ("No route to host"). A sudden total SSH loss is a
  Jetson power symptom first, not a network one.
- **RViz goes to the laptop, not `:0`.** gosling1 is headless; `xrandr -d :0`
  reports 1920x1080 anyway, which is misleading. The operator's X server is on
  the laptop over SSH. Recipe that works (container is `--network host`, so it
  shares the host loopback and can reach the SSH tunnel directly):
  ```bash
  xauth nlist :10 | sed -e 's/^..../ffff/' | xauth -f /mnt/f1tenth_ssd/shared_dir/.rviz.xauth nmerge -
  docker exec -d -e DISPLAY=localhost:10.0 -e XAUTHORITY=/mnt/shared_dir/.rviz.xauth $C bash -lc \
    'source ...; rviz2 -d /mnt/shared_dir/nav2_live.rviz --ros-args -r /tf:=/gosling1/tf -r /tf_static:=/gosling1/tf_static'
  ```
  Check the display number with `ss -ltn | grep :60` — it changes per session.
  `config/f1tenth.rviz` **cannot** be used to send a goal: fixed frame is
  `odom` and it has no map/plan displays. Use `/mnt/shared_dir/nav2_live.rviz`
  (staged, fixed frame `map`, map + costmaps + plan + goal tool). Note
  `local_plan` stays empty — that is a DWB topic and the controller is RPP,
  which publishes `lookahead_point` / `plan_smoothed` instead.

## Better dry-run pattern than `60_nav2_test.sh --dry-run`

Bring the stack up in **full live config** but with
`command_gate_require_enable:=True`. The gate starts CLOSED and is the sole
publisher of `vehicle/ackermann_cmd`, so nothing reaches the VESC, while
everything upstream — including `twist_to_ackermann`, which `--dry-run`
*disables and therefore never tests* — runs exactly as in the live run. Going
live is then one service call, no second bring-up:

```bash
ros2 service call /gosling1/command_gate/set_enabled std_srvs/srv/SetBool "{data: true}"
```

A closed gate publishes **nothing at all** on `vehicle/ackermann_cmd`; an open
gate at idle publishes **zeros at rate** (joystick holding the mux at priority
100). That difference is how you confirm the gate state — never `ros2 topic hz`.

Staged on the SSD and all working: `nav2_launch.sh` (the exact live-config
bringup line), `warmstart.sh <container>` (HEAD -> stack -> lifecycle-ACTIVE
gate -> duplicate/rate/gate checks -> recorder, one shot, ~35 s to READY),
`rec_nav2.sh <label>` (records map + costmaps + plan + goal + chain, with the
transient-local QoS override that `rec.sh` lacks), `analyze_nav2.py`,
`nav2_live.rviz`.

## Next session, in order (2026-08-06 list — SUPERSEDED)

Kept only as a record of what that session queued. Items 1 (upstream the
`twist_to_ackermann` fix) and 4 (regenerate the stale HEAD tarball) are DONE —
see the 2026-08-07 block at the top of this file. The rest survive there in
the same order. **Use the top block, not this one.**

## Operator's asks for the next test session (2026-08-06 22:05)

### A. CPU profiling of the Nav2 stack — NOT captured yet

**No CPU data exists for either run.** `warmstart.sh` does not sample it and
the bags do not contain it, so nothing can be said about Nav2's cost from what
was recorded. Do not infer it from the earlier "planner_server 94 %" note —
that was measured during `on_configure` (global costmap build), not steady
state.

Why the operator wants it: the depth-based obstacle source was disabled at some
point purely for CPU, and both costmaps are currently **LiDAR-only** —
`observation_sources: scan  #depth_scan` at `config/nav2_params.yaml:353`
(local) and `:459` (global). With composed nodes and the current software that
tradeoff may no longer be necessary, so the question is whether `depth_scan`
can be re-enabled.

**How to capture it** — add to `warmstart.sh` after the health gate, and sample
again *during* a goal, since idle and driving differ:

```bash
# per-process, steady state and under a goal
top -b -n1 | grep -E "component_container|planner_server|controller_server"
# composed nodes hide inside the container process, so also thread-level:
top -b -n1 -H -p $(pgrep -f component_container_isolated | head -1)
pidstat -p ALL 1 5    # if sysstat is available
tegrastats --interval 1000   # Jetson-wide CPU/GPU/EMC, the honest system view
```

Take a baseline LiDAR-only, then re-enable `depth_scan` in both costmaps and
repeat the same goal. Compare controller-loop miss rate (`controller_server`
logs "Control loop missed its desired rate") rather than raw CPU % — that is
the number that actually matters.

### B. Controller performance and harder maneuvers

Current values, all in `config/nav2_params.yaml`:
`desired_linear_vel: 0.5` (:130), `xy_goal_tolerance: 0.25` (:109),
`use_rotate_to_heading: false` (:151), **`allow_reversing: true` already set**
(:152). Controller is RPP.

Planned progression — one variable at a time, re-running the same known-good
goal between changes so a regression is attributable:

1. Raise `desired_linear_vel` 0.5 -> 0.8 m/s. Watch for controller-loop misses
   and for the LiDAR's ~8.6 Hz becoming the limiting factor: at 0.8 m/s the car
   covers ~9 cm per scan, so the local costmap ages meaningfully between
   updates.
2. Tighten the stop distance: reduce `xy_goal_tolerance` toward 0.10-0.15 m and
   confirm the goal still formally SUCCEEDS (see item 3 below — this is the
   same 0.38 m question).
3. Reversing: `allow_reversing` is on but has never been exercised. Place a
   goal behind the car. Note the VESC has a **562 ERPM deadband** (from the MPC
   findings) which will bite hardest at the low speeds reversing uses.
4. Obstacles in the path: box in the lab, confirm the local costmap inflates it
   and RPP steers around; then a dead end to exercise the recovery subtree
   live (it has only ever been seen fire on a bag replay).
5. Remember the **asymmetric servo**: left lock 0.257 rad vs right 0.343. A
   left-heavy course saturates earlier than a right-heavy one and will look
   like a controller failure. The 0.56 -> 0.5 offset recalibration is the real
   fix and is scheduled for 2026-08-08/09.

### C. log_level was NEVER actually raised to info

Worth being precise, because the previous session's summary was ambiguous: the
successful run was launched with **`log_level:=warn`**, same as every other run
(`nav2_launch.sh` hardcodes it). A runtime
`bt_navigator/set_logger_levels` call was drafted but dropped when the command
was rewritten, and it never executed. So there is still **no INFO-level
evidence** of the goal's terminal state, which is exactly why the 0.38 m vs
0.25 m question is open. Change `log_level:=warn` in
`/mnt/shared_dir/nav2_launch.sh` to `info` before the next run, or send goals
via `nav2_goal_probe.py`, which captures the action result code directly.

---

**Updated 2026-08-06 ~20:50 EDT.** Everything below the first `---` is older
context and still valid.

## The job this session: the first LIVE Nav2 goal

The car has still never driven under Nav2. `testing_checklist.md` §9 has never
been ticked. Everything else is ready and the battery has some left, so this is
the one task — do not get pulled into the MPC work, which is parked until after
this run.

### Bring-up (the container is DOWN — cold start)

The whole sequence is in "Exact sequence that worked" below and it is now
routine: ~4 min to a healthy stack if you do not re-derive it.

```bash
# HOST first — jetson-containers only adds X11 mounts when DISPLAY is set in ITS
# env, and over ssh it is not (bug-130). Skip this and the RealSense dies.
export VEHICLE_NAME=gosling1 DISPLAY=:0 XAUTHORITY=$HOME/.Xauthority
rm -f /tmp/.docker.xauth
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge -
chmod 644 /tmp/.docker.xauth
rm -f /tmp/car_run.fifo; mkfifo /tmp/car_run.fifo
setsid bash -c 'sleep infinity > /tmp/car_run.fifo' </dev/null >/dev/null 2>&1 &
setsid script -qfc "bash $HOME/bolus_ws/f1tenth_launch.sh" /dev/null \
    < /tmp/car_run.fifo > $HOME/car_container.log 2>&1 &

# verify INSIDE the container (env alone does NOT catch it)
docker exec $C bash -c 'ls -l /tmp/.X11-unix/X0 /tmp/.docker.xauth'

# stage HEAD — the image ships an older f1tenth_launch
docker exec $C bash -lc 'cd /workspaces/f1tenth/src/f1tenth_launch && \
  tar xzf /mnt/shared_dir/handoff/head_1912.tar.gz'
```

**Re-build the staging tarball if the repo moved on**: copy the worktree to
local disk first, then tar — a `tar` straight off the OneDrive path fails with
"file changed as we read it". `head_1912.tar.gz` on the SSD is HEAD as of
2026-08-06 19:12 plus the AMCL seed fix; check `git log` before trusting it.

**Ready-made helpers already on the SSD** (`/mnt/f1tenth_ssd/shared_dir/`),
all written this session and all working: `hz3.sh` (one-pass rate check vs
baseline — **sources the workspace overlay, which matters: without it
`vesc_msgs` is undefined and `ros2 topic hz` reports DEAD on healthy topics**),
`abs_pose.py` (absolute pose in every frame + dynamic TFs), `pose_drift.py`
(stationary drift per source), `cleanup.sh` (clean teardown — see below),
`rec.sh <label>` (stack-side recording), `triage2.py` / `deadband.py` /
`stops.py` (bag analysis).

### Then run the Nav2 test

`60_nav2_test.sh --dry-run` first, then live with a hand on the joystick;
releasing R1 is the override. Detail and the two traps are in the older section
below and in `NAV2_OFFLINE_RESULTS.md`. Recap of what matters:

- Watch `vehicle/ackermann_cmd` **values**, not `ros2 topic hz` — a closed gate
  publishes zeros at full rate.
- `60_nav2_test.sh` sets `map_tf_publisher:=ekf`, so the map EKF and not AMCL
  broadcasts `map->odom`; pass `map_frequency:=30.0` (launch default is 10.0).
- Gate readiness on lifecycle **ACTIVE**, not on topics/action servers existing
  (bug-126), and remember a goal must be a *pose* (bug-128 — use
  `goal_poses_from_bag.py`, not `truth_*.csv`).
- Start the stack-side recorder before the run: `rec.sh nav2live`.

## Fixed this session, do not re-litigate

**The AMCL seed (bug-138).** `localizer_amcl.yaml` seeded `initial_pose` at
(0,0,0) but **the map frame is not aligned with the car's parking spot** — the
physical origin of the recorded runs is map **(+0.445, -0.575, -79.8 deg)**. So
a car parked at the origin that reports a big non-zero map pose is *correct*,
not drifting. Seed is now that pose; verified live, `amcl_pose` comes up at
exactly (0.445, -0.575, -79.80) with no motion needed.

Two traps this cost a whole session to untangle, worth not repeating:
AMCL is **motion-gated** (`update_min_d` 0.05 / `update_min_a` 0.1), so parked
it never runs a laser update, `amcl_pose` never republishes, and re-seeding by
hand via `/initialpose` looks like it does nothing. And
`truth_mapping_drive_170025.csv` is the `map->odom` *correction* over the run,
not the start pose — its first row (-25.33 deg) and its decay to zero are both
misleading if read as ground truth.

**Stationary drift is NOT a problem.** Measured parked: `odometry/local`
2.5 mm and **-0.07 deg/min**. The yaw fix (`imu0_config` yaw -> false) holds.

**`CLAUDE.md` corrected**: `pose0` is `relative: false` (was documented as
`true`), plus a note on the `localizer_amcl.yaml` bullet.

### Teardown, because SIGINT alone does not work

`kill -INT` on `ros2 launch` leaves ~10 orphaned nodes running — including
`ackermann_mux` and `ackermann_to_vesc_node`, which then **duplicate the
actuation path** on the next launch. Use `/mnt/shared_dir/cleanup.sh`; it
SIGINTs the launch, then TERM/KILLs the node list, then verifies zero. Note it
keeps the node-name patterns *inside the file* on purpose — `pkill -f` with the
names in the caller's argv matches and kills its own shell.

Also: **close bags with SIGINT** (`pkill -INT -f "bag record"`) and confirm
`metadata.yaml` exists. A killed recorder leaves an unreadable bag. And copy
anything from container `/tmp` to `/mnt/shared_dir/` before teardown — the
launcher uses `--rm`.

## MPC: parked until after the Nav2 run, but two results are banked

Both from `stackside_20260806_203218_figure8` (see its `FINDINGS.md`, and
`handoff/FIGURE8_STACKSIDE.md`):

1. **The mux and command gate are exonerated, twice.** On both the 19:37 run
   and the figure-8, `ackermann_drive` and `vehicle/ackermann_cmd` are
   identical — same counts, timestamps and steering distribution. Heartbeat
   worst gap 0.114 s vs the 0.5 s timeout.
2. **The figure-8 indicts the servo calibration.** A symmetric maneuver
   produced asymmetric saturation: **66.1 % of left commands (388/587) exceeded
   the -14.7 deg servo limit vs 41.5 % of right commands (258/621)** past
   +19.6 deg. Only the `steering_angle_to_servo_offset: 0.56` explains that.
   Recalibrating toward 0.5 is now the primary fix, not a tidy-up — **planned
   for the weekend of 2026-08-08/09**. Re-run a figure-8 after: if 66/41
   converges the offset was the story; if it holds, the *gain* is wrong.

**OPEN, bug-139 — the car repeatedly stopped mid-drive while R1 was held.**
Operator-reported on the figure-8. The stack did not do it: R1 was held once
continuously for 64.7 s, there was exactly one R1-held-but-zero-command episode
(t+95.2, 4.0 s) and `/drive` was itself zero during it. The low-speed deadband
is real but too mild to explain it — 1.5 % of window samples had cmd > 0.05 m/s
with |ERPM| < 100, in two episodes of 0.35 s and 0.95 s; below 0.2 m/s tracking
degrades (at 0.1 m/s only 50 % of samples turn the motor). **Next step: plot
commanded speed vs time on `vehicle/ackermann_cmd` over t+34.5..99.2 and count
sub-0.2 m/s dwells** — the hypothesis is the MPC itself commanded near-zero
speed repeatedly. This is an MPC-side question; we own only the calibration.

## Battery

Dipped to **11.90 V under load** from 12.30 V resting on the figure-8. That is
ordinary sag for a 3S pack and is **not** currently a fault — it was flagged
conservatively. Deferred deliberately. Check resting voltage before the Nav2
run and stop the session if it falls below ~11.4 V resting; if the car goes
dead-stick while every command topic still looks healthy, suspect the VESC
serial EIO abort (bug-068) rather than the pack.

---

**Older handoff (2026-08-06 ~15:40 EDT).** The bag-driven Nav2 test below is **DONE**:
Nav2 plans, controls and recovers correctly, and it turned up two real defects,
both fixed and re-measured. Full write-up: **`NAV2_OFFLINE_RESULTS.md`**.
`deliverables/20260805/` is still frozen v1 with checksums verified.

## START HERE: the first live Nav2 goal, when the car is free

**Everything below the line is bag replay. The car has still never driven
autonomously.** `testing_checklist.md` §9 has never been ticked. What now exists
to make that safe:

- `60_nav2_test.sh --dry-run` is finally honest. It passed `cmd_vel_topic` to
  the velocity smoother only, while `nav2_behaviors` published recovery velocity
  straight onto `cmd_vel` — measured 0.5 m/s escaping the diversion on
  2026-08-06 (bug-125). `nav2_navigation.launch.py` now remaps `cmd_vel` on
  `behavior_server` too, and the A/B is in `NAV2_OFFLINE_RESULTS.md`. **A dry run
  before this fix would have moved the car the moment a recovery fired.**
- `60_nav2_test.sh` now passes `launch_twist_to_ackermann:=True` in live mode
  (`False` under `--dry-run`). Without it Nav2's `cmd_vel` had no route to
  `drive` and the car could not move under Nav2 at all. Verified the argument
  reaches `vehicle.launch.py` through bringup's launch-config inheritance
  (`/gosling1/twist_to_ackermann_converter` appears); its `cmd_vel -> drive`
  conversion is still untested on hardware.

Order for the live session: dry run first, then live with a hand on the
joystick — releasing R1 is the override. Watch `vehicle/ackermann_cmd`, not
`ros2 topic hz`, and note `60_nav2_test.sh` sets `map_tf_publisher:=ekf`, so it
is the map EKF and not AMCL that broadcasts `map->odom` there; pass
`map_frequency:=30.0` for the rate `LOCALIZER_FOLLOWUPS.md` §6 was measured at
(the launch default is 10.0).

### What the bag test settled, and what it could not

Measured on `mapping_drive_170025`, domain 51, no vehicle — three goals:
accepted in 1-2 ms, first plan in **0.00-0.05 s** against a 5 s budget,
`cmd_vel_nav2` continuous at **20 Hz** bounded to the configured ±0.5 m/s, the
recovery subtree fired and completed, nothing crashed or hung. planner_server
peaks ~100 % **during on_configure only** (costmap build) and runs at 2.5-5.5 %
while planning — that is the answer to the "94 % with no goal pending" note.

It could NOT test anything closed-loop: the bag drives the pose, so the
controller can never reduce its own error, `SUCCEEDED` only means the bag's
trajectory passed inside the goal tolerance, and
`RegulatedPurePursuitController detected collision ahead!` is the expected
consequence of a replayed pose diverging from the commanded path.

Re-run it any time (~4 min, no hardware):

```bash
export ROS_DOMAIN_ID=51    # NOT 42 — the offline localization scripts use it
export CYCLONEDDS_URI=file:///mnt/shared_dir/cyclonedds_offline_lo.xml
export MAP_ROOT=/mnt/shared_dir/maps/20260805
export BAG_ROOT=/mnt/shared_dir/bags/20260805
cd /workspaces/f1tenth/src/f1tenth_launch/scripts/live_runs
./61_nav2_offline.sh --bag $BAG_ROOT/mapping_drive_170025 --goal-timeout 30
```

### Three traps this session paid for — read before writing a test like it

- **Goals must be poses.** `maps/*/truth_<bag>.csv` is the `map->odom`
  TRANSFORM, not a robot pose; on this bag it stays inside a 0.3 m ball around
  the origin, so goals drawn from it sit on top of the robot and Nav2 reports
  success without planning (bug-128). Use `goal_poses_from_bag.py`.
- **Wait for ACTIVE, not for topics** (bug-126). Costmap topics and action
  servers exist from CONFIGURE, but `nav2_util::SimpleActionServer` rejects every
  goal while inactive **and logs nothing at all**. Gate on `<node>/get_state`.
- **`ps -o pcpu` is a lifetime average, and `sleep infinity` reaps nothing**
  (bug-127). Zombies from earlier runs kept their names and frozen CPU figures
  and looked exactly like two extra nav2 stacks. `docker run --init`, sample with
  `top -b -n2`, skip state `Z`.

Container for offline work: `f1t_nav2_0806` on domain 51, HEAD staged over
`src/` from `/mnt/shared_dir/handoff/nav2_head_0806.tar.gz`. Regenerate that
tarball from HEAD if you change anything, and verify what you extracted.

### Still deferred to when the car is free

**Item 1 (live bringup) and item 2 (the first Nav2 goal)** below. `A` (seeding,
`bab2172`) and `B` (the motion model, `58a700a`) are both DONE and the offline
work on `ekf_map` has run out of levers worth pulling.

Runs kept for comparison: `bags/20260805/loc_ekfseed_*` (A's result),
`loc_ekflocal_*` (B's, current HEAD — now on all three bags).

### Read this before the next live session

- **bug-115 — the seed is racy.** B's cross-bag pass caught `figure8_172338`
  starting at the origin despite the seeder reporting success; the other two
  bags seeded fine. 2 of 3. Might be offline-only (`reset_on_time_jump` versus a
  bag's backwards clock jump) or might be a subscription-matching race that also
  bites live. Not separated. **Check `first map pose` against the seed on every
  run** — it is one line of `check_map_frame.py` and it is the only detector.
- **bug-116 — always check for an already-running stack before a bringup.** A
  1h40m-old `teleop.launch.py` was still up on gosling1 on 2026-08-06; a bringup
  on top of it put **two `vesc_driver_node`s on one serial port**, which is what
  the `Out-of-sync with VESC, discarding N bytes` flood is. Strong candidate for
  the same root cause as bug-068's dead-stick. Also: `kill -9` on a `ros2 launch`
  parent **orphans every child**, and SIGINT to a launch started with
  `docker exec -d` (no pty) does not reliably stop it — use the pty recipe.
  When sweeping up afterwards, note that two nodes evade the obvious
  name-greps: `joy_teleop` runs as `python3 .../joy_teleop/joy_teleop`, and the
  EKFs run as `robot_localization/ekf_node` (the *node* name `ekf_odom_node` is
  only a `--ros-args` value). Both survived a sweep on 2026-08-06 that looked
  complete. Verify a teardown by publisher count on a topic the stack owns, not
  by `ros2 node list` — that disagreed with the publisher counts in **both**
  directions during the same session.
- On **one clean** bringup the stack is correct: every nav2 server singleton,
  `odometry/local`, `odometry/global` and `vehicle/ackermann_cmd` all at exactly
  one publisher. The duplication above was operational, not a launch defect.
- `60_nav2_test.sh` does **not** pass `launch_twist_to_ackermann:=True`, so in
  its live (non-`--dry-run`) mode Nav2's `cmd_vel` never reaches `drive`. Add it
  before expecting the car to move under Nav2.
- The bench run used `map_frequency`'s launch default of **10.0 Hz**
  (`odometry/global` measured at 10.002 Hz). The 30 Hz that B was measured at is
  not what a live bringup gives you — pass `map_frequency:=30.0` if you want it.

**Watch this on the first live run:** the seeding fix is verified on bags but not
on hardware, and it changes live behaviour. RViz's "2D Pose Estimate" button now
resets `ekf_map` as well as relocating AMCL — that is the intent, but it is the
first time that button does two things. Expect `odometry/global` to jump when
someone presses it mid-run.

### A. Fix bug-111 — seed `ekf_map`  ✅ DONE 2026-08-06 ~10:15, commit `996d33c`

Fixed with a **remap, not a node**: `set_pose` is the same message type as
`/initialpose`, and robot_localization's header calls it *"usually published from
rviz"* — the library always meant the RViz pose tool to seed it, only the topic
name differed. `ekf_map.launch.py` gained `seed_from_initialpose` (default
`True`) which remaps `set_pose` -> `initialpose`. One pose estimate now seeds the
localizer and the filter together, offline and live. Not applied to `ekf_odom`
(its `world_frame` is `odom`; a map-frame pose would be wrong there).

Verified: `first map pose` is now `(+0.446, -0.576, -79.78 deg)` against the seed
`(+0.445, -0.575, -79.82)` — 1 mm / 0.04 deg, and the full-run mean dropped
191.6 -> **69.6 mm**. **Smoothness did not improve** (step p95 38.7 mm vs AMCL's
15.7 mm), so AMCL remains the broadcaster and **B below is still the open
question.**

Still open in this area, and NOT to be written speculatively: if the
`particle_filter`'s automatic global initialization converges without anyone
publishing `/initialpose`, `ekf_map` is unseeded again. Forwarding a localizer's
first converged fix to `set_pose` needs a convergence criterion and a
fire-exactly-once latch — real logic, and it would be the **first actual node in
this otherwise pure-launch package**, so it is a deliberate decision for the
operator, not a default. Wait for the PF measurements first.

<details><summary>Original brief for A, kept for the API details</summary>

Verified in the image (`privvyledge/f1tenth:humble-devel-08052026`):
`robot_localization`'s `ekf_node` **does** expose two seeding paths, and neither
is `/initialpose`:

| | |
|---|---|
| topic | `set_pose`, node-relative -> `/gosling1/set_pose`, `geometry_msgs/PoseWithCovarianceStamped` |
| service | `set_pose`, `robot_localization/srv/SetPose` |
| YAML | **none** — there is no `initial_state` parameter in this build, so it cannot be seeded from a config file at all |

`seed_initialpose.py` publishes only to `/initialpose`, which nav2_amcl consumes
and `robot_localization` ignores. Extend it to publish the same pose to
`set_pose` as well, reusing the existing sim-time stamp-0 handling (the wall-clock
trap in `MAP_FRAME_DELIVERY.md` applies identically here). Sequence it after
`/clock` exists and after `odom->base_link` is available, exactly as the AMCL seed
already is.

**This gap is not offline-only.** RViz's "2D Pose Estimate" button also publishes
`/initialpose` and nothing else, so on the car `ekf_map` is equally unseeded and
equally silent about it. The durable fix belongs in the launch or a small node,
not only in the offline script.

Verify: `check_map_frame.py` should report `first map pose` at the seed
(`+0.445, -0.575, -79.82 deg`) instead of the origin, and the full-run and
`--skip 35` numbers should converge on each other.

</details>

### B. Give `ekf_map` a motion model that matches what it is differenced against  ✅ DONE 2026-08-06 ~10:55

**It worked, and it closed the whole smoothness gap.** `odom0` changed from
`vehicle/vesc_odom` (velocities) to **`odometry/local` fused differentially**, so
the map EKF's prediction is the same increments as the `odom->base_link` its
correction is differenced against. Full write-up: `LOCALIZER_FOLLOWUPS.md` §6,
bug-114. Run kept: `bags/20260805/loc_ekflocal_mapping_drive_170025`.

Steady state (`--skip 35`), same control, same map, same truth, same seed:

| | mean err | **step p95** | **step max** | inside 126 mm |
|---|---|---|---|---|
| AMCL direct — the target | 74.7 mm | 15.7 mm | 59.6 mm | 85.2 % |
| ekf_map + `vyaw` + seed (previous HEAD) | 84.3 mm | 38.7 mm | 280.2 mm | 80.5 % |
| **ekf_map + `odometry/local` (now HEAD)** | **74.6 mm** | **18.2 mm** | **64.0 mm** | **87.7 %** |

Whole run, unskipped: **62.3 mm mean, step p95 16.2 mm, 90.3 % inside the bar** —
beating AMCL's 64.7 mm / 87.3 %. The seed took: `first map pose`
`(+0.446, -0.576, -79.67 deg)` against the seed `(+0.445, -0.575, -79.82)`. The
unskipped mean being *lower* than the skipped one is the seed being nearly exact,
not a gap — a failed seed makes the unskipped number much worse, as 191.6 vs
85.2 mm did before A.

`ekf_map` is now a real option for the MPC — AMCL's smoothness, slightly better
accuracy, and a steady 30 Hz instead of the 8.5 Hz scan rate. **It is still not
the default broadcaster**: `map_tf_publisher` stays `amcl`, because this is one
bag replay. Switching the default is a decision for after the live test.

One residual, not chased: whole-run step max is 235.6 mm while the skipped run's
is 64.0 mm, so a single ~236 mm correction happens inside the first 35 s and
nowhere else — almost certainly AMCL's first real correction away from the seed.
A startup transient on a filter that is smooth in tracking.

`pose0_rejection_threshold` (2.0 Mahalanobis) and `sensor_timeout` (0.13 s against
an 0.118 s `amcl_pose` interval) remain untouched, deliberately. With the motion
model fixed they may not be worth touching at all — a filter that is no longer
fighting its own prediction rejects fewer poses.

### How to re-run either one (both are already done — this is for a re-measure)

A container may already be up — check `docker ps` for `f1t_offline_0806b`. If not,
recreate it and re-stage, because the image predates the fixes (see "Container and
staging" below). Then:

```bash
export ROS_DOMAIN_ID=42
export CYCLONEDDS_URI=file:///mnt/shared_dir/cyclonedds_offline_lo.xml
export MAP_ROOT=/mnt/shared_dir/maps/20260805
export BAG_ROOT=/mnt/shared_dir/bags/20260805
cd /workspaces/f1tenth/src/f1tenth_launch/scripts/live_runs
./51_localize_offline.sh --bag $BAG_ROOT/mapping_drive_170025 \
    --publisher ekf --map-frequency 30.0 --out loc_<name>_mapping_drive_170025

cd ../analysis
python3 check_map_frame.py $BAG_ROOT/loc_<name>_mapping_drive_170025 \
    --map $MAP_ROOT/rtabmap_2d_final.yaml \
    --truth $MAP_ROOT/truth_mapping_drive_170025.csv --skip 35
```

The full history, steady state (`--skip 35`):

| | mean err | step p95 | step max | inside 126 mm |
|---|---|---|---|---|
| **AMCL direct — the target** | **74.7 mm** | **15.7 mm** | **59.6 mm** | **85.2 %** |
| ekf_map as configured | 125.0 mm | 83.5 mm | 920.1 mm | 66.9 % |
| ekf_map + `vyaw` | 85.2 mm | 37.3 mm | 288.5 mm | 79.9 % |
| ekf_map + `vyaw` + seed | 84.3 mm | 38.7 mm | 280.2 mm | 80.5 % |
| **+ `odom0` = `odometry/local` (current HEAD)** | **74.6 mm** | **18.2 mm** | **64.0 mm** | **87.7 %** |

(Full-run means: AMCL 64.7 mm, ekf+vyaw 191.6 mm, ekf+vyaw+seed 69.6 mm,
current HEAD **62.3 mm**.)

Report **`correction step p95` and `max`** every time — smoothness is the
complaint, not mean error. Report the unskipped number too: since A landed the
two should agree, and a gap between them means the seed did not take.

Kept for comparison: `bags/20260805/loc_ekf30_*` (as configured),
`loc_ekf30nv_*` (`--no-vslam`), `loc_ekfvyaw_*` (vyaw only), `loc_ekfseed_*`
(vyaw + seeding), `loc_ekflocal_*` (**current HEAD**).

## Container and staging — READ BEFORE RUNNING ANYTHING

`privvyledge/f1tenth:humble-devel-08052026` was built 2026-08-05 14:19, which is
**before** commits `e974a93`, `e28b2b4`, `58d1ae0` and `bab2172`. Its
`/workspaces/f1tenth` is in the container filesystem, not a bind mount, so a
fresh container starts with a stale workspace — verified on 2026-08-06, it had
`pose0_relative: true`, `max_particles: 500` and no `fuse_vslam_global`.

`/mnt/shared_dir/handoff/fix_0806.tar.gz` holds the current versions of the six
changed launch/config files plus the live-run and analysis scripts. Extract it
over `/workspaces/f1tenth/src/f1tenth_launch` and re-verify; `install/` symlinks
into `src/`, so no rebuild is needed. **Regenerate that tarball from HEAD** if you
change anything, and verify what you extracted rather than assuming.

Container creation: the operator's `~/bolus_ws/f1tenth_launch.sh` is `-it --rm`
and dies with its terminal. For unattended work use the detached recipe in
`DRIVE_SESSION_HANDOFF.md` "Rule 1". No hardware is needed for A or B, so the
device/X11 flags do not matter for them — but **do not** reuse the offline
container for a live bringup; create that one with the operator's script.

## Also in flight

- **`particle_filter` is being measured by a separate Claude**, who will report
  back. Their brief is item 4 below. Do not duplicate that work. When their number
  arrives it is directly comparable to the table above only if they scored with
  `check_map_frame.py --truth truth_mapping_drive_170025.csv` on
  `mapping_drive_170025`; check that before comparing.
- Expect them to find the same seeding class of problem if they route through
  `ekf_map` — `pose0` is `amcl_pose`, so a PF would need rewiring there anyway.

---

# Background — the map-frame deliverable is BUILT and VERIFIED

**Recording is done (2026-08-05 16:54–17:40). Maps are built (18:00–19:55).
All three runs are localized into one map frame and the derived bags are
verified (20:30–21:10).** Do not re-run the drive session, do not rebuild the
map, do not re-localize.

**Read `MAP_FRAME_DELIVERY.md` first.** It has the artifacts, the accuracy
numbers and their caveat, the two corrections it made to the earlier handoff,
and how to reproduce. `MAP_BUILD_HANDOFF.md` is still current for the map build
itself but its "What's next" section is done and two of its claims are struck
through in place.

## What exists now

`/mnt/f1tenth_ssd/shared_dir/deliverables/20260805/` — three derived bags for
the LUCIO consumer, each carrying **both** requested shapes:
`/gosling1/tf` (map→odom only) and `/gosling1/pose_map`
(`nav_msgs/Odometry`, `map`→`base_link`, 30 Hz). No sensor data; play alongside
the originals. Each has a sidecar README.

| derived bag | pose_map | map→odom tfs |
|---|---|---|
| `mapframe_mapping_drive_170025` | 4403 | 1247 |
| `mapframe_figure8_172338` | 4651 | 1333 |
| `mapframe_loop_laps_173558` | 3088 | 886 |

All three PASS `verify_map_frame_bag.py`: header stamps byte-exact (sec and
nanosec) against the source, the two shapes agree to 0.0 nm, 30.01 Hz, stamps
monotonic, tf carries map→odom and nothing else.

**Accuracy: mean 65 mm / p95 144 mm** against RTABMap's own optimized graph on
the control run; 87% inside LUCIO's 126 mm bar. That is agreement between two
estimators sharing one LiDAR and one map, **not** absolute accuracy — quote it
with that caveat. Supporting evidence that does not share it: the three runs'
independently-localized start poses agree to 2 mm / 0.06°, loop closure improves
from 0.36–1.17 m (odom) to 0.04–0.08 m (map), and 100% of every path lies in
mapped free space.

Source bags and maps are unchanged, in
`/mnt/f1tenth_ssd/shared_dir/{bags,maps}/20260805/`.

## HARD CONSTRAINT: `deliverables/20260805/` is v1 and is FROZEN

Other Claudes on other machines — LUCIO (pixel→world) and the MPC project
(waypoints) — are consuming those exact files right now, and quoting the
accuracy numbers in `MAP_FRAME_DELIVERY.md`. **They cannot see this repo and
will not be told if the ground moves under them.**

All three tasks below can plausibly produce a better `map->odom`. That is a good
outcome and it is *not* permission to regenerate v1 in place.

- **Never write into `deliverables/20260805/`.** It is `chmod a-w` on gosling1,
  but that stops nothing: the container runs as uid 0 and root writes straight
  through it (tested). The rule is the protection, not the bits.
  `make_map_frame_bag.py` also refuses to overwrite an existing output — do not
  work around it with `rm -rf`.
- **`md5sum -c MD5SUMS.txt`** in that directory before trusting or re-quoting
  anything. That is how a silent change gets caught.
- A better result goes to **`deliverables/20260805_v2/`**, with v1 left intact
  and readable until every consumer has moved.
- **Beat v1 on the same control before claiming an improvement**:
  `mapping_drive_170025` scored against
  `maps/20260805/truth_mapping_drive_170025.csv`. v1 is **mean 64.7 mm /
  p95 143.5 mm / 87.3% inside 126 mm**. Anything that does not clearly beat that
  is churn.
- **Run `verify_map_frame_bag.py` on v2.** Passing it is what "delivered" means.
- **Then tell the operator, so the consumers can be told in writing** what
  changed and by how much. A silent swap is worse than no swap.

The same applies to `maps/20260805/` and `bags/20260805/`: v1's provenance
points at those exact files. Do not rebuild the map or re-record.

Full provenance — commit, config, seed, MD5s — is in `MAP_FRAME_DELIVERY.md`
under "Version and freeze status".

## The three assigned items are DONE (2026-08-05 22:40) — read `LOCALIZER_FOLLOWUPS.md`

All three were worked and measured. **v1 is untouched and still stands**: nothing
found here produced a better `map->odom`, so there is no v2 and no consumer needs
telling.

1. **RTABMap localizer — measured, does not work.** It had never been *able* to
   work: its TF remap was written `SetRemap(src=['/tf','/tf_static'], dst=[...])`,
   which launch concatenates into the single rule `-r /tf/tf_static:=tftf_static`,
   so it ran with an empty TF tree (bug-107, **fixed**). With TF fixed it still
   accepts zero loop closures: `rtabmap_final_nf.db` has an **empty visual
   vocabulary** (bug-108) and against `rtabmap_final.db` every candidate fails
   geometric verification (`Not enough inliers 0/15`). It publishes a perfectly
   steady identity `map->odom` while failing — 344 mm mean error, all of it
   un-removed odometry drift. Cost while achieving nothing: ~1 core and ~1 GB,
   against AMCL's 3-4 % of one core. Reviving it is a `Vis/*` tuning job.
2. **Both `ekf_map.yaml` items — resolved.** `pose0_relative` is now `false`.
   `odom1` (VSLAM `slam_odometry`) is switched by the launch instead of the file:
   new `fuse_vslam_global` arg, default off, set from `use_gpu AND
   localize_on_startup`, plumbed `localization.launch.py -> dual_ekf ->
   ekf_map.launch.py`. Verified in the container both ways.
3. **AMCL tuning — folded in, all of it.** The partial fold-in the last session
   proposed was measured and is *harmful*: sharpening the likelihood field while
   keeping 100-500 particles scores 345 mm, worse than the 267 mm it replaced.
   But the particle count turned out to cost **4.1 % of one core**, not the
   assumed CPU problem — so the whole tuned configuration went into
   `localizer_amcl.yaml` and was re-measured as the file the car will load:
   **63.9 mm mean / 147.4 mm p95 / 88.4 % inside the bar**.
   `localizer_amcl_mapframe.yaml` is left byte-for-byte unchanged as v1's
   provenance.

### What that leaves for the next session

Ordered. 1 and 2 are the ones that matter.

**1. Live tests — everything above is bag replay.** One bringup on the car covers
all of it:
- the folded `localizer_amcl.yaml`: does AMCL converge from its startup pose, and
  is `map->odom` steady at driving speed (not just on a 1 m/s replay)?
- `ekf_map` with the new `fuse_vslam_global` plumbing, `pose0_relative: false`,
  the `vyaw` fix (bug-112), the `initialpose` seed (bug-111) and the
  `odometry/local` motion model (bug-114). It is **still not** the map-frame
  broadcaster — AMCL is — but that is now a "one bag replay is not enough"
  decision, not a measurement against it (`LOCALIZER_FOLLOWUPS.md` §6). What to
  watch live is `odometry/global`: that it starts at the seed rather than the
  origin, and that it tracks `amcl_pose` smoothly at driving speed. **If it does,
  switching `map_tf_publisher` to `ekf` is the decision to make** — it buys a
  steady 30 Hz global pose instead of the localizer's 8.5 Hz staircase, which is
  the MPC's original complaint;
- `/dev/ydlidar -> ttyUSB0` after the 17:45 reboot, still unconfirmed.

**2. Send a Nav2 goal. The car has never driven autonomously.** `testing_checklist.md`
§9 "Send a navigation goal" has never been ticked, and it is now unblocked: there
is a map, a localizer that clears the bar, and a command gate whose R1 handover is
verified. Remember `launch_twist_to_ackermann:=True` when driving under Nav2 —
it defaults False because the MPC publishes `drive` directly, and
`60_nav2_test.sh` does not pass it.

**Status 2026-08-06 ~11:30: still not done, and no verdict.** A bench bringup
(car on a table, no battery) got as far as: all eight servers present exactly
once, `/map` published, both costmaps up, preflight clean. **No goal was sent**
— planning, control, the BT and the recoveries are untested. Do the bag-based
version described at the top of this file first; it needs no vehicle and will
surface the crash-and-hang class of bug before the car is on the ground.

**3. DONE 2026-08-06 — and after three fixes `ekf_map` IS a live option.
See `LOCALIZER_FOLLOWUPS.md` §6.** As configured it was far *worse* than AMCL
direct (step p95 83.5 mm vs 15.7, max 920 vs 59.6, 125.0 vs 74.7 mm mean). Three
real defects came out of it, all fixed: no angular-rate input at all
(`odom0_config` `vyaw` false, IMUs excluded by design — bug-112, worth
125.0 -> 85.2 mm); never seeded, starting at the origin for ~30 s at ~740 mm /
82 deg, silently (bug-111); and a motion model that did not match the transform
its correction is differenced against (bug-114, the big one). It now scores
**74.6 mm / step p95 18.2 mm / max 64.0 mm** against AMCL's **74.7 / 15.7 /
59.6**, and beats it whole-run (62.3 mm, 90.3 % inside the bar). v1 is untouched:
it never used this path, and a ~2 mm improvement is not worth a v2. **So the live
lever for the MPC's jerkiness is back — a steady 30 Hz `map->odom` instead of an
8.5 Hz staircase — pending the live test in item 1.** The offline smoothing for
LUCIO below is still the right answer for that consumer, which reads a finished
bag. The original reasoning, now superseded, was:


The consumer complaint is expected behaviour, not a defect: v1's `map->odom` is a
**piecewise-constant** correction at 8.5 Hz (the scan rate) applied to smooth
30 Hz odometry. Measured steps: **p95 14.6 mm, max 59.6 mm**. Differentiate that
for velocity and the steps become spikes. Three responses, in order:
- ~~**Measure `ekf_map` as the broadcaster.**~~ **Done — it is worse on both axes.**
  `51_localize_offline.sh` gained `--map-frequency` (the launch default is 10.0,
  *below* the 30 Hz it is meant to smooth) and `--no-vslam`; `check_map_frame.py`
  gained `--skip`. Results and the two defects: `LOCALIZER_FOLLOWUPS.md` §5.
- **Offline smoothing for LUCIO — now the ONLY remaining response to the
  jerkiness complaint.** They consume a finished bag, not a live stream;
  smoothing `map->odom` over the localizer's own corrections removes the steps with
  no filter tuning. A legitimate `v2` under the freeze protocol. (Being forwarded
  to that consumer directly.)
- **Do NOT rebuild the map at 0.025 m for accuracy.** Corrected in
  `MAP_FRAME_DELIVERY.md`: the arithmetic says it buys ~1 mm, because quantisation
  (sigma 14.4 mm) is nowhere near the dominant term in 64.7 mm.

**4. `particle_filter` — worth measuring, second.** Owned by a separate claude.
Brief for them: smoothness should improve (`mcl_hz: 40.0` is decoupled from the
scan rate, so it predicts between scans and emits a weighted mean rather than
AMCL's 8.5 Hz jumps); accuracy should NOT be expected to improve (same LiDAR, same
grid, same information). **First step is confirming the package is actually
installed** — `localization.launch.py` swallows `PackageNotFoundError` silently,
and this session is a lesson in what "never been run" is worth. Score with
`check_map_frame.py --truth truth_mapping_drive_170025.csv` so the number is
directly comparable to AMCL's 64.7 mm, and add a `--publisher pf` path to
`51_localize_offline.sh`.

**5. The VSLAM map, deliberately, once.** Nothing saves one today (`save_map`
defaults False). `localize_on_startup` is already gated on the map directory being
non-empty (`isaac_ros_visual_slam_realsense.launch.py:227`), but existence is not
correctness — gosling1's stale Sep 2025 map would have passed that gate and failed
every relocalization. **And `fuse_vslam_global:=True` needs more than "VSLAM is
localized in a saved map": it needs VSLAM's map to be CO-REGISTERED with the
RTABMap grid.** That has been asserted but never measured, and the two maps come
from different mappers and different sensors. So: one session that drives,
`save_map:=True`, re-runs with `localize_on_startup:=True` to prove relocalization,
and scores VSLAM's map-frame pose against the same truth CSV to settle whether the
frames agree. Until then `fuse_vslam_global` has no verified-correct `True` setting
and should stay off.

**6. Optional: revive the RTABMap localizer** by tuning `Vis/*` against
`rtabmap_final.db`. Only worth it if the 2D+3D single-frame property is wanted;
AMCL already clears the accuracy bar at a fraction of the cost.

## Also open

- **Hand the deliverables over**, with `MAP_FRAME_DELIVERY.md`. State in writing
  that the pose frame is `base_link` = **rear axle** — not `base_footprint`
  (33 mm down) and not `front_axle` (256 mm forward). A wrong choice there is a
  ~0.13 m error invisible to every check on our side.
- **Waypoints — NOT this session.** A separate Claude on the MPC project owns
  it. It is unblocked and is pure post-processing on `pose_map`; all three
  trajectories are already in one `map` frame.

## Open items carried forward

1. **Apply the udev fix to the other goslings.** `ydlidar-V2.rules` matches the
   VESC's USB ID and makes `/dev/ydlidar` point at the VESC, which kills the
   LiDAR *and* SIGABRTs the VESC driver (bug-073, root cause of bug-068).
   gosling1 is fixed; the rest are not. See `UDEV_YDLIDAR_VESC_COLLISION.md`.
2. **`vesc_driver` should catch the serial exception and reconnect** instead of
   aborting, and the safety chain should notice the actuator is gone (bug-068,
   still open as a code defect even though its trigger is fixed).
3. **Raise `MAX_STEERING` back to 0.34** in `00_env.sh` after the operator
   recalibrates `steering_angle_to_servo_offset` toward 0.5 (scheduled weekend
   of 2026-08-08). Temporarily 0.25. Related: `vehicle/vesc_odom` diverges
   1.6–1.7 m with correct path length, which points at the same calibration —
   and these runs show ~25° of odom yaw drift per 150 s.
4. ~~**Verify `/dev/ydlidar → ttyUSB0` survived the 17:45 reboot** on gosling1.~~
   **DONE 2026-08-06 ~10:55 — it did.** `/dev/ydlidar -> ttyUSB0` and
   `/dev/sensors/vesc -> ../ttyACM0`, which is the correct split (bug-073 is the
   case where `ydlidar` points at a **ttyACM**). RealSense D435i enumerates.
   Still unverified live, since this is device presence, not a working driver.
   Note the **DualSense was not connected** at that check (`/dev/input/js*`
   absent) — with no joystick the command gate's heartbeat never arrives and the
   gate stays closed, so a bringup started in that state will not actuate.
5. **`rtabmap_2d_overfiltered.yaml` names the wrong PGM** (`rtabmap_2d_final.pgm`
   with the overfiltered origin). Cosmetic; that file is marked do-not-use.

## If you do need to record again

`25_drive_session.sh` for the session, `DRIVE_SESSION_HANDOFF.md` for the
protocol and container rules. The two pre-flight checks that caught real
failures: **pack voltage and `fault_code`** from `vehicle/sensors/core`
(BEST_EFFORT QoS), and **`ls -l /dev/ydlidar`** must show a **ttyUSB**, never a
ttyACM. Ordering rule: **park at the start pose, *then* relaunch, *then*
record** — relaunching alone does not zero the EKF and VSLAM.
