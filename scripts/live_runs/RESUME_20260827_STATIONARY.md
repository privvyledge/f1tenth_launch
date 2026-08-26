# Resume — the EKF track is closed; the next three items are all stationary

**Repo:** `f1tenth_launch` · branch `perf/config-tuning`
**gosling1:** `192.168.2.195`, direct `ssh gosling1@192.168.2.195` (no jump host)

Written 2026-08-26 ~16:00 EDT, re-scoped ~16:50 after the operator picked the next chat's work.
**Replaces `RESUME_20260826_IMU_JUMP.md`**, which is deleted — both of its open items are done, and
everything of its content that was still true is carried forward below. Recover it from git history
if you want the original wording.

**Next chat does §2(a), §2(b) and §2(c). All three are stationary — a parked car and a warm
container, no drive battery.** The moving-odometry check is §2(e) and waits.

---

## 1 · Closed 2026-08-26 — do not re-open either of these

Full write-up with every number: **`BUG244_CLOSEOUT_20260826.md`**. Commits `b07af1c` (the fix) and
`7712160` (docs + tooling).

**`bug-244` — `ekf_odom` runaway. CLOSED.** Three cold launches (`run14/15/16`, 240 s each) in the
arm it diverged in — joystick disconnected, `command_gate` closed, `vehicle/vesc_odom` silent
(`Count: 0` in every bag). `/odometry/local` at t+30 s: 5.5e-06 / 8.2e-06 / 3.6e-04 m/s, against
32.7 m/s and pose (478, 140) in the pre-fix control. The counting argument (four passes against
~3-in-8, p~0.15) is the weaker one — the trigger fired in all three runs and the runaway did not
follow.

**`bug-248` — the attitude jump. FIXED and verified.** It was a *timestamp* defect: the RealSense
driver stamps its first 2-3 IMU samples seconds off, and `imu_filter_madgwick` with
`constant_dt: 0.0` took its integration timestep from them. The raw gyro moves 0.000-0.003 deg while
the filtered attitude steps 63-167 deg. Fixed by the new `imu_filter_constant_dt` launch argument
(default `0.0`), set to `0.005` from `realsense_d435i.launch.py`. `run17_constdt`: largest attitude
step over 217 s is **0.187 deg**, `yaw_drift.py` `odometry/local` **+0.04 deg/min**.

Three claims that were in circulation are now retired: the jump is **not** ~10 s into a launch (that
was the camera's startup delay), **not** intermittent (4 of 4), and **not** an attitude defect.

---

## 2 · Next chat: all three of these are stationary. Do not book drive time for them

Scoped by the operator 2026-08-26. **(a), (b) and (c) below need nothing but a parked car and a
warm container.** The moving-odometry check is §2.5 and waits for a session with battery time.

### (a) One more cold launch for bug-248

The fix has **one** verification run. That is worth more here than it was for bug-244, because the
failure was deterministic (4 of 4 launches) rather than 3-in-8, so a single clean run is not a coin
flip. Still, confirm it:

```bash
bash /mnt/shared_dir/run_0826.sh run18_constdt 240
python3 /mnt/shared_dir/imujump.py /mnt/shared_dir/claude_bringup_0826/run18_constdt
```

Expect the largest attitude step well under 1 deg (run17: 0.187 deg). Tens of degrees means the
parameter did not reach the node — check `ros2 param get /realsense_imu_filter constant_dt` first,
which must read `Double value is: 0.005`.

### (b) IMU bias remover — stationary. It is no longer blocked, and the spec's version is wrong

`NEXT_CHAT_IMU_BIAS.md` and `docs/imu_bias_removal_spec.md` own this. **Two blockers named in those
docs are stale as of 2026-08-26** — check these before re-deriving anything:

- **"The Jetson has no internet, so this cannot be `apt install` on the robot."** Retired. The ROS
  apt repo is reachable from inside the container: `apt-get update` succeeds and
  `apt-cache policy ros-humble-imu-pipeline` returns a candidate. You can install into the warm
  container and test **today**, without waiting on the image-build repo. The image-build change is
  still owed — a container install vanishes with the container — but it no longer gates the test.
- **"`imu_processors` is released for Humble at 0.5.2 and that tag builds `imu_bias_remover`."**
  The version actually available is **0.4.1** (`0.4.1-1jammy.20260804.210108`), not 0.5.2. It does
  ship the node — `dpkg -c` on the downloaded .deb lists
  `/opt/ros/humble/lib/imu_processors/imu_bias_remover_node`, `libimu_bias_remover.so` and an
  `rclcpp_components` resource-index entry — so the plan survives, but **every behavioural claim in
  the spec was written against 0.5.2 and must be re-checked against 0.4.1**, in particular the
  `bias` topic name and the `twist_is_zero_ || odom_is_zero_` stationary test. Read the installed
  headers, do not assume.

**How to verify it stationary — this is the part that needs care.** A parked re-measurement reads
~0 *whether or not the correction works*, because the zeroing path is active and the subtraction
path is not. So the parked test must be:

1. Subscribe to the node's **`bias` topic** and confirm it converges to the measured RealSense
   z-gyro bias, **-0.00214 rad/s** (reproduced today at -0.002173). That is the only stationary
   observation that discriminates.
2. `yaw_drift.py` 60 s parked before and after. `odometry/local` must stay in the documented band
   (+0.04 / +0.01 / +0.17 deg/min; today's reading was +0.04). **Stop and report if it regresses.**
3. Do **not** conclude the correction works from a parked yaw-drift figure alone.

The **staleness hazard is still real and still cannot be closed stationary**: the node's stationary
test has no timeout, so a velocity source that dies while reading "stopped" pins `angular_velocity`
at zero *forever, including while driving* — and the VESC driver on this car does abort on serial
EIO and go dead-stick while the command topics look healthy. Its velocity source must be
`vehicle/vesc_odom` (not `cmd_vel`, silent under teleop; not `odometry/local`, which closes a loop
with the IMU being corrected). **Kill `vesc_odom` while parked and confirm the node's behaviour, but
the driving half of that check is owed and must be carried to the first drive session.**

Scope discipline from the spec, unchanged: the RealSense chain only. **The VESC chain is spec step 4
and must not be bundled** — their effects on `odometry/local` are not separable after the fact.

Two things today changed for this work:

- `launch/filters/imu_filter.launch.py` now carries `constant_dt` in **both** madgwick blocks — the
  with-correction path and the without. Keep them in sync when you swap the node in.
- The VESC chain is **not** affected and must not be assumed to be. `launch_imu_filter` defaults
  **`False`** in `vehicle.launch.py`, so `vesc_imu_filter` does not run at all today; the only
  madgwick node in a live stack is `/realsense_imu_filter`. If that chain is ever enabled it needs
  its own `imu_filter_constant_dt` — `vehicle/sensors/imu/raw` measured 100.4 Hz, so 0.01, not 0.005.

### (c) Decide the uncommitted doc consolidation

In the working tree: content folded out of `BATTERY_SESSION_PLAN.md`, `MPC_BENCH_HANDOFF.md` and
`NEXT_CHAT_ODOM_ROTATION.md` into `NAV2_OFFLINE_RESULTS.md` and `rf2o_zero_velocity_brief.md`, with
those three staged for deletion, plus edits to `51_localize_offline.sh`, `61_nav2_offline.sh`,
`check_map_frame.py`, `BRIEF_PARTICLE_FILTER.md`, `MAP_BUILD_HANDOFF.md`, `MAP_FRAME_DELIVERY.md`.

It is from an earlier session, coherent, and looks finished, but it was never reviewed. It has now
survived two sessions untouched. Read it on its own merits and either commit it as its own commit or
discard it deliberately — **do not fold it into an unrelated commit**, and note that the three
deletions are already staged in the index, so a bare `git commit` sweeps them in. Commit with an
explicit pathspec.

### (d) `max_steering` — this does NOT need LUCIO, and the record is wrong about it

See §2.5 for the correction. This is a local change; it was mis-filed as cross-repo.

### (e) The moving-odometry check — waits for battery time

Unblocked for the first time (bug-244 was the blocker: a filter that believes it is doing 46 m/s
produces confident, wrong control), but it needs **battery power and a driven leg**, so it is not in
this chat's scope. `scripts/live_runs/odom_moving_check.py` is the tool. Two things to get on the
same drive:

- `odometry/local` yaw drift while *moving*, which no parked test can reach. Parked is +0.04 deg/min.
- The **wheelbase change from 0.25 to 0.256 m** (2026-08-07), still unverified on hardware. It biased
  `vesc_odom`'s kinematic yaw rate ~2.4 % against the frames it fuses into.

Also owed on that drive: the driving half of the bias-remover staleness check (§2b), and driving into
the steering limit both ways, which is the only way `servo_min` ever gets exercised.

---

## 2.5 · `max_steering`: the LUCIO coupling was overstated, and a delivered claim is untrue

Asked directly on 2026-08-26: *why do we need LUCIO's ego-MPC for this?* **We do not.** Checked in
source rather than repeated from the note:

There are **three** different steering limits in this repo and they are routinely conflated:

| what | value | who reads it |
|---|---|---|
| `max_steering` | **0.34** at all four entry points | `joystick.launch.py` only — it scales the joy_teleop steering axis. **Manual driving.** |
| `vesc_max_steering_angle` | `0.0` (default, i.e. off) | passed to the VESC driver nodes |
| `max_steering_angle` | **0.25**, hardcoded `vehicle.launch.py:320` | `twist_to_ackermann` — the **Nav2** path |

LUCIO's ego-MPC **publishes `drive` directly** and so goes through none of them; its limit is its own
`delta_bound`, landed in `LUSCIO_ROS` on 2026-08-09 as `(-18.0, 18.0)` **degrees** = ±0.314 rad.

So the coupling is **not** that LUCIO consumes our launch argument. It is that both sides must agree
with the same *measured mechanical* constant. Changing `max_steering` 0.34 -> 0.314 touches joystick
teleop scaling and nothing else — it is a local change that moves us *into* agreement with what
LUCIO already has. What genuinely requires telling them is **re-measuring** the limit: recentring the
servo horn, or re-running the gain calibration at the Stage 4a re-record, would make their box
constraint stale, and a solver planning past the mechanical stop looks healthy right up until the
driver's clamp eats the difference in a corner.

**CLAUDE.md is imprecise here** ("changing it is a cross-repo decision because LUCIO's ego-MPC
consumes the same limit") and should be corrected to say the *measured constant* is shared, not the
argument.

**And a delivered claim is untrue.** `LUCIO_REPLY.md` told them on 2026-08-08 that the
"post-recalibration steering limit" was **applied** and that "`max_steering` is now 0.314 rad". It is
not: all four entry points still read `0.34`, and the string `0.314` appears **nowhere** in `launch/`
or `config/`. Either it was never applied or a staged tarball reverted it (that has happened before).
**Fix the repo first, then decide whether the LUCIO side needs a correction** — their ±18 deg box is
still the right number for the hardware, so this is a bookkeeping error on our side, not a wrong
constraint on theirs. Note 0.34 clips right (servo 0.9492 vs the 0.92 bound) while 0.314 does not.

---

## 3 · Getting the robot back

The container is **not** persistent, and neither is anything in `/workspaces` — that is a container
layer. On 2026-08-26 the previous day's container was simply gone, taking the live bug-244 fix with
it. Assume this will happen again.

**The container dies with the terminal that started it.** `f1tenth_launch.sh` runs `jetson-containers run ... -it --rm`, so when the operator's SSH/MobaXterm session ends — including a laptop dying — the container exits and `--rm` deletes it. This happened twice on 2026-08-26. Budget the ~8.5 min bring-up plus a re-stage every time, or start it detached with the FIFO-pty recipe if the session needs to outlive the terminal.

**The operator starts the container**, from the Jetson desktop session, not from an agent SSH shell:

```bash
bash ~/bolus_ws/f1tenth_launch.sh
```

Then re-stage, from the host:

```bash
/mnt/f1tenth_ssd/shared_dir/stage_0826.sh <container-name>
```

- Tarball is **`f1tenth_stage_20260826b.tgz`**, md5 `84e4258ed4a4fa2a7bc9c837ff8f96f5`. It carries
  **both** fixes. The `0825` tarball predates bug-244's and the first `20260826.tgz` predates
  bug-248's — re-staging from either silently reverts one.
- The script verifies six things in the **installed** tree. Two of them are today's:
  ```bash
  ros2 param get /ekf_odom_node imu1_config          # entries 12,13,14 must be False
  ros2 param get /realsense_imu_filter constant_dt    # must read: Double value is: 0.005
  ```
  Check these on the **live node**, not in the source tree.

`/mnt/shared_dir` inside the container is `/mnt/f1tenth_ssd/shared_dir` on the host.

---

## 4 · Environment notes still current

- **X11 works through the operator's SSH forwarding**, not `:0`. On 2026-08-26 the container had
  `DISPLAY=localhost:10.0` (MobaXterm on the workstation) and `xdpyinfo` succeeded there while `:0`
  refused authorization. RViz and the RealSense GL init were both fine. Probe with
  `xdpyinfo -display <d>` before assuming a display problem.
- **DualSense** `10:18:49:9D:72:FC` is paired and trusted but normally left **disconnected** (a PS
  press reconnects). Connected, it publishes the heartbeat, `command_gate` opens, `vesc_odom` starts
  at ~50 Hz — **and the car can drive**. Mind the desk.
- **`command_gate_require_heartbeat:=False` does NOT hold the gate shut** — it collapses the logic to
  always-open. To keep it closed, leave the default `True` and disconnect the joystick.
- Bags: `claude_bringup_0826/run1{4,5,6}` (before the bug-248 fix, with raw **and** filtered camera
  IMU) and `run17_constdt` (after), ~90 MB each, 898 MB total. The 2026-08-25 set is in
  `claude_bringup_0825/`, of which `ekfdiag_run12_control` is the bug-244 onset — but it has **no**
  raw `/camera/imu`, so it cannot answer anything about the driver's stamps.
- `/mnt/data` inside the container is a container layer despite looking persistent.
- Docker's default **bridge** network is broken on this kernel; jetson-containers uses `--network host`.
- `jetson-containers` bind-mounts `/tmp/argus_socket`, a *socket* on R36.4.3 but a *directory* in the
  image. `run.sh` is patched; **a fresh checkout reintroduces this** (bug-242).
- CycloneDDS spams `ddsi_udp_conn_write ... failed with retcode -3` at absent peers — `192.168.2.140`
  (gosling5) historically, `192.168.2.194` on 2026-08-26. Harmless, shared config, but it will drown
  a `ros2 param get`; pipe through `grep -v ddsi`.

## 5 · Two traps that each cost a run on 2026-08-26

- **`ros2 launch` does not reliably die on a SIGINT to its process group here.** With a proper
  `setsid` group and `kill -INT -- -$PID`, all four runs left `component_container_isolated` alive
  after a 30 s wait, and one left `rviz2`. The parent exits and the script reports "done" while the
  composable container keeps publishing — so back-to-back runs are **not** independent trials. After
  every teardown, list processes, `kill -9` the survivors, and confirm the list is empty before the
  next launch. `26_cold_launch_imu_capture.sh` only *reports* the leftovers; treat that line as a
  required manual step (bug-249).
- **Never put `set -u` in a script that sources `/opt/ros/humble/setup.bash`** — it reads an unbound
  `AMENT_TRACE_SETUP_FILES`, the script exits before launching anything, and the status is **0**, so
  it looks like a completed run (bug-250).
