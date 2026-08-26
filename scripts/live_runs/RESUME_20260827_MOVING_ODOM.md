# Resume — the EKF track is closed; the next open item needs the car to actually move

**Repo:** `f1tenth_launch` · branch `perf/config-tuning`
**gosling1:** `192.168.2.195`, direct `ssh gosling1@192.168.2.195` (no jump host)

Written 2026-08-26 ~16:00 EDT. **Replaces `RESUME_20260826_IMU_JUMP.md`**, which is deleted in the
same commit — both of its open items are done, and everything of its content that was still true is
carried forward below. Recover it from git history if you want the original wording.

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

## 2 · What is next, in order

### (a) The moving-odometry check — the real next item, and now unblocked

It has been "still open, unchanged" for weeks only because bug-244 blocked it: a filter that believes
it is doing 46 m/s produces confident, wrong control. That reason is gone.

Needs **battery power and a driven leg** — everything since 2026-08-25 has been a parked desk test.
`scripts/live_runs/odom_moving_check.py` is the tool. Two things to check on the same drive:

- `odometry/local` yaw drift while *moving*, which no parked test can reach. The parked figure is
  +0.04 deg/min.
- The **wheelbase change from 0.25 to 0.256 m** (2026-08-07) is still unverified on hardware. It
  biased `vesc_odom`'s kinematic yaw rate ~2.4 % against the frames it fuses into. CLAUDE.md has
  asked for a re-check of `odometry/local` yaw drift after the next drive ever since.

Read the battery and bring-up notes in §3 before booking the time — a driven leg costs drive battery,
and the Jetson runs off a separate supply that has no ROS topic.

### (b) One more cold launch for bug-248 — cheap, opportunistic, not blocking

The fix has **one** verification run. That is worth more here than it was for bug-244, because the
failure was deterministic (4 of 4) rather than 3-in-8, so a single clean run is not a coin flip. Still,
piggyback one more on the next bring-up you do for any reason:

```bash
bash /mnt/shared_dir/run_0826.sh run18_constdt 240
python3 /mnt/shared_dir/imujump.py /mnt/shared_dir/claude_bringup_0826/run18_constdt
```

Expect the largest attitude step to be well under 1 deg. Anything in the tens of degrees means the
parameter did not reach the node — check `ros2 param get /realsense_imu_filter constant_dt` first.

### (c) `max_steering` is still 0.34 at every entry point

0.314 rad is the symmetric no-clip value (`+24.0 deg` left / `-18.0 deg` right measured). Changing it
is a **cross-repo decision** — LUCIO's ego-MPC consumes the same limit. Unchanged today.

### (d) IMU bias remover — not started, and it now has a neighbour

`NEXT_CHAT_IMU_BIAS.md` and `docs/imu_bias_removal_spec.md` own this. Two things today's work changes
about it:

- `launch/filters/imu_filter.launch.py` now carries `constant_dt` in **both** madgwick blocks — the
  with-correction path and the without. When you swap in `imu_processors::ImuBiasRemover`, keep them
  in sync.
- The VESC chain is **not** affected and must not be assumed to be. `launch_imu_filter` defaults
  **`False`** in `vehicle.launch.py`, so `vesc_imu_filter` does not run at all today; the only
  madgwick node in a live stack is `/realsense_imu_filter`. If that chain is ever enabled, it needs
  its own `imu_filter_constant_dt` — `vehicle/sensors/imu/raw` measured 100.4 Hz, so 0.01, not 0.005.

That work is still blocked on `ros-humble-imu-pipeline` going into the **image build** (a separate
repo). It is unrelated to bug-248: bias removal takes out a static offset, and the jump was a step.

### (e) Deferred / for someone else

- `LUCIO_MAP_HEADING_NOTICE.md` — delivered 2026-08-25, no reply needed from this repo.
- The **uncommitted doc consolidation** in the working tree (content folded out of
  `BATTERY_SESSION_PLAN.md`, `MPC_BENCH_HANDOFF.md`, `NEXT_CHAT_ODOM_ROTATION.md`, with those three
  staged for deletion) is from an earlier session, was not reviewed here, and is deliberately still
  untouched. Decide on it on its own merits; do not fold it into an unrelated commit.

---

## 3 · Getting the robot back

The container is **not** persistent, and neither is anything in `/workspaces` — that is a container
layer. On 2026-08-26 the previous day's container was simply gone, taking the live bug-244 fix with
it. Assume this will happen again.

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
