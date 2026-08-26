# Resume — `bug-244` is root-caused and fixed; the IMU defect underneath it is not

**Repo:** `f1tenth_launch` · branch `perf/config-tuning`
**gosling1:** `192.168.2.195`, direct `ssh gosling1@192.168.2.195` (no jump host)

Written 2026-08-25 ~21:50 EDT. **Supersedes `/mnt/d/f1tenth_handoff_20260813/RESUME_20260826_EKF_DIVERGENCE.md`**,
which is now wrong in its headline claim: it says the bug-244 trigger is unknown. It is known.

---

## Read this, then `scripts/live_runs/NEXT_CHAT_IMU_BIAS.md`.

---

## 1 · What closed on 2026-08-25 evening

### `bug-241` — the `ekf_map` seed race. **ACCEPTANCE MET. Done.**

Six clean cold launches put `map->base_link` within ~2 cm / 0.3 deg of the
`localizer_amcl.yaml` seed (0.445, -0.575, -84.5 deg), plus one control.

| run | `map->base_link` | verdict |
|---|---|---|
| 1 | (0.464, -0.558, -84.64) | PASS |
| 4 | (0.453, -0.572, -84.69) | PASS |
| 7 | (0.452, -0.577, -84.44) t+60 & t+416 | PASS |
| 8 | (0.447, -0.579, -84.52) t+60 & t+381 | PASS |
| 9 | (0.459, -0.559, -84.75) t+60 & t+330 | PASS |
| 10 | (0.464, -0.570, -84.49) t+60 & t+330 | PASS |
| 5 | ~identity, `seed_initialpose:=False` | control — the seed does the work |

Runs 2, 3, 6 are void (`ekf_odom` was diverging underneath). **Nothing further is needed here.**

> The car sat on a **desk at an arbitrary lab spot**, not the mapped parking spot. That is fine for
> this test — it is a plumbing test, and AMCL is motion-gated so parked it reports the seed wherever
> the car physically is. It does **not** validate that the pose is physically correct, and
> `heading_from_scan.py` will fail its ~0.85 fit gate anywhere but the real spot.

### `bug-244` — `ekf_odom` runaway. **ROOT-CAUSED AND FIXED. Needs 2-3 more launches to close.**

The onset was finally captured, in `ekfdiag_run12_control`, because the recorder was started before
the launch. What it shows:

```
t+9.81s   camera/imu/filtered orientation jumps 82.56 deg   <-- discontinuity
t+9.85s   odometry/local starts accelerating                 <-- 40 ms later
          ramp = 9.34 m/s^2   (g = 9.807, i.e. 95% of gravity)
t+13.8s   attitude re-converges to within 12 deg of settled
          ...but the filter is already gone and nothing pulls it back
```

That 82.56 deg jump is **the largest attitude step of the entire 155 s launch by a factor of 90**
(next largest: 0.91 deg). It is not noise.

**Mechanism.** `ekf_odom.yaml` fused `imu1` linear acceleration with
`imu1_remove_gravitational_acceleration: true`. `robot_localization` removes gravity by rotating the
measured acceleration by **the IMU's own orientation quaternion**. An 82 deg attitude error leaves a
~1 g residual, integrated straight into velocity — exactly the measured 9.34 m/s^2. `imu1` was the
**only** acceleration input; `imu0`'s ax/ay/az are all `false`.

**Why it stuck.** rf2o's 5.0 Mahalanobis gates reject the measurements that would recover the state,
and `vehicle/vesc_odom` — the one ungated input that would assert "stopped" — is silent while parked
(`vesc_to_odom` returns early until a servo command arrives). Previously a hypothesis; **confirmed**
by the run-12 control arm.

**Why intermittent.** It depends on whether the jump lands while `ekf_odom` is fusing. Consistent
with the observed ~3-in-8.

**The fix, applied and committed:** `config/localization/ekf_odom.yaml`, `imu1_config` last row
`[true,true,true]` -> `[false,false,false]`. Acceleration off, **angular rates deliberately left on**
(with `imu0` yaw disabled they are the only gyro-rate input to yaw).

**Verification, run 13**, in the exact arm that had just diverged (joystick disconnected, gate closed,
`vesc_odom` silent):

| | run 12 (before) | run 13 (after) |
|---|---|---|
| imu1 accel fused | yes | no |
| t+9.85s | onset, 9.34 m/s^2 ramp | — |
| t+30s | 27.9 m/s, pose (2697, 882) | **-5.3e-06 m/s**, pose (-0.003, 0.005) |

`yaw_drift.py`, 60 s parked, as CLAUDE.md requires after any imu0/imu1 change — **passes**:
`odometry/local` **-0.04 deg/min** (documented range +0.04 / +0.01 / +0.17); VESC z-gyro bias
+0.003971 rad/s and RealSense -0.002215 rad/s both reproduce their recorded values.

> **This is ONE trial against a ~3-in-8 failure mode — p~0.6 of passing by luck.** What makes it more
> than a lucky run is that it confirms a *measured mechanism*, not a correlation. Still: **2-3 more
> cold launches with the joystick DISCONNECTED** before bug-244 is called closed. Run them the same
> way — recorder started first, check `odometry/local` twist at t+30 s.

### `bug-245` — cuVSLAM autosave. **Verified on hardware.**
`/mnt/data/maps/nvidia/vslam_map` did not exist after two full launch/Ctrl-C cycles. Nothing is
autosaved, so repeat launches are genuinely independent trials.

---

## 2 · The open defect this leaves — `bug-248`

**`camera/imu/filtered` emits an 82.56 deg discontinuous attitude jump ~10 s into a cold launch, and
that is still happening.** `ekf_odom` merely stopped listening to the acceleration it corrupted.

Still consuming that stream:
- **Isaac VSLAM**
- the rest of the **IMU filter chain**
- **`ekf_odom`'s own `imu1` angular rates** (`vroll/vpitch/vyaw` are still `true`, and with `imu0`
  yaw disabled they are the *only* gyro-rate input to yaw)

**Cause unknown.** The timing sits near localization's `TimerAction(period=10s)` — **this is untested
speculation; do not repeat it as fact.** Unexamined candidates: RealSense IMU stream restart, madgwick
(re)initialisation, bursty delivery (a 33 ms gap immediately precedes the jump while surrounding
samples arrive ~1 ms apart).

**Evidence bag:** `/mnt/f1tenth_ssd/shared_dir/claude_bringup_0825/ekfdiag_run12_control` (29 MB).
Reproduce the analysis with `ekfdiag.py` in that directory, plus the two ad-hoc scripts described in
`.wolf/buglog.json` under bug-248.

**Suggested first step:** the jump is in the *filtered* topic, so the question is whether the
discontinuity originates in the sensor stream or in madgwick. Compare `camera/imu/filtered` against
the **raw** RealSense IMU across the same window.

> **This needs a NEW capture — `ekfdiag_run12_control` cannot answer it.** That recorder's filter
> caught `camera/imu/filtered` but **not** the raw RealSense stream; its `vehicle/sensors/imu/raw` is
> the *VESC* IMU (`imu0`), not the camera. Only the filtered side is in the bag. Add the raw topics
> — `camera/imu`, or `camera/gyro/sample` + `camera/accel/sample`, whichever the driver publishes with
> the current `unite_imu_method` — and re-record across cold launches until one shows the jump.
> Budget several: the jump accompanied a divergence in roughly 3 of 8 launches, and it is not known
> whether it occurs on launches that *don't* diverge (worth checking directly — if it does, the
> trigger and the divergence are separable and the jump can be studied without waiting for a runaway).
>
> Confirm the raw topic names first: `ros2 topic list | grep -i imu`. Recording the wrong name
> silently yields another bag that cannot answer the question — which is exactly how this note came
> to be written.

---

## 3 · Also not started — IMU bias remover

`launch/sensors/realsense_d435i.launch.py:366` hardcodes `'remove_imu_bias': 'False'`
(`launch/vehicle/vehicle.launch.py:769` is the second site). Read `docs/imu_bias_removal_spec.md`,
then `NEXT_CHAT_IMU_BIAS.md`.

**Line numbers in those docs were wrong until 2026-08-25** — three different values were in
circulation (356 / 363 / 366, and 370 / 377 for `vehicle.launch.py`, whose real line is **769**, off
by 399). Corrected in this commit. If you find another stale one, fix it rather than working around it.

**This does not help bug-248.** `remove_imu_bias` removes a *static* offset; the 82 deg event is a
step change. Bias removal cannot prevent it and does not shrink the residual a wrong attitude
produces. Unrelated tasks.

---

## 4 · Deployment risk — read before assuming the fix is safe

The `ekf_odom.yaml` fix is committed to git **and** live on the robot, but on the robot it exists only
as a file inside container `jetson_container_20260825_173057`. `/workspaces` is a **container layer,
not a bind mount**. If that container is destroyed before the fix is re-staged, the robot silently
reverts to fusing imu1 acceleration and bug-244 comes back.

- Container was left **warm and running** deliberately (bring-up costs ~8.5 min).
- To re-stage after a container replacement: `bash /mnt/shared_dir/stage_0825.sh`, but note
  **`f1tenth_stage_20260825.tgz` predates this fix** — either re-cut the tarball from the current
  branch or re-copy `config/localization/ekf_odom.yaml` by hand and verify:
  ```bash
  ros2 param get /ekf_odom_node imu1_config    # entries 12,13,14 must be False
  ```
- Verify in the **installed** tree, not the source tree. Here they are the same file
  (`--symlink-install` symlinks `install/...` -> `src/...`), so no rebuild was needed for a YAML edit.

---

## 5 · Environment notes still current

- **DualSense** `10:18:49:9D:72:FC` is paired + trusted but **left disconnected** (a PS press
  reconnects it). Connected, it publishes the heartbeat -> `command_gate` opens -> `vesc_odom`
  publishes at ~50 Hz. That is a **mitigation** for bug-244, not a fix, and it also means **the car
  can drive** — mind the desk.
- **`command_gate_require_heartbeat:=False` does NOT hold the gate shut.** It collapses gate logic to
  *always open* (transparent passthrough). To keep the gate closed, leave the default `True` and
  disconnect the joystick. A run made with that flag is void as a control —
  see `ekfdiag_run11_VOID_gate_was_open/NOTE.txt`.
- Bags on the SSD in `/mnt/f1tenth_ssd/shared_dir/claude_bringup_0825/`: `run12_control` (the onset —
  the valuable one), `run13_fixverify`, `run9`/`run10` (joystick-connected, clean), `run11_VOID`,
  `run7` (no onset).
- `/mnt/data` inside the container is a container layer despite looking persistent.
- Docker's default **bridge** network is broken on this kernel; jetson-containers uses `--network host`.
- `jetson-containers` bind-mounts `/tmp/argus_socket`, a *socket* on R36.4.3 but a *directory* in the
  image. `run.sh` is patched; **a fresh checkout reintroduces this** (bug-242).
- CycloneDDS spams write failures at `192.168.2.140` (gosling5, down). Harmless, shared config.

---

## 6 · Uncommitted work NOT authored by this session — do not sweep it up

The working tree carries an in-progress **doc consolidation** from an earlier session: content folded
out of `BATTERY_SESSION_PLAN.md`, `MPC_BENCH_HANDOFF.md` and `NEXT_CHAT_ODOM_ROTATION.md` into
`NAV2_OFFLINE_RESULTS.md` and `rf2o_zero_velocity_brief.md`, with those three **staged for deletion**.
It is coherent and looks finished, but it was not reviewed here and was deliberately left alone.
Decide on it on its own merits; do not fold it into an unrelated commit.

---

## 7 · Still open, unchanged

- **Moving-odometry check** — needs battery power and a driven leg.
  **Do not drive under Nav2 until bug-244 has its 2-3 confirming launches**; a filter that believes it
  is doing 46 m/s produces confident, wrong control.
- **`max_steering` is still 0.34** at every entry point. 0.314 is the symmetric no-clip value and is a
  cross-repo dependency of LUCIO's ego-MPC.
- **`LUCIO_MAP_HEADING_NOTICE.md`** delivered 2026-08-25. Deferred.
