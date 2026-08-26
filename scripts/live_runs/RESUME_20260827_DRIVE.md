# Resume — one item left, and it needs the battery

**Repo:** `f1tenth_launch` · branch `perf/config-tuning` · pushed through `91c1a3e`
**gosling1:** `192.168.2.195`, direct `ssh gosling1@192.168.2.195` (no jump host)

Written 2026-08-26 ~19:45 EDT. **Replaces `RESUME_20260827_DECISIONS.md`**, three of whose four
items are closed — recover it from git history for the original wording.

Everything stationary is finished. What remains is **one drive session**, one fix that is written
but unverified, and one decision that waits on the drive.

**Start here:** §2's bug-245 check takes 30 seconds on the first launch and gates whether any
measurement that follows is worth trusting.

---

## 0 · What closed 2026-08-26 evening

**bug-251 — the `imu_bias_remover` staleness hazard. Fixed, not worked around.**
`privvyledge/imu_pipeline` @ **`humble-devel`** (upstream tag `0.5.2` + commit `58d227e`) adds a
**`stationary_timeout`** parameter. A velocity source silent longer than the timeout has its
"stationary" verdict dropped and the node falls through to subtracting the last converged bias
instead of zeroing. **Default `0.0` reproduces stock behaviour exactly**, so it is a candidate
upstream PR and inert until a config asks for a timeout. Set to **0.5 s** in
`config/filters/imu_bias_remover.yaml`.

Verified on the same bag, same harness, same scorer, only the parameter differing:

| `stationary_timeout` | after the velocity source dies |
|---|---|
| `0.0` (stock) | **3996 samples pinned at exactly 0.0**, raw gyro live to 0.037 rad/s |
| `1.0` (fork) | pinned **0.986 s** (195 samples), then all **3801** remaining samples exactly `raw − bias`, max residual **0.0** |

The absence of the timeout was confirmed in **three** independent places before forking, because a
previous session remembered otherwise: the source-built 0.5.2 read end to end (no `rclcpp::Time`,
no timer), the **apt 0.4.1** `.deb` (the string `timeout` appears in no file in the package), and
upstream's `ros2` branch at **0.6.1** (same condition, same six parameters — the only change to that
file since 0.5.2 is a `use_stamped` default flip). apt Humble is still 0.4.1, so **apt remains a
downgrade**; `docs/build_repo_requirements.md` now asks for a `.repos` entry instead of the SSD
tarball.

**§4 parked wiring test — PASS.** Chain live at 195.6 / 206.1 / 183.6 / 201.1 Hz
(`camera/imu` → `bias_removed` → `filtered`, plus `bias`), `stationary_timeout` read **0.5** off the
live node, `/camera/imu` showed Publisher 1 / **Subscription 1**. The node was additionally proven a
**bit-exact no-op** for that run: 3000 matched sample pairs between `camera/imu` and
`camera/imu/bias_removed`, max difference across all three gyro axes **`0.000e+00`**. Expected — with
the joystick disconnected `vehicle/vesc_odom` never publishes, so the flag never latches.

**`max_steering` 0.34 → 0.314** at all four entry points, and `twist_to_ackermann`'s
`max_steering_angle` **0.25 → 0.314**, so all three steering limits agree with the measured
mechanics. `LUCIO_REPLY.md` carries dated inline corrections where it claimed on 2026-08-08 that this
was already applied. **Nothing on LUCIO's side changed** — their ego-MPC publishes `drive` directly.

---

## 1 · The one remaining item — the drive session

Not startable without battery and a driven leg. Get all of this onto one drive:

- **`odometry/local` yaw drift while moving** — no parked test reaches it. Parked is +0.04 °/min.
- **The wheelbase change 0.25 → 0.256 m** (2026-08-07), still unverified on hardware. It biased
  `vesc_odom`'s kinematic yaw rate ~2.4 % against the frames it fuses into.
- **Does the bias correction improve real driven heading** — the one thing a synthetic source cannot
  answer, and the evidence the `remove_imu_bias` decision waits on. The staleness half is already
  closed offline and is **not** owed.
- **Drive into the steering limit both ways.** All observed saturation was right-turn; `servo_min`
  has never been reached, so **nothing is known empirically about the left bound**. Now that
  `max_steering` is 0.314, neither side should clip — that is itself a thing to confirm.
- **Coordinate with Stage 4a sysid.** Do not let a fusion change ride along with a Stage 4a bag —
  `k → 1.0` must be measured against a fusion configuration that is not simultaneously changing.
  Sequence: **Stage 4a first on today's fusion, then the bias work.**

**Record `camera/imu` AND `vehicle/vesc_odom` in the same bag.** A sweep of all 66 bags on gosling1
found exactly one (April 2025) carrying both — each campaign recorded for its own investigation.
That is why the offline bias test needed a synthetic velocity source.

---

## 2 · Verify this on the first launch, before measuring anything

**bug-245 — code fix landed 2026-08-26, NOT YET VERIFIED ON HARDWARE.** This is the first thing to
confirm on the next launch, before any measurement.

The autosave half was already fixed (`save_map_folder_path` is `''` unless `save_map` is true). The
half that bit us was different, and it is worth understanding because **our own seed fix is what
triggers it**: `load_map_folder_path` was handed over *unconditionally*, while
`visual_slam/initial_pose` is remapped to `initialpose` — the topic `localization.launch.py`
deliberately publishes on at `initialpose_seed_delay` (20 s) to seed `ekf_map` and AMCL from one
number (bug-241). cuVSLAM reads that same message as a **relocalization hint**, so a load path plus
that seed starts relocalization even with `localize_on_startup` False.

Measured: `Trying to localize in map … around [0.445, -0.575, 0.000]` — the AMCL seed — then
`Failed to localize in map. Error 3` on repeat, and `visual_slam_container` died **exit −6**, at
t+20 s exactly matching the seed delay.

`load_map_folder_path` is now gated on `localize_on_startup_effective`, the same condition as
`localize_on_startup`, so both map paths are symmetric: no intent, no path.

**Verify on the next launch** — VSLAM should survive past t+20 s and `/visual_slam/tracking/odometry`
should be publishing. If it aborts again, move `/mnt/data/maps/nvidia/vslam_map` aside (making
`map_exists` False) and re-check; that isolates whether the gate is working or something else still
hands over a map.

**Why this matters more than it looks — it invalidated a measurement.** With VSLAM dead, `ekf_odom`
lost `odom1` entirely and rf2o degraded to 3.2 Hz, and a parked `yaw_drift` read **+3.77 °/min**
against a +0.04/+0.01/+0.17 baseline measured with VSLAM healthy. It reads exactly like a fusion
regression and is not one. **The tell is the `yaw_drift` source table: check the
`Isaac VSLAM (VO)` row for `0` samples before reading the `odometry/local` number.**

**`remove_imu_bias` stays `'False'`** at both call sites until the driven evidence exists. Only the
stale comment at the RealSense call site was corrected — it had claimed flipping the flag "makes the
launch fail" because the node is not installed. It is installed, source-built into the image.

---

## 3 · Restoring the robot — everything below is ephemeral

The container **dies with the terminal that started it**, and `/workspaces` is a container layer.
The operator starts it from the Jetson desktop session:

```bash
bash ~/bolus_ws/f1tenth_launch.sh
```

Then, from the host or inside the container:

```bash
FLIP=1 bash /mnt/shared_dir/stage_0826c.sh      # ~1 min
```

`stage_0826c.sh` supersedes `stage_0826.sh`. It carries **`f1tenth_stage_20260826c.tgz`**
(md5 `5e39c50d3512756b524a339029450f76`, cut from git HEAD with `git archive`; it carries the bug-245 gate), clones the
`imu_pipeline` fork and rebuilds `imu_processors`, and verifies eight values in the **installed**
tree. `FLIP=1` applies a temporary `remove_imu_bias:='True'` for wiring tests; **omit it** for the
committed `'False'`. **Re-staging `0826b` or `0825` silently reverts today's work.**

Two live checks worth running after any stage:

```bash
ros2 param get /realsense_imu_bias_removal_node stationary_timeout   # 0.5 -> the fork is running
ros2 param get /realsense_imu_filter constant_dt                     # 0.005 -> bug-248 fix present
```

The first is the only reliable proof of the fork: **stock `imu_processors` does not declare
`stationary_timeout` and silently ignores it** — the node starts and logs nothing — so a stock build
reads as configured and is not. That query errors on stock.

---

## 4 · Verify these before planning around them

This handoff family has repeatedly carried blockers that were stale, and each has cost a session.
**Two more were found on 2026-08-26**: `RESUME_20260827_DECISIONS.md` §10 said two commits were local
and unpushed (they were already pushed), and a session note claimed the stale steering-gain comment
at `vehicle.launch.py:320` had been fixed (the working tree was clean and it had not been). Earlier
examples: *"`imu_processors` is at 0.4.1"* (the image builds 0.5.2 from source), *"the driving half
of the bias check is owed to a drive session"* (closable parked with a synthetic source), *"the
Jetson has no internet"*, *"root fs at 96 %"*.

Spend the first minutes checking each named blocker against the machine — `git log @{u}..HEAD`,
`apt-cache policy`, `git status`, read the shipping source — then correct the doc in place, in the
same commit as the work.

---

## 5 · Traps that have each cost a run

- **A silent RealSense has two unrelated causes.** Count kernel re-enumerations:
  `dmesg | grep -c "Found UVC 1.50 device"`, wait 20 s idle, count again.
  *0 new while idle and the device fails to enumerate* → wedged (bug-014/170), fix in software with
  a USB unbind/rebind at its port from inside the privileged container. *0 new while idle but a
  burst of 4–16 every time anything opens it — a bringup, or even a plain `rs-enumerate-devices`* →
  **physical**, reseat the cable (bug-255). On 2026-08-26 the second case cost two full launch
  cycles: the unbind/rebind appeared to work every time (device back at 5000 Mbps, sysfs perfect,
  librealsense reading serial and firmware) and the next launch failed byte-identically. A replug at
  the camera end took launch re-enumerations from 16+ to 3.
- **`package 'f1tenth_launch' not found, searching: ['/opt/ros/humble']`** is an unsourced overlay,
  not a failed build (bug-254). The image's `~/.bashrc` does not source
  `/workspaces/f1tenth/install/setup.bash`. The searched-paths list in the error is the diagnostic.
- **`ros2 topic hz` does not accept `--qos-reliability`** — that is an `echo` flag. With stderr
  suppressed the usage error reads as "NO DATA" on a topic that is streaming at 200 Hz. Do not
  suppress stderr on a check whose negative result you intend to act on.
- **`ros2 launch` does not reliably die on Ctrl-C here** (bug-249). It takes most children and
  leaves the launch process and one container. List with
  `ps -eo pid,args --no-headers | grep "[c]omponent_container"` and kill **by PID** — `pkill -f`
  inside `docker exec bash -lc` kills its own shell, because the shell's command line contains the
  pattern.
- **`yaw_drift.py` takes its namespace from `$VEHICLE_NAME`**, which is set in the container, while
  bringup defaults `use_f1tenth_namespace:=False`. Export **`F1TENTH_NS=""`** or every row reports
  zero samples.
- **Never `set -u` in a script that sources `/opt/ros/humble/setup.bash`** (bug-250) — it reads an
  unbound `AMENT_TRACE_SETUP_FILES`, exits before launching anything, and **exits 0**.
- **A parked yaw-drift figure cannot tell you whether the bias correction works**, because the
  zeroing path is active and the subtraction path is not. Log the `bias` topic instead.

---

## 6 · Environment notes still current

- **X11 works through the operator's SSH forwarding**, not `:0`. On 2026-08-26 the container had
  `DISPLAY=localhost:11.0` and `xdpyinfo` succeeded there. Probe with `xdpyinfo -display <d>` before
  assuming a display problem.
- **DualSense** `10:18:49:9D:72:FC` is paired and trusted but normally left **disconnected**.
  Connected, it publishes the heartbeat, `command_gate` opens, `vesc_odom` starts at ~50 Hz — **and
  the car can drive.** Note the bias remover only leaves passthrough once `vesc_odom` is alive.
- **`command_gate_require_heartbeat:=False` does NOT hold the gate shut** — it collapses the logic to
  always-open. To keep it closed, leave the default `True` and disconnect the joystick.
- CycloneDDS spams `ddsi_udp_conn_write … retcode -3` at absent peers. Harmless, but it will drown a
  `ros2 param get` — pipe through `grep -v ddsi`. The config at
  `/mnt/shared_dir/cyclonedds_config_static.xml` includes `lo` at priority 10 and applies to
  **any** domain, and both `CYCLONEDDS_URI` and `RMW_IMPLEMENTATION` are set in the container
  environment, so `docker exec` shells inherit them.
- Docker's default **bridge** network is broken on this kernel — `docker run` needs `--network host`.
- `jetson-containers` bind-mounts `/tmp/argus_socket`, a *socket* on R36.4.3 but a *directory* in the
  image; **a fresh checkout reintroduces this** (bug-242).
- Bags: `claude_bringup_0826/run1{4,5,6}` (pre-bug-248-fix, raw **and** filtered camera IMU),
  `run17_constdt` and `run18_constdt` (post-fix). Offline bias data `biastest_0826/bias_test.csv`
  (stock) and `biastest_timeout/bias_test.csv` (fork, `stationary_timeout` 1.0).
