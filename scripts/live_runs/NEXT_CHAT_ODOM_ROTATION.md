# Handoff — parked `odom→base_link` rotation, and the container-reset question

**Written:** 2026-08-10 ~14:45 EDT · **Branch:** `perf/config-tuning` · **Robot:** gosling1
**Container at time of writing:** `jetson_container_20260810_015546` (stack up, car parked, AC power)

---

> ## ⚠ SUPERSEDED IN PART — read this box first (updated 2026-08-10 ~15:30 EDT)
>
> Everything below §1 was written before the retest. What actually happened:
>
> 1. **The operator-visible rotation was NOT drift. It is bug-232**, now root-caused and fixed by a
>    launch argument. `ekf_map` fuses `odom1 = visual_slam/vis/slam_odometry` as an **absolute
>    map-frame anchor**; the `fuse_vslam_global` guard that should disable that was defeated by
>    `bringup.launch.py:76` defaulting `localize_isaac_vslam_on_startup` to **True**, while the VSLAM
>    node itself ran with `localize_on_startup: False` from its own power-up origin. Measured:
>    `amcl_pose` −79.80° vs `odometry/global` −7.50°, VSLAM −7.45° at 29.9 Hz, `/amcl_pose` **0
>    messages in 180 s** (motion-gated, parked). Relaunching with
>    **`localize_isaac_vslam_on_startup:=False`** took AMCL-vs-EKF disagreement from **72.3° to
>    0.56°**. See `DEMO_RUNBOOK_20260810.md` §4 and buglog `bug-232`.
> 2. **bug-231 did not reproduce.** Cold launch, parked 180 s: `odometry/local` **−0.05 °/min**,
>    absolute `odom→base_link` steady at −0.22° / 18 mm over 5.5 min. Per-source and gyro-bias table
>    in the runbook §4. The 2.9 °/min remains real but **unexplained** — treat as intermittent.
> 3. **The container-reset hypothesis in §2 is disproven.** All 111 files of `src/f1tenth_launch`
>    were md5-identical to the repo. The rf2o gate and the bug-140 twist fix are both **PRESENT**;
>    the twist-fix "miss" was a bad grep, as suspected. IMU bias removal is off because
>    `realsense_d435i.launch.py:363` **hardcodes** it off — not because anything is missing, and its
>    "node not installed" comment is stale (`imu_processors` was built 2026-08-09).
> 4. **The seed step is optional, not mandatory.** With the guard engaged the correct pose came up
>    with **no seed at all**. bug-230 was a symptom of bug-232.
> 5. **The parking-spot heading is settled: ≈ −84.5°** (single clear peak, not ambiguous). The config
>    seed is 4.70° off and figure-8 waypoint 0 does not fit the map. Runbook §5b.
>
> §3's launch block is superseded by the runbook's — it now needs `RMW_IMPLEMENTATION` and
> `localize_isaac_vslam_on_startup:=False`. **Use `DEMO_RUNBOOK_20260810.md` §4, not §3 below.**

---

## 1. The open defect (bug-231)

On a car that **had not moved**, measured live:

```
odom->base_link : (0.338, -1.200)  yaw +62.08 deg
map->odom       : (1.466, -1.294)  yaw -140.61 deg
map->base_link  : (0.445, -0.580)  yaw  -78.86 deg   <- correct, after seeding
```

`odom→base_link` should be ~zero. It was **+4.6°** about 20 minutes earlier, so it is ramping at
roughly **2.9 °/min**, against the **+0.01…+0.17 °/min** recorded 2026-08-06 after the bug-129 fix.

AMCL is then *forced* to absorb the error into `map→odom`, which is why the operator sees the odom
frame metres from the map and rotated ~80–90°, the car rotated ~30° from odom, and the live scan and
footprint rotated inside a static 2D costmap.

**The maps are not the problem — this was checked and the check is conclusive.** The 3D cloud and the
2D grid both came out of `rtabmap_final_nf.db` (db 19:42, grid 19:42:34, cloud 19:43:12/39) and are
co-registered to about one cell: median distance from an occupied grid cell to the nearest projected
cloud cell is **0.05 m at zero shift**. No rotation is supported: the cloud is an isotropic blob whose
wall-orientation profile has contrast **1.11×** (flat), so correlation-derived "best angles" of −27°
and −30.75° were artifacts of a dense blob smeared over sparse walls. Do not re-open this. Do not
shim the cloud.

### Instrumentation trap that hid it

`/mnt/shared_dir/tf_chain_watch.py` reports **displacement from its first sample**. Over 90 s it
reported 10 mm and −0.14° and looked perfectly healthy while the *absolute* offset was 1.25 m and
62°. **Read absolute `tf2_echo odom base_link` on a parked car.** Fix the script or read past it.

---

## 2. Leading hypothesis: fixes that do not survive a container reset

`/workspaces` is a container layer, not a bind mount, so every new container reverts to the image.
The operator's hypothesis is that robot-side fixes are silently missing. Checked 2026-08-10:

| fix | state in this container | how checked |
|---|---|---|
| rf2o zero-velocity gate | **PRESENT** | `grep -l "Zero-velocity detected" src/rf2o_laser_odometry/src/CLaserOdometry2D.cpp` |
| IMU bias removal | **NOT RUNNING** — config present, no node | `ros2 node list` shows only `/realsense_imu_filter` |
| twist fix (bug-140) | **UNKNOWN** — grep found no `atan`/`yaw_rate_to_steering` in `trajectory_following_ros2`; more likely a bad search than a missing fix. Re-check properly. |
| `f1tenth_launch` package | was **STALE**, repaired this session (see §4) | md5 vs repo |

### Why the IMU one is the strongest lead

`config/localization/ekf_odom.yaml:231` — `imu1_config` angular-velocity row is `[true, true, true]`,
so `camera/imu/filtered` **vyaw is fused**, and the comment at line 239 records that with `imu0` yaw
disabled this IMU's vyaw is the primary rate source. `realsense_imu_filter` is Madgwick, which passes
the gyro through. So an uncorrected RealSense gyro-z bias feeds yaw directly. The measured biases were
written up but never applied — see `docs/imu_bias_removal_spec.md`, `docs/imu_yaw_bias_notes.md`,
`config/filters/imu_bias_remover.yaml`.

**This is a hypothesis, not a conclusion.** rf2o's ZV gate should hold pose while parked and VSLAM
also contributes, so the IMU is not necessarily the sole driver. Attribute it before changing config.

### Suggested attribution method

Park the car, log 3–5 min, and compare per-source yaw rate rather than guessing:
`scripts/live_runs/yaw_drift.py` (60 s, parked) is the documented tool, and CLAUDE.md's rule applies —
**if every source reports the same large drift, the car moved rather than the fusion breaking.**
Then A/B: drop `imu1` vyaw, re-measure; drop `odom1` (VSLAM), re-measure.

---

## 3. Required launch sequence (verified live, except end-to-end ordering)

```bash
source /opt/ros/humble/setup.bash && source /workspaces/f1tenth/install/setup.bash
export ROS_DOMAIN_ID=42
export CYCLONEDDS_URI=file:///mnt/shared_dir/cyclonedds_offline_lo.xml
export ROS_LOG_DIR=/mnt/shared_dir/claude_mpc_0810/roslogs_nav2b
ros2 launch f1tenth_launch bringup.launch.py \
  launch_visualization:=True launch_twist_to_ackermann:=True \
  max_steering:=0.314 \
  map_file:=/mnt/shared_dir/maps/20260805/rtabmap_2d_final.yaml

# THEN, required (bug-230):
cd /workspaces/f1tenth/src/f1tenth_launch/scripts/live_runs
python3 seed_initialpose.py --ns "" --no-use-sim-time --x 0.445 --y -0.575 --yaw -1.4748
```

**Why the seed step is mandatory.** Through bringup, `map_tf_publisher` defaults to `'ekf'`
(`bringup.launch.py:91`, passed down at `:904`, beating `localization.launch.py`'s `'amcl'`), so
`ekf_map` owns `map→odom` and AMCL runs `tf_broadcast: False`. `ekf_map` seeds from a
`set_pose`→`initialpose` remap (`ekf_map.launch.py:164-205`, the BUG-111 fix), but nothing publishes
`/initialpose` on a live run — `localizer_amcl.yaml`'s `set_initial_pose: True` is applied by AMCL
*internally*. Unseeded, `ekf_map` starts from a zero state and sits at identity while reporting
healthy. Seeding moved `map→base_link` from identity to `(0.445, -0.580, -78.86°)`.

`seed_initialpose.py` gained `--use-sim-time/--no-use-sim-time` and empty-namespace handling on
2026-08-10; defaults are unchanged so the offline scripts (`51_localize_offline.sh:268`,
`61_nav2_offline.sh:367`) behave exactly as before.

**Not yet done: an end-to-end cold run of DEMO_RUNBOOK §4.** In particular the ordering of the seed
step relative to Nav2 lifecycle activation is untested.

---

## 4. What was repaired this session

The container came up from the image with `f1tenth_launch` older than the repo — this presented as a
dead `pcd_to_pointcloud` and an RViz with an empty Displays panel. Repaired and checksum-verified:

| item | was | now |
|---|---|---|
| `launch/bringup.launch.py` | `launch_pointcloud_map` hardcoded `False`, pcd default = Jan-2024 `raslab/cloud.pcd` | follows `launch_visualization`; pcd default `20260805/cloud_voxel_0p05.pcd` |
| `config/f1tenth.rviz` | 264 lines, fixed frame `odom`, no map/cloud/Nav2 displays | 478 lines with `Map 2D`, `Map Cloud 3D`, Nav2 set, fixed frame `map` |
| `data/maps/20260805/` | absent | pcd + pgm + yaml |

Backups: `…/src/f1tenth_launch/{launch/bringup.launch.py,config/f1tenth.rviz}.bak_20260810_020046`.
**The `colcon build --symlink-install --packages-select f1tenth_launch` is not optional** — symlink
install links files that existed at build time, so files added later are absent from `install/`
until a rebuild.

`bringup.launch.py` is otherwise **unmodified** — the operator asked to be consulted before any
change there, and with the seed step the `'ekf'` default works.

---

## 5. Environment notes that cost time today

- **Jetson clock.** Reads ~12 h behind after a manual `date -s` that was taken as AM. Check `date`
  against a real clock before any timed measurement.
- **Root is back on the SD card** (`/dev/mmcblk0p1`, 82 % full). The NVMe is healthy and mounted
  (`/mnt/shared_dir` in-container = `/mnt/f1tenth_ssd/shared_dir` on the host, 33 % used). Keep
  `ROS_LOG_DIR` on the SSD. The SD card failed read-only three times (2026-08-09, once at 82 % full
  with 5 GB free — so it is hardware, not capacity; diagnose with `journalctl -k`, and note that
  `docker exec` dies with the fs, leaving stdin-FIFO as the way in). Buglog has the detail.
- **SSH from the agent workstation** works again (key `claude-wsl@msi`). The operator launches the
  stack; an agent-initiated bringup over SSH has previously killed the RealSense via the GLFW/X11
  context.
- **This run is NOT namespaced.** A raw `ros2 launch bringup.launch.py` leaves
  `use_f1tenth_namespace` False, so TF is bare `/tf` and plain `tf2_echo` is correct. The
  `live_runs` scripts *are* namespaced. `ros2 topic list | grep -c gosling1` settles it.

---

## 6. Corrections to trust over older notes

- CLAUDE.md said the cloud and grid came from `rtabmap_final.db`. **Wrong** — `rtabmap_final_nf.db`.
  Fixed 2026-08-10.
- CLAUDE.md said a running AMCL is the default `map→odom` broadcaster. **True only for standalone
  `localization.launch.py`**; through bringup it is `ekf_map`. Fixed 2026-08-10.
- `bug-228` originally blamed a VSLAM SIGABRT and CPU stall for ~9 m of drift. **Disproven on
  hardware**; the entry has been corrected in place. The crash and the stall were real but were not
  the cause.
- An earlier revision of DEMO_RUNBOOK §4 told you to pass `launch_stereo_odometry:=False` and
  `launch_pointcloud_map:=False`. **Removed** — the cloud is the deliverable and VSLAM is a wanted
  localization input.
