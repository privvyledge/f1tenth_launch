# Next session — START HERE

**Updated 2026-08-06 ~11:00 EDT.** Item 3 below (the MPC's jerky pose) was
answered NO in `LOCALIZER_FOLLOWUPS.md` §5 — **and then reopened and answered YES
in §6**: with B's fix `ekf_map` matches AMCL on smoothness and beats it on
accuracy. `deliverables/20260805/` is still frozen v1 with checksums verified.

## Both offline tasks are DONE. Go to items 1 and 2 — the live tests.

**A is DONE** (seeding, commit `bab2172`) and **B is DONE** (the motion model,
below). The offline work on `ekf_map` has run out of untested levers worth
pulling, and everything measured so far is bag replay. The car was free from
~10:45 EDT, so **item 1 (live bringup) and item 2 (the first Nav2 goal) are now
the priority over anything else in this file.**

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
it defaults False because the MPC publishes `drive` directly.

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
