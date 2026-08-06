# Next session — the map-frame deliverable is BUILT and VERIFIED

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
- `ekf_map` with the new `fuse_vslam_global` plumbing, `pose0_relative: false` and
  the `vyaw` fix (bug-112). Note it is **not** the map-frame broadcaster — AMCL is,
  and measurement says keep it that way (`LOCALIZER_FOLLOWUPS.md` §5). What to
  watch live is `odometry/global`, and that it does not need ~30 s to converge
  from startup (bug-111 is unfixed);
- `/dev/ydlidar -> ttyUSB0` after the 17:45 reboot, still unconfirmed.

**2. Send a Nav2 goal. The car has never driven autonomously.** `testing_checklist.md`
§9 "Send a navigation goal" has never been ticked, and it is now unblocked: there
is a map, a localizer that clears the bar, and a command gate whose R1 handover is
verified. Remember `launch_twist_to_ackermann:=True` when driving under Nav2 —
it defaults False because the MPC publishes `drive` directly.

**3. DONE 2026-08-06 — and `ekf_map` is NOT the fix. See `LOCALIZER_FOLLOWUPS.md` §5.**
Measured on the same control: `ekf_map` broadcasting at 30 Hz is **less smooth than
AMCL direct**, not more — correction step p95 **83.5 mm vs 15.7 mm**, max **920 mm
vs 59.6 mm**, and less accurate too (125.0 vs 74.7 mm steady state). Two real
defects came out of it: `ekf_map` is **never seeded** (starts at the origin, ~30 s
at ~740 mm / 82 deg, silently — bug-111), and it had **no angular-rate input at
all** (`odom0_config` `vyaw` was false, IMUs excluded by design — bug-112,
**fixed**, worth 125.0 -> 85.2 mm and step p95 83.5 -> 37.3 mm, still short of
AMCL). v1 is untouched: it never used this path. **The live lever for the MPC's
jerkiness is gone; what remains is the offline smoothing for LUCIO below.**
The original reasoning, now superseded, was:


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
4. **Verify `/dev/ydlidar → ttyUSB0` survived the 17:45 reboot** on gosling1.
5. **`rtabmap_2d_overfiltered.yaml` names the wrong PGM** (`rtabmap_2d_final.pgm`
   with the overfiltered origin). Cosmetic; that file is marked do-not-use.

## If you do need to record again

`25_drive_session.sh` for the session, `DRIVE_SESSION_HANDOFF.md` for the
protocol and container rules. The two pre-flight checks that caught real
failures: **pack voltage and `fault_code`** from `vehicle/sensors/core`
(BEST_EFFORT QoS), and **`ls -l /dev/ydlidar`** must show a **ttyUSB**, never a
ttyACM. Ordering rule: **park at the start pose, *then* relaunch, *then*
record** — relaunching alone does not zero the EKF and VSLAM.
