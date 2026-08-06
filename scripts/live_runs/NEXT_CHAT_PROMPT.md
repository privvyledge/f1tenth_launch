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

- **Run the new `localizer_amcl.yaml` on the car.** Everything above is bag
  replay. Confirm AMCL converges from its startup pose and `map->odom` is steady
  while driving, and that `ekf_map` comes up with the new `fuse_vslam_global`
  plumbing on a real bringup.
- Optional: revive the RTABMap localizer by tuning `Vis/*` against
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
