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

## Start here — the operator assigned these three to this session (2026-08-05 21:36)

**1. Measure the RTABMap localizer.** Still untested, as it has been since the
map-build handoff. It was originally disabled on an *assumption* that it is too
slow, never a measurement. Localize `mapping_drive_170025` against
`rtabmap_final_nf.db` in localization mode and report two numbers: accuracy
against `truth_mapping_drive_170025.csv` (same method as
`check_map_frame.py --truth`, so it is directly comparable to AMCL's 65 mm mean
/ 144 mm p95), and CPU on the Orin Nano's 6 cores. If it holds up it also keeps
the 2D and 3D maps in one frame, and it can feed `rtabmap/localization_pose`
into `ekf_map` as that file already intends.

**2. The two `ekf_map.yaml` items.** Both were found by reading the config, not
by a failure; the map-frame deliverable routed around them by broadcasting from
AMCL directly. Neither has been changed — `ekf_map.yaml` is untouched.

- **`pose0_relative: true` on `amcl_pose`.** robot_localization treats the first
  measurement as the origin, which discards the absolute map pose in a filter
  whose `world_frame` is `map`. This one looks straightforwardly wrong.
- **`odom1` = `visual_slam/vis/slam_odometry`, `differential: false,
  relative: false`** — i.e. an absolute map-frame anchor. **This is only wrong
  under one startup mode.** Isaac VSLAM does persist a map
  (`visual_slam_map_path`, `load_map`/`save_map`), so when it is localized into
  a matching saved map `slam_odometry` genuinely is in a global frame and the
  config is correct. It is wrong when VSLAM starts fresh — which is the default,
  since `localize_on_startup` defaults to `False`, and is what these bags
  recorded (VSLAM started at yaw ~0, ~54° off the real `odom` frame on
  `mapping_drive_170025`). So decide it as a function of startup mode rather
  than editing it flat. See bug-104.

**3. Fold the AMCL tuning back into the live config, if it survives review.**
`config/localization/localizer_amcl_mapframe.yaml` was created as a *separate*
file rather than an edit to `localizer_amcl.yaml`, deliberately: the live values
are defensible for driving on a Jetson and changing them alters on-car
behaviour. The operator has now asked for the fold-in to be considered here.

What the live config actually gets wrong on this map — it never converged,
reporting its own sigma as 419 mm / 16.7°:
- `sigma_hit: 0.4` smears each beam over 8 of the map's 0.05 m cells
- `z_rand: 0.5` puts half the probability mass on "this reading is noise"
- `max_beams: 90` of the X4's ~625, against only 1732 occupied cells

`sigma_hit`, `z_hit`/`z_rand` and `max_beams` are close to free and are the ones
worth folding in. **The particle count is not free** — the offline file runs
1000–4000 against the live 100–500, and `update_min_d` 0.05 vs 0.2 raises the
update rate five-fold. Benchmark on the Orin before moving those; the live
numbers are a CPU-budget decision, not an oversight.

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
