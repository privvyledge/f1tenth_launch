# Next session — recording is DONE, and so is the mapping-bag map

**The three bags were recorded and audited on 2026-08-05 (16:54–17:40 EDT).**
Do not re-run the drive session. If you were handed this file expecting to
record, stop and read `DRIVE_SESSION_HANDOFF.md` — "Status" and "Session 3".

**2D + 3D maps were then built offline from `mapping_drive_170025`
(2026-08-05, 18:00–19:55 EDT).** Read **`MAP_BUILD_HANDOFF.md`** first — it has
the artifacts, the environment, four fixed defects, and the next task.

## Start here

**The deliverable is a derived bag per run**, carrying either `map→odom` on
`/gosling1/tf` or a `map`-frame pose topic, for an external consumer (LUCIO
pixel→world calibration) that cannot access this repo and cannot drive the car.
Their written spec is `REQUEST_f1tenth_map_frame_pose.md` in
`.../CAPS People/LUSCIO_ROS/`; the requirements are summarised in
`MAP_BUILD_HANDOFF.md` §2. Two that will bite if missed: **keep the original
header stamps byte-exact** (the cross-machine camera merge is on header stamp,
chrony-synced to ~1 µs), and **all three runs must share the one map** built
from `mapping_drive`.

Getting there means **localizing `loop_laps_173558` and `figure8_172338` against
`rtabmap_2d_final.yaml`**, so all three trajectories share one `map` frame. It
is a localization job, not a mapping job:

- **A map does not publish `map→odom` — a localizer does.** The `.pgm`/`.yaml`
  and `.db` are static files. `map→odom` comes from AMCL, slam_toolbox
  localization mode, `particle_filter`, or RTABMap localization mode at runtime.
  Nothing needs re-mapping to get it.
- The bags already carry `odom→base_link` at 30 Hz from the live EKF, so a
  localizer only has to supply the missing edge.
- **Broadcast it from the global EKF** (`ekf_map`, `map_tf_publisher:='ekf'`),
  which already fuses `amcl_pose` and `rtabmap/localization_pose` — operator's
  call. `particle_filter` is out until its known bugs are fixed. The RTABMap
  localizer was disabled on an *assumption* that it is slow and has never been
  measured: test its accuracy and CPU, and if it holds up feed it into the EKF.
- The hard part is the **initial pose**: each bag zeroes position but **not
  heading** (EKF starts at −0.951 / −0.411 / −0.436 rad), and the physical start
  pose differs per run, so the offsets are unknown until estimated.
  `50_localization_test.sh` is the entry point.

Full reasoning, including why a "fused map" would mean RTABMap multi-session
mapping rather than post-hoc alignment, is in `MAP_BUILD_HANDOFF.md`.

## What exists

`/mnt/f1tenth_ssd/shared_dir/bags/20260805/` on the host,
`/mnt/shared_dir/bags/20260805/` inside any container:

| bag | purpose | `max_speed` | duration | size |
|---|---|---|---|---|
| `mapping_drive_170025` | offline 2D + 3D RTABMap source | 1.0 | 146.9 s | 24 G |
| `loop_laps_173558` | 2–3 laps of a loop | 1.5 | 103.1 s | 17 G |
| `figure8_172338` | 2–3 figure-8s | 1.5 | 155.4 s | 25 G |

All audited clean, all start with every localizer at the origin, all carry the
full 39-topic set including actuator commands and VESC state. Maps and waypoints
are built from these **offline** — nothing further is needed from the car.

Maps built so far, in `/mnt/f1tenth_ssd/shared_dir/maps/20260805/` (has its own
`README.md`): **`rtabmap_2d_final.*` is the 2D map to use**, with
`rtabmap_final_nf.db` and `cloud_clean.pcd` for 3D. The SLAM Toolbox grid
(`slamtoolbox_2d_final.*`) was **checked as a costmap and rejected** — smeared
walls and speckle through the free space. Do not use it, and do not rank the two
by occupied-cell count; the noise inflates it.

Also on disk from earlier in the day: `mapping_drive_145639` (practice; missing
`imu` and `imu/mag`, and it clipped steering — prefer the three above),
`detection_093214`, `mpc_wiring_093214`.

## Open items carried forward

1. **Apply the udev fix to the other goslings.** `ydlidar-V2.rules` matches the
   VESC's USB ID and makes `/dev/ydlidar` point at the VESC, which kills the
   LiDAR *and* SIGABRTs the VESC driver (bug-073 — the root cause of bug-068).
   gosling1 is fixed; the rest are not. Procedure and per-car check:
   `UDEV_YDLIDAR_VESC_COLLISION.md`.
2. **`vesc_driver` should catch the serial exception and reconnect** instead of
   aborting, and the safety chain should be able to notice the actuator is gone
   (bug-068, still open as a code defect even though its trigger is fixed).
3. **Raise `MAX_STEERING` back to 0.34** in `scripts/live_runs/00_env.sh` after
   the operator recalibrates `steering_angle_to_servo_offset` toward 0.5
   (scheduled weekend of 2026-08-08). It is temporarily 0.25.
4. **Verify `/dev/ydlidar → ttyUSB0` survived the 17:45 reboot** on gosling1 —
   the rule fix was applied but the symlink in place at the time was a manual
   one, so the reboot is the first real test of the surviving rule.

## If you do need to record again

Everything still applies: `25_drive_session.sh` for the session,
`DRIVE_SESSION_HANDOFF.md` for the protocol and the container rules. The two
pre-flight checks Session 3 added, both of which caught real failures:

- **Pack voltage and `fault_code`** from `vehicle/sensors/core` (BEST_EFFORT QoS)
  before recording. A flat pack presents as "the speed cap is too low" — see
  bug-072.
- **`ls -l /dev/ydlidar`** must show a **ttyUSB**, never a ttyACM.

And the ordering rule: **park the car at its start pose, *then* relaunch, *then*
record** — relaunching alone does not zero the EKF and VSLAM.
