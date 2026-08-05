# Next session — the recording job is DONE

**The three bags were recorded and audited on 2026-08-05 (16:54–17:40 EDT).**
Do not re-run the drive session. If you were handed this file expecting to
record, stop and read `DRIVE_SESSION_HANDOFF.md` — "Status" and "Session 3".

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
are built from these **offline, in a separate repo** — nothing further is needed
from the car.

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
