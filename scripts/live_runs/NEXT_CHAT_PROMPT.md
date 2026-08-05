We're resuming a live bag-recording session on the F1/10 car `gosling1`. A
previous chat did a practice run, verified the recording path end to end, and
found and fixed several defects. **Read
`scripts/live_runs/DRIVE_SESSION_HANDOFF.md` first — it has the full procedure,
the findings, and the gotchas.**

## What you're doing

Recording three bags that a separate repo will use to build maps and generate
waypoints **offline**. No SLAM, no Nav2, no live mapping.

| # | Bag name | Purpose | max_speed |
|---|---|---|---|
| 1 | `mapping_drive` | offline 2D + 3D RTABMap source | 1.0 |
| 2 | `loop_laps` | 2–3 laps of a loop | 1.5 |
| 3 | `figure8` | 2–3 figure-8s | 1.5 |

`max_speed` is a launch arg, so **relaunch the stack between bag 1 and bag 2.**

## How I want to run it

1. I say **"start"** → you run `start <name>`, verify the bag is actually
   growing on disk, then tell me **"recording has started, move the robot now."**
2. I drive, then say **"I am done, stop recording, on to the next bag."**
3. You run `stop`, report the per-topic audit, and we move on.

Don't tell me recording started until you've confirmed bytes are landing.

## Before anything else

I rebooted `gosling1` onto battery, reseated the VESC USB cable, and I'm now on
a **new container from `privvyledge/f1tenth:humble-devel-08052026`** (the old
session ran on the June 6 image). **The Jetson has no internet, so `git pull`
will fail.** The updated files are staged on the SSD bind mount. Inside the new
container, run:

```bash
bash /mnt/shared_dir/handoff/live_runs_20260805/install_handoff.sh
```

It verifies everything by checksum and tells you if a rebuild is needed. It
must end with `install OK.` before we record.

## Then

```bash
cd /workspaces/f1tenth/src/f1tenth_launch/scripts/live_runs
export ROS_DOMAIN_ID=0
./25_drive_session.sh launch --max-speed 1.0 -y
./25_drive_session.sh status     # show me the rates before we record
```

Run commands on the robot as:
`ssh gosling1 'docker exec -i <container> bash -s' < script.sh`
(`bash -lc "…"` mangles quoting through ssh + docker).

## Watch for these — all seen on hardware 2026-08-05

- **VESC driver aborts on serial EIO** and the car goes dead-stick while every
  ROS command topic still looks perfectly healthy (the mux and command_gate are
  downstream-blind to it). `respawn_delay` is now 2 s, so an outage should be
  ~2 s not 10.7 s. Detect it as a **gap in `vehicle/sensors/core`**, not from
  the command topics. If it fires mid-bag, discard and re-drive that bag.
- **YDLidar** may need a physical unplug/replug (symptom: connects, then
  `health code -1` / `Failed to start scan mode -1`). The driver *exits*, so
  relaunch the stack afterwards. Healthy is ~8.6 Hz on `scan_filtered`.
- `drive`, `cmd_vel`, `estop` are silent by design here. Anything else silent
  is a real defect.
- Budget **~11 GB/min**. Check free space on `/mnt/shared_dir` before starting.

## Open items, not for today

- `vesc_driver` should catch the serial exception and reconnect instead of
  aborting; the safety chain has no way to notice the actuator is gone (bug-068).
- I'm recalibrating `steering_angle_to_servo_offset` toward 0.5 this weekend;
  `MAX_STEERING` is temporarily 0.25 rad to avoid clipped (and therefore
  misleading) steering in the bags. Raise it back to 0.34 afterwards.
