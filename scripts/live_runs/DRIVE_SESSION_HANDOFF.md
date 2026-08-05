# Drive-Session Handoff — multi-bag recording on `gosling1`

**Written 2026-08-05 ~15:05 EDT.** Everything below was measured on hardware
that afternoon, not inferred from launch files.

The job: record three bags on `gosling1` for a downstream repo that will build
maps and generate waypoints **offline**. A practice run was completed and
verified; the real runs are still to do.

---

## Status

| | |
|---|---|
| Practice run | **done** — `mapping_drive_145639`, 171.5 s, 27.5 GiB, 34 live topics all at rate |
| Real runs | **not started** — operator is rebooting `gosling1` onto battery and reseating the VESC USB cable |
| Recording path | **verified end to end** (start → growth check → stop → per-topic summary) |
| Stack on the robot | **shut down** cleanly before the reboot |
| Container | **changing** — old run was on the June 6 image; real runs use `humble-devel-08052026`. Run the installer first (next section). |

Bags live at `/mnt/shared_dir/bags/<YYYYMMDD>/` in the container
(`/mnt/f1tenth_ssd/shared_dir/...` on the host). 684 GB free. **Never write to
`/`** — the host root is a 28 GB SD card at 95%.

---

## The three bags still to record

All three use one identical 40-topic set, so they are interchangeable
downstream. Only the name and the speed cap change.

| # | Name | Purpose | `max_speed` |
|---|---|---|---|
| 1 | `mapping_drive` | offline 2D + 3D RTABMap source | **1.0** |
| 2 | `loop_laps` | 2–3 laps of a loop, trajectory tracking | **1.5** |
| 3 | `figure8` | 2–3 figure-8s, trajectory tracking | **1.5** |

The operator chose a slower cap for mapping (rotation rate is what breaks scan
matching). `max_speed` is a launch argument, so **the stack must be relaunched
between bag 1 and bag 2.**

---

## FIRST: the container is changing images — install the fixes before recording

Everything in this session ran on `privvyledge/f1tenth:humble-devel-06062026`
(8 weeks old). The operator built
**`privvyledge/f1tenth:humble-devel-08052026`** at 14:19 EDT that day — an hour
and a half *after* the old container started — bringing packages up to date and
adding dependencies via a reverse proxy. **The real runs use the 08/05 image.**

That means a **fresh container with a stale workspace**, because
`/workspaces/f1tenth` lives in the container filesystem and is **not** a bind
mount. Today's four changed files are not in that image.

**Do not try `git pull` — the Jetson has no internet.** The files are staged on
the SSD instead, which *is* bind-mounted into every container:

```bash
# host
/mnt/f1tenth_ssd/shared_dir/handoff/live_runs_20260805/
# same directory seen from inside any container
/mnt/shared_dir/handoff/live_runs_20260805/
```

So, inside the new container, before anything else:

```bash
bash /mnt/shared_dir/handoff/live_runs_20260805/install_handoff.sh
```

It copies the four files in (backing up whatever the image shipped), verifies
each by md5, confirms the three behavioural changes are actually present, and
checks that `install/` is symlinked to `src/`. **If it reports
`install/ holds a REAL COPY`, the image was not built with `--symlink-install`
and you must rebuild** — otherwise the `respawn_delay` fix silently does
nothing and the dead-stick window stays at 10.7 s:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to f1tenth_launch
```

It was smoke-tested against the old container on 2026-08-05 and passed all
checks. Expected output ends with `install OK.`

Note the VESC EIO crash below was observed on the **June 6** build. If the
08/05 image carries a newer `vesc_driver`, it may behave differently — watch for
it, and re-check before assuming the 2 s respawn is still the whole mitigation.

---

## Exact commands

Everything runs **inside the container**, from `scripts/live_runs/`:

```bash
ssh gosling1
docker exec -it <new_container_name> bash     # NOT f1tenth_claude_test — that was the old image
cd /workspaces/f1tenth/src/f1tenth_launch/scripts/live_runs
export ROS_DOMAIN_ID=0
```

Then:

```bash
./25_drive_session.sh launch --max-speed 1.0 -y   # ~2 min (RealSense reset + VSLAM)
./25_drive_session.sh status                      # confirm rates before recording

./25_drive_session.sh start mapping_drive         # verifies bytes are landing, then says "move the robot"
#   ...operator drives...
./25_drive_session.sh stop                        # closes bag + prints per-topic count/Hz audit

./25_drive_session.sh shutdown
./25_drive_session.sh launch --max-speed 1.5 -y   # relaunch for the trajectory bags

./25_drive_session.sh start loop_laps
./25_drive_session.sh stop
./25_drive_session.sh start figure8
./25_drive_session.sh stop

./25_drive_session.sh shutdown
```

Driving from a remote/agent shell (no TTY for Ctrl-C):

```bash
ssh gosling1 'docker exec -i f1tenth_claude_test bash -s' < some_script.sh
```

`bash -lc "…"` through two levels of quoting mangles `${...}` and `"$@"` —
pipe a script file on stdin instead. This cost several failed attempts.

### The operator's protocol

They asked for this loop explicitly, so follow it:

1. They say **"start"** → run `start <name>`, then tell them
   **"recording has started, move the robot now"** — but only after the growth
   check passes.
2. They drive, then say **"I am done, stop recording, on to the next bag"**.
3. Run `stop`, report the audit, move to the next bag.

**Never announce "recording" on faith.** `start` proves the recorder is alive
and that bytes are landing on disk (it measures growth over 4 s) before
printing the go-ahead. On the practice run that read 187 MB/s.

---

## What `25_drive_session.sh` is, and why it exists

`20/21/30_*.sh` are one-shot: launch → record one bag → tear down on Ctrl-C.
A run day that records three bags against one stack needs the opposite shape,
and needs to be drivable without an interactive Ctrl-C. `25_drive_session.sh`
brings the stack up **once** and starts/stops named bags against it, so every
bag in a session shares one calibration, one TF tree, one set of sensor clocks.

Subcommands: `launch`, `status`, `start <name>`, `stop`, `shutdown`,
`summarize <dir>`. State lives in `/mnt/shared_dir/run/session/` (on the SSD,
not `/tmp`, so a container restart cannot strand a running stack with no pid).
Stack and recorder both run under `setsid`, so they survive the ssh call that
started them and are signalled as process groups.

### Stack configuration it launches

`teleop.launch.py`, namespace `/gosling1` (auto-resolved from `$VEHICLE_NAME`),
`use_gpu:=True`, `publish_realsense_pointcloud:=True`, command gate armed with
heartbeat. Localization is **local only** — `launch_global_localization:=False`,
EKF owns `odom→base_link`. No SLAM, no Nav2: maps are built offline from these
bags, so nothing here needs a map.

### On adding `map→odom` to these bags later

The operator asked whether to update the bags once a map exists. **Don't
rewrite them.** Keep them as the pristine raw record and have the offline
pipeline emit *derived* bags, or play a TF-only bag alongside. Rewriting a raw
sensor bag means the mapping run can no longer be reproduced from the thing it
was built from.

---

## Findings from the practice run — read before driving

### 1. The VESC driver aborts on serial EIO (bug-068) — **the important one**

Mid-drive the car stopped for 10.7 s and then resumed on its own under constant
throttle. Cause:

```
[ERROR] [SerialPort::async_send_handler]: Input/output error
terminate called after throwing an instance of 'std::system_error'
[ERROR] vesc_driver_node-9: process has died [exit code -6]   ← SIGABRT
[INFO]  vesc_driver_node-9: process started with pid [10767]  ← respawn
```

**Every ROS topic upstream looked perfectly healthy throughout.** Of 3,828
messages where the operator demanded throttle, *zero* arrived at the VESC as
speed 0 — the mux, the command gate and `ackermann_to_vesc` were all fine. The
fault is downstream of the entire safety chain, because `ackermann_to_vesc`
(which publishes `commands/motor/speed`) is a different node from the driver
that writes the serial port.

Ruled out: supply brownout (`voltage_input` steady 11.50–11.70 V through the
stall) and steering (0.003 rad at gap start). Fired **3×** on 2026-08-05.

- **Done:** `respawn_delay` 10.0 → **2.0** in `launch/vehicle/vehicle.launch.py`,
  cutting the dead-stick window from 10.7 s to ~2 s. Live on the robot already
  (`--symlink-install`, no rebuild needed).
- **Done by operator:** reseating the VESC USB cable during the reboot.
- **STILL OPEN:** the driver should catch the serial exception and reconnect
  rather than abort, and the safety chain should be able to notice that the
  actuator is gone.

**If the car stops and restarts by itself mid-run, this is why.** Check
`grep 'process has died' /mnt/shared_dir/run/session/stack.log`, and check for
a gap in `vehicle/sensors/core` in the bag. Discard and re-drive that bag.

### 2. Steering clipped at full left lock (bug-071)

Hundreds of `servo command value (1.036000) above maximum limit (0.920000),
clipping.` With `servo = -1.4·angle + 0.56` and `servo_max 0.92`, anything past
`(0.92−0.56)/1.4 = 0.257 rad` clips. A clipped command makes the bag **lie** —
recorded steering keeps rising while the servo is pinned, which corrupts any
trajectory fit made from it.

- **Done:** `MAX_STEERING` default 0.34 → **0.25** in `00_env.sh` (live-run
  default only; package default untouched).
- **Scheduled:** operator recalibrates `steering_angle_to_servo_offset` toward
  0.5 over the weekend of 2026-08-08. Raise `MAX_STEERING` back to 0.34 after.

The operator initially read this as "steering kills the throttle". It is a
separate fault that coincided with #1; the throttle cut was entirely #1.

### 3. `wait_for_topic` gave a false pass on a dead LiDAR (bug-069, fixed)

`have_topic()` accepted any topic whose name resolved. A topic with only a
*subscriber* resolves and reports a `Type:`, so the filter chain's subscription
to a scan it never received was enough to pass — with the YDLidar driver dead
and `Publisher count: 0`. It now requires a live publisher.

### 4. `vehicle/sensors/imu/data` does not exist (bug-070, fixed)

There is no filter for the VESC IMU (`realsense_imu_filter` serves the camera).
The driver publishes `vehicle/sensors/imu` (`vesc_msgs/VescImuStamped`),
`…/imu/raw` and `…/imu/mag`, all ~97 Hz. The magnetometer was being dropped
from every bag. `TOPICS_VEHICLE` is corrected, which fixes every phase set.

**The practice bag `mapping_drive_145639` used the old 38-topic list** and is
therefore missing `imu` and `imu/mag`. Real runs get all 40.

### 5. The YDLidar needs a physical replug after repeated restarts

Symptom — port opens, device answers nothing:

```
Lidar successfully connected [/dev/ydlidar:128000]
[error] Error, cannot retrieve Lidar health code -1
[error] Fail to get baseplate device information!
[error] Failed to start scan mode -1
```

The driver then **exits**, so it will not recover on its own — the stack must be
relaunched after replugging. Unplug/replug the lidar, confirm
`/dev/ydlidar → ttyUSB*`, relaunch. Healthy is **~8.6 Hz** on `scan_filtered`
(the X4 is configured for 12 Hz but delivers ~8 under USB/CPU load).

---

## Verified-good baseline (practice bag, for comparison)

`mapping_drive_145639` — 171.5 s, 27.5 GiB, 359,356 msgs, **172 MB/s**:

| stream | Hz |
|---|---|
| `teleop`, `ackermann_drive`, `vehicle/ackermann_cmd`, `commands/*` | ~207 |
| `camera/imu`, `camera/imu/filtered` | 199.9 |
| `vehicle/sensors/imu/raw` | 93.7 |
| `vehicle/vesc_odom`, `vehicle/sensors/core` | 46.9 |
| `safety` | 40.0 |
| `odometry/local`, `tf` | 30.0 |
| `camera/color`, `depth`, `aligned_depth`, `infra1/2`, `points` | 25–32 |
| `visual_slam/tracking/odometry`, `vis/slam_odometry` | 26.1 |
| `lidar/scan`, `scan_filtered`, `odom/rf2o` | 8.7 |

**Budget ~11 GB/min.** A 5-minute mapping drive is ~56 GB.

`drive`, `cmd_vel` and `estop` are silent by design here (no Nav2/MPC running;
`estop` only carries traffic while held) and are listed in `SILENT_OK` in
`25_drive_session.sh` so they report `[ -- ]`, not `[FAIL]`. **Anything else
silent is a real defect.**

TF edges to confirm before recording: `odom→base_link`, `base_link→lidar`,
`base_link→camera_link`, `base_link→imu_link`.

---

## Gotchas

- **`ros2 topic hz` cannot tell you the command gate is open** — a closed gate
  publishes at full rate with zero payloads. Echo values, not rates.
- **`ros2 bag record` in Humble has no record-for-N-seconds flag.** `--duration`
  is rejected outright and `-d` is a *split* interval. Time a SIGINT yourself.
- **`metadata.yaml` is the only proof a bag is usable** — rosbag2 writes it only
  on clean shutdown. `stop` checks for it and refuses to claim success without
  it.
- The recorder took ~60 s to honour SIGINT on a 27 GiB bag and needed a TERM.
  That is normal for a large bag; `stop` handles it. Don't panic and kill it.
- `ros2 launch` ignores SIGINT here; `shutdown` escalates INT → TERM → KILL.
- CycloneDDS static-peer log spam floods `stack.log` (391k lines for one run).
  Filter with `grep -v ddsi_udp_conn_write`. See `CYCLONEDDS_PEERS.md`.
- The container workspace is **not** a bind mount. Edits made on the dev machine
  must be `scp` + `docker cp`'d in. `install/` *is* symlinked to `src/`, so
  launch/YAML edits need no rebuild.
- The robot's git checkout sits at an older commit than the dev machine even
  when the working files match. Compare with `md5sum`, not `git log`.
