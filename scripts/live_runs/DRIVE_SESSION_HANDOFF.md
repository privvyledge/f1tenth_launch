# Drive-Session Handoff — multi-bag recording on `gosling1`

**Written 2026-08-05 ~15:05 EDT.** Everything below was measured on hardware
that afternoon, not inferred from launch files.

The job: record three bags on `gosling1` for a downstream repo that will build
maps and generate waypoints **offline**. A practice run was completed and
verified; the real runs are still to do.

---

## Status — COMPLETE as of 2026-08-05 17:40 EDT

**All three bags are recorded, audited and on the SSD.** Session 3 (16:54–17:40)
finished the job. If you are here to *use* the bags, you only need the next
section and "Session 3". Sessions 1 and 2 are kept for the hardware history.

| | |
|---|---|
| Real runs | **done** — all three bags below, every topic at rate, clean `metadata.yaml` |
| Stack on the robot | **shut down**; operator rebooting onto AC power and disconnecting VESC + joystick |
| Container | `-it --rm`, dies with the operator's terminal — **create with `~/bolus_ws/f1tenth_launch.sh`, never by hand** |
| Open item | the `ydlidar-V2.rules` fix must be applied to the **other goslings** — see `UDEV_YDLIDAR_VESC_COLLISION.md` |

---

## The three delivered bags

`/mnt/shared_dir/bags/20260805/` in the container ·
`/mnt/f1tenth_ssd/shared_dir/bags/20260805/` on the host. 597 GB free.
**Never write to `/`** — the host root is a 28 GB SD card at 95%.

| bag | purpose | `max_speed` | duration | size | path | net displ. |
|---|---|---|---|---|---|---|
| `mapping_drive_170025` | offline 2D + 3D RTABMap source | 1.0 | 146.9 s | 24 G | 53.2 m | 0.75 m |
| `loop_laps_173558` | 2–3 laps of a loop, trajectory tracking | 1.5 | 103.1 s | 17 G | 37.6 m | 0.81 m |
| `figure8_172338` | 2–3 figure-8s, trajectory tracking | 1.5 | 155.4 s | 25 G | 40.6 m | 0.36 m |

All three share one identical 39-topic set (36 live + 3 silent-by-design), so
they are interchangeable downstream. All three **start with every localizer at
the origin**. Every one audited clean: 0 process deaths, 0 VESC EIO, 0
under-voltage faults, 0 servo clipping, `vehicle/sensors/core` max gap ≤ 0.065 s.

Actuator ground truth is present in all three: `vehicle/commands/motor/speed`
(ERPM, `std_msgs/Float64`), `vehicle/commands/servo/position` (normalized servo),
`vehicle/ackermann_cmd` (gated command, m/s + rad), `ackermann_drive` (pre-gate),
`teleop` (raw joystick intent), and measured `vehicle/sensors/core` (duty,
current, ERPM, voltage, fault code @ 50 Hz).

**Two things a downstream consumer must know:**

- **`MAX_STEERING` was 0.25 rad, not 0.34** (temporary, pending the
  `steering_angle_to_servo_offset` recalibration). Commanded steering is
  therefore *unclipped* in these bags — recorded steering equals actual servo
  demand, which is exactly what a trajectory fit needs. Contrast the practice
  bag `mapping_drive_145639`, which clipped (bug-071).
- **`mapping_drive_145639` (the 14:56 practice bag) is missing `imu` and
  `imu/mag`** — it used the old 38-topic list. The three delivered bags have all
  40. Prefer the delivered bags.

Superseded and deleted: `mapping_drive_162715` (flat drive battery, car rolled
for only 10.5% of it) and `loop_laps_171017` (valid, but started at
`odometry/local` (+1.001, +0.246) instead of the origin).

---

## Session 3 (2026-08-05 16:54–17:40) — the successful run, and the three things it found

Four bags recorded, three kept. Two hardware faults diagnosed from bag data, one
of which turned out to be the root cause of bug-068.

### Finding 1 — a flat drive battery looks exactly like "the speed cap is too low" (bug-072)

The operator's first attempt failed with *"1.0 m/s is too low when the car is on
the ground, it kept stopping and I wasn't able to drive."* That reading was
wrong, and the correction matters because the intuitive fix makes it worse.

The VESC (a 60/75 MK2) had latched **`fault_code 2` = `UNDER_VOLTAGE`** and was
tapering motor current toward its battery cutoff:

| t | throttle | commanded | measured | motor current | fault |
|---|---|---|---|---|---|
| 20–25 s | 0.138 | 534 erpm | 573 | 1.71 A | **2** |
| 25–30 s | 0.444 | 1661 | 638 | 2.27 A | **2** |
| 35–40 s | 0.706 | 2664 | 670 | **0.66 A** | **2** |
| 40–45 s | 0.687 | 2828 | **5** | **0.10 A** | **2** |

**Commanded ERPM rising while delivered current collapses is the signature.** A
genuinely-too-low speed cap looks the opposite: current present, ERPM tracking
the command, car creeping. Cross-run comparison settled it:

| | idle V | under load | fault 2 | rolling |
|---|---|---|---|---|
| flat pack (16:27) | 11.49 | 11.10 | 15.1% | 10.5% |
| practice pack (14:56) | 11.61 | 11.50 | 5.2% | 10.0% |
| fresh pack (17:00+) | **12.80** | 12.30–12.50 | **0.0%** | **55–66%** |

**Do not raise `max_speed` to work around this** — more speed draws more current,
sags the pack further and trips the cutoff sooner. Charge the pack.

**Rule adopted:** read `fault_code` and `voltage_input` from
`vehicle/sensors/core` as part of the pre-record check. Neither the mux, the
command gate, nor any command topic shows anything wrong when this fires — same
blind spot as bug-068.

### Finding 2 — `/dev/ydlidar` was pointing at the VESC (bug-073) — **root cause of bug-068**

`ydlidar-V2.rules` matches USB `0483:5740`, which is the **VESC's** USB CDC
identity (`STMicroelectronics ChibiOS_RT_Virtual_COM_Port`), not a LiDAR's. Both
`/dev/ydlidar` and `/dev/sensors/vesc` ended up on `ttyACM0`; the YDLidar driver
then opened the VESC's port and wrote 128000-baud probe traffic into it while
`vesc_driver` was using it. The LiDAR got `health code -1` and exited; the VESC
got `EIO` and `SIGABRT`ed.

**bug-068 was never a flaky VESC USB cable.** Before/after on one stack:

| | colliding | after fix |
|---|---|---|
| `lidar/scan_filtered` | 0 Hz (driver exited) | **8.59 Hz** |
| `process has died` per launch | 2 | **0** |
| `async_send_handler` EIO | 8 | **0** |

Full write-up, the fix, and the per-car check: **`UDEV_YDLIDAR_VESC_COLLISION.md`**.
Fixed on gosling1 2026-08-05 17:44; **still to do on the other goslings.**

### Finding 3 — relaunching the stack does *not* zero odometry

The operator asked for the localizers to be restarted between bags so odometry
would start at zero. Restarting is necessary but **not sufficient**: anything
that happens between relaunch and `start` is integrated, including *carrying the
car back to the start line*.

| bag | `odometry/local` first | `vesc_odom` first | VSLAM first |
|---|---|---|---|
| `loop_laps_171017` (relaunched, then car moved) | **(+1.001, +0.246)** | (0, 0) | **(+1.884, −0.925)** |
| `loop_laps_173558` (car parked, then relaunched) | (−0.005, −0.003) | (0, 0) | (+0.009, −0.003) |

`vesc_odom` reads zero in both because carrying the car doesn't turn the wheels —
so **`vesc_odom` being zero is not evidence the other localizers are zeroed.**

**Correct order: park the car at its start pose → relaunch → `start` immediately.**

### Other notes from this session

- **The RealSense worked first try** on `humble-devel-08052026` once the
  container came from `f1tenth_launch.sh`. Cause (a) from Session 2 is confirmed:
  the hand-built container's missing X/xauth plumbing, **not** an image
  regression. `/tmp/.docker.xauth` was a real 54-byte file and `xdpyinfo`
  succeeded from inside the container. A burst of libusb `control_transfer`
  warnings and two `Right MIPI error` notifications during the startup reset are
  **normal** and stop once streaming starts.
- **The Jetson's logic battery going flat kills the LiDAR too** — the X4 is
  bus-powered, and a sagging supply produces the same `health code -1` as the
  udev collision. Two different causes, one symptom; check the symlinks *and* the
  pack.
- **`25_drive_session.sh shutdown` can leave an orphaned
  `component_container_isolated`.** It survived `shutdown` on three of four
  attempts and had to be `pkill`ed separately. Always confirm with
  `pgrep -f "component_container|ros2 launch|ros2 bag record"` before relaunching.
- **A standalone `ydlidar.launch.py` probe is a cheap pre-flight** — it proves
  the LiDAR without powering the whole stack. Note it publishes **un-namespaced**
  (`/lidar/scan_filtered`, no `/gosling1` prefix), so don't poll the namespaced
  topic and conclude the sensor is dead.
- **`ros2 topic hz` under-reports vs a Python subscriber** using
  `spin_once` in a loop (32 Hz measured vs 50 Hz actual on `sensors/core`). Judge
  health by **max gap**, not by the rate a quick script reports.
- `vehicle/sensors/core` is **BEST_EFFORT**. A default-QoS subscriber gets
  `incompatible QoS ... RELIABILITY` and silently receives nothing.
- The bags are **mcap**, not sqlite3 — `storage_id='mcap'` when opening with
  `rosbag2_py`.

---

## Session 2 (2026-08-05 15:37–16:05) — what happened, and the two rules it produced

Nothing was recorded. The session went: found the robot in an unexpected state,
fixed that, installed the handoff, launched twice, RealSense failed both times,
operator called it and rebooted. ~25 minutes, LiDAR and VESC powered the whole
time on battery.

### What was found on the robot (all real, all still relevant after a reboot)

1. **No container from `humble-devel-08052026` was running.** The operator's
   `~/bolus_ws/f1tenth_launch.sh` starts one *interactively* (`-it --rm`), and
   after the reboot nothing had run it.
2. **A stale teleop stack had auto-started at boot** inside `docker-ros2-1`
   (compose project `~/bolus_ws/ros1_to_ros2_communication/docker`, image
   `privvyledge/f1tenth:humble-latest` from **May 2025**), running
   `teleop.launch.py … launch_vehicle:=True` in namespace `/gosling1` on
   **ROS_DOMAIN_ID=0** — the same domain the live-run scripts use. It held
   `/dev/sensors/vesc` and the joystick, and would have put duplicate nodes and
   duplicate TF publishers into any bag. It was stopped with `docker stop`
   (clean stop + `on-failure` policy = it stays down, but **re-check after every
   reboot**).
3. `docker-bridge-1` (foxy ROS1 bridge) is in a permanent restart loop. It was
   there during the good practice run too — ignore it, but don't mistake it for
   ours.

### The mistake that cost the session

The agent recreated the container **by hand**, replicating
`docker inspect f1tenth_claude_test` instead of using
`~/bolus_ws/f1tenth_launch.sh`. One mount in that copy (`/tmp/nv_jetson_model`)
failed, and the retry dropped several flags along with it — including the X11 /
`XAUTHORITY` plumbing and the explicit `--device /dev/video*` /
`--device /dev/bus/usb` lines.

The stack then came up with everything healthy **except** the camera:

```
[INFO]  [camera]: Device with name Intel RealSense D435I was found.
[INFO]  [camera]: Device USB type: 3.2
[INFO]  [camera]: Resetting device...
[INFO]  [camera]: Device with name Intel RealSense D435I was found.
[ERROR] [camera]: Error starting device: Could not open OpenGL window,
        please check your graphic drivers or use the textual SDK tools
```

librealsense **finds the D435i over USB 3.2 and then fails on an OpenGL/X
call**. Everything else was fine at the time — `lidar/scan_filtered` 8.654 Hz,
`vehicle/sensors/imu/raw` 100 Hz, `vehicle/vesc_odom` 50 Hz, `odometry/local`
30 Hz.

A second launch with `DISPLAY`/`XAUTHORITY` unset (headless) **also** failed —
`timed out after 120s waiting for camera/color/image_raw`. That attempt was
interrupted before its log was read, so **why the headless attempt failed is
unverified.** Do not assume it is the same cause.

Two candidate causes remain open, in priority order:

- **(a) the hand-built container** — missing `-v /tmp/.docker.xauth`,
  `XAUTHORITY`, `--device /dev/bus/usb`, `--device /dev/video0…5`,
  `--shm-size=8g`. Test first: it is free.
- **(b) an image regression** — the RealSense worked on
  `humble-devel-06062026` (the 27.5 GiB practice bag has all camera topics at
  25–32 Hz) and this is the first camera use on `humble-devel-08052026`.
  If (a) is ruled out, fall back to the June 6 image, which is still on disk and
  is a known-good camera path.

### Rule 1 — create the container with the operator's script, never by hand

```bash
ssh gosling1
./bolus_ws/f1tenth_launch.sh          # interactive; leave this terminal open
docker ps --format '{{.Names}}\t{{.Image}}'   # from a second shell — note the name
```

It names containers `jetson_container_<YYYYMMDD>_<HHMMSS>` and runs `-it --rm`,
so **it disappears when that terminal exits**. Give the agent the name.

If the agent must create it unattended, use the operator's flags verbatim with
only the interactive bits swapped — and `DISPLAY=:0`, which is what the working
June container used (`f1tenth_launch.sh` inherits `localhost:10.0` from the
operator's X-forwarded ssh session, which won't exist for a detached container):

```bash
docker run -d --name f1tenth_run --runtime nvidia --privileged \
  --network host --ipc host --shm-size=8g \
  --env NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
  -v /dev:/dev --device /dev/bus/usb --device /dev/ttyACM0 \
  --device /dev/video0 --device /dev/video1 --device /dev/video2 \
  --device /dev/video3 --device /dev/video4 --device /dev/video5 \
  --device /dev/i2c-0 --device /dev/i2c-1 --device /dev/i2c-2 --device /dev/i2c-4 \
  --device /dev/i2c-5 --device /dev/i2c-7 --device /dev/i2c-9 --device /dev/snd \
  -v /mnt/f1tenth_ssd/shared_dir:/mnt/shared_dir \
  -v /usr/lib/aarch64-linux-gnu/nvidia:/usr/lib/aarch64-linux-gnu/nvidia:ro \
  -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra:ro \
  -v /usr/lib/aarch64-linux-gnu/tegra-egl:/usr/lib/aarch64-linux-gnu/tegra-egl:ro \
  -v /tmp/argus_socket:/tmp/argus_socket -v /etc/enctune.conf:/etc/enctune.conf \
  -v /etc/nv_tegra_release:/etc/nv_tegra_release \
  -v /tmp/nv_jetson_model:/tmp/nv_jetson_model \
  -v /var/run/dbus:/var/run/dbus -v /run/jtop.sock:/run/jtop.sock \
  -v /home/gosling1/Downloads/jetson-containers/data:/data \
  -v /etc/localtime:/etc/localtime:ro -v /etc/timezone:/etc/timezone:ro \
  -v /tmp/.X11-unix/:/tmp/.X11-unix -v /tmp/.docker.xauth:/tmp/.docker.xauth \
  -e DISPLAY=:0 -e XAUTHORITY=/tmp/.docker.xauth -e QT_X11_NO_MITSHM=1 \
  -e VEHICLE_NAME=gosling1 -e USER=gosling1 -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///mnt/shared_dir/cyclonedds_config_static.xml \
  privvyledge/f1tenth:humble-devel-08052026 sleep infinity
```

`/tmp/.docker.xauth` **does not exist on the host** (the operator's own script
logs `xauth: file /tmp/.docker.xauth does not exist` and an `X_ChangeHosts`
BadValue error). Docker will then create it as an empty *directory*, which is
not a valid `XAUTHORITY`. If the camera fails on OpenGL again, create the file
first — `touch /tmp/.docker.xauth && xauth nlist :0 | sed -e 's/^..../ffff/' |
xauth -f /tmp/.docker.xauth nmerge -` — or drop both X flags and test headless.

### Rule 2 — prove the camera before the operator is asked to stand by

The camera is the one device that has failed twice and the one the launch script
waits 120 s for. Check it **while the car is still idle**, before saying
anything about launching:

```bash
docker exec -i <container> bash -lc 'ls -l /dev/sensors/vesc /dev/ydlidar; rs-enumerate-devices -s'
```

`rs-enumerate-devices -s` should list the D435i without opening a window. If it
errors, fix the container *before* burning battery on a stack launch.

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

### 1. The VESC driver aborts on serial EIO (bug-068) — **root-caused in Session 3**

> **Read this first: the cause was found on 2026-08-05 17:15 and it is not a
> cable.** `/dev/ydlidar` was pointing at the VESC because of a bad udev rule, so
> the LiDAR driver was writing into the VESC's serial port. See "Session 3,
> Finding 2" and `UDEV_YDLIDAR_VESC_COLLISION.md`. After the fix, four
> consecutive launches produced **zero** aborts on a car that had been aborting
> twice per launch. The analysis below is the original investigation, kept
> because the *detection* advice is still correct — and because the fault could
> in principle recur for other reasons.


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

### 5. The YDLidar "needs a physical replug" — usually it doesn't

> **Session 3 update.** This exact symptom has **three** causes, and a replug
> only fixes the third. Check in this order:
> 1. **`/dev/ydlidar` points at the VESC** (bug-073, udev rule collision) —
>    `ls -l /dev/ydlidar` must show a **ttyUSB**, never a ttyACM. See
>    `UDEV_YDLIDAR_VESC_COLLISION.md`. This was the cause on gosling1.
> 2. **The Jetson's logic battery is flat** — the X4 is bus-powered and a sagging
>    supply produces the identical `health code -1`. Put the Jetson on AC or a
>    fresh pack.
> 3. Only then, a genuine USB re-enumeration needing a replug.


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
