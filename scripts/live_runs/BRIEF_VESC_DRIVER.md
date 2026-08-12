# Brief for the `vesc_driver` Claude — an open code defect (bug-068)

**Written 2026-08-06 by the `f1tenth_launch` agent.** I own the launch/config
package that starts your driver on an F1/10 car. You own `vesc_driver` (the
`vesc_ackermann` / `vesc_driver` stack). You have never seen this machine or my
repo, so this is self-contained.

**There is one defect here and it belongs to you, not to me.** I applied the only
mitigation available from a launch file and it is not a fix. Sections 1–3 are the
bug; sections 4–6 are how to get on the machine and reproduce it.

---

## 1. The bug, stated plainly

**`vesc_driver_node` does not catch the `std::system_error` thrown when a serial
write to the VESC returns `EIO`. It calls `std::terminate` and dies with
SIGABRT, and the entire ROS-side safety chain stays blind to it — the car goes
dead-stick under constant throttle while every command topic reports healthy.**

Observed on the vehicle `gosling1` (NVIDIA Jetson Orin Nano, ROS 2 Humble),
2026-08-05. Fired **3 times that day** — twice at launch, once mid-drive.

```
SerialPort::async_send_handler: Input/output error
terminate called after throwing an instance of 'std::system_error'
[vesc_driver_node-N] process has died [exit code -6]
```

Operator's account: *"the VESC died"* — the car stopped mid-drive and then
resumed on its own, under constant throttle, with no input change.

### 1a. Why it goes unnoticed — this is the serious half

`ackermann_to_vesc` (which publishes `vehicle/commands/motor/speed`) is a
**different node** from `vesc_driver_node` (which writes the serial port). When
the driver aborts, every upstream node keeps running perfectly:

| topic | during the outage |
|---|---|
| `teleop` | publishing normally |
| `ackermann_drive` (mux output) | publishing normally |
| `vehicle/ackermann_cmd` (safety gate output) | publishing normally |
| `vehicle/commands/motor/speed` | publishing normally, correct values |

**Measured: 0 of 3,828 throttle demands were dropped anywhere in the ROS chain
during the outage.** The priority mux reports healthy. The command gate reports
open. Nothing reaches the motor.

The only signal that anything is wrong is a **gap in the driver's own feedback**
— `vehicle/sensors/core` and `vehicle/vesc_odom` stop for the duration. That is
the diagnostic to look for; never inspect the command topics.

Measured dead-stick window on 2026-08-05: **10.686 s**, in the middle of a drive
(bag `mapping_drive_145639`, gap in `vehicle/sensors/core` starting at
t = 24.510 s).

### 1b. Ruled out as causes

Both of these were checked with data, so don't re-investigate them:

- **Not a supply brownout** — `voltage_input` was steady at 11.50–11.70 V
  straight through the stall.
- **Not steering load** — steering was 0.003 rad (essentially centred) at the
  moment the gap started.

### 1c. The trigger, which is separately fixed

The `EIO` was not a flaky cable. It was a **udev symlink collision** (bug-073):
a stale `ydlidar-V2.rules` matched USB ID `0483:5740`
(STMicroelectronics ChibiOS Virtual COM Port) — which is **the VESC's** USB CDC
identity, not the LiDAR's — and claimed `SYMLINK+=ydlidar`. After a VESC
re-enumeration, both `/dev/ydlidar` and `/dev/sensors/vesc` pointed at the same
`ttyACM0`. The YDLidar driver then opened the VESC's port and wrote 128000-baud
probe traffic into it while your driver was using it. The LiDAR got garbage
(`health code -1`) and exited; your write failed with `EIO` and SIGABRTed. It is
self-reinforcing — the abort triggers re-enumeration, which can re-steal the
symlink.

That rule is disabled on `gosling1` (still to do on the other cars).

**This does not close your bug.** A driver holding the actuator on a
safety-critical vehicle must survive a serial write error without aborting,
whatever caused the error — cable, re-enumeration, contention, brownout, a USB
hub hiccup. The udev fix removed *one* trigger.

---

## 2. The mitigation I applied, and why it is not a fix

`ros2 launch` respawns the driver, so `respawn_delay` in my
`launch/vehicle/vehicle.launch.py` is **literally how long the car is
dead-stick**. It was 10.0 s; I set it to **2.0 s** on 2026-08-05.

That is the whole extent of what a launch file can do. Two seconds of
uncommanded motion on a car that does 1.5+ m/s is still 3+ metres. Respawn also
loses whatever the driver was mid-way through and re-runs its startup handshake.

I am not going to paper over this further from my side; the reconnect belongs in
the driver.

---

## 3. What I think the fix looks like

Yours to design — you have the code, I don't. What the system needs from it:

1. **Catch the serial exception and reconnect instead of aborting.**
   Specifically, `SerialPort::async_send_handler` returning `EIO` (and the read
   side too) must not propagate a `std::system_error` out of a callback into
   `std::terminate`. Close, back off briefly, reopen the port, and re-establish.
   The device symlink may point somewhere new after a re-enumeration, so
   re-resolve the path rather than caching the fd's identity.

2. **Command state on reconnect must be safe, not stale.** After a reopen, do
   not replay the last commanded ERPM. The vehicle may have been unpowered or
   moved. Zero speed until a fresh command arrives is the conservative behaviour
   and matches what the rest of the safety chain does on timeout.

3. **Make the outage observable to the safety chain.** This is the part with no
   workaround anywhere else in the system. Something must publish "the actuator
   is not reachable" — a `diagnostic_msgs/DiagnosticStatus`, a latched health
   topic, a lifecycle transition, anything a downstream node can subscribe to.
   Right now the only evidence is the *absence* of `vehicle/sensors/core`
   messages, and nothing in the stack watches for that. With such a signal, my
   command gate (which already closes on a heartbeat timeout and publishes zeros)
   could gain an actuator-liveness input; without it, I have nothing to react to.

4. **Distinguish transient from fatal.** If reconnection fails repeatedly, that
   should escalate loudly rather than loop silently — a driver that spends 30 s
   quietly retrying is as dangerous as one that aborts, just harder to see.

Context on the mux/gate side, so you know what's downstream: commands flow
`drive` → priority mux → `ackermann_drive` → command gate → `vehicle/ackermann_cmd`
→ `ackermann_to_vesc` → `vehicle/commands/motor/speed` → **your driver** →
serial. Every link in that chain has a timeout and a fallback except the last
one. The gate already closes and publishes zeros within 0.5 s of a joystick
heartbeat loss; it just has no way to know the *serial port* is gone.

---

## 4. Secondary, lower priority — steering calibration asymmetry

Not the same bug, and it may be config rather than code, but it produces log
noise from your driver and one operator-visible artifact:

```
servo command value ... above maximum limit, clipping
```

The calibration is `servo = -1.4 * angle + 0.56`, servo range [0.08, 0.92]. The
0.56 offset (rather than 0.50) makes the physical range asymmetric:
**[−0.257 rad left, +0.343 rad right]**. At the package default
`max_steering = 0.34`, any full-left command asks for servo 1.036 and clips.

The driver clips correctly — this is not a safety issue. But **a clipped command
makes a recorded bag lie**: the logged steering keeps rising while the wheels
have stopped moving, which quietly corrupts any trajectory-tracking or
system-identification fit done from that bag. My workaround is
`MAX_STEERING = 0.25` in the run scripts, sacrificing ~27 % of right-turn range,
until the operator recalibrates `steering_angle_to_servo_offset` toward 0.50
(scheduled the weekend of 2026-08-08).

Possibly related, unresolved: `vehicle/vesc_odom` diverges **1.6–1.7 m** from the
fused estimate over a run *while reporting the correct total path length*, and
these bags show **~25° of odometry yaw drift per ~150 s run**. That signature —
right path length, wrong heading — points at the same steering calibration rather
than at the wheel-speed conversion (`erpm = 3750 * speed`). Worth a look from
your side if the odometry integration lives in your repo.

---

## 5. Getting on the machine

### 5.1 Connect and start a container

```bash
ssh gosling1                       # the operator has the SSH config
bash ./bolus_ws/f1tenth_launch.sh  # starts the container, interactive
```

That script is `jetson-containers run …` carrying the full flag set: X11,
`--device /dev/bus/usb`, `/dev/video*`, `--shm-size=8g`, the SSD bind mount, the
DDS config. **You need the USB device passthrough**, so this matters more for you
than for anyone else.

> **Never hand-build the container from `docker inspect` of a previous one.**
> Someone did on 2026-08-05, silently dropped `--device /dev/bus/usb` and
> `-v /tmp/.docker.xauth` among others, and burned 25 minutes and a battery
> before the failure was understood (bug-073).

The container self-names `jetson_container_<YYYYMMDD>_<HHMMSS>`; read it from
`docker ps`, then `docker exec -it <name> bash`.

**Unattended** (the script needs a TTY and a stdin that never closes):

```bash
mkfifo /tmp/vesc.fifo
setsid bash -c "exec sleep infinity > /tmp/vesc.fifo" </dev/null >/dev/null 2>&1 &
setsid script -qfc "bash $HOME/bolus_ws/f1tenth_launch.sh" /dev/null \
  < /tmp/vesc.fifo > ~/vesc_container.log 2>&1 &
```

### 5.2 Isolate yourself

```bash
export ROS_DOMAIN_ID=42     # NOT 0 — 0 is the operator's live stack
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export VEHICLE_NAME=gosling1
```

Everything on this vehicle is namespaced **`/gosling1`** and `/tf`/`/tf_static`
are remapped to relative `tf`/`tf_static`. Topic names are
`/gosling1/vehicle/sensors/core`, `/gosling1/vehicle/commands/motor/speed`, etc.

### 5.3 Getting your code in — there is no internet

`gosling1` has **no internet access**. `git pull` / `pip install` / `apt` inside
the container will not work.

`/workspaces/f1tenth` is **inside the container image, not a bind mount**. The
only bind mount is host `/mnt/f1tenth_ssd/shared_dir` → container
`/mnt/shared_dir`. So: `scp` your files to
`/mnt/f1tenth_ssd/shared_dir/handoff/vesc_driver/` from your dev machine, copy
them into the workspace from inside the container, and verify with `md5sum`
(the robot's git checkout can sit at an older commit than yours even when files
are byte-identical — compare hashes, not `git log`).

You are C++, so you must rebuild:

```bash
cd /workspaces/f1tenth
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-select vesc_driver
```

### 5.4 Storage

Host `/` is a **28 GB SD card at 95 % full**. Never write bags or logs there —
use `/mnt/shared_dir` (916 GB NVMe).

### 5.5 Devices — check these before launching anything

```bash
ls -l /dev/sensors/vesc      # must be -> ../ttyACM0
ls -l /dev/ydlidar           # must be a ttyUSB*, NEVER a ttyACM
udevadm info -q property -n /dev/ttyACM0 | grep DEVLINKS   # must NOT contain /dev/ydlidar
```

If `/dev/ydlidar` points at a `ttyACM`, you are about to reproduce §1c by
accident. In-container workaround: `ln -sfn ttyUSB0 /dev/ydlidar`. Host fix:
`mv /etc/udev/rules.d/ydlidar-V2.rules{,.disabled} && udevadm control
--reload-rules && udevadm trigger --subsystem-match=tty`.

The VESC device path is **hardcoded to `/dev/sensors/vesc`** in my config
(`config/vehicle/vesc.yaml`) and depends on a host udev rule.

---

## 6. Reproducing and testing

### 6.1 Post-mortem, no hardware needed

Bag `mapping_drive_145639` on the SSD contains a real occurrence. The signature:

```bash
# in the container
ros2 bag info /mnt/shared_dir/bags/20260805/mapping_drive_145639
# then look for the ~10.7 s gap in vehicle/sensors/core starting at t=24.510,
# with vehicle/commands/motor/speed continuous straight through it.
```

`scripts/analysis/bag_stats.py` in my repo prints per-topic rate and worst-gap
statistics, which is the fastest way to see it. Note `vehicle/sensors/core` is
**BEST_EFFORT QoS** — a `RELIABLE` subscriber gets nothing from it live.

### 6.2 Injecting the fault deliberately

Cheapest reproduction is to make a write fail while the driver holds the port.
Options, roughly in order of safety:

1. **Car on blocks, wheels off the ground.** Non-negotiable for anything that
   commands throttle.
2. **Physically unplug the VESC USB** mid-run, then replug. This is the honest
   test of reconnect — it exercises re-enumeration and a possibly-changed device
   node, which is what actually happened in the field.
3. **`usbreset` / unbind-rebind** the CDC-ACM device from sysfs for a
   deterministic, timing-controlled version of the same thing.
4. **Reproduce the original mechanism**: open `/dev/sensors/vesc` from a second
   process and write at a different baud while the driver is running. This is
   exactly what the LiDAR driver was doing. Only do this on blocks.

For each, the pass criteria are: the process **does not die**; `sensors/core`
resumes within a bounded time; the resumed state is **zero speed, not the stale
command**; and something observable told the rest of the system it was down.

### 6.3 Launching the vehicle stack

```bash
ros2 launch f1tenth_launch teleop.launch.py \
    launch_vehicle:=True launch_sensors:=False launch_localization:=False
```

Two things about my safety layer you will run into:

- **The command gate starts CLOSED and needs a joystick heartbeat.** The
  DualSense must be connected, or pass
  `command_gate_require_heartbeat:=False`. Without one of those, no command
  reaches your driver and you will think the driver is broken.
- **Do not use `ros2 topic hz` to check whether commands are flowing.** A closed
  gate publishes at full rate with zero payloads. `hz` counts messages, not
  values. **Echo the values.** This is the single most common way to be misled on
  this vehicle, and it is the same failure mode as your bug: full-rate,
  correct-looking traffic with nothing behind it.

Battery: the LiDAR and VESC drain the pack while idle. Check `voltage_input` and
`fault_code` on `vehicle/sensors/core` before each session and don't leave the
stack running while you think.

---

## 7. What "done" looks like

1. `vesc_driver_node` survives a serial `EIO` — reconnects, does not abort.
   Demonstrated by one of the §6.2 injections, with the process PID unchanged
   across the fault.
2. Post-reconnect command state is safe (zero until a fresh command).
3. An observable actuator-liveness signal exists that downstream nodes can
   subscribe to — with the topic/type documented, so I can wire it into the
   command gate on my side.
4. Repeated reconnect failure escalates loudly.
5. Measured worst-case time from fault to restored actuation, to compare against
   the current respawn-based **~2 s**.

Once (3) exists, tell the operator what it is called and I will consume it.

---

## 8. Cross-repo notes

- The operator (Boluwatife) is your only channel to me and to the machine.
- `respawn_delay: 2.0` in my `launch/vehicle/vehicle.launch.py` should stay until
  your reconnect lands; once it does, we should discuss whether respawn is still
  wanted at all.
- I keep a bug log; this is **bug-068**, with **bug-073** as its root-cause
  trigger (udev) and **bug-049** as a related actuation-path issue. Quote those
  IDs to the operator and I will know exactly what you mean.
- Please don't edit `f1tenth_launch` directly — send a diff through the operator.
  Several odd-looking choices in it (steering caps, timeouts, respawn delays) are
  load-bearing and were paid for in measurements on the car.
