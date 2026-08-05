# `/dev/ydlidar` steals the VESC's port — udev rule collision

**Found on `gosling1`, 2026-08-05 17:15 EDT, during a live bag-recording session.**
Apply this check to every gosling. It is a one-line rule file, but it caused two
separate "hardware" failures that we had been chasing independently all day.

---

## Symptom

Either or both of these, intermittently, with no bad cable and no bad sensor:

1. **The YDLidar does not start.** The port opens, the device never answers:
   ```
   Lidar successfully connected [/dev/ydlidar:128000]
   [error] Error, cannot retrieve Lidar health code -1
   [error] Fail to get baseplate device information!
   [error] Failed to start scan mode -1
   ```
   The driver then **exits cleanly**, so nothing respawns it and
   `lidar/scan_filtered` sits at 0 Hz.

2. **The VESC driver aborts and the car goes dead-stick** (this is bug-068):
   ```
   [ERROR] [SerialPort::async_send_handler]: Input/output error
   terminate called after throwing an instance of 'std::system_error'
   [ERROR] vesc_driver_node-9: process has died [exit code -6]   <- SIGABRT
   ```
   Every ROS command topic upstream looks perfectly healthy while this happens,
   because the fault is below the entire safety chain.

**These are the same bug.** If you have been treating them as a flaky LiDAR and a
flaky VESC cable, stop — check the symlinks first.

---

## Root cause

Two udev rules both create a `/dev/ydlidar` symlink, and one of them matches the
**VESC**, not a LiDAR:

```
/etc/udev/rules.d/ydlidar.rules      KERNEL=="ttyUSB*", 10c4:ea60  SYMLINK+="ydlidar"   # CP2102 -> real YDLidar X4
/etc/udev/rules.d/ydlidar-V2.rules   KERNEL=="ttyACM*", 0483:5740  SYMLINK+="ydlidar"   # <-- THIS IS THE VESC
/etc/udev/rules.d/ydlidar-2303.rules KERNEL=="ttyUSB*", 067b:2303  SYMLINK+="ydlidar"   # PL2303 variant
/etc/udev/rules.d/99-vesc.rules      KERNEL=="ttyACM[0-9]*", 0483:5740 SYMLINK+="sensors/vesc"
```

`0483:5740` is `STMicroelectronics ChibiOS_RT_Virtual_COM_Port` — the USB CDC
identity that **VESC firmware presents**. It is a generic STM32 VCP ID, not
YDLidar-specific.

So both `99-vesc.rules` and `ydlidar-V2.rules` fire on the VESC, and the VESC ends
up owning **both** names. On gosling1 both symlinks pointed at the same device:

```
/dev/ydlidar      -> ttyACM0     # the VESC
/dev/sensors/vesc -> ../ttyACM0  # the VESC
```

`udevadm info` shows the collision directly — note `/dev/ydlidar` in the VESC's
`DEVLINKS`:

```
/dev/ttyUSB0  ID_VENDOR=Silicon_Labs  10c4:ea60
              DEVLINKS=... /dev/ydlidar ...            <- real LiDAR also claims it
/dev/ttyACM0  ID_VENDOR=STMicroelectronics  0483:5740
              DEVLINKS=... /dev/ydlidar /dev/sensors/vesc ...   <- VESC claims it too
```

**Whichever device enumerates last wins the symlink.** That is why this is
intermittent: boot order, a USB re-enumeration, or the VESC respawning after a
crash silently flips `/dev/ydlidar` from the LiDAR to the VESC.

### Why it takes the VESC down too

Once `/dev/ydlidar` points at the VESC, the YDLidar driver opens the VESC's serial
port and writes 128000-baud probe traffic into it *while `vesc_driver` is using
it*. Two processes on one tty:

- the LiDAR driver gets garbage back → `health code -1` → exits
- the VESC driver's write fails → `EIO` → `std::system_error` → **SIGABRT**

It is self-reinforcing: the VESC abort causes a re-enumeration, which re-runs the
udev rules, which can hand `/dev/ydlidar` back to the VESC again.

---

## Fix

On a car whose LiDAR is a **YDLidar X4** (CP2102, `10c4:ea60`), the V2 rule is
simply wrong and should go:

```bash
sudo mv /etc/udev/rules.d/ydlidar-V2.rules /etc/udev/rules.d/ydlidar-V2.rules.disabled
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
```

Then verify — **`ydlidar` must be a ttyUSB, `vesc` must be the ttyACM**:

```bash
ls -l /dev/ydlidar /dev/sensors/vesc
#   /dev/ydlidar      -> ttyUSB0
#   /dev/sensors/vesc -> ../ttyACM0

udevadm info -q property -n /dev/ttyACM0 | grep DEVLINKS
#   must NOT contain /dev/ydlidar
```

If a car genuinely has a V2-style LiDAR on an STM32 VCP, do **not** just delete
the rule — disambiguate by serial instead, since the product ID alone cannot tell
the two apart:

```
KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", \
  ATTRS{serial}=="<lidar_serial>", SYMLINK+="ydlidar"
```

Read the serials with `udevadm info -q property -n /dev/ttyACM0 | grep ID_SERIAL_SHORT`
(the VESC on gosling1 is `304`).

### Emergency in-session workaround (no sudo, survives until the next USB event)

From inside a privileged container with `-v /dev:/dev`, this fixes the host too:

```bash
ln -sfn ttyUSB0 /dev/ydlidar
```

Then relaunch the stack. This is what unblocked the 2026-08-05 session; it is
**not** a substitute for fixing the rule, because the next VESC re-enumeration
takes the symlink back.

---

## Verification that the fix works

Measured on gosling1, same stack, immediately before and after:

| | before (colliding) | after |
|---|---|---|
| `lidar/scan_filtered` | 0 Hz — driver exited | **8.59 Hz** |
| `process has died` per launch | 2 (`vesc_driver_node`, SIGABRT) | **0** |
| `async_send_handler` EIO errors | 8 | **0** |

The `figure8` bag recorded straight afterwards ran 155 s with a
`vehicle/sensors/core` max gap of 0.065 s and zero deaths — i.e. no VESC dropout
at all, on a car that had aborted twice at every launch beforehand.

---

## Check this on every gosling

```bash
ls -l /dev/ydlidar /dev/sensors/vesc          # must be different devices
ls /etc/udev/rules.d/ | grep ydlidar          # ydlidar-V2.rules present = vulnerable
```

A car can look fine right now and still be vulnerable — the symlink only flips
when the VESC happens to enumerate after the LiDAR. Treat the presence of
`ydlidar-V2.rules` alongside a CP2102 LiDAR as the defect, not the current
symlink state.
