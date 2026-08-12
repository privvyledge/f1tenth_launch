# One-battery session plan: MPC, Nav2, and the bags

> **Unblocked 2026-08-09 19:49 — but read this first.** At 19:15 the kernel
> remounted gosling1's root read-only after I/O errors (binaries on `/` failing,
> ssh dropping mid-command). A reboot fixed it: root is back `rw` with no mmc or
> EXT4 errors logged. Docker images, bags and the NVMe were never affected.
> **It is not established whether that was transient or an early sign of a dying
> card — treat a second occurrence as "replace the card", not "reboot again".**
> The card is still **97 % full (980 M free)**; the reclaim plan (~3.4 G of
> leftover JetPack local apt repos in `/var`, to be *relocated* to the NVMe, not
> deleted) is in `/tmp/f1tenth_handoff_sdcard_20260809.md`. Note the space is
> **not** ROS logs — all of `/home` is 705 M.

Written 2026-08-09, branch `perf/config-tuning`. This merges the three open
handoffs into a single running order so one battery covers all of it:

| track | doc | what it needs from this session |
|---|---|---|
| actuator sysid | `NEXT_CHAT_PROMPT.md` | Stage 4a confirmation drive, `k → 1.0` |
| heading + Nav2 | `NEXT_CHAT_HEADING_AND_NAV2.md` | live Nav2 (Job C), rf2o moving check (Job B) |
| IMU bias + rf2o | `NEXT_CHAT_IMU_BIAS.md` | rf2o moving test |

## The one scheduling fact that shapes everything

**MPC and Nav2 cannot share a bring-up.** `71_mpc_stack.sh` deliberately leaves
Nav2 and `twist_to_ackermann` **off** so that the mux's `navigation` channel
(topic `drive`) has exactly one publisher — your MPC. `60_nav2_test.sh` requires
`launch_twist_to_ackermann:=True`, which puts a second publisher on `drive`.

So the session is **two bring-ups**, not one. Bring-up is ~8.5 min each, and the
container stays warm in between — do not tear the container down between phases,
only the launch.

The bags are where the "one shot" saving actually lives: **the MPC-phase bag
serves the sysid Stage 4a and the rf2o moving test as well.** The `mpc` topic set
already records the full actuation chain, the VESC IMU, `odom/rf2o` and
`odometry/local`, and it was extended on 2026-08-09 to include VSLAM odometry so
a heading question can be answered against a source that is not fused into the
EKF. Nothing extra needs recording for those two tracks.

## What is already true, measured today

The rf2o zero-velocity gate is **verified on this vehicle, stationary**:

| source | baseline 2026-08-09 | run A | run B |
|---|---|---|---|
| `odom/rf2o` | +3.08 °/min (prior: −5.58, +4.60) | **+0.02** | **+0.40** |
| `odometry/local` | −0.30 °/min | +0.08 | −0.15 |

The gate is on by default, needs no parameter on this side, holds pose while
publishing exactly zero twist at 7.9 Hz (never silence), and loaded its shipped
0.05 rad/s angular threshold. What is **not** yet tested is the moving case —
specifically whether the gate ever latches while the car is genuinely creeping,
and whether the `odom2` Mahalanobis gates at 3.0 reject legitimate rf2o updates
mid-turn (`NEXT_CHAT_HEADING_AND_NAV2.md` Job B). Both need this drive.

### The moving test should be done offline first, and is not working yet

The 2026-08-05 bags already contain everything the moving test needs — the
figure-8 (`figure8_172338`, 155 s) carries 1334 `lidar/scan_filtered`, the **old**
node's `odom/rf2o` from the live recording, TF, and 15506 VESC gyro samples as a
yaw reference independent of the LiDAR. Replaying the scans through the new node
compares old against new on *identical* input, which no live drive can do, and
costs no battery.

`replay_moving.sh` and `analyze_replay.py` are staged in
`/mnt/f1tenth_ssd/shared_dir/rf2o_zv_0809/` and **do not work yet.** Across three
attempts the node logged `Waiting for laser_scans....` for the entire replay
while the scans were demonstrably flowing (`ros2 topic hz` on the replayed topic:
8.6 Hz; `ros2 node info` confirmed the subscription resolved to
`/gosling1/lidar/scan_filtered`; the player publishes RELIABLE, so QoS is
compatible). Ruled out so far: topic name, QoS, `use_sim_time`, and startup
order. Global-vs-namespaced `/tf` remapping was added and did not fix it either.

**The next thing to try is the obvious one and was not reached:** drive the
replay through `localization.launch.py` with `use_sim_time:=True` instead of a
bare `ros2 run`, so the node gets the exact namespace, remap and parameter
context the live stack gives it. Almost everything ruled out above is context the
launch file supplies and a hand-rolled `ros2 run` does not.

Until that works, the drive in Phase 3 is the only source of a moving answer —
so do not skip it.

---

## Phase 0 — before you power down (do now, on AC)

Nothing. Everything needed is staged on the SSD, which survives the reboot:

```
/mnt/f1tenth_ssd/shared_dir/rf2o_zv_0809/
  prep_container.sh          # container prep, idempotent, verified
  cfg_head.tar.gz            # f1tenth_launch config + launch + live_runs at HEAD
  0001-rf2o-zero-velocity.patch
```

## Phase 1 — power on, before anything else: the clock

The Jetson boots at 1969, and **chrony will not currently fix it**. Diagnosed
2026-08-09:

```
chronyc sources   ->  ^? 192.168.2.10 ... Reach 0      (never a valid sample)
chronyc tracking  ->  Reference ID 00000000, Stratum 0, Ref time Jan 1 1970
```

The Jetson's config is fine — `makestep 1 3` is set, so a 56-year step is
permitted, and `rtcsync` is on. The network path is fine too: same subnet
(`192.168.2.195/24` → `192.168.2.10`), ping 0 % loss at 2 ms. A raw NTP probe
gets a reply with the **correct time** but carrying `LI = 3 (ALARM / NOT
SYNCHRONISED)`, `stratum 0`, `refid 00000000`. The all-zero refid rules out a
kiss-of-death rejection (`DENY`/`RSTR`/`RATE`), so this is not access control:
**the lab server is answering but is itself unsynchronised**, almost certainly
because it has no upstream to reach without internet. Chrony is correctly
refusing to use a stratum-0 source.

So until the lab server is fixed, set the clock by hand, first, every boot:

```bash
date                                   # if the year is 1969, fix it now
sudo date -s "YYYY-MM-DD HH:MM:SS"
```

This is first because correcting it later jumps the clock 56 years forward
mid-run and silently empties any measurement using a wall-clock deadline, and it
puts a 56-year discontinuity in every bag recorded before the fix.

**Once the lab server does serve time, do not just trust it — wait for it.**
Chrony steps the clock a few seconds *after* boot, so a stack launched
immediately still starts at 1969 and then takes the step mid-run, which is the
same failure. Gate on synchronisation, not on elapsed time:

```bash
chronyc waitsync 60 0.1 && chronyc tracking | head -3
```

`prep_container.sh` checks this and refuses to pass if the clock looks unset.

### Fixing it properly (server side, one line)

The lab server needs either working upstream NTP, or to be told to serve its own
clock as a local reference. For chrony on that host:

```
local stratum 10
allow 192.168.2.0/24
```

`local stratum 10` makes it authoritative for the LAN even with no upstream —
absolute accuracy is then only as good as its own RTC, but for this work
"consistent across machines and not 1969" is the requirement, not UTC accuracy.
A GPS or RTC-backed stratum-1 is the answer only if absolute accuracy matters.

## Phase 2 — container, ~4 min

On the **desktop session** (not over ssh), or the RealSense comes up at 0 Hz
while the LiDAR looks fine and the stack reads half-healthy:

```bash
xhost +local:
xhost +SI:localuser:root
```

Then:

```bash
~/bolus_ws/f1tenth_launch.sh                       # never a hand-built docker run
/mnt/f1tenth_ssd/shared_dir/rf2o_zv_0809/prep_container.sh
```

`prep_container.sh` must print `is ready`. It re-does the three things that live
in the container layer and are therefore destroyed by the reboot:

1. **Config.** The image still carries `steering_angle_to_servo_gain: -1.4` and
   `wheelbase: .25`. It re-stages and then *greps* — a stale tarball reverts
   committed config silently, and every sysid number is void if this is wrong.
2. **The `twist_to_ackermann` sign fix.** Without it every Nav2 right turn
   steers hard left (bug-140). Nav2 phase depends on this.
3. **The rf2o zero-velocity gate.** The image's rf2o is one commit behind it.
   Without the rebuild the drive tests the *old* rf2o and today's stationary
   result does not carry over.

If it prints `PREP FAILED`, stop — do not spend battery on a run whose config is
unknown.

## Phase 3 — MPC, and the bag that serves three tracks

```bash
cd /workspaces/f1tenth/src/f1tenth_launch/scripts/live_runs
ROS_DOMAIN_ID=7 DDS_PROFILE=lo ./71_mpc_stack.sh
```

`DDS_PROFILE=lo` kills the CycloneDDS peer spam at source. **The cost: nothing
off-robot sees the topics, and it fails silently** — remote RViz shows an empty
world with no error. Drop it to `static` if you want RViz on the desktop.

Note that sourcing `00_env.sh` is also what sets `RMW_IMPLEMENTATION=
rmw_cyclonedds_cpp`. The container image sets `CYCLONEDDS_URI` but **not**
`RMW_IMPLEMENTATION` (bug-166, re-confirmed on `humble-devel-08052026` today),
so a launch started outside these scripts runs FastRTPS and reads no DDS config
at all. Use the scripts.

Then hand over to the MPC agent — `71_mpc_stack.sh` is the bring-up half only and
that agent runs its own checks. Handover is **hold R1 (SDL button 10)**; never
button 5, which is PS/power-off.

Check before trusting anything: `visual_slam/tracking/odometry` **by rate**, not
by existence. VSLAM aborts on roughly one launch in three, does not respawn, and
fails silently because `odometry/local` stays at 30 Hz on the remaining sources.
Mid-run that quietly hands heading to rf2o.

**Then, still in this bring-up and this bag: the drive loop.** A normal loop that
returns to the start pose. That single recording is:

- the sysid **Stage 4a** confirmation set — `k → 1.0` is measured from
  `vehicle/commands/servo/position` against TF-corrected gyro-z, both recorded;
- the **rf2o moving test** — closed loop back to start, so final heading can be
  compared against truth;
- the **`odom2` gate check** — watch whether rf2o's contribution disappears
  mid-turn, when deltas are largest.

Two cautions on reading it afterwards:

- Stage 4a's `k` is measured off gyro-z and servo command, so the new rf2o does
  **not** contaminate it. The tracks are genuinely separable in this one bag.
- Right-turn steering still clips: with the calibrated gain the no-clip range is
  +0.419 rad left / −0.314 rad right, so a loop driven mostly one way will
  saturate on the right and those samples are not usable for the fit.

## Phase 4 — Nav2

Stop the **launch**, keep the container. Tear down with a script piped on stdin
matching on *paths* (`/opt/ros/|/workspaces/f1tenth/install/`) plus
`ros2 launch|component_container`, excluding PID 1, TERM then KILL, then print
survivors. A pattern matching only node names missed 11 orphans last time, and
the resulting load average starved rf2o to 2.5 Hz and the LiDAR to 6.5 Hz —
**low-but-nonzero sensor rates can be CPU contention, not a sensor fault; check
`uptime` first.** A `pkill -f` typed inside `docker exec bash -lc "..."` matches
the exec'd shell's own command line and kills itself.

```bash
ROS_DOMAIN_ID=7 DDS_PROFILE=lo ./60_nav2_test.sh --map <map.yaml> --dry-run
ROS_DOMAIN_ID=7 DDS_PROFILE=lo ./60_nav2_test.sh --map <map.yaml>
```

Always the dry run first: it diverts the smoother to `cmd_vel_nav2` so Nav2 plans
and publishes exactly as normal but nothing reaches the actuation chain.

Four things bite, in the order they bite:

1. `launch_twist_to_ackermann:=True` — defaults `False` because the MPC publishes
   `drive` directly. Without it Nav2's `cmd_vel` never reaches the vehicle and
   everything looks healthy while the car sits still.
2. The twist fix must be applied (Phase 2 did it).
3. Gate the servers on lifecycle **ACTIVE**, not on topics or action servers
   existing — those appear at CONFIGURE, and goals are rejected while inactive
   with nothing logged anywhere (bug-126).
4. Goals must be **poses**. Rows from `maps/*/truth_<bag>.csv` are the
   `map→odom` transform and land on top of the robot (bug-128). Use
   `goal_poses_from_bag.py`.

On `log_level:=info`: worth it, and **log to `/mnt/f1tenth_ssd/shared_dir/logs/`,
never the Jetson home directory** — at `warn`, five bring-ups wrote 1.16 GB into
`/` and took it to 100 %, 0 bytes free. `/` is at 96 % with ~1.1 GB free right
now. Also note there is **no recorded diagnosis** of the previous "controller
stopped shy of the goal" observation anywhere in this repo — treat it as an open
question to investigate, not a known issue to confirm.

## Phase 4b — IMU gyro bias, optional, and only if there is battery left

**Do this offline instead if you can.** `bags/20260805/figure8_172338` already
contains `camera/imu` (31010 msgs) and `vehicle/vesc_odom` (7751) across a 155 s
drive with stationary periods at both ends — which is the node's entire input.
Replaying those two topics through `imu_bias_remover` answers the question with
no car and no battery, and it is a far easier replay than the rf2o one that
failed on 2026-08-09 (this node touches neither TF nor the scan pipeline).
Recipe and pass criteria are in `docs/imu_bias_removal_spec.md` §8.

The chain is wired and proven to pass data end-to-end, but **the correction
itself has never been measured** — the only test so far ran without the vehicle
stack, so `vesc_odom` never published, the node never accumulated, and `bias`
held exactly 0.0 as a pass-through. `remove_imu_bias` is `'False'` in the repo.

If you do want it on the car this session, it is a **separate bring-up after
the MPC, Nav2 and Stage 4a bags are already recorded**, with:

```bash
# in-container, temporary; do not commit
sed -i "s/'remove_imu_bias': 'False'/'remove_imu_bias': 'True'/" \
  /workspaces/f1tenth/src/f1tenth_launch/launch/sensors/realsense_d435i.launch.py
```

and record `/bias`, `/camera/imu`, `/camera/imu/bias_removed`. Pass criterion:
with the car parked and the **VESC powered**, `bias.z` converges toward
**−0.00214 rad/s**. A parked drift measurement proves nothing here — the node
zeroes the rate while stationary, so `yaw_drift.py` reads ≈0 whether or not the
correction works.

Two hard rules, both from `docs/imu_bias_removal_spec.md` §8:

1. **Never enabled during an autonomous MPC or Nav2 run.** The stationary test
   has no staleness timeout, and this vehicle's VESC driver aborts on serial EIO
   while its command topics still look healthy. If `vesc_odom` dies while parked,
   the gyro rate is pinned at zero indefinitely — including mid-drive, feeding a
   silently wrong yaw rate into the EKF.
2. **Never on the same bring-up as a Stage 4a bag.** `k → 1.0` must be measured
   against a fusion configuration that is not simultaneously changing.

## Phase 5 — before shutdown

- Copy anything you want to keep off the container; it is `AutoRemove=true`.
- Bags and logs are already on the SSD if the scripts were used.

## Battery notes

The Jetson and the VESC are on **separate** batteries. `voltage_input` on
`vehicle/sensors/core` is the *drive pack only* — the Jetson's supply has no ROS
topic at all, so there is no warning before it dies. Budget by wall clock, not by
telemetry, and put the phase you care about most first.

Given that, the running order above puts MPC first because that is the phase you
asked for first and the one whose bag serves the other two tracks. If battery
gets tight, **Phase 3 is the one to protect** — Nav2 can be repeated on AC power
with the car on a stand for everything except the final on-floor run.
