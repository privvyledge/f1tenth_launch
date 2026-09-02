# Demo runbook — MPC, then Nav2 with obstacles

**gosling1 · image `humble-devel-08302026` · `ROS_DOMAIN_ID=42` · map `20260805/rtabmap_2d_final.yaml`**

---

# 0★ · CURRENT PROCEDURE — verified end to end 2026-08-27

**Read this section instead of §§0–2 below.** Everything under it still holds unless contradicted
here; the parts that went stale are marked inline. Verified on a session that ran eight cold
launches and drove the car under Nav2.

## 0★.1 · Machine check (30 s, ssh)

```bash
date                     # 1969 => fix before ANY timed measurement or bag
uptime                   # load; a busy machine starves rf2o and the LiDAR
ls /dev/sensors/vesc /dev/input/js0   # VESC needs its battery on to enumerate
lsusb | grep -iE "intel|cp210"        # RealSense + the CP210x (that IS the YDLidar)
```

- **The `mount | grep " / "` / "SD card is failing" advice below is RETIRED.** Since the 2026-08-24
  reflash the M.2 is root, ~915 G with ~770 G free. `/dev/ttyUSB0` in the old §0 is also wrong —
  the LiDAR shows up as the CP210x bridge.
- **RealSense health**, when the camera is silent — count kernel re-enumerations, do not guess:
  `docker exec $C bash -lc 'dmesg | grep -c "Found UVC 1.50 device"'`, wait 20 s idle, count again.
  A burst of 4–16 every time anything opens it = **physical, reseat the cable** (bug-255). 2–3 at
  boot and a `/sys/.../speed` of 5000 or 10000 is healthy.

## 0★.2 · Container and staging

```bash
bash ~/bolus_ws/f1tenth_launch.sh                 # T1, the Jetson desktop session, NOT ssh
docker exec $C bash -lc "bash /mnt/shared_dir/stage_0901.sh"     # ~1 min, idempotent, NO NETWORK
```

- **The image changed on 2026-08-30: `humble-devel-08302026`.** It is a `docker commit` of a
  container already staged by `stage_0830b.sh`, so **a fresh container off it is staged at birth** —
  `stagecheck.sh` passes on it, the 20260805 map set is in the installed tree, `imu_processors`
  carries `stationary_timeout`, and the rviz ParticleCloud display is present. Verified by running
  the checks inside a throwaway container off the new image, not just inside the one that was
  committed. `~/bolus_ws/f1tenth_launch.sh` now points at it; the previous tag is on a commented
  line one above, and `f1tenth_launch.sh.bak-20260830` is the untouched original.
- **Still run the stage script after pulling new commits.** The image is a snapshot of a working
  tree, not a reproducible build — it freezes the repo at `9f97980` and nothing later.
  `stage_0830b.sh` is the reproducible artifact.
- **`humble-devel-08092026` (the previous tag) is UNSTAGED** and serves the retired raslab map and
  the Jan-2024 cloud, which renders exactly like a rotated 3D map. If you roll back to it, stage
  before launching.

- **`stage_0901.sh` is CURRENT (2026-09-01 evening, git HEAD `f50e12d`) and is the one to run.**
  Tarball `f1tenth_stage_20260901.tgz` (md5 `5a7cc874a49bfd8e6d62f970e69d6b12`, **199** entries),
  **twenty-one** checks. On top of `stage_0831.sh` it carries the exec-bit fix (bug-268), the
  `stop_launch_tree` teardown (bug-269), the `velox1` DDS profile, and the bug-265/266 cloud
  fingerprint in `10_preflight.sh`. Ran on gosling1 2026-09-01, all checks pass.
  **Note `scripts/` is NOT installed by CMakeLists** — it is used out of
  `src/f1tenth_launch/scripts/`, so script checks read `$S`, not `$I`. The first cut of this
  script got that wrong and reported five spurious MISSINGs.
- **`stage_0831.sh` (superseded) now lives in the repo** (`scripts/live_runs/stage_0831.sh`, tracked
  2026-09-01) as well as at `/mnt/shared_dir/stage_0831.sh` on the robot. Until then it existed
  **only on the SSD**, which put the whole map-set-in-the-tarball fix one reflash away from
  being lost — the same class of single-copy risk the fix itself was written to close. The
  robot copy is still the one that runs; the repo copy is the record. **If you edit one, copy
  it to the other**, and note the tarball it references (`f1tenth_stage_20260831.tgz`) is data
  and stays on the SSD.
- **`stage_0831.sh` is CURRENT (2026-08-30 night) and is the one to run.** It carries
  `f1tenth_stage_20260831.tgz` (md5 `0a9f6da6d5b229d7f5cd5bbfa5ff3703`, **198** entries,
  `git archive` over git HEAD `a8e027f`) and verifies **sixteen** values. On top of `stage_0830b.sh`
  it adds the bug-265 fix (`a51bdc6`, the cloud re-expressed in the grid's frame) and the deletion
  of `map_cloud_align.py` (`a8e027f`).
- **The map set now ships INSIDE the tarball, and that is the point.** `stage_0830b.sh` copied
  `data/maps/20260805/` out of `/mnt/shared_dir/maps/20260805/` *after* unpacking, so the SSD copy
  silently won over whatever git HEAD held — precisely the drift that would have reverted the
  bug-265 cloud fix on the next re-stage. `data/maps/20260805` is now in the `git archive` path set
  and nothing is copied over it. **Keep it that way**: the grid, the cloud and the AMCL seed must
  move in lockstep, and one git commit is the only thing that guarantees it. Two of the sixteen
  checks read the cloud's own header — `POINTS 96376` (de-rotated) and `FIELDS x y z rgb`
  (bug-266, colour intact).
- **`stage_0830b.sh` (superseded)** It carries
  `f1tenth_stage_20260830b.tgz` (md5 `19faed92cc4e6188efd66b37630ad916`, 189 entries, `git archive`
  over git HEAD `9f97980`) and verifies **fourteen** values in the installed tree. On top of
  `stage_0830.sh` it adds the `f1tenth.rviz` ParticleCloud display (`07c5149`, bug-261) and the
  stagecheck guards (`9f97980`), plus two checks the earlier scripts did not make: that
  `data/maps/20260805/` is present in the **installed** tree, and that the rviz display is there.
- **It needs no network, and that is the point.** `stage_0830.sh`'s step 2 cloned
  `privvyledge/imu_pipeline` from github, and that clone **fails on the lab network** — TCP 443
  connects and the transfer then times out at ~130 s, reproduced three times on 2026-08-30. Because
  the script is `set -eo pipefail`, it aborted *after* unpacking the tarball and *before* the build,
  leaving the launch files pointing at `data/maps/20260805/` while the installed tree had no such
  directory — **a state strictly worse than an unstaged container**, since `map_file` then resolves
  to a path that does not exist and `map_server` loads no map at all. `stage_0830b.sh` unpacks the
  same tree from `/mnt/shared_dir/imu_pipeline_58d227e.tgz` (= `humble-devel` @ `58d227e`, the commit
  adding `ImuBiasRemover`'s `stationary_timeout`) instead of cloning it.
- **If you ever must re-fetch that fork:** clone it on a machine with working internet, `tar czf`
  it, `scp` it to `/mnt/f1tenth_ssd/shared_dir/`. Do not wait on the Jetson's clone.
- **`stage_0830.sh` (superseded)** carried `f1tenth_stage_20260830.tgz` from `e07c2e1` and the
  bug-260 teleop fix; twelve checks; requires internet.
- **`stage_0827b.sh` supersedes `stage_0827.sh`** — it carries `f1tenth_stage_20260827b.tgz`
  (md5 `e61a8151d0d074f7b581808d41b1e912`, cut from `aced708`), which adds the
  `container_multi_threaded` fix, and checks eleven values instead of nine. The 0827 entry below is
  kept because its md5 is what the earlier runs of that evening were staged from.
- **`stage_0827.sh` supersedes `stage_0826c.sh`** (which superseded `prep_container.sh` in §2
  below). It carries `f1tenth_stage_20260827.tgz`, md5 `9c1c6b66fb8531e8fb4c48e917908655`, cut from
  git HEAD `614c463` with `git archive` over the same path set as 0826c — so it adds
  `movement_time_allowance: 10.0` and the `nav2_goal_probe.py` live-robot fix on top of everything
  0826c carried. It verifies **nine** values in the *installed* tree (the ninth is
  `movement_time_allowance`) and rebuilds the `imu_processors` fork. **Re-staging `0826c` reverts
  the `movement_time_allowance` change** — the tarball predates it. Add `FLIP=1` only for a bias-remover
  wiring test; omit it for the committed `remove_imu_bias:='False'`.
- **`/mnt/shared_dir` is the IN-CONTAINER path. On the host it is `/mnt/f1tenth_ssd/shared_dir/`.**
  There is no `/mnt/shared_dir` on the Jetson itself.
- `package 'f1tenth_launch' not found, searching: ['/opt/ros/humble']` is an **unsourced overlay**,
  not a failed build (bug-254). The searched-paths list is the diagnostic.

## 0★.3 · The launch environment — every shell, including diagnosing shells

```bash
source /workspaces/f1tenth/install/setup.bash
export ROS_DOMAIN_ID=42
export CYCLONEDDS_URI=file:///mnt/shared_dir/cyclonedds_offline_lo.xml  # /mnt/shared_dir/cyclonedds_offline_lo.xml or /mnt/shared_dir/cyclonedds_velox1.xml
export ROS_LOG_DIR=/mnt/shared_dir/<run-dir>/log_<name>       # never the Jetson home
```

- **A shell on the wrong domain or URI sees an empty graph and reads as a dead stack.** This is the
  same class of error as suppressing stderr on a check whose negative result you will act on.
- **`RMW_IMPLEMENTATION` is now set in the container environment** — the §4 warning below that it is
  not, and that a raw `ros2 launch` silently runs FastRTPS, is **STALE as of 2026-08-25**. Verify in
  one command rather than trusting either statement: `echo $RMW_IMPLEMENTATION`.
- **Why `_lo` and not the container default**, measured 2026-08-27: the static config binds
  `wlP1p1s0` and lists four remote robots, **all of which pinged ABSENT** that day, costing ~200
  failed `sendto`/sec per process across 41 nodes. RViz is unaffected — it runs same-host and
  reaches you over SSH X11, not DDS.
- **STALE, corrected 2026-09-01:** this bullet used to add "both configs already carry `lo` at
  priority 10, so the loopback fix is in both". `lo` at priority **10** was the bug — it outranks
  the physical NIC, which silently breaks cross-machine discovery and is the whole `retcode -3`
  flood. Both `cyclonedds_config_static.xml` and `cyclonedds_velox1.xml` now carry `lo` at
  **priority 0**, below the NIC, and the loopback benefit is unaffected (measured). The ~200
  failed `sendto`/sec figure above also predates that fix — the count is now **zero**.
- **Choose the profile by what the run needs, not by noise:** `_lo` for local-only runs
  (off-robot is invisible, with no error); `cyclonedds_velox1.xml` when velox1 publishes drive
  commands. Both are quiet now.

## 0★.3b · Remote drive commands from velox1 (`192.168.2.13`) — **VALIDATED ON THE PAIR 2026-09-01**

velox1 is going to publish drive commands to the car, so it has to be a DDS peer.

**Status 2026-09-01: built and validated end to end**, `demo_nodes_cpp talker` on gosling1 seen
and echoed from a container on velox1, with **zero `retcode -3` on either end**. Files:
`/mnt/f1tenth_ssd/shared_dir/cyclonedds_velox1.xml` (robot) and
`/home/digitalstorm/f1tenth_dds/cyclonedds_velox1.xml` (velox1). Not yet exercised with the
real stack — steps 4-6 of the table below still need a bringup.

**The interface PRIORITY is the whole trick, and the first attempt failed on it.** See the
box below; do not copy the static file's `lo priority="10"` into a profile that has to reach
another machine.

Two facts to get straight before editing any XML:

- **Adding the peer to `cyclonedds_offline_lo.xml` does nothing.** That profile binds **`lo`
  only** and carries a single `localhost` peer. A `192.168.2.x` address is unroutable over `lo`,
  so `<Peer address="192.168.2.13"/>` there buys one `EHOSTUNREACH` per announcement and no
  discovery. **The peer list is not the whole config — `<Interfaces>` has to carry the WiFi NIC
  (`wlP1p1s0`) as well.** This is the same mechanism that makes the static profile spam
  `retcode -3` (`CYCLONEDDS_PEERS.md`, correction of 2026-08-07).
- **`cyclonedds_config_static.xml` already binds `wlP1p1s0`, but does not list `.13`** — and it
  does list four other robots at ~200 failed `sendto`/sec each.

So neither shipped profile is right. Make a third: **`lo` + WiFi, exactly one remote peer.**

**Derive it from the known-good static file rather than typing XML from scratch** — the
`<Interfaces>` block and the `<AllowMulticast>false</AllowMulticast>` line have to survive. The
**one thing that must NOT survive verbatim is the `lo` entry's `priority="10"`**:

> ### `lo` must rank BELOW the physical NIC — measured on the pair, 2026-09-01
>
> With `lo` at priority 10 and the NIC at `default`, `lo` outranks the NIC, and the participant
> **advertises `127.0.0.1` as its preferred locator**. The remote peer dutifully tries to reach
> it there, hits its own loopback, and discovers nothing — **silently, with a healthy-looking
> config on both ends**. `ros2 node list` on velox1 was empty while `ping` and raw UDP passed
> cleanly in **both** directions.
>
> The same ordering is also the entire source of the `retcode -3` flood: every announcement to a
> remote peer is attempted on the `lo` socket first, where a `192.168.2.x` address is
> unroutable, so each one costs an `EHOSTUNREACH`.
>
> Set the NIC to `priority="20"` and `lo` to `priority="0"`. Measured effect, all three at once:
> cross-machine discovery **works**; `retcode -3` count goes **3600 → 0** on both ends; and the
> same-host loopback preference that the VSLAM-jitter fix depends on is **preserved** — a 133 MB
> two-container transfer on gosling1 put **132.6 MB on `lo` / 0 KB on WiFi** at `lo=10` and
> **133.8 MB on `lo` / 639 KB on WiFi** at `lo=0`, the 639 KB being peer announcements that
> previously failed. The bulk data path does not move.
>
> **This retires the standing claim in `CYCLONEDDS_PEERS.md`** that silencing the spam requires
> giving up the `lo` entry and is therefore "off the table by operator decision". It was a
> priority-ordering defect, not a trade-off.

```bash
# in the container; the SSD copy survives a container rebuild
cd /mnt/shared_dir
cp cyclonedds_config_static.xml cyclonedds_velox1.xml
$EDITOR cyclonedds_velox1.xml     # or sed; the whole edit is inside <Peers>
```

`<Peers>` must end up exactly this — everything else in the file untouched:

```xml
<Discovery>
    <Peers>
        <Peer address="localhost"/>       <!-- same-host, over lo -->
        <Peer address="192.168.2.195"/>   <!-- gosling1 (self) -->
        <Peer address="192.168.2.13"/>    <!-- velox1 — sends drive commands -->
        <!-- Every other robot REMOVED on purpose. A configured peer costs ~200 failed
             sendto()/sec per process whether or not it is powered on, and 41 nodes
             multiply that. Re-add one only while it is actually needed. -->
    </Peers>
    <ParticipantIndex>auto</ParticipantIndex>
    <MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>
</Discovery>
```

Then launch with it instead of `_lo` — **every shell, the diagnosing ones included**:

```bash
export CYCLONEDDS_URI=file:///mnt/shared_dir/cyclonedds_velox1.xml
```

**velox1 needs the mirror of this, not just the same domain.** Multicast is off on both ends, so
static peering is symmetric — if velox1 does not list the robot, it will never discover it, with
no error on either side:

```bash
# on velox1 (fish shell — pipe scripts on stdin: ssh velox1 bash -s <<'EOF' … EOF)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp     # inert CYCLONEDDS_URI without this
export ROS_DOMAIN_ID=42                          # must match the robot
export CYCLONEDDS_URI=file://$HOME/cyclonedds_velox1.xml   # <Peer address="192.168.2.195"/>
```

**Validation at the lab, in this order** — each step distinguishes a real failure from the next:

| # | On | Command | Pass |
|---|---|---|---|
| 1 | either | `ping -c3 192.168.2.13` / `ping -c3 192.168.2.195` | replies both ways; a firewall drop here is not a DDS problem |
| 2 | robot | `grep -c 'ddsi_udp_conn_write' <launch stdout>` after 60 s | **zero.** Measured 0 on both ends 2026-09-01 with `lo` below the NIC. A nonzero count means the `lo` priority regressed — fix that before reading any step below |
| 3 | velox1 | `ros2 node list` | the robot's nodes appear. Empty = domain or peer-list mismatch, **not** a stack fault |
| 4 | velox1 | `ros2 topic hz /odometry/local` | ~30 Hz. Measure by name; `ros2 topic list` returns a partial graph under static peers |
| 5 | velox1 | `ros2 topic pub` a zero-speed `AckermannDriveStamped` on `drive` | robot-side `ros2 topic echo /drive` shows it — **echo values, not `hz`** |
| 6 | robot | `ros2 topic echo /vehicle/ackermann_cmd` | the command reaches the gate. Zeros with the joystick disconnected are the gate holding shut, working as designed |

**Open question — SETTLED 2026-09-01.** `.194` *was* velox1, on WiFi. velox1 has since moved to
**wired `eno1` at `.13`**, its WiFi NIC (`wlx3c37862315ae`) is DOWN with no address, and `.194`
has been **reassigned by DHCP to some other machine** — it answers today from a different MAC
(`68:3e:26:c5:15:52` vs velox1's `f0:2f:74:cc:b9:b1`). So `.194` is neither velox1 nor a viz
desktop any more: the label in `CYCLONEDDS_PEERS.md` is stale, and `.194` must **not** be in
this profile — it would announce to an unrelated machine. Confirmed by the operator, who set up
the original config while velox1 was still on WiFi.

**General lesson: a peer list keyed on DHCP addresses decays without anyone noticing.** Check
`ip -br addr` and the MAC, not the hostname or the label.

**Bandwidth caveat, unmeasured.** Once WiFi is bound, every topic velox1 subscribes to crosses
the air. Drive commands are tiny; a pointcloud or an image stream is not — see the note on
running RViz off-board in `RUNBOOK.md`.

## 0★.4 · Pose seeding is now AUTOMATIC — do not seed by hand

**bug-241 was verified on hardware 2026-08-27: six cold launches, six passes, zero identities**
(and two more later the same session). `localization.launch.py` reads `initial_pose` out of
`localizer_amcl.yaml` and publishes it on `initialpose` at `initialpose_seed_delay` (20 s), seeding
AMCL and `ekf_map` from one number. Expected reading, ~35 s after launch:

```bash
ros2 run tf2_ros tf2_echo map base_link    # ~(0.445, -0.575, -84.5 deg), NOT identity
```

Across the eight launches: x 0.440–0.449, y 0.572–0.578, yaw −83.09° to −84.73°. **Identity means
the bug-238 race fired** — diagnose with `ros2 topic info /amcl_pose --verbose` and read the
**durability** of both ends, never the subscriber count. The manual `seed_initialpose.py` in §4
below is now a fallback, not a step.

**Parked at the spot, `map→base_link` ≈ (0.45, −0.57, −84.5°) is CORRECT, not drift.** And
`odom→base_link` on a car that has not moved should be ≈ 0 — measured (0.003, 0.005, −0.13°) on
2026-08-27, which **closes bug-231**. Read it as an absolute, never as drift-since-first-sample.

## 0★.5 · Expected noise that is NOT a fault

- **`visual_slam_node: Map folder '' does not exist. / Failed to localize in map. Error 4`**, five
  times per launch, at the seed. This is the bug-245 gate **working**: the map path is correctly
  empty, and cuVSLAM still reads the seed as a relocalization hint because
  `visual_slam/initial_pose` is remapped to `initialpose` unconditionally (**bug-256**, open, noise
  only). Before the fix it was `Error 3` and the container died **exit −6**.
- **`twist_to_ackermann: Saturating steering_angle`** during a turn. Correct behaviour: Nav2 asks
  for turns tighter than 18° and the node clamps to the configured 0.314 rad. Hit **12 %** of
  commands on a real drive.
- **rf2o alternating "Motion detected" / "Zero-velocity detected"** while parked, at v ≈ 0.02–0.03
  m/s. The gate chatters at its threshold; it contributes −0.17 °/min and fused yaw is unaffected.
  **Do not retune it.**

## 0★.6 · Nav2 demo — the sequence that worked

```bash
# ... the 0*.3 exports first, then:
ros2 launch f1tenth_launch bringup.launch.py \
  slam:=False launch_navigation:=True launch_visualization:=True \
  launch_twist_to_ackermann:=True log_level:=info \
  > $ROS_LOG_DIR/launch_stdout.log 2>&1
```

1. **`launch_twist_to_ackermann:=True` is required** — it is the only `cmd_vel → drive` bridge, and
   it defaults **False** because LUCIO's ego-MPC publishes `drive` directly. Without it Nav2 plans
   happily and the car never moves. It is **not** declared in `bringup.launch.py`; it reaches
   `vehicle.launch.py` by launch-config inheritance. That works — verified — but confirm the node
   exists rather than assuming: `ros2 node list | grep twist_to_ackermann`.
2. **`use_composition:=False` is NO LONGER NEEDED — bug-257 is fixed** (2026-08-27 evening,
   `aced708`). The abort was `--use_multi_threaded_executor` on `f1tenth_container`, not composition:
   under a multi-threaded executor rclcpp can call `is_ready()`/`take_data()`/`execute()` for an
   action **client** out of order across threads, and the client throws
   `std::runtime_error("Taking data from action client but no ready event")`, aborting the process
   (ros2/rclcpp#2242; the upstream fixes #2250/#2495 are not in Humble). `bt_navigator` is an action
   client of the servers in its own container, which is why only an *executing goal* triggered it.
   The flag is now the `container_multi_threaded` argument, default `False`. Composed runs then took
   three consecutive 70 s goals across two launches with no abort. Leave composition on.
   **The redirect on the launch line is not optional** — see §0★.7.
3. **WAIT for lifecycle ACTIVE before clicking.** A goal sent earlier is discarded with **no log
   line at all** (bug-126), and looks exactly like a goal sent after a container abort.
   ```bash
   for n in bt_navigator planner_server controller_server behavior_server velocity_smoother; do
     printf "%-20s " $n
     ros2 service call /$n/get_state lifecycle_msgs/srv/GetState | grep -o "label=.*"
   done
   ```
4. **Start the bag before the goal.** Include `/goal_pose` — without it you cannot tell afterwards
   whether the click was even published.
5. **Hold R1 (SDL button 10) for the WHOLE drive.** Releasing stops the car but does **not** stop
   Nav2; it keeps replanning. Never button 5 (PS = power off).
6. **Cancel the goal before relaxing — this is a safety step, not tidiness.** After stalling, Nav2
   kept the goal active and decided to **reverse at 0.5 m/s**; with R1 held that drives the car
   backwards unannounced. Humble has no `ros2 action cancel` CLI, so:
   ```bash
   ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal \
     "{goal_info: {goal_id: {uuid: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}, stamp: {sec: 0, nanosec: 0}}}"
   ```
   An all-zero UUID cancels every active goal. Confirm with `ros2 topic hz /cmd_vel` going silent.

**What to expect**, from `nav2_drive4`: ~10 s of commands, ~5.8 m of path, 203/203 commands reaching
the VESC, and a stop **0.379 m short** of the 0.25 m tolerance with heading well inside it. **No
goal has yet formally SUCCEEDED.**

- **One open lead remains**: the last command before it quits is **0.269 m/s** against a measured
  ground breakaway of **0.20–0.26 m/s** — RPP decelerates into the speed below which this car
  physically cannot move. Carry that number in **ERPM**, not m/s, if `speed_to_erpm_gain` is ever
  recalibrated.
- **The `movement_time_allowance` lead is CLOSED.** It is `10.0` (the nav2 default) as of `614c463`,
  and that was **verified on hardware 2026-08-27**, parked: `SimpleProgressChecker` failed
  `FollowPath` at 10.01–10.03 s and the recovery ladder ran `ClearCostmaps → Wait → BackUp`, which
  is the first time `BackUp` has been reached by the BT itself. Every stage script from `0827` on
  carries it. It had been `100.0` deliberately — it bought *operator* setup time, because a goal sent
  before R1 is held produces no motion and aborts in 10 s. **Hold R1 before sending the goal** and
  10.0 is plenty: bringup time is never counted (the checker resets at goal acceptance), and
  `required_movement_radius: 0.5` in 10 s is 0.05 m/s against 0.5 m/s commanded.

## 0★.7 · Reading a crashed run

- Composable-node output is **not** in `launch.log`. It is in
  `$ROS_LOG_DIR/<container>_<pid>_<t>.log` — top level, **not** the dated subdirectory.
- That file is **buffered**: on an abort it truncates at a 16384-byte boundary mid-word and the last
  ~3 s are lost. `RCUTILS_LOGGING_BUFFERED_STREAM=0` does **not** fix it (console stream only).
- **Redirect `ros2 launch`'s own STDOUT to a file instead** — `ros2 launch … > $ROS_LOG_DIR/launch_stdout.log 2>&1`.
  The launch system pipes every process's stdout through with a `[<process>-N]` prefix, so the C++
  runtime's dying words (`terminate called after throwing…`, `what(): …`) land there even though
  they never reach the rcutils log file or `launch.log`. Three sessions hunted bug-257 inside the
  truncated log; the stdout redirect produced the exact exception on the first try. Do it on every
  run — it costs nothing and it is the only place an abort explains itself.
- `launch.log` is drowned by the safety publisher: it is a `ros2 topic pub --rate 40` that prints
  every message — **10 MB in 5 minutes** (bug-259). Filter `[ros2-N]` before reading anything.
- Bags stopped with SIGINT have **no `metadata.yaml`**, so `ros2 bag info` fails. The `.db3` is
  complete — read it with sqlite3 + `rclpy.serialization.deserialize_message`.

## 0★.8 · Keeping the container alive, and the display — learned 2026-08-30

**Start the container under `tmux` on the HOST.** It runs with `--rm` and its main process is the
shell of the session that created it, so when that session drops the container **exits and is
auto-removed**. That cost a warm 3-hour stack on 2026-08-30; only the logs survived, because they
are on the SSD.

```bash
tmux new -s f1tenth        # on the Jetson desktop session (T1)
bash ~/bolus_ws/f1tenth_launch.sh
# Ctrl-b d to detach; `tmux attach -t f1tenth` to come back
```

`~/bolus_ws/f1tenth_launch_detached.sh` is a no-tmux fallback that detaches via a FIFO.

**Validate `$DISPLAY` BEFORE creating the container.** `jetson-containers` bakes `$DISPLAY` in at
creation and it **cannot be repointed afterwards** — a wrong value means recreating the container.

```bash
xdpyinfo -display $DISPLAY     # the ONLY valid check
```

A dead forwarded display still shows an sshd listener on its port, so `ss -ltn` proves **nothing**.

**The xauth key form matters.** `xauth extract` writes a key of the form `<host>/unix:N`, which
matches only a **Unix-socket** connection. A container connecting over **TCP** to `localhost:N.0`
needs the **FamilyWild** form, or you get `MoTTY X11 proxy: No authorisation provided`:

```bash
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge -
```

**The launch wrappers now refuse to start an unstaged container.** `/mnt/shared_dir/stagecheck.sh`
is sourced by `viz_launch.sh`, `mapcheck_launch.sh`, `throttle_launch.sh` and `launch_run.sh`; it
fails if the installed tree lacks `data/maps/20260805` or still defaults `map_file` to
`raslab.yaml`. **Read the map fingerprint off `/map`, never off a filename**: origin
**(−9.29, −6.06)** = current 20260805, **(−3.95, −9.87)** = retired raslab.

---

---

Terminals: **T1** = Jetson **desktop session** (physical/VNC, *not* ssh). **T2–T5** = any ssh shell.
`C` below = the container name from T1, e.g. `jetson_container_20260810_083904`.

---

## 0 · Before anything (T2) — 30 s

- `date` → if the year is **1969**, fix it now, or every bag gets a 56-year jump mid-run:
  - `sudo date -s "2026-08-10 HH:MM:SS"` # ssh -t gosling1 'sudo date -s "2026-08-10 09:36:00"'
- ~~`mount | grep " / "` → must say `rw`. If `ro`, **reboot**; the SD card is failing.~~ **STALE** — since the 2026-08-24 reflash the M.2 is root (~915 G). See §0★.1.
- ~~`ls /dev/sensors/vesc /dev/ttyUSB0 /dev/input/js0`~~ **STALE** — the LiDAR is the CP210x, not `ttyUSB0`. See §0★.1. (VESC still needs its **battery** on to enumerate.)

## 1 · Container — T1, desktop session only

- `xhost +local: && xhost +SI:localuser:root`
- `bash ./bolus_ws/f1tenth_launch.sh`
- **Must be T1.** Launching this over ssh kills the RealSense: librealsense loses its GL
  context and aborts `sensing_container` with `_glfwPlatformGetTls … Assertion failed`,
  taking the camera and VSLAM with it.
- Note the container name it prints → that is `C`.

## 2 · Prep the container (T2) — ~90 s, idempotent

- ~~`/mnt/f1tenth_ssd/shared_dir/rf2o_zv_0809/prep_container.sh`~~ **SUPERSEDED 2026-08-26 by `stage_0826c.sh`** — see §0★.2. The audit notes below are history.
- Must end `container … is ready.` Re-run after **every** container restart — `/workspaces` is a container layer, not a bind mount.
- It verifies steering gain `-1.1448`, wheelbase `0.256`, the `twist_to_ackermann` sign fix, and the rf2o zero-velocity gate.
- **Audited 2026-08-10 on container `…_015546` — the package tree is NOT the problem it was.** All 111
  launch/config/urdf/rviz/script files in `src/f1tenth_launch` were byte-identical (md5) to the repo,
  `install/` is symlinked into `src` and current, and `data/maps/20260805/` is present. The three
  previously-"unknown" robot-side fixes all resolve **PRESENT**: rf2o zero-velocity gate
  (`zv_enabled(true)`, 0.02/0.05/0.03, hold 3), the bug-140 twist fix
  (`math.atan(WHEELBASE / radius)`, `max_steering_angle` a real parameter, wheelbase 0.256), and
  `imu_processors` — **built from `src/imu_pipeline` on 2026-08-09, so `imu_bias_remover_node` now
  exists.** The only real gap was 10 `scripts/analysis` + `scripts/live_runs` tools that the tarball
  never carried (`CMakeLists.txt` installs `launch params config rviz urdf data`, **not** `scripts`);
  `odom_moving_check.py`, `heading_from_scan.py`, `analysis/check_map_frame.py` and
  `rtabmap_ground_truth.py` were staged to `/mnt/f1tenth_ssd/shared_dir/` and into the container src
  tree on 2026-08-10. **Re-stage them after any container reset.**
- **The IMU bias remover is one flag from running, and its documented blocker is stale.**
  `realsense_d435i.launch.py:366` hardcodes `'remove_imu_bias': 'False'` with a comment saying the
  node "is not installed yet" — it is, as of 2026-08-09. `imu_filter.launch.py` already defaults the
  arg `True`, so only that hardcoded string stops the chain. Not flipped yet: parked measurement
  shows the RealSense bias is not currently driving fused yaw (see §4), so this is an improvement to
  make deliberately, with a before/after, not mid-demo.

---

## 3 · RUN A — MPC

**Stack (T2)** — Nav2 and `twist_to_ackermann` **off**, so `/gosling1/drive` has exactly one publisher: your MPC.

- `docker exec -it $C bash`
- then inside:
  ```
  source /opt/ros/humble/setup.bash && source /workspaces/f1tenth/install/setup.bash
  export ROS_DOMAIN_ID=42 CONFIRM=yes RESET_REALSENSE=true
  #export DDS_PROFILE=lo          # kills the ddsi_udp_conn_write spam. Comment out for LUCIO runs
  export ROS_LOG_DIR=/mnt/shared_dir/claude_mpc_0810/roslogs
  /workspaces/f1tenth/src/f1tenth_launch/scripts/live_runs/71_mpc_stack.sh \
    --map /mnt/shared_dir/maps/20260805/rtabmap_2d_final.yaml --domain 42 -y
  ```
- Wait for **`stack healthy`** (~2 min). Leave this terminal running — Ctrl-C here tears the stack down.

**MPC node (T3)**

- `docker exec -it $C bash` → `export ROS_DOMAIN_ID=42` before launching your controller.
- It publishes `ackermann_msgs/AckermannDriveStamped` on `/gosling1/drive`.

**Pre-drive check (T4) — 20 s, do this BEFORE wondering why the car won't move**

- `ros2 topic info /gosling1/drive` → **Publisher count must be ≥ 1.** If it is 0, your MPC is not running. This is the single most common "VESC doesn't respond" cause; the driver, gate and mux will all look perfectly healthy.
- `ros2 topic echo /gosling1/ackermann_drive --field drive.speed` → the mux output. Zeros with nothing held is correct (`safety` channel).
- Hand-driving needs a **human deadman held** (4, 9 or 10) *and* stick movement — `teleop` is silent otherwise. A live `command_gate/heartbeat` does **not** mean teleop commands are flowing; they are different topics.

**Drive it (T4 + controller)**

- Optional heading check, parked: `python3 …/scripts/live_runs/yaw_drift.py 60`
- **Hold R1** (SDL button **10**) to hand over. Releasing R1 is the override.
- **Never button 5** — that is PS/power-off.
- Watch by **value**, never `ros2 topic hz` (a closed gate publishes zeros at full rate):
  - `ros2 topic echo /gosling1/vehicle/ackermann_cmd --field drive`

**Validate `odometry/local` while moving (T5) — never once measured**

Every odometry number this vehicle has on record was taken **parked**. A scale error in
`speed_to_erpm_gain`, the 2.4 % wheelbase yaw-rate bias in `vesc_odom`, and an rf2o whose
zero-velocity gate latches on under motion are all invisible parked and all corrupt the MPC's
state estimate. This costs one extra straight leg at the top of Run A.

- **Prerequisite** (once per container): the robot gets `f1tenth_launch` from a **tarball, not
  git**, so `odom_moving_check.py` will not be there. Stage it first:
  `scp scripts/live_runs/odom_moving_check.py scripts/analysis/check_map_frame.py gosling1:/mnt/f1tenth_ssd/shared_dir/`
  (it imports `check_map_frame` from `../analysis`, so keep that relative layout or run it from the repo tree).
- **Mark the floor before moving.** Tape at each **rear-wheel contact patch**, both wheels. Repeat
  at the end of the leg. Distance between the two midpoints = net displacement; angle between the
  two wheel-pair lines = heading change. Do **not** index off a tire sidewall — a pointer on a
  curved surface pivots freely (1.9–16.4° error, measured).
- Start the bag before the drive, from a shell with `00_env.sh` sourced:
  ```
  source …/scripts/live_runs/00_env.sh && source …/scripts/live_runs/topic_sets.sh
  set_array mpc
  bag_record mpc_moving "${TOPIC_LIST[@]}"
  ```
- **First leg: straight, ~4 m, then stop.** Straight is what makes path length and net displacement
  agree, so a scale error cannot hide inside a curve. Then run the MPC trajectory as normal — same
  bag, no need to restart it.
- Ctrl-C the bag, then:
  ```
  python3 …/scripts/live_runs/odom_moving_check.py "$(cat "$BAG_ROOT/.last_bag")" \
    --tape 4.00 --tape-yaw 0
  ```
- Read it in this order: **health** first (a frozen or rate-starved source invalidates everything
  above it), then **scale vs the tape**, then **heading**. Without `--tape` the report can only show
  the sources agreeing with each other, and a wrong `speed_to_erpm_gain` moves `vesc_odom` and the
  EKF together — agreement is not accuracy.
- **There is no acceptance band yet.** This run sets the baseline; the only hard failures the script
  asserts are rate/gap/frozen. Copy the table into `SYSID_RESULTS.md` with the tape values.

**Then: Ctrl-C the stack in T2.** MPC and Nav2 cannot share one launch.

---

## 4 · RUN B — Nav2 with obstacles

**Stack (T2)** — same container, `twist_to_ackermann` **on** (it is the only `cmd_vel → drive` bridge).

- ```
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp                             # REQUIRED — see below
  export CYCLONEDDS_URI=file:///mnt/shared_dir/cyclonedds_offline_lo.xml   # no DDS spam
  export ROS_LOG_DIR=/mnt/shared_dir/claude_mpc_0810/roslogs_nav2
  ros2 launch f1tenth_launch bringup.launch.py \
    launch_visualization:=True \
    launch_twist_to_ackermann:=True \
    max_steering:=0.314 \
    localize_isaac_vslam_on_startup:=False \
    map_file:=/mnt/shared_dir/maps/20260805/rtabmap_2d_final.yaml
  ```

- **STALE as of 2026-08-25 — `RMW_IMPLEMENTATION` IS now set in the container environment; verify with `echo $RMW_IMPLEMENTATION` rather than trusting either version of this note.** Original text: **`RMW_IMPLEMENTATION` is not optional here** (2026-08-10, bug-233). The image exports
  `CYCLONEDDS_URI` but **not** `RMW_IMPLEMENTATION`, and only `00_env.sh:30` sets the latter. So a raw
  `ros2 launch` runs **FastRTPS** while every `live_runs` script runs **CycloneDDS** — the
  `CYCLONEDDS_URI` line above is silently inert without the RMW export, including the `lo` loopback
  setting that fixed the VSLAM jitter. Worse, a diagnosing shell on the other DDS sees an empty
  system. Export it in *every* shell that talks to this stack.

- **RESOLVED — the guard is engaged by default since 2026-08-11 and was confirmed live 2026-08-27 (`ekf_map_node odom1` reads `...__NOT_FUSED`). bug-231 is CLOSED: parked `odom→base_link` measured (0.003, 0.005, −0.13°).** Original text: **`localize_isaac_vslam_on_startup:=False` is what stops the map looking rotated** (2026-08-10,
  bug-232). `ekf_map` fuses `odom1 = visual_slam/vis/slam_odometry` as an **absolute map-frame
  anchor**, which is only valid if VSLAM started localized into a saved, co-registered map. A guard
  exists (`ekf_map.launch.py:141` renames the topic to `…__NOT_FUSED`) and
  `localization.launch.py:523` computes it as `use_gpu AND localize_on_startup` — but
  `bringup.launch.py` defaulted `localize_isaac_vslam_on_startup` to **True** (**fixed 2026-08-11 —
  it and `teleop.launch.py` now default `False`, matching the three other entry points and their
  own argument descriptions; the `:=False` above is now redundant but kept as documentation of
  intent**), so with `use_gpu`
  also True the guard evaluated True while the VSLAM node itself ran with `localize_on_startup:
  False`, fresh from its own power-up origin. Measured: `amcl_pose` yaw **−79.80°**,
  `odometry/global` **−7.50°**, VSLAM **−7.45° at 29.9 Hz**, and `/amcl_pose` **0 messages in 180 s**
  (AMCL is motion-gated; parked, it never republishes). `ekf_map` tracked VSLAM to 0.05° and AMCL
  contributed nothing. With `map_tf_publisher='ekf'`, `map→odom` was pinned to VSLAM's origin — a
  **72.3°** error that rotates the car, the footprint, the live scan and the 3D cloud inside a
  perfectly good map. Confirm the guard took, in one command:
  ```
  ros2 param get /ekf_map_node odom1     # must read  visual_slam/vis/slam_odometry__NOT_FUSED
  ```

- **SUPERSEDED 2026-08-27 — seeding is now AUTOMATIC and verified on hardware (bug-241, 6/6 cold launches). Do not seed by hand; see §0★.4.** Original text: **Seeding the pose is now optional insurance, not a required step.** With the guard engaged,
  `pose0 = amcl_pose` is the only global input and AMCL's own `set_initial_pose: True` (from
  `localizer_amcl.yaml`) propagates into `ekf_map` unaided — verified 2026-08-10 with **no seed at
  all**: `map→base_link` came up at `(0.463, -0.602, -79.31°)` and AMCL-vs-EKF agreement was 0.56°.
  The old "this step is required and there is no substitute" (bug-230) was a **symptom of bug-232**:
  the manual seed only appeared to be the cure because `set_pose` *resets* the filter, briefly
  beating the 30 Hz VSLAM pull. AMCL publishes that initial pose essentially once, so a startup race
  where `ekf_map` is not yet subscribed is plausible though not observed. Run the seed anyway if
  `map→base_link` is not at the spot; it is idempotent and costs a second:
  ```
  cd /workspaces/f1tenth/src/f1tenth_launch/scripts/live_runs
  python3 seed_initialpose.py --ns "" --no-use-sim-time --x 0.445 --y -0.575 --yaw -1.4748
  ```
  Note the **`src`** path: `CMakeLists.txt` installs `launch params config rviz urdf data` and
  **not** `scripts`, so there is no copy of this under `install/…/share`. The `--ns ""` and
  `--no-use-sim-time` flags exist for this live case — the script's defaults are for bag replay
  (namespaced TF, `/clock`), and on a raw un-namespaced live launch the default would remap `/tf` to
  `//tf` and wait out its timeout against a healthy tree.
- **Why RUN B is a raw `ros2 launch` and RUN A is a script — keep it that way.** The two paths differ
  in more than style and the difference is not cosmetic:

  | | `live_runs` scripts (RUN A) | raw `ros2 launch` (RUN B) |
  |---|---|---|
  | namespace | **`/gosling1/…`** (`use_f1tenth_namespace` on) | **bare `/…`** |
  | RMW | CycloneDDS, set by `00_env.sh:30` | FastRTPS unless you export it — **do export it** |
  | DDS profile | handled by `DDS_PROFILE` | you set `CYCLONEDDS_URI` yourself |
  | health gate | built in, waits for `stack healthy` | none — you check by hand |

  **`config/f1tenth.rviz` is un-namespaced** (its Nav2 displays had the `/gosling1/` prefix stripped
  precisely because `use_f1tenth_namespace` defaults False). So a namespaced script launch shows an
  RViz full of silent displays, which is why RUN B — the one with the visuals and the goals — uses the
  raw launch. The scripts' advantages (env hygiene, the health gate) are recovered by exporting
  `RMW_IMPLEMENTATION` and `CYCLONEDDS_URI` as shown above and running the §4 gate checks.
  `ros2 topic list | grep -c gosling1` tells you which world you are in, in one second, and it decides
  whether every `tf2_echo` and `topic echo` in this runbook needs a `/gosling1` prefix.

- **`launch_pointcloud_map` and VSLAM both stay ON.** An earlier revision of this runbook told you to
  disable them. That was wrong and has been removed: the cloud is the deliverable, and VSLAM is a
  wanted localization input. If `pcd_to_pointcloud` dies with
  `[pcl::PCDReader::read] Could not find file …` it is **aborting on a missing file**
  (`std::runtime_error`, not a warning) because the container's *installed* share tree is older than
  your repo — fix the staleness, see §4a, do not disable the node.
- Dry run first if you want Nav2 to plan without moving the car:
  `…/scripts/live_runs/60_nav2_test.sh --map <map.yaml> --dry-run`

**4a · A fresh container reverts the package — check before blaming the config**

`/workspaces` is a container layer, not a bind mount, so every new container starts from whatever the
image shipped. On 2026-08-10 that was older than the repo in three places, which presented as a dead
`pcd_to_pointcloud` and an RViz with an empty Displays panel:

```
docker exec $C ls -lL /workspaces/f1tenth/install/f1tenth_launch/share/f1tenth_launch/config/f1tenth.rviz
docker exec $C ls -lL /workspaces/f1tenth/install/f1tenth_launch/share/f1tenth_launch/data/maps/20260805/
docker exec $C grep -n "pointcloud_map_file = LaunchConfiguration" \
    /workspaces/f1tenth/src/f1tenth_launch/launch/bringup.launch.py
```

Expect `f1tenth.rviz` present (~13 kB, 478 lines), `data/maps/20260805/` holding
`cloud_voxel_0p05.pcd`, and the pcd default pointing at `20260805`, not `rtabmap/raslab`. To repair:
untar the staged files over `…/src/f1tenth_launch`, then
`colcon build --symlink-install --packages-select f1tenth_launch`. **The rebuild is not optional** —
`--symlink-install` links files that existed at build time, so files *added* since are simply absent
from `install/` until you rebuild.

**RViz on your workstation (T5)** — the Jetson has no monitor.

- Start VcXsrv/X410 on the workstation first, then:
- `docker exec -e DISPLAY=192.168.2.110:0 -e XAUTHORITY= -e LIBGL_ALWAYS_INDIRECT=0 $C bash -lc "source /opt/ros/humble/setup.bash; source /workspaces/f1tenth/install/setup.bash; export ROS_DOMAIN_ID=42; rviz2 -d /workspaces/f1tenth/install/f1tenth_launch/share/f1tenth_launch/config/f1tenth.rviz"`
- Fixed frame is **`map`**. Set goals with **2D Goal Pose**.
- Obstacles come from the **LiDAR** local costmap — the camera is not required for them.
- **`launch_visualization:=True` already starts one RViz.** Do not also start the one above — two
  remote-X RViz sessions on the Orin is real load. Pick one.
- **An RViz that comes up with nothing in it is a stale install, not a display problem.** Check
  before re-launching:
  ```
  docker exec $C ls -l /workspaces/f1tenth/install/f1tenth_launch/share/f1tenth_launch/config/f1tenth.rviz
  docker exec $C grep -c 'Class:'  …/config/f1tenth.rviz   # empty Displays panel  -> file missing/old
  docker exec $C grep -c 'gosling1' …/config/f1tenth.rviz   # displays listed but silent -> namespaced config
  ```
  Override with `rviz_config_file:=` a known-good copy on `/mnt/shared_dir` rather than re-staging
  mid-demo.

**Before trusting a goal**

- Nav2 servers must be lifecycle **ACTIVE**, not merely present — they reject goals while inactive and log nothing.
- Parked at the origin, AMCL correctly reads ≈ `(0.45, -0.57, -80°)`. That is the map frame, **not** drift.
- **Run this gate first, and check the ABSOLUTE transforms, not their drift** (bug-231, bug-232):
  ```
  ros2 param get /ekf_map_node odom1          # must be  ...slam_odometry__NOT_FUSED   (bug-232)
  ros2 run tf2_ros tf2_echo odom base_link    # PARKED: must be ~0. See below.
  ros2 run tf2_ros tf2_echo map base_link     # must be ~ (0.445, -0.575, -84.5 deg)
  ros2 topic hz /lidar/scan_filtered          # ~10-12 Hz, no multi-second gaps
  ros2 lifecycle get /planner_server          # must be "active"
  ```
- **A rotated car / scan / cloud inside a good map has THREE possible causes** (the third was
  added 2026-08-30, bug-265). **First ask what moved**: if the *car, footprint and live scan* are
  rotated inside a correct-looking map it is one of the two below; if the *two static maps* are
  rotated relative to each other with the car sitting correctly, it is the map artifacts themselves
  — see the bug-265 bullet further down. Then separate the first two with one comparison, because
  they look identical in RViz and have nothing to do with each other:
  ```
  ros2 topic echo /amcl_pose      --field pose.pose.orientation --once
  ros2 topic echo /odometry/global --field pose.pose.orientation --once
  ```
  * **They disagree (tens of degrees) → bug-232**, the global EKF is tracking un-relocalized VSLAM.
    Fix is the `localize_isaac_vslam_on_startup:=False` launch arg in §4. Was 72.3° apart; is now
    0.56°.
  * **They agree, but `odom→base_link` is large on a parked car → bug-231**, genuine odometry drift.
- **`odom→base_link` on a car that has not moved is the single most useful number here, and it must
  be read as an absolute, not as drift.** On 2026-08-10 it was measured at `(0.338, -1.200)` with yaw
  **+62°**, having grown from +4.6° over ~20 min (≈2.9 °/min, vs +0.01…+0.17 °/min on 2026-08-06).
  **That did not reproduce on a cold launch later the same day**: parked 180 s gave `odometry/local`
  −0.05 °/min, and absolute `odom→base_link` held −0.22° / 18 mm over 5.5 min. Per-source, parked:
  VESC wheel odom 0.00, Isaac VSLAM −0.05, rf2o −0.34 °/min; z-gyro bias VESC **+0.004651 rad/s**
  (matching its −15.76 °/min orientation walk, which is why `imu0` yaw is disabled) and RealSense
  **−0.002208 rad/s** (matching `docs/imu_bias_removal_spec.md`). So the RealSense bias is real and
  uncorrected, but rf2o and VSLAM hold fused yaw down and it drives nothing while they are healthy.
  Treat the 2.9 °/min as an **unexplained intermittent** — measure, don't assume.
  A watcher that reports *displacement since its first sample* will call all of this healthy: over
  90 s the drift was 10 mm and −0.14°. Read the absolute value.
- ~~**The 2D grid and the 3D cloud will never look perfectly aligned, and that is expected.**~~
  **WRONG, and it cost three sessions — RETRACTED 2026-08-30 (bug-265).** The cloud really was yawed
  **+25.33°** against the grid, the operator reported it repeatedly, and this bullet is what talked
  three sessions out of believing them. **It is fixed**: `data/maps/20260805/cloud_voxel_0p05.pcd`
  was regenerated in the grid's frame (`a51bdc6`, shipped by `stage_0831.sh`) and the operator
  verified it in RViz.
  * **The cause, which matters for every future map build:** `rtabmap-export` re-roots the
    optimized pose graph at the **first** node, so its clouds are in the frame the run *started* in;
    the `rtabmap` ROS node publishes `grid_prob_map` anchored so the **latest** pose agrees with
    odometry, and `map_saver` captures the `.pgm` in *that* frame — which is the frame AMCL
    localizes in. The offset is exactly the yaw the optimizer removed over the run. Full detail and
    the export recipe: `MAP_BUILD_HANDOFF.md`.
  * **Every argument in the old bullet was an artefact of one bad metric.** The "0.05 m at zero
    shift" figure came from `map_cloud_align.py`'s **one-way** score (grid cell → nearest cloud
    point). The cloud is far denser than the grid and does not cover its footprint, so *every* pose
    scores well and the metric cannot tell alignment from density. That is also why two sessions
    "disagreed in sign" — they were reading noise. The script is **deleted** (`a8e027f`); use
    `map_cloud_align2.py`, judge by **where the peak sits**, not its height, and require **both**
    coverage columns to rise.
  * **The partial-coverage point was true but irrelevant.** The cloud is RGB-D and does miss the
    western third of the room, which is why absolute F1 stays ~0.42 even when perfectly aligned
    (LiDAR-vs-LiDAR scores 0.67 on the same pair). Low overlap is expected; a peak away from 0° is
    not.
  * **Wall-orientation histograms DO work here**, contrary to the old claim that the lab is a ring:
    grid 87.0°, corrected cloud 86.5°, pre-fix cloud 61.0°. That measure never compares one map to
    the other, so density cannot bias it — it is the cheapest independent check available.
  * **If the cloud and grid look misaligned in RViz, believe the picture** and check the two static
    files against each other before suspecting localization. Set the camera to the saved
    `Top Down (old default)` view first: an orbit view rotates the reference grid and the map
    together, which produced a *separate* false alarm on 2026-08-30.
- **`Robot is out of bounds of the costmap!` is a report, not the fault.** Chase `odom→base_link`
  first, then the global correction. Do not touch the costmap config.
- **Repeated `amcl: Message Filter dropping message … earlier than all the data in the transform
  cache`** is normal for the first second or two. Continuing for tens of seconds means the TF buffer
  is being outrun by a CPU stall — read it together with `ekf_node: Failed to meet update rate!`.
  Both EKFs missing by the same ~2 s at the same instant is a machine-wide stall, not a filter bug.

---

## 5 · Gotchas worth 10 seconds each

- **Right turns clip, left does not.** No-clip range is **+24° left / −18° right**; `max_steering:=0.314` is the symmetric setting.
- **`ddsi_udp_conn_write … retcode -3` spam** is a powered-off static DDS peer (`192.168.2.194`). Harmless but drowns real errors (12 MB in 2 min).
  Kill it with `DDS_PROFILE=lo` (or `CYCLONEDDS_URI=…cyclonedds_offline_lo.xml` for a raw `ros2 launch`).
  **`DDS_PROFILE` must be set explicitly** — `00_env.sh` defaults it to `none` whenever `CYCLONEDDS_URI` is already exported, which the container always does, so "unset" means "keep the noisy static profile".
  Safe here: RViz runs *inside* the container with only X11 forwarded, and the MPC container shares host networking. You only lose ROS tools run *natively* on the workstation, and a second robot.
- **`map_tf_publisher` defaults to `'ekf'` in `bringup.launch.py:91` and is passed down at line 904,
  overriding `localization.launch.py`'s own `'amcl'` default.** So through bringup the *global EKF*
  owns `map→odom` and AMCL runs with `tf_broadcast: False` — CLAUDE.md's claim that a running AMCL is
  the default broadcaster is true only for standalone `localization.launch.py`. This is a working
  arrangement (it can coast on VSLAM through AMCL dropouts) **provided `odom1` is not fusing
  un-relocalized VSLAM as an absolute anchor** — see bug-232 in §4, which is what actually made the
  unseeded case look broken. `map_tf_publisher:=amcl` is the quick alternative, but it gives up the
  coasting.
- **A stale `ros2 daemon` will show you an empty system** (bug-233). `ros2 topic list` / `node list`
  answer from a background daemon, and one left over from an earlier session — different domain, or
  different RMW — reports 2 topics while 137 are live in that same container. Cost ~15 min on
  2026-08-10. First command in any new diagnosing shell:
  `ros2 daemon stop` (it restarts itself with your env).
- **Check which namespace you are actually in before checking TF.** `use_f1tenth_namespace` defaults
  **False** (`bringup.launch.py:55`), so it depends on *how you launched*:
  - Via `71_mpc_stack.sh` / the other `live_runs` scripts → namespaced. TF is `/gosling1/tf`, and a
    bare `tf2_echo map odom` reports MISSING on a perfectly healthy tree. Add
    `--ros-args -r /tf:=/gosling1/tf -r /tf_static:=/gosling1/tf_static`.
  - Via a raw `ros2 launch f1tenth_launch bringup.launch.py …` as in §4 → **not** namespaced. Topics
    are bare `/tf`, `/scan`, `/odometry/local`, and plain `tf2_echo map base_link` is correct.
  - `ros2 topic list | grep -c gosling1` settles it in one second. This also decides whether an RViz
    config saved from a namespaced run will show anything.
- **Nothing should reseed `/gosling1/initialpose` from figure-8 waypoint 0.** Waypoint 0's yaw is −92.08°, and "seeded from waypoint 0, verified 19 mm from waypoint 0" is circular; it cannot detect a wrong pose. **Settled 2026-08-10 and confirmed 2026-08-11: waypoint 0 is the worst candidate of the three, scoring 62–64 % against the map's −84.5°, i.e. 7.6–7.8° off — see §5b.**
- **Never `kill -9` a `ros2 launch`** — it orphans every node (they keep publishing and look like duplicates). Ctrl-C and wait.
- **Never `pkill -f <pattern>` from a `bash -c`** whose own command line contains that pattern — it kills itself first and the rest of the list silently survives.
- **Logs go to the SSD** (`/mnt/shared_dir/…`). ROS logs on the 28 GB SD card filled it to 100 % once.
- **`docker exec` dead, `read-only file system`?** The card has gone ro. Reboot; don't fight it.
- **Use `tmux` on the HOST for the container itself, and optionally inside it for the stack.**
  The "host has no tmux" half of this is **STALE** — tmux was installed on gosling1 on
  2026-08-30, and the host session is the one that matters. See §0★.8.

## 5b · The heading question — SETTLED 2026-08-10, CONFIRMED AND APPLIED 2026-08-11

Three values claimed the same parking spot: **−79.80°** (config `initial_pose`), **−86.5°** (AMCL
settle 2026-08-06), **−92.08°** (figure-8 waypoint 0). All three are estimator outputs, so they could
not settle each other. `heading_from_scan.py` scores the raw scan against the occupancy grid —
geometry, no localizer — and it did:

```
BEST  x=+0.385  y=-0.515  yaw=-84.50 deg   score=0.8566   (2432 endpoints, 20 scans)
  AMCL settle 2026-08-06      -86.50 deg   96.2% of best,  2.00 deg away
  config initial_pose         -79.80 deg   81.0% of best,  4.70 deg away
  figure-8 waypoint 0         -92.08 deg   63.7% of best,  7.58 deg away
```

- **Single clear peak — NOT ambiguous.** The room-symmetry hypothesis is dead; the recurring ~90°
  confusion was not the map being symmetric to a planar LiDAR.
- **The true heading is ≈ −84.5°**, so the config's −79.80° is **4.70° off**. That matches the
  operator's independent RViz eyeball ("maybe 5 degrees to the left") on the same run.
- **Figure-8 waypoint 0 (−92.08°) does not fit the map** at 63.7 %. Anything seeding itself from
  waypoint 0 — including the MPC — starts ~7.6° wrong. Its self-check ("seeded from waypoint 0,
  verified 19 mm from waypoint 0") is circular and cannot detect this.

### Confirmed 2026-08-11 and applied

The 2026-08-10 result was one 20-scan sample from one spot, so it was repeated on a **fresh cold
launch** the next day — three recordings, with the car rolled off the spot and back on between them:

| sample | x | y | yaw | score |
|---|---|---|---|---|
| 2026-08-10 | +0.385 | −0.515 | **−84.50°** | 0.8566 |
| 2026-08-11 A (as parked) | +0.415 | −0.515 | **−85.00°** | 0.9162 |
| 2026-08-11 B (rolled off, re-parked on the marks) | +0.415 | −0.515 | **−85.00°** | 0.9128 |
| 2026-08-11 C (rolled off, re-parked deliberately off-mark) | +0.385 | −0.515 | **−84.25°** | 0.8933 |
| 2026-08-11 AMCL settle (different algorithm) | +0.516 | −0.472 | **−83.97°** | — |

**Total spread 1.03°; the four scan fits span 0.75°, mean −84.69°.** Applied: `localizer_amcl.yaml`
`initial_pose` yaw **−1.3928 → −1.4748** (−84.50°). The mean would be −1.4781; the difference is
0.19° against a 0.25° search step, so the pre-registered −84.50° was kept rather than refitted to
the newest data.

Three things worth keeping from the repeat:

- **A and B landed on the identical grid cell** (0.03 m / 0.25° steps) from different data — so the
  method's own noise is below its resolution, and the tile-line-plus-sticker parking marks are
  repeatable to ~1.5 cm. Confirm the car actually moved before believing this: `odom→base_link`
  went from `(−0.025, 0.002, −0.30°)` to `(0.318, 0.225, −5.40°)` across the out-and-back, which is
  what proves B is an independent sample rather than a re-measurement of an untouched car.
- **AMCL was seeded at −79.80° and settled at −83.97°** — it walked 4.17° *toward* the geometry, by
  a different algorithm. That is corroboration, not a restatement of the seed.
- **The `seed_initialpose.py` "matching default" does not exist.** That script defaults to
  `--x -0.008 --y -0.004 --yaw -0.951`, which are bag-replay values, not the parking spot. The
  second home of this constant is the seed command line in §4, not a script default.

**x/y was deliberately NOT changed.** Every fit prefers `y = −0.515` (identical in all four) and
`x = 0.385–0.415`, ~6 cm from the configured (0.445, −0.575). Position sits in a far shallower
basin than yaw at this resolution, 6 cm is well inside AMCL's convergence, and the offline replay
seeds share those numbers. Measure it deliberately before moving it.

**Still open — the offline seeds were left alone.** `51_localize_offline.sh`, `check_map_frame.py`,
`MAP_BUILD_HANDOFF.md`, `BRIEF_PARTICLE_FILTER.md` and `LOCALIZER_FOLLOWUPS.md` all carry
`(+0.445, −0.575, −79.82°)` as the start pose of the archived 2026-08-05 bags, taken from the
RTABMap database's optimized poses — a *different* measurement that seeds replays of those specific
bags. Either those bags really started 4.7° from where the car parks today, or the RTABMap ground
truth carries the same error. It is decidable offline in one command — run `heading_from_scan.py`
against the first few seconds of an archived bag — and until someone does, changing those files
would desynchronize each replay from its own data.

**Tell the LUCIO side.** They consume this frame, and figure-8 waypoint 0 (−92.08°, 62–64 % of best)
is 7.6–7.8° from the map across every sample — anything seeding from waypoint 0 starts that wrong.

**To repeat it** (~40 s, parked, stack up — note the topic is bare on a raw §4 launch, `/gosling1/…`
via the `live_runs` scripts):
```
ros2 bag record -o heading_check /lidar/scan_filtered      # ~15-25 s is plenty
python3 scripts/live_runs/heading_from_scan.py --bag heading_check \
  --map /mnt/shared_dir/maps/20260805/rtabmap_2d_final.yaml --xy-range 0.15
```
Pass the **bag directory**, not its parent — a wrong path fails with a bare
`RuntimeError: No storage could be initialized from the inputs`.

## 6 · Shutdown

- Ctrl-C the stack terminal, **once**, and wait.
- **If Ctrl-C never finishes** (measured 2026-08-10): `use_respawn:=True` resurrects `vesc_driver_node`, shutdown SIGINTs it ~1.4 s later, it aborts with
  `RCLError: failed to publish message: publisher pointer is invalid`, and respawn restarts it — an endless loop. 15 restarts in 2 min. Pressing Ctrl-C again does not help.
  Fix: **kill the launch parent first** (that stops respawn), *then* the leftovers:
  ```
  docker exec $C bash -c 'kill -9 $(pgrep -f "bin/ros2 launch")'
  docker exec $C bash -c 'pgrep -af "[v]esc_driver_node|[j]oy_node|[c]omponent_container"'   # kill -9 by PID
  ```
- Verify nothing orphaned: `docker exec $C bash -c 'pgrep -af "[c]omponent_container|[y]dlidar|[v]esc_driver|[j]oy_node" || echo CLEAN'`
- **Run one container, not two.** A second `f1tenth_launch.sh` gives you a second `jetson_container_*`; on 2026-08-10 its launch had no children at all and only added confusion.
- **Keep the container warm** between Run A and Run B — bring-up is ~8.5 min.
