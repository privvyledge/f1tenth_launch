# Demo runbook — MPC, then Nav2 with obstacles

**gosling1 · image `humble-devel-08092026` · `ROS_DOMAIN_ID=42` · map `20260805/rtabmap_2d_final.yaml`**

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
docker exec $C bash -lc "bash /mnt/shared_dir/stage_0826c.sh"    # ~1 min, idempotent
```

- **`stage_0826c.sh` supersedes `prep_container.sh` in §2 below.** It verifies eight values in the
  *installed* tree and rebuilds the `imu_processors` fork. Add `FLIP=1` only for a bias-remover
  wiring test; omit it for the committed `remove_imu_bias:='False'`.
- **`/mnt/shared_dir` is the IN-CONTAINER path. On the host it is `/mnt/f1tenth_ssd/shared_dir/`.**
  There is no `/mnt/shared_dir` on the Jetson itself.
- `package 'f1tenth_launch' not found, searching: ['/opt/ros/humble']` is an **unsourced overlay**,
  not a failed build (bug-254). The searched-paths list is the diagnostic.

## 0★.3 · The launch environment — every shell, including diagnosing shells

```bash
source /workspaces/f1tenth/install/setup.bash
export ROS_DOMAIN_ID=42
export CYCLONEDDS_URI=file:///mnt/shared_dir/cyclonedds_offline_lo.xml
export ROS_LOG_DIR=/mnt/shared_dir/<run-dir>/log_<name>       # never the Jetson home
```

- **A shell on the wrong domain or URI sees an empty graph and reads as a dead stack.** This is the
  same class of error as suppressing stderr on a check whose negative result you will act on.
- **`RMW_IMPLEMENTATION` is now set in the container environment** — the §4 warning below that it is
  not, and that a raw `ros2 launch` silently runs FastRTPS, is **STALE as of 2026-08-25**. Verify in
  one command rather than trusting either statement: `echo $RMW_IMPLEMENTATION`.
- **Why `_lo` and not the container default**, measured 2026-08-27: both configs already carry `lo`
  at priority 10, so the loopback fix behind the VSLAM-jitter result is in **both**. The static
  config additionally binds `wlP1p1s0` and lists four remote robots, **all of which pinged ABSENT**
  that day, costing ~200 failed `sendto`/sec per process across 41 nodes. RViz is unaffected — it
  runs same-host and reaches you over SSH X11, not DDS.

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
  launch_twist_to_ackermann:=True log_level:=info use_composition:=False
```

1. **`launch_twist_to_ackermann:=True` is required** — it is the only `cmd_vel → drive` bridge, and
   it defaults **False** because LUCIO's ego-MPC publishes `drive` directly. Without it Nav2 plans
   happily and the car never moves. It is **not** declared in `bringup.launch.py`; it reaches
   `vehicle.launch.py` by launch-config inheritance. That works — verified — but confirm the node
   exists rather than assuming: `ros2 node list | grep twist_to_ackermann`.
2. **`use_composition:=False` is a bug-257 workaround**, not a normal setting. Composed, the Nav2
   container aborts with a silent SIGABRT *while executing a goal* (1.9 s and 7.2 s in, on two
   runs). Non-composed it survived two full runs. n=2 — a lead, not a fix. Non-composed also gives
   each server its own process, so an abort names itself, and costs zero-copy IPC.
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
goal has yet formally SUCCEEDED.** Two open leads: the last command before it quits is 0.269 m/s
against a measured ground breakaway of 0.20–0.26 m/s; and `movement_time_allowance: 100.0` (default
10.0) makes Nav2 idle for 100 s rather than failing and recovering.

## 0★.7 · Reading a crashed run

- Composable-node output is **not** in `launch.log`. It is in
  `$ROS_LOG_DIR/<container>_<pid>_<t>.log` — top level, **not** the dated subdirectory.
- That file is **buffered**: on an abort it truncates at a 16384-byte boundary mid-word and the last
  ~3 s are lost. `RCUTILS_LOGGING_BUFFERED_STREAM=0` does **not** fix it (console stream only).
  This is why `use_composition:=False` is the diagnostic that actually works.
- `launch.log` is drowned by the safety publisher: it is a `ros2 topic pub --rate 40` that prints
  every message — **10 MB in 5 minutes** (bug-259). Filter `[ros2-N]` before reading anything.
- Bags stopped with SIGINT have **no `metadata.yaml`**, so `ros2 bag info` fails. The `.db3` is
  complete — read it with sqlite3 + `rclpy.serialization.deserialize_message`.

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
  export DDS_PROFILE=lo          # kills the ddsi_udp_conn_write spam
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
- **A rotated car / scan / cloud inside a good map has TWO possible causes. Separate them with one
  comparison**, because they look identical in RViz and have nothing to do with each other:
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
- **The 2D grid and the 3D cloud will never look perfectly aligned, and that is expected.** The grid
  is LiDAR-built (360°, 10 m); the cloud is RGB-D (camera-facing, short range). The cloud spans
  x ∈ [−5.5, 3.8] while the grid spans x ∈ [−9.3, 4.0] — **the cloud is simply missing the western
  third of the room**, and a partial cloud inside a full grid reads as "rotated" even when every
  point is correct. Two independent correlation attempts on the same two files disagree in sign on
  the "best" angle (−27°/−30.75° one session, +24° the next, both broad and shallow), which is the
  signature of a metric scoring coverage rather than geometry — consistent with the flat 1.11×
  wall-orientation contrast. Median occupied-grid-cell to cloud distance is **0.05 m, one cell, at
  zero shift**. **Do not shim the cloud.** If you want this settled properly, the test is a *scan*
  against each map separately (`heading_from_scan.py`), not cloud-against-grid correlation.
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
- **Prefer `tmux` inside the container** for the stack (`tmux new -s f1t`), not on the host — the host has no tmux, and the container's session survives `docker exec` dying.

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
