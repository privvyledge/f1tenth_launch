# On-Robot Test Session — 2026-07-23

Copy-paste run sheet for the open items in `testing_checklist.md` (§5 loop closure, §6 particle cloud, §8 map save + RTABMap online, §9 nav goal + CPU, §10 integration), plus a sanity check of the newly-activated laser filter chain (`9b76dd9`).

Record results inline after each `➡` line, then transfer to `testing_checklist.md`.

## RUN LOG — 2026-08-04 (agent-driven, gosling1)

Conditions that qualify every result below:

- **VESC unpowered** (motor not on battery): `/dev/sensors/vesc` absent, so `vehicle/vesc_odom`
  and `vehicle/sensors/imu/raw` had NO DATA. Any test depending on them is marked invalid.
- **Robot stationary throughout.** A5 / A6-convergence / C2 were done by bag replay or deferred.
- **Live sessions A–D ran WITHOUT a namespace** (bringup default `use_f1tenth_namespace:=False`),
  so topics were `/lidar/scan`, `/odometry/local`, … NOT `/gosling1/…`. The bag-replay run
  (A5 + valid A3) DID use `use_f1tenth_namespace:=True`. **Re-run A1–A7/B/C/D namespaced** —
  namespacing is exactly where BUG-006/007 bite.
- Live sessions used **`use_composition:=False`** to work around BUG-006; the default
  (`use_composition:=True`) leaves the camera dead.
- Container on `ROS_DOMAIN_ID=77`; domain 0 = lab, 42 = concurrent agent.
- `ros2 node list` is unreliable under the static-peer CycloneDDS config (intermittently empty
  while nodes are healthy) — all node counts below were confirmed against `ps`.

## Prerequisites (once)

```bash
# Every terminal:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && source install/setup.bash
# Namespace below assumes $USER/gosling1. Substitute if different.

# Rebuild so the newly-active laser_filter.yaml and any repos updates take effect:
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to f1tenth_launch

# Confirm the FORKED rf2o is what's built (should show privvyledge remote):
git -C src/rf2o_laser_odometry remote -v

# For §6 (RViz machine, not the Jetson if RViz runs elsewhere):
sudo apt install ros-humble-nav2-rviz-plugins
```

---

## Session A — Teleop bringup: filter sanity, rf2o/EKF retest, loop closure, AMCL, §10 teleop mode

```bash
ros2 launch f1tenth_launch bringup.launch.py slam:=False launch_navigation:=False launch_global_localization:=True launch_visualization:=True
```

### A1. Laser filter chain sanity (NEW — speckle filter activated in 9b76dd9)

```bash
ros2 topic hz /gosling1/lidar/scan_filtered   # expect ~8.7 Hz, same as raw scan
ros2 topic hz /gosling1/lidar/scan            # compare — rates should match
```
- RViz: display both `lidar/scan` (grey) and `lidar/scan_filtered` (color).
  - Isolated single-point fliers should be gone in filtered.
  - Wall corners, the L-junction, and thin objects must SURVIVE. If real returns
    are being eaten, revert to `filter_window: 1` check / disable speckle filter.

➡ filtered rate: **8.646** Hz | corners preserved: **Y** | fliers removed: **Y** — **PASS**

Raw 8.648 Hz vs filtered 8.646 Hz — no scan drops. Instead of eyeballing RViz, removed beams
were classified by the run-length of consecutive valid raw beams they belonged to (75 matched
scan pairs, 6.1 beams removed/scan):

| run length of removed beam | share |
|---|---|
| 1 (isolated singleton)     | 89.7% |
| 2                          |  2.2% |
| ≥5 (structure)             |  4.8% |

91.9% of removals are isolated singletons — true speckle. The 4.8% from runs ≥5 are
run-edge points on wall segments, which is expected. Real structure survives; no need to
fall back to `filter_window: 1`.

### A2. Forked rf2o verification (§5 — was `[!]`, now expected fixed)

```bash
# Fork respects publish_tf: node must NOT be a /tf publisher
ros2 param get /gosling1/CLaserOdometry2DNode publish_tf     # expect: false
ros2 node info /gosling1/CLaserOdometry2DNode | grep -A5 Publishers
# expect: NO /gosling1/tf in publisher list (upstream bug published TF anyway)

# Fork adds covariance: must NOT be all zeros
ros2 topic echo /gosling1/odom/rf2o --once | grep -A6 covariance | head -8
# expect: non-zero diagonal values
```
➡ tf publisher absent: **Y** | covariance non-zero: **Y** — **PASS**

Node is `rf2o_laser_odometry` (NOT `CLaserOdometry2DNode` — this run sheet's node name is stale).
`publish_tf` → `False`; publishers are exactly `/odom/rf2o`, `/parameter_events`, `/rosout` —
no `/tf`. Covariance diagonal 0.0025 (non-zero). The fork's fixes are confirmed on-robot.

### A3. EKF fusion health (§5)

```bash
ros2 topic hz /gosling1/odometry/local        # expect ~30 Hz
ros2 topic echo /gosling1/odometry/local --once
# pose covariance diagonal: small, not NaN, ideally < 1.0 for x/y/yaw
# (previously 4.39/4.42 with zero-cov rf2o over-weighting)

# No differential/relative warnings should appear in the launch log anymore
```
➡ rate: **29.86** Hz | cov diag (x,y,yaw): **0.0003 / 0.0003 / 0.0007** — **PASS** (bag replay)

**0 differential/relative warnings** in the launch log — that fix holds.

Two measurements, only the second is valid:

- **Live, VESC unpowered — INVALID.** 6.97 Hz with 2.8 s gaps, cov 7.61/6.78. With the camera
  also dead (BUG-006) the EKF ran on rf2o alone. Do not read this as a regression.
- **Bag replay (`loop3x_no_localization`, all sources incl. `vesc_odom`) — VALID.**
  29.86 Hz, cov 0.0003/0.0003/0.0007 — far below the <1.0 target, vs 4.39/4.42 previously.

Noted: 25 × `Failed to meet update rate!` (took ~0.076 s vs the 30 Hz budget) — the EKF misses
its deadline under full sensor load on this hardware. Not fatal, but worth watching.

### A4. Stationary yaw drift check (§5 — was ~0.5°/s from competing rf2o TF)

```bash
# Robot STATIONARY for 60 s:
ros2 run tf2_ros tf2_echo odom base_link --ros-args -r /tf:=/gosling1/tf -r /tf_static:=/gosling1/tf_static
# watch yaw; expect < 0.05 rad total drift over 60 s (was ~0.5 deg/s = 0.52 rad/60s)
```
➡ yaw drift over 60 s: **+0.0798 rad** (+4.57°, = **0.079 °/s**) — **IMPROVED** (was ~0.5 °/s)

1569 samples over 57.9 s. Total excursion 4.573° — monotonic, not oscillation. Comfortably
under the 0.5 rad/60 s the competing-TF bug used to produce.

**Position drift over the same window was 3012 m** (dx +1867, dy −2364), EKF twist pegged at
(+22.8, −41.9) m/s while rf2o read 1.27 m and VSLAM 0.05 m of drift. Root-caused to accel
integration with no velocity anchor — see BUG-010. This is an artifact of the unpowered VESC,
NOT a config regression, but it exposes a real dropout fragility.

### A5. Loop-closure drive test (§5 — headline test of the session)

Previous result: position closure sub-cm, yaw residual **−24.3°**. With forked
rf2o (proper covariance, no competing TF) + `cb3b567` EKF fixes, yaw should
improve substantially.

```bash
# Note start pose, drive the same ~5 m elliptical loop, return to origin:
ros2 topic echo /gosling1/odometry/local | grep -A11 'pose:'
# At closure record x, y, and quaternion z/w.
# yaw_deg = 2*atan2(z, w) * 180/pi
```
➡ x: **+0.0021** m | y: **−0.0317** m | yaw residual: **+4.78 °** (prev −24.3°) — **MUCH IMPROVED**

Done by **bag replay**, not a live drive (robot stationary this session).
`loop3x_no_localization` replayed through the current localization stack, namespaced,
`use_sim_time:=True`, bag `/gosling1/tf` excluded so the live EKF owned `odom→base_link`.

- 1859 samples, 62.0 s sim time, max excursion 3.02 m from start (loop size sanity).
- Position closure |d| = **0.0317 m**; yaw residual **+4.78°** vs previous **−24.3°** (~5× better).

Caveat: May-2026 sensor data through today's fusion config — fair for the fusion layer, but the
bag's `scan_filtered` is the OLD pre-`9b76dd9` filter output, and this is not a live drive.
**4.78° is right at the ~5° threshold in the Wrap-up**, so the live drive decides whether IMU
yaw drift remains the open suspect.

### A6. AMCL particle cloud in RViz (§6 — needs nav2_rviz_plugins)

Root cause was display type: Nav2 Humble publishes `nav2_msgs/ParticleCloud`,
not `PoseArray`.

- In RViz: **Add → By display type → nav2_rviz_plugins → Particle Cloud**, topic `/gosling1/particle_cloud`.
- Set 2D Pose Estimate; particles should render and converge while driving.

```bash
ros2 topic hz /gosling1/particle_cloud   # ~0.8 Hz stationary is expected (motion-gated)
```
➡ particles visible: **N/A (headless)** | converge on motion: **NOT TESTED** (needs motion)

`/particle_cloud` publishes at **8.97 Hz**, type **`nav2_msgs/msg/ParticleCloud`** — confirms the
diagnosed root cause (it is not `PoseArray`, so the RViz display type must come from
`nav2_rviz_plugins`). `/amcl_pose` at 8.68 Hz. Note the 8.97 Hz vs the "~0.8 Hz stationary"
expectation: the motion gate was being satisfied by the runaway EKF odometry (BUG-010), so AMCL
believed it was moving. Re-check this rate once the VESC is powered.

### A7. §10 Teleop mode: full stack TF tree

```bash
ros2 run tf2_tools view_frames --ros-args -r /tf:=/gosling1/tf -r /tf_static:=/gosling1/tf_static
# Expected single chain: map -> odom -> base_link -> {lidar, camera_link, sensor_kit_link, ...}
# Exactly ONE publisher per edge (no duplicate odom->base_link).
```
➡ tree complete: **Y** | duplicate edges: **N** — **PASS**

Sampled `/tf` for 12 s and counted (parent → child) pairs instead of using `view_frames`
(headless). Exactly two dynamic edges, one publisher each:

```
map       -> odom        n=119   (amcl)
odom      -> base_link   n=282   (ekf_odom_node)
```

No duplicate `odom→base_link`. Caveat: run WITHOUT a namespace — re-verify namespaced.

Shut down Session A.

---

## Session B — Mapping mode: §10 TF conflicts + §8 map save service

```bash
ros2 launch f1tenth_launch bringup.launch.py slam:=True launch_navigation:=False use_gpu:=False
```

### B1. §10 Mapping mode: no TF conflicts

- Watch launch log for TF extrapolation errors / duplicate TF warnings.
```bash
ros2 run tf2_tools view_frames --ros-args -r /tf:=/gosling1/tf -r /tf_static:=/gosling1/tf_static
# ONE odom->base_link (EKF), ONE map->odom (slam_toolbox)
```
➡ conflicts: **Y — duplicate odometry nodes (BUG-007)**

0 TF extrapolation / TF_REPEATED / authority warnings in the log. But `slam:=True` **double-launches
the whole localization stack**: `icp_odometry` ×2 and `stereo_odometry` ×2 (confirmed via `ps`).
Chain: `bringup:943 → mapping:577 → teleop:645 → localization`, **plus** `bringup:855 → localization`
directly. Consequence in the log:

```
[map_server] Unable to start transition 1 from current state active: Transition is not registered
[lifecycle_manager_localization] Failed to change state for node: map_server
[lifecycle_manager_localization] Failed to bring up all requested nodes. Aborting bringup.
```

Also: **`slam:=True` alone starts NO slam_toolbox** — `launch_2d_mapping` defaults `False`
(`mapping.launch.py:91`), so there is no `/map` and no `map_saver`. This run sheet's B1/B2 need
`launch_2d_mapping:=True` added to the command above. See BUG-008.

### B2. §8 Map save via service (VERIFIED wiring 2026-07-22 — this is the supported path)

`map_saver_server` runs at `/gosling1/map_saver` and never auto-saves; save on demand:

```bash
# Drive a bit first so there is a map, then:
ros2 service call /gosling1/map_saver/save_map nav2_msgs/srv/SaveMap \
  "{map_topic: map, map_url: data/maps/raslab_test, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"
ls -la data/maps/raslab_test.yaml data/maps/raslab_test.pgm
```
➡ service returned success: **Y** | files written: **Y** — **PASS**

Requires `launch_2d_mapping:=True` (see B1). Both `/map_saver/save_map` and
`/slam_toolbox/save_map` services present. Call returned `result=True`:

```
-rw-r--r-- 44649 /tmp/raslab_test.pgm
-rw-r--r--   129 /tmp/raslab_test.yaml
```

Saved to `/tmp` inside the container rather than `data/maps/` to avoid writing into the repo;
the map is near-empty since the robot never moved. Service wiring is confirmed working —
this remains the supported save path.

### B3. §8b RTABMap online mode (was `[!]`, blocked by use_gpu leak — since fixed)

Shut down B, then:
```bash
ros2 launch f1tenth_launch bringup.launch.py slam:=True launch_navigation:=False use_gpu:=False launch_2d_mapping:=False launch_3d_mapping:=True
ros2 node list | grep rtabmap          # rtabmap nodes present
ros2 node list | grep -E 'nvblox|visual_slam'   # expect EMPTY (use_gpu=False)
```
➡ rtabmap up: **Y** | no GPU nodes: **Y** — **PASS** (the `use_gpu` leak is fixed)

Verified via `ps` (node list unreliable, see run log). RTABMap online:

```
1 rtabmap_slam/rtabmap      1 rtabmap_sync      1 rtabmap_viz
2 rtabmap_odom/icp_odometry           2 rtabmap_odom/stereo_odometry
```

**0** nvblox / visual_slam processes with `use_gpu:=False` — the leak that blocked this is
genuinely fixed. The ×2 odometry nodes are BUG-007 (double-include), not a GPU issue.
Note `rtabmap_viz` (a GUI) starts here and costs CPU on the Jetson.

Shut down Session B.

---

## Session C — Navigation: §9 nav goal + CPU

Container-crash blocker was fixed by isolating VSLAM in its own container (`b47d45d`) — retest.

```bash
ros2 launch f1tenth_launch bringup.launch.py slam:=False launch_navigation:=True launch_global_localization:=True launch_visualization:=True
```

### C1. Stack stays up (crash regression check)

```bash
# after ~2 min uptime:
ros2 node list | grep -cE 'controller_server|planner_server|bt_navigator'   # expect 3
```
➡ alive after 2 min: **Y** — **PASS**, no crash regression

4 processes matching `controller_server|planner_server|bt_navigator` alive after >2 min uptime
(counted via `ps`; `ros2 node list` returned 0 due to the discovery flakiness noted in the run log).
The container-crash blocker fixed by `b47d45d` did not recur.

### C2. Send a navigation goal (§9)

- RViz: 2D Pose Estimate first (AMCL init), then **2D Nav Goal** a few meters away.
- Expect: path renders, robot drives, no oscillation/hesitation at 10 Hz controller.
- Keep the joystick heartbeat alive (command_gate) or the gate closes.

➡ **NOT TESTED** — needs motion + RViz. Deferred to the live session.

Blockers to be aware of when you run it: with no joystick connected, `command_gate` starts closed
(`command_gate_require_heartbeat:=True` by default in bringup) and will block all commands to the
VESC. Either keep the joystick alive or pass `command_gate_require_heartbeat:=False`.

### C3. CPU load during navigation (§9)

```bash
top -b -n1 | head -20
# composed nodes hide under the container process — watch container CPU%.
# Compare against pre-tuning baseline (~previous runs); expect lower with 5 Hz/1 Hz costmaps.
```
➡ container CPU during nav: **`planner_server` 94.1% of one core, idle with no goal**

```
%Cpu(s): 32.1 us, 11.3 sy, 53.8 id     load average: 12.60, 9.74, 6.15   (6 cores)
 117767 planner_server   94.1%
 113953 realsense2_...   23.5%
 116357 component_...    23.5%
```

`planner_server` pegging a core while **stationary with no goal sent** is not expected and is
worth investigating (BUG-009). Load 12.6 on 6 cores is oversubscribed, though a concurrent agent's
MPC sim was also running on this host, so total load is not solely this stack. A clean
pre/post-tuning comparison still needs a quiet machine + an actual nav goal.

Shut down Session C.

---

## Session D — §10 TF-publisher arg propagation

```bash
ros2 launch f1tenth_launch bringup.launch.py odom_tf_publisher:=rf2o map_tf_publisher:=rtabmap slam:=False
ros2 param get /gosling1/CLaserOdometry2DNode publish_tf    # expect: true (rf2o now owns odom TF)
ros2 node info /gosling1/CLaserOdometry2DNode | grep /gosling1/tf   # expect: present now
# and the EKF must NOT publish odom->base_link in this mode:
ros2 run tf2_tools view_frames --ros-args -r /tf:=/gosling1/tf -r /tf_static:=/gosling1/tf_static
```
➡ rf2o owns odom TF: **Y** | single odom→base_link edge: **Y** — **PASS**

- `rf2o_laser_odometry` publishers now include `/tf` (contrast with A2, where it did not) —
  `odom_tf_publisher:=rf2o` propagates correctly through bringup.
- `ekf_odom_node` `publish_tf` → **`False`** — the EKF correctly yields TF ownership.

Gotcha for future runs: `ros2 node info /ekf_odom_node` still lists `/tf` under Publishers even
in this mode. That is NOT evidence of a conflict — robot_localization constructs the TF
broadcaster unconditionally and simply does not broadcast when `publish_tf` is false.
**Check the `publish_tf` param, not the publisher list.**

---

## Wrap-up

- Transfer all ➡ results into `testing_checklist.md` (flip `[ ]`/`[!]` accordingly).
- If A5 yaw residual is still large (> ~5°), the remaining suspect is IMU yaw drift
  (no magnetometer correction) — log it as a new bug with the measured value.
- Log any new bugs in the Known Bugs section with date + repro.

### 2026-08-04 outcome

Passed: A1, A2, A7, B2, B3, C1, D. Improved: A4 (0.079 °/s), A5 (+4.78° vs −24.3°),
A3 covariance (0.0003 vs 4.39). New bugs: **BUG-006 … BUG-011** (see `.wolf/buglog.json`
and the Known Bugs section of `testing_checklist.md`).

A5 landed at **+4.78°, just under the ~5° threshold** — so the IMU-yaw-drift suspect is
neither confirmed nor cleared. The live drive resolves it.

**Still outstanding (all need the car powered/moving):**

1. A5 live loop closure — the bag result is not a substitute.
2. A6 particle convergence under motion, and the stationary `/particle_cloud` rate re-checked
   with a powered VESC.
3. C2 nav goal + a clean C3 CPU comparison on a quiet machine.
4. **Re-run everything namespaced** (`use_f1tenth_namespace:=True`) — live A–D were not.
5. **Re-run with `use_composition:=True`** once BUG-006 is fixed; the default path was never
   exercised end-to-end this session.
6. Sections 1–3 of `testing_checklist.md` (VESC actuation, joystick, safety mux) were entirely
   untestable with the motor unpowered and no joystick attached.

---

## RUN LOG — 2026-08-04 (namespaced re-run, agent-driven, gosling1)

Follow-up session addressing item 4 above: re-run A–D with `use_f1tenth_namespace:=True`.

Conditions:

- **VESC still unpowered**, robot stationary. `vehicle/vesc_odom` and `vehicle/sensors/imu/raw`
  had NO DATA throughout — A3/A5 remain unresolved by this session.
- All sessions used `use_composition:=False` (to keep the comparison against the un-namespaced
  runs apples-to-apples); a separate composition check is recorded at the end.
- `reset_realsense:=True` added from Session B onward — the D435i wedges after repeated
  container restarts, and this reliably recovers it.
- Container `f1tenth_claude_test`, `ROS_DOMAIN_ID=77`, restarted between every session so that
  leftover processes could not contaminate results (an early bisect **was** contaminated this
  way — see the note under BLOCKER below).

### BLOCKER found and fixed: double namespace via sibling-include config leak

The first namespaced run failed completely. Sensors/vehicle/camera/VSLAM landed on `/gosling1/*`
while **every localization node landed on `/gosling1/gosling1/*`** and therefore received no data:

```
/gosling1/lidar/scan                      8.611 Hz
/gosling1/lidar/scan_filtered             8.663 Hz
/gosling1/visual_slam/tracking/odometry   30.028 Hz
/gosling1/camera/imu/filtered            200.172 Hz
/gosling1/gosling1/lidar/scan_filtered    NO DATA
/gosling1/gosling1/odom/rf2o              NO DATA
/gosling1/gosling1/odometry/local         NO DATA   <- EKF produced nothing
/gosling1/gosling1/amcl_pose              NO DATA
```

`rf2o` logged `Waiting for laser_scans....` for the whole run; `icp_odometry` reported
subscribing to `/gosling1/gosling1/lidar/scan_filtered`.

**Root cause — the launch-config inheritance leak, sibling-to-sibling via `ld` ordering.**
`bringup.launch.py`'s `ld` is `[nodes_to_launch, sensors_launch, localization_launch]`. The
sensors include passes `use_namespace: True` / `namespace: gosling1` as `launch_arguments`,
which leak into the shared context. `localization_launch` is visited **after** it, so its
`PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace)` — dormant by
default because `use_namespace` defaults `False` — silently fired, on top of the namespacing
`localization.launch.py` already does itself. `vehicle_bringup_group` escaped only because
`nodes_to_launch` is visited *before* the leak.

Bisect (each run on a freshly restarted container):

| Run | Localization namespace |
|---|---|
| `localization.launch.py` standalone, `use_namespace:=True` | `/gosling1` ✓ |
| bringup, `launch_sensors:=False` | `/gosling1` ✓ |
| bringup, `launch_sensors:=False use_gpu:=True` | `/gosling1` ✓ |
| bringup, `launch_sensors:=True` | `/gosling1/gosling1` ✗ |

Delaying sensors to 25 s/30 s (past localization's 10 s timer) still produced the double at
t=18 s — so this is description-build time, **not** runtime timer ordering.

**Fix applied** to `launch/bringup.launch.py` (uncommitted, working tree): removed the
`PushRosNamespace` from the localization include (~line 850) and the mapping include (~938),
keeping both `SetRemap` `/tf` pairs. `teleop.launch.py` needed no change — it already avoids
this by placing `sensors_launch`/`localization_launch` *outside* its pushing `GroupAction`
(`ld = launch_args + [nodes_to_launch, sensors_launch, localization_launch]`).

After the fix every node is single-namespaced, with the intended `/gosling1/lidar` and
`/gosling1/vehicle` sub-namespaces preserved.

### Namespaced results

| Test | Result | Notes |
|---|---|---|
| A1 laser filter | **PASS** | scan 8.725 Hz, filtered 8.735 Hz — no drops |
| A2 forked rf2o | **PASS** | `publish_tf=False`, covariance diag 0.0025, no `/tf` publisher |
| A3 EKF fusion | **INVALID** | 11.47 Hz, but camera was dead this run — see Do-Not-Repeat |
| A4 yaw drift | **+4.944° / 57 s = 0.0867 °/s** | un-namespaced was 0.079 °/s; still ≫ better than the old 0.5 °/s |
| A4 position drift | **0.154 m** | vs **3012 m** last session — see BUG-010 note below |
| A6 particle cloud | **no data — CORRECT** | AMCL is motion-gated and the robot was stationary |
| A7 TF tree | **PASS** | `odom→base_link` n=809, `map→odom` n=286, one publisher each |
| B1 mapping TF | **PASS** | 2 edges, **0** lifecycle aborts, 0 TF warnings |
| B2 map save | **PASS** | `result=True`; pgm 44649 B + yaml written |
| B3 RTABMap online | not re-run | covered un-namespaced last session |
| C1 nav2 alive | **PASS** | 3 servers up after 100 s, all at `/gosling1` |
| C3 planner CPU | **0–4.5% instantaneous** | 24% lifetime avg, front-loaded at costmap init — BUG-009 did **not** reproduce |
| D TF-publisher args | **PASS** | rf2o `publish_tf=True`, ekf `False`, single `odom→base_link` (n=149 ≈ 7.5 Hz) |

AMCL and `map_server` both reached lifecycle state `active` namespaced. AMCL is correctly wired
(`scan_topic: lidar/scan_filtered` → `/gosling1/lidar/scan_filtered`; `/gosling1/map` 1 pub /
1 sub) — it simply does not publish `amcl_pose` while stationary.

**Two results corroborate earlier diagnoses from the opposite direction:**

- A4's position drift collapsing from **3012 m → 0.154 m** is exactly what BUG-010 predicts: the
  camera IMU was dead in that run, so there was no gravity-leak acceleration to integrate.
- BUG-009 did not reproduce. Last session's sustained 94.1% `planner_server` coincided with the
  runaway EKF feeding garbage odometry into the global costmap. **BUG-009 may be a symptom of
  BUG-010 rather than an independent defect** — re-check it once the VESC is powered.

### Composition check (`use_composition:=True`, namespaced) — and BUG-006 FIXED

First pass: the default composition path ran clean (0 USB-busy errors, `camera/imu/filtered`
200.373 Hz, one `/gosling1/camera`) **but `ros2 node list` still showed
`/gosling1/sensing_container` three times** — BUG-006's name collision was unchanged and the
RealSense had merely been lucky. Treat that class of bug as a **race**, never "fixed because one
run passed".

**BUG-006 then fixed** with two edits that remove all three same-name creators:

1. `localization.launch.py` — `vslam_container_name` was `'{ns}/sensing_container'`, i.e. VSLAM
   was told to *create* a container under the RealSense's name, directly contradicting the
   adjacent comment (`we now want vslam to be isolated from the rest`) and the `b47d45d` intent.
   Changed to `'{ns}/visual_slam_container'` — the name the non-composed branch already used.
2. `sensors.launch.py` — the `stereo_and_depth_image_processing` include inherited
   `attach_to_shared_component_container: False`. Both it and `realsense_d435i.launch.py` create
   their container under `condition=UnlessCondition(attach_to_shared_component_container)`, so
   with the same name two `sensing_container` nodes appeared. Now passes `'True'` to attach,
   which also preserves intra-process zero-copy with the RealSense driver.

Verified on-robot, namespaced, `use_composition:=True`:

```
container names: sensing_container, visual_slam_container,
                 localization_container, command_gate_container   (one each; was sensing_container x3)
ros2 component list: /gosling1/sensing_container    -> /gosling1/camera
                     /gosling1/visual_slam_container -> /gosling1/visual_slam_node
USB-busy / "share an exact name" errors: 0
/gosling1/camera/imu/filtered            199.831 Hz
/gosling1/visual_slam/tracking/odometry   30.019 Hz   <- VSLAM alive on the composed path
/gosling1/lidar/scan_filtered              8.723 Hz
/gosling1/odom/rf2o                        8.334 Hz
/gosling1/odometry/local                  30.001 Hz
```

**This is the first end-to-end success of the default (composed) path.** Regression-checked with
`use_composition:=False`: 0 errors, camera 200.125 Hz, `odometry/local` 30.001 Hz, namespaces
clean — no regression from the `sensors.launch.py` change.

### A3 EKF fusion — now VALID (composed, namespaced)

With the camera and VSLAM finally alive, A3 can be measured properly:

➡ rate **30.001** Hz | cov diag (x, y): **1.778e-4 / 1.778e-4** — **PASS**

Better than the bag-replay figure (0.0003) and far below the <1.0 target; **0**
differential/relative warnings. Live inputs: `camera/imu/filtered` 199.8 Hz,
`visual_slam/tracking/odometry` 30.0 Hz, `odom/rf2o` 8.3 Hz. Still missing `vehicle/vesc_odom`
(VESC unpowered), so this is not yet the full sensor set.

Caveat carried forward: **30 × `Failed to meet update rate!`** in the log — the EKF still misses
its 30 Hz deadline under load on this hardware, same as last session.

### Session B: BUG-007 is worse under a namespace, but its symptom inverts

With `slam:=True launch_2d_mapping:=True`, the duplicate localization stack lands at **root `/`**
(8 nodes: `amcl`, both EKFs, `map_server`, `rf2o`, `icp`, `stereo`, `lifecycle_manager`) because
`mapping.launch.py:645` passes `use_namespace: 'False'` to its teleop include. The duplicate is
**inert** — `/odometry/local`, `/odom/rf2o`, `/tf`, `/map` at root all read NO DATA — so it
wastes CPU/RAM but does not corrupt `/gosling1/tf`. Side effect: the `map_server` lifecycle
abort no longer occurs, because the duplicate carries its own root-level `map_server`.

### Still outstanding

1. A5 live loop closure, A6 convergence under motion, C2 nav goal — all need the car moving.
2. A3 namespaced with the camera confirmed live (this run's number is invalid).
3. Checklist §1–3 (VESC actuation, joystick, safety mux) — need motor power + joystick.
4. Re-run A–D with `use_composition:=True` (the default) as the primary path.
5. Verify `teleop.launch.py` standalone under namespace + composition.

---

## RUN LOG — 2026-08-04 (composed-path re-run + teleop coverage, agent-driven, gosling1)

Addresses items 4 and 5 of the previous block: exercise the **default composed path** end to end,
and cover `teleop.launch.py`, which had never been launched in any configuration.

Conditions: container `f1tenth_claude_test`, `ROS_DOMAIN_ID=77`, `docker restart` between every
session. **VESC still unpowered** — `vehicle/vesc_odom` and `vehicle/sensors/imu/raw` NO DATA
throughout, so A3/A5 remain on a partial sensor set. Robot stationary. `reset_realsense:=True`
on every run. All runs `use_f1tenth_namespace:=True use_composition:=True`.

Container repo fast-forwarded `9b76dd9 → 03faef3` (the working-tree copies from `docker cp` were
byte-identical to the commit, so the fast-forward was clean).

### teleop.launch.py under namespace + composition — the suspected defect was real (BUG-017)

```bash
ros2 launch f1tenth_launch teleop.launch.py use_f1tenth_namespace:=True use_composition:=True reset_realsense:=True
```

```
ros2 component list:
  /gosling1/f1tenth_container      -> amcl, map_server, lifecycle_manager_localization, rf2o
  /gosling1/visual_slam_container  -> visual_slam_node
  /gosling1/command_gate_container -> command_gate
                                      <- NO sensing_container, NO camera anywhere
ps: no realsense2_camera_node process at all
camera/imu/filtered            NO DATA
camera/infra1/image_rect_raw   NO DATA
visual_slam/tracking/odometry  NO DATA    <- VSLAM loads, then starves
lidar/scan_filtered            8.671 Hz   <- LiDAR path unaffected
odometry/local                11.145 Hz   <- degraded, rf2o only
```

Same command with `use_composition:=False`: camera IMU **200.480** Hz, color **30.082** Hz,
VSLAM **30.042** Hz, `odometry/local` **30.025** Hz. So the defect is specific to the composed path.

**No error is logged.** The log has `Loaded node ...` lines for command_gate/amcl/map_server/rf2o/
visual_slam and simply nothing for the camera — `LoadComposableNodes` waits forever on a load
service that never appears. This is the failure mode to watch for: a *missing* log line, not an
error one.

Root cause and fix: see BUG-017 in `testing_checklist.md` / `.wolf/buglog.json`.

### The fix unmasked a second defect of the same class (BUG-018)

After BUG-017 was fixed, `ps` showed **four** containers where `ros2 component list` showed three —
a second `f1tenth_container` (`component_container_mt`, loading nothing) alongside teleop's
`component_container_isolated`. teleop's localization include passed `container_name` but
*inherited* `attach_to_shared_component_container`; that inherited value had been `'True'` only
because the sensors include leaked it sideways (BUG-013's mechanism), and flipping sensors to
`'False'` exposed the dormant second creator. Fixed by passing the argument explicitly.

**Generalisable point: the BUG-013 leak can be load-bearing.** A launch file that works today may
be relying on a sibling's leaked configuration. Fixing one leak can activate a latent second bug,
so re-inspect the container topology (`ps` **and** `ros2 component list`, they disagree) after any
change to these arguments.

### teleop.launch.py — final state, namespaced + composed: PASS

```
containers (one process each, all uniquely named):
  sensing_container -> /gosling1/camera        visual_slam_container -> /gosling1/visual_slam_node
  f1tenth_container -> amcl, map_server, lifecycle_manager_localization, rf2o
  command_gate_container -> /gosling1/command_gate
camera/imu/filtered            200.376 Hz
camera/color/image_raw          36.387 Hz
camera/infra1/image_rect_raw    30.008 Hz
lidar/scan  8.711 Hz  |  lidar/scan_filtered  8.726 Hz  |  odom/rf2o  9.091 Hz
visual_slam/tracking/odometry   31.147 Hz
odometry/local                  30.038 Hz   pose cov 1.768e-4
RS2_USB_STATUS_BUSY: 0 | "share an exact name": 0 | OpenGL errors: 0 | process has died: 0
/gosling1/gosling1/* topics: 0
```

Namespacing under teleop was clean in both composed and non-composed modes, as predicted — teleop
already keeps `sensors_launch`/`localization_launch` outside its pushing `GroupAction`.

One VSLAM `SIGABRT` (`exit code -6`, preceded by `Delta between current and previous frame
[633.8 ms] is above threshold [34.0 ms]` and `Failed to meet update rate! Took 3.07 s`) occurred in
the intermediate run that still had the duplicate `_mt` container spinning. **It did not reproduce**
after BUG-018 was fixed. Treated as load-induced, not a wiring defect — but worth re-checking under
the VESC-powered load.

### Session A — bringup, composed, namespaced: PASS

```bash
ros2 launch f1tenth_launch bringup.launch.py slam:=False launch_navigation:=False \
  launch_global_localization:=True use_f1tenth_namespace:=True use_composition:=True reset_realsense:=True
```

| Test | Result |
|---|---|
| A1 laser filter | scan **8.777** Hz / filtered **8.773** Hz — no drops — **PASS** |
| A2 forked rf2o | `publish_tf` **False**, covariance diag **0.0025** — **PASS** |
| A3 EKF fusion | **30.000** Hz, pose cov **1.522e-4** — **PASS** |
| A4 stationary drift | yaw 0.031°→0.099° over 59 s = **0.0012 °/s**; position **0.005 m** — **PASS** |
| A6 particle cloud | NO DATA — **correct**, AMCL is motion-gated and the robot was stationary |
| A7 TF tree | `odom→base_link` 637, `map→odom` 175 over 25 s, one edge each — **PASS** |
| namespacing | **0** `/gosling1/gosling1/*` topics — **PASS** |
| containers | 5, all uniquely named — **PASS** |

A4 improved from the previous namespaced run's **0.0867 °/s** to **0.0012 °/s**, and the reason is
in the data rather than in any config change: VSLAM was alive here (30.174 Hz) and dead then.
Live inputs: camera IMU 200.032 Hz, VSLAM 30.174 Hz, rf2o 9.096 Hz; `vehicle/vesc_odom` NO DATA.

`ros2 param get` returned `Node not found` first try and succeeded on retry — BUG-012 again.

Noted: `/gosling1/f1tenth_container` was **empty** in this run — but that is correct, not a defect:
it is the **nav2 container** (`bringup.launch.py:671` builds `nav2_container_name` from it and
passes it to the nav2 include at line 1018), and this session ran `launch_navigation:=False`.
What still needs checking on the composed path: `nav2_navigation.launch.py` declares its OWN
defaults `use_composition='False'` / `container_name='nav2_container'` (lines 328-331). Bringup
overrides both, but if that override were ever missed or leaked over, nav2's `LoadComposableNodes`
would target a `nav2_container` that nothing creates — the exact silent failure mode of BUG-017.
Session C must confirm via `ros2 component list` that the nav2 servers actually land inside
`/<ns>/f1tenth_container`, not merely that they are alive.

### Session B — mapping, composed, namespaced

```bash
ros2 launch f1tenth_launch bringup.launch.py slam:=True launch_2d_mapping:=True \
  launch_navigation:=False use_gpu:=False use_f1tenth_namespace:=True use_composition:=True reset_realsense:=True
```

- `/gosling1/map` **10.063** Hz, `odometry/local` **28.998** Hz, `lidar/scan_filtered` 8.721 Hz.
  `launch_2d_mapping:=True` still required (BUG-008 unchanged).
- **B2 map save — PASS**: `result=True`, `sessB_map.pgm` 44649 B + `.yaml` written.
- **B1 map-frame ownership — PASS**: `map→odom` is published by `slam_toolbox` alone;
  `amcl` has `tf_broadcast: False`, so no competing map-frame publisher.
  **[CORRECTION 2026-08-04]** The second clause cited the wrong mechanism. AMCL's `tf_broadcast` is
  set by the **launch file**, not the YAML: `localization.launch.py:384/429` passes
  `tf_broadcast: True if map_tf_publisher == 'amcl' else False` after `params_file`, so it overrides
  `localizer_amcl.yaml` either way. The real reason there was no competing publisher here is simply
  that **AMCL was not running** in this 2D run. On the RTABMap path it was, and `map_tf_publisher`
  defaults to `'amcl'`, so it became one of three competing `map→odom` broadcasters — see BUG-027.
  The PASS verdict on `slam_toolbox` sole ownership stands; the stated reason for it does not.
- BUG-007 unchanged: root-level duplicate stack present (48 non-namespaced topics) and inert
  (`/odometry/local` at root NO DATA).

### BUG-019 — SAFETY: `slam:=True` ran two `command_gate` nodes on the actuation topic (FOUND + FIXED)

Session B surfaced this. `ros2 component list` showed `/gosling1/command_gate` **twice** inside
`/gosling1/command_gate_container`, `ps` showed two `command_gate_container` processes, the log had
two `Loaded node '/gosling1/command_gate'` lines, and:

```
ros2 topic info -v /gosling1/vehicle/ackermann_cmd  ->  Publisher count: 2
```

bringup includes `command_gate.launch.py` directly *and*, under `slam:=True`, reaches it again via
`mapping.launch.py → teleop.launch.py`. Bringup's mapping include already passes `'False'` for
`launch_joystick`/`launch_sensors`/`launch_vehicle`/`launch_tfs`/`launch_localization`;
`launch_command_gate` was missing from that list.

This one does **not** share BUG-007's saving grace. The duplicate localization stack lands at root
`/` and is inert; both `command_gate` copies land at `/<namespace>` and both publish
`vehicle/ackermann_cmd`, each running an independent heartbeat watchdog. Since the gate publishes a
zero-initialized `AckermannDriveStamped` whenever it closes, the two can interleave zero and real
commands on the same topic. Invisible with the VESC unpowered; an actuation hazard the moment the
motor is powered and driving in mapping mode — which is exactly what the A5 loop-closure drive needs.

Fixed by adding `"launch_command_gate": 'False'` to bringup's mapping include.

```
slam:=True  after fix: 1 'Loaded node' line | 1 container process | Publisher count: 1
                       /gosling1/map 4.569 Hz | odometry/local 31.865 Hz | scan_filtered 8.746 Hz
slam:=False regression: 1 'Loaded node' line | 1 container process | Publisher count: 1
                       odometry/local 30.003 Hz
```

### Still outstanding

1. **Sessions C and D not re-run on the composed path** — deferred to the next session.
2. RTABMap is the primary mapper (it also produces 2D maps); this session only exercised the
   SLAM Toolbox path. RTABMap composed coverage deferred with C/D.
3. A5 live loop closure, A6 convergence under motion, C2 nav goal — all need the car moving.
4. Checklist §1–3 (VESC actuation, joystick, safety mux) — need motor power + joystick.
5. Still open, no fix attempted: BUG-007, BUG-008, BUG-010, BUG-012.
6. VSLAM `SIGABRT` seen once under load — re-check with the VESC powered.

---

## RUN LOG — 2026-08-04 (Session C composed + RTABMap, agent-driven, gosling1)

Addresses items 1 and 2 of the previous block: Session C/D on the composed path, and the first
composed test of RTABMap — the primary mapper.

Conditions: container `f1tenth_claude_test`, `ROS_DOMAIN_ID=77`, `docker restart` between runs.
**VESC still unpowered** (`vehicle/vesc_odom`, `vehicle/sensors/imu/raw` NO DATA throughout);
robot stationary; `reset_realsense:=True` on every run. All runs
`use_f1tenth_namespace:=True use_composition:=True`.

Container repo note: the container's `launch/` tree was byte-identical to `051bf31` but sitting on
`03faef3` with an uncommitted worktree, and `051bf31` is **docs-only** — so the code under test was
already current and no `git pull` was required. Two attempted git cleanups (`git checkout --`,
`git stash`) were blocked by the permission classifier and deliberately not worked around.

### Session C — nav2, composed + namespaced: FAILED, root-caused, FIXED, re-verified

```bash
ros2 launch f1tenth_launch bringup.launch.py slam:=False launch_navigation:=True \
  launch_global_localization:=True use_f1tenth_namespace:=True use_composition:=True reset_realsense:=True
```

First run: nav2 loaded into the right container but under a **doubled namespace**, and the whole
nav2 lifecycle bringup aborted (see BUG-020). The tell was in `ros2 component list`:

```
/gosling1/f1tenth_container
  1  /gosling1/gosling1/controller_server      <- doubled
  ...
  8  /gosling1/gosling1/lifecycle_manager_navigation
```

```
[gosling1.gosling1.controller_server]: Couldn't load critics! ... No critics defined for FollowPath
[gosling1.gosling1.controller_server]: Caught exception in callback for transition 10
[gosling1.gosling1.lifecycle_manager_navigation]: Failed to change state for node: controller_server
[gosling1.gosling1.lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup.
```

All 7 servers `unconfigured`; `local_costmap` had `costmap`/`costmap_raw` but `global_costmap` had
**only** `transition_event` — and it would have been subscribed to `/gosling1/gosling1/map`, which
nothing publishes. Sensors and localization were unaffected (`lidar/scan` 8.783 Hz,
`scan_filtered` 8.782 Hz, `odometry/local` 30.005 Hz).

Root cause and fix in BUG-020. One-line version: **a parent that pushes a namespace requires its
children to use absolute namespaces; a relative one silently doubles.** The non-composable path
used absolute `node_ns` and was always immune — only the composable path was broken, which is
precisely why C1 passed at `use_composition:=False` last session.

After the fix (`namespace=namespace` → `namespace=node_ns`, 8 `ComposableNode` entries):

| Check | Before | After |
|---|---|---|
| nav2 node names | `/gosling1/gosling1/*` | `/gosling1/*` |
| lifecycle states | all 7 `unconfigured` | **all 7 `active [3]`** |
| `/gosling1/gosling1/*` topics | 15 | **0** |
| `global_costmap/costmap` | absent | **present** |
| container placement | `/gosling1/f1tenth_container` | `/gosling1/f1tenth_container` |

**C1 — PASS.** All 8 nav2 components verified *inside* `/gosling1/f1tenth_container` via both
`Loaded node ... in container ...` log lines and `ros2 component list` — placement, not merely
liveness, as the previous run log asked for. The BUG-017 failure mode (nav2 targeting a
`nav2_container` nothing creates) did **not** occur: bringup's `use_composition` /
`container_name` overrides reach the child correctly.

**C3 — inconclusive; BUG-009 did not reproduce.** Quiet host (load 2.15), nav2 fully active, and
nothing close to the original 94.1% of a core. But **per-node CPU is no longer measurable with
`ps`/`top` on the composed path** — all seven servers share one process (container total 48.6%).
Needs a different instrument, and still needs the VESC powered to test the BUG-010 hypothesis.

`ros2 lifecycle get /gosling1/controller_server` returned `Node not found` on the first attempt and
`active` on the next three — BUG-012 again, retry-fixes-it as always.

**C2 (nav goal) not run** — needs motion and RViz; left for the live session. Session C was left
running on the robot for exactly that.

### RTABMap, composed + namespaced: FAILED — no map in 88 minutes (BUG-021)

```bash
ros2 launch f1tenth_launch bringup.launch.py slam:=True launch_3d_mapping:=True \
  launch_2d_mapping:=False launch_navigation:=False use_gpu:=False \
  use_f1tenth_namespace:=True use_composition:=True reset_realsense:=True
```

`/gosling1/map` **NO DATA** for the entire run; `rtabmap: Did not receive data since 5 seconds!`
logged **1055 times, continuously from startup**. Root cause is a QoS reliability mismatch on
RTABMap's odometry input, confirmed twice:

```
ros2 topic info -v /gosling1/rtabmap/odom
  Publisher  : rgbd_odometry           Reliability: BEST_EFFORT
  Subscribers: rtabmap, rtabmap_viz    Reliability: RELIABLE
-> "New publisher discovered on topic '/gosling1/rtabmap/odom', offering incompatible QoS.
    No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS"
```

Incompatible QoS ⇒ DDS delivers nothing ⇒ the synchroniser never fires ⇒ no map node is ever
added. Not a camera fault: `rgbd_sync` starved only at startup and during an unrelated WiFi
outage (20 warnings total), so RGB-D was live for most of the run while odometry was dead
throughout. **Left unfixed pending a decision** on who should own RTABMap's odometry input —
see BUG-021 for the three candidates. Consequence worth stating plainly: **the primary mapper has
still never produced a map on the composed path.**

Also surfaced in this run:
- **BUG-022** — two `f1tenth_container` processes with byte-identical args. `ros2 component list`
  was empty (BUG-012), so `ps` was the only detector, exactly as with BUG-018. Not root-caused.
- **BUG-007 amendment** — the root-level duplicates `rtabmap_icp_odom` / `rtabmap_stereo_odom`
  were *actively warning* (~1030 each), so the "duplicate stack is inert" verdict does not
  generalise. Audit each member individually, per the BUG-019 lesson.

### BUG-012 — first measured evidence (and a caveat)

The RTABMap log contained **~330,000** lines of:

```
tev: ddsi_udp_conn_write to udp/<ip>:<port> failed with retcode -1
```

| Destination | Failures | What it is |
|---|---|---|
| `192.168.2.195` | 118,049 | **the Jetson's own `wlP1p1s0` address** |
| `192.168.2.194` | 85,198 | static `Peer`, not present |
| `192.168.2.193` | 85,000 | static `Peer`, not present |
| `192.168.2.141` | 85,000 | static `Peer`, not present |
| `192.168.2.140` | 85,000 | static `Peer`, not present |

**Caveat, stated up front: this was a bounded 87-second burst (16:50:55–16:52:22) coinciding with
a lab WiFi changeover, not steady state.** It therefore does *not* explain the continuous
introspection flakiness on its own. What it does do is turn suspect (a) from hypothesis into a
falsifiable test: because Cyclone is bound to `wlP1p1s0` with **no loopback in `<Interfaces>`**,
same-host traffic egresses over the radio — so a `ros2 param get` against a node in the *same
container on the same host* was disrupted by a WiFi event that should have been irrelevant to it.
**Next step: add loopback to `<Interfaces>` and confirm a WiFi disturbance no longer perturbs
same-host discovery — before touching `MaxAutoParticipantIndex` or `AllowMulticast`.**
The tight-loop baseline measurement described in the plan was not run this session.

### Still outstanding

1. **BUG-021 (RTABMap QoS)** — blocker for the primary mapper; needs an ownership decision.
   Also still untested at `use_composition:=False`.
2. **BUG-022** duplicate container, and the per-member BUG-007 liveness audit.
3. **Session D not re-run composed** — deferred again.
4. C2 nav goal, A5 live loop closure, A6 convergence — all need the car moving. Session C was
   left up on the robot for C2.
5. Checklist §1–3 (VESC actuation, joystick, safety mux) — need motor power + joystick.
6. **VSLAM SIGABRT (task 3): no new evidence.** It did not occur in either session today, so the
   single earlier non-reproduction remains weak evidence and the crash is still unexplained.
7. BUG-008, BUG-010 untouched. BUG-009 re-checked but now needs a per-node CPU instrument that
   works with composed containers.

---

## RUN LOG — 2026-08-04 (Session D: RTABMap QoS fix + TF ownership audit, agent-driven, gosling1)

Follows directly from the previous block. Addresses BUG-021 (the primary mapper's blocker), the
`odom→base_link` ownership question it raised, and the `use_composition:=False` gap.

Conditions: container `f1tenth_claude_test`, `ROS_DOMAIN_ID=77`, `docker restart` between every run,
`reset_realsense:=True` on every run. **VESC still unpowered** — `vehicle/vesc_odom` and
`vehicle/sensors/imu/raw` had NO DATA throughout, so every EKF number below comes from
`odom/rf2o` (8.8 Hz) + the two IMU inputs only. Robot stationary. Launch files deployed by
`scp` → `docker cp` (no rebuild, `--symlink-install`).

Note: the Session C stack left running for the C2 nav-goal test was killed by the first
`docker restart` of this session.

### BUG-021 — FIXED (option (a)), verified on both composition paths

`mapping.launch.py`: `qos_odom: "1"` → `"2"`. The mismatch was structural, not a typo — upstream
`rtabmap.launch.py:183` hands `rgbd_odometry` `"qos": qos_image`, and that one parameter drives both
its image subscriptions and its `odom` publisher, so the BEST_EFFORT we correctly want for the
RealSense propagates to `rtabmap/odom`. **Candidate (b) is therefore impossible** without breaking
the image input.

```
composed:      incompatible-QoS warnings 1055 -> 0 | rtabmap/odom 2.90 Hz
               /gosling1/map  NO DATA (88 min) -> 0.443 Hz
               rtabmap starvation warnings  1055 continuous -> 2 (startup only)
non-composed:  /gosling1/map 0.102 Hz, Publisher count 1, Memory.cpp forget() = adding signatures
               odometry/local 29.876 Hz | launch exceptions 0
```

**The primary mapper has produced a map for the first time on the composed path.**

Honest limit: the fix is verified on both paths, but the original failure was never *reproduced* at
`use_composition:=False` — the fix was already deployed by then. "Composition-independent" is
inferred from the wiring, not observed.

Warning attribution matters here. Of the residual `Did not receive data since 5 seconds` lines,
`rtabmap` itself accounts for **2**; the rest are `rtabmap_icp_odom` (55) and `rtabmap_stereo_odom`
(54) — the BUG-007 set, not the mapper.

### TF ownership audit — two edges had competing broadcasters (BUG-026, BUG-027)

Instrumented by `ros2 param get <node> publish_tf` on every `/tf` publisher, cross-checked against
per-edge rates counted off `/gosling1/tf`.

| Edge / topic | Before | After |
|---|---|---|
| `odom→base_link` | `ekf_odom_node` + `rgbd_odometry` (32.70 Hz ≈ 30 + 2.9) | **`ekf_odom_node` alone** |
| `map→odom` | `rtabmap` + `ekf_map_node` + `amcl` (25.2 Hz) | **`rtabmap` alone** |
| `/gosling1/map` publishers | **2** — `rtabmap` *and* `map_server` | **1** |
| `odometry/local` | 30.0 Hz | **30.000 Hz** (no regression) |

Two independent causes, both fixed:

- **BUG-026 — an overloaded flag.** `launch_localization`/`launch_local_localization` mean *"should I
  START localization?"*; `mapping.launch.py` read them as *"does odometry EXIST?"*. Because bringup
  starts the EKF itself it passes both `False`, so `enable_odom_here` was **always True under
  bringup** — the very reasoning that block exists for never fired on the path that matters. Fixed
  with an explicit `external_odometry` arg.
- **BUG-027 — no `slam` gate on global localization.** AMCL, the map EKF and `map_server` all ran
  while SLAM was building the map. A map saved during mapping could have come from the stale file.
  Fixed with `global_localization_effective` / `map_server_effective` in bringup.

Stationary, neither is very visible. Under motion two broadcasters on one edge disagree and the TF
buffer returns whichever arrived last — this would have corrupted the A5 loop-closure drive.

### BUG-028 — an empty lifecycle list aborts the whole launch (found by the composition:=False run)

Fixing BUG-027 turned off both `map_server` and `amcl`, which left `localization.launch.py` building
its **non-composed** lifecycle manager with `node_names=[]`. ROS 2 Humble rejects the empty tuple and
the exception took down every node in the launch:

```
[ERROR] [launch]: Caught exception in launch: Expected 'value' to be one of
[float, int, str, bool, bytes], but got '()' of type '<class 'tuple'>'
```

Worth recording as a **diagnostic trap**: what dominates the log is `realsense2_camera_node` dying
with a GLFW assertion (`_glfwPlatformGetTls`, exit `-6`) and a wall of `image_transport` plugin-load
failures. That is all shutdown cascade. The cause is one line ~120 lines earlier. The composable
manager was already guarded (`*([...] if lifecycle_nodes else [])`); the non-composed `Node` was not.
Same hazard already documented for Nav2, now shown to apply to localization too.

### Still outstanding

1. **BUG-022** duplicate `f1tenth_container` — not re-examined this session.
2. **BUG-007** — new evidence: the root duplicates subscribe to **root** `/lidar/scan_filtered`
   while sensors publish under `/gosling1/`, so they are permanently starved, not merely redundant.
   `ps` shows 2 at `ns=/` plus 2 at `ns=/gosling1` for the stereo/ICP pair. Still not fixed.
3. **BUG-012** — the loopback-`<Interfaces>` test and the tight-loop failure-rate baseline were
   **not** run this session. `ros2 param get` / `topic info` flakiness recurred as usual and retry
   fixed it every time, consistent with all previous sessions.
4. C2 nav goal, A5 live loop closure, A6 convergence — still need the car moving.
5. Checklist §1–3 (VESC actuation, joystick, safety mux) — still need motor power + joystick.
6. BUG-008, BUG-009, BUG-010 untouched. BUG-009 still needs a per-node CPU instrument for composed
   containers, and BUG-010 still needs the VESC powered.
7. **VSLAM SIGABRT: still no new evidence** (`use_gpu:=False` throughout). Noting one possible
   thread: the RealSense abort in the BUG-028 cascade was *also* a GLFW/TLS assertion — if the VSLAM
   abort ever produces a stack, check whether it shares that signature before assuming they differ.
8. **Two CLAUDE.md errors found** (see BUG-027): `launch_global_localization` defaults **True**, not
   False; and `localizer_amcl.yaml` has `tf_broadcast: true`, not false. Not yet corrected.

---

## RUN LOG — 2026-08-04 (Session E: BUG-022 root-cause + fix)

Scope: BUG-022 only. Robot gosling1, container `f1tenth_claude_test`, `ROS_DOMAIN_ID=77`, namespaced
(`/gosling1`), `use_gpu:=False`, VESC **unpowered** (so `vehicle/vesc_odom` and
`vehicle/sensors/imu/raw` had no data; `ekf_odom` ran on `odom/rf2o` + the two IMUs).

Command used for BOTH the before and after run:
```
ros2 launch f1tenth_launch bringup.launch.py slam:=True launch_3d_mapping:=True \
  launch_2d_mapping:=False use_composition:=True use_f1tenth_namespace:=True \
  use_gpu:=False reset_realsense:=True
```

### The duplicate was real, and it was two entire nav2 stacks
BEFORE (HEAD `f7d9967` launch files redeployed, so the baseline was *measured*, not inferred):

| evidence | result |
|---|---|
| `ps -eo args \| grep component_container` | **2** × `component_container_isolated` named `f1tenth_container` at `__ns:=/gosling1` (pids 80884, 81226) |
| `/proc/80884/maps`, `/proc/81226/maps` | full `libnav2_*.so` set (~60 libs) mapped in **BOTH** |
| `ros2 topic info /gosling1/local_costmap/costmap` | Publisher count **2** |
| `ros2 topic info /gosling1/global_costmap/costmap` | Publisher count **2** |
| `ros2 node list` / `ros2 component list` | showed only **ONE** container — both dedupe by name |

`LoadComposableNodes` resolves its target by **service name**; with two nodes offering
`/gosling1/f1tenth_container/_container/load_node`, both served the load request. So the system was
running two `controller_server`s, two `planner_server`s, two `bt_navigator`s and two lifecycle
managers. **BUG-019's lesson held: the duplicate was not inert.**

The original report's "byte-identical args" is wrong and worth correcting, because the difference is
the diagnostic: bringup's container carries two `--params-file` entries (nav2 params), mapping's
carries `-p thread_num:=6`.

### Root cause
`bringup.launch.py:712` creates `f1tenth_container`. Under `slam:=True` it includes
`mapping.launch.py` with `use_composition:=True`, and `mapping.launch.py:565` created its own
container named `container_name` (default `f1tenth_container`) guarded **only** by
`IfCondition(use_composition)`. The file had **no** `attach_to_shared_component_container` argument,
so no parent could tell it "I already made this". Both live in a `nodes_to_launch` group that pushes
the namespace, so both land at `/gosling1/f1tenth_container`.

Note what this was **not**: not `teleop.launch.py` (the suspected asymmetry in the handoff) and not a
consequence of BUG-007. This is the `cb3b567` pattern called out in the git-archaeology notes — a fix
applied to one composition path (`teleop.launch.py` got the attach guard in BUG-018) and never
carried to the sibling.

### Fix
1. `mapping.launch.py` — added `attach_to_shared_component_container` (default `'False'`, declared,
   registered in `launch_args`, performed to a string) and gated creation on a computed
   `creates_own_container = use_composition AND NOT attach_…`, applied to both the `GroupAction` and
   the inner `Node`. Same pattern `teleop.launch.py` has used since BUG-018. Standalone behaviour is
   unchanged.
2. `bringup.launch.py` — the mapping include now passes `attach_to_shared_component_container` **and**
   `container_name` explicitly, per the anti-leak rule (never rely on a child default for a generic
   arg name).

`mapping.launch.py`'s own teleop include still passes `attach_to_shared_component_container:
use_composition`, which is correct in both branches — `container_name` is created either by mapping
(standalone) or by bringup (attached). Only the stale comment there changed.

### AFTER — verified on-robot
- exactly **ONE** `f1tenth_container`; four uniquely-named containers, one process each
  (`f1tenth_container`, `command_gate_container`, `sensing_container`, `localization_container`)
- all **8** nav2 components loaded into the single container
- `local_costmap/costmap` and `global_costmap/costmap` Publisher count **2 → 1**
- `controller_server FollowPath.desired_linear_vel` = `0.5`, matching `nav2_params.yaml:130` — params
  still reach the servers
- `0` × `Caught exception in launch`, `0` × `process has died`
- `odometry/local` **30.020 Hz**; `/gosling1/map` publishing, Publisher count **1**

Two honest limits, stated rather than smoothed over:
1. `/gosling1/map` measured **~0.14 Hz** vs session D's 0.443 Hz. The robot was **stationary** —
   RTABMap only updates the grid as it adds signatures. Publisher count is correct and `rtabmap` is
   running with `-d`. Not a regression, but not an equal-conditions comparison either.
2. `/gosling1/rtabmap/odom` had **no data**. This is **expected** post-BUG-026: bringup passes
   `external_odometry:=True`, so no `rgbd_odometry` node is started at all (confirmed by `ps`) and the
   EKF owns `odom→base_link`. Session D's 2.90 Hz figure was measured under different wiring.

### Incidental observations (not acted on — out of scope)
- **BUG-009 lead:** two `planner_server`s planning against two `global_costmap`s is a far better
  explanation for the original 94% CPU than the BUG-010 EKF hypothesis. Re-measure BUG-009 first.
- **BUG-007 unchanged:** `ps` still shows `rtabmap_stereo_odom` and `rtabmap_icp_odom` at `__ns:=/`
  alongside the `/gosling1` pair. Untouched — that is TASK 2.
- **BUG-012 again:** `ros2 component list` returned empty on the first attempt of *both* runs and
  worked on retry. Retry before believing empty output.
- `/gosling1/cmd_vel` reported **Publisher count 14** with a mixed type list
  (`geometry_msgs/msg/Twist` *and* `TwistStamped`). Not investigated; flagged for a future session.
