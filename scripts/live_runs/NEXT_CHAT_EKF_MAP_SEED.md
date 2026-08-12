# Handoff — `ekf_map` unseeded, odometry closure error, and three carried-over items

**Written:** 2026-08-11 ~16:20 EDT · **Branch:** `perf/config-tuning` · **Robot:** gosling1
**Container at time of writing:** `jetson_container_20260811_145440` · `ROS_DOMAIN_ID=42`
**Stack:** raw `ros2 launch` per `DEMO_RUNBOOK_20260810.md` §4 (un-namespaced, CycloneDDS,
`localize_isaac_vslam_on_startup:=False`), map `20260805/rtabmap_2d_final.yaml`

The heading question is **closed** — see §0. Everything below is what that session turned up on the
way past, none of it chased.

---

## 0 · What was settled (do not re-litigate)

The parking spot is at **−84.5°**. `localizer_amcl.yaml` `initial_pose.yaw` was changed
**−1.3928 → −1.4748** on 2026-08-11 after five samples across two cold launches agreed to within
1.03°. Full table, method and the list of things deliberately left unchanged:
`DEMO_RUNBOOK_20260810.md` §5b. Buglog `bug-234`.

Bags kept at `/mnt/shared_dir/claude_heading_0811/heading_{A,B,C}` (raw `/lidar/scan_filtered`,
~20 s each, parked; B and C after rolling the car off the spot and back on).

**The robot is still running the old value.** The container gets `f1tenth_launch` from a tarball,
not git, so the running stack has −1.3928 until the package is re-staged and rebuilt. Verify with
`ros2 param get /amcl initial_pose.yaw` rather than assuming.

---

## 1 · `ekf_map` sat at identity while AMCL was correct  ← main item

**Measured 2026-08-11 ~15:11, on the gate before any recording:**

```
ros2 param get /ekf_map_node odom1   ->  visual_slam/vis/slam_odometry__NOT_FUSED   (guard engaged)
tf2_echo map base_link               ->  (-0.025, 0.002), yaw -0.30 deg             (identity)
ros2 topic echo /amcl_pose           ->  (0.5159, -0.4723), yaw -83.97 deg          (correct)
```

So AMCL had a good global fix and `ekf_map` — which owns `map→odom` under bringup
(`map_tf_publisher='ekf'`) — never took it. `map→odom` stayed at identity, putting the car ~0.7 m
and ~84° from where it actually was, in a stack where every individual node reports healthy.

**Why this matters more than it looks:** the runbook currently says the opposite. §4 states that
with the bug-232 guard engaged, `pose0 = amcl_pose` is the only global input and AMCL's own
`set_initial_pose: True` propagates unaided — "verified 2026-08-10 with **no seed at all**:
`map→base_link` came up at `(0.463, -0.602, -79.31°)`". That verification is real, and so is
today's contradiction. The same runbook already names the suspect:

> AMCL publishes that initial pose essentially once, so a startup race where `ekf_map` is not yet
> subscribed is plausible though not observed.

**It has now been observed.** Treat the seed step as required-but-intermittently-unnecessary until
this is root-caused, and do not delete the seed line from any script on the strength of the
2026-08-10 result.

Note the interaction with motion gating: AMCL is gated on `update_min_d`/`update_min_a`, so parked
it publishes its initial pose and then goes quiet. If `ekf_map` misses that single message there is
no second chance until the car moves. That is exactly the shape of a subscription race.

### Diagnostics, in order

Stack up, car parked, before touching anything:

```bash
ros2 daemon stop                                   # always, bug-233
ros2 topic info /amcl_pose --verbose               # subscriber count: is ekf_map_node even on it?
ros2 param get /ekf_map_node pose0                 # must be  amcl_pose  (not renamed by a guard)
ros2 param get /ekf_map_node pose0_differential    # expect False
ros2 param get /ekf_map_node pose0_relative        # expect False  (bug-104 — must stay False)
ros2 topic echo /amcl_pose --field header          # frame_id must be map; check the stamp is sane
ros2 run tf2_ros tf2_echo map odom                 # the actual broadcast; identity == the symptom
```

Then the discriminating test — **does a late `amcl_pose` get taken?**

```bash
# force AMCL to run a laser update by moving the car ~0.5 m and back,
# then re-read map->base_link. If it snaps to the right pose, it is a startup
# race (ekf_map subscribed after AMCL's one-shot publish). If it stays at
# identity, ekf_map is rejecting the measurement and the Mahalanobis gate
# (pose0_rejection_threshold: 2.0) against a 2.0 m initial covariance is
# the next thing to look at.
```

And confirm the manual seed still works as the escape hatch:

```bash
cd /workspaces/f1tenth/src/f1tenth_launch/scripts/live_runs
python3 seed_initialpose.py --ns "" --no-use-sim-time --x 0.445 --y -0.575 --yaw -1.4748
```

### Candidate fixes, none applied

- A first-fix forwarder that re-publishes the localizer's pose onto `initialpose` once `ekf_map` is
  subscribed. Buglog `bug-111`'s RESIDUAL note already anticipates needing this for the
  `particle_filter` auto-global-init case — same gap, different trigger.
- Have AMCL re-publish its initial pose a few times at startup rather than once.
- Latch/transient-local QoS on `amcl_pose`, if upstream allows it.

Whichever it is, the acceptance test is a **cold launch with no seed**, repeated enough times to
show the race is gone — one clean launch proves nothing, which is the lesson from 2026-08-10.

---

## 2 · Odometry closure error over a short out-and-back — first moving data point

Rolling the car off the parking spot and back onto its marks (tile lines + sticker, so the physical
return was within ~1.5 cm and ~0.12°, confirmed independently by the scan fit landing on the same
grid cell) left:

```
odom->base_link  before:  (-0.025,  0.002)  yaw  -0.30 deg
odom->base_link  after:   ( 0.318,  0.225)  yaw  -5.40 deg
```

**0.39 m and 5.1° of accumulated error on a round trip that physically closed.** The path was hand-
rolled and its length was not measured, so this is an observation, not a number to tune against —
but it is the first non-parked odometry data this vehicle has on record, and every parked
measurement says the stack is healthy (`odometry/local` −0.05 °/min, per-source drift ≤0.34 °/min).

This is precisely the gap `DEMO_RUNBOOK_20260810.md` §3 flags: *"Every odometry number this vehicle
has on record was taken parked. A scale error in `speed_to_erpm_gain`, the 2.4 % wheelbase yaw-rate
bias in `vesc_odom`, and an rf2o whose zero-velocity gate latches on under motion are all invisible
parked."* Hand-rolling adds a fourth candidate the runbook does not: with the motor unpowered the
VESC still counts ERPM, but the rf2o zero-velocity gate and any control-input path see nothing.

**Do it properly rather than chasing this number.** Run the §3 procedure: tape both rear-wheel
contact patches, one straight ~4 m leg, bag it, then

```bash
python3 …/scripts/live_runs/odom_moving_check.py "$(cat "$BAG_ROOT/.last_bag")" --tape 4.00 --tape-yaw 0
```

Read health → scale vs tape → heading, in that order. Stage `odom_moving_check.py` and
`analysis/check_map_frame.py` together first; the tarball carries neither.

---

## 3 · Carried over, not started

**LUCIO notification — WRITTEN 2026-08-11, not yet sent.**
`scripts/live_runs/LUCIO_MAP_HEADING_NOTICE.md`. Covers the heading change, the method, the
five-sample table, and the waypoint-0 problem (−92.08° scores 62–64 % against the map in every
sample; anything seeding from it starts ~7.7° wrong, and its "seeded from waypoint 0, verified
19 mm from waypoint 0" self-check is circular and cannot detect that). Two asks are open in it: is
their pipeline sensitive to the ~6 cm x/y question, and do they have an independent reason to
believe waypoint 0. **The operator sends it, not the agent.**

**Are the archived bags' seeds wrong too?** `51_localize_offline.sh`, `scripts/analysis/check_map_frame.py`,
`MAP_BUILD_HANDOFF.md`, `BRIEF_PARTICLE_FILTER.md` and `LOCALIZER_FOLLOWUPS.md` all carry
`(+0.445, −0.575, −79.82°)` as the 2026-08-05 bags' start pose, read from the RTABMap database's
optimized poses. Either those bags really started 4.7° from where the car parks today, or that
ground truth shares the error the AMCL seed had. **One command decides it** — run
`heading_from_scan.py` against the first seconds of an archived bag (it takes any bag with a
`LaserScan` topic). Left alone deliberately: changing those seeds without the measurement would
desynchronize each replay from its own data.

**IMU bias remover — assigned to the next session.** `realsense_d435i.launch.py:363` hardcodes
`'remove_imu_bias': 'False'` and its "node not installed" comment is stale (`imu_processors` was
built on the robot 2026-08-09). `imu_filter.launch.py` already defaults the arg `True`, so that one
hardcoded string is the whole blocker. Parked measurement says the RealSense bias
(−0.002208 rad/s) is real but drives nothing while rf2o and VSLAM hold fused yaw to −0.05 °/min —
so this needs a deliberate before/after with `yaw_drift.py`, not a flip. **The operator reports
internet access is back on the Jetson as of 2026-08-11**, which is why this is now actionable;
re-verify that before planning around it, and note `/workspaces` is still a container layer either
way.

---

## 4 · The `localize_isaac_vslam_on_startup` default — APPROVED AND APPLIED 2026-08-11

`localize_isaac_vslam_on_startup` now defaults **`False`** in both `bringup.launch.py` (was line 76)
and `teleop.launch.py` (was line 58). All five entry points that carry the arg now agree:

```
bringup.launch.py                          False   (changed 2026-08-11)
teleop.launch.py                           False   (changed 2026-08-11)
mapping.launch.py                          False
localization/localization.launch.py        'False'
nvidia_isaac_ros/…_visual_slam_realsense   False
```

With it `True` the bug-232 guard (`use_gpu AND localize_on_startup`) evaluated True while the VSLAM
node ran unlocalized from its own power-up origin, so `ekf_map` pinned `map→odom` to that origin —
72.3° of error, rotating car, footprint, live scan and 3D cloud inside a perfectly good map with
every node reporting healthy. Forgetting the arg was the whole failure mode.

`teleop.launch.py` was included in the change without being named in the approval, because it is an
independent entry point that includes localization directly — leaving it `True` would have
reintroduced the identical bug through the other door. **Both `DeclareLaunchArgument` descriptions
already read "False (default)"**, so the docstrings had been describing the intended behaviour all
along and only the code disagreed.

Anyone who genuinely has a saved VSLAM map co-registered with the nav map must now opt in with
`localize_isaac_vslam_on_startup:=True`. The §4 launch block in `DEMO_RUNBOOK_20260810.md` still
passes `:=False` explicitly; that is now redundant but harmless, and worth keeping as documentation
of intent.

**Already changed this session (disclosure, not a request):** `DEMO_RUNBOOK_20260810.md` §5b was
rewritten from "Proposed, NOT yet applied" to the confirmed result with the five-sample table; the
§4 seed command lines and the §4 gate's expected `map→base_link` moved from −79.8° to −84.5°; the
waypoint-0 gotcha in §5 was reworded. Plus `localizer_amcl.yaml`, `ekf_map.yaml` (comment),
`CLAUDE.md`, `NEXT_CHAT_ODOM_ROTATION.md` (seed line) and `heading_from_scan.py` (candidate table
now carries both the old and new seed as labelled rows).

---

## 5 · Gate that was clean, so you can skip re-deriving it

```
ros2 topic list | grep -c gosling1   ->  0        (raw launch, bare topics — no /gosling1 prefix)
ros2 param get /ekf_map_node odom1   ->  ...__NOT_FUSED     (bug-232 guard engaged)
ros2 topic hz /lidar/scan_filtered   ->  8.38 Hz, max gap 0.128 s
```

**8.4 Hz is correct for this sensor on this machine**, not a fault — the runbook's "~10–12 Hz" is
optimistic; CLAUDE.md's YDLidar X4 note ("observed ~8 Hz on Jetson Orin Nano due to USB/CPU
overhead") is the accurate one. Worth reconciling in the runbook at some point.
