# Handoff: heading-drift fixes, then live Nav2 validation

Written 2026-08-08. Branch `perf/config-tuning`. Separate thread from
`NEXT_CHAT_PROMPT.md`, which covers actuator system-identification — don't
confuse the two.

**gosling1 is powered down.** The operator switches it on. Job A can be prepared
offline; Jobs B and C need the robot.

## What landed on 2026-08-08

| commit | change |
|---|---|
| `c4c3a9c` | EKF `odom2` (rf2o) Mahalanobis gates 5.0 → 3.0 |
| `d0b7ebb` | `docs/rf2o_zero_velocity_brief.md`, `docs/imu_yaw_bias_notes.md` |
| `dc40931` | `RESET_REALSENSE` opt-in (was forced `True` in all 8 run scripts) |
| (this one) | `MAX_STEERING` 0.25 → 0.314 |

Also: bug-170 (RealSense USB wedge, fixed by an unbind/rebind from inside the
privileged container) and bug-171 (intermittent missing colour stream, open).

Live state at last shutdown, all green: `vehicle/sensors/core` 50.4,
`imu/raw` 100.0, `scan_filtered` 8.5, `odometry/local` 30.0, `odom/rf2o` 8.35,
`camera/color` 30.2, `visual_slam` 30.0 Hz. AMCL active, four TF edges,
`odometry/local` parked drift −0.03 to +0.16 °/min.

---

## Job A — RealSense gyro bias, then the VESC (start here)

Read `docs/imu_yaw_bias_notes.md`. Everything is measured; nothing is applied.

The correction that reorders this work: **`imu1`'s `vyaw` is already enabled and
fused uncorrected** (`config/localization/ekf_odom.yaml:226`), so the RealSense is
the IMU biasing yaw today — not the VESC, whose angular-rate entries are all
`false`. Do the RealSense first: it is one constant on a path that already exists.

The operator has approved doing the VESC as well. Do them **sequentially with a
measurement between**, not together — they are not separable afterwards.

Verification is `scripts/live_runs/yaw_drift.py 60`, parked, **twice per state**.
One run cannot distinguish a bias from a random walk that landed small — that is
precisely how rf2o's drift was misread until its sign flipped between runs.
`odometry/local` is currently −0.03 to +0.16 °/min and **must not get worse**;
improving an input while degrading the fused output means the change fought a
source that was already compensating.

## Job B — confirm the EKF gate change (rides along with Job A)

`odom2`'s gates went 5.0 → 3.0. Frame this as a **no-regression check, not a fix
confirmation**: the gate cannot address rf2o's stationary walk, because every step
in that walk is well inside the declared covariance and the gate never fires on it.
See the reasoning committed in `ekf_odom.yaml`.

The real risk is the opposite one — that 3.0 is now too tight and rejects
legitimate rf2o updates during turns, when deltas are largest. So the **moving**
run matters more than the parked one. `yaw_drift.py` already reports rf2o, so
Job A's parked runs cover the stationary half for free; add a driving pass and
watch whether rf2o's contribution disappears mid-turn.

The actual fix for the walk is upstream — the operator has briefed a separate
agent with `docs/rf2o_zero_velocity_brief.md`. Don't reimplement it here.

## Job C — live Nav2 control and perception

Obstacle avoidance and plan completion, on the floor. Nothing closed-loop has ever
been validated: prior `SUCCEEDED` results came from bag replays where the bag drove
the pose, so they only prove the recorded trajectory passed inside the goal
tolerance.

**Prerequisites, in the order they bite:**

1. `launch_twist_to_ackermann:=True`. It defaults `False` because the MPC publishes
   `drive` directly. Without it Nav2's `cmd_vel` never reaches the vehicle, and
   everything looks healthy while the car sits still.
2. Re-apply the twist fix to the container: `/mnt/shared_dir/apply_twist_fix.sh
   <container>`. `/workspaces` is a container layer and the Jetson has no internet,
   so every fresh container starts from pre-fix source where right turns steer hard
   left (bug-140).
3. Gate the servers on lifecycle **ACTIVE**, not on topics or action servers
   existing — those appear at CONFIGURE, and goals are rejected while inactive with
   nothing logged anywhere (bug-126).
4. Goals must be **poses**. Rows from `maps/*/truth_<bag>.csv` are the `map→odom`
   transform and land on top of the robot (bug-128). Use
   `scripts/live_runs/goal_poses_from_bag.py`.

**On `log_level:=info`, which the operator specifically wants.** The motivation is
sound: at `warn` a previous session could not see why the controller stopped shy of
the goal. Be aware that **no diagnosis of that shortfall exists** in this repo's
buglog or in the assistant memory — it was not recorded, so treat it as an open
question to investigate afresh, not a known issue to confirm. The nearest recorded
item is bug-140 (wrong steering direction on the first live goal, fixed
2026-08-06), which could plausibly have produced a shortfall but was never
connected to it.

Before running at `info`, do both of these:

- **`DDS_PROFILE=lo`** — kills the CycloneDDS peer spam at the source. The cost is
  that nothing off-robot sees the topics, and it fails silently (remote RViz shows
  an empty world with no error). Fine for a local-only run; wrong if the operator
  wants remote RViz.
- **Log to the SSD**, e.g. `/mnt/f1tenth_ssd/shared_dir/logs/`. At `warn`, five
  bring-ups wrote 1.16 GB into the Jetson's home directory and took `/` to 100 %,
  0 bytes free. `info` will be worse.

Also check `visual_slam/tracking/odometry` **by rate** at the start of every run
and after any anomaly: VSLAM aborts on roughly one launch in three, does not
respawn, and fails silently because `odometry/local` stays at 30 Hz on the
remaining sources. Mid-run, that quietly hands heading to rf2o's walk.

---

## Operating notes that cost time on 2026-08-08

- **Tear down with a script piped on stdin**, matching on *paths*
  (`/opt/ros/|/workspaces/f1tenth/install/`) plus `ros2 launch|component_container`,
  excluding PID 1, TERM then KILL, then print survivors. A pattern matching only
  node names missed 11 orphans (`joy_teleop`, `static_transform_publisher`), and
  the resulting load average of 8.24 on 6 cores starved rf2o to 2.5 Hz and the
  LiDAR to 6.5 Hz. **Low-but-nonzero sensor rates can be CPU contention, not a
  sensor fault — check `uptime` first.**
- A `pkill -f` typed inside `docker exec bash -lc "..."` matches the exec'd shell's
  own command line and kills itself (exit 143).
- The container is `AutoRemove=true`; staged source is at
  `/mnt/f1tenth_ssd/shared_dir/f1tenth_launch_01dc83d.tar.gz`. `--symlink-install`
  means extracting over `src/` is the whole deployment — but re-grep the values
  afterwards, since a stale tarball reverts committed config silently.
- Camera at 0 Hz is usually X11/GL, but not always: on 2026-08-08 the display
  checks all passed and it was a stuck USB state after `initial_reset`. Fix is an
  unbind/rebind of the device from inside the privileged container (host `sudo` is
  not passwordless). QoS is worth ruling out with `--qos-reliability best_effort`.
