# MPC bench bring-up — handoff (2026-08-06, ~18:15 EDT)

Session goal: bring gosling1 up for an **externally-owned MPC node** on
`/gosling1/drive`, on the bench, wheels off the ground, AC power. Achieved. The
MPC agent then ran its own checks and shut down; both stacks are now down and
the operator is rebooting the Jetson to put the car **on the ground on battery**.

## The one thing to do first

**`scripts/live_runs/71_mpc_stack.sh` and `scripts/live_runs/yaw_drift.py` are
untracked, and `config/localization/ekf_odom.yaml` is modified but uncommitted.**
This working tree lives on a OneDrive-synced path where untracked files have
gone missing before. Decide whether to commit before doing anything else.

```
 M config/localization/ekf_odom.yaml
?? scripts/live_runs/71_mpc_stack.sh
?? scripts/live_runs/yaw_drift.py
?? scripts/live_runs/MPC_BENCH_HANDOFF.md   (this file)
```

Branch `perf/config-tuning`, last commit `f4621d5`.

## RESOLVED 2026-08-06 ~18:32 EDT — the fix works, on the floor, on battery

Re-measured on the ground on battery, domain 42, car parked. **`odometry/local`
drift is gone:**

| window | EKF local (odom) | EKF global (map) | Isaac VSLAM | VESC IMU (unfused) |
|---|---|---|---|---|
| 60 s | **+0.04 °/min** | −0.41 | +0.06 | −14.71 |
| 90 s | **+0.01 °/min** | −0.21 | −0.01 | −14.82 |
| 60 s | **+0.17 °/min** | +0.17 | +0.20 | −14.06 |

Baseline was −13.94 °/min. Heading now tracks VSLAM. The VESC quaternion still
free-runs at ~−14 °/min and its z-gyro bias reproduced at +0.00435…+0.00441
rad/s — it is simply no longer fused for yaw.

**The open risk below did not materialise.** The RealSense z-gyro bias is real
(−0.00215 rad/s, −7.4 °/min) but `odom1` (VSLAM) and `odom2` (rf2o) hold the
heading against it. So **options 2 and 3 are not needed** — no `imu_corrector`
offset, and no Autoware `gyro_bias_estimator` (which the operator did not want
wired into this repo anyway).

Two things learned in the process:

- A window where *every* source agrees on a large drift means the car was
  **physically moved**, not that fusion broke. One window showed −560 °/min
  across the board with the VESC gyro std at 0.31 rad/s (parked ≈0.0015).
  Discard it and re-run.
- `yaw_drift.py` was subscribing to the odometry topics RELIABLE (a bare depth),
  which is an incompatible QoS match against `vehicle/vesc_odom`'s BEST_EFFORT
  publisher — that row printed `n=0`, indistinguishable from a silent topic.
  Fixed to use the BEST_EFFORT profile (**bug-133**).

The rest of this document is the pre-fix context, kept for the record.

## Original next task: re-measure yaw drift on battery

The operator is rebooting and reconnecting the battery specifically so this can
be re-measured. The config change below is **applied but never yet run**.

```bash
# host, BEFORE starting the container — or the RealSense dies (bug-130)
export DISPLAY=:0 XAUTHORITY=$HOME/.Xauthority
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge -
chmod 644 /tmp/.docker.xauth
setsid script -qfc "bash $HOME/bolus_ws/f1tenth_launch.sh" /dev/null \
    < /tmp/mpc_run.fifo > ~/mpc_container.log 2>&1 &

# verify INSIDE the container (env alone will NOT catch this)
ls -l /tmp/.X11-unix/X0 /tmp/.docker.xauth

# re-stage HEAD — the image ships an older f1tenth_launch (was d25c7fc)
#   from the dev box:  git archive --format=tar HEAD launch config scripts | gzip > head.tar.gz
#   scp to /mnt/f1tenth_ssd/shared_dir/handoff/, then inside the container:
cd /workspaces/f1tenth/src/f1tenth_launch && tar xzf /mnt/shared_dir/handoff/head_0806_mpc.tar.gz
grep -n "yaw" config/localization/ekf_odom.yaml | head   # confirm imu0 yaw is false

# bring up, then measure with the car PARKED
./71_mpc_stack.sh --domain 42 --map /mnt/shared_dir/maps/20260805/rtabmap_2d_final.yaml
python3 scripts/live_runs/yaw_drift.py 60
```

**Success criterion:** `EKF local (odom)` drift well under the −13.94 °/min
baseline. **It is not guaranteed** — see the open risk below.

## What was changed, and why

`config/localization/ekf_odom.yaml`: `imu0_config` yaw → `false` (roll/pitch
kept). Measured parked, 2026-08-06, two windows agreeing to 0.9 %:

| source | stationary yaw drift |
|---|---|
| VESC IMU orientation | −13.99 °/min |
| `odometry/local` | −13.94 °/min |
| `odometry/global` | −3.93 °/min |
| rf2o | +1.94 °/min |
| **Isaac VSLAM (VO)** | **+0.07 °/min** |

VESC z-gyro bias +0.00434 rad/s; RealSense z-gyro −0.00202 rad/s.
`vehicle/sensors/imu/mag` publishes 100 Hz of **identical zeros** — there is no
magnetometer, so the odom frame has no absolute yaw reference at all, and the
old config comment claiming the quaternion was "globally referenced" was wrong.
The IMU nevertheless advertises `orientation_covariance` yaw = 0.005, which is
why the EKF preferred it over VSLAM. Logged as **bug-129**.

**Do not "fix" this with a gyro offset.** The VESC computes that quaternion
onboard and the EKF reads no angular-rate entries from `imu0`, so an
`imu_corrector` offset changes nothing. (Offsets already sit unused in
`config/filters/imu_corrector.yaml`; `launch_imu_filter` is `'False'` in both
bringup and teleop.)

### Open risk on this change

`imu1` (RealSense) contributes **`vroll/vpitch/vyaw` rates, no orientation**.
With `imu0` yaw now off, yaw is integrated from the RealSense gyro — which
carries its own −6.94 °/min bias — fought by `odom1` (VSLAM, absolute-as-
increments) and `odom2` (rf2o, differential). Whether the result settles near
zero or merely drifts *less* is an empirical question. If drift is still
material, the operator has pre-approved options 2 and 3:

2. Fuse `vyaw` with a −0.00434 rad/s `imu_corrector` offset, `launch_imu_filter:=True`.
   Temperature-sensitive; will need periodic re-checking.
3. Autoware `gyro_bias_estimator` (already configured at
   `config/filters/gyro_bias_estimator.yaml`, unwired) for online estimation.

**Constraint on option 3:** the operator wants **Autoware removed as a
dependency** from this repo. IMU tooling will most likely become a separate
repo. Autoware experiments continue on bags (weekend work) — but do not wire new
Autoware dependencies into `f1tenth_launch`.

## Why this matters to the MPC

The MPC consumes **`odometry/local` with TF corrections**. The operator
explicitly rejected `odometry/global` and `amcl_pose` as discrete. So
`odometry/local` heading quality is a controller-critical property, not a
localization nicety — which is the whole reason the fix was applied before the
floor run rather than after.

## Verified-healthy baseline (bench, 2026-08-06, domain 42)

VESC `sensors/core` + `vesc_odom` 50.0 Hz · VESC IMU 99.9 Hz ·
`lidar/scan_filtered` 8.4 Hz · RealSense color 30.0 Hz · camera IMU 194.7 Hz ·
Isaac VSLAM 29.9 Hz · `odometry/local` 30.0 Hz · `odometry/global` 30.0 Hz ·
`command_gate/heartbeat` 13.9 Hz · AMCL lifecycle **active**, pose (0.148,
0.396), `tf_broadcast: False` (map EKF owns `map→odom`) · map 265×199 @ 0.05 m ·
all four TF edges present · `vehicle/ackermann_cmd` publishing clean zeros.

`amcl_pose` and `map` read 0 Hz — both are latched/event-driven. Assert AMCL's
**lifecycle state**, not a rate. `71_mpc_stack.sh` does this.

## Environment notes

- Bench run used `ROS_DOMAIN_ID=42`; a second agent's particle-filter bag replay
  was live on domain 0 at the time. Always check for other launches **from the
  host** (`ps -eo cmd | grep "[r]os2 launch"`) — a container's `ps` only sees
  its own PID namespace.
- The VESC does not enumerate until its **battery** is connected.
- Teardown: `docker stop` on the container is reliable; `pkill -INT` alone left
  38 processes alive. Verified 0 stray launches afterwards.
- Containers left running by **other** sessions at handoff time (not ours, not
  cleaned): `mpc_claude_0806`, `jetson_container_20260806_164732`,
  `jetson_container_20260806_174022`, `f1t_nav2_0806`. The reboot clears them.

## Still untouched

Nav2 has never driven the car — `testing_checklist.md` §9 unticked. See
`NEXT_CHAT_PROMPT.md` and `NAV2_OFFLINE_RESULTS.md`; that work is unaffected by
this session except that the `imu0` change alters `odometry/local` beneath it.
