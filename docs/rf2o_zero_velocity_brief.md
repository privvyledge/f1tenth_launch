# Brief: zero-velocity detection for `rf2o_laser_odometry`

**For:** whoever owns the `rf2o_laser_odometry` fork.
**From:** the `f1tenth_launch` side, which consumes `odom/rf2o` as EKF `odom2`.
**Status:** proposed, not implemented. Nothing in `f1tenth_launch` depends on it yet.

---

## The problem, measured

On a **stationary** car, `odom/rf2o` accumulates heading error. Two back-to-back
60 s windows on gosling1, 2026-08-08, car parked and powered, nothing touching it:

| window | rf2o yaw drift |
|---|---|
| A | **−5.58 °/min** |
| B | **+4.60 °/min** |

For reference, in the same windows: Isaac VSLAM +0.16 / −0.04 °/min, and the fused
`odometry/local` +0.16 / −0.03 °/min.

**The sign flips between runs.** That is the whole diagnosis: this is not a
calibration bias, a mounting error, or a systematic offset — those reproduce with a
sign. It is a random walk in the scan-matching solution, and the node integrates it
because it has no notion of "not moving".

Per step the noise is small and entirely in-family: ~5° accumulated over ~500
updates is σ ≈ 0.22 °/step, against the 2.86 °/step the node already declares in
`pose_covariance_diagonal[5]`. That matters because it rules out the two fixes a
consumer can apply on its own:

- **A tighter Mahalanobis gate downstream does not help.** Every step is well
  inside the declared covariance, so an outlier gate never fires. (We tightened
  the EKF's `odom2` gates 5.0 → 3.0 anyway, for spike protection — it does not
  touch this.)
- **Inflating the declared covariance does not help either.** It would slow the
  walk's influence, not stop it, and it would also degrade the source when the car
  *is* moving, which is when we actually want it.

The only fix is to **not integrate the delta when the car is not moving**, and that
has to happen inside rf2o, where the per-scan delta is produced.

## Why this matters to us

`odom/rf2o` is the EKF's yaw source of last resort. Today yaw is carried by Isaac
VSLAM (`odom1`) with rf2o (`odom2`) alongside, and the VESC IMU's yaw is disabled
outright (its onboard Madgwick free-runs at ~−15 °/min with no magnetometer). VSLAM
aborts on roughly one launch in three and does not respawn. When that happens rf2o
*is* the heading, and a ±5 °/min walk becomes the vehicle's heading error.

## What we're asking for

A zero-velocity gate that suppresses the pose delta when consecutive scans indicate
no motion.

### Constraint: keep it self-contained

**Do not make this depend on a wheel-odometry or VESC topic.** That hardware is
external to the package, differs across vehicles, and would make rf2o unusable
standalone. The detector should work from the laser stream alone, which is
sufficient — two consecutive scans from a stationary sensor are near-identical, and
that is directly measurable.

If you want a second opinion signal, take it as an **optional** input:

- a parameter like `zero_velocity_twist_topic` (default `''` = disabled), accepting
  any `TwistWithCovarianceStamped`/`Odometry`, used only to *confirm* a
  scan-derived stationary decision, never as the sole trigger;
- when unset, behaviour is scan-only and identical on every robot.

### Suggested detection

Scan-derived, in rough order of preference:

1. **Residual of the fitted motion.** rf2o already solves for a
   (Δx, Δy, Δθ) per scan pair. Declare stationary when the solved translation and
   rotation are both below thresholds *and* the range-flow residual is consistent
   with sensor noise rather than real motion. This reuses what the solver computes.
2. **Direct scan similarity.** Mean absolute range difference between consecutive
   scans over valid beams, thresholded. Cheap, independent of the solver, and a
   good cross-check on (1).

Suggested parameters (names are yours to pick):

| parameter | purpose | suggested default |
|---|---|---|
| `enable_zero_velocity_detection` | master switch | `true` |
| `zero_velocity_linear_threshold` | m/s below which translation counts as still | ~0.02 |
| `zero_velocity_angular_threshold` | rad/s below which rotation counts as still | ~0.01 |
| `zero_velocity_hold_scans` | consecutive scans required before latching | 3 |
| `zero_velocity_twist_topic` | optional external confirmation | `''` |

The `hold_scans` hysteresis matters — latch into and out of the stationary state
over several scans so a single noisy pair can't chatter the gate on and off.

### Behaviour while gated

Please **keep publishing** on `odom/rf2o` at the normal rate, with the pose held
and the twist set to exactly zero. Do not stop publishing.

The consumer-side reason: our EKF treats this source as `differential: true`, and
a source that goes silent is indistinguishable from a dead node. Worse, on this
vehicle a topic that exists but publishes nothing is the failure mode that has
hidden several bugs, so our health checks assert *rates*, not presence — a gated
rf2o that went quiet would trip them on every parked start-up.

Inflating the covariance while gated instead of zeroing the twist is also fine, and
arguably cleaner, if you'd rather express "no information" than "known zero". Either
is easy for us to consume; silence is not.

### How to verify

`scripts/live_runs/yaw_drift.py 60` in this repo, car parked, reports per-source
stationary drift including an `rf2o LiDAR odom` row. Success is that row dropping
from ±5 °/min to well under 1 °/min across at least two runs — **two runs matters**,
because a single run cannot distinguish a fixed bias from a walk that happened to
land small.

Please also check a moving run for the opposite failure: the gate must not latch
during slow creeping motion. A slow constant-speed pass is the case to watch, since
that is where a stationary threshold is most likely to be wrongly tripped.

---

# Test environment

## Start with a bag, not the robot

For this change a recorded bag is **better** than live hardware, not a fallback:
the stationary case is exactly reproducible, you can A/B the gate on identical
input, and the failure you are fixing is a *random* walk — on live hardware two
runs of the same code give different numbers, which is what made it hard to
diagnose in the first place.

Bags live on the robot's SSD, which is the only place large data belongs (`/` on
the Jetson is a 28 GB SD card that runs ~96 % full):

```
/mnt/f1tenth_ssd/shared_dir/bags/20260804/
/mnt/f1tenth_ssd/shared_dir/bags/20260805/     # mapping_drive_170025, figure8_172338
/mnt/f1tenth_ssd/shared_dir/bags/20260806/
/mnt/f1tenth_ssd/shared_dir/bags/claude_laser_filtered/
```

**Verify before trusting this list** — it was read on 2026-08-08 and the contents
were not opened; `scripts/live_runs/90_inspect_bag.sh` will tell you what topics
and duration a bag actually has. You need `lidar/scan_filtered` (rf2o's input,
`laser_scan_topic`), and for the stationary test a segment where the car is parked
— the start of any drive bag, before the operator takes the controller, is usually
several seconds of exactly that.

Replay with `use_sim_time:=True`. The topics are namespaced `/gosling1/...`.

## If you do need the live robot

**gosling1 is powered down as of 2026-08-08** — the operator has to switch it on.
The VESC does not enumerate until its *battery* is connected; USB power alone is
not enough, and you will find `/dev/sensors/vesc` missing with no other symptom.
For rf2o work you only need the LiDAR (`/dev/ydlidar` → `ttyUSB0`), not the VESC.

```bash
ssh gosling1
```

Start the container with the **operator's script only** — never hand-build a
`docker run` from `docker inspect`, which silently drops the X11 and USB flags:

```bash
export DISPLAY=:0 XAUTHORITY=$HOME/.Xauthority   # or the RealSense dies
xhost +local:
mkfifo /tmp/rf2o.fifo
setsid bash -c "exec sleep infinity > /tmp/rf2o.fifo" </dev/null >/dev/null 2>&1 &
setsid script -qfc "bash $HOME/bolus_ws/f1tenth_launch.sh" /dev/null \
  < /tmp/rf2o.fifo > /mnt/f1tenth_ssd/shared_dir/logs/rf2o_container.log 2>&1 &
docker ps          # it self-names jetson_container_<date>_<time>
```

The container is `AutoRemove=true` — it is destroyed on stop and `/workspaces` is
a container layer, not a bind mount, so anything you build there is lost on
restart. The Jetson has **no internet**, so `git fetch` will not work inside it;
stage source by dropping a tarball on the SSD (which is bind-mounted at
`/mnt/shared_dir`) and extracting it over the target directory. This repo's
current HEAD is staged at
`/mnt/f1tenth_ssd/shared_dir/f1tenth_launch_01dc83d.tar.gz`.

Bring the stack up with `scripts/live_runs/71_mpc_stack.sh` (localization on, Nav2
off) or, if you only want the LiDAR, `10_preflight.sh` then a sensors-only launch.

## Four things that will cost you an hour each

1. **Log to the SSD, never to `~`.** Five bring-ups at `log_level:=warn` wrote
   1.16 GB to the Jetson's home directory on 2026-08-08 and took `/` to 100 %,
   0 bytes free. It is almost all CycloneDDS spam from a powered-off static peer
   (~200 failed `sendto()`/sec *per process*). Set `DDS_PROFILE=lo` for local-only
   work and the noise disappears entirely — the cost is that nothing off the robot
   can see the topics, and it fails silently.

2. **Pick a non-colliding `ROS_DOMAIN_ID`** (e.g. `export ROS_DOMAIN_ID=43`) and
   check for other stacks *from the host*, not from inside a container — a
   container's `ps` sees only its own PIDs. Multiple agents share this robot.

3. **Tearing down a launch orphans its nodes.** `kill -INT` on the launch is
   unreliable without a controlling terminal, and a kill pattern matching only
   `ros2 launch|component_container` misses `joy_teleop` and
   `static_transform_publisher`. On 2026-08-08 eleven orphans survived a teardown
   and the resulting CPU contention pushed load average to 8.24 on 6 cores — which
   starved rf2o down to **2.5 Hz from its configured 10** and the LiDAR to 6.5 Hz.
   After a clean teardown they were 8.3 and 8.5 Hz.

   **This matters directly to you:** if you measure rf2o's rate and find it far
   below `freq`, check `uptime` before concluding anything about the algorithm.
   That was a machine-load artefact, not an rf2o defect.

4. **A topic that exists proves nothing.** Assert *rates*, never presence — a
   dead driver with a live subscriber still resolves and reports a type. Use
   `scripts/live_runs/00_env.sh`'s `require_rate` helper, or `ros2 topic hz`.

## Consumer-side context

`f1tenth_launch` consumes `odom/rf2o` as EKF `odom2` in
`config/localization/ekf_odom.yaml` — `differential: true`, `relative: false`,
x/y/yaw only. Its Mahalanobis gates were tightened 5.0 → 3.0 on 2026-08-08; that
gates outliers and does not interact with the change proposed here. Node
parameters are set in `launch/localization/localization.launch.py` (~line 805),
not in a YAML.
