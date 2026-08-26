# What the workspace-build repo needs to change

**Audience:** whoever owns the Dockerfile / `.repos` for the robot image. Nothing
in this document is actionable inside `f1tenth_launch` — it is a pure launch
configuration package and deliberately gains no build dependencies of its own.

Written 2026-08-09. Everything below was verified on gosling1 against
`privvyledge/f1tenth:humble-devel-08052026`, not read from documentation.

> **Revised 2026-08-26.** Three of its framing assumptions have expired, and the
> corrections are inline below rather than rewritten over, so the original
> reasoning stays auditable:
> - **The Jetson has internet** (since 2026-08-24). Workarounds *can* be
>   re-fetched on the robot. That removes the urgency, not the ask — a container
>   layer still dies with the container.
> - **Item 1's apt install would now be a DOWNGRADE.** See the correction there.
> - **The root filesystem is no longer 96 % full** — gosling1 was reflashed to
>   L4T R36.4.3 on 2026-08-24 and the M.2 is root, with ~775 G free.
>
> The live image is now `privvyledge/f1tenth:humble-devel-08092026`.

## Why this exists

Three things the running stack needs are missing from the image, and each has
been worked around inside a **container layer**, which `/workspaces` is. Every
one of those workarounds is destroyed on container restart. (~~and because the
Jetson has no internet none of them can be re-fetched on the robot~~ — the
no-internet half was retired 2026-08-24; re-fetching is possible now, it is just
manual work repeated on every restart.) The workarounds are re-applied by
`/mnt/f1tenth_ssd/shared_dir/rf2o_zv_0809/prep_container.sh`; they should stop
being workarounds.

## 1. `ros-humble-imu-pipeline` — add to the apt list

```dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-imu-pipeline
```

> **CORRECTED 2026-08-26 — do not add the apt package.** The Humble *binary* is
> **0.4.1**; **0.5.2 is the source tag**. The image already ships 0.5.2 built from
> source, so `apt install ros-humble-imu-pipeline` would be a downgrade. What this
> section should ask the build repo for is: **keep building the 0.5.2 tag from
> source** until upstream releases >= 0.5.2 into Humble, then switch. Re-confirmed
> from the robot with working internet: candidate
> `0.4.1-1jammy.20260804.210108`.

**Verified 2026-08-09:** `imu_processors` source tag **0.5.2**.
The 0.5.2 tag builds `imu_bias_remover` and registers both an executable
`imu_bias_remover_node` and a composable component
`imu_processors::ImuBiasRemover` — confirmed by building that exact tag from
source in the container and running `ros2 pkg executables` / `ros2 component
types`.

This replaces Autoware's `imu_corrector`, which is **not installed in any robot
image** (checked the full `install/` tree of both `humble-devel-08052026` and
`humble-latest`) and was switched off at every call site anyway. It moves the
dependency to `ros-perception`, the same ecosystem as the `imu_filter_madgwick`
the chain already uses.

**Why it is needed:** `imu_filter_madgwick` is a pass-through for
`angular_velocity` — it writes only `orientation` — so nothing in the running
chain corrects the gyro. The RealSense's stationary z-gyro bias is
**−0.00214 rad/s (−7.4 °/min)**, stable to 1.2 % across five 60 s measurements
over two days, two container instances and a system-clock correction. That bias
is fused into vehicle heading today via `ekf_odom.yaml`'s `imu1` `vyaw` row.

**Interim state:** built from source in the container from
`/mnt/f1tenth_ssd/shared_dir/rf2o_zv_0809/imu_pipeline_0.5.2.tar.gz` (the
upstream 0.5.2 tag with `.git` stripped), and **baked into
`humble-devel-08092026`** — `ros2 pkg executables imu_processors` lists
`imu_bias_remover_node` in that image today. ~~The apt package is preferred —
this tarball exists only because the Jetson cannot reach the network.~~
**Corrected 2026-08-26: the source build is not a stopgap here, it is the
correct route**, because apt Humble carries only 0.4.1. Keep building the tag.

**Not yet enabled — but the blocker has changed, and it is no longer yours.**
`remove_imu_bias` stays `'False'` at both call sites. ~~until the image carries
the package~~ — the image *does* carry it. The chain was wired and proven
end-to-end on 2026-08-09 (`camera/imu` 200.6 Hz → `camera/imu/bias_removed`
200.3 → `camera/imu/filtered` 200.1, `bias` publishing), and the correction
itself was verified offline on 2026-08-26: the estimate reproduces the measured
−0.00214 rad/s to 3.1e-05 and the subtraction is bit-exact.

**UPDATED 2026-08-26 — the blocker is closed, and this item now does ask you for
something.** What had been blocking it was a design decision on our side: the
node's stationary test is `twist_is_zero_ || odom_is_zero_` with **no staleness
timeout**, so a velocity source that dies while reading "stopped" pins
`angular_velocity` at zero indefinitely — measured at 3996 consecutive samples
with the raw gyro live (bug-251). That is now fixed in a fork.

Confirmed absent in three independent places before forking, so this is not a
version-skew story: the source-built **0.5.2** in the image (read end to end — no
`rclcpp::Time` stored, no timer), the **apt 0.4.1** `.deb` (the string `timeout`
appears in no file in the package), and upstream's **`ros2` branch at 0.6.1**
(same condition, same six parameters). The only change between 0.5.2 and 0.6.1 in
that file is a `use_stamped` default flip.

### What to change

Replace the tarball with a `.repos` entry, so the fix survives a container
rebuild instead of being re-applied by hand:

```yaml
  imu_pipeline:
    type: git
    url: https://github.com/privvyledge/imu_pipeline.git
    version: humble-devel
```

That branch is upstream tag **0.5.2** — the version this vehicle has been
validated against — plus one commit adding a **`stationary_timeout`** parameter
to `ImuBiasRemover`. Each velocity source records when it last published; a
source silent for longer than the timeout has its "stationary" verdict dropped,
and the node falls through to subtracting the last converged bias rather than
zeroing. **The default is `0.0`, which disables the check and reproduces stock
behaviour exactly**, so adopting the fork changes nothing until a config asks for
a timeout.

This supersedes both the apt line at the top of this section and the
`imu_pipeline_0.5.2.tar.gz` interim route. It does not change the "do not use
apt" conclusion — apt Humble is still 0.4.1.

### How it was verified

Same recorded gyro, same harness, same scoring script, only the parameter
differing (`scripts/live_runs/run_bias_test.sh`, phase D):

| `stationary_timeout` | behaviour after the velocity source dies |
|---|---|
| `0.0` (stock) | **3996 samples pinned at exactly 0.0**, raw gyro live to 0.037 rad/s |
| `1.0` (fork) | pinned **0.986 s** (195 samples), then recovered; all **3801** remaining samples exactly `raw − bias`, max residual **0.0** |

One trap worth carrying: stock `imu_processors` does not declare
`stationary_timeout` and **silently ignores** it — the node starts normally and
logs nothing. A stock build given a positive timeout therefore looks configured
and is not. Check with `ros2 param get /imu_bias_remover stationary_timeout`,
which errors on stock and returns the value on the fork.

## 2. `rf2o_laser_odometry` — the branch pin in `.repos` does not exist

`f1tenth.repos` pins:

```yaml
rf2o_laser_odometry:
  version: humble-devel        # <- this branch does not exist on the remote
```

`git ls-remote --heads https://github.com/privvyledge/rf2o_laser_odometry`
returns exactly two branches: `ros1` and `ros2`. **The correct pin is `ros2`.**

The image currently carries `5dbfd7e`, which is one commit before **`664e0fc`,
"Add scan-derived zero-velocity detection to stop stationary pose drift"**. That
commit is the fix for rf2o's stationary heading random walk and is verified on
this vehicle:

| | baseline | run A | run B |
|---|---|---|---|
| `odom/rf2o` parked drift | +3.08 °/min (prior: −5.58, +4.60) | **+0.02** | **+0.40** |

Until the image is rebuilt, the patch is re-applied and rf2o rebuilt on every
container start. A drive done without that step silently tests the **old** rf2o.

## 3. `RMW_IMPLEMENTATION` is never set (bug-166)

The image sets `CYCLONEDDS_URI` but **not** `RMW_IMPLEMENTATION`, so any process
launched outside `scripts/live_runs/00_env.sh` runs **FastRTPS and reads no DDS
configuration at all**. Re-confirmed on `humble-devel-08052026` on 2026-08-09
(`RMW=` empty, `CYC=file:///mnt/shared_dir/cyclonedds_config_static.xml`).

This matters beyond tidiness: the CycloneDDS loopback fix that eliminated the
Isaac VSLAM frame-jitter stalls lives in that config file, so on any manually
launched stack it was inert. The coworkers' newer `humble-latest` image *does*
set it.

```dockerfile
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

`00_env.sh` defaults it too, so scripted runs are already correct — this fixes
the manual ones.

> **Partly addressed 2026-08-24, outside the image.** `~/bolus_ws/f1tenth_launch.sh`
> now passes `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` as a `--env` flag, so the
> operator's normal bring-up is correct. **The ask above still stands** — that flag
> is one launch script on one machine, and anything started another way still gets
> FastRTPS with no DDS configuration. Setting it in the image is the durable fix.

## 4. `trajectory_following_ros2` — carry the steering sign fix

`yaw_rate_to_steering_angle()` used `math.atan2(WHEELBASE, radius)`, which
returns a second-quadrant angle for right turns and pinned the saturator to full
**left** lock — every Nav2 right turn steered hard left (bug-140). Fixed upstream
as `math.atan(WHEELBASE / radius)`, commit **`4770ecc`** on
`privvyledge/trajectory_following_ros2`, branch `refactor/unify-backends`. The
same commit turns the hardcoded ±0.4 rad saturation into a `max_steering_angle`
parameter.

The image predates it, so it is re-applied per container from
`/mnt/shared_dir/apply_twist_fix.sh`. Pin the repo at or past `4770ecc`.

## Interim image, so this is not blocking

A container with all four workarounds applied was committed on 2026-08-09 as:

```
privvyledge/f1tenth:humble-devel-08092026
```

`~/bolus_ws/f1tenth_launch.sh` on gosling1 now references it (previous version
saved as `f1tenth_launch.sh.bak_pre08092026`). Docker's `data-root` is
`/mnt/f1tenth_ssd/docker` on the NVMe, so the extra image costs nothing that
matters. (~~not the 96 %-full SD card~~ — after the 2026-08-24 reflash the M.2
*is* root and there is no SD card in the storage path at all.)

**This is a stopgap, not a substitute.** A committed container is undocumented
and unreproducible — it records *what* changed but not *why* or *from where*.
The four items above are the reproducible form, and the commit should be
discarded once the image build carries them.
