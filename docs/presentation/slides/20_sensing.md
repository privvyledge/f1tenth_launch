<!--
S20 Sensing

Section file for the F1TENTH deck. Slides are separated by a line containing
only `---`. Every slide carries one cut tag; a slide with no tag is
reference-only and appears in `build.sh full` alone.

Conventions (plan §1) - the build enforces the first three:
  - a placeholder card's ID must be listed in ASSETS.md, and an asset marked
    DONE there must be embedded, not carded;
  - no slide body may name an agent file or a bug ID (speaker notes may);
  - every number on a slide needs a note of the form "src: FILE; measured DATE"
    in an HTML comment on that slide;
  - the title is the claim the slide makes, not the topic;
  - rates, latency, resolution and defaults/min/max go in tables, not prose.

Owner: B1 (f1tenth_launch)
Plan rows for this section are quoted above each slide, verbatim from §3.
-->

<!-- cut: lab sponsor research -->
<!-- _class: cols -->
<!-- plan §3 row 2.1 | owner: B1 (f1tenth_launch) -->

## Nothing reaches an estimator raw: every sensor passes a pre-processing stage first

<div class="split">
<div>

![w:520](../assets/figures/arch/1_Sensing_crop.png)

</div>
<div>

Four sensor groups, each with its own filter chain:

- **Stereo** → depth, disparity and colour cloud, aligned to the RGB frame
- **2D LiDAR** → a noise/crop filter before anything consumes the scan
- **Wheel encoder and steering** → the only sensor measuring the *actuator*
- **IMU** → noise filtering, then orientation estimation

*The rates printed here are design targets. What the hardware delivers is in the next slides, and it is lower.*

</div>
</div>

<!-- src: figure exported from the Obsidian `1_Sensing` note, received 2026-09-02, cropped above its notes block; reference export, redraw queued. Configured/measured rates: config/sensors/*.yaml and the bag statistics slide. -->


---

<!-- cut: lab research -->
<!-- _class: dense -->
<!-- plan §3 row 2.2 | owner: B1 (f1tenth_launch) -->

## The camera runs at 848×480×30 with the IR emitter off, and no post-processing filter is enabled

| Topic | Format | Configured | QoS | On by default |
|---|---|---|---|---|
| `camera/color/image_raw` | 848×480, RGB8 | 30 fps | `SENSOR_DATA` | yes |
| `camera/infra1/image_rect_raw`, `…infra2…` | 848×480, Y8 | 30 fps | `SENSOR_DATA` | yes |
| `camera/depth/image_rect_raw` | 848×480, Z16 | 30 fps | `SENSOR_DATA` | yes |
| `camera/aligned_depth_to_color/image_raw` | 848×480 | 30 fps | `SENSOR_DATA` | yes |
| `camera/depth/color/points` | coloured cloud | 30 fps | `SENSOR_DATA` | yes — from `depth_image_proc`, **not** the RealSense SDK |
| `camera/imu` → `camera/imu/filtered` | gyro + accel, interpolated | 200 Hz | `SENSOR_DATA` | yes |

- **IR emitter off** — on, it improves depth and prints a structured-light pattern across both IR images that makes them useless for visual odometry. This platform's odometry needs the IR pair, so the emitter loses.
- **Every post-processing filter is commented out** — decimation, spatial, temporal, disparity and hole-filling. Depth arrives full-resolution, unsmoothed, clipped at 6.0 m.
- 848×480×60 works standalone but falls to ~20 Hz under full teleop load.

> [!PLACEHOLDER CHART-RATES]
>
> Measured per-topic rates from a drive bag — the column this table cannot fill offline.

<!-- src: config/sensors/realsense_config.yaml and launch/sensors/realsense_d435i.launch.py (qos default SENSOR_DATA, emitter_enabled 0, align_depth True, enable_pointcloud False); cloud publisher is depth_image_proc::PointCloudXyzrgbNode in launch/sensors/stereo_and_depth_image_processing.launch.py; read 2026-09-02 -->
<!-- CORRECTION to plan §3 row 2.2: the plan says "decimation 2". It is not enabled -
     the decimation_filter block is commented out in the YAML and nothing sets it in
     the launch files. Flagged in the hand-back. -->


---

<!-- reference-only: no cut tag (plan §3 marks this R) -->
<!-- plan §3 row 2.3 | owner: B1 (f1tenth_launch) -->

## Depth is computed on the camera, not on the Jetson

![h:390](../assets/figures/arch/1A_StereoVision.png)

Rectification, cost matching and disparity all happen in the camera's ASIC; the computer receives depth and reprojects it. That is why the emitter is a *camera* setting with a *software* consequence — and why depth costs the host almost nothing.

<!-- src: figure exported from the Obsidian `1A_StereoVision` note, received 2026-09-02. Reference export; it embeds a third-party stereo-geometry figure that must be attributed or cropped before this deck is shown outside the lab. -->


---

<!-- cut: lab sponsor research -->
<!-- _class: dense -->
<!-- plan §3 row 2.4 | owner: B1 (f1tenth_launch) -->

## The LiDAR is configured to its spec, not its datasheet maximum — the difference was phantom obstacles

| Parameter | Value | Why not the obvious value |
|---|---|---|
| `range_max` | 10.0 m | 12.0 m admitted out-of-spec returns that appeared as phantom obstacles |
| `range_min` | 0.12 m | 0.10 m produced ghost returns clustered at the sensor origin |
| `frequency` | 12.0 requested, **~8 Hz observed** | USB and CPU contention on this compute; the driver does not report the shortfall |
| `invalid_range_is_inf` | `true` | out-of-range must be `inf`, not `0.0`, or the occupancy grid and the particle filter both mis-handle it |
| `intensity` | `false` | `true` causes checksum errors and a driver crash; `false` reports a constant 1008.0 status word |

`lidar/scan` → **speckle filter** (drop any return with no neighbour within 0.15 m) → `lidar/scan_filtered`, which is what every consumer subscribes to. The intensity filter in the same chain is permanently commented out: this sensor has no usable intensity to filter on.

> [!PLACEHOLDER VID-SENS-LIDAR]
>
> 10–20 s of `lidar/scan_filtered` in RViz while the car is pushed through the lab. The one sensor clip the talk shows.

<!-- src: config/sensors/ydlidar_X4.yaml (with its inline justifications), config/filters/laser_filter.yaml; read 2026-09-02 -->


---

<!-- cut: lab research -->
<!-- _class: dense -->
<!-- plan §3 row 2.5 | owner: B1 (f1tenth_launch) -->

## Two IMUs, and neither one is trusted for heading

| | VESC IMU | RealSense D435i IMU |
|---|---|---|
| Topic | `vehicle/sensors/imu/raw` | `camera/imu` → `camera/imu/filtered` |
| Rate | ~100 Hz | 200 Hz gyro and accel, linearly interpolated to one message |
| Magnetometer | none — the `mag` topic is a constant zero | none |
| Orientation from | the VESC's own on-board filter | `imu_filter_madgwick`, `constant_dt: 0.005` |
| Fused into the local EKF | roll and pitch only — **yaw disabled** | angular rate only — **acceleration disabled** |

**Why yaw is off on the VESC IMU**: with no magnetometer its on-board quaternion integrates a stable gyro bias and free-runs at about −14 °/min, while still advertising a tight yaw covariance — so the filter preferred it. Disabling it moved parked drift from **−13.94 °/min to +0.01…+0.17 °/min**.

**Why `constant_dt` is set**: the camera driver stamps its first two or three IMU samples seconds away from the rest. With `constant_dt: 0.0` the filter takes its integration step from those stamps, and one update swings attitude by **63–167°**. Pinning the step to 5 ms leaves a largest step of **0.187°** over a 217 s run — and the raw gyro shows the sensor never moved.

<!-- src: config/localization/ekf_odom.yaml, launch/filters/imu_filter.launch.py, launch/sensors/realsense_d435i.launch.py (imu_filter_constant_dt 0.005); yaw drift measured 2026-08-06, attitude step measured on run17 2026-08-26 -->


---

<!-- cut: lab sponsor research -->
<!-- plan §3 row 2.6 | owner: B1 (f1tenth_launch) -->

## Wheel odometry is three constants and a bicycle model — and it is silent while the car is parked

$$
v = \frac{\text{erpm}}{3750} \qquad
\delta = \frac{\text{servo} - 0.56}{-1.1448} \qquad
\omega = \frac{v \tan\delta}{0.256}
$$

- `vesc_to_odom` reads the ERPM and servo position the VESC reports back, not the values that were commanded — so it measures the actuator, not the intent.
- Every one of those three constants is calibration, and each is covered later: the steering pair is measured on this car, the speed gain is inherited and only corroborated.
- **It returns early until the first servo command arrives.** Parked with nothing driving, `vehicle/vesc_odom` publishes nothing at all — so the one source that could assert "the car is stopped" is absent exactly when a diverging filter would need it.

<!-- src: config/vehicle/vesc.yaml (speed_to_erpm_gain 3750, steering_angle_to_servo_gain -1.1448, offset 0.56), urdf/launch wheelbase 0.256 m; vesc_to_odom early-return confirmed in the f1tenth_system working copy 2026-08-25 -->


---

<!-- cut: lab sponsor research -->
<!-- _class: dense -->
<!-- plan §3 row 2.7 | owner: B1 (f1tenth_launch) -->

## Recording the image streams costs 125 MB/s — and it silently starves visual odometry

> [!PLACEHOLDER CHART-RATES]
>
> Per-topic measured rate and bandwidth from a drive bag: the sensor rates this section quotes as configured, shown as achieved.

| Bag topic set | Size | What it costs |
|---|---|---|
| `drive` — includes depth, infra and the coloured cloud | **26 GiB over 214 s** (~125 MB/s) | starves the camera's USB thread |
| `sysid` — the two IMU topics, no images | **47 MB over 105 s** | nothing measurable |

**The failure mode is worth the slide on its own.** With the image set recording, only the *infra1/infra2* pair stops — colour, depth, the cloud and the camera IMU all keep streaming at full rate. Visual SLAM consumes the IR pair, so it is starved in lockstep: **107 VSLAM messages against the local EKF's 6415** over the same 214 s. It never crashes, never logs an error, and reads a healthy 30 Hz both before and after.

**So: verify a stream by message *count* in the recording, never by a topic rate before or after it.** Fusion stays at 30 Hz on its remaining sources and looks entirely healthy while one input is gone.

<!-- src: scripts/live_runs/topic_sets.sh, scripts/analysis/bag_stats.py; measured on gosling1 2026-09-01. Stall began 4.9 s after the recorder opened and ended 1.1 s after it closed. -->
