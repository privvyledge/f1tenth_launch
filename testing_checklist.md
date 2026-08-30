# F1/10 Testing Checklist — humble-dev branch

Track progress by replacing `[ ]` with `[x]` (done) or `[!]` (bug found).
Add bug notes inline under the failing item.

---

## 0. Pre-flight (offline, no robot required)

- [x] **Python syntax check** — verify all modified launch files parse without error.
  ```bash
  python3 -c "import ast; ast.parse(open('launch/bringup.launch.py').read())"
  python3 -c "import ast; ast.parse(open('launch/mapping.launch.py').read())"
  python3 -c "import ast; ast.parse(open('launch/teleop.launch.py').read())"
  python3 -c "import ast; ast.parse(open('launch/localization/localization.launch.py').read())"
  python3 -c "import ast; ast.parse(open('launch/vehicle/ackermann_mux.launch.py').read())"
  python3 -c "import ast; ast.parse(open('launch/vehicle/joystick.launch.py').read())"
  ```

- [x] **joy_config.yaml exists and is correct** — split from joy_teleop.yaml; contains joy node params only.
  ```bash
  cat config/vehicle/joy_config.yaml
  # Expected: device_id, deadzone, autorepeat_rate, coalesce_interval
  ```

- [x] **YAML syntax** — check all modified config files.
  ```bash
  python3 -c "import yaml; yaml.safe_load(open('config/vehicle/joy_teleop.yaml'))"
  python3 -c "import yaml; yaml.safe_load(open('config/vehicle/mux.yaml'))"
  python3 -c "import yaml; yaml.safe_load(open('config/vehicle/vesc.yaml'))"
  python3 -c "import yaml; yaml.safe_load(open('config/localization/ekf_odom.yaml'))"
  python3 -c "import yaml; yaml.safe_load(open('config/localization/ekf_map.yaml'))"
  python3 -c "import yaml; yaml.safe_load(open('config/localization/localizer_amcl.yaml'))"
  python3 -c "import yaml; yaml.safe_load(open('config/localization/localizer_slam.yaml'))"
  python3 -c "import yaml; yaml.safe_load(open('config/sensors/realsense_config.yaml'))"
  python3 -c "import yaml; yaml.safe_load(open('config/nav2_params.yaml'))"
  ```

- [x] **Build package** — confirm no install-time errors.
  ```bash
  # From workspace root:
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to f1tenth_launch
  ```

---

## 1. Vehicle Actuators (VESC)

**What changed**: `max_acceleration` in vesc.yaml lowered 2.5 → 1.0 m/s²; `vesc_poll_rate` default 200 → 50 Hz; variable name bug fixed (`max_servo_rate` → `max_steering_rate`).

- [x] **VESC driver starts** — check no parameter errors on launch.
  ```bash
  ros2 launch f1tenth_launch teleop.launch.py launch_sensors:=False launch_localization:=False
  ros2 node list | grep vesc
  ```

- [x] **vesc_poll_rate is 50 Hz** — verify actual publish rate of VESC state topic.
  ```bash
  ros2 topic hz /sensors/core
  # Expected: ~50 Hz
  ```

- [x] **Encoder odometry publishes** — confirm `odom/vesc` topic is active. (Actual topic: `/gosling1/vehicle/vesc_odom`)
  ```bash
  ros2 topic echo /odom/vesc --once
  ```

- [x] **Motor responds to drive command** — send a zero-speed command, wheel should not spin.
  ```bash
  ros2 topic pub --once /drive ackermann_msgs/msg/AckermannDriveStamped \
    '{drive: {speed: 0.0, steering_angle: 0.0}}'
  ```

- [!] **Acceleration limit works** — with the lower limit (1.0 m/s²), the car should accelerate more gradually. Drive forward with joystick and observe start behavior.
  > **Bug**: When `launch_throttle_interpolator_node:=True`, throttle acceleration is limited correctly but steering misbehaves — returns to center instead of responding to joystick.
  >
  > **[2026-08-30] The operator reports this was diagnosed and fixed in an earlier session, cause attributed to the mux and the joystick — but the fix was never recorded, so the item stays `[!]` until it is re-verified.** Searched and found nothing: `git log --all --grep=throttle` shows only the two original "Setup actuation interpolation/smoothing" commits, `git log -S throttle_interpolator -- launch/vehicle/vehicle.launch.py` shows nothing since the original wiring, and **none of the 141 entries in the buglog is this bug**. The nearest match is **bug-049** (mux `joystick.timeout` 0.1 → 0.3 s, because the joystick's priority-100 claim lapsed ~155×/min and let `navigation` command the car). Its mechanism is adjacent — a competing publisher winning mux arbitration and zeroing what the joystick asked for — but its recorded symptom is spurious parked servo twitches, not steering centering under the interpolator, so it is **not** established to be the same fix.
  >
  > **Re-test, and it needs NO battery and no VESC** — steering passthrough is visible in the command stream. Connect the DualSense (that also supplies the `command_gate` heartbeat), launch with `launch_throttle_interpolator_node:=True`, hold the deadman and sweep the steering stick while echoing **values**, not rates:
  > ```bash
  > ros2 topic echo /ackermann_drive --field drive.steering_angle        # mux output
  > ros2 topic echo /vehicle/ackermann_cmd --field drive.steering_angle  # gate output -> VESC
  > ```
  > Steering tracking the stick on both = fixed, tick the box. Pinned at 0.0 on either = still broken, and the difference between the two topics localises it to the mux or to the gate. Then reconcile `CLAUDE.md` and the RESUME §10 table, which both still call this open.

---

## 2. Joystick / Teleop

**What changed**: `joystick.launch.py` now loads separate `joy_config.yaml` (joy node) and `joy_teleop.yaml` (joy_teleop node). Previously both pointed to the same file. `joy_teleop.yaml` had the joy node params (device_id etc.) removed.

- [x] **Joy node starts with correct config** — verify device_id and deadzone are applied.
  ```bash
  ros2 launch f1tenth_launch teleop.launch.py launch_sensors:=False launch_localization:=False
  ros2 param get /joy joy/device_id
  # Expected: 0
  ros2 param get /joy joy/deadzone
  # Expected: 0.01
  ```

- [x] **Joy_teleop starts without error** — should not log any missing parameter warnings.
  **[2026-08-27] PASS.** Zero `joy_teleop` WARN/ERROR lines across a full `bringup.launch.py`
  launch log; `/joy` and `/joy_teleop` both present under CycloneDDS. The prior `[!]` predated the
  two-heartbeat-command rework (bug-048) and was stale. The only WARNs in that launch are benign
  startup-order ones: `rf2o` waiting for scans/TF at t=0, camera `unite_imu_method` param notes,
  and `visual_slam_node` frame-delta messages during init.
  ```bash
  ros2 node list | grep joy_teleop
  ros2 node info /joy_teleop
  ```
  > **Note**: `joy_teleop` not visible in `ros2 node list` under rmw_fastrtps_cpp. Appears normally under rmw_cyclonedds_cpp (confirmed). Run `ros2 component list` to verify when using fastrtps. `/gosling1/teleop` publishes and drive is functional.

- [x] **DualSense / controller connects** — `/joy` topic should publish on button press.
  ```bash
  ros2 topic echo /joy --once
  ```

- [x] **Manual drive works** — hold deadman button (L1/R1), push left stick forward, car should drive.
  Verify `teleop` topic receives commands:
  ```bash
  ros2 topic echo /teleop --once
  ```

- [x] **Steering works** — left stick left/right produces correct `steering_angle` (positive=left on F1/10).
  Check steering axis (should be axis 2 for DualSense):
  ```bash
  ros2 topic echo /teleop
  ```

- [x] **`require_deadman:=False` disables interlock** — `teleop` should publish drive commands without any button held.
  ```bash
  ros2 launch f1tenth_launch teleop.launch.py launch_vehicle:=False launch_sensors:=False launch_localization:=False require_deadman:=False
  # Push stick without holding L1/R1 — teleop topic should publish:
  ros2 topic echo /teleop --once
  # Also verify the parameter reached joy_teleop:
  ros2 param get /joy_teleop human_control.deadman_buttons
  # Expected: [] (empty list)
  ```
  > **Fix applied**: `deadman_buttons` param in `joy_teleop.yaml` was commented out; upstream `joy_teleop` disables deadman when the param is absent. Works correctly.

- [x] **`require_deadman:=True` (default) still enforces interlock** — release L1/R1 and confirm `teleop` stops publishing.
  ```bash
  ros2 launch f1tenth_launch teleop.launch.py launch_vehicle:=False launch_sensors:=False launch_localization:=False
  ros2 param get /joy_teleop human_control.deadman_buttons
  # Expected: [4, 9]
  ```
  > **Fix applied**: `deadman_buttons` commented out of `joy_teleop.yaml`; `joystick.launch.py` injects it via `teleop_params` only when `require_deadman:=True` (lines 108–110). Both modes tested on robot and confirmed functional.

- [x] **`max_speed` launch arg overrides YAML** — the `scale` in `joy_teleop.yaml` is overridden at runtime by the `max_speed` launch arg; editing the YAML has no effect. Verify the runtime value.
  ```bash
  ros2 launch f1tenth_launch teleop.launch.py launch_vehicle:=False launch_sensors:=False launch_localization:=False
  ros2 param get /joy_teleop human_control.axis_mappings.drive-speed.scale
  # Expected: 5.0 (default launch arg), regardless of what joy_teleop.yaml says
  ```
  > **Note**: Tested at `max_speed:=1.0`. `/gosling1/drive` consistently published 1.0. `ackermann_cmd` also showed 1.0 after mux timeout was raised from 0.05 → 0.1 s (see below).

- [x] **Speed cap is respected at full stick deflection** — hold deadman, push left stick to max; `teleop` speed field should equal `max_speed`. Relaunch at a lower value to confirm the cap changes.
  ```bash
  # At default max_speed:=5.0:
  ros2 topic echo /teleop --field drive.speed
  # Expected: ~5.0 at full forward stick
  #
  # Relaunch at 2.0 m/s and recheck:
  ros2 launch f1tenth_launch teleop.launch.py launch_vehicle:=False launch_sensors:=False launch_localization:=False max_speed:=2.0
  ros2 topic echo /teleop --field drive.speed
  # Expected: ~2.0 at full forward stick
  ros2 param get /joy_teleop human_control.axis_mappings.drive-speed.scale
  # Expected: 2.0
  ```
  > **Bug found → fixed**: `ackermann_cmd` occasionally showed 0.0 spikes at `max_speed:=1.0`. Root cause: `/gosling1/teleop` publishes at 16–65 Hz (highly variable) while mux joystick timeout was 0.05 s. Gaps between publishes exceeded the timeout, causing the safety fallback to fire. Fixed: raised joystick `timeout` in `mux.yaml` from 0.05 → 0.1 s. Confirmed stable 1.0 output after fix.

---

## 3. Safety Mux

**What changed**: `mux.yaml` adds a `safety` topic at priority 1 (lowest). `ackermann_mux.launch.py` spawns a 40 Hz zero-speed publisher on `safety`. Joystick timeout tightened 0.2 → 0.05 s. Namespace bug fixed: safety publisher now resolves to `/<ns>/safety` when `use_f1tenth_namespace=True`.

- [x] **Mux starts and safety publisher is running**.
  ```bash
  ros2 launch f1tenth_launch teleop.launch.py launch_sensors:=False launch_localization:=False
  ros2 topic hz /safety
  # Expected: ~40 Hz
  ros2 topic echo /safety --once
  # Expected: AckermannDriveStamped with speed=0.0
  ```

- [x] **Safety publisher reaches the mux under namespace** — `ExecuteProcess` bypasses `PushRosNamespace`; the fix must publish to the correct absolute topic.
  > **Note**: `ros2 topic info -v /gosling1/safety` reports `Publisher count: 0` under rmw_fastrtps_cpp despite 40 Hz traffic. This is a fastrtps discovery artifact for intra-process (component) publishers — not an issue under rmw_cyclonedds_cpp (confirmed). No root `/safety` in topic list — namespace fix confirmed correct.
  ```bash
  # Launch with namespace active (the default when VEHICLE_NAME or USER is set):
  ros2 launch f1tenth_launch teleop.launch.py launch_sensors:=False launch_localization:=False \
    use_f1tenth_namespace:=True f1tenth_namespace:=f1tenth
  # Safety publisher should appear on the NAMESPACED topic, not root /safety:
  ros2 topic hz /f1tenth/safety
  # Expected: ~40 Hz
  ros2 topic hz /safety
  # Expected: no publishers (root topic should be empty)
  ```

- [x] **Mux output topic exists**.
  ```bash
  ros2 topic echo /ackermann_cmd --once
  ```

- [x] **Joystick priority overrides safety** — when joystick is active (deadman held), `/ackermann_cmd` reflects joystick commands, not zero.

- [x] **Safety fallback fires on joystick disconnect** — disconnect the controller (or release deadman for >0.05 s). The car should stop within 50 ms (joystick timeout).
  ```bash
  ros2 topic echo /ackermann_cmd
  # After releasing deadman: speed should drop to 0.0 quickly
  ```

- [x] **No noise from safety topic** — verify safety publisher output has exactly zero speed and steering; no floating point garbage.

---

## 4. Sensors

**What changed**: RealSense `ordered_pc` false (was true); IMU rate lowered 200 → 100 Hz; decimation filter enabled (magnitude 2). YDLidar: `intensity: true`, `range_max: 10.0`, `range_min: 0.12`, `frequency: 12.0`, `invalid_range_is_inf: true`; intensity filter added to `laser_filter.yaml` as commented-out `filter1` (requires tuning before enabling).

- [x] **LiDAR scan publishes** — 8.7 Hz (expected 8-9 Hz for YDLidar X4). Frame id: `lidar`. ✓
  ```bash
  ros2 launch f1tenth_launch teleop.launch.py launch_localization:=False
  ros2 topic hz /lidar/scan
  # Expected: ~8-9 Hz (YDLidar X4)
  ros2 topic echo /lidar/scan --once
  ```

- [x] **LiDAR scan_filtered publishes** — 8.7 Hz, matching raw scan rate. ✓
  ```bash
  ros2 topic hz /lidar/scan_filtered
  ```

- [x] **LiDAR intensity field is populated** — **CLOSED, won't-fix (hardware limitation).** `ydlidar_X4.yaml:16` ships `intensity: false` and must stay there; the line above previously claimed `intensity: true` "is now set", which was never true of the committed config.
  ```bash
  ros2 topic echo /lidar/scan --once | grep -A5 intensities
  # Expected: array of non-zero values (typically 0–255 for X4)
  # If all zeros or field absent: intensity not supported on this firmware/driver build
  ```
  > **Bug confirmed**: YDLidar X4 does NOT support per-return intensity. Setting `intensity: true` causes continuous checksum errors and driver failure. With `intensity: false` (current config), the driver publishes `intensities: [0.0, 1008.0, ...]` — constant 1008.0 values that are internal status bits, not real intensity. **Conclusion**: intensity filter in `laser_filter.yaml` is unusable on the X4. Keep `intensity: false`; disable the `filter1` intensity filter entry permanently (leave it commented). See https://github.com/YDLIDAR/ydlidar_ros2_driver/blob/humble/details.md for X4 channel details.

- [x] **Intensity filter tuning** — **CLOSED, won't-fix**: the YDLidar X4 has no real intensity data, so the `laser_filter.yaml` intensity filter is permanently unusable. Leave it commented out and skip this item; it is not pending work.
  ```bash
  # Step 1: observe raw intensity distribution while driving near walls and across open space
  ros2 topic echo /lidar/scan | grep -A2 intensities
  # Note min/max of values for valid wall returns vs ghost/noise returns.
  # Typical valid returns on X4: 100–255. Ghost returns from multipath/near-range: <50.

  # Step 2: uncomment filter1 in config/filters/laser_filter.yaml with a candidate threshold, e.g.:
  #   lower_threshold: 100
  # No rebuild needed (symlink-install). Restart the laser_filters node:
  ros2 launch f1tenth_launch teleop.launch.py launch_localization:=False

  # Step 3: compare raw vs filtered in RViz
  # Add two LaserScan displays: /lidar/scan (red) and /lidar/scan_filtered (green)
  # Drive near walls. Green scan should match red for solid walls; ghost returns should disappear.

  # Step 4: if real wall returns are being dropped, lower the threshold and repeat.
  # If ghost returns persist, raise the threshold.
  ros2 topic hz /lidar/scan_filtered
  # Expected: same Hz as /lidar/scan (filter does not change rate)
  ```
  > **Tuning note**: Start at `lower_threshold: 100`. Raise to remove phantoms; lower if real walls disappear. Do not set above ~150 without confirming wall returns are preserved at distance (>5 m returns have lower intensity on the X4). Leave `upper_threshold: 65535` unless specular spikes appear.

- [x] **RealSense color stream starts** — confirmed at 848×480×30 (~28 Hz) after clean Jetson reboot with USB 3 enumeration.
  > **Resolved**: Prior "USB 2.1" warning and 640×360×30 rejection were artifacts of a GLSL runtime restart, not clean startup. After full USB re-enumeration the camera is detected as USB 3 and accepts the configured profile. Current config: `848x480x30` in `realsense_config.yaml`.
  ```bash
  ros2 topic hz /camera/color/image_raw
  # Observed: ~28 Hz
  ros2 topic echo /camera/color/image_raw --once | grep -E 'width|height'
  # Observed: height: 480, width: 848
  ```

- [x] **RealSense depth stream starts** — aligned depth at 640×480 (aligned to color); raw depth at 640×360. ✓
  > **Note**: Decimation filter (magnitude 2) halves depth resolution. `camera/depth/image_rect_raw` shows 640×360 — this is BEFORE decimation (raw sensor). Decimation output visible on the pointcloud/aligned topic path. Confirm decimated dimensions if needed.
  ```bash
  ros2 topic echo /camera/aligned_depth_to_color/image_raw --once | grep -E 'width|height'
  ```

- [x] **RealSense IMU rate** — confirmed at 200 Hz after `gyro_fps: 200` fix.
  > **Bug fixed**: D435i gyroscope only supports 200 Hz and 400 Hz. `gyro_fps: 100` is invalid and rejected. Fixed: `gyro_fps: 200` in realsense_config.yaml. `accel_fps: 100` is valid. IMU at 200 Hz is acceptable for EKF at 30 Hz.
  ```bash
  ros2 topic hz /camera/imu/filtered
  # Observed: ~200 Hz
  ```

- [x] **SDK pointcloud (`camera/depth/color/points`) publishing** — confirmed after clean boot and `color_format: RGB8` fix.
  > **Root cause found and fixed**: `color_format: BGR8` in `realsense_config.yaml` caused "No stream match for pointcloud chosen texture Process - Color" — the RealSense SDK requires `RGB8` for colored pointcloud generation. Fixed: `color_format: "RGB8"`. After fix, SDK pointcloud publishes at ~27 Hz. RTABMap `depthimage_to_pointcloud` path also works but costs ~200% CPU vs ~72% for the SDK path; use SDK path by default (`depthimage_to_pointcloud:=False`).
  > **Note on `is_dense`**: Both `camera/depth/color/points` and `camera/downsampled_cloud_from_depth` report `is_dense: true` even with `ordered_pc: false`. This is correct — `is_dense` means no NaN/Inf points in the cloud, which is independent of point ordering.
  ```bash
  ros2 topic hz /camera/depth/color/points
  # Observed: ~27 Hz (stabilizes after ~10s)
  ros2 topic echo /camera/depth/color/points --once | grep is_dense
  # Observed: is_dense: true (all points valid; ordered_pc affects grid layout, not validity)
  ```

---

## 5. Localization — rf2o + EKF Odom

**What changed** (most significant block):
- `rf2o_odometry_node` was **commented out** in the previous version; it is now **active**.
- EKF `odom2` source changed from `odom/rtabmap/icp` (ran ICP odometry which was **also** controlled by `launch_laserscan_odometry`) to `odom/rf2o`.
- ICP odometry now has its own `launch_icp_odometry` flag (default `False`); rf2o `publish_tf` condition was checking for the wrong string (`'icp'`), now correctly checks `'rf2o'`.
- EKF frequency 50 → 30 Hz; `use_control: true`; acceleration limits uncommented (3.0 m/s²).

- [x] **rf2o_odometry_node starts** — was previously commented out.
  ```bash
  ros2 launch f1tenth_launch teleop.launch.py
  ros2 node list | grep rf2o
  ros2 topic hz /odom/rf2o
  # Expected: rf2o publishes at ~8-9 Hz (tied to LiDAR)
  ```
  > **Note**: Node names are `CLaserOdometry2D` and `CLaserOdometry2DNode` — `grep rf2o` returns empty. Use `grep CLaserOdometry` instead. Topic `/gosling1/odom/rf2o` publishes at ~8.5 Hz as expected. rf2o outputs verbose INFO logs every scan cycle (two lines per 100 ms); see bug note for silencing.

- [x] **EKF odom node starts at 30 Hz**.
  ```bash
  ros2 topic hz /odometry/local
  # Expected: ~30 Hz
  ```
  > **Confirmed**: ~29–30 Hz. Note: EKF was tested with `use_control: true` (since reverted to `false` in ekf_odom.yaml). Three warnings appeared — see bug notes for `odom2`/`imu0` differential+relative conflict.

- [x] **EKF is fusing VESC odom + rf2o** — check covariance is not exploding.
  ```bash
  ros2 topic echo /odometry/local --once
  # covariance diagonal values should be small (not NaN or >1.0 for x,y,yaw)
  ```
  > **Bug**: rf2o publishes all-zero covariance (see `/gosling1/odom/rf2o` echo). robot_localization treats zero-covariance as infinitely precise, biasing the EKF heavily toward rf2o. Pose covariance values on `/odometry/local` are non-NaN (4.39, 4.42 for x/y) but large relative to short-term drift. Root fix: use the forked rf2o (https://github.com/privvyledge/rf2o_laser_odometry.git) which adds proper covariance estimation. Also: `odom2_differential: true` and `odom2_relative: true` are both set — EKF warns and picks differential; set `odom2_relative: false` to suppress.
  > **RESOLVED (2026-07-22)**: forked rf2o is pinned in `f1tenth.repos` (adds covariance estimation); commit `cb3b567` set `odom2_relative: false` and added Mahalanobis rejection thresholds on odom2 in `ekf_odom.yaml`.

- [x] **odom → base_link TF publishes** — with `odom_tf_publisher=ekf`, EKF should publish this TF.
  ```bash
  ros2 run tf2_ros tf2_echo odom base_link
  ```
  > **Confirmed**: TF publishes. Observed ~0.5°/s yaw drift while stationary — see rf2o competing TF bug below.

- [x] **rf2o publish_tf is False** — with default `odom_tf_publisher=ekf`, rf2o should not publish a competing TF.
  ```bash
  ros2 param get /rf2o_odometry_node publish_tf
  # Expected: false
  ```
  > **Bug**: `ros2 param get /gosling1/CLaserOdometry2DNode publish_tf` returns `false`, but `ros2 node info /gosling1/CLaserOdometry2DNode` shows the node is a publisher on `/gosling1/tf`. Upstream rf2o does not respect the `publish_tf` parameter — it publishes `odom→base_link` TF regardless. This creates a second competing TF source alongside the EKF and is the likely cause of the stationary yaw drift (~0.5°/s). Fix: switch to the forked rf2o package (https://github.com/privvyledge/rf2o_laser_odometry.git).
  > **RESOLVED (2026-07-22)**: forked rf2o is pinned in `f1tenth.repos`; the fork respects `publish_tf: False`.

- [x] **ICP odometry is NOT running** — verify `launch_icp_odometry` defaults to False.
  ```bash
  ros2 node list | grep icp
  # Should NOT see rtabmap_icp_odom node
  ```
  > **Confirmed**: empty output.

- [ ] **Drive robot in a loop** — odom should roughly close the loop over ~5 m. Visualize in RViz or check via:
  ```bash
  ros2 topic echo /odometry/local | grep -A3 'pose:'
  ```
  > **Result (elliptical loop, robot returned to origin)**: Position closure excellent — x=0.005 m, y=0.0004 m (sub-centimeter). Yaw at closure: z=−0.211, w=0.977 → **−24.3°** residual heading error. Position fusion (VESC + rf2o) is working well; yaw accumulates drift. Likely causes: (1) rf2o competing TF (upstream bug — publishes `odom→base_link` TF despite `publish_tf: False`); (2) rf2o zero covariance over-weighting its yaw; (3) IMU yaw drift with no magnetometer correction. Retest after switching to forked rf2o (proper covariance + TF fix) to isolate the contribution.
  > **Update (2026-07-22)**: forked rf2o now deployed (`f1tenth.repos`) + EKF fixes in `cb3b567`. Marked `[ ]` — on-robot retest pending to confirm the yaw closure improvement.

---

## 6. Localization — AMCL (with pre-built map)

**What changed**: `max_beams` 360→90, `max_particles` 2000→500, `min_particles` 500→100, `update_min_a` -1.0→0.5 rad, `update_min_d` -1.0→0.2 m (motion-triggered updates).

> **Dependency**: Requires a pre-existing 2D map. Complete **Section 8a** (2D Online Mapping) first to generate `data/maps/raslab.yaml`, then return here.

- [x] **AMCL starts without error**.
  ```bash
  ros2 launch f1tenth_launch bringup.launch.py slam:=False launch_navigation:=False
  ros2 node list | grep amcl
  ```
  > **Confirmed**: Node visible in `ros2 node list`. No errors on launch.

- [x] **Particle cloud visible in RViz** — **[2026-08-30] CLOSED, verified on hardware with the car rolled by hand (no battery, no VESC).** Both 2026-08-27 blockers are cleared. **(1) QoS — fixed in the tracked config, not by hand.** `config/f1tenth.rviz` now carries a `nav2_rviz_plugins/ParticleCloud` display on `/particle_cloud` with **`Reliability Policy: Best Effort`**, so it renders on every launch with no GUI steps. Confirmed live: `ros2 topic info /particle_cloud --verbose` reads `BEST_EFFORT` on *both* ends (amcl and rviz2). Read the *reliability* of both ends, never the subscriber count — it reads 1 in the working and the broken case alike, as with bug-238's durability. **(2) The motion gate — satisfied by ROLLING the car, not carrying it.** The 2026-08-27 attempt failed because the car was *lifted*: `odometry/local` moved 0.000 m, since without the VESC its only sources are rf2o and VSLAM and a lifted car gives neither a usable scan delta nor VSLAM tracking. Pushed along the floor, both produce real odometry and the gate opens. **Measured** (`scripts/live_runs/pcloud_watch.py`, 180 s): parked t=0–40 s, odom path 0.093 m of sensor creep, **0** messages; the push t=43.3–59.0 s took the path to 3.4 m and produced **42 messages of 4000 particles** (matching `max_particles: 4000` in `localizer_amcl.yaml`); motion stopped and publishing stopped, frozen at 42 through t=180 s. Publishing tracks motion in *both* directions, which is the gate working as designed. `map→base_link` moved (0.447, −0.579, −85.0°) → (0.907, −1.535, −65.5°). **Caveat, stated rather than glossed:** particle spread **expanded** 0.076 → 0.364 m across the push rather than contracting. That is expected for ~4 s of hand-push — the motion model injects noise per update and there is no time to reconverge — but it means **convergence under sustained motion is still unproven**; take that on the drive session.
  > **Bug**: `particle_cloud` publishes at ~0.8 Hz (confirmed via `ros2 topic hz /gosling1/particle_cloud`) but is not visible in RViz. Root cause: Nav2 Humble publishes `particle_cloud` as `nav2_msgs/msg/ParticleCloud`, not `geometry_msgs/msg/PoseArray` — the standard RViz2 PoseArray display cannot render it. Fix: install `ros-humble-nav2-rviz-plugins` and add the **"Particle Cloud"** display type from that package (not the generic PoseArray display). The ~0.8 Hz rate while stationary is expected — AMCL resamples slowly when motion triggers (`update_min_d: 0.2 m`, `update_min_a: 0.5 rad`) are not met.

- [x] **AMCL localizes** — set initial pose estimate in RViz (2D Pose Estimate). Particles should converge.
  ```bash
  ros2 topic echo /amcl_pose --once
  # Expected: reasonable (x,y,yaw) close to actual robot position
  ```
  > **Confirmed**: Pose looks correct. Localization converges with `set_initial_pose: True` at origin.

- [x] **Motion-triggered updates work** — particles should only update when robot moves >0.2 m or turns >0.5 rad. Verify AMCL is NOT spinning the CPU when robot is stationary.
  ```bash
  # Check CPU while robot is stationary:
  top -b -n1 | grep amcl
  ```
  > **Note**: AMCL runs as a composable node inside `component_container` — `top | grep amcl` returns empty. Monitor `component_container` instead. Observed 28–48% CPU for the whole container (all composed nodes combined). AMCL's own contribution is small; CPU is dominated by other components.

- [x] **map → odom TF publishes**.
  ```bash
  ros2 run tf2_ros tf2_echo map odom
  ```
  > **Confirmed**: `map→odom` TF publishes correctly.

> **Tuning note**: Attempted to improve accuracy by increasing `max_beams: 90→180`, `max_particles: 500→1000`, tightening `sigma_hit: 0.4→0.2`, and adjusting `z_hit`/`z_rand`. Result: particle cloud rate dropped from 8.7 Hz → 0.4 Hz causing pose jumps. Computation scales O(particles × beams) — doubling both is ~2.7× overhead. Since AMCL is one source fused into `ekf_map`, high update rate matters more than per-update precision. All parameters reverted to original values.

---

## 7. Localization — SLAM Toolbox (localization mode)

**What changed**: `map_update_interval` 0.02 → 0.2 s (was updating at 50 Hz; now 5 Hz).

- [x] **slam_toolbox localizer starts** (requires `odom_tf_publisher:=slam` or explicit launch arg).
  ```bash
  ros2 launch f1tenth_launch localization.launch.py launch_slam_toolbox_localizer:=True launch_amcl:=False
  ros2 node list | grep slam
  ```
  > **Confirmed**: `/gosling1/slam_toolbox` present. `/gosling1/visual_slam_node` also running — launched with `use_gpu:=True`, expected.

- [x] **CPU usage is lower** — 5 Hz map updates vs the previous 50 Hz should be visible.
  ```bash
  top -b -n1 | grep slam
  ```
  > **Confirmed**: `top | grep slam` returns empty (process runs under a different name); `top` directly shows ~10% CPU for the slam_toolbox process. `map_update_interval: 0.2 s` change confirmed effective.
  > **Pose rate**: `ros2 topic hz /gosling1/slam_toolbox/pose` shows ~0.8 Hz. slam_toolbox only publishes a pose when it accepts a scan match (motion-gated), so each correction is a larger step — causes visible jumps. Less accurate and jumpier than AMCL. **Conclusion**: AMCL is the preferred localizer for this platform; slam_toolbox localization mode is a fallback only.

---

## 8. Mapping

**What changed**: `use_sim_time` default in `mapping.launch.py` changed **True → False** (was accidentally enabling sim time for online mapping). `odom_tf_publisher` and `map_tf_publisher` are now proper top-level args.

### 8a. 2D Online Mapping (SLAM Toolbox)

- [x] **Starts without sim time when mapping online**.
  ```bash
  ros2 launch f1tenth_launch mapping.launch.py launch_2d_mapping:=True launch_3d_mapping:=False launch_localization:=True
  ros2 param get /slam_toolbox use_sim_time
  # Expected: false
  ```
  > **Confirmed (live robot)**: `false`. Confirmed (bag): `true` when `use_sim_time:=True` is passed.

- [x] **Map is building** — open RViz, add `/map` topic. Drive robot around; map should grow.
  > **Confirmed**: Map generated successfully via bag playback (`loop3x_no_localization`). Results are rough — localizer tuning still needed. See rosbag command in `COMMANDs.md`.

- [x] **Save map works**.
  ```bash
  # After mapping, save manually:
  ros2 run nav2_map_server map_saver_cli -f raslab_test -t /gosling1/map --ros-args -p map_subscribe_transient_local:=true
  ls raslab_test.yaml
  ```
  > **Bug**: Automatic map saving via `map_saver` (configured in `2d_mapping.launch.py`) did not save on its own during the bag test. Manual save with the command above worked. Needs investigation — likely the `map_saver` lifecycle or topic remapping is not wired correctly for the namespaced case. TODO: verify map_saver node is running and subscribed to the correct `/gosling1/map` topic.
  > **RESOLVED — not a namespace bug (2026-07-22)**: verified offline by resolving the launch tree with the ROS 2 launch API (`PushRosNamespace('gosling1')` from `mapping.launch.py:851` propagates through the `2d_mapping.launch.py` include): `slam_toolbox` publishes `/gosling1/map`, `map_saver_server` runs at `/gosling1/map_saver` with relative `map_topic: map` → `/gosling1/map`. Wiring is correct. The actual cause: **`map_saver_server` has no auto-save feature** — it is an on-demand service server; the `map_url`/`map_topic` launch params are not declared node parameters (the `SaveMap` service *request* carries them). To save in the namespaced case:
  > `ros2 service call /gosling1/map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: <path>, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"`
  > If periodic auto-save is wanted, it must be added (e.g. a timer node calling the service) — feature request, not a bug.

### 8b. 3D Online Mapping (RTABMap CPU)

- [x] **RTABMap starts in online mode**. **[2026-08-04] PASS** — `bringup.launch.py slam:=True launch_navigation:=False use_gpu:=False launch_2d_mapping:=False launch_3d_mapping:=True` brings up `rtabmap_slam/rtabmap`, `rtabmap_sync`, `rtabmap_viz`, and **zero** nvblox/visual_slam processes (the `use_gpu` leak is genuinely fixed). Caveats: `icp_odometry`/`stereo_odometry` each start ×2 (double-include bug, 2026-08-04), and `rtabmap_viz` starts a GUI that costs CPU. Verified via `ps`, not `ros2 node list`.
  ```bash
  ros2 launch f1tenth_launch mapping.launch.py launch_3d_mapping:=True use_gpu:=False launch_localization:=True
  ros2 node list | grep rtabmap
  ```
  > **Bug (fixed)**: With `use_gpu:=False launch_localization:=True`, `nvblox_node` and `visual_slam_node` still launched. Two root causes, both now fixed:
  >
  > 1. **`mapping.launch.py` line 531** (teleop_launch `use_gpu` arg): `'True' if (...or not enable_odom_here) else 'False'` forced `use_gpu='True'` into `teleop_launch`'s `launch_arguments` when `launch_localization=True`. `IncludeLaunchDescription` `launch_arguments` write directly to the shared `context.launch_configurations` dict (not scoped), so this mutated `use_gpu` for all subsequent actions — including the `mapping_3d_gpu_node` condition and `localization.launch.py`'s `gpu_group`. **Fix**: pass `use_gpu` as-is (`"use_gpu": use_gpu`).
  >
  > 2. **`mapping.launch.py` `mapping_3d_gpu_node` condition**: `IfCondition(use_gpu)` used a lazy `LaunchConfiguration` substitution evaluated *after* the action list runs. By that point the shared context had `use_gpu='True'` from the bug above. **Fix**: replaced both `mapping_3d_cpu_node` and `mapping_3d_gpu_node` conditions with the eager string `gpu_enabled = 'True' if use_gpu_string.lower() == 'true' else 'False'`, computed inside the OpaqueFunction where `use_gpu_string` is already the correct user-provided value.

- [x] **RTABMap database is fresh** — verify `life_long_mapping:=False` (default) deletes old DB.
  ```bash
  ls -la data/maps/rtabmap/rtabmap.db
  # Should be a new file (recent timestamp)
  ```
  > **Confirmed**: Fresh DB created on launch. RTABMap DB, occupancy grid, and point cloud (PCD/PLY) all saved successfully.

- [x] **Occupancy grid is building** — RViz, add `/rtabmap/grid_map` topic.
  > **Confirmed**: Occupancy grid visible and building during mapping session.

- [x] **No odometry TF conflict** — with default `launch_localization:=True`, `enable_odom_here=False`, RTABMap should NOT publish its own odom TF.
  ```bash
  ros2 run tf2_tools view_frames
  # Only one odom->base_link edge expected (from EKF)
  ```
  > **Confirmed**: No TF conflicts. Single `odom→base_link` edge from EKF as expected.

### 8c. Offline/Rosbag Mapping

- [x] **Sim time is enabled when using rosbag**.
  ```bash
  ros2 launch f1tenth_launch mapping.launch.py launch_3d_mapping:=True use_sim_time:=True ...
  ros2 param get /gosling1/slam_toolbox use_sim_time
  # Expected: true
  ```
  > **Confirmed**: `true` when `use_sim_time:=True` is passed.

- [x] **ICP odom `guess_from_tf` errors during bag playback**.
  > **Resolved**: Errors no longer appear when the bag's TF tree is properly initialized (static TFs present at playback start). Root cause was that `loop3x_no_localization` was recorded without static TFs — `sensor_kit_link` disconnected from `base_link`. **Fix**: pass `launch_tfs:=True` when replaying bags that lack static TFs. **Prevention**: confirm `ros2 run rqt_tf_tree rqt_tf_tree` shows a complete tree before starting a recording; record several seconds after all static TF publishers are active.

- [x] **SLAM Toolbox message filter queue full at high playback rates**.
  > **Resolved**: No longer occurs at `--clock 10` with `--rate 0.2`. Not a code bug — at 100× the slam_toolbox internal TF message filter cannot drain fast enough. Fix is to reduce bag playback rate; `--clock 10` / `--rate 0.2` is sufficient. If higher-speed replay is needed, increase `message_filter_queue_size` in the slam_toolbox YAML config.

- [!] **Visual SLAM frame delta warnings at high playback rates**.
  > **Observed**: `Delta between current and previous frame [66ms] is above threshold [34ms]` from `visual_slam_node` during 100× bag playback. Expected: the Jetson Orin Nano GPU pipeline cannot process stereo frames at 100× realtime. This warning also appeared because `visual_slam_node` was incorrectly launched due to the `use_gpu` bug above. After the bug fix, `visual_slam_node` will not launch with `use_gpu:=False`, eliminating this warning entirely in CPU mode.

---

## 9. Navigation (Nav2)

**What changed**: `controller_frequency` 20→10 Hz; local costmap `update_frequency` 20→5 Hz; global costmap `update_frequency` 2→1 Hz; removed STVL and NonPersistentVoxelLayer (LiDAR-only costmap now); `max_planning_time_ms` renamed to `max_planning_time` (was silently ignored in Humble).

- [x] **Nav2 stack starts**.
  ```bash
  ros2 launch f1tenth_launch bringup.launch.py slam:=False
  ros2 node list | grep -E 'controller|planner|bt_navigator|costmap'
  ```
  > **Confirmed**: `bt_navigator`, `controller_server`, `planner_server`, `local_costmap/local_costmap`, `global_costmap/global_costmap` all visible. Stack started; see `component_container_isolated` crash bug below.

- [x] **Local costmap uses only LiDAR** — STVL and NonPersistentVoxelLayer removed.
  ```bash
  ros2 param get /local_costmap/local_costmap plugins
  # Expected: ['obstacle_layer', 'inflation_layer']
  ```
  > **Confirmed**: `['obstacle_layer', 'inflation_layer']`. Width: 10 m. STVL and NonPersistentVoxelLayer correctly absent.

- [x] **Global costmap uses LiDAR only**.
  ```bash
  ros2 param get /global_costmap/global_costmap plugins
  # Expected: ['static_layer', 'obstacle_layer', 'inflation_layer']
  ```
  > **Confirmed**: `['static_layer', 'obstacle_layer', 'inflation_layer']`.

- [x] **Planner uses max_planning_time (not ms variant)** — was silently ignored before; now should actually apply the 5 s limit.
  ```bash
  ros2 param get /planner_server GridBased.max_planning_time
  # Expected: 5.0
  # If key doesn't exist or returns error, the rename may not have taken effect
  ```
  > **Confirmed**: `5.0`. Rename took effect correctly.

- [x] **Send a navigation goal** — 2D Nav Goal in RViz. Robot should plan and drive without crashing.
  **[2026-08-27] The car drove itself under Nav2 for the first time.** Four goals across three
  launches. Best run (`nav2_drive4`): goal (−3.902, −2.459) yaw −178.85°, **10.10 s of commands,
  203 of 203 reaching the VESC**, path **5.781 m**, net displacement 4.555 m, no crash.
  **It did not formally reach the goal**: it stopped **0.379 m** short (tolerance 0.25 m) with
  heading error **4.8°** (tolerance 14.3°) — heading good, position short. Two leads, neither yet
  confirmed: the last command before it quit was **0.269 m/s**, and this car's measured ground
  breakaway is 0.20–0.26 m/s, so RPP decelerated into the physical floor; and
  `movement_time_allowance: 100.0` (default 10.0) means Nav2 then idles for 100 s instead of
  declaring failure and firing a recovery. Bags `nav2_drive3` / `nav2_drive4` on the SSD (SIGINT-
  stopped, so no `metadata.yaml` — read the `.db3` with sqlite).
  **Prerequisite learned the hard way**: a goal clicked before the servers reach lifecycle ACTIVE is
  discarded silently (bug-126), and a goal clicked after `f1tenth_container` has aborted (bug-257)
  looks identical. Check `ros2 node list | grep bt_navigator` before debugging a dead goal.
  **[2026-08-27 evening, parked on AC — no battery, no VESC, no joystick]** Three further goals,
  and the two blockers above are closed. **bug-257 is fixed** (`aced708`): the abort was
  `--use_multi_threaded_executor` on `f1tenth_container` triggering the rclcpp action-client race
  ros2/rclcpp#2242, not composition and not Nav2; the flag is now the `container_multi_threaded`
  argument, default `False`, and composed runs took 3 × 70 s goals across 2 launches with no abort.
  **`movement_time_allowance: 10.0` is verified on hardware**: `FollowPath` failed at 10.01, 10.22,
  10.01, 10.02 and 10.03 s. Safety held throughout — `/vehicle/ackermann_cmd` carried **0 messages**
  against 2678 on `/drive`, because the `command_gate` never saw a joystick heartbeat.

- [x] **Recovery behaviours reached by the behavior tree itself** — not invoked directly.
  **[2026-08-27]** First observation, parked. On a progress-checker failure the BT ran the recovery
  ladder in order: `ClearingActions` (local + global costmap clears) → `Wait` (5.0 s) →
  **`BackUp` IDLE→RUNNING→FAILURE** at t+50.90 s. `BackUp` failing in 30 ms is correct — the car
  physically could not move. Read from `/behavior_tree_log` in `bag_parked2_nocomp`
  (`scripts/live_runs/RESUME_20260827_DRIVE.md` §4a); the bag has no `metadata.yaml` (SIGINT), so
  read the `.db3` with sqlite3.
  - Path should appear on map.
  - Robot should not oscillate or hesitate excessively at 10 Hz controller rate.
  > **[2026-08-26] The cited blocker is stale** — the `component_container_isolated` crash was fixed 2026-08-04. Live history since: **one** goal has ever driven this car under Nav2 (2026-08-06 21:47, a −44.8° right turn over 2.628 m, recorded as the bug-140 steering-sign verification). The 2026-08-10 demo attempt never got a clean goal — costmap out-of-bounds (bug-228) and the rotated-frame report (bug-231). Four fixes have landed since and **none has run as a set with Nav2**: bug-232 (VSLAM startup localization defaults False), bug-237 (map paths → `20260805`), bug-234 (AMCL yaw seed), bug-241 (auto-seed at t+20 s). Re-test is a drive-session item; pass `launch_twist_to_ackermann:=True` or `cmd_vel` never reaches the VESC.

- [x] **CPU load during navigation**
  **[2026-08-27] Measured live with a goal pending, `top -b -n2 -d2`, second block only.**
  **There is no Nav2 CPU problem.** `f1tenth_container` — which holds every Nav2 server under the
  default `use_composition:=True` — does not appear in the top 11 processes at all, i.e. under
  ~3.5 %. The load is RViz (70 %), `sensing_container` (50 %) and `visual_slam_container` (34–72 %).
  System 44.7 % idle with a goal pending vs 48.0 % idle after, load ~5.3 on 6 cores.
  **Why the old per-process figures cannot be reproduced**: under composition `planner_server` is
  **not a process**, it is a component inside a container — so bug-009's 94 % and even bug-127's
  corrected 2.5–5.5 % only apply to a non-composed launch. Ask for the container, not the node.
  *(Original entry retained below for history.)*
   — with the reduced costmap frequencies, CPU on Nano should be lower than before. **[2026-08-04] Inconclusive**: stationary with no goal ever sent, `planner_server` appeared to run at 94.1% of one core. **[2026-08-06] That measurement was wrong (bug-127)** — `top -b -n1` reports cumulative-since-start, not an interval, and zombies were counted as live copies. Corrected: `planner_server` peaks ~100% of one core **during `on_configure`** while the global costmap is built, and sits at **2.5–5.5%** while actually planning. There is no standing CPU defect. What remains is a clean pre/post-tuning comparison on a quiet machine with a real goal pending — sample with `top -b -n2 -d2` and read only the second block.
  ```bash
  top -b -n1 | grep -E 'controller|costmap|planner'
  ```

---

## 10. Integration — Full Bringup

- [x] **Teleop mode: full stack launches**.
  **[2026-08-27] PASS.** `bringup.launch.py slam:=False launch_navigation:=False`, 41 nodes.
  Full chain resolves from `map`: `odom`, `base_link`, `lidar`, `camera_link`, `imu_link`,
  `base_footprint`, `front_axle`. `/odometry/local` 30.0 Hz, `/visual_slam/tracking/odometry`
  30.0 Hz, `/lidar/scan_filtered` 8.7 Hz, `/odom/rf2o` 8.2 Hz (the ~8 Hz YDLidar rate on this
  Jetson, not CPU starvation).
  ```bash
  ros2 launch f1tenth_launch bringup.launch.py slam:=False launch_navigation:=False
  ros2 run tf2_tools view_frames
  # Expected TF chain: map -> odom -> base_link -> lidar, camera_link, etc.
  ```

- [x] **Mapping mode: no TF conflicts**. **[2026-08-30]** PASS. 0 TF extrapolation / TF_REPEATED / authority warnings, **0 duplicate node names**, no `/amcl` or `/map_server` under `slam:=True`, and `/map` has **1 publisher** (`slam_toolbox` only — bug-027 closed). 42 nodes. The double-launch (bug-260) was `teleop.launch.py`'s localization `TimerAction`: its `IfCondition(launch_localization)` was evaluated ~10 s after `launch_setup`, by which time the enclosing `GroupAction` scope carrying bringup's `'False'` had been popped, so it resolved against bringup's top-level `'True'`. Fixed by freezing the value at `launch_setup` time. Verified with `bringup.launch.py slam:=True launch_navigation:=False use_gpu:=False launch_2d_mapping:=True`.
  ```bash
  ros2 launch f1tenth_launch bringup.launch.py slam:=True launch_navigation:=False use_gpu:=False
  # Watch for TF extrapolation errors or duplicate TF warnings in the log
  ```

- [x] **odom_tf_publisher and map_tf_publisher propagate through bringup**: **[2026-08-04] PASS** — with `odom_tf_publisher:=rf2o map_tf_publisher:=rtabmap`, `rf2o_laser_odometry` gains `/tf` in its publisher list (vs. absent under the default) and `ekf_odom_node` reports `publish_tf: False`. **Caveat when re-testing**: `ros2 node info /ekf_odom_node` still lists `/tf` under Publishers even in this mode — robot_localization constructs the broadcaster unconditionally and simply does not broadcast. Check the `publish_tf` **param**, not the publisher list. Run was un-namespaced.
  ```bash
  ros2 launch f1tenth_launch bringup.launch.py odom_tf_publisher:=rf2o map_tf_publisher:=rtabmap slam:=False
  ros2 param get /rf2o_odometry_node publish_tf
  # Expected: true (rf2o is now the odom TF publisher)
  ```

---

## Known Bugs / Issues Found

<!-- Add entries here as bugs are discovered during testing -->
<!-- Format: - [DATE] Short description. Reproduction steps. -->

- [2026-05-19] **Throttle interpolator breaks steering**. When `launch_throttle_interpolator_node:=True`, steering does not respond to joystick — car tries to return to center. Throttle/acceleration is limited correctly. Repro: launch teleop with `launch_throttle_interpolator_node:=True`, push left stick sideways. TODO: inspect throttle_interpolator source for steering passthrough logic.

- [2026-05-19] **`joy_teleop` node not discoverable in `ros2 node list`** (rmw_fastrtps_cpp). `/gosling1/teleop` publishes and drive works. Run `ros2 component list` to confirm it's loaded inside `f1tenth_container`. May be a fastrtps visibility limitation for intra-process components.

- [2026-05-19] **`ros2 topic info -v /gosling1/safety` shows `Publisher count: 0`** despite 40 Hz traffic confirmed via `ros2 topic hz`. Using rmw_fastrtps_cpp; likely a discovery artifact for intra-process component publishers. Safety fallback tested and confirmed functional.

- [2026-05-19] **`gyro_fps: 100` rejected by D435i hardware**. D435i gyroscope only supports 200 Hz and 400 Hz; setting 100 Hz is invalid. Camera falls back to 200 Hz silently at startup (error appears explicitly during runtime reconfiguration). Fixed: updated `gyro_fps: 200` in `config/sensors/realsense_config.yaml`. `accel_fps: 100` is valid and unaffected.

- [2026-05-19] **SDK pointcloud (`camera/depth/color/points`) not published** — retest after clean Jetson restart. "No stream match" warning appeared only during a GLSL runtime restart (transient USB 2.1 re-enum). On clean USB 3 startup, color at 640×360×30 + depth at 640×360×30 may resolve the stream mismatch. `accelerate_gpu_with_glsl: true` must be set at startup (not runtime) since it IS compiled in.

- [2026-05-19] **`rtabmap_depth_to_pointcloud_xyzrgb` had no remapped inputs** — `SetRemap` with `condition=IfCondition(...)` is unreliable inside `GroupAction` in ROS2 Humble. Fixed: removed conditions from all input SetRemap calls in `stereo_and_depth_image_processing.launch.py` and replaced `LaunchConfiguration` dst values with resolved Python strings. Node now correctly subscribes to `camera/aligned_depth_to_color/image_raw` and `camera/color/image_raw`. Retest `camera/downsampled_cloud_from_depth` after rebuild.

- [2026-05-24] **VESC steering servo clips on full-left joystick deflection**. With default `max_steering:=0.34` rad, full left stick sends −0.34 rad → servo = −1.4×(−0.34)+0.56 = 1.036, which is clipped to max 0.92. Root cause: `steering_angle_to_servo_offset: 0.56` shifts the servo center above 0.5, giving an asymmetric physical range: [−0.257 rad left, +0.343 rad right]. The default `max_steering=0.34` is within the right-side limit but exceeds the left-side limit (0.257 rad). The car steers to mechanical maximum on both sides (clipping is benign since the VESC handles it correctly), but the log is noisy. Options: (a) reduce `max_steering` default to 0.25 for symmetric no-clip behavior at the cost of ~25% less right-turn range; (b) recalibrate `steering_angle_to_servo_offset` toward 0.5 for symmetric range; (c) accept the clip as-is. TODO: recalibrate servo offset after verifying physical center position with VESC Tool.

- [2026-05-24] **YDLidar X4 does not support per-return intensity**. `intensity: true` causes continuous checksum errors and driver crash. `intensity: false` produces constant 1008.0 values (internal status bits, not real intensity). The intensity filter in `laser_filter.yaml` is unusable for this sensor. Fixed: `intensity: false` confirmed in `ydlidar_X4.yaml`; intensity filter entry left commented out permanently.

- [2026-05-24] **RealSense pointcloud required `color_format: RGB8`**. `color_format: BGR8` caused "No stream match for pointcloud chosen texture Process - Color" and no pointcloud output. Fixed: `color_format: "RGB8"` in `realsense_config.yaml`. SDK pointcloud (`camera/depth/color/points`) now publishes ~27 Hz at ~72% CPU. RTABMap `depthimage_to_pointcloud` path publishes ~49 Hz but uses ~200% CPU — prefer SDK path.

- [2026-05-26] **rf2o node name is `CLaserOdometry2DNode`, not `rf2o_odometry_node`**. `ros2 node list | grep rf2o` returns empty. Use `grep CLaserOdometry` to find the node. The forked package (https://github.com/privvyledge/rf2o_laser_odometry.git) may rename it; verify after switching.

- [2026-05-26] **rf2o publishes `odom→base_link` TF despite `publish_tf: False`**. Upstream rf2o (`CLaserOdometry2DNode`) appears on `ros2 topic info /gosling1/tf` as a publisher even when `publish_tf` is confirmed false via `ros2 param get`. This creates a competing TF alongside the EKF and causes ~0.5°/s yaw drift while the robot is stationary. Fix: switch to the forked rf2o (https://github.com/privvyledge/rf2o_laser_odometry.git) which fixes TF publication control. **RESOLVED (2026-07-22)**: fork pinned in `f1tenth.repos`.

- [2026-05-26] **rf2o publishes zero covariance**. All 36 elements of both pose and twist covariance in `/odom/rf2o` are 0.0. robot_localization EKF treats zero covariance as infinite precision, heavily biasing the fused estimate toward rf2o. Fix: the forked rf2o adds proper covariance estimation. Until deployed, consider adding a `odom2_pose_rejection_threshold` and `odom2_twist_rejection_threshold` in `ekf_odom.yaml`, or setting `odom2_config` to only fuse twist (not pose) to reduce exposure. **RESOLVED (2026-07-22)**: fork pinned in `f1tenth.repos` (adds covariance); odom2 rejection thresholds also added in `cb3b567`.

- [2026-05-26] **EKF `odom2_differential` and `odom2_relative` both true in `ekf_odom.yaml`**. robot_localization warns "Both odom2_differential and odom2_relative were set to true. Using differential mode." Same issue on `imu0_differential`/`imu0_relative`. These flags are mutually exclusive. Fix: set `odom2_relative: false` and `imu0_relative: false` (keep only `_differential: true`).

- [2026-05-26] **rf2o verbose INFO logging at every scan cycle**. Two INFO lines per ~100 ms ("execution time" + "Laser odom [x,y,yaw]") flood the terminal. To suppress without code changes, set the logger level at launch: add `--ros-args --log-level gosling1.CLaserOdometry2D:=WARN` and `--log-level gosling1.CLaserOdometry2DNode:=WARN`, or configure via `log_level` arg in the launch file. The forked package may expose a `verbose` parameter.

- [2026-05-27] **`nvblox_node` and `visual_slam_node` launched despite `use_gpu:=False`** in `mapping.launch.py`. Two interacting root causes: (1) `teleop_launch` `launch_arguments` had `"use_gpu": 'True' if (...or not enable_odom_here) else 'False'` — when `launch_localization=True`, this forced `use_gpu='True'` into the shared `context.launch_configurations` (ROS2 `IncludeLaunchDescription` `launch_arguments` are not context-scoped); (2) `mapping_3d_gpu_node` used `condition=IfCondition(use_gpu)` — a lazy `LaunchConfiguration` substitution evaluated after the action list, by which time the context was already mutated to `'True'`. **Fixed**: (1) changed `teleop_launch` arg to pass `use_gpu` as-is; (2) replaced both `mapping_3d_cpu_node` and `mapping_3d_gpu_node` conditions with an eager string `gpu_enabled` computed at OpaqueFunction time, immune to downstream context mutations.

- [2026-05-27] **`lifecycle_manager_localization` waits indefinitely for AMCL in mapping mode**. In `localization.launch.py`, the `lifecycle_nodes` logic appended `'amcl'` when `map_tf_publisher_str == 'amcl'`, even when `launch_amcl=False`. Since `mapping.launch.py` defaults `map_tf_publisher='amcl'` but `launch_global_localization=False` (which controls `launch_amcl`), the lifecycle manager managed a node that was never started. **Fixed**: removed the `or map_tf_publisher_str.lower() == 'amcl'` branch; now only adds `'amcl'` to lifecycle nodes when `launch_amcl_str == 'true'`.

- [2026-05-27] **Static TFs missing from recorded bags**. Bags recorded with the `gosling1` namespace (`loop3x_no_localization`) have `sensor_kit_link` disconnected from `base_link` in the TF tree — the static TF publishers were not running (or their latched `/tf_static` messages were missed) at bag record time. Breaks rf2o, EKF, and ICP odom during playback. **Workaround**: pass `launch_tfs:=True` when replaying. **Prevention**: confirm `ros2 run rqt_tf_tree rqt_tf_tree` shows a complete tree before starting the bag recording; use `--all-topics` and record for several seconds after all static publishers are active.

- [2026-05-29] **RTABMap localizer incompatible with VSLAM in same launch session**. Launching `launch_rtabmap_localizer:=True use_gpu:=True` fails with "parameter {image_qos} is of type {string}, setting it to {integer} is not allowed" on the VSLAM composable node. Root cause: upstream `rtabmap_launch/rtabmap.launch.py` uses `SetParameter(name='image_qos', value=<integer>)` globally, which bleeds into the shared launch context and conflicts with VSLAM's string-typed `image_qos`. Workaround: pass `use_gpu:=False` when using the RTABMap localizer — it does not need VSLAM. Not planned for active use; RTABMap localization deprioritized in favour of AMCL.

- [2026-05-30] **`component_container_isolated` crashes with `free(): invalid pointer` / SIGABRT during Nav2 bringup (bag replay)**. `visual_slam_node` runs in the same `f1tenth_container` as all Nav2 composable nodes. At 1× bag playback (default `--rate`) with a 30 fps stereo stream, the Jetson Orin Nano GPU falls behind: VSLAM processes only ~3–5 fps, causing 200–300 ms sim-time frame deltas. When VSLAM falls far enough behind it hits heap corruption and SIGABRT, which kills the entire container — taking `controller_server`, `planner_server`, costmaps, and all other composable nodes with it. **Fix options**: (1) **Architectural (permanent)**: move `visual_slam_node` to its own `component_container_isolated` (`vslam_container`) in `localization.launch.py` so a VSLAM crash cannot cascade into Nav2. Tradeoff: intra-process zero-copy between VSLAM and realsense nodes is lost (they communicate via DDS instead — adds ~1 copy per image frame). (2) **Lower bag playback rate**: add `--rate 0.3` so VSLAM has ~3× more wall time per frame; `--rate 0.2` confirmed stable in earlier tests (see Section 8c). (3) **Reduce camera FPS**: lower `stereo_fps` in `realsense_config.yaml` from 30 → 15 Hz to halve VSLAM GPU load at the source — applies to live robot as well.

- [2026-08-04] **Duplicate `sensing_container` name kills the RealSense (default composition path)**. With `use_composition:=True` (the bringup default), THREE containers are created with the name `sensing_container`: one by `sensors.launch.py` (default `component_container_name`, line 78), one by `realsense_d435i.launch.py` (its `realsense_d435i_container` default is overridden by the inherited value), and one by the VSLAM include in `localization.launch.py:914-915` which hardcodes `vslam_container_name = '/sensing_container'` with `attach_to_shared_component_container: 'False'` (so it *creates* rather than attaches). `LoadComposableNodes` targets containers **by name**, so the RealSense driver is loaded more than once → two `/camera` nodes → both open the same D435i → `failed to claim usb interface ... RS2_USB_STATUS_BUSY` → **the camera never starts**, and with it VSLAM and `camera/imu/filtered` are dead. ROS logs `WARNING: Be aware that are nodes in the graph that share an exact name`; `ros2 node list` shows `/camera` ×2, `/sensing_container` ×3, `/visual_slam_node` ×2. Repro: `ros2 launch f1tenth_launch bringup.launch.py slam:=False launch_navigation:=False launch_global_localization:=True`. **Workaround**: `use_composition:=False` — all node names unique, zero USB-busy errors, camera starts. **Fix**: give the VSLAM container a distinct name (restore the commented-out `component_container_name: 'realsense_d435i_container'` at `bringup.launch.py:795-797`), and never reuse one container name across independent creators. Note this defeats the `b47d45d` VSLAM-isolation intent.

- [2026-08-04] **`slam:=True` double-launches the entire localization stack**. `bringup.launch.py:943` includes `mapping.launch.py`, which at line 577 includes `teleop.launch.py`, which at line 645 includes `localization.launch.py` — while `bringup.launch.py:855` already included `localization.launch.py` directly. Result: `icp_odometry` ×2 and `stereo_odometry` ×2 (confirmed via `ps`), duplicate lifecycle managers, and the map_server lifecycle transition fails: `Unable to start transition 1 from current state active: Transition is not registered` → `Failed to change state for node: map_server` → `Failed to bring up all requested nodes. Aborting bringup.` Repro: `ros2 launch f1tenth_launch bringup.launch.py slam:=True launch_navigation:=False use_gpu:=False`. **Fix**: bringup should either include localization directly OR delegate via mapping/teleop, not both — gate the direct include on `not slam`, or stop `mapping.launch.py` from pulling in `teleop.launch.py` when the parent already launched localization.

- [2026-08-04] **`slam:=True` alone starts no 2D mapping**. `launch_2d_mapping` defaults to `False` (`mapping.launch.py:91`), so `bringup.launch.py slam:=True` brings up no `slam_toolbox`, no `/map` topic, and no `map_saver` — the map-save test silently has nothing to save. Must pass `launch_2d_mapping:=True` explicitly. Arguably `slam:=True` should default at least one mapping backend on, or warn loudly when both `launch_2d_mapping` and `launch_3d_mapping` are false.

- [2026-08-04] **`planner_server` consumes ~94% of a CPU core while idle**. Observed stationary, with no navigation goal ever sent: `planner_server` at 94.1% in `top`, system load 12.60 on a 6-core Orin. Repro: `ros2 launch f1tenth_launch bringup.launch.py slam:=False launch_navigation:=True launch_global_localization:=True`, wait 2 min, run `top -b -n1`. A planner with no active goal should be near-idle; suspect the global costmap update loop or a planner spin without goal gating. Caveat: a concurrent MPC sim was running on the same host, so total load is not attributable to this stack alone — but the 94% figure is per-process.

- [2026-08-04] **VESC driver respawns in a tight loop with no backoff**. With `/dev/sensors/vesc` absent (motor unpowered), `vesc_driver_node` logs `FATAL ... Failed to connect to the VESC` and is immediately respawned — ~30 restarts observed in a single launch log, several per second. `use_respawn:=True` in bringup has no `respawn_delay`. Adds log noise and churn during any VESC disconnect. **Fix**: set a `respawn_delay` (e.g. 2.0 s) on the VESC node.

- [2026-08-04] **A topic literally named `/dev/null` is created**. `ros2 topic list` includes `/dev/null`. It is also present in the May-2026 recordings (`loop3x_no_localization` metadata), so this is long-standing, not new. Something is passing a log/file path where a topic name is expected (likely a remap or an output redirect string reaching a topic argument). Harmless but pollutes the graph — worth tracking down.

- [2026-08-04] **Local EKF diverges catastrophically if the VESC odometry source drops out**. Stationary with the VESC unpowered, `/odometry/local` drifted **3012 m in 58 s** with twist pegged at (+22.8, −41.9) m/s, while `/odom/rf2o` showed 1.27 m and `/visual_slam/tracking/odometry` 0.05 m of drift — i.e. the EKF manufactured ~48 m/s from inputs reading ~0.004 m/s; pose covariance reached 7.5e6. Root cause: `imu1_config` in `ekf_odom.yaml` fuses linear acceleration (`ax, ay, az = true`) from `camera/imu/filtered` at 200 Hz with `imu1_linear_acceleration_rejection_threshold` **commented out**; the D435i sits ~3.3° off level, so ≈0.57 m/s² of gravity leaks into the horizontal axes after gravity removal (0.57 m/s² × 70 s ≈ 40 m/s, matching the observed velocity). The only thing normally bounding this is `odom0_config`, which fuses **vx and vy** from `vehicle/vesc_odom` (vy≈0, nonholonomic). **This does not occur in normal operation with the VESC powered** — but a mid-run VESC dropout (USB glitch, brownout) would diverge within seconds instead of degrading gracefully. **Fix**: uncomment/set `imu1_linear_acceleration_rejection_threshold`, and consider dropping `az` (2D robot) or adding a zero-velocity update when no odometry source is alive.

- [2026-08-04] **[Environment, not a repo bug] RealSense needs a working GL context inside the container**. `realsense2_camera_node` fails with `Error starting device: Could not open OpenGL window, please check your graphic drivers or use the textual SDK tools` — **even with `accelerate_gpu_with_glsl:=false` passed explicitly on the command line** (the wrapper in this image is compiled with GLSL support and initialises GL regardless; the device itself is detected fine — serial 140122071097, USB 3.2). Fix: mount the X socket into the container (`-v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=:0`) **and** grant access on the host with `DISPLAY=:0 xhost +local:`. Note `bolus_ws/f1tenth_launch.sh` runs its `xhost` line without `DISPLAY` set, which is why it emits `X Error of failed request: BadValue ... X_ChangeHosts` — worth fixing in that script.

- [2026-08-04] **[Tooling caveat] `ros2 node list` is unreliable under the static-peer CycloneDDS config**. With `CYCLONEDDS_URI=cyclonedds_config_static.xml` (`AllowMulticast=false`, `EnableMulticastLoopback=false`, unicast peers), `ros2 node list` / `ros2 param get` intermittently return empty or `Node not found` while the nodes are demonstrably healthy and publishing at full rate. Always cross-check with `ps -eo args` before concluding a node did not start. This differs from the older fastrtps intra-process visibility caveat above — here even non-composed nodes vanish.

- [2026-08-04] **Namespaced bringup double-namespaced the entire localization stack (`/gosling1/gosling1/*`)** — every localization node received no data. With `use_f1tenth_namespace:=True`, sensors/vehicle/camera/VSLAM correctly landed on `/gosling1/*` while `amcl`, `ekf_odom_node`, `ekf_map_node`, `map_server`, `lifecycle_manager_localization`, `rf2o_laser_odometry` and `rtabmap_icp_odom` landed one level deeper and starved (`rf2o`: `Waiting for laser_scans....`; `/gosling1/gosling1/odometry/local`: NO DATA). Root cause: **the launch-config inheritance leak, sibling-to-sibling via `ld` ordering** — `bringup.launch.py`'s `ld` is `[nodes_to_launch, sensors_launch, localization_launch]`; the sensors include passes `use_namespace: True` / `namespace: gosling1` as `launch_arguments`, which leak into the shared context, so `localization_launch`'s `PushRosNamespace(condition=IfCondition(use_namespace), ...)` — dormant by default since `use_namespace` defaults `False` — fires on top of the namespacing `localization.launch.py` already performs. `vehicle_bringup_group` escaped only because `nodes_to_launch` is visited *before* the leak. Bisect: standalone `localization.launch.py` → `/gosling1` ✓; bringup with `launch_sensors:=False` → `/gosling1` ✓; with `launch_sensors:=True` → `/gosling1/gosling1` ✗. Delaying sensors past localization's 10 s timer did **not** help — this is description-build time, not runtime ordering. **Fixed (2026-08-04, uncommitted)**: removed the `PushRosNamespace` from the localization and mapping includes in `bringup.launch.py` (the children namespace themselves), keeping the `SetRemap` `/tf` pairs. `teleop.launch.py` was already correct — it places `sensors_launch`/`localization_launch` outside its pushing `GroupAction`. **Lesson**: when a parent and child both handle namespacing, only one may push; and a *sibling* include visited earlier can silently arm a condition in a later one.

- [2026-08-04] **Under a namespace, the `slam:=True` duplicate localization stack lands at root `/`** (refines the double-launch entry above). `mapping.launch.py:645` passes `use_namespace: 'False'` to its teleop include, so with `use_f1tenth_namespace:=True` the second copy is created un-namespaced: 8 root-level nodes (`amcl`, `ekf_odom_node`, `ekf_map_node`, `map_server`, `lifecycle_manager_localization`, `rf2o_laser_odometry`, `rtabmap_icp_odom`, `rtabmap_stereo_odom`). Verified **inert** — `/odometry/local`, `/odom/rf2o`, `/tf` and `/map` at root all read NO DATA — so it wastes CPU/RAM but does not corrupt `/gosling1/tf`. Side effect: the `map_server` lifecycle abort documented above does **not** occur in namespaced mode, because the duplicate stack carries its own root-level `map_server`. Repro: `ros2 launch f1tenth_launch bringup.launch.py slam:=True launch_2d_mapping:=True use_f1tenth_namespace:=True`.

- [2026-08-04] **RESOLVED — `sensing_container` name collision fixed.** Two edits removed all three same-name creators: (1) `localization.launch.py` — `vslam_container_name` changed from `'{ns}/sensing_container'` to `'{ns}/visual_slam_container'` (VSLAM was told to *create* a container using the RealSense's name, despite the adjacent comment saying it should be isolated; the non-composed branch already used `visual_slam_container`); (2) `sensors.launch.py` — the `stereo_and_depth_image_processing` include now passes `attach_to_shared_component_container: 'True'` instead of inheriting `False`, since both it and `realsense_d435i.launch.py` create their container under `condition=UnlessCondition(attach_to_shared_component_container)` and so produced two `sensing_container` nodes; attaching also preserves intra-process zero-copy with the RealSense driver. **Verified on-robot, namespaced, `use_composition:=True`**: every container name unique (`sensing_container`, `visual_slam_container`, `localization_container`, `command_gate_container` — one each, previously `sensing_container` ×3), exactly one `/gosling1/camera`, 0 USB-busy errors, `camera/imu/filtered` 199.8 Hz, `visual_slam/tracking/odometry` 30.02 Hz, `odometry/local` 30.001 Hz — the first end-to-end success of the **default** composed path. Regression-checked with `use_composition:=False`: 0 errors, camera 200.1 Hz, `odometry/local` 30.001 Hz.

- [2026-08-04] **[Superseded by the entry above — kept for the reasoning] BUG-006 is a race, not a deterministic failure.** A namespaced run with `use_composition:=True` came up **clean**: zero `RS2_USB_STATUS_BUSY` / "share an exact name" errors, `camera/imu/filtered` at 200.4 Hz, and exactly one `/gosling1/camera`. However `ros2 node list` still showed **`/gosling1/sensing_container` three times**, so the underlying duplicate-container-name defect is unchanged — the RealSense simply happened to load into only one of them. Do not treat a single successful composition run as evidence the bug is fixed; the name collision must still be removed. Note `reset_realsense:=True` was set in that run and may also mask the symptom.

- [2026-08-04] **[Amendment] `planner_server` 94% CPU did NOT reproduce.** Re-measured namespaced with a quiet host: `planner_server` sat at **0–4.5% instantaneous** (24% averaged over its 1:53 lifetime, front-loaded during costmap initialisation), system load 5.71 vs the earlier 12.60. The earlier 94.1% coincided with the runaway local EKF (BUG-010) feeding garbage odometry into the global costmap, which would drive continuous replanning. **Hypothesis: the idle-CPU bug is a symptom of BUG-010, not an independent defect.** Confirm once the VESC is powered and the EKF has a velocity anchor.

- [2026-08-04] **RealSense wedges after repeated container restarts; `reset_realsense:=True` recovers it.** After several `docker restart` cycles the D435i enumerates and the `camera` node starts, but no stream ever publishes (`camera/imu/filtered`, `infra1/2` all silent, `imu_filter_madgwick`: `Still waiting for data on topic imu/data_raw...`) with no error in the log. Passing `reset_realsense:=True` on the next launch restores it (200 Hz IMU). Worth defaulting on for test sessions that cycle the container.

- [2026-05-27] **`map_saver` does not auto-save map in namespaced mapping mode**. During 2D mapping via bag playback with `use_f1tenth_namespace:=True`, the map was not automatically saved by the `map_saver` node. Manual save via `ros2 run nav2_map_server map_saver_cli -f <path> -t /gosling1/map --ros-args -p map_subscribe_transient_local:=true` worked. Likely cause: map_saver is subscribed to `/map` (unnamespaced) rather than `/gosling1/map`. TODO: verify map_saver topic remapping in `2d_mapping.launch.py` and add namespace handling. **RESOLVED (2026-07-22): hypothesis refuted** — offline launch-tree resolution shows `map_saver_server` correctly lands in `/gosling1` (via `PushRosNamespace` in `mapping.launch.py`; the hardcoded `namespace: ''` in the include args intentionally avoids double-nesting) and its relative `map_topic: map` resolves to `/gosling1/map`. Real cause: `map_saver_server` never auto-saves — it only saves on a `save_map` service call (`/gosling1/map_saver/save_map`); the `map_url` launch param is ignored (not a declared node parameter). Use the namespaced service call or `map_saver_cli -t /gosling1/map` to save.

- [2026-08-04] **BUG-017 — `teleop.launch.py` with `use_composition:=True` starts no camera at all** (FIXED). No `sensing_container` is ever created, there is no `realsense2_camera_node` process, and `camera/imu/filtered` / `camera/infra1/image_rect_raw` read NO DATA. `visual_slam_node` loads but starves (`visual_slam/tracking/odometry` NO DATA), and `odometry/local` degrades to ~10 Hz on rf2o alone. **No error is logged** — `LoadComposableNodes` silently waits on a container that never appears. Repro: `ros2 launch f1tenth_launch teleop.launch.py use_f1tenth_namespace:=True use_composition:=True reset_realsense:=True`, then `ros2 component list`. Cause: `teleop.launch.py:591-592` told the sensors include to *attach* to `sensing_container`, but only `container_name` (`f1tenth_container`) is ever created (line 504), and `realsense_d435i.launch.py` creates its container under `UnlessCondition(attach_to_shared_component_container)` so it stood down too. `bringup.launch.py` was immune only because its equivalent overrides (lines 794-797) are commented out. Fixed: teleop now passes `attach_to_shared_component_container:='False'`, letting RealSense create and own `sensing_container` as it does under bringup. Verified: camera IMU 200.376 Hz, color 36.387 Hz, VSLAM 31.147 Hz, `odometry/local` 30.038 Hz, cov 1.768e-4.

- [2026-08-04] **BUG-018 — duplicate `f1tenth_container` under composed teleop** (FIXED). `ps` showed two containers named `f1tenth_container` at `/gosling1` (one `component_container_isolated`, one `component_container_mt`) while `ros2 component list` showed one; the `_mt` duplicate loaded nothing. Cause: `localization.launch.py:973-978` creates a container named `container_name` under `IfCondition(use_composition and not attach_to_shared_component_container)`, and teleop's localization include passed `container_name` but **inherited** `attach_to_shared_component_container`. Before the BUG-017 fix that inherited value happened to be `'True'` — leaked sideways from the sensors include visited earlier in teleop's `ld` (the BUG-013 mechanism) — which kept the second creator dormant by luck. Fixing BUG-017 flipped it to `'False'` and the duplicate appeared. Fixed: teleop now passes `attach_to_shared_component_container` explicitly to the localization include. Verified: four uniquely-named containers, one process each, 0 `process has died`.

- [2026-08-04] **BUG-019 — SAFETY: `slam:=True` runs TWO `command_gate` nodes, both publishing the actuation topic** (FIXED). Repro: `ros2 launch f1tenth_launch bringup.launch.py slam:=True launch_2d_mapping:=True use_f1tenth_namespace:=True use_composition:=True`, then `ros2 component list` → `/gosling1/command_gate` listed twice inside `/gosling1/command_gate_container`; `ps` shows two `command_gate_container` processes; `ros2 topic info -v /gosling1/vehicle/ackermann_cmd` → **`Publisher count: 2`**; the log has two `Loaded node '/gosling1/command_gate'` lines. Cause: `bringup.launch.py` includes `vehicle/command_gate.launch.py` directly *and*, under `slam:=True`, includes `mapping.launch.py` → `teleop.launch.py` → `command_gate.launch.py` again. Bringup's mapping include already suppresses the other double-launched subsystems (`launch_joystick`/`launch_sensors`/`launch_vehicle`/`launch_tfs`/`launch_localization` all `'False'`); `launch_command_gate` was simply missing from that list. **Unlike the rest of the BUG-007 duplicates, which land inert at root `/`, both `command_gate` copies land at `/<namespace>`** and both publish `vehicle/ackermann_cmd`, each with its own heartbeat watchdog — so one can publish zero-initialized commands while the other passes real ones. Harmless with the VESC unpowered; an actuation hazard as soon as the motor is powered and driving in mapping mode. Fixed: bringup's mapping include now also passes `"launch_command_gate": 'False'`. Verified `slam:=True`: 1 load / 1 container / `Publisher count: 1`, mapping healthy (`/gosling1/map` 4.569 Hz, `odometry/local` 31.865 Hz). Regression-checked `slam:=False`: 1 load / 1 container / `Publisher count: 1`, `odometry/local` 30.003 Hz.

- [2026-08-04] **BUG-020 — nav2 is double-namespaced on the composed path, and the entire nav2 lifecycle bringup aborts** (FIXED). Repro: `ros2 launch f1tenth_launch bringup.launch.py slam:=False launch_navigation:=True launch_global_localization:=True use_f1tenth_namespace:=True use_composition:=True`. Every nav2 node came up as `/gosling1/gosling1/<server>`, then: `controller_server`: `Couldn't load critics! Caught exception: No critics defined for FollowPath` → `Caught exception in callback for transition 10` → `Lifecycle node controller_server does not have error state implemented`; `lifecycle_manager_navigation`: `Failed to change state for node: controller_server` → **`Failed to bring up all requested nodes. Aborting bringup.`** All 7 servers stayed `unconfigured`, `global_costmap` never published a `costmap` topic, and 15 topics sat under `/gosling1/gosling1/`. Cause: `bringup.launch.py:1002-1006` wraps the nav2 include in a `PushRosNamespace` **and** passes `namespace=` down. `nav2_navigation.launch.py` computes an absolute `node_ns` (`/gosling1`, line 72) and also has the relative `namespace` (`gosling1`). The non-composable `Node` entries use `namespace=node_ns` — and **an absolute namespace is immune to an enclosing `PushRosNamespace`**, which is why that path was always fine. The composable `ComposableNode` entries used `namespace=namespace` (relative), so the push prepended the namespace a second time. The doubled name means `nav2_params.yaml` (keyed `controller_server:` under `RewrittenYaml(root_key=namespace)`) no longer matches the node, so it configures with an empty critics list and throws — and one server throwing in `on_configure` aborts *all* of nav2. **This is exactly the BUG-017 class of silent wiring failure**, and it is why C1 passed at `use_composition:=False` last session and failed at `use_composition:=True`. Fixed: all 8 `ComposableNode` entries now use `namespace=node_ns`, matching the non-composable path; bringup needed no change, and nav2 is now correct standalone or under any namespacing parent. Verified composed + namespaced: names `/gosling1/<server>`, all 8 components in `/gosling1/f1tenth_container`, all 7 servers `active [3]`, **0** `/gosling1/gosling1/*` topics, and `/gosling1/global_costmap/costmap` present (previously absent). **Lesson: when a parent pushes a namespace, children must use absolute namespaces — relative ones silently double.**

- [2026-08-04] **BUG-021 — RTABMap builds no map: QoS reliability mismatch on its odometry input** (**FIXED 2026-08-04**, see the resolution note appended to this entry). Repro: `ros2 launch f1tenth_launch bringup.launch.py slam:=True launch_3d_mapping:=True launch_2d_mapping:=False use_gpu:=False use_f1tenth_namespace:=True use_composition:=True reset_realsense:=True`. Over an 88-minute run `/gosling1/map` had **NO DATA** and `rtabmap: Did not receive data since 5 seconds!` was logged **1055 times, continuously from startup**. The log also carries `New publisher discovered on topic '/gosling1/rtabmap/odom', offering incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS`. Confirmed twice with `ros2 topic info -v /gosling1/rtabmap/odom`:

  ```
  Type: nav_msgs/msg/Odometry
  Publisher  : rgbd_odometry            Reliability: BEST_EFFORT
  Subscribers: rtabmap, rtabmap_viz     Reliability: RELIABLE     -> nothing delivered
  ```

  Incompatible QoS means DDS delivers **nothing**, so RTABMap's synchroniser never fires and no node is ever added to the map. This is not a camera fault: `rgbd_sync` starved only at startup and during an unrelated 87 s WiFi outage (20 warnings total), so the RGB-D input was live for most of the run — the odometry input was dead throughout. **Not fixed** — needs a decision on which node should own RTABMap's odometry input: (a) match QoS on rtabmap's subscription, (b) publish `/rtabmap/odom` RELIABLE from `rgbd_odometry`, or (c) feed RTABMap the EKF's `odometry/local` instead (`mapping.launch.py` already computes `enable_odom_here` for exactly this question). **Not yet checked against `use_composition:=False`** — it looks composition-independent, but that is untested. Note this means **the primary mapper has never produced a map in any composed test**; only the SLAM Toolbox 2D path (Session B) has.

  **[RESOLUTION 2026-08-04] Option (a) applied — `mapping.launch.py`, `qos_odom: "1"` → `"2"`.** The mismatch is built in by construction: upstream `rtabmap_launch/rtabmap.launch.py:183` gives the `rgbd_odometry` node `"qos": qos_image`, and in `rtabmap_odom` that **single** `qos` parameter drives both its image subscriptions *and* its `odom` publisher. `mapping.launch.py` passes `qos_image = realsense_qos_int = 2` (BEST_EFFORT, correct for the RealSense), so `rtabmap/odom` is published BEST_EFFORT while `qos_odom` was hardcoded `'1'` (RELIABLE) for the subscription. **This rules out candidate (b)**: making `rgbd_odometry` publish RELIABLE requires setting its `qos` to 1, which would simultaneously make its image subscriptions RELIABLE against a BEST_EFFORT RealSense — trading a dead odom link for a dead image link. Candidate (c) was applied separately and for a different reason (see BUG-026).

  | Check | Before | After (composed) | After (`use_composition:=False`) |
  |---|---|---|---|
  | `incompatible QoS` warnings | present | **0** | **0** |
  | `rtabmap/odom` subscriber reliability | RELIABLE vs BEST_EFFORT pub | **BEST_EFFORT (matched)** | — |
  | `rtabmap/odom` rate | nothing delivered | **2.90 Hz** | — |
  | `/gosling1/map` | NO DATA for 88 min | **0.443 Hz** | **0.102 Hz, 1 publisher** |
  | `rtabmap:` starvation warnings | 1055, continuous | **2, startup only** | **1, startup only** |

  Non-composed additionally shows `Memory.cpp:2088::forget()` — rtabmap is actively adding signatures, i.e. genuinely building a map. **Honest limit on the composition question:** the *fix* is verified on both paths, but the original *failure* was never reproduced at `use_composition:=False` (the fix was already deployed by then), so "composition-independent" remains inferred from the wiring being composition-independent rather than directly observed.

- [2026-08-04] **BUG-026 — two nodes broadcast `odom→base_link` under `slam:=True`** (**FIXED 2026-08-04**). Measured **32.70 Hz** on that TF edge = `ekf_odom_node` (30 Hz) + `rgbd_odometry` (2.9 Hz), with `publish_tf: True` confirmed on both by `ros2 param get`. Root cause is an **overloaded flag**: `mapping.launch.py` derived `enable_odom_here` from `launch_localization`/`launch_local_localization`, but those answer *"should I START localization?"*, **not** *"does odometry EXIST?"*. `bringup` starts the EKF itself and therefore passes both as `False` to avoid a second copy (the BUG-007/BUG-016 lesson) — so mapping concluded nobody was providing odometry, stood up its own visual odometry with `odom_topic='rtabmap/odom'` and `publish_odom_tf=True`, and competed with the EKF for the edge. **`enable_odom_here` was therefore always `True` under bringup**, meaning `mapping.launch.py`'s existing reasoning about this never actually fired in the path that matters. Fixed by adding an explicit `external_odometry` launch arg that carries the second question; bringup passes `'True'`. Verified: `rgbd_odometry` gone, `ekf_odom_node` sole broadcaster, `odometry/local` unchanged at **30.000 Hz**. Stationary this is nearly invisible; under motion the two broadcasters disagree and the TF buffer returns whichever arrived last — it would have corrupted the A5 loop-closure drive.

- [2026-08-04] **BUG-027 — bringup runs the whole global-localization stack while SLAM is mapping** (**FIXED 2026-08-04**). Under `slam:=True`, `map→odom` had **three** broadcasters (`rtabmap` + `ekf_map_node` + `amcl`, 25.2 Hz total) and `ros2 topic info -v /gosling1/map` reported **`Publisher count: 2`** — `rtabmap` **and** `map_server`. So a map saved during a mapping run could have come from the stale file rather than the run. Root cause: `bringup.launch.py:75` defaults `launch_global_localization=True` and feeds it to `launch_ekf_map`/`launch_amcl`, with `launch_map_server=True`, and **nothing gates any of them on `slam`**. Fixed by computing `global_localization_effective` / `map_server_effective`, both forced `'False'` when `slam` is true. Verified: `/gosling1/map` publishers 2 → **1**; `amcl`, `ekf_map_node`, `map_server` absent; `rtabmap` sole `map→odom` broadcaster.
  **CLAUDE.md statement contradicted by this and corrected:** *"AMCL is off by default in bringup: `bringup.launch.py` defaults `launch_global_localization:=False`"* — it defaults **True** (`bringup.launch.py:75`).

  **[CORRECTION — earlier revision of this entry mis-attributed AMCL's TF broadcast to the YAML.]** Ownership of `map→odom` is decided by the **launch file, not the config**: `localization.launch.py:384/429` passes `tf_broadcast: True if map_tf_publisher == 'amcl' else False` *after* `params_file` in the `parameters` list, so it overrides the YAML. `map_tf_publisher` defaults to `'amcl'`, which is why a running AMCL was broadcasting — the YAML value was inert throughout. `localizer_amcl.yaml` has since been set to `tf_broadcast: false` with a comment, matching the existing `localizer_pf.yaml` convention, so the config no longer implies ownership. **This does not change BUG-027**: the defect was that AMCL, the map EKF and `map_server` were *running at all* during SLAM, not how AMCL's broadcast flag was set.

- [2026-08-04] **BUG-028 — an empty lifecycle node list aborts the ENTIRE launch on the non-composed path** (**FIXED 2026-08-04**). `[ERROR] [launch]: Caught exception in launch: Expected 'value' to be one of [float, int, str, bool, bytes], but got '()' of type '<class 'tuple'>'`, after which every node received SIGINT — sensors included. **Diagnostic trap:** the loudest thing in the log is `realsense2_camera_node` dying with a GLFW assertion (`_glfwPlatformGetTls`, exit `-6`) plus a cascade of `image_transport` plugin-load failures and `rcl node's context is invalid`. All of that is the *shutdown cascade*, not the cause; the real failure is a single line ~120 lines earlier. Root cause: `localization.launch.py` builds `lifecycle_nodes` at runtime from `launch_map_server` and `launch_amcl`. The **composable** lifecycle manager is guarded at its LaunchDescription entry (`*([load_composable_lifecycle_manager] if lifecycle_nodes else [])`) but the **non-composed `Node`** version was not, so with both servers off it was created with `node_names=[]`, which ROS 2 Humble rejects. Latent until BUG-027's fix turned both off under `slam:=True` — which is exactly why it appeared only at `use_composition:=False`. Fixed by applying the same guard idiom to the non-composed Node. Verified: `Caught exception in launch` count **0**, full stack up, `odometry/local` 29.876 Hz. This is the same hazard already documented for Nav2 ("an empty list crashes ROS 2 Humble") — it applies to the localization lifecycle manager too.

- [2026-08-04] **[Amendment to BUG-007] The root-level duplicates subscribe to root topics, which is why they starve forever.** The non-composed run log shows `rtabmap_icp_odom subscribed to /lidar/scan_filtered and /scan_cloud` — the **un-namespaced** paths, while the sensors publish under `/gosling1/`. Confirms the duplicates are not merely redundant but permanently starved, warning ~30×/run each while consuming CPU. `ps` shows the full set: **2 at `ns=/` and 2 at `ns=/gosling1`** for the stereo/ICP odometry pair.

- [2026-08-04] **BUG-022 — duplicate `f1tenth_container` in the RTABMap mapping path** (OPEN, not root-caused). During the BUG-021 run, `ps -eo args | grep component_container` showed **two** `component_container_isolated` processes with byte-identical arguments (`-r __node:=f1tenth_container -r __ns:=/gosling1`). `ros2 component list` returned empty on that run (BUG-012), so `ps` was the only thing that revealed it — the same detection asymmetry that exposed BUG-018. Same class as BUG-018 (two independent creators agreeing on one container name) and most likely another consequence of BUG-007, since `slam:=True` reaches the container-creating includes twice (directly, and via `mapping.launch.py` → `teleop.launch.py`). The `attach_to_shared_component_container` / `container_name` arguments on both paths need the same audit BUG-018 got. **Per the BUG-019 lesson, do not assume it is inert** — check whether both containers load components and whether any topic ends up with two publishers.
  - **[2026-08-04] ROOT-CAUSED AND FIXED.** Not `teleop.launch.py`, and not BUG-007: the second creator was `mapping.launch.py` itself. `bringup.launch.py:712` creates `f1tenth_container`; under `slam:=True` it included `mapping.launch.py` with `use_composition:=True`, and `mapping.launch.py:565` created its own container named `container_name` (default `f1tenth_container`) guarded **only** by `IfCondition(use_composition)` — the file had no `attach_to_shared_component_container` argument at all, so a parent had no way to say "I already made this". Both containers sit in a `nodes_to_launch` group that pushes the namespace, so both land at `/<ns>/f1tenth_container`.
  - **Correction to the original report: the argv are *not* byte-identical.** bringup's carries two `--params-file` entries (the nav2 params); mapping's carries `-p thread_num:=6`. That is exactly how you tell the two creators apart in `ps`.
  - **It was NOT inert — this was two full nav2 stacks.** `LoadComposableNodes` resolves its target by *service name*, and with two nodes offering `/<ns>/f1tenth_container/_container/load_node` **both** served the request. `/proc/<pid>/maps` showed the complete `libnav2_*.so` set (~60 libraries) mapped into **both** processes, and `local_costmap/costmap` + `global_costmap/costmap` each had **Publisher count 2**. Two `controller_server`s, two `planner_server`s, two lifecycle managers. **This is a strong candidate explanation for BUG-009 (`planner_server` 94% CPU) — re-test BUG-009 now that the duplicate is gone.**
  - **Detection lesson:** both `ros2 node list` and `ros2 component list` dedupe by node name and showed only ONE container. Only `ps -eo args` and `/proc/<pid>/maps` revealed it. When auditing composition, `/proc/<pid>/maps` is the tool that proves *which process actually loaded the components*.
  - **Fix:** `mapping.launch.py` gained an `attach_to_shared_component_container` argument (default `False`) and now gates container creation on `use_composition AND NOT attach_…`, the same pattern `teleop.launch.py` has used since BUG-018; `bringup.launch.py` passes `attach_to_shared_component_container` **and** `container_name` explicitly to the mapping include (anti-leak rule). Standalone `mapping.launch.py` is unchanged.
  - **Verified on-robot, before *and* after on the same command** (namespaced, `use_gpu:=False`, VESC unpowered): 2 → **1** `f1tenth_container`; four uniquely-named containers, one process each; all 8 nav2 components in the single container; costmap Publisher count 2 → **1**; `controller_server FollowPath.desired_linear_vel` still `0.5` (params still reach the servers); 0 launch exceptions; 0 `process has died`; `odometry/local` 30.020 Hz; `/<ns>/map` publishing, Publisher count 1. Two honest limits: `/<ns>/map` measured ~0.14 Hz vs session D's 0.443 Hz, explained by the robot being **stationary** (RTABMap only updates the grid on new signatures), not by the fix; and `/<ns>/rtabmap/odom` has no data, which is **expected** post-BUG-026 (`external_odometry:=True` means no `rgbd_odometry` node is started — confirmed by `ps`).

- [2026-08-04] **[Amendment to BUG-007] The "duplicate stack is inert" finding does NOT generalise.** In the RTABMap run the root-level duplicates `rtabmap_icp_odom` and `rtabmap_stereo_odom` were **actively running and warning** (~1030 un-namespaced log warnings each), i.e. consuming CPU rather than sitting idle. Together with BUG-019 (both duplicate `command_gate` nodes live, both publishing the actuation topic), this means **every member of the BUG-007 duplicate set must be audited individually for liveness** — the earlier "inert" verdict covered only the topics that were sampled (`/odometry/local`, `/odom/rf2o`, `/tf`, `/map` at root).

- [2026-08-04] **[Amendment to BUG-012] First measured evidence, and suspect (a) becomes falsifiable.** A mapping run logged **~330,000** CycloneDDS socket-write failures of the form `tev: ddsi_udp_conn_write to udp/<ip>:<port> failed with retcode -1`. Destinations: **118,049 to `192.168.2.195` — the Jetson's own `wlP1p1s0` address** — plus ~85,000 each to `192.168.2.194`, `.193`, `.141`, `.140` (the static `Peers`, none present on the network). **Caveat, stated plainly: this was a bounded 87-second burst (16:50:55–16:52:22) coinciding with a lab WiFi changeover, not a steady-state condition** — so it does *not* by itself explain the continuous introspection flakiness. What it does establish is the mechanism behind suspect (a): with `<NetworkInterface name="wlP1p1s0">` bound and **no loopback entry in `<Interfaces>`**, same-host participant traffic egresses over the radio — so a purely local CLI-to-node query (`ros2 param get` against a node in the *same container on the same host*) can be broken by a WiFi event that should be completely irrelevant to it. That turns suspect (a) into a concrete test: **add loopback to `<Interfaces>`, and a WiFi disturbance should no longer affect same-host discovery.** Run that before touching `MaxAutoParticipantIndex` or `AllowMulticast`.

- [2026-08-04] **[Amendment to BUG-009] Still not reproduced, and no longer measurable the same way.** Re-checked on the composed path with nav2 fully active on a quiet host (load 2.15): no process anywhere near the original 94.1% of a core. The composed nav2 container totals 48.6% CPU, but **because all seven servers now share one process, `planner_server` can no longer be isolated with `ps`/`top`** — per-node CPU on the composed path needs a different instrument. Still not the decisive test: the VESC remains unpowered, so the BUG-010 hypothesis (that the 94% was a symptom of the runaway EKF) is neither confirmed nor cleared.
  - **[2026-08-04] New lead, higher priority than the BUG-010 hypothesis:** BUG-022 turned out to load **two complete nav2 stacks** into two same-named containers — two `planner_server`s planning against two `global_costmap`s. Re-measure BUG-009 now that BUG-022 is fixed *before* pursuing the EKF explanation.
