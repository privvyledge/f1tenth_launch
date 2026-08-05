# F1/10 sensor dataset — handoff to consuming repos

You are working in a different repository and probably have never touched the
robot. This tells you how to get the data, what is in it, and the handful of
things about these sensors that will waste your afternoon if nobody tells you.

You do **not** need `f1tenth_launch` checked out to use the bag.

---

## 1. The robot

`gosling1` is a Jetson Orin on the lab network. There is already an SSH config
entry for it on this machine, so this works with no extra setup:

```bash
ssh gosling1
```

If that fails, the host is `192.168.2.195` and auth is by key
(`~/.ssh/config` on this machine has the entry). The ROS 2 stack does not run on
the host — it runs in a Docker container named `f1tenth_claude_test`, and the
robot's bulk storage is an NVMe SSD at `/mnt/f1tenth_ssd` on the host, which the
container sees as `/mnt/shared_dir`.

**Do not write to the robot's root filesystem.** It is a 28 GB SD card that sits
around 95% full. Everything large goes on the SSD.

## 2. Getting the bag

Bags live under `/mnt/f1tenth_ssd/shared_dir/bags/<YYYYMMDD>/` on the host.

The first dataset for you is:

```
/mnt/f1tenth_ssd/shared_dir/bags/20260805/detection_093214/          # 9.9 GB
/mnt/f1tenth_ssd/shared_dir/bags/20260805/detection_093214.README.md
/mnt/f1tenth_ssd/shared_dir/bags/20260805/detection_093214.topics.txt
```

Copy all three — the sidecars describe the conditions the bag was recorded under:

```bash
rsync -avP --info=progress2 \
  gosling1:/mnt/f1tenth_ssd/shared_dir/bags/20260805/detection_093214* \
  ./data/bags/
```

It is ~10 GB for 59 seconds. Uncompressed MCAP, because the Jetson was better
spent on the sensors than on zstd. Copy it once and work locally; do not replay
it over the network.

**Read `detection_093214.README.md` first.** It is generated at record time and
states the actual conditions of that specific take, which matter more than
anything in this file.

## 3. What is in it

Recorded 2026-08-05, **59.2 s, 60,708 messages**, no dropped frames on any
stream. The car was **stationary** for the whole take; a person moved obstacles
of varying size through the scene from ~0.5 m out to the far wall.

Everything is namespaced under `/gosling1/`.

| Topic (prefix `/gosling1/`) | Type | Count | Rate |
|---|---|---|---|
| `lidar/scan` | `sensor_msgs/LaserScan` | 515 | 8.7 Hz |
| `lidar/scan_filtered` | `sensor_msgs/LaserScan` | 516 | 8.7 Hz |
| `camera/color/image_raw` | `sensor_msgs/Image` | 1774 | 30 Hz |
| `camera/color/camera_info` | `sensor_msgs/CameraInfo` | 1773 | 30 Hz |
| `camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 1773 | 30 Hz |
| `camera/aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo` | 1775 | 30 Hz |
| `camera/depth/image_rect_raw` | `sensor_msgs/Image` | 1774 | 30 Hz |
| `camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 1773 | 30 Hz |
| `camera/depth/color/points` | `sensor_msgs/PointCloud2` | 1773 | 30 Hz |
| `camera/infra1/image_rect_raw` | `sensor_msgs/Image` | 1771 | 30 Hz |
| `camera/infra2/image_rect_raw` | `sensor_msgs/Image` | 1774 | 30 Hz |
| `camera/infra1,2/camera_info` | `sensor_msgs/CameraInfo` | ~1773 | 30 Hz |
| `camera/extrinsics/depth_to_{color,infra1,infra2}` | `realsense2_camera_msgs/Extrinsics` | 1 each | latched |
| `camera/imu`, `camera/imu/filtered` | `sensor_msgs/Imu` | ~11840 | 200 Hz |
| `vehicle/sensors/imu/raw` | `sensor_msgs/Imu` | 5917 | 100 Hz |
| `vehicle/sensors/core` | `vesc_msgs/VescStateStamped` | 2955 | 50 Hz |
| `odometry/local` | `nav_msgs/Odometry` | 1772 | 30 Hz |
| `odom/rf2o` | `nav_msgs/Odometry` | 516 | 8.7 Hz |
| `visual_slam/tracking/odometry` | `nav_msgs/Odometry` | 1773 | 30 Hz |
| `visual_slam/vis/slam_odometry` | `nav_msgs/Odometry` | 1772 | 30 Hz |
| `tf` | `tf2_msgs/TFMessage` | 1772 | 30 Hz |
| `tf_static` | `tf2_msgs/TFMessage` | 10 | latched |
| `robot_description` | `std_msgs/String` | 1 | latched |
| `vehicle/vesc_odom` | `nav_msgs/Odometry` | **0** | see below |

## 4. Playing it back

`/tf` and `/tf_static` are namespaced too (`/gosling1/tf`). That is deliberate
multi-robot support, but most tooling — RViz, `tf2_ros`, anything using a plain
`TransformListener` — subscribes to the absolute `/tf`. Remap on playback:

```bash
ros2 bag play ./data/bags/detection_093214 \
  --remap /gosling1/tf:=/tf /gosling1/tf_static:=/tf_static
```

Set `use_sim_time:=True` on your nodes and play with `--clock` if you care about
timestamps lining up.

If your node subscribes to unnamespaced topics, either push your node into the
`/gosling1` namespace or add more `--remap` pairs. Do not strip the namespace by
re-recording; you will lose the latched messages.

## 5. Frames

`base_link` is the **rear axle**. Full static tree is in `tf_static` and
`robot_description` (both recorded), so you can rebuild it offline:

```
base_link
├── sensor_kit_link
│   ├── lidar          ← frame_id of lidar/scan and lidar/scan_filtered
│   └── imu_link       ← VESC IMU
├── base_footprint
├── front_axle / rear_axle / *_wheel
└── camera_link → camera_{color,depth,infra1,infra2}_optical_frame  (from URDF)
```

`odom → base_link` is broadcast at 30 Hz by the EKF (`robot_localization`),
fused from rf2o LiDAR odometry, the camera IMU, the VESC IMU and Isaac VSLAM.

**There is no `map` frame in this bag.** No global localizer was running. If you
need a fixed world frame for a stationary take, `odom` is it.

## 6. Sensor caveats that will cost you time

These are not opinions; they are measured properties of this hardware.

**YDLidar X4**
- Valid range **0.12–10.0 m**. Below 0.12 m you get ghost returns at the origin;
  the driver is configured to 10.0 m because 12.0 m produced phantom obstacles
  from out-of-spec returns.
- Out-of-range returns are **`inf`, not `0.0`**. If your segmentation treats 0.0
  as "no return" you will still be fine, but if it treats `inf` as a valid range
  you will get points at infinity. Filter explicitly.
- Nominal 12 Hz, **actually ~8.7 Hz** on the Jetson under USB/CPU load. Do not
  hardcode 12.
- **Intensity is meaningless.** The X4 cannot report real intensity; with
  `intensity: true` the driver crashes on checksum errors, and with
  `intensity: false` every point reports a constant ~1008.0, which is internal
  status bits. Do not build a feature on the intensity channel.
- `scan` is raw, `scan_filtered` has a tuned speckle filter applied. Both are in
  the bag so you can measure what the filter costs you. `scan_filtered` is what
  the rest of the stack consumes.

**RealSense D435i**
- **The IR emitter was OFF for this recording.** That means `infra1`/`infra2`
  are clean rectified stereo IR and are usable for stereo matching. The price is
  noisier depth and a holier colored pointcloud, especially on low-texture,
  white or glossy surfaces. If your detector needs the best possible depth and
  does not care about the stereo pair, ask for a re-record with the emitter on —
  it is a one-argument change.
- **`aligned_depth_to_color/image_raw` and `depth/image_rect_raw` are not
  interchangeable.** The aligned one is resampled into the *colour* intrinsics
  (pair it with `color/camera_info`) and is what you want for pixel-aligned
  RGB-D. The native one keeps the wider depth FOV (pair it with
  `depth/camera_info`). Using the wrong `camera_info` will silently give you a
  plausible-looking, wrong reprojection.
- `camera/depth/color/points` is the driver's own XYZRGB cloud, in the **depth
  optical frame**.
- The three `camera/extrinsics/*` messages are latched and appear once at the
  start of the bag. If you seek past the beginning you will miss them.

**Vehicle**
- **`vehicle/vesc_odom` contains zero messages.** This is expected and is not a
  recording failure. The VESC was connected and healthy, but the bag was
  recorded with no command path to the motor (deliberately — the car must not
  move), and `vesc_to_odom` only publishes once a steering command has been
  seen. **Use `odometry/local` for pose**; it is the EKF output and the thing
  that carries `odom → base_link`.
- `vehicle/sensors/imu/raw` (100 Hz) is the VESC IMU and is fully populated.

## 7. Asking for more data

Re-recording is cheap; a trip to the lab is not. The script that produced this
lives in the `f1tenth_launch` repo at
`scripts/live_runs/21_detection_dataset_bag.sh`. Say what you need and it can be
regenerated the same day:

- **A different scene** — `--name <label>`, e.g. more clutter, specific object
  classes, a particular range band.
- **A different length** — `--duration <seconds>`.
- **Emitter on** for better depth (costs you the clean stereo pair).
- **The car moving** — a different script, `30_mapping_drive.sh`, gives you the
  same sensors with real motion and a `map` frame.
- **Extra derived clouds** — a GPU XYZRGB cloud from aligned depth, or a
  pre-segmented ground/obstacle split, are both one launch argument away.

State what you need in terms of the *scene and the topics*, not the launch
arguments; whoever runs the robot will translate.
