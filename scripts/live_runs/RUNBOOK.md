# F1/10 Live Run Runbook

Copy/paste blocks for a full run day on `gosling1`. Every block assumes you are
**inside the container**, in `scripts/live_runs/`.

Phases are numbered in the order they should be run. Each script prints what it
is doing and refuses to continue past a failed safety or health check.

---

## 0. Enter the environment

```bash
# on the host
ssh gosling1
docker restart f1tenth_claude_test        # zombies survive pkill; restart is cleaner
DISPLAY=:0 xhost +local:                  # RealSense needs a GL context
docker exec -it f1tenth_claude_test bash
```

```bash
# inside the container
cd /workspaces/f1tenth/src/f1tenth_launch/scripts/live_runs
export ROS_DOMAIN_ID=0                    # 77 = test container, 42 = concurrent agent
./10_preflight.sh
```

`10_preflight.sh` must pass before anything else. It checks the workspace, the
SSD mount, free disk, the VESC symlink, the joystick, USB enumeration, and the
GPU/VSLAM preconditions.

### Storage

| Path | What |
|---|---|
| `/mnt/shared_dir` (container) = `/mnt/f1tenth_ssd/shared_dir` (host) | NVMe, ~700 GB free |
| `/mnt/shared_dir/bags/<YYYYMMDD>/` | all bags |
| `/mnt/shared_dir/maps/<YYYYMMDD>/` | all maps |

**Never write bags to `/`** — the host root is a 28 GB SD card sitting at 95% full.

### Knobs

Everything is overridable by exporting before the script:

```bash
export NS=gosling1            # robot namespace
export USE_GPU=true           # false -> CPU path, no Isaac VSLAM
export MAX_SPEED=1.5          # m/s joystick cap; keep low until you trust the run
export ROS_DOMAIN_ID=0
export BAG_COMPRESSION=zstd   # 'none' if the Jetson is CPU-bound
```

---

## 1. Sensor bag — objective 1

Data for tuning the object-detection repos. Record **both** variants so we can
settle whether publishing the colored pointcloud costs camera frames.

### 1a. Raw streams (baseline)

> **Safety:** battery disconnected, or car elevated with wheels clear.

```bash
./tmux_run.sh sensors
# or without tmux:
./20_sensor_bag.sh
```

During the run:
1. Car **stationary** ~30 s while people/objects move through the scene at
   varying ranges.
2. Drive slowly around the objects ~60 s so detectors see multiple viewpoints.
3. Press **joystick button 5** once — this is the autonomous handover. Watch
   whether `ackermann_drive` keeps flowing.

`Ctrl-C` to stop the recorder.

### 1b. With the colored pointcloud

```bash
./20_sensor_bag.sh --pointcloud
```

### 1c. Compare

```bash
./90_inspect_bag.sh /mnt/shared_dir/bags/<date>/sensors_*  \
                    /mnt/shared_dir/bags/<date>/sensors_cloud_*
```

Look at the `Hz` column for `camera/color/image_raw` and
`aligned_depth_to_color`, and the `stalls` column. If the cloud variant drops
camera rates, keep `publish_realsense_pointcloud:=False` and regenerate the
cloud offline from depth + colour + `camera_info`.

### 1d. Stationary detection dataset — for the downstream perception repos

A different job from 1a–1c. This one produces a hand-off dataset for the
detection / segmentation repos, not a diagnostic bag for this one. The vehicle
stays still and the **VESC does not need to be connected**; you move obstacles
of varying scale and range through the scene by hand.

```bash
./21_detection_dataset_bag.sh                 # 120 s
./21_detection_dataset_bag.sh --duration 90 --name aisle_boxes
```

It launches sensors and local localization only — no vehicle, no joystick, no
command_gate — and always records the colored pointcloud plus the depth
`camera_info`, the inter-stream extrinsics and `/robot_description`, so a repo
that does not have `f1tenth_launch` checked out can still reconstruct the
geometry. `EXPECT_VESC=false` / `EXPECT_JOYSTICK=false` keep preflight from
failing on hardware this phase does not use.

The IR emitter stays **off**, so `infra1`/`infra2` are clean stereo at the cost
of holier depth. Flip that with `realsense_emitter_enabled:=1` in the launch
line inside the script if a consumer needs the best possible depth instead.

Two sidecar files are written next to the bag and must travel with it:
`<bag>.README.md` (conditions, namespace, frames, sensor caveats) and
`<bag>.topics.txt`.

Consumers replaying outside this namespace need:

```bash
ros2 bag play <bag> --remap /gosling1/tf:=/tf /gosling1/tf_static:=/tf_static
```

---

## 2. Mapping drive — objective 2

No SLAM runs live. This records the drive; maps are built afterwards from the
bag, so you can retune and rebuild without re-driving.

> **Safety:** car on the ground, joystick in hand, `MAX_SPEED` low.

```bash
./tmux_run.sh mapping
# or: ./30_mapping_drive.sh
```

Driving for a good map:
- **Slowly.** Rotation breaks scan matching — take corners wide, don't spin in place.
- Cover the space; drive corridors **both** directions.
- **Mark the start pose with tape.** Return to it exactly and pause ~10 s. That
  gives the offline run a loop closure and gives you the ground-truth reference
  for phase 3.
- 3–8 minutes is plenty for a lab.

```bash
./90_inspect_bag.sh          # confirm complete TF + full-rate sensors
```

### 2b. Several bags in one session — `25_drive_session.sh`

`30_mapping_drive.sh` is one-shot: it launches, records exactly one bag, and
tears down on Ctrl-C. When a run day needs a mapping drive *and* trajectory
runs, use `25_drive_session.sh` instead — it brings the stack up **once** and
starts/stops named bags against it, so every bag shares one calibration, one TF
tree and one set of sensor clocks. It also needs no interactive Ctrl-C, so it
can be driven from a remote shell.

```bash
./25_drive_session.sh launch --max-speed 1.0 -y
./25_drive_session.sh status                  # check rates BEFORE recording
./25_drive_session.sh start mapping_drive     # proves bytes are landing, then says "move the robot"
#   ...drive...
./25_drive_session.sh stop                    # closes the bag + per-topic count/Hz audit
./25_drive_session.sh shutdown
```

All bags use the `drive` topic set (40 topics): everything in `mapping`, plus
the colored pointcloud, the native depth stream, the inter-stream extrinsics and
the latched `/robot_description` — so a downstream repo without
`f1tenth_launch` checked out can still reconstruct the geometry.

`start` never announces "recording" on faith: it confirms the recorder is alive
and measures bytes landing on disk over 4 s first. `stop` refuses to claim
success unless `metadata.yaml` exists, then audits every requested topic against
its `MIN_RATE` floor. `drive`, `cmd_vel` and `estop` are silent by design
without Nav2/MPC and report `[ -- ]`; anything else silent is a real defect.

Budget **~11 GB/min** with the pointcloud on (172 MB/s measured).

Driving it from a remote/agent shell — note the stdin form, since `bash -lc`
through two levels of quoting mangles `${...}` and `"$@"`:

```bash
ssh gosling1 'docker exec -i <container> bash -s' < some_script.sh
```

See `DRIVE_SESSION_HANDOFF.md` for the full three-bag procedure and the
hardware findings from 2026-08-05.

---

## 3. Build maps offline — no robot needed

```bash
./40_build_map_offline.sh --bag /mnt/shared_dir/bags/<date>/mapping_drive_* --mode both
```

`--mode rtabmap` gives the RTABMap 2D grid **and** the 3D map;
`--mode slamtoolbox` gives the SLAM Toolbox 2D grid; `--mode both` runs each in
turn so you can compare. (You've found RTABMap's 2D grid cleaner — this is where
that gets confirmed on real data.)

Options: `--rate 0.2` (playback rate; higher overruns the SLAM TF filter),
`--append` (keep the existing RTABMap DB), `--no-viz` (skip RViz over SSH).

Compare the results before choosing:

```bash
ls -lh /mnt/shared_dir/maps/<date>/
eog /mnt/shared_dir/maps/<date>/*_2d_*.pgm
```

Export a 3D cloud from the RTABMap DB (GUI, **not** `rtabmap-export`):

```bash
rtabmap-databaseViewer /mnt/shared_dir/maps/<date>/rtabmap_*.db
# File -> Export 3D clouds -> voxel size 0.02-0.05 m -> .pcd
```

---

## 4. Localization accuracy test — objective 3

Drive a loop, return to the taped start pose, measure how far each estimator
thinks it moved. TF ownership: `ekf_map_node` owns `map→odom`, `ekf_odom_node`
owns `odom→base_link`. AMCL runs with `tf_broadcast:=False` and feeds
`amcl_pose` into the global EKF.

> **Safety:** car on the ground, joystick in hand.

```bash
./tmux_run.sh localization -- --map /mnt/shared_dir/maps/<date>/<chosen>.yaml
# or: ./50_localization_test.sh --map <map.yaml>
```

Set the initial pose (RViz "2D Pose Estimate", or the CLI command the script
prints), rotate gently until the particle cloud tightens, then press Enter.

Verify TF ownership before recording — this is the failure that was BUG-027:

```bash
ros2 param get /gosling1/amcl tf_broadcast      # expect: false
ros2 run tf2_tools view_frames --ros-args -r __ns:=/gosling1
```

The run:
1. Park exactly on the marked pose, pause ~10 s.
2. Drive a loop, return to the **same** pose and heading.
3. Pause ~10 s. `Ctrl-C`.

Then produce the comparison plot:

```bash
python3 ../analysis/plot_localization.py \
    /mnt/shared_dir/bags/<date>/localization_loop_* \
    --map /mnt/shared_dir/maps/<date>/<chosen>.yaml \
    --out ../../docs/figures/localization
```

---

## 5. Nav2 — objective 4

**Dry run first.** `--dry-run` sends the velocity smoother to `cmd_vel_nav2`
instead of `cmd_vel`, so Nav2 plans and publishes normally but nothing reaches
the vehicle.

```bash
./tmux_run.sh nav2 -- --map <map.yaml> --dry-run
```

The script prints a lifecycle-node census — **every server must appear exactly
once**; a count of 2 means a duplicate stack. It also dumps `cmd_vel`
publishers, because an earlier session saw 14 of them with mixed
`Twist`/`TwistStamped` and that was never explained.

Send a goal from RViz ("2D Goal Pose", fixed frame `map`) or:

```bash
ros2 action send_goal /gosling1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'
```

Once the dry run plans correctly:

```bash
./60_nav2_test.sh --map <map.yaml>      # live; hand on the joystick
```

---

## 6. MPC handover — after the four objectives

Your MPC publishes `AckermannDriveStamped` to `/gosling1/drive`.

```bash
./tmux_run.sh mpc -- --map <map.yaml>
```

Mux priorities: `estop` 255 > `joystick` 100 (`teleop`) > `navigation` 10
(`drive`) > `safety` 1. Holding button 5 routes joy_teleop to its `/dev/null`
sink, `teleop` goes silent, and `drive` wins.

The open question the script tests: `command_gate`'s heartbeat is `teleop` with
a 1.0 s timeout. If the gate closes ~1 s after you press button 5, rerun with
`--no-heartbeat`.

Watch in another pane:

```bash
ros2 topic hz /gosling1/teleop /gosling1/drive \
              /gosling1/ackermann_drive /gosling1/vehicle/ackermann_cmd
```

---

## 7. Clean up

```bash
./90_inspect_bag.sh <bag>     # always inspect before deleting
./99_prune_bags.sh            # interactive; no force flag, no glob delete
```

---

## Hard-won operating rules (2026-08-05)

Learned by losing a 120 s take and chasing three phantom failures in one
session. All five are now handled in the scripts, but the operating habits
below are what keep them from recurring in a form the scripts do not cover.

### Shutting a stack down

**Never `kill <pid>` the `ros2 launch` wrapper.** SIGTERM to the wrapper does
not reap its children — it orphans them, and orphans keep publishing. An
orphaned `ekf_node` gives you two publishers on `odometry/local` *and two
broadcasters on `odom → base_link`*. Orphaned `scan_to_scan_filter_chain`
instances stack one per launch and multiply `scan_filtered`.

Correct shutdown, in order of preference:

1. Let the run script's own `EXIT` trap do it (Ctrl-C the script).
2. `pkill -INT -f "ros2 launch f1tenth_launch"`, then wait **20+ s** — SIGINT
   propagates properly but shutdown is not instant.
3. Only then sweep stragglers by name, and include **every** executable:
   `component_container`, `ekf_node`, `ydlidar_ros2_driver`, `vesc_driver_node`,
   `scan_to_scan_filter_chain`, `rf2o_laser_odometry`, `robot_state_publisher`,
   `static_transform_publisher`, `ackermann_to_vesc`, `vesc_to_odom`,
   `ackermann_to_twist`. A stubborn `component_container` may need `-9`.

**Verify by rate, not by process list.** The tell for a surviving duplicate is a
topic reading an exact integer multiple of its expected rate:

```bash
ros2 topic hz /$NS/odometry/local     # 30 Hz good, 60 Hz means two EKFs
ros2 topic hz /$NS/lidar/scan_filtered  # must match lidar/scan, not 2-3x it
ros2 topic info /$NS/lidar/scan_filtered | grep "Publisher count"   # want 1
```

Ignore `<defunct>` entries — zombies hold nothing and publish nothing.

### Trusting the tooling

**`ros2 topic list` returns a partial graph on this vehicle.** Under the
CycloneDDS static-peer config each CLI call discovers from scratch and comes
back with a different random subset. One pass reported colour, both infra
streams and the colored pointcloud all "missing" while every one was publishing
at 30 Hz. `have_topic` now retries and falls back to a direct query, but when
diagnosing by hand, **measure the topic by name (`ros2 topic hz`) before
believing any list**.

**Echo values, not rates, when a gate is involved.** A closed `command_gate`
publishes at full rate with zero payloads; `ros2 topic hz` cannot see the
difference.

**Expect the DDS peer noise, and grep past it.** Every ROS process emits
`ddsi_udp_conn_write ... failed with retcode -3` at ~200 lines/s for each
powered-off machine in the static peer list — one stack launch produced a
381,976-line log that hid a four-line YDLidar failure. Filter with
`grep -v ddsi_udp_conn_write` when reading any log. The cause, the measurements,
and the fix (which lives in the build repo, not here) are in
[CYCLONEDDS_PEERS.md](CYCLONEDDS_PEERS.md).

**Off-board subscribers need a static peer entry, on both ends.** velox1
(`192.168.2.13`) is to send drive commands, and remote RViz would be the same
problem. Multicast is off, so a machine absent from `<Discovery><Peers>` is
invisible **with no error on either side**. The procedure, the third DDS profile
it needs (`lo` + WiFi, one remote peer — the `_lo` profile cannot do this, it
binds loopback only), and the lab validation are in
[DEMO_RUNBOOK_20260810.md](DEMO_RUNBOOK_20260810.md) §0★.3b. **Untested as of
2026-08-30.**

### Recording

**`ros2 bag record` has no record-for-N-seconds flag in Humble.** `-d` /
`--max-bag-duration` is a *bag split* interval. Passing `--duration` makes the
CLI exit immediately — which, combined with a `bag_record` that printed
"bag written" without checking, silently threw away a full scene. `bag_record`
now times a SIGINT itself and verifies `metadata.yaml` exists before claiming
anything.

**Never trust a "recorded" message you have not checked.** After any take:

```bash
ros2 bag info <bag> | head -20     # Messages: must be non-zero
du -sh <bag>                       # ~10 GB per minute with the cloud on
```

Counts should be `duration x rate` per topic. Anything at half or double is a
drop or a duplicate.

### Sequencing a run

Preflight the *stack you are about to record*, not a stack you started fifteen
minutes ago — and re-verify after any restart. `--launch-only` brings a stack up
and leaves it running so you can inspect it; `--record-only --skip-preflight`
then records against that same verified stack without making anyone stand at the
car for two minutes of rate checks. `--skip-preflight` is only valid when
preflight has already passed against *that exact running stack*.

---

## Abort criteria

Stop and diagnose rather than pressing on if:

| Symptom | Likely cause |
|---|---|
| Camera topics absent after 90 s | D435i wedged — restart container, relaunch (scripts already pass `reset_realsense:=True`); check `/tmp/.X11-unix` is mounted and `DISPLAY` set |
| VSLAM aborts / SIGABRT | `localize_on_startup` against an empty map dir. Scripts default it False; if it still aborts, `export USE_GPU=false` |
| Duplicate nodes in the census | duplicate container or duplicate launch include — do **not** record, the topics will be starved |
| A topic present at 0 Hz | starved subscriber, usually a namespace mismatch. `10_preflight.sh` treats this as a hard failure |
| `map→odom` has 2+ broadcasters | competing TF owners; check `map_tf_publisher` and that AMCL has `tf_broadcast:=false` |
| Vehicle ignores the joystick | `command_gate` closed (no heartbeat), or `launch_command_gate:=False` removed the only publisher of `vehicle/ackermann_cmd` |

## Known landmines already handled by these scripts

- `map_2d_file` is inert — the value actually read is `default_2d_map_file`.
- `launch_2d_mapping` defaults `False`; `slam:=True` alone silently produces no map.
- Offline map building goes through `mapping.launch.py` directly, never
  `bringup slam:=True` — the latter includes localization twice and strands
  duplicate rtabmap odometry nodes in the root namespace (BUG-007, still open).
- The Isaac VSLAM map is **never saved by the launch files**
  (`localization.launch.py:915-916` hardcodes `save_map`/`load_map` to `False`).
  `41_save_map.sh --mode vslam` calls the service directly instead.
- `/dev/null` in the topic list is the **intentional** autonomous-handover sink.
  Do not "fix" it.
