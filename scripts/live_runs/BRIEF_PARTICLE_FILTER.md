# Brief for the `particle_filter` Claude — tuning against real bags on `gosling1`

**Written 2026-08-06 by the `f1tenth_launch` agent.** You own the
`particle_filter` repo (the MIT/range_libc MCL localizer). I own the launch and
config package that starts your node on the car. You have never seen this
machine or this repo, so this document is self-contained: how to get on the
robot, how to start a container, where the data is, what your task is, and the
gotchas that cost me hours.

Nothing here asks you to change my repo. Where a change on my side is needed,
it says so and you should ask the operator.

---

## 1. The system, in one page

**`gosling1`** is an F1/10 (1/10-scale autonomous race car) on an **NVIDIA
Jetson Orin Nano** — 6 CPU cores, 7 GB RAM (memory pressure is real; two
processes were SIGKILLed during my RTABMap tests). ROS 2 **Humble**.

Sensors and actuators that matter to you:

| | |
|---|---|
| LiDAR | **YDLidar X4**, ~625 beams/rev, **~8.5 Hz** actual (not the 12 Hz configured — USB/CPU overhead), 0.12–10 m |
| Odometry | 30 Hz EKF (`robot_localization`) fusing VESC wheel odom + IMUs + rf2o LiDAR odom |
| Motor/servo | VESC over USB serial |
| Camera | RealSense D435i (irrelevant to you — never replayed in localization passes) |

**Everything is namespaced `/gosling1`.** Topic names you will type are
`/gosling1/lidar/scan_filtered`, `/gosling1/odometry/local`, `/gosling1/tf`,
`/gosling1/map`. The namespace comes from `$VEHICLE_NAME`. `/tf` and
`/tf_static` are remapped to relative `tf`/`tf_static` throughout, so **there is
no root `/tf` on this vehicle** — a node that subscribes to `/tf` sees an empty
TF tree and fails silently. That exact bug (bug-107) is why the RTABMap
localizer had never worked.

Frames: `map` → `odom` → `base_link`. **`base_link` is the REAR AXLE**, not
`base_footprint` (33 mm below) and not `front_axle` (256 mm ahead). A wrong
choice there is a 0.13 m error that no check catches — the same size as the
entire accuracy budget.

The localizer's only job is to publish **`map` → `odom`**. `odom` → `base_link`
comes from the EKF at 30 Hz and is already in the bags.

---

## 2. Getting on the machine and into a container

### 2.1 Connect

```bash
ssh gosling1
```

The operator has the SSH config. `gosling1` has **no internet** — `git pull`,
`pip install`, `apt install` inside the container will not work. See §2.4 for how
to get code onto it.

### 2.2 Start a container — use the operator's script, never `docker run`

```bash
bash ./bolus_ws/f1tenth_launch.sh
```

This is `jetson-containers run …` with the full flag set: X11 forwarding, USB
device passthrough, `/dev/video*`, `--shm-size=8g`, the SSD bind mount, and the
CycloneDDS config. It is interactive (`-it --rm`) and self-names the container
`jetson_container_<YYYYMMDD>_<HHMMSS>`; read the name from `docker ps`.

> **Do not hand-build the container by copying `docker inspect` output from a
> previous one.** Someone did that on 2026-08-05, silently lost
> `-v /tmp/.docker.xauth`, `--device /dev/bus/usb` and `--shm-size=8g`, and burned
> 25 minutes of the operator's time plus a battery before the RealSense failure was
> understood (bug-073). You need none of those devices for bag replay, but you do
> want the SSD mount and the DDS config, and the script is the only sanctioned
> way to get a container at all.

**Unattended / from an agent shell**, the script needs a TTY and a stdin that
never closes:

```bash
mkfifo /tmp/pf.fifo
setsid bash -c "exec sleep infinity > /tmp/pf.fifo" </dev/null >/dev/null 2>&1 &
setsid script -qfc "bash $HOME/bolus_ws/f1tenth_launch.sh" /dev/null \
  < /tmp/pf.fifo > ~/pf_container.log 2>&1 &
sleep 20 && docker ps --format '{{.Names}}'
```

Then `docker exec -it <name> bash`.

### 2.3 Isolate yourself from other sessions

Other agents and the operator may have a stack running. **Before anything
else**, in every shell inside your container:

```bash
export ROS_DOMAIN_ID=42                     # NOT 0 — 0 is the live stack
export CYCLONEDDS_URI=file:///mnt/shared_dir/cyclonedds_offline_lo.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

The `_offline_lo.xml` config confines DDS to loopback. The default
(`cyclonedds_config_static.xml`) uses static peers for the live vehicle network.
**A CycloneDDS config that omits the `lo` interface silently breaks
intra-machine traffic** — that was the cause of a "GPU-bound VSLAM jitter"
red herring that took a whole session to disprove.

### 2.4 Getting your code in

`/workspaces/f1tenth` is **inside the container image, not a bind mount** — a
fresh container starts with whatever the image shipped and cannot fetch updates.

The only bind mount is host `/mnt/f1tenth_ssd/shared_dir` → container
`/mnt/shared_dir`. So: `scp` your files to
`/mnt/f1tenth_ssd/shared_dir/handoff/particle_filter/` from your dev machine,
then copy them into the workspace from inside the container and verify with
`md5sum` (the robot's git checkout can sit at an older commit than your machine
even when the files are byte-identical — compare hashes, not `git log`).

The workspace is built `--symlink-install`, so **launch-file and YAML edits go
live with no rebuild**. C++/Python source changes need
`colcon build --symlink-install --packages-select particle_filter` from
`/workspaces/f1tenth`.

### 2.5 Storage

`/` on the host is a **28 GB SD card at 95% full**. Never write bags, maps or
anything bulky there. Everything goes under `/mnt/shared_dir` (the 916 GB NVMe).

---

## 3. Where the data is

Host `/mnt/f1tenth_ssd/shared_dir/…` = container `/mnt/shared_dir/…`.

```
/mnt/shared_dir/
├── bags/20260805/
│   ├── mapping_drive_170025/     ← THE CONTROL. Use this one.
│   ├── figure8_172338/
│   ├── loop_laps_173558/
│   └── loc2_*/                   ← my AMCL localization outputs (reference)
├── maps/20260805/
│   ├── rtabmap_2d_final.yaml/.pgm   ← the occupancy grid. 265x199 @ 0.05 m,
│   │                                   1732 occupied cells (a sparse lab room)
│   ├── rtabmap_final.db              ← RTABMap DB with the optimized pose graph
│   └── truth_mapping_drive_170025.csv  ← GROUND TRUTH for the control bag
└── deliverables/20260805/         ← FROZEN. See §7. Do not write here.
```

The three bags were recorded 2026-08-05 16:54–17:40, all from the **same
hand-placed physical start pose**, ~1 m/s peak, ~150 s each, in the RAS lab.
Each is ~25 GB, almost all camera data you will never touch.

Topics you care about inside a source bag:

| topic | type | rate | note |
|---|---|---|---|
| `/gosling1/lidar/scan_filtered` | `LaserScan` | **8.5 Hz** | speckle-filtered; `inf` for out-of-range |
| `/gosling1/odometry/local` | `Odometry` | 30 Hz | the EKF output your node consumes as `odom` |
| `/gosling1/tf` | `TFMessage` | | carries `odom`→`base_link` (and `map`→`odom` in my loc bags) |
| `/gosling1/tf_static` | | | sensor extrinsics |
| `/gosling1/vehicle/vesc_odom` | `Odometry` | | raw wheel odom |

---

## 4. Your task

### 4.1 The one-sentence version

**Measure the particle filter against the same control bag and the same ground
truth that AMCL was measured on, and report smoothness (`map→odom` correction
step size) as the headline number, not accuracy.**

### 4.2 Why — the context you are missing

A downstream consumer (an MPC controller, a separate repo/Claude) complained
that its global pose is **jerky**. The cause is understood and is not a defect:

- The delivered `map→odom` is **nav2_amcl broadcasting directly**, updated at
  the **8.5 Hz scan rate**, composed with smooth **30 Hz** odometry.
- So `map→odom` is a **piecewise-constant (zero-order-hold) correction**.
  Measured step sizes: **p95 14.6 mm, max 59.6 mm**.
- Differentiate that for velocity and every step becomes a spike. That is what
  the MPC sees.

Your filter is structurally different in exactly the relevant way:
`localizer_pf.yaml` sets **`mcl_hz: 40.0`, decoupled from the odometry and scan
rates** — it predicts through the motion model between scans and emits a
weighted mean of the particle set rather than a resampled jump. **That should
produce a much smaller correction step.** That is the hypothesis to test.

**Do not expect better accuracy.** Same LiDAR, same grid, same information,
same floor. If your mean error lands near AMCL's 64.7 mm, that is the expected
result and a success — the win you are looking for is in the step statistics.

### 4.3 The bar to compare against

From `MAP_FRAME_DELIVERY.md`, nav2_amcl on `mapping_drive_170025` scored against
`truth_mapping_drive_170025.csv`:

| metric | AMCL |
|---|---|
| translation error, mean | **64.7 mm** |
| p95 | **143.5 mm** |
| max | 236.2 mm |
| yaw error, mean | 1.69° |
| inside the consumer's 126 mm bar | **87.3 %** |
| AMCL's own converged sigma | 75/66 mm, 4.45° |
| **`map→odom` correction step, p95 / max** | **14.6 / 59.6 mm** ← beat this |
| CPU | 3.3–4.1 % of one core (peak 20.4 %), 21 MB RSS |

CPU budget context: you have 6 cores. AMCL uses ~4 % of one. RTABMap's localizer
used a **full core and ~1 GB** and did not work. `max_particles: 2000` at 40 Hz
is the current guess for what the Orin can carry — **measure it**, from
`/proc/<pid>/stat` (this is how the other numbers were taken; report as percent
of *one* core, and include peak RSS).

### 4.4 Step zero, and I mean zero: **is your package even installed?**

`launch/localization/localization.launch.py` in my repo wraps your node in:

```python
try:
    get_package_share_directory('particle_filter')
    particle_filter_node = Node(...)
except ROS2PackageNotFoundError as e:
    particle_filter_node = LogInfo(msg=f'Failed to launch particle filter node: {e}. Skipping...')
```

So if the package is absent, **launch succeeds, prints one info line, and starts
nothing.** Confirm before anything else:

```bash
ros2 pkg prefix particle_filter && ros2 pkg executables particle_filter
python3 -c "import range_libc; print(range_libc.__file__)"
```

This session is a lesson in what "never been run" is worth: the RTABMap
localizer had been in the launch file for months, was assumed to work, and
turned out to have **never been capable of working**. It also failed *silently*,
publishing a perfectly steady identity `map→odom` at 20.8 Hz while every liveness
check passed. **A localizer that publishes at the right rate is not a localizer
that is localizing.** Check the values, never the rate.

### 4.5 How my launch wires your node

`localization.launch.py` starts you with `launch_particle_filter:=True`, and
overrides these **at the node level, on top of your YAML** (launch wins):

```
namespace         /gosling1
remap  scan   ->  lidar/scan_filtered
remap  odom   ->  odometry/local
use_sim_time      (from the launch arg — True for bag replay)
scan_qos_reliability / odom_qos_reliability = best_effort
range_method      rmgpu   if use_gpu else pcddt
rangelib_variant  2       if use_gpu else 4
global_frame_id   map
base_frame_id     base_link
odom_frame_id     odom
tf_broadcast      True only if map_tf_publisher == 'pf'
```

Two consequences worth internalising:

1. **`launch_particle_filter:=True` starts the node; `map_tf_publisher:='pf'`
   makes it the TF owner.** They are separate args and you need both. Setting
   only the second starts nothing.
2. **`tf_broadcast` in your YAML is ignored** on this vehicle. Same pattern
   applies to AMCL, slam_toolbox, RTABMap and the EKF — exactly one node owns
   `map→odom`, chosen by one launch arg.

Your current tuned config (`config/localization/localizer_pf.yaml` in my repo —
copy is on the SSD if you cannot see the repo) and the reasoning behind the
non-default values:

| param | value | why |
|---|---|---|
| `angle_step` | 10 | ~62 of the X4's ~625 beams per cycle; was 18, raised for a sharper yaw constraint |
| `mcl_hz` | 40.0 | decoupled from odom/scan rate — **the whole smoothness argument** |
| `max_particles` | 2000 | half the upstream 4000; a guess for the Orin, never benchmarked |
| `squash_factor` | 1.0 | no flattening; was 2.2 |
| `sigma_hit` | 2.0 px | 0.05 m grid → 10 cm beam sigma; was 4.0 |
| `motion_dispersion_theta` | 0.05 | was 0.25 — was diffusing heading too much |
| `update_min_d` / `_a` | 0.02 m / 0.01 rad | stops a parked car random-walking |
| `z_hit/z_rand/z_short/z_max` | .75/.12/.01/.07 | Thrun beam model, X4-tuned |

For reference, the equivalent AMCL retune that took it from a non-converging
419 mm sigma to 65 mm was: `sigma_hit` 0.4→**0.15**, `z_hit`/`z_rand` →
**0.80/0.20**, `z_max`/`z_short` → **0**, `max_beams` 90→**200**, particles
100–500 → **1000–4000**, `update_min_d` 0.2→**0.05**. The lesson from that
exercise, which applies directly to you:

> **The likelihood-field parameters are NOT separable from the particle count
> and the update gate.** Sharpening `sigma_hit` while leaving few particles and a
> coarse update gate made AMCL *worse* (345 mm vs 267 mm) — a sharp likelihood
> field needs enough particles to sample it and enough updates to exploit it.
> Tune them as a group.

### 4.6 The seeding problem — read this before your first run

The car does not start at the map origin. The correct seed, in the map frame,
identical for all three bags:

```
x = +0.4451   y = -0.5750   yaw = -1.3931 rad  (-79.82°)
```

> **Superseded in accuracy, not in use (2026-08-24):** that `-79.82°` is RTABMap's
> t=0 pose and reads off against RTABMap's own grid; the true 08-05 park heading is
> at least `-85.0°`. Still used as the seed on purpose — a seed costs convergence
> time, not correctness. See `scripts/live_runs/LUCIO_MAP_HEADING_ANSWER.md`.


Seeding at (0,0,0) costs 0.73 m and 25° of initial error; AMCL needed **~28 s of
driving** to pull in from it, and every pose in that window misses the bar. If
you compare an unseeded pf run to a seeded AMCL run you will conclude your filter
is worse when it is not.

Three traps here:

- **`ros2 topic pub` cannot seed under `use_sim_time`.** It stamps with wall
  clock; the localizer rejects that as a future extrapolation and stays silently
  on its YAML default. Use `scripts/live_runs/seed_initialpose.py` — it runs as a
  real sim-time node, waits for the replayed `odom→base_link`, and stamps with 0.
- **Your node treats `/initialpose` differently from AMCL.** Per your config,
  an `/initialpose` message *triggers the coarse-to-fine hybrid global
  initialization* (`global_loc_*` params) rather than directly seeding a tight
  Gaussian, and after `global_loc_timeout: 5.0` s with no pose it auto-triggers
  global localization anyway. **Verify in your own source what actually happens**
  and, if the hybrid path does not converge to the seed reliably, use
  `set_initial_pose: true` with the numbers above written into a params file for
  the replay. Either way, **prove the filter started where you think it did**
  before you score anything.
- `51_localize_offline.sh` **hard-aborts** if seeding fails rather than record an
  unseeded run. Keep that behaviour in whatever you write.

### 4.7 The concrete procedure

There is a script, `scripts/live_runs/51_localize_offline.sh`, that does the
whole replay-and-score loop. It currently supports `--publisher amcl|ekf`.
**Item 4 of my next-session list is "add a `--publisher pf` path" — that is
yours to write**, and the change is small: pass `launch_particle_filter:=True`
and `map_tf_publisher:=pf` instead of `launch_amcl:=True`. Coordinate with the
operator before editing my repo, or write your own copy and hand me the diff.

Baseline run today, no edits to my repo needed:

```bash
export ROS_DOMAIN_ID=42
export CYCLONEDDS_URI=file:///mnt/shared_dir/cyclonedds_offline_lo.xml
export MAP_ROOT=/mnt/shared_dir/maps/20260805
export BAG_ROOT=/mnt/shared_dir/bags/20260805
export NS=gosling1
cd /workspaces/f1tenth/src/f1tenth_launch/scripts/live_runs

# 1. Bring up map_server + your node, no sensors, no vehicle:
ros2 launch f1tenth_launch localization.launch.py \
    use_namespace:=True namespace:=gosling1 \
    use_sim_time:=True use_composition:=False use_gpu:=False \
    map_file:=$MAP_ROOT/rtabmap_2d_final.yaml \
    launch_map_server:=True launch_amcl:=False \
    launch_particle_filter:=True map_tf_publisher:=pf \
    launch_sensor_fusion:=False launch_ekf_odom:=False launch_ekf_map:=False \
    launch_slam_toolbox_localizer:=False launch_rtabmap_localizer:=False \
    launch_pointcloud_odometry:=False launch_rgbd_odometry:=False \
    launch_stereo_odometry:=False launch_laserscan_odometry:=False \
    launch_icp_odometry:=False \
    odom_tf_publisher:=bag autostart:=True log_level:=info

# 2. In a second shell: record what the localizer produces
ros2 bag record -o $BAG_ROOT/pf_mapping_drive_170025 -s mcap --use-sim-time \
    /gosling1/tf /gosling1/tf_static /gosling1/odometry/local  # + your pose topics

# 3. In a third: replay ONLY what a localizer consumes (5 topics, ~3.5 min at 1.0x;
#    the full bag is 25 GB of camera data nothing here subscribes to)
ros2 bag play $BAG_ROOT/mapping_drive_170025 --clock 100 --rate 1.0 --topics \
    /gosling1/tf /gosling1/tf_static /gosling1/lidar/scan_filtered \
    /gosling1/odometry/local /gosling1/vehicle/vesc_odom

# 4. Seed (as soon as playback starts)
python3 ./seed_initialpose.py --ns gosling1 --x 0.4451 --y -0.5750 --yaw -1.3931

# 5. Score it against the same truth AMCL was scored on
cd ../analysis
python3 check_map_frame.py $BAG_ROOT/pf_mapping_drive_170025 \
    --map $MAP_ROOT/rtabmap_2d_final.yaml \
    --truth $MAP_ROOT/truth_mapping_drive_170025.csv
```

`check_map_frame.py` reads `map→odom` out of `/gosling1/tf`, composes it with the
bag's `odom→base_link`, and compares to the truth CSV. It is publisher-agnostic —
it does not care that the transform came from you instead of AMCL. It also reads
`/gosling1/amcl_pose` for covariance reporting if present; yours will not be,
which is fine.

**Report `correction step p95/max` explicitly.** If `check_map_frame.py` does not
print it, compute it: the per-update translation delta of consecutive distinct
`map→odom` transforms. Smoothness, not mean error, is the complaint you are
answering.

### 4.8 Why the truth CSV, and why not identity

`mapping_drive_170025` is the bag the map was *built* from, so RTABMap kept its
own optimized pose graph for it — that graph is the ground truth
(`rtabmap_ground_truth.py` extracts it; the CSV is already on the SSD).

**Do not score a mapping bag against identity.** RTABMap optimizes the graph
*from the end of the trajectory*, so `map` coincides with `odom` at the **last**
keyframe, not the first. The true `map→odom` at t=0 is
(+0.4545, −0.5746, −25.33°), decaying to zero. Scoring against identity charges
the localizer for drift it correctly removed: the same pipeline scores **267 mm**
against identity and **65 mm** against the graph. This mistake has already been
made twice in this project.

Caveat to carry into your writeup, honestly: **65 mm is agreement between two
estimators sharing one LiDAR and one map**, not absolute accuracy. Their errors
are correlated. Quote it with that qualifier, as I do.

---

## 5. Gotchas that will cost you time

- **`set -u` + `source /opt/ros/humble/setup.bash` silently kills a piped
  script.** The ROS setup files reference unset variables. `00_env.sh` works
  around it; ad-hoc `docker exec bash -s` scripts must too.
- **`ros2 bag record --ros-args -p use_sim_time:=true` dies on startup** — `-p`
  is `--polling-interval`. The flag is `--use-sim-time`.
- **A sim-time deadline computed before `/clock` exists is instantly expired.**
  `get_clock().now()` reads 0 until the first `/clock`, then jumps to bag time,
  so `now() + 60 s` blows through on the first spin. Use `time.monotonic()` for
  anything guarding sim-time startup.
- **Nodes survive a hard kill of the launch parent** and contaminate the next
  pass. Reap explicitly between runs:
  `pkill -9 -f nav2_map_server/map_server; pkill -9 -f particle_filter`.
- **`pkill -f <pattern>` typed inside `docker exec bash -lc "…"` matches the
  exec'd shell's own command line and kills it.** Pipe a script on stdin instead:
  `docker exec -i <c> bash -s < script.sh`.
- **`ros2 topic list` is not trustworthy on this vehicle.** Under the static-peer
  DDS config, one pass reported four topics "missing" that were all publishing at
  30 Hz. Retry, or ask about the topic directly.
- **Under `rmw_fastrtps_cpp`, composable nodes do not appear in
  `ros2 node list`.** Use `ros2 component list`. Not an issue under CycloneDDS,
  which is what you should be using.
- **Watch memory.** 7 GB total; two processes were SIGKILLed at ~45 s during a
  replay that also had the camera stream running. Yours will not (5 topics only),
  but log RSS anyway.
- The map: 265×199 cells at 0.05 m with **only 1732 occupied cells**. This is a
  sparse room, not a corridor maze. It is why the stock AMCL likelihood field was
  far too flat to converge, and it is the reason `sigma_hit` matters so much here.

---

## 6. What "done" looks like

A short report containing:

1. **Confirmation the package is installed and the node actually ran** — with
   evidence it localized (a non-identity, time-varying `map→odom`), not just that
   it published.
2. **`map→odom` correction step p95 and max**, against AMCL's 14.6 / 59.6 mm.
   This is the headline.
3. **Translation/yaw error mean, p95, max, and % inside 126 mm**, against
   AMCL's 64.7 / 143.5 / 236.2 mm, 1.69°, 87.3 %.
4. **CPU (% of one core, mean and peak) and peak RSS**, from `/proc/<pid>/stat`,
   at `max_particles: 2000` — and, if you tune it, at whatever you land on.
5. Any config changes, with the measurement that justifies each.
6. A `--publisher pf` path for `51_localize_offline.sh` (hand me the diff).

Run the **control bag first**. Only once that is scored is it worth touching
`figure8_172338` and `loop_laps_173558` (which have no ground truth — for those,
the checks are cross-run start-pose agreement, loop closure, path length, and
100 % of poses in mapped free space).

---

## 7. HARD CONSTRAINT — the frozen deliverable

`/mnt/shared_dir/deliverables/20260805/` holds three derived bags that **two
other Claudes on other machines are consuming right now**, along with the
accuracy numbers above, which they are quoting. They cannot see this repo and
will not be told if the ground moves under them.

- **Never write into `deliverables/20260805/`.** It is `chmod a-w`, but the
  container runs as uid 0 and root writes straight through that — tested. **The
  rule is the protection, not the bits.**
- Do not modify `bags/20260805/` or `maps/20260805/` either — v1's provenance
  points at those exact files. Read only. Write your outputs to new directories.
- `md5sum -c MD5SUMS.txt` in the deliverables directory before trusting or
  re-quoting anything.
- If your filter genuinely beats v1, that is a **good outcome and not permission
  to regenerate v1 in place.** It goes to `deliverables/20260805_v2/`, v1 stays
  intact and readable, and the operator is told in writing what changed and by
  how much. A silent swap is worse than no swap.

---

## 8. Who to ask

The operator (Boluwatife) is your only channel to me and to the machine. If you
need a change in `f1tenth_launch` — a launch arg, a remap, the `--publisher pf`
path merged — route it through them with a diff. Ask before assuming a
constraint here is wrong; several of the odd-looking choices above are load-
bearing and were paid for in measurements.
