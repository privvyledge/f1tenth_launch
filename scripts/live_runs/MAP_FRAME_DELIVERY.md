# Map-frame pose for the 2026-08-05 f1tenth runs — delivery notes

**Written 2026-08-05 ~21:10 EDT.** Everything below was measured, on `gosling1`,
by localizing the three drive bags against the map built earlier the same day.
This is the document to hand to the LUCIO consumer along with the bags.

---

## Version and freeze status — read before regenerating anything

**This is `v1`, and `deliverables/20260805/` is FROZEN.** Consumers on other
machines are working against these exact files and against the numbers in this
document. If either changes underneath them, their analyses silently shift and
nothing tells them.

| | |
|---|---|
| version | **v1** |
| frozen | 2026-08-05 21:40 EDT |
| built from commit | `ffa58c5` (branch `perf/config-tuning`) |
| localizer | nav2_amcl, `config/localization/localizer_amcl_mapframe.yaml` |
| broadcaster | AMCL direct (`map_tf_publisher:=amcl`) |
| map | `maps/20260805/rtabmap_2d_final.yaml` |
| seed | `(+0.4451, -0.5750, -1.3931 rad)`, identical for all three runs |
| source bags | `bags/20260805/{mapping_drive_170025,figure8_172338,loop_laps_173558}` |
| intermediate | `bags/20260805/loc2_*` (the raw localization captures) |

MD5 of the delivered `.mcap` files:

```
34357ec79bf29858eb909c7a0600944b  mapframe_figure8_172338/mapframe_figure8_172338_0.mcap
9320714b46decd7b6acb7636d835dbc1  mapframe_loop_laps_173558/mapframe_loop_laps_173558_0.mcap
46d9d3c8440c4d390301de0ac5bec2aa  mapframe_mapping_drive_170025/mapframe_mapping_drive_170025_0.mcap
```

The directory is `chmod a-w` on `gosling1`, but **do not rely on that**: the
container runs as uid 0, and root bypasses the permission bits entirely. Tested
2026-08-05 21:41 — a `touch` into the "read-only" directory succeeded. The bits
are a visual cue only.

What actually protects this is (a) the rule, written here and in
`NEXT_CHAT_PROMPT.md`, and (b) `MD5SUMS.txt`, which makes any change
**detectable**. Before trusting a copy of these bags:

```bash
cd /mnt/shared_dir/deliverables/20260805 && md5sum -c MD5SUMS.txt
```

### If a better localizer comes along

Likely: the RTABMap localizer is still unmeasured, and the two `ekf_map.yaml`
items are unresolved. Any of those could produce a better `map->odom`. When it
does:

1. **Write to a NEW directory** — `deliverables/20260805_v2/` — never in place.
   `make_map_frame_bag.py` refuses to overwrite an existing output for exactly
   this reason; do not work around it with `rm -rf`.
2. **Leave v1 intact and readable** until every consumer has moved.
3. **Re-run `verify_map_frame_bag.py`** on v2. Passing verification is what
   "delivered" means here; it is not optional for a second version.
4. **Quantify the improvement against the same control** — `mapping_drive_170025`
   scored against `truth_mapping_drive_170025.csv`. v1 is mean 64.7 mm /
   p95 143.5 mm. A v2 that does not clearly beat that is churn, not progress.
5. **Tell the consumers, in writing, what changed and by how much.** They cannot
   see this repo. A silent swap is worse than no swap: their fits would move and
   they would have no reason to suspect why.

## What you get

`/mnt/f1tenth_ssd/shared_dir/deliverables/20260805/` on `gosling1`
(`/mnt/shared_dir/deliverables/20260805/` inside a container):

| derived bag | from | pose_map msgs | map->odom tfs |
|---|---|---|---|
| `mapframe_mapping_drive_170025` | `mapping_drive_170025` | 4403 | 1247 |
| `mapframe_figure8_172338` | `figure8_172338` | 4651 | 1333 |
| `mapframe_loop_laps_173558` | `loop_laps_173558` | 3088 | 886 |

Each has a sidecar `.README.md`. They carry **no sensor data** — play them
alongside the original bags. Both requested shapes are present in each:

- **`/gosling1/tf`** — only `map` -> `odom`, nothing else. Composes onto the
  original bag's untouched `odom` -> `base_link`.
- **`/gosling1/pose_map`** — `nav_msgs/msg/Odometry`, `frame_id: map`,
  `child_frame_id: base_link`, already composed, 30 Hz.

All three are localized into **one** `map` frame: the grid
`maps/20260805/rtabmap_2d_final.yaml`, built from `mapping_drive_170025`.

### The frame is the REAR AXLE

`base_link` is the **rear axle** on this vehicle (`base_link` -> `rear_axle` is
the identity transform in `vehicle/static_transformations.launch.py`). It is
**not** `base_footprint` (33 mm below in z) and **not** `front_axle` (256 mm
ahead in x). This is stated because a wrong choice here is a ~0.13 m error that
no check on either side would catch — the same size as the whole accuracy
budget.

### Verified before handover

`verify_map_frame_bag.py` passes on all three:

- every `pose_map` header stamp is identical, **sec and nanosec**, to the
  `odometry/local` message it was derived from — nothing was re-stamped,
  re-sampled or re-ordered, so a merge on header stamp behaves exactly as it
  does against the original bag;
- `pose_map` equals `tf` composed with the source `odom` -> `base_link` to
  **0.0 nm**, so the two shapes cannot disagree;
- 30.01 Hz mean across every run, worst gap 63 ms, against the >=20 Hz ask;
- stamps strictly increasing, bag write order monotonic;
- the `tf` topic contains map->odom and nothing else.

---

## How accurate is it

**Headline: mean 65 mm, p95 144 mm against an independent estimate of the same
trajectory. 87% of samples inside the 126 mm bar.** Read the caveat below before
quoting that number.

### The measurement

`mapping_drive_170025` is a control with real ground truth available: RTABMap
built the map from it and kept its own optimized pose graph, so the true
`map->odom(t)` for that run can be read straight out of `rtabmap_final.db`
(`rtabmap_ground_truth.py`). Localizing that bag with exactly the same pipeline
used for the other two, and comparing:

| | |
|---|---|
| translation error | **mean 64.7 mm, p95 143.5 mm, max 236.2 mm** |
| yaw error | mean 1.69 deg, p95 4.92 deg, max 7.19 deg |
| inside 126 mm | **87.3 %** of samples |

Per 15 s window: 6, 48, 92, 120, 108, 78, 54, 75, 33, 26 mm — worst in the
middle of the run, tightening to ~25 mm by the end.

### The caveat, stated plainly

That comparison is **agreement between two estimators, not absolute accuracy**.
AMCL and RTABMap both localize the same YDLidar scans against the same room, so
their errors are correlated; a systematic error common to both is invisible
here. Treat 65 mm as a lower bound on what an independent measurement would
report. LUCIO's cross-machine pixel-motion-vs-odom-velocity check is exactly the
external evidence this cannot provide, and is worth running.

### Supporting evidence that does not depend on that caveat

- **AMCL's own converged covariance**, which it computes independently of any
  ground truth: sigma 75/66 mm and 4.45 deg (mapping_drive), 85/63 mm and
  3.69 deg (figure8), 73/72 mm and 3.86 deg (loop_laps). Consistent across all
  three, and consistent with the error measured on the control.
- **Cross-run start-pose agreement.** The operator hand-placed the car on the
  same tile-aligned pose for all three runs. Localized independently, the three
  first map-frame poses come out at (+0.446, -0.574, -79.87 deg),
  (+0.445, -0.575, -79.81 deg) and (+0.444, -0.573, -79.85 deg) — agreeing to
  **2 mm and 0.06 deg**. Three independent runs landing on one physical pose is
  not something a mislocalized filter does.
- **Loop closure improved by the map frame.** In each run's own odom frame the
  car ends 0.88 m (mapping_drive), 0.36 m (figure8) and 1.17 m (loop_laps) from
  where it started. In the map frame those become **0.04, 0.07 and 0.08 m** —
  the localizer removed odometry drift rather than adding its own.
- **Path length sanity.** 52.19 / 41.14 / 38.08 m in the map frame against
  53.2 / 40.6 / 37.6 m of raw odometry — within 2%, so the correction is not
  injecting spurious motion.
- **100% of every path lies in mapped free space** (trinary PGM value 254; free,
  not unknown). No pose lands in a wall or off the map.

### Known floors and residuals

- **Map resolution — and why it is NOT the lever.** The grid is 0.05 m/cell, so
  cell quantisation alone is ~+-25 mm before any localizer error.
  **Corrected 2026-08-05 23:45:** this file previously said that a consumer
  needing materially better than 65 mm should first rebuild the grid at 0.025 m
  (`Grid/CellSize`). The arithmetic does not support that. Quantisation over a
  0.05 m cell is a uniform error of sigma = 50/sqrt(12) = **14.4 mm**, which
  against a 64.7 mm total leaves **63.1 mm** of everything-else. Halving the cell
  size takes quantisation to 7.2 mm and the total to **63.5 mm** — a ~1 mm
  improvement for a full map rebuild. Even treating the +-25 mm bound itself as
  the sigma (the pessimistic reading) only predicts 65 -> 61 mm.
  The error is dominated by the localizer, not the grid: AMCL's own converged
  sigma is 75/66 mm, the same order as the total. **Do not rebuild the map for
  accuracy.** If 65 mm is genuinely not enough, the levers are the localizer and
  the sensor geometry — and note the headline is agreement between two estimators
  sharing one LiDAR, so an independent check (LUCIO's pixel-motion comparison)
  should come before any of it.
- **`map->odom` is post-dated by 1.0 s.** nav2_amcl stamps the transform with
  `scan_time + transform_tolerance`, which is how it is supposed to work and
  what a live tf2 consumer would also see. Measured cost of the resulting
  attribution lag: **~3 mm** (mean error 64.7 mm at zero lead vs 61.7 mm when the
  stamps are shifted back by 1.0 s). Left as AMCL published it, so tf2 behaves
  identically to live. Not worth correcting at this magnitude.
- **The vehicle's odometry drifts ~25 deg of yaw per ~150 s run**, which is what
  `map->odom` swings by over each bag (-25.5 deg on mapping_drive, -56 to
  -33 deg on figure8, -55 to -39 deg on loop_laps). That is a real property of
  these bags and is consistent with the known asymmetric steering calibration
  (`steering_angle_to_servo_offset: 0.56`). The localizer removes it; nothing
  downstream should assume `odom` yaw is trustworthy over a full run.

---

## Two corrections to the earlier handoff

Both were load-bearing and both were wrong. `MAP_BUILD_HANDOFF.md` has been
updated.

### 1. `map` does not coincide with `odom` at the START of the mapping run

It coincides at the **end**. RTABMap optimized that graph from the end of the
trajectory, so its own optimized poses give a true `map->odom` of
**(+0.4545, -0.5746, -25.33 deg) at t=0**, decaying to exactly zero at the last
keyframe. Consequences:

- The shared physical start pose in the map frame is
  **(+0.445, -0.575, -79.82 deg)**, not ~(0, 0, -54.5 deg).
- The per-run `map->odom` rotations previously quoted as **-30.96 deg**
  (figure8) and **-29.51 deg** (loop_laps) are each off by the same 25.33 deg.
  Measured values from the localizer: **-56.35 deg** and **-54.91 deg**, against
  a prediction of -56.29 and -54.84 from the corrected start pose. They are
  outputs, not inputs — seed the pose and let AMCL produce them.
- Seeding at the origin is not free: measured, AMCL needed **~28 s of driving**
  to pull in from the 0.73 m / 25 deg seed error, and every pose in that window
  misses the 126 mm bar.

### 2. A mapping bag's localization must not be scored against identity

For the same reason. The true `map->odom` on `mapping_drive_170025` reaches
0.76 m and 25.3 deg, so scoring against identity charges the localizer for drift
it correctly removed. Scored that way the run reported 267 mm mean error; scored
against RTABMap's optimized graph, with the seed also corrected, the same
pipeline reports 65 mm. Use `rtabmap_ground_truth.py`.

This also disposes of the previous session's unexplained result that naive scan
matching "failed its own ground-truth control" by best-fitting `mapping_drive`
at -2.29 deg instead of 0. There was nothing wrong with the control: the true
value simply is not 0. It sweeps from -25.33 deg to 0 over the run.

---

## Why AMCL, and how it is configured

**AMCL broadcasts `map->odom` directly** (`map_tf_publisher:=amcl`), not the
global EKF. The earlier handoff left this as the operator's call and leaned
toward `ekf_map`; measurement and a config read both argue against it *for this
deliverable*:

- `ekf_map.yaml` has **`pose0_relative: true`** on `amcl_pose`. In
  robot_localization that treats the first measurement as the origin, which
  discards AMCL's absolute map pose — precisely the thing being delivered.
- Its **`odom1` is `visual_slam/vis/slam_odometry` with
  `differential: false, relative: false`**, i.e. an absolute global anchor in
  the map frame. That topic is not in the map frame; the same handoff records
  VSLAM sitting ~54 deg off the real `odom` frame on `mapping_drive_170025`.
  Feeding it in as absolute would drag the map frame.

Neither is a reason to abandon `ekf_map` for live driving, where smoothing
AMCL's jumps is the point. They are reasons not to route a metrology deliverable
through it. **Both are worth fixing as config defects** — see "Still open".

`config/localization/localizer_amcl_mapframe.yaml` is a separate file, not a
change to `localizer_amcl.yaml`, because the live config's choices are correct
for live use. The stock config never converged on this map — it reported its own
sigma as 419 mm / 16.7 deg — because its likelihood field is far too flat for a
0.05 m grid with only 1732 occupied cells: `sigma_hit` 0.4 m smears each beam
over 8 cells, `z_rand` 0.5 puts half the mass on noise, and `max_beams` 90 keeps
one beam in seven of the X4's ~625. Retuned (`sigma_hit` 0.15, `z_hit`/`z_rand`
0.80/0.20, 200 beams, 1000-4000 particles, `update_min_d` 0.05) it converges to
75 mm / 4.5 deg. The RTABMap localizer, which the earlier handoff also wanted
measured, was not needed once AMCL cleared the bar. It has since been measured
(`LOCALIZER_FOLLOWUPS.md`) and does not work at all.

**The tuning in `localizer_amcl_mapframe.yaml` has since been folded into the
live `localizer_amcl.yaml`**, after measuring that the particle count costs 4.1 %
of one core rather than the assumed CPU problem. **This file is unchanged** — it
is this deliverable's provenance and must keep matching what is written here.

---

## Reproducing this

Inside the offline container (no hardware needed):

```bash
export ROS_DOMAIN_ID=42          # NOT 0 — keeps clear of a live stack
export CYCLONEDDS_URI=file:///mnt/shared_dir/cyclonedds_offline_lo.xml
export MAP_ROOT=/mnt/shared_dir/maps/20260805
export BAG_ROOT=/mnt/shared_dir/bags/20260805
cd /workspaces/f1tenth/src/f1tenth_launch/scripts/live_runs

for b in mapping_drive_170025 figure8_172338 loop_laps_173558; do
  ./51_localize_offline.sh --bag $BAG_ROOT/$b \
      --params ../../config/localization/localizer_amcl_mapframe.yaml \
      --out loc2_$b
done

cd ../analysis
python3 rtabmap_ground_truth.py $MAP_ROOT/rtabmap_final.db --out $MAP_ROOT/truth.csv
python3 check_map_frame.py $BAG_ROOT/loc2_mapping_drive_170025 \
    --map $MAP_ROOT/rtabmap_2d_final.yaml --truth $MAP_ROOT/truth.csv
python3 make_map_frame_bag.py --source $BAG_ROOT/$b \
    --localization $BAG_ROOT/loc2_$b --out /mnt/shared_dir/deliverables/20260805/mapframe_$b
python3 verify_map_frame_bag.py --source $BAG_ROOT/$b --derived .../mapframe_$b
```

Each localization pass is ~3.5 min: only 5 topics are replayed (tf, tf_static,
`lidar/scan_filtered`, `odometry/local`, `vehicle/vesc_odom`), so real-time
playback is comfortable and the 25 GB of camera data is never touched.

### Gotchas that cost real time here

- **`ros2 topic pub` cannot seed an initial pose under `use_sim_time`.** It
  stamps with wall time; AMCL rejects that as a future extrapolation and stays
  on the `(0,0,0)` YAML default — a localizer that looks completely healthy and
  is simply in the wrong place. `seed_initialpose.py` runs as a real sim-time
  node and stamps with 0. `51_localize_offline.sh` now aborts if seeding fails
  rather than recording an unseeded run.
- **`ros2 bag record --ros-args -p use_sim_time:=true` dies on startup** — `-p`
  is `--polling-interval`. The flag is `--use-sim-time`.
- **A sim-time deadline computed before `/clock` exists is instantly expired.**
  `get_clock().now()` reads 0 until the first `/clock` message and then jumps to
  bag time, so `now() + 60 s` blows through on the first spin. Wall-clock
  timeouts (`time.monotonic()`) for anything guarding sim-time startup.
- **`rtabmap_final_nf.db` has no optimized graph** (`Admin.opt_poses` is NULL);
  `rtabmap_final.db` does. The Admin blobs are zlib-compressed; `Node.pose` is
  not.
- **`set -u` plus `source /opt/ros/humble/setup.bash` silently kills a piped
  script** — the setup files reference unset variables. `00_env.sh` already
  works around this; ad-hoc `docker exec bash -s` scripts must too.

---

## Still open

> **Items 1, 2 and 4 were worked on 2026-08-05 ~22:00-22:40 and are resolved or
> answered — see `LOCALIZER_FOLLOWUPS.md`. None of it changed this deliverable:
> nothing beat 64.7 mm, so there is no v2 and consumers need no notification.**

1. ~~**`ekf_map.yaml`'s `pose0_relative: true`** discards AMCL's absolute pose.~~
   **Fixed**: `pose0_relative: false`.
2. ~~**`ekf_map.yaml`'s `odom1`** treats `visual_slam/vis/slam_odometry` as an
   absolute map-frame anchor when it is not in the map frame.~~ **Fixed**: the
   input is now switched by the launch (`fuse_vslam_global`, default off, set
   from `use_gpu` and `localize_on_startup`) rather than by editing the file,
   because the config is correct in the localized-into-a-saved-map case.
3. **`rtabmap_2d_overfiltered.yaml` points at `image: rtabmap_2d_final.pgm`** —
   it names the wrong PGM, so loading it silently gets the good grid with the
   wrong origin. Cosmetic today because that file is marked do-not-use.
4. ~~**The RTABMap localizer is still unmeasured.**~~ **Measured, and it does not
   work.** Two separate blockers: its TF remap was malformed so it ran with an
   empty TF tree (bug-107, fixed), and even fixed it accepts zero loop closures —
   `rtabmap_final_nf.db` has an empty visual vocabulary (bug-108) and against
   `rtabmap_final.db` every candidate fails geometric verification. It costs ~1
   full core and ~1 GB while doing so, against nav2_amcl's 3-4 % of one core.
   AMCL remains the localizer.
5. **Waypoint generation** is now unblocked: all three trajectories are in one
   map frame, and it is pure post-processing on `pose_map`.
6. Everything already carried forward in `NEXT_CHAT_PROMPT.md` (udev fix on the
   other goslings, `vesc_driver` serial reconnect, `MAX_STEERING` back to 0.34
   after recalibration).

---

## 3. A third correction — the `-79.82 deg` start pose is itself off (2026-08-24)

**Added after v1 was frozen. It changes no delivered file and no delivered
number.** The bags, `MD5SUMS.txt` and every figure above stand exactly as
written; what changes is how two of them should be *read*.

`-79.82 deg` came from RTABMap's optimized poses at t=0. Evidence gathered since
says it is wrong relative to RTABMap's own rendered grid:

- On both 2026-08-05 runs with a genuine stationary window (car parked, <=5 mm of
  position change over 8-10 s), AMCL **walks away from the -79.80 deg seed** and
  reaches at least **-85.0 deg**, still moving (`mapping` -85.79 deg at
  -0.74 deg/s; `loop_laps` -85.00 deg at -0.07 deg/s). One-sided: neither run
  converged, so the endpoint has no upper bound from this data.
- Independently, the raw-scan fit at the same physical spot on 2026-08-11 gives
  **-84.50 deg** with no filter in the loop (`heading_from_scan.py`).
- The seed pull cuts the right way: AMCL was seeded at -79.80 deg, so any
  residual pull biases *toward* it, and it went the other way regardless.

Structurally this is unsurprising. That pose graph was optimized **from the end**
of the trajectory (see §1 above), so t=0 is its least-constrained end. The seed
was also read from `rtabmap_final.db` while the grid and clouds were rendered
from `rtabmap_final_nf.db` — two optimizations, whose `map` origins need not
coincide.

### What to re-read, and what not to

- **The delivered `pose_map` track is unaffected.** It is AMCL's `map->odom`
  composed with the source bag's `odom->base_link`. RTABMap's optimized poses
  enter only as the t=0 seed (which AMCL walks off, as above) and as the scoring
  control. Neither is written into a delivered bag.
- **Re-read the accuracy figures in "How accurate is it".** The
  **yaw error (mean 1.69 deg, p95 4.92 deg, max 7.19 deg)** was measured against
  `rtabmap_ground_truth.py`. If RTABMap's t=0 is off against its own grid, part
  of that disagreement is **RTABMap's error, not AMCL's** — a t=0 offset decaying
  to zero has qualitatively the shape of that profile. **This is deliberately not
  quantified**: the parked-AMCL data above is one-sided and cannot bound the t=0
  error tightly enough to set a number against 7.19 deg. The translation figures
  (mean 64.7 mm, p95 143.5 mm) inherit the caveat this document already stated —
  agreement between two estimators, not absolute accuracy.
- **Do not regenerate v1 over this.** Nothing here improves the delivered track;
  it reinterprets the control that track was graded against. If a v2 ever happens
  for other reasons, the rules in "If a better localizer comes along" still apply.
- **The replay seeds still say `-79.82 deg`** (`51_localize_offline.sh:81`,
  `61_nav2_offline.sh:90`, `check_map_frame.py:274`, `BRIEF_PARTICLE_FILTER.md`).
  Left as-is deliberately: a seed costs **convergence time, not correctness**, and
  changing it would make future replays non-comparable with the archived ones.

**Open, and the only thing that can put a number on this:** run
`heading_from_scan.py` against an archived 2026-08-05 bag's raw scan. Blocked
2026-08-24 (gosling1 reflashed, velox1 unreachable), not declined.

Full derivation and the LUCIO exchange that produced it:
`scripts/live_runs/LUCIO_MAP_HEADING_ANSWER.md`.
