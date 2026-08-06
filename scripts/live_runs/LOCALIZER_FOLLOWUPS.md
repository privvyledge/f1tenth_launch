# The three localizer follow-ups — measured

**Written 2026-08-05 ~22:30 EDT**, on `gosling1`, against the same control the
map-frame deliverable was scored on: `mapping_drive_170025` localized against
the 2026-08-05 map and scored with `check_map_frame.py --truth` (RTABMap's own
optimized graph). Every number below is measured, not inferred.

**The v1 deliverable is untouched.** `deliverables/20260805/`,
`maps/20260805/*` and `bags/20260805/{mapping_drive,figure8,loop_laps}` were
read only — nothing here produced a better `map->odom`, so there is no v2 and
nothing for the LUCIO or MPC consumers to be told about. `MAP_FRAME_DELIVERY.md`
stands as written.

Reference to beat, from `MAP_FRAME_DELIVERY.md`:

| | mean | p95 | inside 126 mm | localizer CPU |
|---|---|---|---|---|
| **v1: nav2_amcl, `localizer_amcl_mapframe.yaml`** | **64.7 mm** | **143.5 mm** | **87.3 %** | see §3 |

---

## 1. The RTABMap localizer — measured, and it does not work

Asked: accuracy against the same truth, and CPU on the Orin's 6 cores. Both
taken. The answer is that this path **has never been able to localize**, for two
independent reasons, and neither is about it being slow.

### 1a. It was structurally broken: the TF remap never applied (bug-107)

`3d_mapping.launch.py` declared its TF remap as

```python
SetRemap(src=['/tf', '/tf_static'], dst=['tf', 'tf_static'])   # WRONG
```

launch **concatenates each list into one substitution**, so the nodes were
started with the single nonsense rule `-r /tf/tf_static:=tftf_static` — visible
in the process command line. Neither real remap existed, so `rtabmap` and
`rgbd_sync` subscribed to the **root** `/tf` and `/tf_static` while every
transform was published on `/gosling1/tf`. The node therefore ran with an empty
TF tree:

```
"odom" passed to lookupTransform argument target_frame does not exist
TF of received image for camera 0 at time ... is not set!
Could not convert rgb/depth msgs! Aborting rtabmap update...
```

The mapping path never showed this because `mapping.launch.py` wraps the same
include in its own **correct** pair of `SetRemap` actions. The localization path
(`localization.launch.py` -> `3d_mapping.launch.py`, `launch_rtabmap_localizer:=True`)
has no such wrapper. So the RTABMap localizer could not have worked in any run,
ever; it had simply never been run.

**Fixed** — split into two `SetRemap` actions, the form used everywhere else in
the package. Verified in the running process command line
(`-r /tf:=tf -r /tf_static:=tf_static`) and by the TF errors going from
thousands to zero.

### 1b. With TF fixed it still never relocalizes

Four passes, all on the control bag at 1.0x with the camera replayed:

| pass | map database | accepted loop closures | `map->odom` | error vs truth |
|---|---|---|---|---|
| 2 | `rtabmap_final_nf.db` | 0 of 538 | identity, forever | 344.5 mm mean / 732.6 p95 |
| 3 | `rtabmap_final.db` | 0 of 50 | — (node SIGKILLed at ~45 s) | — |
| 4 | `rtabmap_final.db`, 150 s load wait | 0 of 67 | identity | — |
| 5 | `rtabmap_final.db` + `RGBD/LoopClosureReextractFeatures true` | 0 of 8 | identity | 343.6 mm mean / 732.6 p95 |

**`rtabmap_final_nf.db` cannot be used as a localization map at all** (bug-108):
its `Word` table is **empty** — the visual vocabulary was lost when the grid was
regenerated — against 43 446 words in `rtabmap_final.db`. RTABMap relocalization
is BoW-based, so it logs `Not found word ... (dict size=0)` and rejects every
candidate. Check any localization DB with
`sqlite3 map.db 'select count(*) from Word'` before trusting it. `nf.db` is
still the right source for the 2D and 3D **grids**; it is only useless as a
localization map.

With the good database it gets further but still fails: candidates are proposed,
features now match (`matches=28`), and every one is rejected —
`Not enough inliers 0/15`. So the visual registration itself is not working with
the current `Vis/*` settings on this map. That is a tuning problem nobody has
started on, not a "it is too slow" problem.

**The failure is silent, and that is the dangerous part.** A localizer that never
relocalizes keeps publishing a perfectly steady `map->odom` of exactly identity
at 20.8 Hz. Every liveness check passes. The 344 mm mean error it scores is not
localizer error — it is the odometry drift it failed to remove.

### 1c. The CPU number, for what it is worth

Sampled from `/proc/<pid>/stat`, percent of **one** core (6 cores = 600 %):

| pass | mean | peak | peak RSS |
|---|---|---|---|
| `nf.db`, no relocalization | 108.8 % | 123.0 % | 680 MB |
| `final.db` + re-extraction | 94.2 % | 144.2 % | 1064 MB |

So roughly **one full core, sustained, and ~1 GB**, while doing no successful
relocalization work — a floor, not the real cost. For comparison, nav2_amcl on
the same bag is **1.8 % of one core and 22 MB** (§3). The original assumption
that this is expensive for an Orin Nano was right in magnitude, even though it
was never the reason the path did not work.

Also seen in pass 3: `rtabmap` and `rgbd_sync` were both SIGKILLed ~45 s in on a
7 GB machine while the camera bag was replaying. Any future attempt needs to
watch memory.

### Recommendation

Leave `launch_rtabmap_localizer` off. The TF fix is worth keeping regardless —
it is a real defect that also affects anyone who tries this path later. Reviving
the localizer is a `Vis/*` tuning exercise against `rtabmap_final.db`, and it
starts a full core in the hole against AMCL.

### One unexpected corroboration of the v1 seed

`rtabmap_final.db`'s own saved last-localization pose is
`xyz = 0.443647, -0.574866  yaw = -1.393732 rad`. The seed used for all three
delivered runs was `(+0.4451, -0.5750, -1.3931)`. Those agree to **1.5 mm and
0.03 deg**, from an artifact that had no part in deriving the seed — the car
ended each run where it started, and `map` coincides with `odom` at the end of
the mapping run. Independent support for the delivered start pose.

---

## 2. The two `ekf_map.yaml` items — both resolved

### 2a. `pose0_relative: true` on `amcl_pose` — changed to `false`

This one was straightforwardly wrong. `relative` makes robot_localization
subtract the **first** measurement from every later one, which re-origins the
map frame onto wherever AMCL happened to be when it first published. In a filter
whose `world_frame` is `map`, that discards the only thing `amcl_pose`
contributes: the absolute pose on the grid.

Concretely, on these runs the start pose is `(+0.445, -0.575, -79.8 deg)`, so
the whole map frame would have been offset by **0.73 m and 80 deg** — every Nav2
goal in map coordinates wrong by that much, with a filter that looks perfectly
healthy from the outside.

### 2b. `odom1` = `visual_slam/vis/slam_odometry` — now switched by the launch

The config (`differential: false, relative: false`, an absolute map-frame
anchor) is **correct when Isaac VSLAM is localized into a matching saved map**,
and wrong when VSLAM starts fresh — which is the default, since
`localize_on_startup` defaults to `False`, and is what these bags recorded
(VSLAM ~54 deg off the real `odom` frame on `mapping_drive_170025`).

So it is decided by startup mode rather than edited flat:

```
localization.launch.py            fuse_vslam_global := use_gpu AND localize_on_startup
  -> dual_ekf.launch.py           passes it through
     -> ekf_map.launch.py         odom1 := visual_slam/vis/slam_odometry            (true)
                                  odom1 := visual_slam/vis/slam_odometry__NOT_FUSED (false, default)
```

When off, `odom1` points at a name nobody publishes, so the input is inert and
`ros2 param dump` shows plainly that it is disabled rather than looking like a
typo. `ekf_map.yaml` keeps the input configured for the case in which its
settings are right, with the condition written next to it.

**Verified in the container** (no hardware needed), both ways:

| launched with | `ekf_map_node` `odom1` |
|---|---|
| `use_gpu:=False localize_on_startup:=False` (the default) | `visual_slam/vis/slam_odometry__NOT_FUSED` |
| `use_gpu:=True localize_on_startup:=True` | `visual_slam/vis/slam_odometry` |

In both cases the node starts clean, reports `pose0_relative: False`, and
publishes `odometry/global`. Checked through `dual_ekf.launch.py` directly and
through `localization.launch.py`, so the whole chain is exercised.

**Still not exercised on the car.** What a live bringup should confirm: that
`ekf_map` converges with AMCL feeding it and that `map->odom` is steady while
driving.

---

## 3. Folding the AMCL tuning into the live config — do NOT fold it partially

The proposal was that `sigma_hit`, `z_hit`/`z_rand` and `max_beams` are close to
free and worth folding into `localizer_amcl.yaml`, while the particle count and
`update_min_d` need an Orin benchmark first. Both halves were measured on the
control bag.

| config | particles | `update_min_d` | likelihood params | mean err | p95 | inside 126 mm | final sigma | AMCL CPU |
|---|---|---|---|---|---|---|---|---|
| live `localizer_amcl.yaml` (from `MAP_FRAME_DELIVERY.md`) | 100–500 | 0.2 | stock | 267 mm | 534 mm | — | 419 mm / 16.7 deg | not taken |
| **candidate: live + the three "free" params** | 100–500 | 0.2 | tuned | **345 mm** | **733 mm** | **20.4 %** | 150/119 mm, 10.2 deg | **1.8 % of one core, 22 MB** |
| `localizer_amcl_mapframe.yaml` (v1) | 1000–4000 | 0.05 | tuned | **64.7 mm** | 143.5 mm | 87.3 % | 75/66 mm, 4.45 deg | see below |

**The partial fold-in does not converge.** Sharpening the likelihood field while
leaving 100–500 particles and a 0.2 m update gate makes the filter *worse*, not
better: it never locks on, its own sigma stays at 150 mm / 10 deg, and its error
is indistinguishable from having no localizer at all (identity scores 344 mm on
this bag). A sharper likelihood field needs enough particles to sample it and
enough updates to exploit it; the three parameters are not separable from the
other two.

So: **do not fold in `sigma_hit` / `z_hit` / `z_rand` / `max_beams` on their
own.** It is either the whole configuration or none of it.

### And the particle count turns out to be affordable, so the whole thing is folded in

The reason the particle count was held back was a CPU-budget worry on the Orin
Nano. Measured, from `/proc/<pid>/stat` during a 1.0x replay (the scan is 8.5 Hz
in the bag exactly as it is live):

| config | AMCL CPU, mean | peak | RSS |
|---|---|---|---|
| live (pre-fold, 100–500 particles, 90 beams) | 1.8 % of one core | 3.4 % | 22 MB |
| **`localizer_amcl_mapframe.yaml` (1000–4000 particles, 200 beams, `update_min_d` 0.05)** | **4.1 % of one core** | **20.4 %** | **21 MB** |

That is ~2 % of **one** of six cores for the whole accuracy difference — 267 mm
to 65 mm. The budget worry does not survive measurement, so **the full tuned
configuration has been folded into `config/localization/localizer_amcl.yaml`**:
`sigma_hit` 0.15, `z_hit`/`z_rand` 0.80/0.20, `z_max`/`z_short` 0, `max_beams`
200, `laser_likelihood_max_dist` 2.0, `max_particles` 4000, `min_particles` 1000,
`pf_err` 0.02, `update_min_d` 0.05, `update_min_a` 0.1.

Deliberately **not** folded in, because they are replay-specific: `use_sim_time`
(stays `False`) and `set_initial_pose` (stays `True` with the 0,0,0 pose — live
gets its pose from RViz or a launch-time seed, not from a file written for a bag).

`localizer_amcl_mapframe.yaml` is left byte-for-byte unchanged: it is the
provenance of the frozen v1 deliverable and must keep matching what
`MAP_FRAME_DELIVERY.md` says was used.

### The folded file was then re-measured as the file the car will load

Not the offline file — `config/localization/localizer_amcl.yaml` itself, replayed
against the same control and truth:

| | mean | p95 | inside 126 mm | AMCL CPU |
|---|---|---|---|---|
| v1 offline file (`..._mapframe.yaml`) | 64.7 mm | 143.5 mm | 87.3 % | 4.1 % / 20.4 % peak |
| **folded live file (`localizer_amcl.yaml`)** | **63.9 mm** | **147.4 mm** | **88.4 %** | **3.3 % / 7.6 % peak** |

Same accuracy, so the fold-in is faithful. The one difference is at t=0: the live
file keeps `set_initial_pose: True` with a `(0,0,0)` pose, so the very first
`map->odom` is identity for one sample until the seed arrives (visible as the
`max 740 mm` outlier). That is correct live behaviour — the car starts where the
file or RViz says — and it is why this file was not simply swapped for the
offline one. **v1 was not regenerated with it**; the numbers above are a
validation run, and the frozen deliverable is untouched.

### Caveats on the folded values

- Measured on **one map** (raslab, 265x199 @ 0.05 m, 1732 occupied cells) and
  **one bag** (~1 m/s peak). `update_min_d` 0.05 means update rate scales with
  speed: at 1.5 m/s expect ~1.5x the update count and roughly that much more CPU.
  Still small, but it is not a constant.
- `recovery_alpha_slow`/`fast` remain 0.0 — no random-particle injection, so this
  configuration cannot recover from a kidnap. That was already true before the
  fold-in; it is called out in the file now.
- **Not yet run on the car.** The next live bringup should confirm AMCL converges
  from its startup pose and that `map->odom` is steady while driving.

---

## 4. Follow-on, decided 2026-08-05 23:45 after the consumers reported back

Two consumer complaints arrived: the MPC's global pose is **jerky**, and LUCIO's is
**less accurate than they would like**. They have different causes and only one of
them is a defect-shaped problem.

### Jerkiness is the zero-order hold, and it is expected

The delivered `pose_map` is **AMCL broadcasting `map->odom` directly**, composed
with the EKF's 30 Hz `odom->base_link` — `ekf_map` was deliberately routed around
because of the two defects fixed in §2. `map->odom` is therefore **piecewise
constant at the 8.5 Hz scan rate**: measured steps p95 **14.6 mm**, max **59.6 mm**.
A smooth 30 Hz curve with 8.5 Hz stair-steps in it; differentiated for velocity,
the steps become spikes. That is what the MPC is seeing.

Fix, in order: measure `ekf_map` as the broadcaster (`--publisher ekf`, with
`map_frequency` raised from 10.0 to 30.0) — smoothing AMCL's jumps is exactly what
that filter is for, and it only became usable today; and for LUCIO specifically,
smooth `map->odom` offline over the localizer's own corrections, which needs no
filter tuning at all and would be a legitimate `v2` under the freeze protocol.

### Accuracy is NOT floored by the grid — correcting earlier guidance

`MAP_FRAME_DELIVERY.md` used to say the first lever for better than 65 mm was
rebuilding the grid at 0.025 m. **That was wrong and has been corrected in place.**
Quantisation over a 0.05 m cell is uniform with sigma = 50/sqrt(12) = **14.4 mm**;
against a 64.7 mm total that leaves 63.1 mm of everything-else, so halving the cell
predicts **63.5 mm** — about 1 mm for a full map rebuild. Even the pessimistic
reading (treating the +-25 mm bound as the sigma) only gives 65 -> 61 mm. The error
is dominated by the localizer, whose own converged sigma is 75/66 mm. Do not
rebuild the map for accuracy.

And the honest caveat still applies: 65 mm is **agreement between two estimators
sharing one LiDAR and one map**, so LUCIO's independent pixel-motion check should
land before anyone spends effort chasing the number down.

### `particle_filter`: measure it, expect smoothness not accuracy

`localizer_pf.yaml` runs `mcl_hz: 40.0` decoupled from the odom rate, so it
predicts through the motion model between scans and emits a weighted mean —
structurally smoother than AMCL's 8.5 Hz resampled jumps. Accuracy should not be
expected to improve: same LiDAR, same grid, same information, same floor. It is
also the last localizer here that has never been measured, and §1 is a lesson in
what that is worth — so step one is confirming the package is installed at all,
since `localization.launch.py` swallows `PackageNotFoundError` silently.

---

## 5. `ekf_map` as the broadcaster — measured 2026-08-06

> **Superseded in its conclusion by §6.** Everything measured here stands, but
> the "untested lever" at the end of this section was then tested and it *works*:
> with `odom0` fused from `odometry/local` instead of `vehicle/vesc_odom`,
> `ekf_map` matches AMCL on smoothness and beats it on accuracy. Read §6 for the
> current answer; this section is the road to it.

§4 predicted that routing `map->odom` through `ekf_map` would smooth away the
8.5 Hz staircase the MPC is complaining about, since "smoothing AMCL's jumps is
exactly what that filter is for". **Measured, that is wrong.** Same control
(`mapping_drive_170025`), same map, same truth, same seed; `map_frequency` raised
from the `localization.launch.py` default of 10.0 to 30.0 so the output matches
`odometry/local`.

Steady state, first 35 s dropped (see the seeding defect below):

| | mean err | p95 | **step p95** | **step max** | inside 126 mm | yaw err |
|---|---|---|---|---|---|---|
| **AMCL direct (v1)** | **74.7 mm** | 143.9 mm | **15.7 mm** | **59.6 mm** | **85.2 %** | 1.96 deg |
| `ekf_map`, as configured | 125.0 mm | 356.8 mm | 83.5 mm | 920.1 mm | 66.9 % | 3.87 deg |
| `ekf_map`, `--no-vslam` | 152.9 mm | 406.0 mm | 72.1 mm | 503.2 mm | 52.2 % | 6.73 deg |
| `ekf_map` + the `vyaw` fix below | 85.2 mm | 182.8 mm | 37.3 mm | 288.5 mm | 79.9 % | 2.70 deg |

**`ekf_map` is less smooth than the thing it was supposed to smooth** — 5.3x the
correction-step p95 as configured, still 2.4x after fixing a real defect in it.
It is also less accurate. Whole-run numbers are worse again (222.3 mm as
configured, 191.6 mm fixed) because of the startup transient.

So **do not route the MPC's global pose through `ekf_map` expecting smoothness.**
The remaining lever for the jerkiness complaint is the offline smoothing already
proposed for LUCIO in §4, which needs no filter at all. The MPC, consuming live,
has no fix from this direction today.

### Two real defects found on the way

**bug-111 — `ekf_map` was never seeded, and nothing said so. FIXED.**
`seed_initialpose.py` seeds `nav2_amcl` through `/initialpose`;
`robot_localization` does not listen there (it seeds via its own `set_pose`). So
with `map_tf_publisher:=ekf` the filter started from a zero state:
`first map pose x=+0.000 y=+0.001 yaw=+0.05 deg` against a true start of
`(+0.445, -0.575, -79.82 deg)`. It spent **~30 s slewing in from the origin at
~740 mm / 82 deg of error** while publishing a perfectly steady 30 Hz
`map->odom`. Same silent-failure shape as the RTABMap localizer in §1. Live, that
was the first half-minute of every run — RViz's "2D Pose Estimate" also publishes
`/initialpose` and nothing else.

**The fix is a remap, not a bridge node.** `set_pose` is
`geometry_msgs/PoseWithCovarianceStamped` — the same type as `/initialpose` — and
robot_localization's own header describes it as *"usually published from rviz"*.
The library always intended the RViz pose tool to seed it; only the topic name
differed. `ekf_map.launch.py` now remaps `set_pose` -> `initialpose` under a new
`seed_from_initialpose` argument (default `True`), so one operator pose estimate
seeds the global localizer and the filter together, offline and live alike.
Deliberately **not** applied to `ekf_odom`, whose `world_frame` is `odom` — a
map-frame pose would be wrong there. It is opt-out because `set_pose` *resets*
the filter, which is right for an on-demand relocalization and wrong for anything
publishing `initialpose` periodically.

Verified on the control: `first map pose x=+0.446 y=-0.576 yaw=-79.78 deg`
against the seed `(+0.445, -0.575, -79.82)` — **1 mm and 0.04 deg**. The transient
is gone, and full-run and `--skip 35` numbers now agree instead of differing by
100 mm.

| | full-run mean | steady mean | step p95 | step max | bar (steady) |
|---|---|---|---|---|---|
| **AMCL direct** | **64.7 mm** | **74.7 mm** | **15.7 mm** | **59.6 mm** | **85.2 %** |
| ekf as configured | 222.3 mm | 125.0 mm | 83.5 mm | 920.1 mm | 66.9 % |
| ekf + `vyaw` | 191.6 mm | 85.2 mm | 37.3 mm | 288.5 mm | 79.9 % |
| **ekf + `vyaw` + seed** | **69.6 mm** | 84.3 mm | 38.7 mm | 280.2 mm | 80.5 % |

**Accuracy is now near parity; smoothness is still 2.5x worse**, and smoothness
was the entire reason for considering `ekf_map`. So the recommendation does not
move: AMCL stays the map-frame broadcaster. What remains untested is the
structural cause below.

`check_map_frame.py` gained **`--skip SEC`** so a seeded localizer and an
unseeded one can be compared on tracking quality instead of on startup. Report
both numbers; the transient is real.

**bug-112 — the map EKF had no angular-rate input at all.** `odom0`
(`vehicle/vesc_odom`) had `odom0_config` index 11 (`vyaw`) set `false`, and IMUs
are excluded from this filter by design. Its only heading information was
therefore AMCL's absolute yaw at 8.5 Hz, so it could not propagate heading
between scans — it lagged through turns and snapped on each update, producing
precisely the staircase it existed to remove.

Confirmed by subtraction: `51_localize_offline.sh --no-vslam` removes the VSLAM
inputs, and with neither `vyaw` nor `odom2`'s differential increments the
steady-state yaw error **triples** (3.87 -> 6.73 deg) and translation error goes
125 -> 153 mm. VSLAM's differential odometry had been silently standing in for
the missing yaw rate. **Fixed** (`vyaw: true`), worth 125.0 -> 85.2 mm and step
p95 83.5 -> 37.3 mm — a large improvement that still does not reach AMCL direct.

Neither defect touches v1: it was produced with `map_tf_publisher:=amcl`, which
uses none of this. **`deliverables/20260805/` is unchanged and still stands.**

### The structural reason it still loses — and the one untested lever

`ekf_map` does not estimate `map->odom`. It estimates `map->base_link`, and
`map->odom` is published as `(map->base_link) o (odom->base_link)^-1`. That second
term comes from **`ekf_odom`** — a different filter with a much better motion
model (2 IMUs + rf2o + VESC at 30 Hz). `ekf_map` meanwhile dead-reckons on
`vehicle/vesc_odom` velocities alone, since IMUs are excluded from it by design.
**Every disagreement between the two motion models lands in `map->odom` at 30 Hz
even when the localizer has said nothing new** — and VESC odometry is the
known-bad one (1.6-1.7 m divergence, ~25 deg of yaw drift per 150 s run).

AMCL has no such problem: it looks up the *actual* `odom->base_link` from TF and
differences its own pose against it, so its correction is consistent by
construction with the transform the consumer composes.

**Untested lever:** change `odom0` from `vehicle/vesc_odom` (velocities) to
`odometry/local` (the odom EKF's own output) fused differentially, making the map
EKF's prediction identical to the transform it is differenced against. If that
closes most of the remaining gap to AMCL's 15.7 mm step p95, `ekf_map` becomes a
real option for the MPC; if not, the ZOH answer stands and this path should be
left off the map-frame route for good. **Tested — it closes it. See §6.** Only
after that are `pose0_rejection_threshold` (2.0 Mahalanobis — rejections leave it
coasting on dead reckoning, then jumping) and `sensor_timeout` (0.13 s against an
0.118 s `amcl_pose` interval, so one dropped scan trips it) worth touching.

Runs kept for comparison: `bags/20260805/loc_ekf30_*` (as configured),
`loc_ekf30nv_*` (no VSLAM), `loc_ekfvyaw_*` (vyaw only),
`loc_ekfseed_*` (vyaw + seeding, current HEAD).

### Note for the `particle_filter` work

`localizer_pf.yaml` has its own coarse-to-fine global initialization, triggered by
`/initialpose` or automatically after `global_loc_timeout` (5 s). The remap above
covers the first case — a PF seeded from `/initialpose` seeds `ekf_map` at the
same moment. It does **not** cover the second: if the PF self-localizes with
nobody publishing `/initialpose`, `ekf_map` is unseeded again and bug-111 returns
in full. Forwarding a localizer's first converged fix to `set_pose` needs a
convergence criterion and a latch (`set_pose` resets the filter, so it must fire
exactly once) — that is real logic and would be the first actual node in this
otherwise pure-launch package. Do not write it speculatively; wait for the PF
measurements and see whether the auto-init path is even used.

**Do NOT ask the PF to publish its own estimate to `/initialpose` or `set_pose`.**
It looks like it would remove the need for that node. It would not:

- **`/initialpose` is an input the PF itself consumes** — it is what triggers the
  coarse-to-fine global localization (see `localizer_pf.yaml`), so publishing its
  own result there re-triggers its own relocalization. `nav2_amcl` subscribes as
  well, so it would also be commanding AMCL to jump. And it gives one topic two
  opposite meanings: "something external asserts the robot is here" (a command)
  versus "the localizer estimates the robot is here". That is the same defect
  this package already fixed on `cmd_vel`, where the executed-command echo had to
  move to `vehicle/cmd_vel_executed`.
- **`set_pose` RESETS the filter.** Published on every relocalization it would
  make `ekf_map` a pass-through with repeated resets — discarding covariance and
  history each time and spiking `map->odom`. Strictly worse than not seeding. It
  also would not reach `ekf_map` at all while `seed_from_initialpose` is `True`,
  since the filter is then listening on `initialpose`.

**The contract to ask the PF for instead:** publish the pose on its own normal
pose topic with an honest covariance — large while searching, small once
converged. Then point `ekf_map`'s `pose0` at that topic here. That is the
ordinary measurement path: continuous, covariance-weighted, no resets, no new
node. `ekf_map.yaml` already anticipates it (`pose0: amcl_pose  # replace with pf
topic when that is working`).

If the PF instead becomes the map-frame broadcaster (`map_tf_publisher:='pf'`),
`ekf_map` leaves the delivery path entirely and this question is moot for the
deliverable.

---

## 6. `odom0` = `odometry/local`, fused differentially — measured 2026-08-06, and this is the fix

§5's structural argument was right and its lever works. `ekf_map` was
dead-reckoning on `vehicle/vesc_odom` velocities while the `odom->base_link` its
correction gets differenced against came from `ekf_odom` — a different filter,
much better informed (2 IMUs + rf2o + VESC at 30 Hz). The two motion models
disagreed continuously, and *that* disagreement, not the localizer, was what
moved `map->odom` at 30 Hz.

Fusing `odometry/local` differentially makes the prediction the same increments
as the transform it is differenced against — the consistency AMCL gets for free
by looking `odom->base_link` up from TF.

Same control (`mapping_drive_170025`), same map, same truth, same seed,
`map_frequency` 30.0. Steady state, first 35 s dropped:

| | mean err | **step p95** | **step max** | inside 126 mm | yaw err |
|---|---|---|---|---|---|
| **AMCL direct (v1) — the target** | 74.7 mm | **15.7 mm** | **59.6 mm** | 85.2 % | 1.96 deg |
| `ekf_map`, as configured | 125.0 mm | 83.5 mm | 920.1 mm | 66.9 % | 3.87 deg |
| `ekf_map` + `vyaw` | 85.2 mm | 37.3 mm | 288.5 mm | 79.9 % | 2.70 deg |
| `ekf_map` + `vyaw` + seeding | 84.3 mm | 38.7 mm | 280.2 mm | 80.5 % | — |
| **+ `odom0` = `odometry/local` (this change)** | **74.6 mm** | **18.2 mm** | **64.0 mm** | **87.7 %** | 2.12 deg |

**The smoothness gap is gone.** Correction step p95 38.7 -> **18.2 mm**, against
AMCL's 15.7 mm — a 2.1x improvement, and within 2.5 mm of the target. Step max
280.2 -> **64.0 mm** against AMCL's 59.6 mm. Mean error lands on 74.6 mm, which is
AMCL's 74.7 mm to within a tenth of a millimetre, and it puts **more** samples
inside LUCIO's 126 mm bar than AMCL does (87.7 % vs 85.2 %).

Whole-run, unskipped — the seed makes this the honest number now:

| | mean err | step p95 | step max | inside 126 mm |
|---|---|---|---|---|
| AMCL direct (v1) | 64.7 mm | 14.6 mm | 59.6 mm | 87.3 % |
| **`ekf_map`, this change** | **62.3 mm** | **16.2 mm** | 235.6 mm | **90.3 %** |

**The seed took**, and this is how to tell: `first map pose` is
`(+0.446, -0.576, -79.67 deg)` against the seed `(+0.445, -0.575, -79.82)` — 1 mm
and 0.15 deg. The unskipped mean being *lower* than the skipped one (62.3 vs
74.6 mm) is not a gap of the bug-111 kind; it is the seed being nearly exact, so
the first 15 s window scores 3.8 mm and pulls the whole-run mean down. A failed
seed shows up as the opposite — a much *worse* unskipped number, which is what
191.6 vs 85.2 mm looked like before the fix.

**One residual, not chased:** whole-run step max is 235.6 mm against the skipped
run's 64.0 mm, so a single ~236 mm correction happens inside the first 35 s and
nowhere else. That is almost certainly AMCL's first real correction away from the
seed. It is a startup transient on a filter that is now smooth in tracking; worth
a look before anyone differentiates the first half-minute of a run for velocity,
not worth blocking on.

### What this changes

`ekf_map` is now a real option for the MPC's global pose: it matches AMCL's
smoothness, slightly beats its accuracy, and unlike AMCL it emits at a steady
30 Hz instead of the localizer's 8.5 Hz scan rate. **It is still not the default
broadcaster** — `map_tf_publisher` defaults to `amcl` and that is unchanged here,
because this is one bag replay and the live behaviour is unmeasured. Switching
the default is a decision for after the live test.

**v1 is untouched.** It never used this path, and nothing here regenerates it.
This *would* clear the §"HARD CONSTRAINT" bar for a `v2` on the control bag
(62.3 mm vs 64.7 mm, 90.3 % vs 87.3 %) — but by ~2 mm, which is not worth moving
the ground under three consumers for. If a v2 happens it should be for the
offline smoothing in §4, which addresses the actual complaint.

**Not touched, deliberately:** `pose0_rejection_threshold` and `sensor_timeout`,
per §5's ordering. With the motion model fixed they may no longer be worth
touching at all — a filter that is no longer fighting its own prediction rejects
fewer poses.

Run kept for comparison: `bags/20260805/loc_ekflocal_mapping_drive_170025`.

### Live caveat: this input now comes from the other EKF

`odom0` is `odometry/local`, which is `ekf_odom`'s output. Any configuration that
runs `ekf_map` **without** `ekf_odom` now leaves the map filter with no motion
model at all, where before it would still have had `vehicle/vesc_odom`. That is
the correct coupling — `map->odom` is only meaningful against an `odom` frame
somebody is publishing — but it is a sharper failure than it was. Offline this is
fine and is what was measured: `51_localize_offline.sh` runs with
`launch_ekf_odom:=False` and `odom_tf_publisher:=bag`, and `odometry/local` is
replayed from the bag alongside the TF it was recorded with.

---

## What changed in the repo

| file | change |
|---|---|
| `config/localization/ekf_map.yaml` | `odom0_config` `vyaw` false -> **true** (bug-112) |
| `config/localization/ekf_map.yaml` | `odom0` `vehicle/vesc_odom` (velocities) -> **`odometry/local` fused differentially** (bug-114) — §6, the change that closed the smoothness gap |
| `launch/localization/ekf_map.launch.py` | new `seed_from_initialpose` arg (default True): remaps `set_pose` -> `initialpose` so one pose estimate seeds the localizer and the filter together (bug-111) |
| `scripts/analysis/check_map_frame.py` | new `--skip SEC`, to separate startup transient from tracking |
| `scripts/live_runs/51_localize_offline.sh` | new `--map-frequency` (the launch default 10.0 is below the 30 Hz it smooths) and `--no-vslam` |
| `launch/mapping/3d_mapping.launch.py` | split the concatenated `/tf` + `/tf_static` `SetRemap` (bug-107) |
| `config/localization/ekf_map.yaml` | `pose0_relative` true -> false; `odom1` documented as launch-switched |
| `launch/localization/ekf_map.launch.py` | new `fuse_vslam_global` arg; `odom1` overridden from it |
| `launch/localization/dual_ekf.launch.py` | passes `fuse_vslam_global` through |
| `launch/localization/localization.launch.py` | computes it from `use_gpu` and `localize_on_startup` |
| `scripts/live_runs/52_localize_rtabmap_offline.sh` | new: offline RTABMap-localizer pass with CPU/RSS sampling |

Bugs logged: **bug-107** (SetRemap concatenation), **bug-108** (`nf.db` has no
visual vocabulary). **bug-104** updated with the fix.
