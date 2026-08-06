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

## What changed in the repo

| file | change |
|---|---|
| `launch/mapping/3d_mapping.launch.py` | split the concatenated `/tf` + `/tf_static` `SetRemap` (bug-107) |
| `config/localization/ekf_map.yaml` | `pose0_relative` true -> false; `odom1` documented as launch-switched |
| `launch/localization/ekf_map.launch.py` | new `fuse_vslam_global` arg; `odom1` overridden from it |
| `launch/localization/dual_ekf.launch.py` | passes `fuse_vslam_global` through |
| `launch/localization/localization.launch.py` | computes it from `use_gpu` and `localize_on_startup` |
| `scripts/live_runs/52_localize_rtabmap_offline.sh` | new: offline RTABMap-localizer pass with CPU/RSS sampling |

Bugs logged: **bug-107** (SetRemap concatenation), **bug-108** (`nf.db` has no
visual vocabulary). **bug-104** updated with the fix.
