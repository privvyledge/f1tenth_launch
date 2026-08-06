# Offline map build — handoff

**Written 2026-08-05 ~19:55 EDT.** Everything below was measured by rebuilding
maps from `mapping_drive_170025` on `gosling1`, not inferred from launch files.

The job was: build 2D + 3D maps offline from the three bags recorded that
afternoon. **Done for the mapping bag.** The two trajectory bags have not been
touched yet — that is the next session's work, and it is a *localization* job,
not a mapping job. See "What's next".

---

## What exists

`/mnt/f1tenth_ssd/shared_dir/maps/20260805/` on the host,
`/mnt/shared_dir/maps/20260805/` in a container. It has its own `README.md`
aimed at a downstream consumer; read that for field-by-field detail.

| artifact | what it is |
|---|---|
| `rtabmap_2d_final.pgm/.yaml` | 2D grid from RTABMap, 265x199 (13.3 x 10.0 m), 1732 occupied cells. **Use this one.** |
| `slamtoolbox_2d_final.pgm/.yaml` | 2D grid, 264x320, 2766 occupied cells — **rejected on inspection, not usable** (see below) |
| `slamtoolbox_2d_final_posegraph.posegraph/.data` | pose graph; only meaningful if the slam_toolbox map is ever salvaged |
| `rtabmap_2d_overfiltered.pgm/.yaml` | the old over-filtered grid; kept only as a before/after. **Do not use.** |
| `rtabmap_final_nf.db` | RTABMap database, 399 poses / 813 links — the real 3D deliverable |
| `cloud_clean.pcd` / `.ply` | 285 288 pts, range-limited + noise-filtered. **Preferred cloud.** |
| `cloud_full.pcd` / `.ply` | 700 445 pts, unfiltered; heavy radial smear from D435i depth error |
| `*_preview.png` | rendered previews of each grid and the cloud |

The 2D RTABMap grid and both clouds all come from `rtabmap_final_nf.db`, so they
share one optimized pose graph and one `map` origin. `rtabmap_final.db` (118 MB)
is the earlier build with the old grid parameters; its 3D content is equivalent
but its cached 2D grids are the bad ones.

### Which 2D grid to use — the SLAM Toolbox one was rejected

**Use `rtabmap_2d_final.*` for costmaps and localization.** The operator checked
both as 2D costmaps on 2026-08-05 and found the SLAM Toolbox grid unusable.
Its preview shows why: smeared and doubled walls along the bottom edge, plus
speckle scattered through what should be open free space — pose-graph drift
rasterised into the map. Its higher occupied-cell count (2766 vs 1732) is partly
that noise, so **do not rank these two by cell count**; look at the previews.
The post-fix RTABMap grid is a clean, closed four-wall room.

Consequence for localization: **slam_toolbox localization mode is off the table**
for this map, since it needs that pose graph. Localize with AMCL
(`localizer_amcl.yaml`) or `particle_filter` (`localizer_pf.yaml`) against
`rtabmap_2d_final.yaml` instead. RTABMap's own localization mode against
`rtabmap_final_nf.db` is a third option and keeps the 2D and 3D maps in one
frame.

## What the drive actually covered — read this before planning

| bag | start | x extent | y extent | span | path |
|---|---|---|---|---|---|
| `mapping_drive_170025` | (0,0) | −2.38 … 1.73 | −3.52 … 1.76 | 4.1 x 5.3 m | 53.2 m |
| `loop_laps_173558` | (0,0) | −1.55 … 2.21 | −2.88 … 0.01 | 3.8 x 2.9 m | 37.6 m |
| `figure8_172338` | (0,0) | −2.12 … 1.85 | −3.67 … 0.06 | 4.0 x 3.7 m | 40.6 m |

All three drove a comparable, small area, so one map plausibly covers all three.
Every map here is a small-room map because the drive was — that is a property of
the bag, not a truncation. **Each bag's odom frame starts at identity**, which
means the origin is wherever that car happened to be parked at t=0 of that run.
The three origins are therefore *different physical poses*, and the offset
between them is unknown until something estimates it.

---

## What's next

The likely instinct — "build a new map that publishes `map→odom`" — is worth
correcting, because it sends you down the wrong path:

**A map never publishes `map→odom`. A localizer does.** The `.pgm`/`.yaml` and
the `.db` are static artifacts. `map→odom` is broadcast at runtime by AMCL,
slam_toolbox in localization mode, `particle_filter`, or RTABMap in localization
mode — whichever you select with `map_tf_publisher`. Nothing needs to be
re-mapped to obtain it.

So the actual next task is:

### 1. Localize the two trajectory bags against the existing map

For each of `loop_laps_173558` and `figure8_172338`: replay the bag, run a
localizer against **`rtabmap_2d_final.yaml`** (not the SLAM Toolbox grid — it was
rejected, see above), and let it produce `map→odom`.
The bags already carry `odom→base_link` at 30 Hz from the live EKF, so the
localizer only has to supply the missing edge — it does not need to re-derive
odometry. That single transform is what expresses each bag's trajectory in the
shared `map` frame, which *is* the consistency you want. `50_localization_test.sh`
is the existing entry point.

The one real difficulty: because each bag starts at its own identity origin,
each needs an **initial pose in the map frame**. Options, cheapest first:
seed `/initialpose` with a guess if the operator can recall roughly where the
car was parked; or let `particle_filter`'s coarse-to-fine global localization
(`global_loc_*` in `localizer_pf.yaml`) find it; or scan-match the bag's first
`lidar/scan_filtered` against the map. Verify by checking that the resulting
map-frame path stays inside the mapped free space and that the loop closes.

Note `mapping_drive_170025` is the easy case: its map was built from its own
odometry starting at identity, so `map` and `odom` coincide at t=0 for that bag.

### 2. Then, waypoints

Once all three trajectories are in the `map` frame, generating waypoints is a
pure post-processing step on the pose streams — no robot, no replay.

### 3. Optional — a genuinely fused map

Merging all three bags into one map *is* a real technique, but it is RTABMap
**multi-session mapping**, not "align the maps afterwards": append each bag as a
new session into one database (`--append` / `life_long_mapping:=True`) and let
RTABMap find inter-session loop closures, which simultaneously merges the map
and recovers the transforms between the three runs. Worth doing only if step 1
shows the current map's coverage is too thin where the other two bags drove.
Given the extents above, it probably is not.

### Not a factor: multiple `odom→base_link` publishers

This was raised as a possible cause of map quality problems. It does not apply
to `mapping_drive_170025`: `/gosling1/tf` carries **4403 messages containing
exactly 4403** `odom→base_link` transforms — 1:1, 30.0 Hz, matching
`/gosling1/odometry/local`'s 4403 messages exactly. Two publishers would show
~8800 transforms on that edge. There are 12 static transforms, all distinct
pairs. The bag has a single publisher. If another session observed duplicates it
was in the live stack or a different bag; do not go looking for it here.

---

## Four defects fixed while doing this (commit `4bbe9a9`)

All four failed **silently** — the build reported success and left either no map
or an unusable one. Logged as bugs 081–084.

1. **Wrong topic namespace** (`41_save_map.sh`, `topic_sets.sh`). RTABMap runs as
   `__ns:=/<ns> __node:=rtabmap`; ROS 2 resolves relative topic names against the
   *namespace*, not the node name, so the grid is on `/<ns>/grid_prob_map`. The
   script looked under `/<ns>/rtabmap/`, missed both branches, and still exited 0.
   `COMMANDs.md` always had it right.
2. **RTABMap's grid is not latched** (`41_save_map.sh`). Unlike slam_toolbox's
   `/map` (transient_local), it publishes only on map update, so a saver that
   attaches after the mapper finishes waits out its timeout and dies with
   `Failed to spin map subscription`. Perversely, the better you waited for the
   mapper to finish, the more reliably the save failed. Now the saver starts
   first and the `publish_map` service is called second — an un-latched publish
   before the subscriber exists is simply missed.
3. **Teardown could hang the run** (`40_build_map_offline.sh`). RTABMap defers
   SIGINT until it has done a final graph optimization and written the map to the
   `.db` — ~10 min, growing the database 94 MB → 118 MB. The old unbounded
   `wait` meant `--mode both` never started pass 2. Now: SIGINT, generous grace
   (`LAUNCH_SHUTDOWN_GRACE`, default 900 s so that flush completes), then
   escalate, then reap orphans. Orphaned nodes had previously left *two* mappers
   publishing into one namespace at once.
4. **Occupancy noise filtering tuned for a dense cloud** (`mapping.launch.py`).
   With `Grid/Sensor 0` and a YDLidar X4 (~625 pts/rev), point spacing grows with
   range (~0.02 m at 2 m, ~0.05 m at 5 m), so requiring 10 neighbours within
   0.15 m deleted every wall past ~3 m while ray tracing still carved the free
   space in front of it — a grid of free-space spokes with no room boundary.
   Relaxed to `2` / `0.10 m`: 670 → 1732 occupied cells and a closed four-wall
   room. **Local grids are cached per node in the `.db`**, so changing these does
   not re-grid an existing database; the bag must be replayed.

---

## Environment used, and how to get back into it

Offline map building needs **no hardware** — no camera, no USB, no VESC. It also
must not disturb a live stack or another agent session.

```bash
ssh gosling1
# container: created with the operator's script only — see the memory note on
# never hand-building it. It self-names jetson_container_<date>_<time>.
docker ps --format '{{.Names}}\t{{.Status}}'
docker exec -i <container> bash -s < some_script.sh
```

Inside, before anything:

```bash
export ROS_DOMAIN_ID=42          # NOT 0 — keeps clear of a live stack
export CYCLONEDDS_URI=file:///mnt/shared_dir/cyclonedds_offline_lo.xml
source /opt/ros/humble/setup.bash
source /workspaces/f1tenth/install/setup.bash
```

`cyclonedds_offline_lo.xml` was added for this: loopback interface only, single
`localhost` peer. An offline rebuild talks only to itself, so binding the WiFi
NIC only generated discovery traffic to peers that are not there (the failed-
syscall spam in `CYCLONEDDS_PEERS.md`).

Rebuild command:

```bash
cd /workspaces/f1tenth/src/f1tenth_launch/scripts/live_runs
./40_build_map_offline.sh --bag /mnt/shared_dir/bags/20260805/mapping_drive_170025 \
    --mode both --rate 0.2 --no-viz
```

### Gotchas that cost real time

- **`/workspaces/f1tenth` is not a bind mount.** The package inside the image was
  two commits stale. Jetson has no internet, so stage changes through the SSD:
  `git archive HEAD <paths> | gzip > x.tar.gz`, `scp` to
  `/mnt/f1tenth_ssd/shared_dir/handoff/`, then `tar xzf` inside the container.
  The workspace is `--symlink-install`, so edited launch/config files take effect
  with no rebuild.
- **`pkill -f` inside `docker exec bash -lc "..."` kills its own shell**, because
  the pattern matches the exec'd command line. Pipe a script on stdin instead:
  `docker exec -i <c> bash -s < script.sh`. This also dodges the `${...}` mangling
  that two levels of quoting cause.
- **`map_saver` writes trinary PGMs**: occupied 0, free 254, **unknown 205**.
  Classifying "≥200 is free" folds unknown into free and makes a sparse map look
  complete.
- **Open3D is broken in `humble-devel-08052026`** — `import open3d` fails with
  `No module named 'open3d.cpu'`, so the PLY→PCD recipe in `COMMANDs.md` does not
  run. The PLYs were converted with a small direct binary reader instead; note
  the file declares three elements (`vertex`, `face`, `camera`) and only
  `vertex`'s properties define the stride.
- **`rtabmap-export --output` is a base name** and always appends `_cloud.ply`,
  as `COMMANDs.md` already warns.
