# Handoff — `ekf_map` startup race (**fix written, untested on hardware**), archived-bag seeds (closed), IMU bias (not started)

**Rewritten:** 2026-08-13 ~13:40 EDT · **Branch:** `perf/config-tuning` · **Robot:** gosling1
Supersedes the 2026-08-12 revision; §0 of the 08-11 revision (the −84.5° heading result) is still
current and is not repeated here — see `DEMO_RUNBOOK_20260810.md` §5b.

**What changed today (offline, no robot time used):**

- §1 — **fix written and wired** into `localization.launch.py` + `bringup.launch.py` (`bug-241`).
  Untested on hardware; the whole remaining job is the acceptance test in §1a.
- §2 — **CLOSED as an invalid measurement** (`bug-240`). The operator confirmed the car was not
  verified to be on its marks, which explains the 0.61 fit score. Do not chase the 14° spread.

---

## 1 · `ekf_map` startup race — **mechanism identified; fix written 2026-08-13, NOT yet verified**

### The measurement that settles it

Taken live 2026-08-12 ~16:56, stack up per `DEMO_RUNBOOK_20260810.md` §4, no seed:

```
ros2 topic info /amcl_pose --verbose

  PUBLISHER   amcl            RELIABLE     TRANSIENT_LOCAL   KEEP_LAST(1)
  SUBSCRIPTION ekf_map_node   BEST_EFFORT  VOLATILE          KEEP_LAST(2)
```

**The two are QoS-compatible, so the connection forms and `Subscription count` reads 1** — which is
why nothing in any health check ever flagged this. But `VOLATILE` means the subscriber receives
**nothing published before it subscribed**. AMCL publishes `amcl_pose` `TRANSIENT_LOCAL` precisely so
a late joiner still gets the one-shot initial pose; `robot_localization` declines the latch.

Combine that with motion gating (`update_min_d` / `update_min_a`): parked, AMCL publishes its initial
pose essentially once and then goes quiet. Whether `ekf_map` subscribes before or after that single
publish is a race with no second chance. That is the whole defect.

Cold-launch record so far, all with no seed:

| date | `map→odom` | outcome |
|---|---|---|
| 2026-08-10 | `(0.463, -0.602, -79.31°)` composed to base_link | took AMCL |
| 2026-08-11 | identity | **missed it** |
| 2026-08-12 | `(0.486, -0.598, -75.38°)` | took AMCL |

2 : 1. Consistent with a race, and it confirms the standing rule that **one clean launch proves
nothing**.

### What this changes about the candidate fixes

The 2026-08-11 revision listed three. One is now dead:

- ~~"Latch / transient-local QoS on `amcl_pose`, if upstream allows it."~~ **No-op — the publisher
  already latches.** The volatile side is the *subscriber*, and `robot_localization` does not expose
  its subscription QoS as a parameter. Confirm that against the installed source before designing
  around it; if a newer `robot_localization` does expose it, that is the cheapest fix by far.
- **A first-fix forwarder** that republishes the localizer's pose onto `initialpose` once
  `ekf_map` is subscribed. Still the front-runner. `bug-111`'s RESIDUAL note already anticipated
  needing exactly this for the `particle_filter` auto-global-init case — same gap, different trigger,
  so one node closes both.
- **Have AMCL republish its initial pose a few times at startup.** Upstream change; no local knob.

Acceptance is unchanged: **repeated cold launches with no seed.** Given 2:1 odds of passing by luck,
budget at least five or six.

### 1a · The fix as written (2026-08-13, `bug-241`) — and how to test it

Neither candidate above, in the end. The forwarder was a new node in a package that deliberately has
none, and it would have had to *observe* a pose that may never be published. What is there instead is
simpler: **publish the seed from the launch file itself**, because the number is already on disk.

`launch/localization/localization.launch.py`:

- `read_initial_pose(params_file)` reads the `initial_pose` block back out of the AMCL params file.
  The constant therefore still lives in exactly one place, `localizer_amcl.yaml` — this is not a
  second copy to keep in step with the grid and the cloud.
- A `TimerAction` (`initialpose_seed_delay`, default **20.0 s** after localization starts, so ≈30 s
  into a bringup) runs `ros2 topic pub --times 5 --rate 1 /initialpose
  geometry_msgs/msg/PoseWithCovarianceStamped …`.
- `initialpose` is where `ekf_map`'s `set_pose` is already remapped (`seed_from_initialpose`,
  `bug-111`) **and** what AMCL listens on, so one publish seeds both from the same number.
- `header.stamp` is `{sec: 0, nanosec: 0}` — "latest available" to tf2, which therefore cannot
  extrapolate into the future. This is the same trick `seed_initialpose.py` documents.
- Five publishes, not one: a single publish can still land before subscription matching completes,
  and a dropped seed is invisible.
- `condition=UnlessCondition(use_sim_time)`: replays seed themselves with replay-specific values
  (`51_localize_offline.sh`, `61_nav2_offline.sh`), and a second seed carrying the parking-spot pose
  would fight them.
- Gated on `launch_ekf_map` as well as `seed_initialpose`. If the params file has no `initial_pose`
  block it logs and does nothing — it does not crash the launch. Verified offline against
  `localizer_amcl.yaml` (parses −84.5°, round-trips as valid YAML) and `nav2_params.yaml` (→ `None`).

New args, defaulting `True` / `20.0`, declared and passed explicitly at `bringup.launch.py` per the
launch-config inheritance rule: **`seed_initialpose`**, **`initialpose_seed_delay`**.

**The acceptance test.** Cold launch per §5, **with no manual `seed_initialpose.py` run**, then:

```bash
ros2 run tf2_ros tf2_echo map base_link      # must NOT be identity
ros2 topic echo /amcl_pose --once
```

Expect the log line `[localization] seeding /initialpose with (+0.445, -0.575, -84.50 deg)` at ≈30 s,
then `map→base_link` near `(0.445, -0.575, -84.5°)`. **Five or six cold launches**, all passing — at
2:1 prior odds, one pass proves nothing. If a launch still comes up identity, run the §1b test on it
before assuming the delay is wrong.

Two things to watch that would mean tuning rather than failure: the seed arriving *before* AMCL and
`ekf_map` finish coming up (raise `initialpose_seed_delay`), and the five resets visibly disturbing
anything (they should not — the car is parked and this is startup).

### 1b · Not yet run — the discriminating test

The 08-11 revision's push-the-car test (does a *late* `amcl_pose` get taken?) was never run: the
race did not reproduce on the 08-12 launch, so there was no identity state to test against, and the
card faulted before a relaunch. **It is now largely redundant** — the QoS mismatch predicts its
outcome (a late pose *is* taken, because by then both ends are connected) — but it is still worth one
run the next time a launch does come up at identity, as confirmation rather than diagnosis.

---

## 2 · Live pose disagreed with the spot — **CLOSED 2026-08-13: invalid measurement** (`bug-240`)

**The operator confirmed the car was not verified to be on its marks for that launch.** That is the
answer the section below was waiting on, and it resolves it: the 0.61 fit score was the tell, and it
was telling the truth. A uniformly low score is not "right place, wrong heading" — it means a large
fraction of beams never landed on mapped structure. **There is no localization defect here to chase,
and the 14° spread is not evidence of one.**

Two things to carry forward rather than re-derive:

- **`heading_from_scan.py` scores are a validity gate, not decoration.** Archived bags and the
  2026-08-11 samples score 0.89–0.92. Treat anything below ~0.85 as a failed *measurement* and fix
  the physical setup; do not read its heading and reason about the number.
- Any future live heading check starts by confirming the car is on its marks and the room is clear of
  unmapped clutter — a lab day on AC power puts cables, benches and people into the scan.

The original section is kept below for the record.

### (superseded) Live pose disagreed with the spot

On the 08-12 launch, parked:

```
amcl_pose                     yaw -70.7 deg
map->base_link                (0.540, -0.592, -72.5 deg)
odom->base_link               (0.008,  0.087, +3.4 deg)
heading_from_scan.py --live   BEST (0.365, -0.515, -79.75 deg)   score 0.61
config seed                   (0.445, -0.575, -84.5 deg)
```

Three different answers spanning ~14°. **Do not chase this as a localization bug until the physical
question is answered**, because the scan fit — the one measurement with no estimator in it — is
itself untrustworthy here: its score is **0.61**, against **0.89–0.92** for the archived bags and for
yesterday's five samples. A low score everywhere is not the signature of a car at the right place
with the wrong heading; it means a large fraction of beams do not land on mapped structure at all.

Most likely causes, in order: the car was not on its marks; or the room had unmapped clutter (this
was a lab day on AC power — cable, bench, people all read as unmapped returns). **The operator was
asked whether the car was on the spot and had to leave before answering. Ask again first thing.**

Only if the car *was* on its marks and the room was clear does this become a real finding — and in
that case the thing it would indicate is that `ekf_map` took AMCL but AMCL itself had converged 14°
off the seed, which is a different defect from §1.

---

## 3 · Archived bags' start pose — **CLOSED. The ground truth carries the same ~4.4° error.**

Ran `heading_from_scan.py` against the first scans of every 2026-08-05 bag that contains a
`LaserScan`, scored against the map those same bags built
(`/mnt/shared_dir/maps/20260805/rtabmap_2d_final.yaml`), with `--xy-range 0.15 --xy-step 0.03`:

| bag | best fit | vs −84.50 (config today) | vs −79.82 (archived truth) |
|---|---|---|---|
| `mapping_drive_170025` | **−84.25°**, (0.415, −0.515) | 99.8 % of best, 0.25° | 82.9 %, **4.45°** |
| `loop_laps_173558` | −83.50°, (0.445, −0.485) | 97.5 %, 1.00° | 86.0 %, 3.70° |
| `figure8_172338` | −82.25°, (0.415, −0.515) | 91.6 %, 2.25° | 93.5 %, 2.45° |

`mapping_drive_170025` is decisive: it is the bag the RTABMap database was built from and the bag
`truth_mapping_drive_170025.csv` describes. Its own first scan, scored against its own derived map,
lands **0.25° from −84.5° and 4.45° from the −79.82° that five files carry as its start pose.**

**Conclusion: the bags did not start 4.7° from where the car parks today. The RTABMap optimized-pose
ground truth shares the bias the AMCL seed had** — expected, since both were read from the same
source.

Two checks that make this a measurement rather than an impression:

- **Not motion smear.** Re-running at 3, 8 and 20 scans gives bit-identical results per bag
  (`mapping_drive_170025` = −84.25° at all three), so the car was genuinely parked through them.
- **The ~2° spread across bags is real re-placement variation** between separate hand-placements, not
  noise. `figure8_172338` is the outlier and is also the only one where the two candidates score
  within 2 points of each other, so it discriminates weakly on its own — if the seeds are ever
  updated, consider giving that bag its own value rather than the shared one.

**Nothing was changed.** The five files still carry `(+0.445, −0.575, −79.82°)`:
`51_localize_offline.sh`, `scripts/analysis/check_map_frame.py`, `MAP_BUILD_HANDOFF.md`,
`BRIEF_PARTICLE_FILTER.md`, `LOCALIZER_FOLLOWUPS.md`. The measurement to justify updating them now
exists; the edit is deliberate and was left for the operator, because a careless one desynchronizes
each replay from its own data.

---

## 4 · gosling1 SD card — **event 4, 2026-08-12 16:58 EDT** (`bug-239`)

```
16:58:41  mmc0: Tuning failed, falling back to fixed sampling clock   (x11)
16:58:41  blk_update_request: I/O error, dev mmcblk0, sector 24299552 (WRITE)
16:58:41  JBD2: Error -5 detected when updating journal superblock for mmcblk0p1-8
16:58:42  EXT4-fs (mmcblk0p1): I/O error while writing superblock
16:58:42  EXT4-fs (mmcblk0p1): Remounting filesystem read-only
16:59:11  blk_update_request: I/O error, dev mmcblk0, sector 9254148  (READ)
```

1 h 50 min into the boot, at 87 % full — same signature as the three events of 2026-08-09, and
capacity is again not the cause. `docker exec` dies with
`OCI runtime exec failed: open /tmp/runc-process…: read-only file system`.

**The running container and ROS stack survive it** — Docker's data-root and `ROS_LOG_DIR` are both on
the NVMe, so only *new* `docker exec` calls fail; already-open shells keep working. That is the
salvage path if it happens mid-session again.

**Rescued this session:** `/etc/udev/rules.d/*` → `/mnt/f1tenth_ssd/shared_dir/sdcard_rescue/udev_20260812/`,
plus `daemon.json` and `f1tenth_launch.sh`. The udev rules were the known-missing backup — without
`99-vesc.rules` and `ydlidar.rules` a fresh card has no `/dev/sensors/vesc` and no `/dev/ydlidar`.
`chrony.conf` did not copy; `ydlidar-V2.rules.disabled` came across as 0 bytes (it is `.disabled`,
so nothing depends on it).

**Do not plan a long session on this card.** The 08-09 pattern was faults at 28 min, 70 min and
mid-bringup across three boots. Replace it, or move the rootfs to the NVMe (916 GB at 33 %).

---

## 5 · Restarting tomorrow — the container reverts, and it reverts to something wrong

`/workspaces` is a container layer, so a fresh container starts from the image's `f1tenth_launch`.
**Verified 2026-08-12: the image is wrong in three ways at once** — it carries the old AMCL yaw
`-1.3928`, has no `data/maps/20260805/` (only the deleted `raslab` map, so `pcd_to_pointcloud` aborts
on its missing default PCD), and has none of the `scripts/live_runs` or `scripts/analysis` tools.

That is one command, run on the host or inside the container after `prep_container.sh`:

```bash
bash /mnt/shared_dir/stage_0825.sh          # inside the container
/mnt/f1tenth_ssd/shared_dir/stage_0825.sh   # on the host; auto-detects the container
```

> **`f1tenth_stage_20260813.tgz` was cut but never delivered.** It existed only in the Windows-side
> handoff dir, never on the robot's SSD — so a session following that text stages nothing and
> silently runs the image's own package. The lesson is that a tarball named in a handoff has two
> separate states, *built* and *on the robot*, and only the second one matters. Both older tarballs on the SSD are also unusable:
> `20260812` was cut from `4ff8d4e`, *before* the §1a `initialpose` seed landed, and `20260810` is
> older still. Staging either un-tars the old `localization.launch.py` over the container and the
> acceptance test then measures the unfixed code — the same class of trap as the image carrying the
> old AMCL yaw. **Verify the tarball is on the SSD before trusting any staging instruction here.**

`f1tenth_stage_20260825.tgz` (cut 2026-08-25 from `4966f6c` plus the then-uncommitted
`scripts/`+`config/` working tree, md5 `508e7706db2eb26d0cb4b9b8d6e0a587`, 462 KB, 182 entries)
un-tars over the package, copies the three `20260805` map files in from
`/mnt/shared_dir/maps/20260805/`, rebuilds with `--symlink-install`, and then **verifies in the
installed tree** — the yaw, the seed count, the three maps, the two tools. Validated end-to-end in a
throwaway container 2026-08-25 (exit 0; `yaw: -1.4748`, 14 `seed_initialpose` hits, all three maps).
The rebuild is not optional: `--symlink-install` links only files that existed at build time, so
anything *added* by staging is absent from `install/` until you rebuild.

Post-staging spot check on the robot, if you want it independent of the script:

```bash
grep -c seed_initialpose /workspaces/f1tenth/install/f1tenth_launch/share/f1tenth_launch/launch/localization/localization.launch.py   # expect 2+
```

Then the §4 launch block from `DEMO_RUNBOOK_20260810.md`, with today's log dir on the SSD:

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///mnt/shared_dir/cyclonedds_offline_lo.xml
export ROS_LOG_DIR=/mnt/shared_dir/claude_ekfmap_0812/roslogs
ros2 launch f1tenth_launch bringup.launch.py \
  launch_visualization:=True launch_twist_to_ackermann:=True \
  max_steering:=0.314 localize_isaac_vslam_on_startup:=False \
  map_file:=/mnt/shared_dir/maps/20260805/rtabmap_2d_final.yaml
```

**The operator launches the stack, not the agent** — an agent-initiated bringup over ssh kills the
RealSense via the GLFW/X11 context.

---

## 6a · Spare cars are an option — but `vesc.yaml` is gosling1's

Raised by the operator 2026-08-13: there are other F1/10 Jetsons available, not kept up to date.
Bringing one up is mostly pulling the current Docker image and re-applying the udev rules (rescued to
`/mnt/f1tenth_ssd/shared_dir/sdcard_rescue/udev_20260812/`) and `chrony.conf` — that part is cheap,
and it sidesteps the §4 card.

**What does not transfer is the VESC calibration.** `config/vehicle/vesc.yaml`'s measured values are
gosling1's: `steering_angle_to_servo_gain −1.1448` and `..._offset 0.56` were measured on that car
2026-08-07, and the asymmetric servo travel (+24.0° left / −18.0° right) is that car's servo horn.
On another chassis those are inherited numbers again, with the same ~18–23 % over-steer risk the
sysid found. So:

- **The §1a acceptance test transfers fine** — it is localization-only, parked, and touches nothing
  the VESC calibration affects. A spare car is a legitimate way to run it off the failing card.
- **Anything that drives — the moving-odometry check (§6), MPC, Nav2 — does not transfer** without
  re-running the steering sysid on that car. Method: `SYSID_RESULTS.md`.
- The AMCL seed is a property of the *parking spot and map*, not the car, so it carries over — but
  only if the spare is parked on the same marks.

## 6 · Not started

**IMU bias remover.** Unchanged from the 08-11 revision — `launch/sensors/realsense_d435i.launch.py:366`
hardcodes `'remove_imu_bias': 'False'` and that one string is the whole blocker; the chain is wired
and `imu_processors` is present in the image (`prep_container.sh` confirms it every run). Needs a
deliberate before/after with `yaw_drift.py` (60 s, parked), not a flip, because parked the measured
bias (−0.002208 rad/s) drives nothing while rf2o and VSLAM hold fused yaw to −0.05 °/min. Leave
`vehicle.launch.py:377` at `False` — the two effects on `odometry/local` are not separable after the
fact. Internet on the Jetson was confirmed working this session (`apt` reachable), so the stated
blocker is gone.

**Moving-odometry check.** `odom_moving_check.py` and `analysis/check_map_frame.py` are now carried
by `f1tenth_stage_20260812.tgz`, so the staging gap that blocked this is closed. The procedure is
`DEMO_RUNBOOK_20260810.md` §3; it needs battery power and a driven leg.

**LUCIO notification.** `scripts/live_runs/LUCIO_MAP_HEADING_NOTICE.md`, written 2026-08-11, still
not sent. **The operator sends it, not the agent.** §3 above strengthens it: the archived RTABMap
ground truth is now independently shown to carry the same error, which is a second, unprompted
confirmation of the point the notice makes about waypoint 0.
