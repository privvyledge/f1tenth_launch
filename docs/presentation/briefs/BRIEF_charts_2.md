# Brief: offline charts, continuation (Phase F1, session 2)

**Written 2026-09-03 by the first charts chat, which ran out of room.** Read
`BRIEF_charts.md` first — it is still the authority on scope, conventions and
traps. This file records what session 1 actually did, what it found, and what is
left. Where the two disagree, this one is newer.

Working directory `docs/presentation/`, branch `perf/config-tuning`.

> **SUPERSEDED IN PART by `BRIEF_charts_3.md` (session 2).** Two things below are
> now known wrong: §1's flag is `publish_realsense_pointcloud`, **not**
> `depthimage_to_pointcloud`; and §1's mcap-encoding hypothesis is **disproved** —
> bug-272 reproduces with no recorder at all. §3's data filenames also moved.
> §4a is **done**. Everything else here still stands.

---

## 0. State: nothing is committed, one asset has its data

No slide has been edited. No asset is `DONE`. The build is exactly as session 1
found it:

- `STRICT=1 ./build.sh full md` → 70 slides, passes
- `./check_overflow.mjs out/deck_full.md` → 1 overflow, slide 31, pre-existing
  and owned by the perception chat

Two files were added to `scripts/` and two data files to `assets/data/`. Details
in §3.

---

## 1. Do this first: the pointcloud stress re-run

The one experiment session 1 could not finish. It needs the operator and about
five minutes.

**Why.** Slide 2.7 claims recording the image streams starves visual odometry
(bug-272). Session 1 tested whether *consuming* the images is enough to cause it,
by subscribing externally to all five image streams at full rate for 60 s — about
110 MB/s leaving the container, against the recorder's measured 125 MB/s.
**Nothing degraded**: every publisher held 100 %, VSLAM included at 29.97 Hz.

That result is incomplete for one reason: **`camera/depth/color/points` was not
running**, and the original 2026-09-01 recording carried it. So session 1 stressed
five of the six heavy streams. See §2 for why the cloud was off.

**The run.** Ask the operator — do not launch it yourself, see §4 — for:

**`71_mpc_stack.sh` has no passthrough for extra launch arguments** — verified
2026-09-03, there is no `--launch-args`, `launch_args` or `EXTRA` anywhere in it.
So there are two options, and the first is better:

1. **Add a passthrough** (a few lines: collect trailing `k:=v` tokens, append them
   to the `ros2 launch` invocation at line ~76). Small, reusable, and it keeps the
   script's health checks and its `stop_launch_tree` teardown — which matters,
   because killing the launch PID directly orphans every node (bug-269).
2. **Have the operator edit the one line** — append
   `depthimage_to_pointcloud:=True` to the `ros2 launch f1tenth_launch
   bringup.launch.py \` block inside the script and revert afterwards.

Either way the operator runs it, with:

```bash
export ROS_LOG_DIR=/mnt/shared_dir/claude_mpc_0903a/roslogs
export RESET_REALSENSE=True
/workspaces/f1tenth/src/f1tenth_launch/scripts/live_runs/71_mpc_stack.sh \
  --map /mnt/shared_dir/maps/20260805/rtabmap_2d_final.yaml --domain 42 -y
```

Confirm the cloud is actually up before measuring — `ros2 topic list | grep
depth/color/points` must be non-empty, and `ros2 component list` should show more
than just the camera in `sensing_container`. Session 1's whole detour came from
assuming a stream was live because the launch file said it should be.

Then, from inside the container, with `ROS_DOMAIN_ID=42` and
`CYCLONEDDS_URI=file:///mnt/shared_dir/cyclonedds_velox1.xml` (§4):

```bash
python3 /mnt/shared_dir/work/presentation_charts/live_rates.py \
    --ns /gosling1 --duration 60 --json .../rates_stress_pc.json
python3 .../live_rates.py --ns /gosling1 --duration 60 --stress-images \
    --json .../rates_stress_pc_load.json
```

Add `camera/depth/color/points` to `DEFAULT_TOPICS` and to `CAMERA_INFO_PROXY`'s
stress list first — it has no `camera_info` twin, so it must be stressed and
metered directly, and that asymmetry needs a note on whatever chart uses it.

**What the answer means.** If the cloud's presence reproduces the starvation, 2.7
can name the mechanism. If it still does not, the remaining difference is the
recorder's own work — and note session 1 **eliminated slow storage**: there is no
SD card in gosling1 (no `mmcblk`; `/mnt/f1tenth_ssd` is a directory on the NVMe
root, and the card was retired at the 2026-08-24 reflash, a week before the bag).
125 MB/s is nothing for that NVMe, so the live hypothesis is **CPU contention
from serialisation/mcap encoding**, not I/O. Do not write "disk write path" on a
slide; session 1 said that once and it was wrong.

---

## 2. Findings session 1 owes the deck

Four, in descending order of how much they change a slide.

**(a) Slide 2.2's measured column can now be filled, and it is not a chart.**
2.2's card asks for "the column this table cannot fill offline". A live 60 s
measurement now provides it, so **add a `Measured` column to the existing table
and drop the card** — no figure, no overflow risk, and `CHART-RATES` stays a
single chart carded once, on 2.7. This retires the brief's note that
`CHART-RATES` is "carded twice"; that was a planning error.

Numbers, all from `assets/data/rates_live_20260903_baseline.json`
(gosling1, stack up, parked, Nav2 not running, 60 s):

| Stream | Configured | Measured |
|---|---|---|
| colour / depth / aligned-depth / infra1 / infra2 | 30 | **29.97** (100 %) |
| `camera/imu` → `camera/imu/filtered` | 200 | **199.87 / 200.07** |
| `lidar/scan`, `scan_filtered` | 12 | **8.73 (73 %)** |
| `vehicle/sensors/imu/raw` | 100 | 99.98 |
| `vehicle/sensors/core`, `vesc_odom` | 50 | 50.00 / 50.02 |
| `odometry/local`, `visual_slam/tracking/odometry` | 30 | 29.98 |
| `safety` | 40 | 40.00 |

The LiDAR's 73 % is the only shortfall and it independently confirms
`config/sensors/ydlidar_X4.yaml`'s note that 12.0 configured yields ~8 Hz on this
Jetson. That is now a measurement, not a remark, and it is worth a sentence.

**Say "sensing + localization stack, Nav2 not running, car parked"** — not "under
navigation". Nav2 was off in this launch.

**(b) The coloured pointcloud is OFF by default, and 2.2 says otherwise.**
`launch/bringup.launch.py:164` declares `depthimage_to_pointcloud` with default
`'False'`, which beats both `sensors.launch.py`'s and
`stereo_and_depth_image_processing.launch.py`'s `True` by launch-config
inheritance — the exact leak CLAUDE.md documents. Confirmed live: no depth-cloud
topic exists, and `ros2 component list` shows `sensing_container` holding only the
camera node. Slide 2.2's table lists `camera/depth/color/points` as
**"on by default: yes"**. That is wrong for bringup, and it belongs with
bug-274…277 as a fifth doc error. (It *was* recorded at 27.39 Hz on 2026-09-01, so
that run enabled it somehow — worth one look.)

**(c) Only one bag on the SSD has image topics.** Of 75 bag metadata files,
exactly one — `armA_loop`, the starved one. The August loop/figure8/mapping bags
carry LiDAR, VESC and odometry only. So there is no clean-load *recorded* camera
measurement anywhere, and there never will be without a new capture; the live
measurement in (a) is the only path. That closes out the brief's §2 suggestion to
look at the other August bags.

**(d) `armA_loop`'s numbers, straight from metadata, no data read.**
Useful for 2.7's contrast column: infra1/infra2 and both VSLAM topics at
**0.50 Hz** (107 messages / 214 s) while colour 27.45, depth 27.75,
aligned-depth 27.39, cloud 27.39 and `camera/imu` 199.92 all held. `odometry/local`
6415. Reproducible with `yaml.safe_load(metadata.yaml)` — no bag read needed.

---

## 3. Tooling changed — review before building on it

**`scripts/analysis/bag_stats.py` — MCAP fallback added.** It used to `sys.exit`
at import if `rosbag2_py` was missing. It now falls back to the pure-Python `mcap`
reader for `.mcap` bags, so bags can be analysed **on a workstation with no ROS**,
which is how the charts get built. rosbag2_py is still preferred when present, and
both paths use the message *receive* time so the numbers stay comparable.
Verified against `armA_loop2` off-robot. **No existing behaviour changes when
rosbag2_py is importable.**

**`scripts/live_runs/live_rates.py` — new.** Measures achieved rates on a running
stack. Two things about it matter:

- It meters image streams through their paired **`camera_info`**, because
  subscribing to an image stream costs real bandwidth and would perturb the very
  rates being measured. The 1:1 pairing is checkable in any bag carrying both
  (colour 5875/5875, depth 5939/5939 in `armA_loop`).
- `--stress-images` opens the image subscriptions as *load* while still metering
  via `camera_info`, so a rate that falls is the publisher being starved rather
  than the meter lagging. Keeping stressor and meter separate is the whole point;
  do not "simplify" it into `--direct-images`.

One bug was found and fixed in it mid-session: it queried the ROS graph
immediately after node creation, which returns empty and looks exactly like a
wrong namespace or domain. It now spins until discovery settles. The baseline run
predates the fix and only worked by luck — **it is still valid** (the topic list
it resolved was correct), but re-run it if you want belt and braces.

**Data.** `assets/data/rates_live_20260903_{baseline,stress}.json`, committed as
the source for (a). Cite them as
`src: assets/data/rates_live_20260903_baseline.json; measured 2026-09-03`.

`fit_actuators.py` was **not touched** — the `CFG_STEER_GAIN` question in
`BRIEF_charts.md` §3 is still open and still in scope.

---

## 4. Two traps that cost session 1 an hour each

**You cannot start the stack. Do not try.** librealsense needs a GL context;
Xorg on gosling1 authenticates against `/run/user/128/gdm/Xauthority`, which no
SSH shell can read and `sudo -n` will not give you. An agent-launched stack comes
up with

    [ERROR] [gosling1.camera]: Error starting device: Could not open OpenGL window

and **every other check passes** — VESC, LiDAR, both EKFs, all four TF edges and
AMCL lifecycle all report `[ ok ]` while camera and VSLAM are dead. Session 1
burned four launches on this. `RESET_REALSENSE=True` does not help (the device is
found and resets fine; the GL window is what fails), and **`xhost +local:` run
over SSH does not help** — it authorizes the operator's *forwarded* display, not
`:0`. Ask the operator to launch. One failure with that error is enough to stop.

`71_mpc_stack.sh`'s header tells you to verify `/tmp/.X11-unix/X0` and a non-zero
`/tmp/.docker.xauth`. **Both were true every time and meant nothing.** Fixing that
is a task, not an aside — see **§4a**, which the operator asked for directly.

**The container's shell defaults to the wrong domain.** A fresh `docker exec`
gets `ROS_DOMAIN_ID` unset (0) and `CYCLONEDDS_URI` pointing at
`cyclonedds_config_static.xml`, while the stack runs on **domain 42** with
`cyclonedds_velox1.xml`. `ros2 node list` then returns nothing and looks like a
dead stack. Always export both:

```bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 \
       CYCLONEDDS_URI=file:///mnt/shared_dir/cyclonedds_velox1.xml
source /opt/ros/humble/setup.bash
source /workspaces/f1tenth/install/setup.bash
```

Minor, but it wasted a restart: the VESC's `/dev/ttyACM0` re-enumerates when the
drive pack is connected. A stack launched in that window comes up with
`vehicle/sensors/core` at 0 Hz and looks like a dead VESC. Relaunch after the
device settles.

---

## 4a. A script fix the operator has explicitly asked for

**Do this one — it is in scope, the operator asked for it on 2026-09-03, and it is
the fix that stops the next session repeating §4.** Session 1 found the defect but
did not edit the scripts mid-run.

**What is wrong.** The X11 guidance in `scripts/live_runs/` is prose in three
headers, no script actually checks anything, and the prose describes only one of
the two mechanisms:

- `71_mpc_stack.sh:29-40` — the header says a container started from an SSH shell
  "silently loses the X11 mounts: `/tmp/.X11-unix` comes up empty and
  `/tmp/.docker.xauth` is a 0-byte file". **That is a real failure but it was not
  the one session 1 hit.** Both were present and correct (`X0` socket there,
  xauth 54 bytes) on all four dead-camera launches. The actual failure is
  *authorization*: Xorg authenticates against `/run/user/128/gdm/Xauthority`,
  unreadable by any SSH shell, so `:0` refuses the connection and librealsense
  reports `Could not open OpenGL window`.
- `71_mpc_stack.sh:131` — the failure hint says "check `/tmp/.X11-unix/X0` INSIDE
  the container", which sends you to a check that passes.
- `20_sensor_bag.sh:90` and `21_detection_dataset_bag.sh:156-157` carry the same
  "check the container has `/tmp/.X11-unix` mounted and DISPLAY set" advice.

**The fix.** A real probe, run from inside the container before launching:

```bash
xdpyinfo >/dev/null 2>&1
```

Session 1 verified this distinguishes the states: it returns *"Authorization
required, but no authorization protocol specified"* exactly when the camera is
about to fail, and succeeds otherwise. Note `glxinfo` is **not installed** in the
image, so do not reach for it.

Suggested shape: a `require_gl_display()` helper in `00_env.sh` alongside the
other `require_*` helpers, called by `71_mpc_stack.sh`, `20_sensor_bag.sh` and
`21_detection_dataset_bag.sh` before launch, failing with a message that says
**the operator must launch from a session with real `:0` authority** and that
`xhost +local:` over SSH will not fix it. Then correct the three prose blocks so
they describe authorization as well as mounts.

**Be honest about its limits in the comment**: a passing `xdpyinfo` proves the
display is reachable and authorized, which is necessary but not proof that GLX
itself will work. It catches the observed failure; do not claim more.

**`10_preflight.sh` has no X check at all** — verified 2026-09-03, zero matches for
`xdpyinfo`, `X11`, `DISPLAY` or `xauth`. It is the natural home for one. Note
`00_env.sh` currently has exactly one `require_*` helper (`require_rate`, line 301),
so `require_gl_display()` would be establishing the pattern rather than following
an established one — match `require_rate`'s style and error conventions.

---

## 5. What is still untouched

Five of the seven assets, plus both §5 decisions from `BRIEF_charts.md`:

| Asset | State |
|---|---|
| `CHART-RATES` | **data in hand** (§2a, §2d); chart not built, slide not edited |
| `CHART-CLOSURE` | untouched |
| `CHART-STEER` | untouched — needs the `fit_actuators.py` `--json` + plot split |
| `CHART-SPEED` | untouched — produce-or-retire decision still open |
| `CHART-NAV2-APPROACH` | untouched |
| `FIG-MAP-COMPARE` | untouched — needs `40_build_map_offline.sh --mode both` |
| `TABLE-PACKAGES` | untouched — produce-or-retire decision still open |

`CHART-LAG` is still not ours.

Plus two non-asset tasks: the **§4a script fix** (operator-requested) and the
`fit_actuators.py` `CFG_STEER_GAIN` question from `BRIEF_charts.md` §3.

Suggested order, cheapest first: the §4a script fix, finish `CHART-RATES`
(§1 then §2a), then
`CHART-NAV2-APPROACH`, `CHART-CLOSURE`, `CHART-STEER`, the two decisions, and
`FIG-MAP-COMPARE` last.

`HANDBACK_charts.md` has not been written. Write it per `BRIEF_charts.md` §6, and
carry every finding in §2 above into it — several are slide corrections another
chat needs.
