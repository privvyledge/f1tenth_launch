# Brief: offline charts, continuation (Phase F1, session 3)

**Written 2026-09-03 by the second charts chat.** Read `BRIEF_charts.md` first
(scope, conventions, traps — still the authority), then `BRIEF_charts_2.md` for
session 1's findings. **Where any of the three disagree, this one is newest.**
Two things in `BRIEF_charts_2.md` are now known wrong; see §2.

Working directory `docs/presentation/`, branch `perf/config-tuning`.

---

## 0. State: still nothing committed, still no slide edited

The deck is byte-identical to how session 1 left it. Session 2 spent its robot
window on the experiment `BRIEF_charts_2.md` §1 asked for, and on the script fix
its §4a asked for. Neither touches a slide.

- `STRICT=1 ./build.sh full md` → 70 slides, passes (unchanged, not re-run)
- `./check_overflow.mjs out/deck_full.md` → 1 overflow, slide 31, pre-existing,
  perception chat's

Uncommitted working tree (`git status`):

```
 M scripts/analysis/bag_stats.py          <- session 1 (mcap fallback)
 M scripts/live_runs/00_env.sh            <- session 2 (require_gl_display)
 M scripts/live_runs/10_preflight.sh      <- session 2
 M scripts/live_runs/20_sensor_bag.sh     <- session 2
 M scripts/live_runs/21_detection_dataset_bag.sh  <- session 2
 M scripts/live_runs/71_mpc_stack.sh      <- session 2
?? scripts/live_runs/live_rates.py        <- session 1, extended session 2
?? docs/presentation/assets/data/         <- 5 JSONs
```

---

## 1. bug-272 is REPRODUCED, and the mechanism is not what anyone thought

This is the session's main result and it rewrites slide 2.7 and the bug-272
paragraph in `CLAUDE.md`.

Five 60 s conditions on gosling1, stack up, **parked**, `live_rates.py`, with a
plain subscriber — **no recorder, no disk write, no mcap encoding anywhere**:

| # | cloud published | cloud consumed | 5 images consumed | infra1/2 | VSLAM | colour |
|---|---|---|---|---|---|---|
| A | no  | –   | no  | 29.97 | 29.98 | 29.97 |
| B | no  | –   | yes | 29.97 | 29.97 | 29.98 |
| C | yes | yes | no  | 29.28 | 29.30 | 28.37 |
| D | yes | yes | **yes** | **12.63** | **12.63** | 28.75 |
| E | yes | **no**  | yes | 28.48 | 28.43 | 28.15 |

Data: `assets/data/rates_live_20260903_{nocloud_baseline,nocloud_stress,
cloud_baseline,cloud_stress,cloud_unsubscribed_stress}.json`, in that row order.
Cite as `src: assets/data/<file>; measured 2026-09-03`.

**Only D collapses, and it carries bug-272's exact field signature** — infra1,
infra2 and VSLAM in lockstep to two decimals while colour holds at 96 % and
`camera/imu` at 100 %. That selectivity is what makes it the same failure and not
merely a similar one; it is the same shape `CLAUDE.md` records from the 26 GiB
2026-09-01 bag.

Three conclusions, each pinned to a row:

1. **It is an interaction** (A/B/C/D). Consuming the images alone does nothing —
   session 1's negative result was correct, just missing a factor.
2. **It is not the driver computing the cloud** (E). Publish the cloud and leave
   it unsubscribed and the collapse vanishes. `pointcloud.enable`'s CPU cost
   inside librealsense is not the mechanism; the bytes leaving the container are.
3. **No recorder is required, so mcap encoding is not the mechanism either.**
   The subscriber deserialised and discarded. This **overturns the standing
   hypothesis** in `BRIEF_charts_2.md` §1 and in `CLAUDE.md` ("CPU contention
   from serialisation/mcap encoding"). Do not repeat it.

**Why this is worse than a recording bug, and belongs on the slide as such:**
bug-272 is filed as "don't record pixels you don't need". It is not a recording
property. **Any** external consumer pulling that volume can silently starve
VSLAM — off-board RViz with an image display, a second analysis node, someone's
`ros2 topic echo`. This connects directly to the existing measurement that a
single colour stream costs 31.3 MB/s off-board.

**Do not claim an egress threshold.** Aggregate volume and per-message size are
confounded (images ≈110 MB/s in many small messages; the cloud in few large
ones), and **the car was parked** while bug-272 was measured driving. The run
that would separate them is one image stream + cloud; it was not worth holding
the robot for and remains open.

**Slow storage stays eliminated** and "the disk write path" must not appear
anywhere — no `mmcblk` on gosling1, `/mnt/f1tenth_ssd` is a directory on the NVMe
root, card retired at the 2026-08-24 reflash a week before the bag.

---

## 2. Two corrections to `BRIEF_charts_2.md`

**(a) The flag is `publish_realsense_pointcloud`, NOT
`depthimage_to_pointcloud`.** §1 of that brief names the wrong one and the whole
re-run would have measured nothing. `depthimage_to_pointcloud` drives a
`depth_image_proc` node remapped to `camera/camera/depth/color/points` — doubled
prefix, a **different topic**. The 2026-09-01 bag carries
`/gosling1/camera/depth/color/points` (5862 msgs, 27.39 Hz), which is the
RealSense driver's own cloud, i.e. `publish_realsense_pointcloud` →
`enable_pointcloud` → `pointcloud.enable`.

Session 1's finding (b) — that slide 2.2 wrongly lists the cloud as "on by
default: yes" — **still stands**, and is if anything stronger: *both* flags
default `'False'` at `bringup.launch.py` (lines 164 and 168) and both are beaten
by launch-config inheritance.

**(b) The data files were renamed.** §3 cites
`assets/data/rates_live_20260903_baseline.json`. The 2×2 needed distinguishing,
so the five files are now `rates_live_20260903_<condition>.json` per §1 above.
Session 1's cloud-free pair is rows A and B (`nocloud_*`) and is unchanged —
its numbers in §2a of that brief are still correct, only the filename moved.

---

## 3. §4a is DONE — the operator-requested script fix

Implemented and **verified on the robot**, all four branches:

| DISPLAY | `require_gl_display` |
|---|---|
| `localhost:10.0` (live) | `[ ok ] reachable and authorized` |
| `:0` | `[FAIL] … Authorization required, but no authorization protocol specified` |
| `:77` | `[FAIL] … unable to open display` |
| unset | `[FAIL] … librealsense needs a GL context` |

The `:0` line is session 1's exact failure signature, so the probe catches the
observed failure. `require_gl_display()` lives in `00_env.sh` next to
`require_rate` and matches its style. Called by `71_mpc_stack.sh`,
`20_sensor_bag.sh` and `21_detection_dataset_bag.sh` before launch, and by
`10_preflight.sh` gated on a new **`EXPECT_CAMERA`** (defaults true, mirrors
`EXPECT_VESC`/`EXPECT_JOYSTICK` so a LiDAR-only phase can opt out). The three
prose blocks now describe authorization as well as mounts, and `71`'s old
"check `/tmp/.X11-unix/X0`" hint — which sent you to a check that passes — is
replaced.

**Honest limit, now empirically motivated:** the container's baked
`DISPLAY=localhost:10.0` is a *forwarded* display and it passes. Reachable and
authorized is necessary, not sufficient for GLX. The comment says exactly that
and claims no more. `glxinfo` is not installed in the image.

**`71_mpc_stack.sh` also gained a launch-argument passthrough** (option 1 from
`BRIEF_charts_2.md` §1): any trailing `name:=value` token is appended to the
bringup command; anything else still dies as an unknown option. This keeps the
health checks and `stop_launch_tree` teardown. `-h` range updated to the new
header length. All three parse paths smoke-tested on the robot.

**`live_rates.py` gained two things.** `camera/depth/color/points` has no
`camera_info` twin, so metering it means pulling it — it is load in *every* run,
baseline included. That asymmetry is now reported under "load subscriptions" and
recorded as `self_loading_topics` in the JSON, so it reaches any chart. And
`--skip-topics` leaves a topic entirely unsubscribed, which is what made row E
possible. It is deliberately **not** double-subscribed under `--stress-images`;
the meter already pulls every byte.

Everything is deployed to the container
(`/workspaces/f1tenth/src/f1tenth_launch/scripts/live_runs/`), to
`/mnt/f1tenth_ssd/shared_dir/work/presentation_charts/`, and staged at
`.../work/presentation_charts/staged_20260903/`. **`/workspaces` is a container
layer** — a fresh container reverts all of it; re-`docker cp` from the staging
dir.

---

## 4. Robot-side hygiene the next session will trip over

**Orphans.** Session 1's dead launches left `joy_node` and `vesc_driver_node`
running with no parent (bug-269 signature — those two are exactly the ones that
survive, because they block in a device read). The orphaned `vesc_driver_node`
holds `/dev/ttyACM0`, so the next launch comes up with a dead VESC that looks
like a hardware fault. The operator said they would kill the stack themselves at
the end of session 2; **verify before launching**:

```bash
pgrep -af "joy_node|vesc_driver_node"     # kill by PID; never pkill ros2
```

Two stale watcher loops from session 1 also spin forever — they self-match on
`pgrep -f "ros2 launch f1tenth_launch"`, so their own command line keeps them
alive. `pgrep -af "until ! pgrep"`.

**Domain.** A fresh `docker exec` gets `ROS_DOMAIN_ID` unset (0) and
`CYCLONEDDS_URI` pointing at `cyclonedds_config_static.xml`, while the stack runs
on **domain 42** with `cyclonedds_velox1.xml`. `ros2 node list` then returns
nothing and looks like a dead stack. Export both, then source both setups.

**You still cannot launch the stack.** `BRIEF_charts_2.md` §4 stands in full.

---

## 5. What is left

| Asset | State |
|---|---|
| `CHART-RATES` | **all data in hand** (§1 here, §2a/§2d of brief 2); chart not built, slide not edited |
| `CHART-CLOSURE` | untouched |
| `CHART-STEER` | untouched — needs the `fit_actuators.py` `--json` + plot split |
| `CHART-SPEED` | untouched — produce-or-retire decision open |
| `CHART-NAV2-APPROACH` | untouched |
| `FIG-MAP-COMPARE` | untouched — needs `40_build_map_offline.sh --mode both` |
| `TABLE-PACKAGES` | untouched — produce-or-retire decision open |

`CHART-LAG` is still not ours.

Still open besides the assets: the `fit_actuators.py` **`CFG_STEER_GAIN` stale at
−1.4** question from `BRIEF_charts.md` §3 (untouched by both sessions), and the
one-image-stream-plus-cloud run from §1 above.

**Nothing left in the queue needs the robot.** Suggested order: `CHART-RATES`
(2.2's `Measured` column, then 2.7's chart — which now has a five-condition story
instead of a two-condition one), `CHART-NAV2-APPROACH`, `CHART-CLOSURE`,
`CHART-STEER`, the two decisions, `FIG-MAP-COMPARE` last.

`HANDBACK_charts.md` is still unwritten. Write it per `BRIEF_charts.md` §6, and
carry §1 and §2 above into it — the bug-272 mechanism and the flag correction are
slide *and* `CLAUDE.md` corrections another chat needs. Note also that
`.wolf/buglog.json` has no entry for any of this yet.
