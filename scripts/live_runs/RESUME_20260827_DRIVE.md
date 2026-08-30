# Resume — everything left, in the order it can be done

> ## ★ START HERE — handoff written 2026-08-27 ~10:20 EDT
>
> **This document is the handoff. Do not write a new RESUME/HANDOFF file** — §7 records that this
> family has repeatedly shipped stale blockers, and there were three of these in under 24 hours.
> Correct this one in place, in the same commit as the work.
>
> **Machine state:** Jetson was shut down ~09:35 to recharge and comes back **on AC power**. The
> container and everything in `/workspaces` are gone with it — **re-stage before anything**
> (§6, and §0★.2 of `DEMO_RUNBOOK_20260810.md`).
>
> **DONE 2026-08-27 ~18:05 EDT — the tarball was re-cut and staged.** `config/nav2_params.yaml`
> `movement_time_allowance` **100.0 → 10.0** (the nav2 default) is committed as `614c463` and now
> ships: **`stage_0827.sh`** carries **`f1tenth_stage_20260827.tgz`**
> (md5 `9c1c6b66fb8531e8fb4c48e917908655`, `git archive HEAD` over the same path set as 0826c, 180
> entries, verified to contain `movement_time_allowance: 10.0`), and adds a ninth installed-tree
> check for that value. Both are on the robot at `/mnt/f1tenth_ssd/shared_dir/`. **Use `stage_0827.sh`
> — re-staging `0826c` silently reverts to 100.0.** The change is still **UNTESTED on hardware**.
>
> **RESULT 2026-08-27 evening — both of the first two jobs are DONE, parked, with no battery.**
> See §4a. `movement_time_allowance: 10.0` is **verified on hardware**, `BackUp` reached by the BT
> itself is **observed for the first time**, and **bug-257 is root-caused and fixed** (`aced708`).
> Stage with **`stage_0827b.sh`** from here on; `use_composition:=False` is no longer needed.
>
> **Original first job, kept for the record:**
> 1. Stage with `stage_0827.sh`, launch per §0★.6 of the runbook — **`use_composition:=False` is
>    required** until bug-257 is understood.
> 2. Wait for lifecycle **ACTIVE** (not just for the nodes to exist), start a bag *including*
>    `/goal_pose`, hold R1, send the same goal.
> 3. Expected if the change works: the car stalls ~0.38 m short as before, then **aborts within 10 s
>    and fires a recovery** — which would close `testing_checklist.md`'s "`BackUp` reached by the BT
>    itself", never once observed. If instead a legitimately slow approach aborts early, revert to
>    100.0; the comment in the file says so.
>
> ---
>
> ## ★★★ NEXT SESSION STARTS HERE — rewritten 2026-08-30 ~18:35 EDT
>
> **STOP. The container is in a BROKEN PARTIAL-STAGE state. Fix that before anything else.**
>
> `jetson_container_20260830_181631` is up, but `stage_0830.sh` **died halfway** — the lab network
> lost internet and step 2 (`imu_processors` from the `privvyledge/imu_pipeline` fork) failed with
> `SSL connection timeout`. `set -eo pipefail` aborted the script there. Measured state right now:
>
> | | state | consequence |
> |---|---|---|
> | `src/.../data/maps/20260805/` | **present** (step 1 ran) | — |
> | `install/.../launch/*.launch.py` | **updated**, defaults point at `20260805` | symlinked to src |
> | `install/.../data/maps/20260805/` | **MISSING** (a later step never ran) | **`map_file` resolves to a path that does not exist — `map_server` will not load any map** |
>
> **So a launch right now is worse than an unstaged one**: the launch files ask for the current map
> and the file is not there. **Re-run `stage_0830.sh` to completion once the lab network is back.**
> If internet is still down, its step 2 is the only part that needs it — the rest can be run by hand,
> but check whether the image's existing `imu_processors` (present at
> `/workspaces/f1tenth/install/imu_processors`) is the 0.5.2 build with `stationary_timeout` before
> assuming the skip is harmless. **`remove_imu_bias` is `False` at both call sites anyway**, so it is
> not on the critical path for a map or Nav2 session.
>
> **New guards, added 2026-08-30 — they will catch this class of failure for you.**
> `scripts/live_runs/10_preflight.sh` now fails if the installed tree lacks `data/maps/20260805` or
> still defaults `map_file` to `raslab.yaml`. On the robot, `/mnt/shared_dir/stagecheck.sh` does the
> same and is sourced by `viz_launch.sh`, `mapcheck_launch.sh`, `throttle_launch.sh` and
> `launch_run.sh`, which now **refuse to launch** an unstaged container. `testing_checklist.md` §7b
> records the map-origin fingerprint: **(−9.29, −6.06) = current 20260805, (−3.95, −9.87) = retired
> raslab** — read it off `/map` rather than trusting a filename.
>
> ### The map-alignment question — what is settled and what is NOT
>
> **Settled, offline, no robot:** the repo's 2D grid and 3D cloud **are not rotated relative to each
> other.** Rendered as published with no alignment applied they trace the same closed ring
> (`scripts/analysis/map_cloud_align.py --overlay`). The 2026-08-10 co-registration measurement
> stands, and the `origin:` hypothesis is refuted in the same picture.
>
> **Two traps recorded in that script's header, both of which nearly produced a false finding:**
> a yaw sweep scored by median nearest-cloud distance reports a confident **+31 deg** optimum that is
> **entirely spurious** (the cloud is denser than the grid and does not cover its footprint, so any
> rotation packing grid cells into the dense annulus wins) — and it agrees with the operator's
> reported ~30 deg to within a degree, so it would have "confirmed" the wrong answer. Also: **this lab
> is a closed ring, not a rectangle**, so wall-orientation methods do not apply here at all.
>
> **NOT settled — do not repeat this session's overreach.** A fresh, unstaged container was measured
> serving the retired raslab grid (live `/map` origin **(−3.950, −9.870)**, 173x258), the **Jan-2024**
> `rtabmap/raslab/cloud.pcd`, and no `data/maps/20260805/` at all — a pairing that renders exactly
> like a rotated 3D map. That is a **real and reproducible failure mode**, and it is now guarded. But
> it is **not established to be what the operator saw**: the operator points out that the run in
> question was in the *earlier, staged* container (`..._151510`), whose installed tree was verified
> this session to carry the correct 20260805 defaults at both entry points. **So the cause of the
> observed misalignment is still open.** Do not write it up as solved.
>
> **How to actually settle it, in one run:** stage properly, launch with
> `launch_visualization:=True`, and run `scripts/live_runs/map_overlay_check.py --ns ""` — it matches
> the live `/map/pointcloud` against both candidate `.pcd` files by point count and bounding box
> (current = 96199 pts, x max 3.82 m; Jan-2024 = 76122 pts, x max 15.04 m), so the cloud on the wire
> is **named**, not inferred, and it prints the grid extent and `map→base_link` alongside. Then take
> the screenshot. **The screenshot is the part that distinguishes the two live hypotheses**: cloud
> offset from grid (a map problem) versus car, footprint and live scan rotated inside a correct map
> (a localization problem, bug-231/232). Topic data alone cannot separate those.
>
> ### Also measured this session, and unexplained
>
> On that unstaged container, parked with no drive battery, **`map→base_link` read (6303, 1795) m and
> was growing ~71 m/s**. That is the bug-244 signature, and the image predates the bug-248
> `imu_filter_constant_dt` fix, with `vesc_odom` silent (no VESC) so nothing arrests it — consistent,
> but **not verified**, because the run was abandoned. Re-check on a properly staged container; if it
> persists *after* staging, bug-244 is not as closed as `BUG244_CLOSEOUT_20260826.md` claims.
>
> ### Display / container operations — learned the hard way this session
>
> **The container was lost once tonight.** It runs with `--rm` and its main process is the shell of
> the session that created it; when that MobaXterm session dropped, the container exited and was
> auto-removed, taking a warm 3-hour stack. Logs survived only because they are on the SSD.
> **Always start it under `tmux`** (now installed) — `tmux new -s f1tenth`, run
> `~/bolus_ws/f1tenth_launch.sh`, `Ctrl-b d`. A no-tmux fallback that detaches via a FIFO is staged at
> `~/bolus_ws/f1tenth_launch_detached.sh`.
>
> **`jetson-containers` bakes `$DISPLAY` in at container creation and it cannot be repointed
> afterwards.** Validate the display *before* creating the container:
> `xdpyinfo -display $DISPLAY`. A dead forwarded display still shows an sshd listener on its port, so
> `ss -ltn` proves nothing — only `xdpyinfo` does.
>
> **The xauth key form matters, and this is the detail CLAUDE.md was missing.**
> `xauth extract` writes a key of the form `<host>/unix:N`, which matches only a **Unix-socket**
> connection; a container connecting over TCP to `localhost:N.0` needs the **FamilyWild** form. The
> symptom is `MoTTY X11 proxy: No authorisation provided`. The fix:
> ```bash
> xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge -
> ```
>
> ### What this session closed
>
> **VSLAM frame-delta warnings — CLOSED**, `testing_checklist.md` §8 now `[x]`. Present live at 1x
> with `use_gpu:=True` on every launch (so the standing "replay artefact, eliminated by the use_gpu
> fix" prediction is **refuted**), but benign: a cuVSLAM tracker startup transient beginning 39-134 ms
> after tracker init, lasting 129-615 ms, never recurring — 1107 s of silence in the longest run.
>
> **Throttle interpolator — CLOSED**, §1 now `[x]`, logged as **bug-262**. PASS on hardware, parked,
> no battery. **The fix the operator remembered does exist**: `63673f5` in **`f1tenth_system`** — a
> *different repository*, which is why every search of this repo came up empty. General lesson:
> establish which repo owns the file before concluding a fix does not exist. Measured over 60 s of
> stick sweeps: mux and gate ±0.3140 rad, interpolator in and out both servo 0.2005-0.9195, output at
> 75 Hz. The test topics this doc previously named were both *upstream* of the interpolator and could
> never have seen the bug.
>
> **Commits:** `3b55711`, `ab99d59` and the guards commit, all pushed to `origin/perf/config-tuning`.
>
> ---
>
> ## ★★ NEXT SESSION STARTS HERE — rewritten 2026-08-30 ~16:25 EDT
>
> **All five parked items in §1 are now closed. The 2026-08-30 session closed (c) bug-260 and (e)
> the particle cloud, and resolved the 8.4 Hz odometry finding — details further down this block.
> `testing_checklist.md` is at 2 open items, from 9.**
>
> **The operator has set the next two tasks, in this order. Neither needs the drive battery.**
> **Task 1 is closed (2026-08-30, below); task 2 is the live one.**
>
> ### ~~1. VSLAM frame-delta warnings~~ — CLOSED 2026-08-30, `testing_checklist.md` §8 is now `[x]`
>
> **The checklist's prediction was wrong and the item closes anyway.** The warning is present
> **live, at 1x realtime, with `use_gpu:=True`, on every launch** — settled by grepping the
> `claude_bringup_0830/` logs, no replay needed. So it is not a 100x-playback artefact, and the
> `use_gpu` fix did not eliminate it (with `use_gpu:=False` VSLAM does not run at all, which is
> evasion rather than a fix).
>
> It is nonetheless benign: a **cuVSLAM tracker startup transient**. Across all three
> `use_gpu:=True` runs the burst begins **39-134 ms after `cuVSLAM tracker was successfully
> initialized`**, lasts **129-615 ms**, and then never recurs — including **1107 s (18.4 min) of
> silence** in `log_pcloud`. Deltas are marginal (34.0-37.0 ms against a 34 ms threshold) and
> **decay monotonically back under it**, with one ~190-197 ms outlier per burst: the tracker
> draining frames queued during `CUVSLAM_WarmUpGPU()` + `CUVSLAM_CreateTracker()`. VSLAM odometry
> held 29.9-30.2 Hz throughout. Full table in the checklist entry. **Do not tune the threshold or
> `stereo_fps` for this**; a warning appearing more than a second after tracker init would be a
> different finding.
>
> ### 2. The map-alignment question — (a) ANSWERED 2026-08-30, (b) still needs one look
>
> **(a) The 2D grid and the 3D cloud are NOT rotated relative to each other. The 2026-08-10
> measurement stands; the ~30 deg appearance comes from somewhere else.** Settled offline, no robot,
> with `scripts/analysis/map_cloud_align.py --overlay`: rendered as published, with no alignment
> applied, `rtabmap_2d_final` and `cloud_voxel_0p05.pcd` trace **the same closed ring**, the outer
> occupied cells sitting on the cloud boundary. A 30 deg rotation of a 12 m map would be
> unmistakable at that scale and is simply not there. The doc's own `origin:` hypothesis is refuted
> in the same picture: `origin: [-9.29, -6.06]` places the grid exactly co-located with the PCD's own
> coordinates, so the two anchorings agree.
>
> **Two things this cost, both worth not repeating.**
>
> **The obvious metric lies here, and it lies in agreement with the hypothesis.** A yaw sweep scored
> by median distance from each occupied grid cell to the nearest projected cloud point reports a
> confident optimum at **+31 deg** (median 0.018 m vs 0.051 m at zero) — a number that matches the
> operator's ~30 deg almost exactly, and is **entirely spurious**. The cloud is far denser than the
> grid and does not cover the grid's full footprint, so any rotation that packs grid cells into the
> dense annulus scores well with no structural agreement whatsoever. The caveat is now written into
> the top of `map_cloud_align.py`. **Render the overlay and look before reporting any alignment
> figure.**
>
> **This lab is not rectangular — it is a closed ring.** Wall-orientation histograms and anything else
> assuming dominant perpendicular walls do not apply (attempted here: n=45-278 with peak/mean 2.7-2.9,
> i.e. no dominant direction to find). CLAUDE.md's recurring "a rectangular lab is symmetric to a
> planar LiDAR" framing describes the *localization* discussion, not this map's geometry.
>
> **What almost certainly produced the appearance: the OLD cloud.**
> `data/maps/rtabmap/raslab/cloud.pcd` (Jan 2024, 76 122 pts) spans **x [-5.65, 15.04], y [-8.27,
> 7.20]** against the grid's x [-8.76, 3.49], y [-5.53, 3.37] — a different room extent entirely,
> sitting in the upper-right quadrant and spraying out to x=15. Paired with the current grid it looks
> exactly like a rotated, offset 3D map. **That file is still present on the robot.** Until bug-237
> was fixed on 2026-08-11, `teleop.launch.py`'s `pointcloud_map_file` defaulted to it, so an
> observation made before that date is fully explained.
>
> **Verified 2026-08-30: today's stack cannot reproduce it by default** — the *installed tree* on
> gosling1 (`/workspaces/f1tenth/install/f1tenth_launch/share/f1tenth_launch`) defaults
> `pointcloud_map_file` to `data/maps/20260805/cloud_voxel_0p05.pcd` at **both** entry points, and
> `data/maps/20260805/` there symlinks to the correct source files. So this is checked at the level
> that matters (the installed tree, not the repo — cf. the staged-tarball trap).
>
> **What is left on (a):** confirm with the operator *when* they saw it and whether it still
> reproduces on a current launch with `launch_visualization:=True`. If it does reproduce today, the
> old-cloud explanation is wrong and the next suspect is an explicit `pointcloud_map_file` override
> in whatever script was used.
>
> **(b) The `map` frame reading ~90 deg rotated and offset — almost certainly the documented
> -84.5 deg, i.e. not a defect.** Unchanged from the analysis below; it takes one command to settle
> and was not run this session because the localization stack was not up:
> ```bash
> ros2 run tf2_ros tf2_echo map base_link   # expect ~(0.445, -0.575, -84.5 deg), NOT identity
> ```
> Cold launch #8 on 2026-08-30 already reproduced (0.445, -0.575, -84.5 deg) to within a few mm and
> 0.5 deg, so the prior is strong. If it **is** -84.5 deg, (b) is not a defect and the real question
> is why the map frame's origin does not coincide with the bag-recording spot — a map-build/export
> question. If it is **not**, suspect localization rather than the maps (bug-231/bug-232: a bad
> `odom->base_link` rotates the live scan and footprint inside a correct static costmap and looks
> exactly like a rotated map).
>
> **Related, and worth reading before starting:** three conflicting parking-spot headings are on
> record for one spot (-79.8 / -86.5 / -92.1 deg), and figure-8 waypoint 0 (-92.08 deg) does not fit
> the map. Do not seed anything from waypoint 0.
>
> ### ~~3. The throttle interpolator~~ — CLOSED 2026-08-30, `testing_checklist.md` §1 is now `[x]`
>
> **The fix existed all along and this doc was searching the wrong repository.** It is
> **`63673f5` in `f1tenth_system`** ("throttle_interpolator: add startup guard and safety timeout",
> authored by the operator 2026-05-26). `throttle_interpolator.py` lives in
> `f1tenth_system/f1tenth_stack/`, **not in `f1tenth_launch`** — so `git log --all --grep=throttle`
> here, `git log -S` on `vehicle.launch.py`, and all 141 buglog entries were guaranteed to come up
> empty. **General lesson: before concluding a fix does not exist, check which repo owns the file.**
> The operator's recollection of "the mux and the joystick" does not match the commit; the actual
> cause was the node publishing servo *center* at 75 Hz from t=0, as that topic's sole publisher,
> before any upstream command existed.
>
> **Verified on hardware 2026-08-30, parked, on AC, no drive battery, `/dev/sensors/vesc` absent.**
> 60 s of full stick sweeps with `launch_throttle_interpolator_node:=True`
> (`scripts/live_runs/throttle_interp_check.py`, staged as `/mnt/shared_dir/throttle_launch.sh`):
> mux and gate both ±0.3140 rad, interpolator **in** and **out** both servo 0.2005–0.9195, output at
> 75 Hz. Nothing saturates on either side. Logged as **bug-262**.
>
> **Two traps this cost:** the test topics this doc previously named (`ackermann_drive`,
> `vehicle/ackermann_cmd`) are both **upstream** of the interpolator, which sits in VESC command
> space downstream of `ackermann_to_vesc` — they look identical with it on and off and can never see
> the bug. And parked idle reads **0.56** on `commands/servo/position` by design (the 0.5 s
> watchdog), which is not the old centering bug.
>
> ### Then, needing the battery
>
> The 0.379 m Nav2 stall, obstacle avoidance, particle-cloud convergence under *sustained* motion
> (the 2026-08-30 close left that one caveat open), and arm A (§3).
>
> ---
>
> ### What the 2026-08-30 session settled
>
> **Items 1 and 2 below are DONE, parked, on AC. Both are struck through and their results
> recorded. What is left needs the battery — go to item 3.**
>
> **bug-260 — FIXED and verified 2026-08-30.** Root cause was **not** one of the three hops this
> doc named; all three were correct, and in-launch probes proved it (`bringup` -> `mapping` ->
> `teleop` all carry `launch_localization='False'`). The defect is that teleop's localization
> include sits inside `TimerAction(period=10.0)`, so its `IfCondition(launch_localization)` is
> evaluated **~10 s later** — by which time the enclosing `GroupAction` scope that carried
> bringup's `'False'` has been **popped**, and the unevaluated `LaunchConfiguration` resolves
> against bringup's own top-level default `'True'`. Measured directly: `teleop launch_setup`
> reads `'False'`, `teleop@fire` reads `'True'`. Fixed by freezing the value at `launch_setup`
> time (`teleop.launch.py:499`, used at `:682`). Verified on a clean launch: **0 duplicate node
> names, no `/amcl`, no `/map_server` under `slam:=True`, `/map` Publisher count 1
> (`slam_toolbox` only — bug-027 closed), TF still 0/0/0, 42 nodes.** `testing_checklist.md` §10
> mapping mode is now `[x]`.
>
> **This is a general trap, not a teleop one:** a condition on an action inside a `TimerAction`
> is evaluated after enclosing scopes have popped. Any `LaunchConfiguration` a parent set for a
> child, and the child consumes at deferred-execution time, is at risk. bringup's own timers are
> safe **only** because they read bringup's own top-level arguments.
>
> **The 8.4 Hz `odometry/local` finding — RESOLVED 2026-08-30, and it was not what was feared.**
> Controlled A/B, same launch (`slam:=False launch_navigation:=True`), parked, nothing else
> changed:
>
> | `launch_visualization` | `odometry/local` | VSLAM | `Failed to meet update rate` |
> |---|---|---|---|
> | `False` | **30.10 / 30.02 / 30.05 Hz** | 29.96 Hz | 0 |
> | `True` (RViz up) | **30.06 / 30.02 / 30.01 Hz** | 30.23 Hz | 0 |
>
> **Visualization does not gate fusion measurements.** With RViz up it took **98 % of one core**
> (higher than the 70 % previously measured — this run rendered over forwarded X11) and the system
> still showed **41.5 % idle**; `ekf_node` sat at 9 %. Both named candidates are refuted: CPU
> contention does not do it, and there was no degraded EKF input (`launch_icp_odometry` confirmed
> off, no ICP node, VSLAM alive both arms). By elimination the surviving explanation is the
> doc's own third one — **the hand-carry** immediately before the 2026-08-27 reading. 8.4 Hz was
> **not reproduced**, so treat that as the likely cause rather than a proven one, and re-measure
> rather than assume if it ever reappears.
>
> The three items above are **done** (bug-257 fixed, `movement_time_allowance` verified, `BackUp`
> reached by the BT). What follows is what a fresh chat should pick up, in order. The first two need
> **no battery and no VESC** — see §4a for why a parked car exercises Nav2 fully.
>
> **~~1. bug-260 — `slam:=True` double-launches localization.~~ DONE 2026-08-30 — see above.** Fully specified in §1(c): TF is clean,
> but `/ekf_odom_node`, `/rf2o_laser_odometry`, `/rtabmap_stereo_odom` and
> `/pointcloud_map_publisher` each run twice, and **AMCL + `map_server` run under `slam:=True`** so
> `/map` has two publishers (bug-027 — a map saved during a mapping run may come from the stale
> file). The mechanism is pinned to one of three hops; §1(c) names the exact files and line ranges
> and says to trace the *performed* value rather than guess. Self-contained, parked.
>
> **~~2. `odometry/local` at 8.4 Hz instead of ~30 Hz.~~ RESOLVED 2026-08-30 — see above.** Measured 2026-08-27 20:20 on the
> `particlecloud_e` launch (`slam:=False`, `launch_navigation:=True`, **`launch_visualization:=True`**,
> one RViz, car parked after a hand-carry): `ros2 topic hz /odometry/local` → **8.356 then 8.649 Hz**,
> min 0.097 s / max 0.200 s. The healthy baseline for this stack is **29.70 Hz with a 0.180 s max
> gap**, and the parked fusion baseline earlier the same evening had 1800 samples in 60 s (= 30 Hz).
> **This gates every fusion measurement taken with visualization on, so settle it before trusting
> one.** Candidates, cheapest first, and none of them checked yet:
> - **CPU contention.** RViz was measured at **70 %** of a core in the 2026-08-27 CPU comparison
>   (§4 / `testing_checklist.md`). §8's rule applies: *low-but-nonzero sensor rates can be CPU
>   contention, not a fault — check `uptime` first.* The controlled test is one launch with
>   `launch_visualization:=False` and one with `:=True`, nothing else changed.
> - **A degraded EKF input.** The known signature is `ekf_odom` logging
>   `Failed to meet update rate!` — with `launch_icp_odometry:=True` that produced **12.78 Hz with
>   2.44 s gaps**. `launch_icp_odometry` defaults `False`, so confirm it really is off rather than
>   assuming. Check the `yaw_drift.py` source table for a source with **0 samples** (see §2 — a dead
>   VSLAM is silent because `odometry/local` normally *stays* at 30 Hz on the remaining sources;
>   here it did not, which is itself informative).
> - **The hand-carry.** The car had just been lifted and set down, which can leave rf2o and VSLAM
>   unhappy. Re-measure on a launch where nothing touched the car — that alone may retire this.
>
> **§1(e) particle cloud — CLOSED 2026-08-30, and it did NOT need the battery.** Both blockers
> cleared with the car rolled by hand on AC. The QoS half is fixed in the tracked config —
> `config/f1tenth.rviz` now carries a `nav2_rviz_plugins/ParticleCloud` display with
> `Reliability Policy: Best Effort`, so no GUI steps. The motion half turned on a distinction this
> doc had not drawn: **rolling the car works, carrying it does not.** A lifted car gives rf2o no
> scan delta and VSLAM no tracking, so `odometry/local` reads 0.000 m — which is why 2026-08-27
> failed. Measured with `pcloud_watch.py`: parked 0 msgs over 40 s, the push (odom path → 3.4 m)
> produced 42 msgs of 4000 particles, then publishing froze when motion stopped. **Caveat:** spread
> *expanded* 0.076 → 0.364 m over the ~4 s push, so **convergence under sustained motion is still
> unproven** — carry that into the drive session.
>
> **3. Then, needing the battery:** the second lead on the 0.379 m stall (last command 0.269 m/s vs a
> measured 0.20–0.26 m/s ground breakaway — carry it in **ERPM**, not m/s); obstacle avoidance;
> ~~§1(e) the particle cloud~~ (closed 2026-08-30, see above; only convergence under sustained
> motion is left, and it rides along with any driven goal); then arm A (§3),
> which needs a battery and a taped floor.
>
> **Machine state as of 2026-08-30 15:45:** container `jetson_container_20260830_151510` up and
> warm, stack torn down, staged with **`stage_0830.sh`** (md5 `137531c39628609afe260cb19fba27c0`,
> from `e07c2e1`), 12/12 installed-tree checks pass. Jetson on **AC**, clock correct, root fs 12 %
> used, RealSense and YDLidar present.
> **Zero commits local and unpushed as of 2026-08-30 17:05** — `origin/perf/config-tuning` is at `c3b5c4d`, i.e. everything through the bug-260 fix and the particle-cloud work is pushed. The line here previously said sixteen; that was the **ninth** stale claim caught in this family. Re-run `git log @{u}..HEAD`, do not trust this number.
> Drive battery and DualSense still off; `/dev/sensors/vesc` and `/dev/input/js0` absent until the
> operator connects them.
>
> **Teardown warning, learned the hard way this session:** a kill pattern matching only
> `/opt/ros/humble/` leaves every node under `/workspaces/f1tenth/install/` alive, and those orphans
> then read as duplicate nodes and double-launched subsystems on the *next* launch — it voided one
> run of §1(c) before the ages were checked. Use `/mnt/shared_dir/teardown2.sh`, which matches both,
> and confirm a launch is clean by checking that `ps -eo etimes` forms **one** age cluster.
>
> ---
>
> **Read §7 before planning around any blocker named here.** **Seven** stale ones have now been caught
> in this family — §1(c)'s "bug-022, fixed 2026-08-04" was the seventh, found 2026-08-27. Verify each against the machine first — `git log @{u}..HEAD`, `git status`, read the
> shipping source — then fix the doc in the same commit.
>
> **Six commits are local and unpushed**, checked 2026-08-27 18:04 with `git log @{u}..HEAD`:
> `ca645bf`, `b23e8e9`, `57aff53`, `5a4cce9`, `309b1e2`, `614c463` (the `movement_time_allowance`
> change). The line here previously said five and omitted `ca645bf` — re-check rather than trusting
> it, which is exactly the kind of claim §7 is about.
>
> **Machine check 2026-08-27 18:06 (robot up 4 min, on AC):** clock correct, load 0.13, root fs
> 915 G / 12 % used, RealSense **present**, YDLidar CP210x **present**, **`/dev/sensors/vesc` ABSENT
> (drive battery off)** and **`/dev/input/js0` ABSENT (DualSense not connected)**. Both are operator
> actions and both are required before any drive — no VESC means no actuation, no joystick means the
> `command_gate` heartbeat never arrives and the gate stays shut.


**Repo:** `f1tenth_launch` · branch `perf/config-tuning` · pushed through `1ad686c`
**gosling1:** `192.168.2.195`, direct `ssh gosling1@192.168.2.195` (no jump host)

Written 2026-08-26 ~19:45 EDT, extended ~20:20 with the Nav2 arm and the parked items.
**Replaces `RESUME_20260827_DECISIONS.md`** — recover it from git history for the original wording.

This is the whole remaining backlog for the repo, not one thread of it. It is ordered so that a
session with no battery still closes items, and so that everything needing the car happens on one
charge.

**Read this section, then §1.** Nothing else is needed to start.

| Where | What | Needs |
|---|---|---|
| §1 | Five parked items, incl. the cold-launch seed acceptance | Robot powered, no battery |
| §2 | bug-245 check — 30 s, gates every measurement after it | First launch |
| §3 | Drive session, arm A — sysid, IMU bias, steering, wheelbase | Battery |
| §4 | Drive session, arm B — Nav2 closed-loop | Battery + a map |
| §5–§9 | What closed, restoring the robot, traps, environment | — |

`testing_checklist.md` has **9 open items**; §1 closes up to four of them parked, §3–§4 reach the
rest. The mapping in §9 is the authoritative list — the checklist markers are the record, this doc
is the plan.

---

## 1 · Before the battery — five parked items

**STATUS 2026-08-30: (a) ACCEPTED, (b) PASS, (c) FIXED AND PASSING (bug-260), (d) PASS.
(e) attempted — both blockers identified, needs the drive session. Four of the five parked items
are closed; only (e) remains, and it needs the car to move.** (c) mapping mode was never launched; (e) the particle-cloud *display* was never looked
at (the topic exists as `nav2_msgs/msg/ParticleCloud` but stays silent parked, because AMCL is
motion-gated — so this needs a drive *and* an operator at RViz).

These need the robot powered and the stack launched, but no drive. Do them first; each is minutes.

**(a) bug-241 seed acceptance — the one that gates Nav2.** The `initialpose` auto-seed
(`localization.launch.py` reads `initial_pose` out of `localizer_amcl.yaml` and publishes it at
`initialpose_seed_delay`, default 20 s) was written 2026-08-13 and **has never been verified on
hardware**. Acceptance is **repeated cold launches with no manual seed** — at 2:1 prior odds of
passing by luck, budget **five or six**. Per launch, read `map→base_link` once localization settles:

```bash
ros2 run tf2_ros tf2_echo map base_link      # expect ~(0.445, -0.575, -84.5 deg), NOT identity
```

**Tally, 2026-08-27** — the acceptance sample runs under the §6 launch environment (`_lo` + domain
42); a launch under the container-default DDS config is recorded separately, since bug-238 is a
discovery-timing race.

| # | Env | `map→base_link` | Verdict |
|---|---|---|---|
| — | container default (static DDS, domain 0) | (0.441, −0.577, −84.73°) | pass, **not counted** |
| 1 | `_lo` + domain 42 | (0.445, −0.575, −84.46°) | **pass** |
| 2 | `_lo` + domain 42 | (0.444, −0.574, −84.50°) | **pass** |
| 3 | `_lo` + domain 42 | (0.441, −0.578, −84.40°) | **pass** |
| 4 | `_lo` + domain 42 | (0.447, −0.576, −84.55°) | **pass** |
| 5 | `_lo` + domain 42 | (0.449, −0.575, −84.60°) | **pass** |

**ACCEPTED 2026-08-27. bug-241 is verified on hardware.** Five cold launches under the §6
environment, plus one under the container default, **6 passes and 0 identities**. Spread across the
five: 8 mm in x, 4 mm in y, **0.20° in yaw** — the auto-seed lands on `localizer_amcl.yaml`'s
(0.445, −0.575, −1.4748) every time, and `ekf_map` takes it every time.

Residual, stated honestly: at the doc's own 2:1 per-launch odds, five consecutive passes leave
roughly a 13 % chance of a lucky streak, so this is ~87 % confidence, not proof. That is the bar
this doc set and it is met. If a future run ever reads identity, the diagnosis is unchanged —
`ros2 topic info /amcl_pose --verbose` and read the **durability** of both ends, not the subscriber
count.

**bug-245 also verified across the same six launches: 6/6**, `/visual_slam/tracking/odometry` at
29.94–30.19 Hz every time, no exit −6. The crash half of that bug is closed. What remains is
bug-256, the unconditional `visual_slam/initial_pose` remap — cuVSLAM still eats the seed as a
relocalization hint and logs `Error 4` five times per launch. Harmless with an empty map path,
noisy, and it re-arms under `localize_on_startup:=True`.


### Parked fusion baseline, 2026-08-27 (launch 2, `yaw_drift.py 60`)

Take the drive session's moving numbers against this, not against the 2026-08-06 figures.

| source | n | °/min |
|---|---|---|
| **EKF local (`odometry/local`)** | 1800 | **+0.04** |
| Isaac VSLAM (VO) | 1799 | +0.04 |
| rf2o LiDAR odom | 524 | −0.17 |
| EKF global (map) | 600 | −0.42 |
| VESC wheel odom | 3002 | 0.00 |
| VESC IMU (orientation) | 6005 | −14.89 |
| RealSense IMU (orientation) | 12002 | +5.46 |
| AMCL | 0 | — (motion-gated parked, expected) |

z-gyro bias while stationary: **VESC +0.004584 rad/s** (15.76 °/min, the bug-129 defect, still
correctly excluded by `imu0` yaw `false`); **RealSense −0.002114 rad/s**.

**Two findings worth carrying forward.** (a) `rf2o` chatters its zero-velocity gate on a parked car
— alternating "Motion detected (v 0.021–0.029 m/s)" / "Zero-velocity detected" on 0.006–0.012 m scan
diffs, i.e. sitting on the 0.02 m/s threshold — but it costs only −0.17 °/min and fused yaw is
unaffected. **Cosmetic; do not retune it before the drive.** (b) The RealSense z-bias −0.002114
rad/s agrees with the bias remover's offline converged estimate of −0.00214 to **2.6e−05**, live, on
a different day and launch. That independently corroborates `BIAS_REMOVER_OFFLINE_20260826.md`,
which until now rested only on a bag replay. It does **not** discharge the `remove_imu_bias`
decision — §3's owed evidence is *driven* heading, which no parked run can reach.



Identity means `ekf_map` started from a zero state (bug-238: `amcl_pose` is published
TRANSIENT_LOCAL but `robot_localization` subscribes VOLATILE, so a late subscriber gets nothing and
AMCL is motion-gated, so parked there is no second chance). **Diagnose with
`ros2 topic info /amcl_pose --verbose` and read the *durability* of both ends, not the subscriber
count** — the count reads 1 in both the working and the broken case.

**(b) Teleop mode, full stack** — `testing_checklist.md` §10, still `[ ]`:

```bash
ros2 launch f1tenth_launch bringup.launch.py slam:=False launch_navigation:=False
ros2 run tf2_tools view_frames     # expect map -> odom -> base_link -> lidar/camera/imu
```

**(c) Mapping mode, no TF conflicts** — §10, now **`[x]`**. **FIXED AND VERIFIED 2026-08-30
(bug-260).** TF was already clean; the double-launch is now gone too — 0 duplicate node names, no
`/amcl` or `/map_server` under `slam:=True`, `/map` Publisher count 1. **The "one of those three
hops drops the value" diagnosis below was wrong** — all three hops were correct, and probing them
is what proved it. The real cause was `TimerAction` deferral outliving the `GroupAction` scope;
see the ★★ block at the top. Kept below for the record.

*What passes:* **0** `TF_REPEATED`, **0** authority warnings, **0** extrapolation warnings, and no
duplicate `/command_gate` (that suppression works, so the bug-016 safety hazard did not recur).

*What fails,* on a clean launch verified by process ages forming one 136–164 s cluster:
- `/ekf_odom_node`, `/rf2o_laser_odometry`, `/rtabmap_stereo_odom` and `/pointcloud_map_publisher`
  each appear **twice**. `rf2o` is instantiated into `localization_container` at **t+11 s**
  (bringup's own localization, 10 s timer) *and* into `f1tenth_container` at **t+26 s**
  (mapping's 15 s + teleop's 10 s).
- The t+26 batch also instantiates `nav2_amcl::AmclNode` and `nav2_map_server::MapServer`, so
  **AMCL and map_server run under `slam:=True`** and `/map` has **2 publishers** (`map_server` and
  `slam_toolbox`) — that is bug-027, and it means a map saved during a mapping run may come from the
  stale file rather than from the run.

*Where it is not:* `bringup.launch.py` (~1089–1091) **does** pass `launch_localization:'False'`,
`launch_local_localization:'False'` and `launch_global_localization:'False'` to mapping;
`mapping.launch.py` (~630–632) **does** forward all three to teleop; and `teleop.launch.py:674`
**is** gated on `condition=IfCondition(launch_localization)`. One of those three hops drops the
value — trace it by logging the *performed* value in mapping and teleop, do not guess. Same family
as bug-022 and the launch-config inheritance leak.

*Reproduce with:*

```bash
ros2 launch f1tenth_launch bringup.launch.py slam:=True launch_navigation:=False use_gpu:=False launch_2d_mapping:=True
```

**Pass `launch_2d_mapping:=True` explicitly** — bug-008 is still open: `slam:=True` alone starts no
backend, so there is no `/map`, no `slam_toolbox` node and no save service, silently.

**(d) joy_teleop parameter warnings** — §2, `[!]`, and the record predates the two-heartbeat-command
rework, so it may already be clean. `ros2 node info /joy_teleop` plus a look at the launch log.

**(e) Particle cloud in RViz** — §6, `[!]`. **Attempted 2026-08-27 20:20; still open, but both
blockers are now identified and the doc's own contradiction is resolved.**

1. **The display's QoS must be set to Best Effort.** `nav2_rviz_plugins` *is* installed and the
   display was added successfully — `ros2 topic info /particle_cloud --verbose` showed
   `Subscription count: 1` and that subscriber *was* rviz2. It still rendered nothing, because
   **AMCL publishes `BEST_EFFORT` and the RViz display defaults to `RELIABLE`**, which is an
   incompatible pair: the subscription exists and no message is ever delivered. Read the
   *reliability* of both ends, never the subscriber count — the same lesson as bug-238's durability.
2. **It cannot be closed parked.** `/particle_cloud` was silent across 14 s even after the car was
   carried by hand, because AMCL is gated on *odometry* motion and `odometry/local` moved
   **0.000 m** during the carry: with no VESC the only odom sources are rf2o and VSLAM, and a lifted
   car gives neither a usable scan delta nor VSLAM tracking. **The "publishing half is confirmed
   (8.97 Hz)" note is not wrong, it is just from a run where odom was actually moving** — which is
   what the §1 status line meant by "stays silent parked". Both halves now agree.

   So: add the display via **Add → By display type → nav2_rviz_plugins → Particle Cloud** (the
   generic PoseArray display cannot render `nav2_msgs/msg/ParticleCloud`), **set Reliability Policy
   to Best Effort**, and check it on the drive session, not parked.

*Noticed in passing, not chased:* `odometry/local` was running at **8.4 Hz** rather than the usual
~30 Hz on that launch (RViz was up). Worth a look before trusting a fusion measurement from a run
with visualization enabled.

---

## 2 · Verify this on the first launch, before measuring anything

**bug-245 — code fix landed 2026-08-26, NOT YET VERIFIED ON HARDWARE.** First thing on the next
launch, before any measurement.

The autosave half was already fixed (`save_map_folder_path` is `''` unless `save_map` is true). The
half that bit us was different, and it is worth understanding because **our own seed fix is what
triggers it**: `load_map_folder_path` was handed over *unconditionally*, while
`visual_slam/initial_pose` is remapped to `initialpose` — the topic `localization.launch.py`
deliberately publishes on at `initialpose_seed_delay` (20 s) to seed `ekf_map` and AMCL from one
number (bug-241). cuVSLAM reads that same message as a **relocalization hint**, so a load path plus
that seed starts relocalization even with `localize_on_startup` False.

Measured: `Trying to localize in map … around [0.445, -0.575, 0.000]` — the AMCL seed — then
`Failed to localize in map. Error 3` on repeat, and `visual_slam_container` died **exit −6**, at
t+20 s exactly matching the seed delay.

`load_map_folder_path` is now gated on `localize_on_startup_effective`, the same condition as
`localize_on_startup`, so both map paths are symmetric: no intent, no path.

**VERIFIED 2026-08-27 — PASS on 2 of 2 cold launches.** `/visual_slam/tracking/odometry` at
**30.3 Hz** and **29.99 Hz** several minutes past t+20 s, no exit −6, `visual_slam_node` alive in
`ros2 node list`. The `load_map_folder_path` gate holds. Keep counting across the §1(a) launches —
VSLAM aborts on roughly one launch in three for unrelated reasons, so 2 passes is encouraging, not
conclusive; the §1(a) run gives the sample for free.

**Verify** — VSLAM should survive past t+20 s and `/visual_slam/tracking/odometry` should be
publishing. If it aborts again, move `/mnt/data/maps/nvidia/vslam_map` aside (making `map_exists`
False) and re-check; that isolates whether the gate is working or something else still hands over a
map.

**Why this matters more than it looks — it invalidated a measurement.** With VSLAM dead, `ekf_odom`
lost `odom1` entirely and rf2o degraded to 3.2 Hz, and a parked `yaw_drift` read **+3.77 °/min**
against a +0.04/+0.01/+0.17 baseline measured with VSLAM healthy. It reads exactly like a fusion
regression and is not one. **The tell is the `yaw_drift` source table: check the
`Isaac VSLAM (VO)` row for `0` samples before reading the `odometry/local` number.** VSLAM aborts on
roughly one launch in three and does not respawn, and it fails silently because `odometry/local`
stays at 30 Hz on the remaining sources — so check it by **rate** at the start of every run.

---

## 3 · Drive session, arm A — fusion, steering, sysid

Not startable without battery and a driven leg. Get all of this onto one drive:

- **`odometry/local` yaw drift while moving** — no parked test reaches it. Parked is +0.04 °/min.
- **The wheelbase change 0.25 → 0.256 m** (2026-08-07), still unverified on hardware. It biased
  `vesc_odom`'s kinematic yaw rate ~2.4 % against the frames it fuses into.
- **Does the bias correction improve real driven heading** — the one thing a synthetic source cannot
  answer, and the evidence the `remove_imu_bias` decision waits on. The staleness half is already
  closed offline and is **not** owed.
- **Drive into the steering limit both ways.** All observed saturation was right-turn; `servo_min`
  has never been reached, so **nothing is known empirically about the left bound**. Now that
  `max_steering` is 0.314, neither side should clip — that is itself a thing to confirm.
- **Odometry loop closure over ~5 m** — `testing_checklist.md` §5, `[ ]` since 2026-07-22. The last
  measurement was −24.3° residual yaw at closure with sub-centimetre position closure; the forked
  rf2o and the EKF fixes have both landed since and the retest was never run.
- **Coordinate with Stage 4a sysid.** Do not let a fusion change ride along with a Stage 4a bag —
  `k → 1.0` must be measured against a fusion configuration that is not simultaneously changing.
  Sequence: **Stage 4a first on today's fusion, then the bias work.**

**Record `camera/imu` AND `vehicle/vesc_odom` in the same bag.** A sweep of all 66 bags on gosling1
found exactly one (April 2025) carrying both — each campaign recorded for its own investigation.
That is why the offline bias test needed a synthetic velocity source.

**`remove_imu_bias` stays `'False'`** at both call sites until the driven evidence exists. Only the
stale comment at the RealSense call site was corrected — it had claimed flipping the flag "makes the
launch fail" because the node is not installed. It is installed, source-built into the image.

---

## 4 · Drive session, arm B — Nav2 closed-loop

> ### RESULTS 2026-08-27 — the car drove itself under Nav2. Read this before re-planning arm B.
>
> **What is now done:** goal accepted end-to-end, planner, controller, `twist_to_ackermann`, mux,
> `command_gate` and the VESC all exercised live; costmaps against a live LiDAR; the CPU comparison;
> lifecycle-ACTIVE gating; bug-231 closed; bug-232's guard confirmed engaged.
>
> **Best run** (`nav2_drive4`, goal (−3.902, −2.459) yaw −178.85°):
>
> | | |
> |---|---|
> | Commanded drive | 10.10 s, 203 non-zero commands |
> | Commands reaching the VESC | **203 of 203** (R1 held throughout) |
> | Path / net displacement | **5.781 m** / 4.555 m |
> | Speed | 0.500 commanded, `odometry/local` peak **0.622 m/s** |
> | Final error | **0.379 m** (tol 0.25) and **4.8°** (tol 14.3°) |
>
> **It never formally reached a goal.** Heading is comfortably inside tolerance; position stops
> ~0.38 m short. Two candidate causes, neither confirmed — do not "fix" either blind:
> * The last command before it quit was **0.269 m/s**, against this car's measured ground breakaway
>   of **0.20–0.26 m/s**. RPP decelerates straight into the speed below which the car cannot move.
>   Carry this in ERPM, not m/s, if `speed_to_erpm_gain` is ever recalibrated.
> * `nav2_params.yaml` `movement_time_allowance: 100.0` (default 10.0). Nav2 therefore sits
>   commanding zero for 100 s instead of failing and running a recovery. **Restoring 10.0 is the
>   single highest-value next experiment**: it tests the stall fix *and* delivers the never-observed
>   "`BackUp` reached by the BT itself".
>
> **SAFETY — a real hazard found the hard way.** After the stall, Nav2 keeps the goal active and
> keeps replanning. On the previous run it decided to **reverse at 0.5 m/s**. With R1 held that
> would have driven the car backwards with no warning. **Cancel the goal before relaxing**, do not
> just release R1:
> ```bash
> ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal \
>   "{goal_info: {goal_id: {uuid: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}, stamp: {sec: 0, nanosec: 0}}}"
> ```
> An all-zero UUID cancels every active goal. Humble has no `ros2 action cancel` CLI.
>
> **BLOCKER for repeat runs — bug-257.** `f1tenth_container` aborts with a silent SIGABRT **while
> executing a goal** (1.9 s in on one run, 7.2 s on another; never idle). It did **not** fire across
> two full runs with **`use_composition:=False`**, which is the strongest lead but is n=2, not proof.
> Until it is understood, run Nav2 diagnostics non-composed — each server is then its own process and
> the launch system names the one that dies. Expect higher CPU; that run's performance is not
> representative.
>
> **Two diagnosis traps this cost a session to learn.** A goal clicked before the servers are ACTIVE
> is discarded silently (bug-126); a goal clicked after the container aborted looks *identical* —
> both give no plan, no `cmd_vel`, and nothing in any log. Check `ros2 node list | grep bt_navigator`
> **first**. And `ros2 topic info /goal_pose` reporting `Subscription count: 1` proved to be a
> `ros2 topic echo` of my own, not `bt_navigator` — use `--verbose` and read the node *name*, never
> the count. Same lesson as bug-238.
>
> **Still open in arm B:** obstacle avoidance (no obstacles were placed), a formally SUCCEEDED goal,
> `BackUp` reached by the BT, and `map_frequency`.
>
> **`min_turning_r: 0.462`** (`nav2_params.yaml:185`) is stale — its own comment derives it from 30°
> of steering, and this car has 18.0° (0.314 rad), giving 0.788 m. It is currently **inert**, because
> it lives under `AckermannConstraints`, which belongs to MPPI, and the active plugin is RPP. Fix it
> before anyone switches controllers. Related: steering hit the ±0.314 limit in **12 %** of commands,
> so the planner is routinely asking for turns the car cannot execute.


### 4a · Parked session, 2026-08-27 evening — what a car with no battery settled

Run on AC only: **no drive battery, no VESC** (`/dev/sensors/vesc` absent, `vesc_driver_node`
respawning harmlessly), **no joystick**. Nothing could actuate — `/vehicle/ackermann_cmd` carried
**0 messages** against 2678 on `/drive`, because the `command_gate` never saw a heartbeat. That is
the whole safety argument for doing this parked, and it held.

**bug-257 — ROOT-CAUSED AND FIXED (`aced708`).** Not a Nav2 bug, not a config fault, nothing to do
with the vehicle. The container's dying words were never lost, only never captured:

```
terminate called after throwing an instance of 'std::runtime_error'
  what():  Taking data from action client but no ready event
process has died [pid 324, exit code -6 ... __node:=f1tenth_container]
```

That is **ros2/rclcpp#2242** — under a multi-threaded executor, `is_ready()`, `take_data()` and
`execute()` for an action **client** can run on different threads out of order while the pimpl
assumes they are serial. `bt_navigator` is an action client of the controller/planner/behavior
servers *in the same container*, which is exactly why it fired only while a goal was executing and
never idle. The upstream fixes (#2250, #2495) never reached Humble. The trigger is
`--use_multi_threaded_executor`, **not composition** — that flag is now the `container_multi_threaded`
launch argument, default `False`. `component_container_isolated` already gives each component its
own executor thread, so this costs concurrency only *within* a component.

| | with the flag | without it |
|---|---|---|
| Parked goals | died 3.1 s in | 3 goals × 70 s, 2 launches, **0 aborts** |
| Driving goals (earlier) | died 1.9 s, 7.2 s in | — |
| Container log | 16384 B, truncated mid-word | 24–59 kB, still growing |

**How it was finally read, and the lesson worth more than the bug:** redirect `ros2 launch`'s own
**stdout** to a file. The launch system pipes each process's stdout through with a `[<process>-N]`
prefix, so the C++ runtime's `terminate called` message lands there — while `$ROS_LOG_DIR`'s
container log truncates at 16384 bytes and `launch.log` never sees it at all. Three sessions
searched the truncated log; the redirect gave the exception on the first attempt.

**`movement_time_allowance: 10.0` — VERIFIED ON HARDWARE.** With the car unable to move, the
`SimpleProgressChecker` failed `FollowPath` at **10.01, 10.22, 10.01, 10.02, 10.03 s** — the
parameter is on the robot and it bites at 10 s, not 100 s. Parked is arguably the *better* test of
the parameter, since there is no 0.269 m/s breakaway confound; what it cannot judge is whether 10 s
is too tight for a slow real approach. Arithmetic says there is an order of magnitude of headroom:
`required_movement_radius: 0.5` in 10 s is an average of **0.05 m/s**, against 0.5 m/s commanded.

**Why it had been 100.0**, recovered from the author 2026-08-27: it was buying *operator* setup time
on a Jetson TX2 — a goal sent before R1 is held makes no motion and aborts in 10 s. That is the
checker working, and it is separable from approach time. Confirmed against the Humble source:
`progress_checker_->reset()` runs in `ControllerServer::computeControl()` at goal acceptance and
`baseline_time_` is stamped at the *first* `check()` inside the control loop, so **bringup startup
time is never counted**. The fix is to hold R1 before sending the goal, not to widen the window.

**`BackUp` reached by the BT itself — OBSERVED, first time ever.** From `/behavior_tree_log` on
`bag_parked2_nocomp`: `FollowPath RUNNING→FAILURE` ×10, and the recovery ladder ran in order —
`ClearingActions` (local + global costmap clears) → `Wait` (5.0 s) → **`BackUp` IDLE→RUNNING→FAILURE**
at t+50.90 s. `BackUp` failing in 30 ms is correct here: the car cannot move. This closes the
never-observed item in `testing_checklist.md`.

**Cold launch #7 also passed the bug-241 seed** — `map→base_link` = (0.445, −0.583, **−84.500°**),
and VSLAM held 29.95 Hz (bug-245). Both under `_lo` + domain 42.

**One new observation, not chased:** `visual_slam_container` died with **exit −11 (SIGSEGV)** during
the third launch — a segfault, distinct from bug-257's `-6`, and consistent with the standing
"VSLAM aborts roughly one launch in three and does not respawn". Nav2 was unaffected. Bags and logs
in `/mnt/shared_dir/claude_bringup_0827/` (`log_parked1` … `log_parked4_committed`).

**Still open in arm B, and all of it needs the battery:** the 0.379 m stall, obstacle avoidance, a
formally SUCCEEDED goal, and `map_frequency`.

---

**Everything left in Nav2 is closed-loop.** Offline is finished and passed: goals accepted in 1–2 ms,
first plan in 0.00–0.05 s against the 5 s `max_planning_time`, `cmd_vel` continuous at 20 Hz bounded
to ±0.5 m/s, recovery subtree fires and completes, nothing crashed or pegged a core
(`NAV2_OFFLINE_RESULTS.md`, 2026-08-06 bag replay). **That replay drives the pose**, so `SUCCEEDED`
there only means the recorded trajectory passed inside the goal tolerance.

**The live record is one goal.** 2026-08-06 21:47: a −44.8° right turn over 2.628 m, run as the
bug-140 steering-sign verification. *(`NAV2_OFFLINE_RESULTS.md`'s "the car has still never driven
under Nav2" was written six hours before that run and is stale.)* The 2026-08-10 demo attempt never
got a clean goal — `Robot is out of bounds of the costmap` (bug-228) and the rotated-frame report
(bug-231).

**Run §1(a) first.** A bad `map→odom` is exactly what produced the out-of-bounds costmap, so an
unseeded `ekf_map` makes every Nav2 result meaningless.

Then, on the drive:

- **A goal end-to-end, closed-loop.** `testing_checklist.md` §9. This also re-tests, as a set, four
  fixes that have never run together with Nav2: bug-232 (`localize_isaac_vslam_on_startup` defaults
  False), bug-237 (all six map-load paths → `data/maps/20260805`), bug-234 (AMCL yaw seed
  −1.3928 → −1.4748), bug-241 (auto-seed).
- **`launch_twist_to_ackermann:=True`.** It defaults **False** because LUCIO's ego-MPC publishes
  `drive` directly — so with the default, Nav2's `cmd_vel` reaches nothing and the car will sit
  still while every server reports a healthy plan. Its `cmd_vel → drive` conversion has never been
  exercised live. **Confirm the container carries the `atan` fix before trusting a turn direction**
  (`/mnt/shared_dir/apply_twist_fix.sh <container>`, idempotent) — `/workspaces` is a container
  layer, so a fresh container starts from the image's pre-fix source.
- ~~**`BackUp` reached by the BT itself**, inside a goal window.~~ **DONE 2026-08-27 parked** — see
  §4a. The recovery ladder ran `ClearCostmaps → Wait → BackUp` on a progress-checker failure.
- **Costmaps against a live LiDAR** rather than a replayed one.
- **A clean CPU comparison** — §9. Sample with `top -b -n2 -d2` and parse **only the second block**;
  a single-shot `top -b -n1` reports cumulative-since-start and is what produced the discredited
  "94 % while stationary" figure (bug-127). Expect ~100 % of one core during `on_configure` costmap
  build and 2.5–5.5 % while planning.
- **Decide `map_frequency`** — the launch default is 10.0 Hz; §6 of `LOCALIZER_FOLLOWUPS.md` was
  measured at 30.0. `60_nav2_test.sh` already pins `map_tf_publisher:=ekf`.

**Nav2 and the MPC need separate bring-ups** — both would publish `drive`, and `twist_to_ackermann`
would double-publish it. Two launches per session, one shared bag.

**Goals must be poses.** `maps/*/truth_<bag>.csv` is the `map→odom` transform; goals drawn from it
land on top of the robot (bug-128). Use `scripts/live_runs/goal_poses_from_bag.py`.

**Gate on lifecycle ACTIVE, not on topics.** `bt_navigator`, `planner_server` and `controller_server`
expose their action servers and topics at CONFIGURE, and `nav2_util::SimpleActionServer` rejects
goals while inactive **without logging anything** (bug-126). Poll `<node>/get_state`.

**bug-231 — confirm or close it here, do not re-investigate it.** "The car, its footprint and the
scans look rotated inside the costmap" is almost certainly bug-232 seen from the operator's chair
(`ekf_map` fusing un-relocalized VSLAM as an absolute map anchor), and every contributing cause has
been fixed since. It is logged `NOT FIXED` only because no live Nav2 run has happened since
2026-08-10. If the frames are right on this run, close it.

---

## 5 · What closed 2026-08-26 evening

**bug-251 — the `imu_bias_remover` staleness hazard. Fixed, not worked around.**
`privvyledge/imu_pipeline` @ **`humble-devel`** (upstream tag `0.5.2` + commit `58d227e`) adds a
**`stationary_timeout`** parameter. A velocity source silent longer than the timeout has its
"stationary" verdict dropped and the node falls through to subtracting the last converged bias
instead of zeroing. **Default `0.0` reproduces stock behaviour exactly**, so it is a candidate
upstream PR and inert until a config asks for a timeout. Set to **0.5 s** in
`config/filters/imu_bias_remover.yaml`.

Verified on the same bag, same harness, same scorer, only the parameter differing:

| `stationary_timeout` | after the velocity source dies |
|---|---|
| `0.0` (stock) | **3996 samples pinned at exactly 0.0**, raw gyro live to 0.037 rad/s |
| `1.0` (fork) | pinned **0.986 s** (195 samples), then all **3801** remaining samples exactly `raw − bias`, max residual **0.0** |

The absence of the timeout was confirmed in **three** independent places before forking, because a
previous session remembered otherwise: the source-built 0.5.2 read end to end (no `rclcpp::Time`,
no timer), the **apt 0.4.1** `.deb` (the string `timeout` appears in no file in the package), and
upstream's `ros2` branch at **0.6.1** (same condition, same six parameters). apt Humble is still
0.4.1, so **apt remains a downgrade**; `docs/build_repo_requirements.md` now asks for a `.repos`
entry instead of the SSD tarball.

**Parked wiring test — PASS.** Chain live at 195.6 / 206.1 / 183.6 / 201.1 Hz
(`camera/imu` → `bias_removed` → `filtered`, plus `bias`), `stationary_timeout` read **0.5** off the
live node, `/camera/imu` showed Publisher 1 / **Subscription 1**. The node was additionally proven a
**bit-exact no-op** for that run: 3000 matched sample pairs, max difference across all three gyro
axes **`0.000e+00`**. Expected — with the joystick disconnected `vehicle/vesc_odom` never publishes,
so the flag never latches.

**`max_steering` 0.34 → 0.314** at all four entry points, and `twist_to_ackermann`'s
`max_steering_angle` **0.25 → 0.314**, so all three steering limits agree with the measured
mechanics. `LUCIO_REPLY.md` carries dated inline corrections where it claimed on 2026-08-08 that this
was already applied. **Nothing on LUCIO's side changed** — their ego-MPC publishes `drive` directly.

**Four stale records corrected 2026-08-26** (see §6 for why this keeps happening): buglog `bug-244`
still read `NOT FIXED` a day after it was closed; `bug-232` still read "not yet applied" for a fix
applied 2026-08-11; `bug-009`'s 94 %-CPU figure was superseded by `bug-127`; and
`testing_checklist.md` §4 claimed `intensity: true` "is now set" in `ydlidar_X4.yaml`, which was
never true of the committed config (it ships `false`, and must).

---

## 6 · Restoring the robot — everything below is ephemeral

The container **dies with the terminal that started it**, and `/workspaces` is a container layer.
The operator starts it from the Jetson desktop session:

```bash
bash ~/bolus_ws/f1tenth_launch.sh
```

Then, from the host or inside the container:

```bash
bash /mnt/shared_dir/stage_0830.sh             # ~1 min (omit FLIP=1: that is the
                                               # temporary remove_imu_bias wiring test)
```

**`/mnt/shared_dir` is the in-container path. On the host it is `/mnt/f1tenth_ssd/shared_dir/`** —
there is no `/mnt/shared_dir` on the Jetson itself, so the command above only runs inside the
container (or via `docker exec`). Corrected 2026-08-27 after a session lost minutes to it.

**`stage_0830.sh` is current** (2026-08-30). It carries **`f1tenth_stage_20260830.tgz`**
(md5 `137531c39628609afe260cb19fba27c0`, `git archive HEAD` at **`e07c2e1`** over the same 180-entry
path set as 0827b) and verifies **twelve** values in the installed tree — the twelfth being the
bug-260 `IfCondition(launch_localization_string)` gate. Verified end to end on the container
2026-08-30: 12/12. **Re-staging `0827b` or earlier silently reverts the bug-260 fix.**

The superseded lineage, for the record: `stage_0827b.sh` carried
**`f1tenth_stage_20260827b.tgz`** (md5 `e61a8151d0d074f7b581808d41b1e912`, from `aced708`);
`stage_0827.sh` superseded `stage_0826c.sh` (which superseded `stage_0826.sh`) and carried
**`f1tenth_stage_20260827.tgz`** (md5 `9c1c6b66fb8531e8fb4c48e917908655`, cut from git HEAD
`614c463` with `git archive` over the same path set as 0826c — it carries the bug-245 gate,
`movement_time_allowance: 10.0` and the `nav2_goal_probe.py` live-robot fix), clones the
`imu_pipeline` fork and rebuilds `imu_processors`, and verifies **nine** values in the **installed**
tree. `FLIP=1` applies a temporary `remove_imu_bias:='True'` for wiring tests; **omit it** for the
committed `'False'`. **Re-staging any earlier script silently reverts later work** — 0826c
predates `movement_time_allowance`, and everything before 0830 predates the bug-260 fix.

Two live checks worth running after any stage:

```bash
ros2 param get /realsense_imu_bias_removal_node stationary_timeout   # 0.5 -> the fork is running
ros2 param get /realsense_imu_filter constant_dt                     # 0.005 -> bug-248 fix present
```

### The launch environment — set these, they are not in the container env

The container ships `CYCLONEDDS_URI` pointing at `cyclonedds_config_static.xml` and **no
`ROS_DOMAIN_ID` at all** (= domain 0). Neither is what a run should use. Every launch, and **every
`docker exec` shell that checks it**, wants:

```bash
source /workspaces/f1tenth/install/setup.bash                            # bug-254: not in ~/.bashrc
export ROS_DOMAIN_ID=42
export CYCLONEDDS_URI=file:///mnt/shared_dir/cyclonedds_offline_lo.xml
export ROS_LOG_DIR=/mnt/shared_dir/claude_bringup_<date>/log_run<n>       # never the Jetson home
```

**A check shell on the wrong domain or URI sees an empty graph and reads as a dead stack.** This is
the same trap as suppressing stderr on `ros2 topic hz`: a configuration mistake that presents as a
negative measurement.

Why `_lo` and not the container default, measured 2026-08-27: both configs already carry `lo` at
priority 10, so the loopback fix behind the VSLAM-jitter result is in **both** — the difference is
that the static config also binds `wlP1p1s0` and lists four remote robots as static peers, and on
that day **all three reachable-in-principle peers pinged ABSENT**, costing ~200 failed `sendto`/sec
per process across 41 nodes. `_lo` drops that. RViz is unaffected: it runs same-host and reaches the
operator through SSH X11 forwarding, not through DDS. Pair `_lo` with a non-zero domain — its own
header says so, and domain 0 is where a second agent's stack would collide.

**Keep this constant across a multi-launch acceptance.** bug-238 is a discovery-timing race, so the
DDS config is not neutral with respect to it; a five-launch seed acceptance split across two configs
is not a homogeneous sample.

The `stationary_timeout` check is the only reliable proof of the fork: **stock `imu_processors` does not declare
`stationary_timeout` and silently ignores it** — the node starts and logs nothing — so a stock build
reads as configured and is not. That query errors on stock.

---

## 7 · Verify these before planning around them

This handoff family has repeatedly carried blockers that were stale, and each has cost a session.
**Six have now been caught.** From 2026-08-26: `RESUME_20260827_DECISIONS.md` §10 said two commits
were local and unpushed (already pushed); a session note claimed the stale steering-gain comment at
`vehicle.launch.py:320` had been fixed (it had not); and the four records in §5 above. Earlier
examples: *"`imu_processors` is at 0.4.1"* (the image builds 0.5.2 from source), *"the driving half
of the bias check is owed to a drive session"* (closable parked with a synthetic source), *"the
Jetson has no internet"*, *"root fs at 96 %"*.

Spend the first minutes checking each named blocker against the machine — `git log @{u}..HEAD`,
`apt-cache policy`, `git status`, read the shipping source — then correct the doc in place, in the
same commit as the work.

---

## 8 · Traps that have each cost a run

- **A silent RealSense has two unrelated causes.** Count kernel re-enumerations:
  `dmesg | grep -c "Found UVC 1.50 device"`, wait 20 s idle, count again.
  *0 new while idle and the device fails to enumerate* → wedged (bug-014/170), fix in software with
  a USB unbind/rebind at its port from inside the privileged container. *0 new while idle but a
  burst of 4–16 every time anything opens it — a bringup, or even a plain `rs-enumerate-devices`* →
  **physical**, reseat the cable (bug-255). On 2026-08-26 the second case cost two full launch
  cycles: the unbind/rebind appeared to work every time (device back at 5000 Mbps, sysfs perfect,
  librealsense reading serial and firmware) and the next launch failed byte-identically. A replug at
  the camera end took launch re-enumerations from 16+ to 3.
- **`package 'f1tenth_launch' not found, searching: ['/opt/ros/humble']`** is an unsourced overlay,
  not a failed build (bug-254). The image's `~/.bashrc` does not source
  `/workspaces/f1tenth/install/setup.bash`. The searched-paths list in the error is the diagnostic.
- **`ros2 topic hz` does not accept `--qos-reliability`** — that is an `echo` flag. With stderr
  suppressed the usage error reads as "NO DATA" on a topic that is streaming at 200 Hz. Do not
  suppress stderr on a check whose negative result you intend to act on.
- **Do not use `ros2 topic hz` to check whether `command_gate` is open** either — a closed gate
  publishes at full rate with zero payloads. Echo the values.
- **`ros2 launch` does not reliably die on Ctrl-C here** (bug-249). It takes most children and
  leaves the launch process and one container. List with
  `ps -eo pid,args --no-headers | grep "[c]omponent_container"` and kill **by PID** — `pkill -f`
  inside `docker exec bash -lc` kills its own shell, because the shell's command line contains the
  pattern. A teardown pattern matching only node names missed 11 orphans once, and the resulting
  load average starved rf2o to 2.5 Hz and the LiDAR to 6.5 Hz — **low-but-nonzero sensor rates can
  be CPU contention, not a sensor fault; check `uptime` first.**
- **`yaw_drift.py` takes its namespace from `$VEHICLE_NAME`**, which is set in the container, while
  bringup defaults `use_f1tenth_namespace:=False`. Export **`F1TENTH_NS=""`** or every row reports
  zero samples.
- **Never `set -u` in a script that sources `/opt/ros/humble/setup.bash`** (bug-250) — it reads an
  unbound `AMENT_TRACE_SETUP_FILES`, exits before launching anything, and **exits 0**.
- **A parked yaw-drift figure cannot tell you whether the bias correction works**, because the
  zeroing path is active and the subtraction path is not. Log the `bias` topic instead.
- **Read absolute `odom→base_link` on a parked car, not drift** (bug-231). `tf_chain_watch.py`
  reported displacement from its *first* sample and called a 1.25 m / 62 deg offset healthy.

---

## 9 · Environment notes still current

- **X11 works through the operator's SSH forwarding**, not `:0`. On 2026-08-26 the container had
  `DISPLAY=localhost:11.0` and `xdpyinfo` succeeded there. Probe with `xdpyinfo -display <d>` before
  assuming a display problem.
- **DualSense** `10:18:49:9D:72:FC` is paired and trusted but normally left **disconnected**.
  Connected, it publishes the heartbeat, `command_gate` opens, `vesc_odom` starts at ~50 Hz — **and
  the car can drive.** Note the bias remover only leaves passthrough once `vesc_odom` is alive.
- **`command_gate_require_heartbeat:=False` does NOT hold the gate shut** — it collapses the logic to
  always-open. To keep it closed, leave the default `True` and disconnect the joystick.
- CycloneDDS spams `ddsi_udp_conn_write … retcode -3` at absent peers. Harmless, but it will drown a
  `ros2 param get` — pipe through `grep -v ddsi`. **The right fix is `cyclonedds_offline_lo.xml`,
  not the grep** — see the launch-environment block in §6. `RMW_IMPLEMENTATION` and a
  `CYCLONEDDS_URI` pointing at the *static* config are set in the container environment and are
  inherited by `docker exec` shells; `ROS_DOMAIN_ID` is **not set**, so an unexported shell is on
  domain 0. Both need overriding per shell.
- Docker's default **bridge** network is broken on this kernel — `docker run` needs `--network host`.
- `jetson-containers` bind-mounts `/tmp/argus_socket`, a *socket* on R36.4.3 but a *directory* in the
  image; **a fresh checkout reintroduces this** (bug-242).
- Bags: `claude_bringup_0826/run1{4,5,6}` (pre-bug-248-fix, raw **and** filtered camera IMU),
  `run17_constdt` and `run18_constdt` (post-fix). Offline bias data `biastest_0826/bias_test.csv`
  (stock) and `biastest_timeout/bias_test.csv` (fork, `stationary_timeout` 1.0).

---

## 10 · `testing_checklist.md` — the 9 open items and where each is handled

| § | Item | Marker | Where |
|---|---|---|---|
| 1 | Acceleration limit / throttle interpolator | `[!]` | **No longer a won't-fix.** The operator reports it was fixed (mux + joystick), but no record of the fix exists — see item 3 of the ★★ block. Re-test parked, no battery. |
| 2 | joy_teleop starts without warnings | **`[x]` 2026-08-27** | Zero warnings; the `[!]` was stale |
| 5 | Odometry loop closure over ~5 m | `[ ]` | §3 — drive. **Note 2026-08-27**: `nav2_drive4` holds a 5.781 m driven path with `odometry/local`, `vesc_odom`, VSLAM and rf2o all recorded — that bag may close this offline with no further driving, if a tape measurement of the start/end marks can be recovered. It was NOT taped, so treat it as a candidate, not a result. |
| 6 | Particle cloud visible in RViz | **`[x]` 2026-08-30** | Closed with the car rolled by hand, no battery. Display now in `config/f1tenth.rviz` at Best Effort. Convergence under *sustained* motion still unproven |
| 8 | VSLAM frame-delta warnings at high playback rates | `[!]` | **Next session, item 1.** The "replay artefact, no live action" verdict is a prediction that was never re-checked after the `use_gpu` fix — verify at 1x before closing |
| 9 | Send a navigation goal | **`[x]` 2026-08-27** | Drove 5.781 m under Nav2; stopped 0.379 m short of tolerance. See §4 RESULTS |
| 9 | CPU load during navigation | **`[x]` 2026-08-27** | No Nav2 CPU problem; the container is under ~3.5 %. See §4 RESULTS |
| 10 | Teleop mode: full stack launches | **`[x]` 2026-08-27** | Full TF chain resolves, 41 nodes |
| 10 | Mapping mode: no TF conflicts | **`[x]` 2026-08-30** | bug-260 fixed (`TimerAction` outliving its `GroupAction` scope). 0 duplicates, `/map` single publisher |

Update the markers in `testing_checklist.md` as each closes, in the same commit as the work.
