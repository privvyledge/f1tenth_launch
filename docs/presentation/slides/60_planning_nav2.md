<!--
S60 Planning and Nav2

Section file for the F1TENTH deck. Slides are separated by a line containing
only `---`. Every slide carries one cut tag; a slide with no tag is
reference-only and appears in `build.sh full` alone.

Conventions (plan §1) - the build enforces the first three:
  - a placeholder card's ID must be listed in ASSETS.md, and an asset marked
    DONE there must be embedded, not carded;
  - no slide body may name an agent file or a bug ID (speaker notes may);
  - every number on a slide needs a note of the form "src: FILE; measured DATE"
    in an HTML comment on that slide;
  - the title is the claim the slide makes, not the topic;
  - rates, latency, resolution and defaults/min/max go in tables, not prose.

Owner: shared - rows 6.1/6.3/6.4/6.6 are the launch chat's, 6.2/6.5 the control chat's.
Plan rows for this section are quoted above each slide, verbatim from §3.
-->

<!-- cut: lab sponsor research -->
<!-- _class: cols -->
<!-- plan §3 row 6.1 | owner: launch -->

## Planning is two loops at two rates: a route once a second, a trajectory ten times a second

<div class="split">
<div>

![w:520](../assets/figures/arch/3_Planning.png)

</div>
<div>

- **Route planning, 1 Hz** — a global path over the static map, replanned on a timer rather than continuously
- **Behaviour and local planning, 10 Hz** — what the vehicle actually follows, and the only loop the controller sees
- **Sparse vs dense waypoints** — a route is a handful of poses; a trajectory is a dense sequence with a speed at every point

The split matters because replanning is expensive and tracking is not: the global path can afford to be stale, the local one cannot.

</div>
</div>

<!-- src: figure exported from the Obsidian `3_Planning` note, received 2026-09-02; rates from config/nav2_params.yaml (BT RateController 1.0 Hz, controller_frequency 10.0), read 2026-09-02 -->

---
<!-- cut: lab research -->
<!-- _class: cols dense -->
<!-- plan §3 row 6.2 | owner: control -->

## A route is poses plus a **signed** speed column — the sign is what makes reversing expressible

<div class="split">
<div>

![w:560](../assets/figures/waypoints.png)

</div>
<div>

| | |
|---|---|
| Recorder | odometry every `save_interval`, into the target frame |
| Columns | `frame_id, total_time_elapsed, dt, x, y, z, yaw, q*, vx, vy, speed, omega` |
| Loader publishes | path + speed array + markers, all **latched**, so a controller that starts later still gets the route |

**`speed` is a magnitude; `vx` carries the sign.** The loader restores direction from `vx`, with a 0.05 m/s deadband so a noisy standstill does not dither. A bare path message has no speeds at all — so **it cannot express a reversing manoeuvre**, and every route here ends in one.

**Frame is a real trap.** An `odom`-frame route is valid only in the session that recorded it — `odom` is fixed at estimator start-up, so a later session drives the file **rotated**, measured at 45.7° here, with every diagnostic still reading healthy. Replay the `map`-frame copies.

</div>
</div>

<!-- src: column list and deadband from waypoint_recorder.py / waypoint_loader.py (reverse_speed_threshold 0.05); route figure plotted 2026-09-03 from data/gosling1_loop_laps.csv (834 points, 37.0 m, 55 reverse) and data/gosling1_figure8.csv (868 points, 39.4 m, 51 reverse), trajectory_following_ros2 branch refactor/unify-backends. The 45.7 deg rotation was measured on gosling1 2026-08-06. -->
<!-- Cut tag widened from `lab` to `lab research` (the scaffold left a NOTE asking the owner
     to confirm). Deliberately NOT added to `sponsor`: plan 2a drops format/config detail
     from that cut, and the sponsor count is already over target. -->

---

<!-- cut: lab sponsor research -->
<!-- _class: dense cols -->
<!-- plan §3 row 6.3 | owner: launch -->

## Nav2 runs a hybrid-A* planner and a pure-pursuit controller, and any server can be swapped out

<div class="split">
<div>

![w:540](../assets/figures/nav2_bt.svg)

</div>
<div>

| Server | Plugin | Rate |
|---|---|---|
| Planner | Smac Hybrid-A*, Reeds-Shepp | 1 Hz |
| Controller | Regulated Pure Pursuit | 10 Hz |
| Local costmap | LiDAR obstacle + inflation | 5 Hz |
| Global costmap | static + obstacle + inflation | 1 Hz |
| Velocity smoother | open-loop, onto `cmd_vel` | 20 Hz |

Recovery ladder: **clear costmaps → wait 5 s → back up 0.30 m**. No `Spin` — this car cannot rotate in place.

**`twist_to_ackermann`** turns the smoother's `Twist` into a steering command. **Off by default**, because the MPC publishes `drive` directly — enable it for a Nav2 run.

**Any server can be disabled**; replacing the controller with an MPC means implementing `FollowPath`.

</div>
</div>

<!-- src: config/nav2_params.yaml (GridBased/SmacPlannerHybrid, FollowPath/RegulatedPurePursuitController, controller_frequency 10.0, local costmap update 5.0, global 1.0, smoothing_frequency 20.0), config/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml, launch/nav2_navigation.launch.py; read 2026-09-02 -->
<!-- KNOWN DEFECT, deliberately not on the slide (bug-274): this planner's
     `minimum_turning_radius` is 0.462 m, but the car's true minimum at its
     +/-0.314 rad steering limit is 0.788 m, and 0.814 m driven. So it plans
     turns the car cannot execute - 12 % of commands hit the steering limit on
     the 2026-08-27 drive. Changing it alters planner behaviour and needs a
     drive to validate; see the Open items in ASSETS.md. -->

---

<!-- cut: lab sponsor research -->
<!-- _class: dense -->
<!-- plan §3 row 6.4 | owner: launch -->

## Nav2 drives the car, and stops 0.38 m short of the goal

| Best run, of four goals over three launches | Value | Tolerance |
|---|---|---|
| Commands issued, and reaching the VESC | 203 of 203, over 10.10 s | — |
| Path driven / net displacement | 5.781 m / 4.555 m | — |
| **Final distance to goal** | **0.379 m** | 0.25 m |
| Final heading error | 4.8° | 14.3° |
| Last commanded speed | 0.269 m/s | breakaway 0.20–0.26 m/s |

**Heading converges; position stops 0.38 m short** — pure pursuit decelerates into the band below which this car cannot move. **After a stall the goal stays active and the car was measured deciding to reverse at 0.5 m/s** — cancel it. **Obstacle avoidance is untested.**

> [!PLACEHOLDER CHART-NAV2-APPROACH]
>
> Distance-to-goal and commanded speed against time: the stall, in one figure.

> [!PLACEHOLDER VID-NAV2-LIVE]
>
> The goal sent, the car driving it, and the stall. Needs a driving session.

<!-- src: CLAUDE.md §Nav2 and the nav2_drive4 bag on the SSD; driven on gosling1 2026-08-27. Tolerances from config/nav2_params.yaml (xy_goal_tolerance 0.25, yaw_goal_tolerance 0.25 rad = 14.3 deg), progress checker movement_time_allowance 10.0 restored the same day. -->
<!-- VID-NAV2-LIVE stays uncarded here: the chart is the honest gap. The video is listed in ASSETS.md for the capture session. -->

---
<!-- reference-only: no cut tag (plan §3 marks this R) -->
<!-- _class: dense -->
<!-- plan §3 row 6.5 | owner: control -->

## Two go-to-goal paths: Nav2 is sent a goal, the MPC is only ever sent a path

| | Nav2 | MPC node |
|---|---|---|
| Interface | `NavigateToPose` action | a path message — **it never sees a goal** |
| Planner | Smac hybrid-A*, Reeds–Shepp | none in the package |
| How a goal is driven | planner → controller server | an external planner must publish the path |
| Replanning | behaviour tree, 1 Hz | accepted when the path subscription is set to non-latched |
| Driven the car | **yes**, 2026-08-27 | **no — simulator only**, 2026-08-08 |

The stand-in used in simulation fits a **Dubins** curve from the *live* vehicle pose to the goal: shortest forward-only path that never turns tighter than `min_turn_radius`, which is the right primitive for a car and turns "the goal is behind me" into a feasible loop rather than an impossible manoeuvre.

**One honest limit: this mode is forward-only by construction.** Direction of travel lives in the signed speed array, and a path message carries no speeds — so a goal needing reverse, in a space too tight for a Dubins loop, is out of reach. That needs cusps and a signed profile, i.e. a richer message than a path.

<!-- src: MPC-side interfaces from base_tracker.py and tools/goal_to_plan.py (plan_type dubins, min_turn_radius 0.5 m default), trajectory_following_ros2 branch refactor/unify-backends, read 2026-09-03; first simulator go-to-goal runs 2026-08-08. Nav2 side from config/nav2_params.yaml and the live run on slide 6.4, gosling1 2026-08-27. -->

---

<!-- reference-only: no cut tag (plan §3 marks this R) -->
<!-- plan §3 row 6.6 | owner: launch -->

## On replay the whole stack is healthy — which proves less than it looks

| Measured on a bag replay | Result |
|---|---|
| Goal accepted | 1–2 ms |
| First plan produced | 0.00–0.05 s, against a 5 s planning budget |
| `cmd_vel` output | continuous at 20 Hz, bounded to the configured ±0.5 m/s |
| Recovery subtree | fires and completes |
| Crashes | none |

**What a replay cannot test**: the bag drives the pose, so `SUCCEEDED` only means the *recorded* trajectory passed inside the goal tolerance. Nothing closed-loop is exercised — which is exactly why the live run in the previous slide found a failure the replay never could.

Two artefacts worth knowing: `local_plan` never publishes, because it is a DWB topic and the configured controller is pure pursuit; and the planner peaks near 100 % of a core during `on_configure` (building the global costmap), not while planning, where it sits at 2.5–5.5 %.

<!-- src: scripts/live_runs/NAV2_OFFLINE_RESULTS.md; measured 2026-08-06 -->
