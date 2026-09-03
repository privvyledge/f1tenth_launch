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

Owner: B2 (f1tenth_launch)
Plan rows for this section are quoted above each slide, verbatim from §3.
-->

<!-- cut: lab sponsor research -->
<!-- plan §3 row 6.1 | owner: launch -->

## Planning pipeline

![w:900](../assets/figures/arch/3_Planning.png)

<!--
REFERENCE FIGURE, NOT FINAL. This PNG is an Excalidraw export the planning chat
worked from; it is queued for replacement by an accurate SVG / mermaid / TikZ
drawing (see Open items in ASSETS.md). Two things it needs on the slide either
way: its rate annotations are DESIGN TARGETS, not measurements, so pair it with
the measured table and say so in one line; and check the crop/attribution notes
for this figure in the register before shipping it.
-->

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: Excalidraw: route planning 1 Hz, behavior/local planning 10 Hz, sparse vs dense waypoints
plan source:  Obsidian `3_Planning`

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- cut: lab -->
<!-- plan §3 row 6.2 | owner: control -->

## Waypoint recording and loading

> [!PLACEHOLDER FIG-WAYPOINTS]
>
> figure not produced yet - see ASSETS.md for how to produce it.

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: `waypoint_recorder.py` / `waypoint_loader.py`: format, frame, how a figure-8 was recorded
plan source:  trajectory_following_ros2
NOTE: plan §2a does not say which cuts this belongs to. Scaffold tagged it `lab` only; the section owner must confirm or widen it.

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- cut: lab sponsor research -->
<!-- plan §3 row 6.3 | owner: launch -->

## Nav2 as configured

> [!PLACEHOLDER FIG-NAV2-BT]
>
> diagram not produced yet - see ASSETS.md for how to produce it.

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: bt_navigator + custom BT XMLs, planner (name from `nav2_params.yaml`), RPP controller 10 Hz, costmaps 5 Hz / 1 Hz, velocity smoother, `twist_to_ackermann`; per-server toggles; MPC can replace `controller_server` through `FollowPath`
plan source:  `nav2_params.yaml`, `nav2_navigation.launch.py`, CLAUDE.md

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- cut: lab sponsor research -->
<!-- plan §3 row 6.4 | owner: launch -->

## Nav2 on the car

> [!PLACEHOLDER CHART-NAV2-APPROACH]
>
> chart not produced yet - see ASSETS.md for how to produce it.

> [!PLACEHOLDER VID-NAV2-LIVE]
>
> video not produced yet - see ASSETS.md for how to produce it.

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: 2026-08-27: 5.78 m path, 203/203 commands reached the VESC, stopped 0.379 m from goal (tol 0.25 m), heading 4.8 deg (tol 14.3 deg); last command 0.269 m/s vs breakaway 0.20-0.26 m/s; progress checker and recovery ladder observed; obstacle avoidance untested
plan source:  CLAUDE.md §Nav2, `nav2_drive4` bag on SSD

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- reference-only: no cut tag (plan §3 marks this R) -->
<!-- plan §3 row 6.5 | owner: control -->

## Go-to-goal: Nav2 vs the MPC node

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: The two go-to-goal paths (Nav2 `NavigateToPose`; the MPC node's goal mode), their interfaces, and which has driven the car (the Nav2 run in 6.4 is the only one documented in this repo; the control chat states the MPC node's status)
plan source:  trajectory_following_ros2, CLAUDE.md

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- reference-only: no cut tag (plan §3 marks this R) -->
<!-- plan §3 row 6.6 | owner: launch -->

## Offline Nav2 replay results

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: goal accept 1-2 ms, first plan <0.05 s, cmd_vel 20 Hz bounded 0.5 m/s
plan source:  `NAV2_OFFLINE_RESULTS.md`

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->

