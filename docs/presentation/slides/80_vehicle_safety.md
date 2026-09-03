<!--
S80 Vehicle interface and safety

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

Owner: B1 (f1tenth_launch)
Plan rows for this section are quoted above each slide, verbatim from §3.
-->

<!-- cut: lab sponsor research -->
<!-- plan §3 row 8.1 | owner: B1 (f1tenth_launch) -->

## Every command passes two gates

> [!PLACEHOLDER FIG-CMDPATH]
>
> diagram not produced yet - see ASSETS.md for how to produce it.

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: Diagram: `teleop`/`drive`/`estop` -> `ackermann_mux` (255/100/10/1, timeouts 0.5/0.3/0.2/0.05 s) -> `ackermann_drive` -> `command_gate` -> `vehicle/ackermann_cmd` -> `ackermann_to_vesc` -> VESC
plan source:  `mux.yaml`, `command_gate.yaml`, CLAUDE.md

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- cut: lab sponsor research -->
<!-- plan §3 row 8.2 | owner: B1 (f1tenth_launch) -->

## Deadman and handover

> [!PLACEHOLDER TABLE-SAFETY-TIMING]
>
> table not produced yet - see ASSETS.md for how to produce it.

> [!PLACEHOLDER VID-SAFETY-MUX]
>
> video not produced yet - see ASSETS.md for how to produce it.

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: Heartbeat on `command_gate/heartbeat`, 0.5 s timeout; L1 manual, R1 autonomous handover; gate closes 0.5-0.63 s after joystick loss; commanded motion bounded to 0.2-0.35 s; never use PS in a deadman set. **Why the heartbeat has its own topic** (one line): the R1 handover silences `teleop` by design, so a `teleop`-based heartbeat would close the gate 1 s into every autonomous run
plan source:  CLAUDE.md, `joy_teleop.yaml`

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- cut: lab -->
<!-- plan §3 row 8.3 | owner: B1 (f1tenth_launch) -->

## Operating procedures

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: Pre-op checklist, power-up order, connection, normal operation, shutdown, emergency stop; battery management
plan source:  Release doc §Operating Procedures, §Battery Management

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- cut: lab sponsor -->
<!-- plan §3 row 8.4 | owner: B1 (f1tenth_launch) -->

## Teleop controls

> [!PLACEHOLDER PHOTO-DUALSENSE]
>
> photo not produced yet - see ASSETS.md for how to produce it.

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: Labelled DualSense figure with speed/steer scales
plan source:  `dualsense_top_with_arrows_and_labels.png`, `joystick.launch.py`

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- reference-only: no cut tag (plan §3 marks this R) -->
<!-- plan §3 row 8.5 | owner: B1 (f1tenth_launch) -->

## Three gate configurations

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: full safety / passthrough / gate removed, and what each implies
plan source:  CLAUDE.md

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- reference-only: no cut tag (plan §3 marks this R) -->
<!-- plan §3 row 8.6 | owner: B1 (f1tenth_launch) -->

## Teardown and orphans

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: why `kill -INT` is not enough; `stop_launch_tree()`
plan source:  CLAUDE.md §Script teardown

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->

