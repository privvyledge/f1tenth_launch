<!--
S10 Hardware

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
<!-- plan §3 row 1.1 | owner: B1 (f1tenth_launch) -->

## Platform and components

> [!PLACEHOLDER PHOTO-COMPONENTS]
>
> photo not produced yet - see ASSETS.md for how to produce it.

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: Table: Jetson Orin Nano Super 8 GB, 1 TB NVMe; VESC 6 MkVI; RealSense D435i; YDLidar X4; Traxxas 4-Tec 2.0 VXL chassis and its steering servo; DualSense. Photo grid
plan source:  Release doc §Hardware Components, `docs/attachments/.../media`

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- cut: lab sponsor research -->
<!-- plan §3 row 1.2 | owner: B1 (f1tenth_launch) -->

## Where every sensor sits: the static frame tree

> [!PLACEHOLDER FIG-TF-TREE]
>
> figure not produced yet - see ASSETS.md for how to produce it.

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: `base_link` = rear axle; offsets from CLAUDE.md frame tree; camera frames from URDF; rendered `view_frames` output
plan source:  CLAUDE.md §Static TF Frame Tree, `static_transformations.launch.py`, `urdf/f1tenth.urdf.xacro`

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- cut: lab sponsor -->
<!-- plan §3 row 1.3 | owner: B1 (f1tenth_launch) -->

## Two batteries, two failure modes

> [!PLACEHOLDER PHOTO-WIRING]
>
> photo not produced yet - see ASSETS.md for how to produce it.

> [!PLACEHOLDER FIG-ELEC]
>
> figure not produced yet - see ASSETS.md for how to produce it.

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: Jetson pack vs VESC drive pack; XT90; powerbank; VESC `voltage_input` is the drive pack only, the Jetson supply has no topic. Electrical diagram placeholder
plan source:  Release doc §Wiring, memory `jetson-and-vesc-are-on-separate-batteries`

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- reference-only: no cut tag (plan §3 marks this R) -->
<!-- plan §3 row 1.4 | owner: B1 (f1tenth_launch) -->

## Assembly

> [!PLACEHOLDER PHOTO-ASSEMBLY]
>
> photo not produced yet - see ASSETS.md for how to produce it.

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: Rollcage, body shell, NPF adapter, sensor mount photos
plan source:  `docs/figures/teleop/*`

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- reference-only: no cut tag (plan §3 marks this R) -->
<!-- plan §3 row 1.5 | owner: B1 (f1tenth_launch) -->

## Network

> [!PLACEHOLDER TABLE-BANDWIDTH]
>
> table not produced yet - see ASSETS.md for how to produce it.

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: Robot WiFi/wired, velox1 as remote RViz host, DDS profiles, remote bandwidth (RViz set ~99 kB/s, one image stream 31 MB/s)
plan source:  Release doc §Network, CLAUDE.md §CycloneDDS

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- reference-only: no cut tag (plan §3 marks this R) -->
<!-- plan §3 row 1.6 | owner: B1 (f1tenth_launch) -->

## Controller pairing and button map

> [!PLACEHOLDER PHOTO-DUALSENSE]
>
> photo not produced yet - see ASSETS.md for how to produce it.

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: BR/EDR pairing; SDL mapping L1=9, R1=10, PS=5; labelled DualSense figure already exists
plan source:  Release doc §DualSense, `docs/figures/teleop/dualsense_top_with_arrows_and_labels.png`

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->

