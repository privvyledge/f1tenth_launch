<!--
S40 Mapping

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
<!-- plan §3 row 4.1 | owner: B2 (f1tenth_launch) -->

## 2D mapping

> [!PLACEHOLDER FIG-MAP-2D]
>
> figure not produced yet - see ASSETS.md for how to produce it.

> [!PLACEHOLDER VID-MAP-2D-TIMELAPSE]
>
> video not produced yet - see ASSETS.md for how to produce it.

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: SLAM Toolbox (0.05 m, 12 m) vs RTABMap grid (LiDAR `Grid/Sensor 0`, ray tracing); the current `20260805` grid rendered
plan source:  `2d_mapping_*.yaml`, `data/maps/20260805/rtabmap_2d_final.pgm`

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- cut: lab sponsor research -->
<!-- plan §3 row 4.2 | owner: B2 (f1tenth_launch) -->

## 3D mapping

> [!PLACEHOLDER FIG-MAP-3D]
>
> figure not produced yet - see ASSETS.md for how to produce it.

> [!PLACEHOLDER VID-MAP-3D-ORBIT]
>
> video not produced yet - see ASSETS.md for how to produce it.

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: RTABMap RGB-D (CPU) or nvblox (GPU); render of `cloud_voxel_0p05.pcd` in the grid frame
plan source:  `3d_mapping.launch.py`, `data/maps/20260805/cloud_voxel_0p05.pcd`

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- reference-only: no cut tag (plan §3 marks this R) -->
<!-- plan §3 row 4.3 | owner: B2 (f1tenth_launch) -->

## One database, three artifacts

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: Grid, cloud and AMCL seed must come from the same RTABMap database; save commands
plan source:  CLAUDE.md, `COMMANDs.md`

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- reference-only: no cut tag (plan §3 marks this R) -->
<!-- plan §3 row 4.4 | owner: B2 (f1tenth_launch) -->

## RTABMap vs SLAM Toolbox grids

> [!PLACEHOLDER FIG-MAP-COMPARE]
>
> figure not produced yet - see ASSETS.md for how to produce it.

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: Side-by-side stills
plan source:  shot list 3.2

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->

