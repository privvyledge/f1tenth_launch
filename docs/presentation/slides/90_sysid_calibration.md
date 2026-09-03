<!--
S90 System identification and calibration

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
<!-- plan §3 row 9.1 | owner: B2 (f1tenth_launch) -->

## Everything in `vesc.yaml` was inherited from another car

> [!PLACEHOLDER TABLE-CALIB-STATUS]
>
> table not produced yet - see ASSETS.md for how to produce it.

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: Status board: each constant, inherited vs measured, date, method
plan source:  CLAUDE.md §vesc.yaml, `SYSID_RESULTS.md`

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- cut: lab research -->
<!-- plan §3 row 9.2 | owner: B2 (f1tenth_launch) -->

## Method

> [!PLACEHOLDER FIG-SYSID-METHOD]
>
> figure not produced yet - see ASSETS.md for how to produce it.

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: Driven bags with structured excitation; `fit_actuators.py` scores against TF-corrected gyro-z in servo units; three references: rf2o, VSLAM, tape measure
plan source:  `SYSID_RESULTS.md`, `scripts/analysis/fit_actuators.py`

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- cut: lab sponsor research -->
<!-- plan §3 row 9.3 | owner: B2 (f1tenth_launch) -->

## Steering

> [!PLACEHOLDER CHART-STEER]
>
> chart not produced yet - see ASSETS.md for how to produce it.

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: Gain -1.4 to -1.1448 (over-steer 18-23 %), confirmed k = 0.999 / 0.991; offset 0.56 vs fitted 0.5508-0.5583 (contested, bench sweep pending); bounds driven: 17.45 deg L / 17.47 deg R, R_min 0.814 m, 97 % of commanded; `max_steering` 0.314 is the binding limit; left headroom to 24 deg
plan source:  CLAUDE.md, `SYSID_RESULTS.md`

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- cut: lab sponsor research -->
<!-- plan §3 row 9.4 | owner: B2 (f1tenth_launch) -->

## Speed and deadband

> [!PLACEHOLDER CHART-SPEED]
>
> chart not produced yet - see ASSETS.md for how to produce it.

> [!PLACEHOLDER TABLE-DEADBAND]
>
> table not produced yet - see ASSETS.md for how to produce it.

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: 3750 erpm per m/s stands; rf2o -6.1 %, VSLAM +1.3 % over 5.50 m; ground breakaway 0.20-0.26 m/s, deadband table carried in ERPM
plan source:  CLAUDE.md, `SYSID_RESULTS.md` §Deadband

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- cut: lab sponsor research -->
<!-- plan §3 row 9.5 | owner: B2 (f1tenth_launch) -->

## Latency

> [!PLACEHOLDER CHART-LAG]
>
> chart not produced yet - see ASSETS.md for how to produce it.

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: Actuation 60 ms + 40 ms first-order; throttle transport <20 ms; end-to-end joystick-to-servo latency TBD (measurement defined in ASSETS)
plan source:  `SYSID_RESULTS.md`

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- reference-only: no cut tag (plan §3 marks this R) -->
<!-- plan §3 row 9.6 | owner: B2 (f1tenth_launch) -->

## Wheelbase and yaw rate

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: 0.25 -> 0.256 alignment; effect on kinematic yaw
plan source:  CLAUDE.md

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- reference-only: no cut tag (plan §3 marks this R) -->
<!-- plan §3 row 9.7 | owner: B2 (f1tenth_launch) -->

## Bench servo sweep

> [!PLACEHOLDER FIG-BENCH-SWEEP]
>
> figure not produced yet - see ASSETS.md for how to produce it.

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: Planned measurement of centring and `servo_min`/`servo_max`
plan source:  `BENCH_SWEEP_SHEET.md`, `bench_servo_sweep.py`

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->

