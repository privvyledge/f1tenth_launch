<!--
S95 Next steps and lessons

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
<!-- plan §3 row 10.1 | owner: B2 (f1tenth_launch) -->

## What is next

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: Nav2 goal reach (0.38 m stall), servo offset bench sweep, obstacle avoidance test on the car, MPC go-to-goal on the car, perception integration into bringup, `servo_min/max` measurement, closed-loop speed enable after recalibration
plan source:  CLAUDE.md, `LOCALIZER_FOLLOWUPS.md`

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!-- reference-only: no cut tag (plan §3 marks this R) -->
<!-- plan §3 row 10.2 | owner: B2 (f1tenth_launch) -->

## Lessons for measuring on this car

<!--
TODO: write the slide. This comment is the plan, not the slide.

plan content: RealSense IMU first stamps and `constant_dt` (attitude jump 80-167 deg to 0.19 deg); rf2o scale error, so never calibrate against one reference; check `odometry/local` twist before trusting a pose; verify streams by message count, not rate
plan source:  CLAUDE.md

Title must state the claim this slide makes; rewrite it if the plan's
wording is a topic. Put rates/limits/defaults in a table.
Add one note per number:  src: FILE; measured YYYY-MM-DD
-->


---

<!--
S99 Research questions (research cut only). Plan §3, verbatim:

### S99 Research questions (research cut only; T 3-4)

Each owning brief contributes one or two candidate questions from its area, phrased as a question the platform can answer with an experiment we can run. Seeds from this repo, for the owners to accept, reject or replace:

| # | Slide | Candidate question | Evidence already in hand |
|---|---|---|---|
| 11.1 | What this platform can measure | One slide: the sensor set, the three independent ground-truth references (tape, VSLAM, RTABMap loop closure), the bag tooling, and the cost of a run | S90, `scripts/analysis/` |
| 11.2 | Estimation | Can actuator calibration (gain, offset, deadband) be identified from ordinary driving with enough accuracy to replace bench measurement, given that three driven fits disagreed with the bench value of the servo offset by up to 0.009 servo units (47 cm of drift over 5.5 m)? Which odometry reference is trustworthy for scale, given rf2o at -6 % and VSLAM at +1 % against tape? | 9.3, 9.4, 3.4 |
| 11.3 | Control | Speed-floor-aware tracking: the car cannot move below 0.20-0.26 m/s, and the goal approach stalled at 0.269 m/s. How should MPC or RPP terminal behaviour account for a hard breakaway speed? Obstacle-aware MPC on a platform with ~100 ms actuation lag: what horizon and rate does the lag force? | 6.4, 7.6, control brief |
| 11.4 | Perception and fusion | Which detector (laser, pointcloud, image + depth) gives obstacle states good enough for the MPC at what rate on the Orin, and does fusing them help? | perception brief |

**Cut totals (estimates, integration chat confirms from `build.sh`)**: `lab` 42, `sponsor` 32, `research` 38. Reference-only: 26. Full deck: 72.
-->

---

<!-- cut: research -->
<!-- plan §3 row 11.1 | owner: integration chat, seeded by the owning briefs -->

## What this platform can measure

<!--
TODO: one research question the platform can answer with an experiment
we can run, plus the evidence already in hand. See the S99 table quoted
above and the owning brief. Title must be the question's claim.
-->

---

<!-- cut: research -->
<!-- plan §3 row 11.2 | owner: integration chat, seeded by the owning briefs -->

## Estimation

<!--
TODO: one research question the platform can answer with an experiment
we can run, plus the evidence already in hand. See the S99 table quoted
above and the owning brief. Title must be the question's claim.
-->

---

<!-- cut: research -->
<!-- plan §3 row 11.3 | owner: integration chat, seeded by the owning briefs -->

## Control

<!--
TODO: one research question the platform can answer with an experiment
we can run, plus the evidence already in hand. See the S99 table quoted
above and the owning brief. Title must be the question's claim.
-->

---

<!-- cut: research -->
<!-- plan §3 row 11.4 | owner: integration chat, seeded by the owning briefs -->

## Perception and fusion

<!--
TODO: one research question the platform can answer with an experiment
we can run, plus the evidence already in hand. See the S99 table quoted
above and the owning brief. Title must be the question's claim.
-->
