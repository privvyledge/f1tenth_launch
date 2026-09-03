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
<!-- _class: dense -->
<!-- plan §3 row 10.1 | owner: B2 (f1tenth_launch) -->

## Seven open items, and five of them are one measurement each

| Next | Why it is next | Blocked on |
|---|---|---|
| Close the last 0.38 m of a navigation goal | the controller decelerates into the speed below which the car cannot move | a terminal-approach rule that respects the breakaway floor |
| Bench servo sweep | settles the steering offset **and** both servo bounds in one pass | a bench session; the sheet and the script exist |
| Obstacle avoidance on the car | never tested — no obstacle has been placed in front of it | a drive session with cones |
| Model-predictive go-to-goal on the car | path following drives; goal mode has not | the control chat's schedule |
| Perception into the bringup | detectors build and run standalone, nothing is wired in | integration, not research |
| Settle the speed constant | driven fits span 3687–4022 erpm per m/s — about 7 % | a staircase run scored against tape, not against one odometry source |
| Closed-loop speed and adaptive feed-forward | already implemented and wired, all defaulted off | the speed constant above; enabling them first would tune against a wrong gain |

<!-- src: CLAUDE.md open items, scripts/live_runs/LOCALIZER_FOLLOWUPS.md, SYSID_RESULTS.md; compiled 2026-09-02 -->

---

<!-- reference-only: no cut tag (plan §3 marks this R) -->
<!-- _class: dense -->
<!-- plan §3 row 10.2 | owner: B2 (f1tenth_launch) -->

## Six things this car taught us about measuring it

| Lesson | What it cost to learn |
|---|---|
| **A stream can be dead and still read the right rate.** Verify by message count in a recording, never by a topic frequency before or after it | one whole drive's visual odometry, silently: 107 messages against the local filter's 6415 |
| **Never calibrate against a single reference.** Two estimators that each score R² ≈ 0.99 disagreed by 6.3 % | the speed constant nearly moved 6 % in the wrong direction |
| **Check the twist before trusting the pose.** A filter can lock into a runaway velocity and report a confident, wrong position | a parked car whose transform ran to nearly ten kilometres |
| **A bad timestamp is not a bad sensor.** Three IMU samples stamped seconds out of order swung attitude by up to 167°, while the raw gyro showed no motion at all | three sessions hunting an attitude problem that was a clock problem |
| **A closed safety gate publishes at full rate with empty payloads.** Frequency proves nothing; echo the values | — |
| **An unexplained failure is a state bug before it is a code bug.** Check what is on disk, then leftover processes, then the environment | orphaned nodes that made the next run look like a duplicated launch |

<!-- src: CLAUDE.md §Debugging Caveats and the live_runs closeout notes; incidents dated 2026-08-25 through 2026-09-01 -->

---

<!-- cut: research -->
<!-- _class: dense -->
<!-- plan §3 row 11.1 | owner: integration chat, seeded by B2 (f1tenth_launch) -->

## The platform's real asset is three ground-truth references that do not share a failure mode

| Reference | What it is good for | Known error |
|---|---|---|
| **Tape measure** | absolute distance over a straight run | the only one with no model in it — but manual, and one number per run |
| **Visual SLAM** | continuous pose at 30 Hz | +1.3 % over a 5.50 m tape run |
| **LiDAR range-flow odometry** | continuous pose at 10 Hz, no camera | **−6.1 %** over the same run |
| **Loop closure from the map database** | absolute *heading* truth | the three archived mapping runs each start and end parked in the same spot, which turns the graph into a measurable closure |

Plus: six fused estimator inputs, bags with a topic set that costs 47 MB per 105 s instead of 26 GiB per 214 s, and offline scripts that replay a whole navigation or localization stack with no robot attached.

**Cost of a run**: one battery charge, one operator, and about eight minutes of bring-up. That is what makes a repeat-measurement study — six cold launches to accept one fix — practical here and not elsewhere.

<!-- src: scripts/analysis/, scripts/live_runs/SYSID_RESULTS.md, topic_sets.sh; figures measured 2026-08-27 and 2026-09-01 -->

---

<!-- cut: research -->
<!-- plan §3 row 11.2 | owner: integration chat, seeded by B2 (f1tenth_launch) -->

## Can a car identify its own actuators from ordinary driving, or does calibration still need a bench?

**The question.** Actuator constants — steering gain, steering offset, deadband, speed gain — are conventionally measured on a bench. Can they be identified to the same accuracy from ordinary driving logs, so that a fleet re-calibrates itself?

**What the evidence already says, and it splits:**

| Constant | Driven identification | Verdict so far |
|---|---|---|
| Steering **gain** | two independent bags agree to 0.85 %, and the correction they produced holds at full lock | <span class="ok">driving is enough</span> |
| Steering **offset** | three driven fits disagree with the standing value by up to 0.009 servo units — which is 47 cm of lateral drift over 5.5 m | <span class="no">driving is not enough</span> |
| Speed **gain** | two references that both fit at R² ≈ 0.99 disagree by 6.3 % | <span class="warn">needs an external reference</span> |

**The second question follows from the third row**: which odometry reference is trustworthy for *scale*, when one reads −6.1 % and another +1.3 % against tape and both look healthy? **The experiment**: a bench servo sweep gives the offset directly, a speed staircase scored against tape gives the gain — and both then become the truth that scores every driven estimator.

<!-- src: scripts/live_runs/SYSID_RESULTS.md, BENCH_SWEEP_SHEET.md; fits 2026-08-07 and 2026-09-01 -->

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
