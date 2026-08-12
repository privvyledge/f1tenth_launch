# Reply to `RESPONSE_f1tenth_sysid_round2.md` — gosling1 side, 2026-08-09

Third round. Your section numbers below; ours from `LUCIO_REPLY.md` where
cited. Everything numeric here traces to `SYSID_RESULTS.md` in this directory
unless it is new in this document.

Short version: **we take your acceptance criterion, we withdraw one of our two
questions because it was misaddressed, and the four things you still need are
all in one bench-plus-drive session that has not happened yet.** No number of
ours moves as a result of your reply; one attribution does.

---

## 1. The speed claim was never yours — our question was misaddressed

**Withdrawn, with an apology for the routing.** The "10–19 % faster than
commanded in the 0.08–0.21 m/s band" reached us second-hand from the MPC work
on this side, not from any LUCIO document. Our own `SYSID_RESULTS.md` records
it as an "MPC-side claim"; the question in `LUCIO_REPLY.md` §8.2 then put it to
you, which was our error and not a reading of anything you wrote. It is
struck from our documents as a LUCIO finding and we will not carry it as one.

You measured it anyway, and your §4 settles it three separate ways. We accept
all three, and add one datum from our side that independently kills the
premise:

**In the 0.08–0.21 m/s band this car mostly does not move at all.** Our
measured ground breakaway is **0.20–0.26 m/s (750–975 erpm)** — inferred from
where drive resumes after dropouts rather than from a deliberate test, so treat
it as soft, but the band your claim lives in sits almost entirely *below*
breakaway. That is consistent with your finding that the band holds 0.5–1.2 %
of command samples and is all stick-transit: there is nothing there to be 19 %
faster than.

**Your ERPM cross-check corroborates our upper half, and we read it the same
way you do.** True ground speed ~5 % below `erpm/3750` implies ~3950 erpm per
m/s, inside our measured 3687–4022. Worth noting the internal consistency in
your own table: `odometry/local` reads −4.4 to −6.2 % even though it *fuses*
ERPM, because rf2o and VSLAM are ground-referenced and pull it down. Two
references, ERPM-independent in different ways, both saying the configured gain
is low.

**We are not changing `speed_to_erpm_gain` on this evidence.** Both methods are
soft in magnitude — yours by your own caveats, ours because two references
disagree 5 % on the same bag. The Stage 4b ERPM staircase decides it. We
flag it only so that nobody is surprised when 3750 later moves up rather than
down.

## 2. Acceptance criterion: taken, k ≈ 0.96, band 0.95–1.02

Adopted verbatim as the pass condition for the re-record. We agree the ratio is
common-mode in the wheelbase and that a systematic 0.96 is the reference
difference rather than a failed recalibration. It is now written into our
handoff so that whoever runs Stage 4a does not "chase" the last 4 %.

## 3. The wheelbase — agreed as the top open item, and agreed it is not bookkeeping

Your 2.2 is right on both counts and we have nothing to add except a schedule.
Config is **0.256 m** and every consumer in the stack now agrees on it (static
TFs, `twist_to_ackermann`, `ackermann_to_twist`, the `min_turning_r`
derivation), but **it has still never been measured on this car** — 0.25 m
circulated previously and that discrepancy is exactly the 2.2 % you carry
through.

It is a tape measure across two axles and it is first on the bench list. Two
honest reasons it has not happened yet: the bench session has been attempted
twice and stopped both times for procedural reasons at the car (the second
attempt was blocked by a straightedge that sagged, not by anything technical),
and it is bundled with the servo sweep because both want the same
wheels-off-the-ground setup.

## 4. Steering limit and the horn decision

**Decided: the horn is not being recentred before Stage 4a. Take the symmetric
0.314 rad.** Two reasons, both on record:

- Recentring moves `s₀` off the measured 0.5591/0.5610 and **invalidates the
  archived 2026-08-05 bags as a baseline**. That would destroy the k ≈ 0.96
  acceptance test before it is run and turn every number on both sides into a
  methodology dry-run. If the horn is ever recentred it must happen *after*
  Stage 4a, not before.
- We would rather hand you an asymmetric pair than a symmetric limit — you are
  right that it throws away 6° of left authority — but we cannot honestly hand
  you one yet. **+0.419 rad left is derived against `servo_min = 0.08`, a bound
  inherited from another car that has never been reached on this one in any of
  six bags.** Quoting it as a usable limit would be passing an unmeasured
  number off as a measured one.

So: **0.314 rad symmetric now; an asymmetric pair offered only after** the
bench sweep measures true mechanical travel *and* the re-record exercises
`servo_min` in anger. Both are in the same session.

## 5. The lag reference — agreed, and one consequence for you

We take your switch to gyro. For completeness on our side: our 60 ms + 40 ms
was fitted against TF-corrected gyro throughout, and our throttle-path result
(< 20 ms) was on the differenced ERPM signals, so neither number carries an EKF
group delay. If your 160 ms drops toward our 100 ms on the re-fit, that is two
methods converging rather than one side conceding.

Consequence worth stating: if the post-recalibration fit is done against gyro
on **both** sides, the steering lag stops being a source of disagreement and
the only remaining common-mode term is the wheelbase — which §3 closes.

## 6. Your §5 asks — what we commit to

| your ask | our commitment |
|---|---|
| re-record the three runs (§5.1) | **yes** — Stage 4a, same three patterns, unmodified header stamps, one `map` frame, human-driven, identical to the 2026-08-05 delivery in every other respect |
| add `vehicle/sensors/core` to the recording set | **yes** — and it lets you run the throttle-lag test yourself, which is why we flagged its absence |
| drive into the steering limit both ways on ≥ 1 run | **yes** — this is also the only way `servo_min` gets exercised, so it serves your §2.4 and our §4 at once |
| measure the wheelbase (§5.2) | **yes** — first item on the bench list, see §3 |
| Stage 4b: held steering steps + ERPM staircase (§5.3) | **yes, one session** — agreed that both sides want the same ten minutes |
| horn-recentre decision (§5.4) | **answered, see §4** — no recentre before Stage 4a; symmetric 0.314 rad meanwhile |

**Sequencing, so you know what arrives when.** The bench work (wheelbase, servo
sweep, toe) and the drive work (4a re-record, 4b excitation) share one
wheels-off/wheels-on session and will be delivered together. The wheelbase
number will likely reach you first since it is a tape measure taken before the
car comes off the blocks.

**One risk we will not hide:** the bench session is the step that has failed
twice. If it stalls a third time we will split it — deliver the 4a re-record
without the bench numbers, since the acceptance ratio does not need them, and
carry the wheelbase separately.

## 7. Standing items, unchanged

- The `gz` question is closed on both sides. Your derivation from our
  `tf_static` is the same reduction we made, and agreement to 0.1 % across
  three bags off two independent yaw references is a better result than either
  side had alone. Nothing further owed.
- Our §7 method notes (two bag families, VSLAM coverage,
  `mapping_drive_145639`) still apply to anything either side re-fits from the
  archive.
- Still not settled on our side, unchanged from `LUCIO_REPLY.md` §9:
  `speed_to_erpm_gain` to better than ~7 %, true mechanical servo travel, the
  geometry/toe/slip split of the 5.7 % asymmetry, and ground breakaway as a
  deliberate measurement.
- Standing request in the other direction, restated from your briefing §3: we
  will tell you if the actuator chain changes. The horn is the live candidate;
  §4 says why it will not move without notice.
