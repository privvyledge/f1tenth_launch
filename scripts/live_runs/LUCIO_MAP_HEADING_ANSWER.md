# Answer to the LUCIO side — what produced `pose_map` in the 2026-08-05 `mapframe_*` bags

**gosling1 side, 2026-08-24.** Closes the one open item from
`LUCIO_MAP_HEADING_NOTICE.md` §5. Their question, verbatim: were the `pose_map`
poses produced by a localizer that *converges to the map*, or read from
RTABMap's optimized poses?

---

## Short answer

**AMCL, converging on the occupancy grid.** `pose_map` is `nav2_amcl`'s
`map->odom` composed with the source bag's own `odom->base_link`. RTABMap's
optimized poses are not in that path at any point.

**And the follow-up they pre-authorised — "if RTABMap, give us the rotation" —
does not arise, and would not have arisen even on the other branch.** There is
no LUCIO-frame -> map-frame rotation to carry. See §3; that is the part worth
reading even though the headline is the good branch.

## 1. The provenance chain, file by file

| step | file | what it does |
|---|---|---|
| replay + localize | `scripts/live_runs/51_localize_offline.sh` | replays the original bag, launches localization with `PUBLISHER=amcl` (line 74) -> `map_tf_publisher:=amcl`, i.e. **AMCL broadcasts `map->odom` directly**; records `/<ns>/tf` |
| compose | `scripts/analysis/make_map_frame_bag.py` | `read_map_odom()` takes **only** the `map->odom` transform out of that localization bag and composes it with the source bag's untouched `odom->base_link`, one output message per `odometry/local` message |
| freeze | `scripts/live_runs/MAP_FRAME_DELIVERY.md` | v1 freeze table records it independently: `localizer = nav2_amcl`, `config/localization/localizer_amcl_mapframe.yaml`, `broadcaster = AMCL direct (map_tf_publisher:=amcl)` |

RTABMap appears in the 2026-08-05 pipeline in exactly two roles, neither of them
`pose_map`:

1. **As the seed** — one pose at t=0, `(+0.4451, -0.5750, -1.3931 rad)`
   (`51_localize_offline.sh:79-81`), read out of the database's optimized poses.
   After that single value AMCL runs on the laser.
2. **As the scoring control** — `scripts/analysis/rtabmap_ground_truth.py` reads
   `Admin.opt_poses` from `rtabmap_final.db` to grade the `mapping_drive_170025`
   run. That is a comparison track, never written into a delivered bag.

## 2. Does the seed bias survive into the track?

Not rigidly, and there is measured evidence in both directions of the question.

- **AMCL is not locked to the seed frame.** In the delivered control run its yaw
  disagrees with the RTABMap track by mean 1.69 deg, p95 4.92 deg, **max 7.19
  deg** (`MAP_FRAME_DELIVERY.md`). A track that had inherited the RTABMap frame
  rigidly could not disagree with it by 7 deg.
- **AMCL demonstrably walks toward the geometry unaided.** Live, 2026-08-11,
  seeded at -79.80 deg it settled at **-83.97 deg** — 4.17 deg *toward* the
  independent scan fit, by a different algorithm, with no help.
- **The residual is transient, not constant.** The least-converged samples are
  the earliest ones. Per-15 s translation error on the control run is
  6, 48, 92, 120, 108, 78, 54, 75, 33, 26 mm — so it is bounded and it tightens.
  Seeding at the spot pose (rather than the origin) exists precisely to shorten
  that window; seeding at the origin cost ~28 s of unusable convergence.

So: seed error washes out, which is the branch they were hoping for. The honest
caveat is that it washes out *over seconds*, not instantly — if any LUCIO fit
weights the first few seconds of a calibration run equally with the rest, those
samples are the ones carrying whatever is left of it.

## 3. Why there is no rotation to report on either branch

This is the part that reframes the question, and it is worth stating because the
"if RTABMap, send us the offset" plan would have quietly encoded a wrong model.

**The 4.7 deg was never a rotation of the map frame.** `rtabmap_2d_final` — the
grid — is rendered *from* the RTABMap pose graph. The map frame is definitionally
RTABMap's frame. What was wrong was a **single pose expressed inside** that
frame: where the car parks. A wrong pose-in-a-frame does not rotate the frame,
and nothing downstream of it inherits a rigid rotation.

Both numbers in the disagreement are already grid-relative:

- `heading_from_scan.py` asks where the car must be **for its raw scan to fit the
  grid** -> -84.5 deg. Any rotation of the grid relative to the room cancels out
  of this.
- RTABMap `Admin.opt_poses` says the car at t=0 of the mapping drive was at
  -79.82 deg **in the same grid's coordinates**.

So the 4.7 deg is a genuine disagreement *within one frame*, with two live
explanations we have not separated: either the car was parked ~4.7 deg
differently on 2026-08-05 than on 2026-08-11, or RTABMap's t=0 pose is off
relative to its own rendered grid. The second is quite plausible: that graph was
optimized **from the end of the trajectory** (`map` coincides with `odom` at the
*last* keyframe, not the first — `MAP_BUILD_HANDOFF.md`), so t=0 is the
least-constrained end of it.

One extra wrinkle in the same direction, for the record: the seed came from
`rtabmap_final.db`, while the grid and both clouds were rendered from
`rtabmap_final_nf.db`. Two databases, two optimizations, so their `map` origins
need not coincide exactly. This is another reason not to treat -79.82 deg as
map-truth — and again it does not touch `pose_map`, which reads neither database.

## 4. What this means for LUCIO, concretely

- **No refit, no stated offset, no change to your weights.** Your world frame is
  our map frame, and it is map-true to whatever AMCL's convergence is worth
  (delivery control: mean 65 mm / p95 144 mm, with the stated correlated-error
  caveat, since AMCL and RTABMap localize the same scans against the same room).
- **`yaw_source='world'` is unaffected by the seed change.** That path takes ego
  heading from vehicle odometry, not from the parking-spot seed. The 4.7 deg
  correction moved a startup constant, not the delivered track.
- **The v1 `mapframe_*` bags are unchanged and stay valid.** Nothing in this
  answer regenerates them. `deliverables/20260805/` is still frozen, and
  `MD5SUMS.txt` still governs.

## 5. What we did not do

- **We did not run the scan-fit check against an archived bag.** It remains the
  right way to settle *which* of the two §3 explanations is true, and it is still
  one command. It is blocked today, not declined: `gosling1` was reflashed
  2026-08-24 and neither it nor `velox1` (which holds the rescued archive) was
  reachable when this was written. It is worth running for our own map hygiene;
  **it cannot change any LUCIO number**, because it adjudicates the seed, and the
  seed is not in your path.
- **We did not touch x/y.** Per their reply, not prioritised. If a future
  measurement moves the *track* rather than the seed, they get the number.

---

## 6. Round 2 — LUCIO's return measurement, and why it settles our §3

**2026-08-24, same day.** They closed §5 with no open items, having bounded the
seed-residual exposure we flagged in §2. Their bound, for the record: `map_yaw`
reaches their fit through exactly one term — a centroid-to-rear-axle de-offset of
`d ~ 0.13 m` — so a yaw error costs `d*eps` in metres and nothing else. Taking the
whole yaw excursion over each run's provably-parked window and crediting *none*
of it to ordinary AMCL noise: **worst case 5.2 mm**, against their 98.6/99.6 mm
held-out error. ~5% of budget on a few percent of samples. Closed, no refit. They
also confirmed the -79.80 deg seed value never appears in their data: the first
*paired* sample of all six run/camera combinations already reads -83.0 to -90.5
deg, i.e. AMCL had converged before their first usable sample.

**The part they offered as "weak evidence, not a claim" bears on the two
explanations §3 left open — but at less strength than I first wrote it.**

*Correction, same day.* My first pass here said the two runs "both land on
−85.00°", matching our 08-11 scan fits "to the digit", from "two runs, different
data". All three were wrong, caught by LUCIO re-checking their own probe after I
upgraded their hedged line into an inference. Withdrawn:

- **−85.00° was a window edge, not a plateau.** Re-sampled properly, `loop_laps`
  goes −82.99 → −85.00 over 10.2 s and is **still walking at −0.07 °/s** at the
  last parked sample; `mapping` goes −83.52 → **−85.79** over 8.4 s, still walking
  at **−0.74 °/s**, crossing −85.00 around t=5.8 s and continuing. Neither run
  demonstrably converges anywhere. The digit-for-digit agreement was coincidence
  between one run's truncation and another's mid-transient.
- **n = 2, not 4.** `cam1`/`cam2` of a run are two samplings of *one* AMCL
  trajectory merged against two camera clocks, not independent evidence about
  AMCL. Only `mapping` vs `loop_laps` are independent. `figure8` has a 0.4 s
  parked window and says nothing.

**What survives, at its real strength:** both independent runs walk *away* from
the −79.80° seed, in the same direction, while the car is provably parked (≤5 mm
over 8–10 s), and both reach **at least −85.0°** — `mapping` reaches −85.79° and
is still moving. `loop_laps`'s rate decaying to −0.07 °/s near −85.0 is the one
datum resembling an approach to a fixed point; `mapping` is mid-transient
throughout and bounds its endpoint only as ≤ −85.79°.

So: **the 08-05 park heading was at least −85.0° and still moving away, not
−79.8°.** That is a one-sided bound — the endpoint could be −87° or −88° for all
this data says. It is *not* corroboration of −84.50°/−85.00° to any decimal, and
extrapolating either run's decay to an asymptote is unsafe: the two rates differ
by 10× at their truncations.

Why this still adjudicates §3, without needing a number. That walk is a
measurement of the **2026-08-05 scan against the 20260805 grid**. Both runs leave
−79.8° and are still going *away* at −85.0° or beyond, so the 08-05 park is
**materially closer to the 08-11 park than to the RTABMap t=0 pose**. That is a
one-sided comparison, and one side is all the elimination needs: the gap being
eliminated is 4.7°, and the residual is well under it. So §3's first explanation —
*"the car simply parked ~4.7° differently on 08-05"* — is disfavoured, and by
elimination:

> **RTABMap's t=0 optimized pose (−79.82°) is wrong relative to its own rendered
> grid.** Magnitude ~5°, but loosely bounded — see the caveat below.

Which is the explanation §3 already called plausible on independent grounds: that
graph was optimized **from the end** of the trajectory (`map` coincides with
`odom` at the last keyframe), so t=0 is its least-constrained end. The
`rtabmap_final.db` vs `rtabmap_final_nf.db` split points the same way. That
structural argument is untouched by the correction above.

**The seed-pull loophole still cuts in the right direction.** AMCL produced this
while seeded at −79.80°, so any residual pull biases the answer *toward* −79.8.
Walking to −85.79° anyway makes the direction conservative, not contaminated.

**The honest limit.** Two things this cannot do. It is the same measurement class
as `heading_from_scan.py` — both fit scan endpoints to the grid — so it cannot see
an error common to both, i.e. a grid rotated relative to the physical room (a
harmless limit here, since everything in this thread is grid-relative by
construction, §3). And with neither run converged, it **cannot put a number on
the RTABMap t=0 error** — only a direction and a rough size.

### What this changes on our side

- **The archived-bag raw-scan check is no longer what the LUCIO answer waits on,
  but it is the only thing that can put a *number* on the RTABMap t=0 error.**
  Neither parked AMCL walk converged, so they give a direction and a rough size
  and nothing tighter. Run it when `gosling1`/`velox1` come back.
- **The `-79.82 deg` offline replay seeds are off by roughly 5 deg** —
  `51_localize_offline.sh:81`, `61_nav2_offline.sh:90`, `check_map_frame.py:274`,
  `BRIEF_PARTICLE_FILTER.md`, `MAP_BUILD_HANDOFF.md`. This costs **convergence
  time, not correctness**: AMCL walks off the seed, as both runs above show. Worth
  correcting to ~-1.4748 rad for faster settle; not urgent, and not a reason to
  touch v1.
- **`MAP_FRAME_DELIVERY.md`'s "yaw error mean 1.69 deg / max 7.19 deg vs RTABMap
  GT" needs re-reading.** If RTABMap's t=0 is ~5 deg off against its own grid,
  part of that reported disagreement is **RTABMap's error, not AMCL's** — a t=0
  offset decaying to zero has qualitatively the shape of that error profile. This
  is deliberately not quantified: the parked-AMCL data cannot bound the t=0 error
  tightly enough to be set against the 7.19 deg max. That needs the scan check.
  The translation figures (mean 64.7 mm / p95 143.5 mm) are unaffected in kind but
  inherit the same caveat: it was always agreement-between-estimators.
- **Do not regenerate v1.** Nothing here improves the delivered track — it
  reinterprets the *control* it was scored against. `deliverables/20260805/` stays
  frozen and MD5-governed.
