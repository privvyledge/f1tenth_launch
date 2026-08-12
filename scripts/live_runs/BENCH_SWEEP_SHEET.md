# Stage 0 bench sweep — procedure and recording sheet

Wheels off the ground, stack down, ~40 min. Measures what no bag can: the
servo→road-wheel slope with no model and no inherited constant, the real
mechanical travel, and **toe**.

Tools: `scripts/analysis/bench_servo_sweep.py` (publish a held servo value) and
`scripts/analysis/reduce_bench_sweep.py` (reduce the readings).

> **2026-08-07, attempt 1: stopped.** The machine side all worked; the written
> description of the square-and-ruler rig was not followable at the car.
>
> **2026-08-07, attempt 2 (evening): overhead photo method adopted, one trial
> photo taken, stopped for a hardware reason.** The photo settled the important
> questions and the method below is rewritten from what it showed. Stopping
> reason: the ruler taped to the left wheel is too heavy and sags around its
> pivot; the operator is bringing a lighter one. Read
> `## What the 2026-08-07 trial photo established` before running this.

---

## Method A — overhead photographs

Angles lying in a plane parallel to the camera sensor survive projection
unchanged, so a phone held level above the car measures in-plane angles with no
scale reference and no calibration. The only thing that has to be right is that
the phone is level — the iPhone camera shows a level crosshair when pointed
straight down.

**Validated on real data 2026-08-07.** Extraction is not the bottleneck: a
wooden ruler edge fits to better than 0.1°, and three independent tile grout
lines in the same frame agreed to **0.03°**.

1. Tape a ruler to **each front wheel** as a high-contrast pointer (see
   "Mounting a ruler" below). Then do not touch them again.
2. At each servo value, take **one photo from directly overhead**, phone level,
   as high as you can reach, with both front wheels and a floor reference in
   frame.
3. That is the operator's entire job. No square, no perpendicular judgement, no
   arithmetic.

What ruins it: phone tilt (use the level indicator), being too close (stand
back and zoom rather than leaning in — it reduces perspective error), and lens
distortion at the frame edges (keep the car centred; grout lines measured at
opposite sides of one frame differed by 0.35°, which is the distortion budget).

### The rear reference photo is no longer needed

This is the main simplification from the trial. **Nearly every Stage 0
deliverable is invariant to a constant per-wheel mounting error**, because a
constant offset cancels in a slope:

| deliverable | survives a mis-seated ruler? |
|---|---|
| servo → angle **slope** | **yes** |
| **travel limits** at the mechanical stops | **yes** |
| **backlash** (same servo, approached from two directions) | **yes** |
| left/right **asymmetry** (slope ratio) | **yes** |
| **wheelbase** | yes — it is a tape measurement |
| **toe** | **no** — the only casualty |
| zero-steer `s₀` | not needed — bags give 0.5591 / 0.5610, agreeing to 0.002 |

So the ruler does not have to be *aligned*, only **rigid and never touched
again**. Tape both rulers to the front wheels, shoot the whole sweep, done.
Measure toe separately by the differential gap method (below), which needs no
camera. Reduce with per-wheel slopes rather than the combined bicycle angle, so
each wheel's constant offset cancels independently.

`reduce_bench_sweep.py` still expects a `rear` row; pass a row of zeros for it
when using this method, and read the per-wheel slopes rather than `s0`.

### Method B — steel square against a floor line (fallback, and cross-check)

Slower and coarser, but needs no image processing. Worth doing at two or three
servo values regardless, as an independent check on Method A.

Full geometry is in `BENCH_SWEEP_ILLUSTRATION_PROMPT.md`; the short version is
that a right-angle square projects each tape tab down onto a floor line and you
read the offset off the square's long leg. At 1/8 in graduations over an 11 in
span each angle lands to about **±0.4°** — fine for the gain and offset, marginal
for toe.

**Do not use a phone compass.** Steering is rotation about a vertical axis, so a
level or inclinometer reads nothing at all — but a compass fails for a different
reason: there is a motor, a LiPo and steel hardware inches from each wheel, so
the local field is neither north nor even the same at the two wheels.

## What the 2026-08-07 trial photo established

One overhead photo, rulers taped to the two wheels of one axle. Measured against
the car's own body edge (−1.90° in the image frame):

| feature | image angle | vs car body |
|---|---|---|
| upper ruler, resting on the tire's rounded shoulder | **0.00°** | 1.90° off |
| lower "ruler" — a board lying on the **floor** beside the tire | −1.77° | 0.13° |
| tile grout lines, same frame | −1.40° … −1.78° | — |

**Neither pointer was reporting its wheel's heading.** The 1.9° spread is
mounting slop, not geometry. This is the failure to design against.

Two hardware-free alternatives were tried on the same image and **both failed** —
do not spend time rediscovering them:

- **Fitting the tire silhouette**: 5.7° disagreement between the two wheels of
  the ruler-free axle, 16.4° on the other, fit RMS 8–46 px.
- **Fitting the tread grooves**: 3.5–10.7° of internal spread within a single
  tire.

The reason is geometric: a wheel is a circle, so anything pressed against its
curved outer surface is free to pivot about the vertical axis. Only a rigid
straight edge indexed to a **flat** feature reports the wheel plane. Since the
slope survives a constant offset (above), **rigidity is what to optimise for,
not alignment.**

Also learned: the ruler must be **light**. A heavy 12 in wooden ruler taped to
the tire sags around its taped pivot under its own weight — which is both a
mounting error and a drifting one.

## Mounting a ruler on a wheel

Aim for **rigid, not aligned**: flat along the top of the tire, running
front-to-back, with tape wrapped over the ruler and down **both** sidewalls and
pulled tight — ideally two wraps. Use the lightest straightedge available. Then
treat it as untouchable for the rest of the session; if one shifts mid-sweep,
say so and restart from the last good step rather than from the beginning.

The layout below is still right for clearance.

```
        rear ←                                    → front

              ├────┼──────────────────────────────────┤
              0"   0.5"                            11.5"
                    ▲                                ▲
                   tab                              tab
                                d = 11 in (280 mm)

                    ┌──── hub sits about here, 2 in from the rear end
```

- Flat against the **outer** face of the wheel — the face pointing away from the
  car — pressed on the rim lip or the tire sidewall.
- **Horizontal**, at hub height, running front-to-back.
- **Off-centre on purpose**: about 2 in behind the hub and 10 in ahead of it.
  Centred, the 6 in tail sweeps ~2.4 in inward at full lock and fouls the
  chassis; with only 2 in behind, it sweeps under an inch and stays clear.
- Tape tabs at the 0.5 in and 11.5 in marks so there is something physical to
  touch and something high-contrast to see.

**Check clearance before commanding anything**: turn each front wheel by hand to
full lock both ways and confirm the ruler touches nothing.

## Setup

1. **Prop the front of the car up** about 2 in, block under the chassis plate
   just behind the front wheels. Front wheels hang free; rear wheels stay on the
   floor so the car cannot roll. Loaded front tires scrubbing on the floor would
   stall the servo and make this measure friction instead of geometry.
2. **Baseline** (Method B only): a tile grout line along the car's **left** side
   is ideal — straight, long, and free. Position the car 3–4 in to the right of
   it so every reading is comfortably positive. It does **not** need to be
   accurately parallel to the car; the rear-wheel reference cancels that.
3. Tape rulers on per above.

## Order of work

### 1. Wheelbase (settles 0.25 vs 0.256)

Hub centre to hub centre, rear to front, on **each** side, wheels centred.

    wheelbase left side  : ________
    wheelbase right side : ________

### 2. Rear reference — measure once

**The reference must come from the rear wheels alone.** They are on a fixed axle
and cannot steer. Do not substitute "the front wheels look straight": until the
sweep script publishes a value, the servo sits wherever it powered up, and a
degree or two of error there contaminates every later reading.

One ruler on the **rear-left** wheel, one on the **rear-right** — never a single
ruler spanning two wheels, which cannot survive steering and averages the two.

    rear-left    front ______   rear ______
    rear-right   front ______   rear ______

The two should agree within ~1°; the reduction warns if they don't.

### 3. Walk-out — find the real mechanical travel

**This is the part that can cook a servo.** `[0.08, 0.92]` came from a different
car, and `servo_min` has never been reached on this one in any of six archived
bags, so nothing is known about the left bound.

Go **right first** (toward 0.92): the archived bags spent 17–20 % of their
samples pinned there, so it is empirically survivable. **Then left** (toward
0.08), which is unexplored ground.

Step 0.02 at a time. At each step hold **≤ 2 s**, watch and listen, then return
to centre before deciding to go further. Back off one step on **any** of:

- the wheel stops moving while the command keeps changing,
- a buzz or whine that persists rather than settling,
- visible binding of the linkage or servo horn.

    lowest free servo (LEFT lock)  : ________
    highest free servo (RIGHT lock): ________

The driver's own `servo_min`/`servo_max` clamp still applies and must be widened
in steps to get past `[0.08, 0.92]` at all.

### 4. Staircase — the actual measurement

~11 values evenly spanning the measured travel, **including 0.56 and both
bounds**. Method A: one photo each. Method B: fill this in.

| servo | FL front | FL rear | FR front | FR rear | gap front | gap rear |
|---|---|---|---|---|---|---|
|  |  |  |  |  |  |  |
|  |  |  |  |  |  |  |
|  |  |  |  |  |  |  |
|  |  |  |  |  |  |  |
|  |  |  |  |  |  |  |
|  |  |  |  |  |  |  |
|  |  |  |  |  |  |  |
|  |  |  |  |  |  |  |
|  |  |  |  |  |  |  |
|  |  |  |  |  |  |  |
|  |  |  |  |  |  |  |

**`gap front` / `gap rear`** are the distance *between the two front rulers*,
measured at the front tabs and at the rear tabs. This gives toe differentially —
the baseline drops out and only two readings contribute, roughly halving the
uncertainty versus subtracting two absolute angles:

    toe = atan((gap_front − gap_rear) / d)     positive = toe-out

### 5. Backlash check — 3 repeats on the way back

Approach three of the values above **from the opposite direction** and
re-measure. The difference is linkage slop, and it bounds what any controller
can achieve.

| servo | FL front | FL rear | FR front | FR rear |
|---|---|---|---|---|
|  |  |  |  |  |
|  |  |  |  |  |
|  |  |  |  |  |

---

## Reduction

CSV, one row labelled `rear` and one per servo value. Units must be consistent
(all inches or all mm) and `--d` must be in the same unit:

```
# label,fl_front,fl_rear,fr_front,fr_rear
rear,196.3,203.7,196.3,203.7
0.10,141.6,258.4,143.2,256.8
```

```bash
python3 scripts/analysis/reduce_bench_sweep.py readings.csv --d 280 --wheelbase 0.256
```

Outputs the servo→angle slope and zero-steer offset (directly comparable to
`steering_angle_to_servo_gain` / `_offset`), the left and right slopes and their
asymmetry, the travel limits, the symmetric no-clip half-range (the
`max_steering` candidate), and the toe.

Verified 2026-08-07 against synthetic readings generated from a known
`a = −0.87355`, `s₀ = 0.56005`, 0.6° toe-out and a deliberate 1.7° baseline
misalignment: it recovered the slope to 0.02 %, the offset to 0.0001 servo
units, and the toe exactly.

## The servo-horn decision this sweep feeds

Compare `s0` (zero-steer) against the midpoint of the measured mechanical
travel:

- **Far apart** → the horn is clocked off neutral. Re-centring it recovers
  symmetric authority on both sides.
- **Close together, but the angles are still asymmetric** → the asymmetry is in
  the linkage geometry, and re-centring buys nothing; cap `max_steering` at the
  smaller side instead.

**Re-centring invalidates every archived bag for calibration.** If it happens,
the Stage 2 numbers become a methodology dry-run and all calibration must come
from freshly recorded data. That is the operator's call, not the analysis's.
