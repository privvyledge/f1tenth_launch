# Handoff — actuator sysid: Stages 1–3 done, Stage 0 attempted, LUCIO reply remains

Written 2026-08-07, updated 2026-08-07 evening. Continues the
steering/throttle system-identification exercise; the staged plan it follows
(Stages 0-5) is held outside this repo and remains the authoritative spec. This
document says only what changed, what the numbers are, and what is left.

**Read `## Stage 0 — attempt 2` first if you are picking this up.** The method
is now validated and simplified; the blocker is a sagging ruler.

**The LUCIO exchange has had another round.** `LUCIO_REPLY.md` was sent; their
answer is `RESPONSE_f1tenth_sysid_round2.md` (2026-08-08, in the LUCIO_ROS
folder outside this repo) and our reply to *that* is **`LUCIO_REPLY_ROUND3.md`**,
next to this file — drafted 2026-08-09 and **not yet sent**, but complete: the
horn-recentre decision it turns on was taken by the operator the same day.

What that round changed, in one place:

- **The acceptance test is now `k ≈ 0.96` in a 0.95–1.02 band, not `k → 1.0`.**
  See the new section in `SYSID_RESULTS.md`. Do not chase the last 4 %.
- **The "10–19 % faster than commanded" claim is dead and was never LUCIO's** —
  it came from the MPC side and `LUCIO_REPLY.md` §8.2 misaddressed it to them.
  LUCIO measured it anyway and it fails three ways. Details in
  `SYSID_RESULTS.md`; the short version is that the 0.08–0.21 m/s band is below
  this car's breakaway speed and barely exists in the bags.
- **Two new requirements on the Stage 4a re-record**: add
  `vehicle/sensors/core` to the recording set, and drive hard into the steering
  limit **both** ways on at least one run so `servo_min` is finally exercised.
- The `gz` question is **closed on both sides** — nothing further owed.
- LUCIO withdrew their 160 ms lag figure in favour of ours; the reference was
  an EKF state.

## What is done

- **Stage 1** (driver transparency) — passed, `scripts/analysis/check_driver_transform.py`.
- **Stage 2** (offline sysid) — done, `scripts/analysis/fit_actuators.py`.
- **Stage 3** (apply) — **partially applied**: the steering gain only.
- Results, method, and every number: **`SYSID_RESULTS.md`**, alongside this file.

### What was changed in the repo

| file | change |
|---|---|
| `config/vehicle/vesc.yaml` | `steering_angle_to_servo_gain` −1.4 → **−1.1448**. `steering_angle_to_servo_offset` **left at 0.56** — measured 0.5591/0.5610, it was already right. `speed_to_erpm_gain` left at 3750 (measured 3687–4022, good to ~7 %). `servo_min`/`servo_max` untouched and still inherited. |
| `scripts/analysis/` | two new scripts (above). |

The package's architecture notes were also corrected: they described the servo
bounds with left and right transposed.

**Deployed 2026-08-07 evening.** `config/vehicle/vesc.yaml` was copied to
gosling1 via `/mnt/f1tenth_ssd/shared_dir/handoff/vesc.yaml.new` into
`/workspaces/f1tenth/src/f1tenth_launch/config/vehicle/vesc.yaml`. The install
tree is `--symlink-install`, so
`install/f1tenth_launch/share/f1tenth_launch/config/vehicle/vesc.yaml` is a
symlink to that file and **no rebuild is needed**. Verified by md5 (identical to
the repo file) and by grepping the deployed path:

```
steering_angle_to_servo_gain:   -1.1448   (was -1.4)
steering_angle_to_servo_offset:  0.5600
wheelbase:                       0.256
servo_min / servo_max:           0.08 / 0.92   (still inherited)
```

The previous copy is kept at `vesc.yaml.bak-0807` in the same directory. Nothing
has restarted since, so no *running* node has yet loaded −1.1448.

## The one number that matters

The steering gain was over-steering by ~18–23 %. Applied value is the mean of
the two bags that turn **both** directions and **never touch the servo clamp**:
`mapping_drive_170025` (a = −0.8783, s₀ = 0.5591) and `figure8_172338`
(a = −0.8688, s₀ = 0.5610), which agree to 1.1 %. The LUCIO side independently
got `k = 1.177` from different code, a different yaw reference, and a different
regressor.

**Read the bag section of SYSID_RESULTS.md before adding data.** There are two
families of bags and they are not the same recordings. The full-length
2026-08-05 session in `bags/20260805/` is the good one — 0 % servo saturation,
0–1 dropout windows, 12.5–12.8 V. The shorter
`rosbags/*_with_localization_and_pointcloud` excerpts spend 17–20 % of their
samples pinned at the right-hand clamp, and that bias is not cosmetic: it moved
the fitted offset by 0.018 servo units and **flipped the sign of the measured
left/right asymmetry**. An earlier pass of this work used only the short bags and
applied a wrong offset (0.5419) as a result.

## Stage 0 — attempt 2, 2026-08-07 evening: method validated, blocked on a ruler

Read this before the attempt-1 section below; where they disagree, this one wins.

**The overhead-photo method works and the procedure is now much simpler.** One
trial photo was taken and analysed. The stopping reason is hardware and trivial:
the 12 in wooden ruler taped to the left wheel is too heavy and sags around its
taped pivot. The operator is bringing a lighter straightedge.

### What the trial photo proved

Extraction is **not** the bottleneck. In one phone photo:

- a wooden ruler edge fits to better than **0.1°**
- three independent tile grout lines agreed to **0.03°**
- grout lines at opposite sides of the same frame differed by **0.35°** — that is
  the lens-distortion budget, and the reason to keep the car centred

Mounting **is** the bottleneck. Measured against the car's own body edge (−1.90°
in the image frame): the upper ruler, resting on the tire's rounded shoulder,
read 0.00° — **1.9° off**; the lower "ruler" turned out to be a board lying on
the *floor* beside the tire, not on the wheel at all.

Two hardware-free fallbacks were tried on the same image and **both failed** —
do not rediscover them. Fitting the tire silhouette gave 5.7° / 16.4°
disagreement between wheels on one axle; fitting the tread grooves gave 3.5–10.7°
of internal spread. A wheel is a circle: anything pressed against its curved
surface is free to pivot about the vertical. Only a rigid straight edge indexed
to a **flat** feature reports the wheel plane.

### The simplification that follows — no rear reference photo

Nearly every Stage 0 deliverable is **invariant to a constant per-wheel mounting
error**, because a constant offset cancels in a slope: the servo→angle slope,
the travel limits, backlash, and the left/right asymmetry all survive. Only
**toe** does not, and zero-steer `s₀` is not needed from the bench at all — the
bags give 0.5591 / 0.5610, agreeing to 0.002.

So the ruler does not have to be *aligned*, only **rigid and never touched
again**:

1. Tape two light straightedges to the **front** wheels. Do not touch them again.
2. Tape-measure the wheelbase, both sides.
3. Walk out the travel, then the staircase, then the backlash repeats — one
   photo per servo value.
4. Measure toe separately by the differential gap method; it needs no camera.

Reduce with **per-wheel slopes**, not the combined bicycle angle, so each
wheel's offset cancels independently. `reduce_bench_sweep.py` still wants a
`rear` row — pass zeros and read the per-wheel slopes rather than `s0`. Adding a
per-wheel-slope output to that script is a small, worthwhile change.

### Still unknown, and it gates the next session

**Which axle the rulers were on.** Front and rear are not distinguishable from
overhead in the trial photo. Ask; do not guess.

### Operator-facing material

`bench_sweep_card.html` (published as a private Artifact this session) is the
drawing-led field card — plan view, side elevation, ruler detail, run order.
It predates the simplification above in two places: it still calls for a rear
reference photo, and its Fig 3 still implies alignment matters. **Update it
before the next session** rather than handing it over as-is. GPT's figures at
`C:\Users\boluo\Documents\Codex\2026-08-07\...\outputs\` were checked and agree
with the card's geometry; they add nothing further.

Photo drop folder, already created and deliberately outside OneDrive:
`C:\Users\boluo\Documents\f1tenth_bench_sweep\20260807\`. In practice the
operator sent the trial photo through the chat instead, which was faster than
either cloud sync.

### Rig decisions taken

- **Level the car on two blocks**, not one central block. Nose-up on a single
  block pitches the chassis ~11°, tilting the steering axis by the same 11° and
  biasing the measured slope ~1.6 % low. A single central block also lets the car
  rock and yaw between shots. If only one block is available, keep it front-up
  with the rear wheels chocked and apply the 1.6 % correction in reduction.
- **Stay on the floor, not a table** — lower means further above the car for the
  same reach, which is what shrinks perspective error.
- Grey grout on grey tile is still extractable (0.03° repeatability, above), so
  a masking-tape overlay is a nice-to-have rather than a requirement.

### Robot state as left, 2026-08-07 evening

Servo publisher stopped, `vesc_driver_node` killed, `/dev/sensors/vesc` free,
all three containers warm. A `servo_hold.sh` helper was left at
`/mnt/f1tenth_ssd/shared_dir/analysis/servo_hold.sh` — `servo_hold.sh <value>`
holds the servo at a value on domain 7, `servo_hold.sh stop` releases it. It is
simpler to drive from the analysis side than the interactive
`bench_servo_sweep.py`, which needs a live stdin.

Note `pkill -9 -f vesc_driver_node` kills the `bash -lc` wrapper that contains
the pattern before it kills the node. Use `pkill -9 -x vesc_driver_node`, and
check with a bracketed pattern (`pgrep -a -f vesc_dri[v]er`).

## Stage 0 — attempt 1, 2026-08-07 daytime, not completed

**The blocker was the instructions, not the robot.** Everything on the machine
side worked; the operator could not follow a text-only description of the
straightedge-and-square measurement rig and stopped the session. Whoever picks
this up should **lead with drawings, not prose** — see
`BENCH_SWEEP_ILLUSTRATION_PROMPT.md` next to this file, which is a ready-to-use
prompt for generating the orthographic views, and the rewritten
`BENCH_SWEEP_SHEET.md`, which now leads with an easier method.

### What was ready and verified

| item | state |
|---|---|
| `scripts/analysis/bench_servo_sweep.py` | written, staged at `/mnt/shared_dir/analysis/` |
| `scripts/analysis/reduce_bench_sweep.py` | written, staged, **verified against synthetic data** |
| `scripts/live_runs/BENCH_SWEEP_SHEET.md` | written, staged |
| `vesc.yaml` deployment | done and md5-verified (above) |
| standalone `vesc_driver_node` | brought up and torn down cleanly |

The reduction script was checked by generating readings from a known
`a = −0.87355`, `s₀ = 0.56005`, an injected 0.6° toe-out, and a deliberate 1.7°
baseline misalignment. It recovered the slope to 0.02 %, the offset to 0.0001
servo units, and the toe exactly, and cancelled the misalignment. **The maths is
not what needs re-checking.**

### Robot state as left

- Full bringup **stopped**. The other agent occupying
  `jetson_container_20260807_085244` was asked to yield and had already shut its
  stack down. Zero ROS node processes remain; `/dev/sensors/vesc` is free.
- All three containers (`pf_sweep_claude_0807`,
  `jetson_container_20260807_085244`, `mpc_claude_0806`) left **warm** — do not
  pay the 8.5 min bring-up unnecessarily.
- The standalone driver ran as, and should be restarted as:

```bash
docker exec -d jetson_container_20260807_085244 bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspaces/f1tenth/install/setup.bash
  export ROS_DOMAIN_ID=7
  exec ros2 run vesc_driver vesc_driver_node --ros-args \
    -r __ns:=/gosling1/vehicle -r __node:=vesc_driver_node \
    --params-file /workspaces/f1tenth/install/f1tenth_launch/share/f1tenth_launch/config/vehicle/vesc.yaml \
    > /mnt/shared_dir/analysis/vesc_bench.log 2>&1"
```

  It connects to VESC firmware 6.2. Domain 7 is deliberate: domain 0 and domain
  42 both have daemons belonging to other agents. Note `pkill -f
  vesc_driver_node` does **not** kill it; `pkill -9 -x vesc_driver_node` does.

- **The throttle path is structurally dead in this configuration** and it is
  worth telling the operator so explicitly, because they asked. With the stack
  down, `commands/motor/speed` has 0 publishers — the whole chain that normally
  drives it (mux → command_gate → `ackermann_to_vesc`) is stopped, and
  `bench_servo_sweep.py` only ever writes `commands/servo/position`.

### Physical setup the operator built (reusable)

- Car's **front end elevated ~2 in** on a block, front wheels hanging free; rear
  wheels on the floor. This is the right configuration — loaded front tires
  scrubbing on tile would stall the servo and make the sweep measure friction.
- **Baseline is a tile grout line** running along the car's left side. Straight,
  long, and free — better than anything portable. The car should sit 3–4 in to
  the right of it so all readings are comfortably positive.
- Instruments: two 12 in wooden rulers (**1/8 in graduations**) and a steel
  right-angle square with 12 in and 8 in legs.

### The one reading taken

The operator taped a single ruler spanning **both** left wheels and read
`front = 2.9 in`, `rear = 2.8 in`. That is a valid setup sanity check — it puts
the tile line about **0.6° off the car's left-side wheel line** — but it is
**not** a calibration input, for two reasons worth recording:

1. A ruler taped across both wheels cannot survive the front wheel steering.
2. The reference must come from the **rear wheels alone**. Nothing had commanded
   the servo since the stack came down, so the front wheels were at an unknown
   power-up position; "looks straight" could be a degree or two off and would
   contaminate every later reading.

### Precision, worked out — matters for how the next session scopes toe

At 1/8 in graduations over an 11 in span, each wheel angle lands to about
**±0.4°**. That is comfortably good enough for the gain and offset, since the
sweep spans 40-odd degrees and the slope is fitted over ~11 points. It is
**marginal for toe**, which is a difference of two angles and so inherits ±0.9°.

Two ways to fix that, both worth carrying forward:

- **Measure toe differentially.** With a ruler taped to each front wheel,
  measure the gap *between the two rulers* at the front tabs and again at the
  rear tabs. `toe = atan((gap_front − gap_rear) / d)`, positive = toe-out. The
  baseline drops out entirely and only two readings contribute, roughly halving
  the uncertainty.
- **Use the photo method below**, which beats the tape outright.

### Recommended simplification for the next session: overhead photos

Given the operator found the rig hard to follow, the tape-and-square method is
probably the wrong primary. A **top-down photograph** is easier to execute and
*more* accurate:

- Angles in a plane parallel to the sensor are preserved exactly under
  projection, so a phone held level above the car measures in-plane angles with
  no scale reference and no calibration. The iPhone camera shows a level
  crosshair when pointed straight down; that is the only alignment needed.
- Tape a ruler to each front wheel and to each rear wheel as high-contrast
  pointers, then take **one photo per servo step**. The operator's whole job
  becomes "hold the phone level, press the shutter" — no arithmetic, no square,
  no perpendicular judgement.
- Extraction is the analysis side's problem, which is where it belongs. Over a
  ~24 in field at a few thousand pixels, a ruler edge locates to well under
  0.2°, and toe comes out of the same frame for free.
- Keep the tape-and-square method as the fallback and as an independent
  cross-check at two or three servo values.

### Practical notes that still stand from the plan

Three practical notes from the operator that change the plan's Stage 0 text:

**1. There is no protractor.** The plan says "straightedge or phone
inclinometer" — an *inclinometer* is the wrong instrument: steering angle is
rotation about a **vertical** axis, so a level/inclinometer reads nothing. Two
options that do work:

- **Phone compass**, held flat against the wheel's outer face. Read the heading
  at zero-steer as the reference and take **differences**, never absolute
  headings — there is a motor, a LiPo, and steel hardware inches away, and the
  local field is not north. Take every reading from the same phone position.
- **Geometric, and probably better**: lay a straightedge along the wheel face and
  measure its perpendicular offset from a fixed longitudinal reference line at
  two points a known distance `d` apart; the angle is `atan(Δ/d)`. No magnetic
  bias, no calibration, and it resolves small angles better than a phone compass.
  This is also the only method that measures **toe** cleanly, which is a main
  reason Stage 0 exists.

**2. Reaching the mechanical stops is gated by `servo_min`/`servo_max`, not by
the gain and offset.** The operator's instinct that "the current values might be
saturating too early" is right about the symptom and one level off on the cause.
The sweep publishes **directly** on `vehicle/commands/servo/position` with the
stack down, which bypasses `ackermann_to_vesc` and therefore bypasses the gain
and offset entirely. What still clamps is `vesc_driver`, using
`servo_min: 0.08` / `servo_max: 0.92` from `vesc.yaml`. To explore past those,
widen **those two** for the sweep — and widen them **one small step at a time**,
because they are inherited from another car and a servo driven past its stop
stalls and cooks.

**3. `servo_min` has never been reached on this car.** Across all six archived
bags, `servo_min` was never touched; in the two that reach a bound at all, 100 %
of the saturation was at `servo_max` (the right lock). So there is no evidence
whatsoever about the left bound, and the sweep must not assume the two ends are
symmetric.

Also for Stage 0: tape-measure the wheelbase — **this is now the single largest
source of disagreement between the two sides' fits and the cheapest thing on
either list to close.** 0.25 vs 0.256 is still unsettled, the fitted slope
scales with it, and at 0.25 the two sides' `k` values converge from 3.4 % apart
to 1.1 %. Take it before the car comes off the blocks and it can be delivered
ahead of everything else.

The **servo-horn re-centring decision is no longer open** — taken 2026-08-09,
**no recentre before Stage 4a** (see the round-3 summary at the top of this
file). The bench sweep therefore *measures* travel and does not adjust it.
Re-centring invalidates the archived bags for calibration: if it ever happens,
everything above becomes a methodology dry-run and all numbers must come from
fresh data. That is precisely why it waits until after 4a.

## Verify before anything else on the car

The plan's Stage 3 check, now that the gain has actually changed:

1. Straight line at servo centre — the offset is unchanged at 0.56, so this is
   a regression check rather than a new-value check.
2. Fixed-radius circles **both** directions — commanded radius vs measured. The
   right-hand one matters most: right is the limited direction and
   `max_steering:=0.34` still clips it (servo 0.9492 → 0.92). Left no longer
   clips (servo 0.1708, was 0.084 and marginal).

`max_steering` was deliberately **not** changed in this repo. 0.314 rad is the
symmetric no-clip value and it is what LUCIO's MPC has been told to use; the
asymmetry decision behind it is now settled (no recentre before 4a), so the
remaining question is only whether this stack should also move off 0.34 —
answer that against the Stage 4a bags, not before. Calibrated range is
+24.0° left / −18.0° right.

## Still open

- **Stage 3b decision gate.** Item 1 is **confirmed**: `ackermann_to_vesc.cpp:71`
  applies `erpm = gain*speed + offset` unconditionally with no dead-at-zero path,
  so `speed_to_erpm_offset` cannot be deadband compensation. Item 2 is confirmed
  *arithmetically* (562 erpm ceiling from rest vs 750–975 erpm ground breakaway)
  but **item 3 is not done** — nobody has checked the MPC's actual `max_accel`
  and control period against the second-hand 3.0/0.05. Item 4 (VESC firmware vs
  driver code) is untouched. Do not write the VESC brief before all four hold,
  and **ask the operator first** — the plan is explicit that this is a stop point.
- **The reply to LUCIO.** Enough is now known to answer most of their §6/§9:
  the `gz` sign (roll-180 mount, consumer must apply the TF — verified against
  `odometry/local`, not just by reading the TF), `k = 1.177` reproduced,
  measured left/right asymmetry ~5.7 % (more authority left), and that the
  throttle path contributes **< 20 ms** of their 80 ms transport delay. Their
  |scale| ≈ 1.00 for `gz` is reproduced here at 0.97–0.99 against
  `odometry/local`. Two answers still need Stage 0: the measured wheelbase, and
  the post-recalibration steering limit.

  **Sent as `LUCIO_REPLY.md`; answered; our round-3 reply is
  `LUCIO_REPLY_ROUND3.md`, drafted 2026-08-09 and not yet sent.** Both of the
  questions it asked back are now resolved — the `gz` fix is confirmed on their
  side (they derived it from our own `tf_static` and reproduced our scales to
  0.1 %), and the speed claim turned out not to be theirs at all. The two
  numbers still marked pending in the original reply — the measured wheelbase
  and the post-recalibration steering limit — are **still pending**, on Stage 0.
- **The horn-recentre decision is taken (operator, 2026-08-09): no recentre
  before Stage 4a.** This closes the deferred Stage 0 decision that has been
  open since 2026-08-07. Reasoning, recorded in `LUCIO_REPLY_ROUND3.md` §4:
  recentring moves `s₀` off the measured 0.5591/0.5610 and invalidates the
  archived bags as a baseline, which would destroy the k ≈ 0.96 acceptance test
  before it is run. **Do not touch the horn during the bench session** — the
  sweep measures travel, it does not adjust it. LUCIO gets symmetric 0.314 rad
  meanwhile; the asymmetric pair (+0.419 / −0.314) is deliberately withheld
  because +0.419 rests on an inherited `servo_min` never reached on this car.
  Revisit only after Stage 4a, with fresh bags.
- **Stages 4 and 5** — re-record (4a confirmation set, 4b excitation set incl.
  the new `26_sysid_drive.sh`), then the final fit. **`k ≈ 0.96`, band
  0.95–1.02**, on 4a is the acceptance test for the whole exercise (revised
  2026-08-08 with LUCIO; it was `k → 1.0`). Stage 4a must additionally record
  `vehicle/sensors/core` and include one run that drives into the steering
  limit both ways; everything else stays identical to the 2026-08-05 delivery
  — same three patterns, unmodified header stamps, one `map` frame,
  human-driven.

## The container runs FastRTPS, not CycloneDDS — unrelated to sysid, but real

Noticed 2026-08-07 by another agent and confirmed here inside
`jetson_container_20260807_085244`:

```
RMW_IMPLEMENTATION=            (empty)
CYCLONEDDS_URI=file:///mnt/shared_dir/cyclonedds_config_static.xml
ros2 doctor --report  ->  middleware name : rmw_fastrtps_cpp
```

`RMW_IMPLEMENTATION` is unset, so ROS 2 Humble falls back to its default
`rmw_fastrtps_cpp` and **the CycloneDDS config file is read by nothing**. This
matters beyond tidiness: the VSLAM frame-jitter fix was a CycloneDDS peer-config
change (adding `lo`), and the `CYCLONEDDS_PEERS.md` analysis assumes CycloneDDS
is in use. If the container has been on FastRTPS all along, that fix was never
active and whatever resolved the jitter was something else. **Do not fold this
into the sysid work** — it belongs to whoever owns the container image and the
DDS configuration. It did not affect anything here (both bench processes sat in
one container on domain 7, and discovery worked).

## Traps worth not rediscovering

- Run the analysis scripts with **both** overlays sourced —
  `/opt/ros/humble/setup.bash` *and* `/workspaces/f1tenth/install/setup.bash`;
  `vesc_msgs` is only in the second, and the failure is a bare `ModuleNotFoundError`.
- Bags live at `/mnt/shared_dir/rosbags/` **inside** the container, not at the
  host path `/mnt/f1tenth_ssd/shared_dir/rosbags/`. The rosbag2 error for a
  wrong path is a misleading "Could not load/open plugin with storage id 'mcap'".
- **Do not assume VSLAM is a usable speed reference.** On the short figure8 its
  tracking odometry covers ~10 s of a 65 s bag; `np.interp` extrapolated it flat
  and the fit returned a *sign-flipped* gain that looked plausible. (On the long
  bags it is fine, R² 0.986–0.988.) `fit_actuators.py` now selects on measured R²
  and coverage, but the general lesson is broader than this script.
- **`mapping_drive_145639` is not a drive.** The motor turned but the car never
  moved — rf2o and VSLAM both cap at ~0.01 m/s across 171 s. Wheels off the
  ground, or an aborted attempt. The fitter refuses it; that is correct.
- The four full-length bags are 17–28 GB each and take several minutes apiece to
  scan. Run them in the background, not on a foreground timeout.
- `pf_sweep_claude_0807` and `mpc_claude_0806` are other agents' containers.
  Analysis here ran read-only inside `jetson_container_20260807_085244` and
  started no ROS nodes, so no domain collision — keep it that way, or pick a
  non-colliding `ROS_DOMAIN_ID` checked from the **host**.
