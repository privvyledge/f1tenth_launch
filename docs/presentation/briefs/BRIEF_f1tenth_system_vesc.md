# Brief: f1tenth_system / vesc fork

**Read this first, then `PRESENTATION_PLAN.md` if you need the whole picture.**
Everything below that is marked *verbatim* is copied unchanged out of the plan —
it is not a paraphrase, because paraphrase is where constraints get dropped. If
this file and the plan ever disagree, the plan wins and this file is the bug.

Your working directory is `docs/presentation/` in `f1tenth_launch`.

---

## 0. What the plan says about this brief (§5, verbatim)

Every brief has the same shape: **slides you own** (IDs from §3), **what exists in your repo** that answers them (the brief lists what this scan found so the chat starts from evidence), **assets you must produce** (IDs and target paths), **conventions** (§1), **what you must not do** (fabricate media, quote numbers without a source, reference agent files on slides), **cut tagging** (tag each slide with `lab`/`sponsor`/`research` per §2a; return a ranked list of what your area can actually show today so the `sponsor` cut is built from real material), **research questions** (one or two for S99, with the evidence you already have), and **hand-back** (which files to edit, how to run `build.sh` to check).

### BRIEF_f1tenth_system_vesc.md
Owns 7.5, 7.6 and reviews 8.1-8.2 for accuracy. Found: `f1tenth_system` (`vesc_driver`, `vesc_ackermann` with `ackermann_to_vesc.cpp` and `vesc_to_odom.cpp`, `ackermann_mux`, `teleop_tools/joy_teleop` fork with the `default` pseudo-button and heartbeat, `f1tenth_stack/throttle_interpolator.py`), and the `vesc` fork (`humble-devel`) carrying `use_closed_loop_speed`, `speed_kp/ki`, `use_adaptive_ff`, `use_accel_ff`, `accel_to_erpm_gain`, `use_cmd_accel_rate_limit`. Must confirm or refute the template's "cascade PIDs: long_vel, steering -> ERPM, PWM" and draw FIG-LOWLEVEL from the code, not the template. Must state defaults and saturation limits.

---

## 1. Slides you own (§3, verbatim)

The rows below are copied unchanged from the plan's slide-by-slide table.
Legend: **T** = talk deck, **R** = reference deck only. "Source" is where the content
is already written down; "Assets" are IDs from §4. Owner defaults to the
f1tenth_launch chat unless stated.

### S70 Hierarchical control (T 5, R 3) — owner: control chat; 7.5-7.6 vesc chat

*Section file: `slides/70_control.md`*

| # | T/R | Slide | Content | Source | Assets |
|---|---|---|---|---|---|
| 7.5 | T | Low-level actuation chain | `ackermann_to_vesc`: erpm = 3750 v, servo = -1.1448 delta + 0.56, clamp [0.08, 0.92]; saturator; optional closed-loop speed PI (`speed_kp/ki`, anti-windup), adaptive FF, accel FF, rate limit, all default off; `throttle_interpolator` (fixed, optional). Owner confirms whether a steering PID exists (template: "confirm from my vesc fork") | `vesc.yaml`, `bringup.launch.py`, vesc fork | FIG-LOWLEVEL |
| 7.6 | R | Actuation lag | 60 ms delay + 40 ms first-order; throttle transport <20 ms; step-response plot | `SYSID_RESULTS.md` | CHART-LAG |

### S80 Vehicle interface and safety (T 4, R 2)

*Section file: `slides/80_vehicle_safety.md`*

| # | T/R | Slide | Content | Source | Assets |
|---|---|---|---|---|---|
| 8.1 | T | Every command passes two gates | Diagram: `teleop`/`drive`/`estop` -> `ackermann_mux` (255/100/10/1, timeouts 0.5/0.3/0.2/0.05 s) -> `ackermann_drive` -> `command_gate` -> `vehicle/ackermann_cmd` -> `ackermann_to_vesc` -> VESC | `mux.yaml`, `command_gate.yaml`, CLAUDE.md | FIG-CMDPATH (mermaid) |
| 8.2 | T | Deadman and handover | Heartbeat on `command_gate/heartbeat`, 0.5 s timeout; L1 manual, R1 autonomous handover; gate closes 0.5-0.63 s after joystick loss; commanded motion bounded to 0.2-0.35 s; never use PS in a deadman set. **Why the heartbeat has its own topic** (one line): the R1 handover silences `teleop` by design, so a `teleop`-based heartbeat would close the gate 1 s into every autonomous run | CLAUDE.md, `joy_teleop.yaml` | TABLE-SAFETY-TIMING, VID-SAFETY-MUX (R) |

8.1 and 8.2 are **written by the f1tenth_launch chat and reviewed by you**
for accuracy against the code — you own the correction, not the prose. Read them
in `slides/80_vehicle_safety.md` and return corrections in your hand-back.

**Draw FIG-LOWLEVEL from the code in the vesc fork, not from the architecture
diagram.** The `4_Control` Excalidraw note shows a steering PID and a 200 Hz
velocity PID; that is design intent. Slide 7.5 must show the chain as built,
with every optional stage's default state, and must confirm or refute the
template's claim of "cascade PIDs: long_vel, steering -> ERPM, PWM". State the
defaults and the saturation limits.

---

## 2. Where to write them

The scaffold has already created every one of these slides as a skeleton in
`slides/`, in plan order, with:

- the cut tag the scaffold derived from §2a (see §4 below — **check it**),
- a placeholder card for each asset ID the plan lists,
- an HTML comment holding the plan's `Content` and `Source` cells.

Replace the TODO comment with the real slide. Keep the `<!-- plan §3 row N -->`
marker so the integration chat can trace a slide back to the plan. Do not
renumber, add or drop slides without saying so in your hand-back.

---

## 3. Assets you must produce (from `ASSETS.md`)

Target paths are relative to `docs/presentation/`. Slides reference them from
`out/`, so an asset at `assets/figures/x.png` is `../assets/figures/x.png` in a
slide. **Any asset you cannot produce stays a placeholder card** — that is the
honest outcome, not a failure.

| ID | Kind | Status | Owner | Target path | Notes |
|---|---|---|---|---|---|
| FIG-LOWLEVEL | diagram | OWNER | vesc | `assets/figures/lowlevel.svg` | Drawn from the vesc fork's code, not the template. |

`ASSETS.md` also carries §4 of the plan verbatim, which is the authority on how
each one is produced. If you produce an asset, change its `Status` to `DONE` and
fill in the real path — `build.sh` then **fails** if a slide still cards it.

---

## 4. Cut tagging (§2a, verbatim)

### 2a. The three cuts

| Cut | Audience | Gets the core plus | Drops from the core | Target |
|---|---|---|---|---|
| `lab` | advisor and lab group (main) | design-choice slides with their reasons (3.3, 3.5, 8.1-8.2), sysid detail (9.3-9.5), Nav2 on the car (6.4) | nothing | 40-42 slides, 25 min |
| `sponsor` | external collaborators | demo videos (VID-*), status board up front (0.2), controller results (7.3-7.4), roadmap (10.1) | config tables (2.2, 2.5 detail, 3.3 detail), procedures (8.3), sysid method (9.2) | 30-34 slides, 20-25 min; **not presentable until the robot capture session (Phase F) is done** |
| `research` | committee-style | contributions framing on 0.2, the evaluation figures (3.4, 6.4, 9.3-9.5), plus section S99 | hardware photos beyond 1.1, procedures (8.3-8.4), software-stack table (0.4) | 36-40 slides |

**Core (all three cuts)**: 0.1, 0.2, 0.3, 1.1, 1.2, 2.1, 2.4, 2.6, 2.7, 3.1, 3.2, 3.4, 4.1, 4.2, 5.1, 5.3, 5.4, 6.1, 6.3, 7.1, 7.2, 7.3, 7.5, 8.1, 9.1, 10.1. Everything else tagged T in §3 belongs to one or two cuts as the table says; the section owner tags it, the integration chat arbitrates the counts.

**Why cuts rather than one deck**: a single 25-minute talk cannot be a status report, a showcase and a rationale record at once. The core *is* the honest status report; the showcase and the rationale are what the `sponsor` and `lab` cuts add. **Pushback recorded**: I only know this repo, so which controller and perception results deserve showcase slots is for the owning chats to say; their briefs ask for a ranked list of what they can actually show today.

Each of your slides already carries a `<!-- cut: ... -->` comment. The scaffold
derived it mechanically from the core list and the drop rules above; where §2a
did not say, it tagged `lab` only and left a NOTE in the slide. **Confirm or fix
every tag in your sections**, and say in your hand-back which ones you changed.

A slide with no cut tag is reference-only: it appears in `./build.sh full` and
nowhere else. That is the right home for detail that would otherwise bloat the
talk.

**Also return a ranked list of what your area can actually show today**, best
first, so the `sponsor` cut gets built out of real material rather than
aspirations.

---

## 5. Conventions (§1, verbatim)

Conventions the scaffold chat must implement (§6, Chat A):

- **Section file** = Marp slides separated by `---`; only `00_overview.md` carries the Marp front-matter (`marp: true`, `theme: f1tenth`, `paginate: true`).
- **Cut tags**: a slide carries one HTML comment listing the cuts it belongs to: `<!-- cut: lab sponsor research -->`, `<!-- cut: lab research -->`, `<!-- cut: sponsor -->`, etc. A slide with no tag is reference-only. `build.sh lab|sponsor|research` splits on slide boundaries and keeps slides whose tag lists that cut; `build.sh full` keeps everything. In §3 the T/R column means "in at least one cut" / "reference-only", and §2a gives the membership rules. `build.sh` prints the slide count per cut so the 25-minute budget is visible.
- **Placeholder card**: a fenced block rendered by the theme as a dashed box:
  ```
  > [!PLACEHOLDER VID-LOC-LOOP]
  > Trailing odometry/local (blue) vs odometry/global (green) over the map, one lap, 40 s.
  ```
  The ID must exist in `ASSETS.md`. The build fails if an ID is used that the register does not list, or if a listed asset with status `DONE` is still shown as a card.
- **Number sourcing**: every quantitative claim on a slide has a speaker note (`<!-- src: CLAUDE.md §ekf_odom; measured 2026-08-06 -->`) naming the file it came from and the measurement date. No number without a source; unknowns are written `TBD` with the measurement that would settle them. The integration chat greps for digits without a `src:` note.
- **Every slide answers one question**, title is that answer, not a topic ("rf2o under-reports distance by 5-8 %", not "Speed calibration").
- **Rates, latency, resolution, defaults/min/max** go in tables, never in prose bullets.
- **Never** reference agent files (`CLAUDE.md`, `PLAN.md`, bug IDs) *on a slide*. Speaker notes may cite them.

`build.sh` enforces three of these mechanically and will fail the build:

| Check | Failure |
|---|---|
| Placeholder ID not in `ASSETS.md` | error |
| Card shown for an asset marked `DONE` | error |
| Agent file or bug ID in a **slide body** (comments are exempt) | error |
| Number on a slide with no `src:` note on that slide | warning; `STRICT=1 ./build.sh` makes it an error |

## 6. What you must not do

- **No fabricated media.** No stock, staged, mocked-up or synthetic figures,
  screenshots or video. If it has not been captured, it is a placeholder card.
- **No number without a source.** Every quantity carries a `src:` note naming
  the file it came from and the date it was measured. A number you cannot source
  is written `TBD`, together with the measurement that would settle it.
- **No agent files on a slide.** Never name `CLAUDE.md`, `PLAN.md`, a handoff
  doc or a bug ID in slide text. Speaker notes and HTML comments may cite them.
- **No 2024 material without its label.** Any reused February 2024 media carries
  a visible `<span class="y2024">2024 stack</span>`. 2024 *software*
  descriptions are treated as stale and are not copied at all.
- **Do not edit another owner's section file.** The sections are disjoint so the
  merges stay trivial.

---

## 7. Research questions for S99 (§3, verbatim)

Contribute one or two candidate questions from your area, phrased as a question
this platform can answer with an experiment we can actually run, each with the
evidence already in hand. The seeds below are from the plan; accept, reject or
replace them.

### S99 Research questions (research cut only; T 3-4)

Each owning brief contributes one or two candidate questions from its area, phrased as a question the platform can answer with an experiment we can run. Seeds from this repo, for the owners to accept, reject or replace:

| # | Slide | Candidate question | Evidence already in hand |
|---|---|---|---|
| 11.1 | What this platform can measure | One slide: the sensor set, the three independent ground-truth references (tape, VSLAM, RTABMap loop closure), the bag tooling, and the cost of a run | S90, `scripts/analysis/` |
| 11.2 | Estimation | Can actuator calibration (gain, offset, deadband) be identified from ordinary driving with enough accuracy to replace bench measurement, given that three driven fits disagreed with the bench value of the servo offset by up to 0.009 servo units (47 cm of drift over 5.5 m)? Which odometry reference is trustworthy for scale, given rf2o at -6 % and VSLAM at +1 % against tape? | 9.3, 9.4, 3.4 |
| 11.3 | Control | Speed-floor-aware tracking: the car cannot move below 0.20-0.26 m/s, and the goal approach stalled at 0.269 m/s. How should MPC or RPP terminal behaviour account for a hard breakaway speed? Obstacle-aware MPC on a platform with ~100 ms actuation lag: what horizon and rate does the lag force? | 6.4, 7.6, control brief |
| 11.4 | Perception and fusion | Which detector (laser, pointcloud, image + depth) gives obstacle states good enough for the MPC at what rate on the Orin, and does fusing them help? | perception brief |

**Cut totals (estimates, integration chat confirms from `build.sh`)**: `lab` 42, `sponsor` 32, `research` 38. Reference-only: 26. Full deck: 72.

---

## 8. Hand-back

1. Files you edited: `slides/<yours>`, `ASSETS.md` (status changes only), and
   any real asset files you produced.
2. Run the build and paste the output:

   ```bash
   cd docs/presentation
   ./build.sh full md          # assemble + run every check, no renderer
   ./build.sh lab              # your slides in the main cut, rendered
   STRICT=1 ./build.sh full md # numbers-need-sources check as an error
   ```

3. State plainly:
   - which slides you filled, and any you added, dropped or renumbered;
   - which cut tags you changed, and why;
   - which assets are now `DONE`, and which stayed cards and why;
   - **which claims have driven the real car** versus simulation, replay or
     bench — say it in those words;
   - anything in the plan you believe is wrong. Push back in the hand-back
     rather than silently working around it.
