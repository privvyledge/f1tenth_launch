# Brief: robot capture session (gosling1)

**Read this first, then `PRESENTATION_PLAN.md` if you need the whole picture.**
Everything below that is marked *verbatim* is copied unchanged out of the plan —
it is not a paraphrase, because paraphrase is where constraints get dropped. If
this file and the plan ever disagree, the plan wins and this file is the bug.

Your working directory is `docs/presentation/` in `f1tenth_launch`.

---

## 0. What the plan says about this brief (§5, verbatim)

Every brief has the same shape: **slides you own** (IDs from §3), **what exists in your repo** that answers them (the brief lists what this scan found so the chat starts from evidence), **assets you must produce** (IDs and target paths), **conventions** (§1), **what you must not do** (fabricate media, quote numbers without a source, reference agent files on slides), **cut tagging** (tag each slide with `lab`/`sponsor`/`research` per §2a; return a ranked list of what your area can actually show today so the `sponsor` cut is built from real material), **research questions** (one or two for S99, with the evidence you already have), and **hand-back** (which files to edit, how to run `build.sh` to check).

### BRIEF_robot_capture.md
Not a repo, a lab session. Lists every `ROBOT` asset in §4 with the exact command (from `demo_shot_list.md` and `scripts/analysis/`), the bags to use (`armA_loop`, `armA_loop2`, `armA_straight_5m`, `armA_steer_sweep`, `nav2_drive4` on the SSD), the RViz configs (`rviz/demo_*.rviz`), and the operator rules already learned (operator launches bringup, agent checks health; record with `BAG_TOPIC_SET=sysid`; leave 2 s still at clip ends). Also TABLE-PACKAGES. Order: offline charts from existing bags first (no driving), then parked clips, then the one driving session for VID-NAV2-LIVE and VID-LOC-LOOP.

---

## 1. Slides you own

**None.** This brief owns no slides. It owns the `ROBOT` assets that every other
brief is carding, so run it **after** the section chats have said which cards
they actually need.

### Order of work (charts first, driving last)

1. **Offline charts from bags already on the SSD** — no driving, no risk:
   CHART-RATES, CHART-CLOSURE, CHART-STEER, CHART-SPEED, CHART-LAG,
   CHART-NAV2-APPROACH, FIG-MAP-COMPARE. Bags: `armA_loop`, `armA_loop2`,
   `armA_straight_5m`, `armA_steer_sweep`, `nav2_drive4`.
2. **Inventory** — TABLE-PACKAGES, inside the container.
3. **Parked clips** — the sensor set (VID-SENS-*), FIG-TF-TREE, VID-SAFETY-MUX.
4. **One driving session** — VID-NAV2-LIVE, VID-LOC-LOOP, VID-PF-CONVERGE.

### Operator rules that are already established

- **The operator launches bringup; the agent checks health.** Print the command
  for the operator to run; do not launch it over SSH.
- **Record with `BAG_TOPIC_SET=sysid`.** The image topic set starves the visual
  odometry, silently, and the rate check still reads healthy — verify a bag by
  message *count*, not rate.
- **Leave 2 s of stillness at both ends of every clip** so it can be trimmed.
- RViz configs live in `rviz/` (`demo_sensors.rviz`, `demo_localization.rviz`,
  `demo_nav2.rviz`); shot-by-shot commands are in `docs/demo_shot_list.md`.
- Keep the container warm between tests — bring-up costs about 8 minutes.

(The full `ROBOT` asset list is §3 below.)

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
| TABLE-PACKAGES | table | ROBOT | capture | inline | In the container on gosling1: `ros2 pkg list`, `apt list --installed 'ros-humble-*'`, `pip list`. Feeds 0.4. |
| CHART-RATES | chart | ROBOT | capture | `assets/figures/rates.png` | `scripts/analysis/bag_stats.py` on a `sysid` bag (`armA_loop2`) -> CSV -> matplotlib. |
| CHART-CLOSURE | chart | ROBOT | capture | `assets/figures/localization/closure.png` | `scripts/analysis/plot_localization.py <armA_loop2> --map data/maps/20260805/rtabmap_2d_final.yaml --out docs/presentation/assets/figures/localization/`; also yields `summary.md`. |
| CHART-STEER | chart | ROBOT | capture | `assets/figures/steer.png` | `fit_actuators.py` on `armA_loop`, `armA_loop2`, `armA_steer_sweep`; add `--save-fig` if absent. |
| CHART-SPEED | chart | ROBOT | capture | `assets/figures/speed.png` | `fit_actuators.py` on `armA_straight_5m` + the tape run. |
| CHART-LAG | chart | ROBOT | capture | `assets/figures/lag.png` | Step response; feeds both 7.6 and 9.5. |
| CHART-NAV2-APPROACH | chart | ROBOT | capture | `assets/figures/nav2_approach.png` | Distance-to-goal and commanded speed vs time from the `nav2_drive4` bag. |
| FIG-MAP-COMPARE | figure | ROBOT | capture | `assets/figures/map_compare.png` | Needs the SLAM Toolbox grid from `40_build_map_offline.sh --mode both`. |
| VID-SENS-RGB | video | ROBOT | capture | `assets/video/VID-SENS-RGB.mp4` | `docs/demo_shot_list.md` §1, 10-20 s. |
| VID-SENS-DEPTH | video | ROBOT | capture | `assets/video/VID-SENS-DEPTH.mp4` | shot list §1. |
| VID-SENS-CLOUD | video | ROBOT | capture | `assets/video/VID-SENS-CLOUD.mp4` | shot list §1. |
| VID-SENS-LIDAR | video | ROBOT | capture | `assets/video/VID-SENS-LIDAR.mp4` | shot list §1. The one sensor clip the talk shows. |
| VID-SENS-IMU | video | ROBOT | capture | `assets/video/VID-SENS-IMU.mp4` | shot list §1. |
| VID-LOC-LOOP | video | ROBOT | capture | `assets/video/VID-LOC-LOOP.mp4` | shot list §2. Needs a driving session. |
| VID-PF-CONVERGE | video | ROBOT | capture | `assets/video/VID-PF-CONVERGE.mp4` | shot list §2. |
| VID-NAV2-LIVE | video | ROBOT | capture | `assets/video/VID-NAV2-LIVE.mp4` | shot list §4.3; requires a driving session. |
| VID-SAFETY-MUX | video | ROBOT | capture | `assets/video/VID-SAFETY-MUX.mp4` | shot list §5. |

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
