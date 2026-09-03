# Brief: trajectory_following_ros2 (control)

**Read this first, then `PRESENTATION_PLAN.md` if you need the whole picture.**
Everything below that is marked *verbatim* is copied unchanged out of the plan —
it is not a paraphrase, because paraphrase is where constraints get dropped. If
this file and the plan ever disagree, the plan wins and this file is the bug.

Your working directory is `docs/presentation/` in `f1tenth_launch`.

---

## 0. What the plan says about this brief (§5, verbatim)

Every brief has the same shape: **slides you own** (IDs from §3), **what exists in your repo** that answers them (the brief lists what this scan found so the chat starts from evidence), **assets you must produce** (IDs and target paths), **conventions** (§1), **what you must not do** (fabricate media, quote numbers without a source, reference agent files on slides), **cut tagging** (tag each slide with `lab`/`sponsor`/`research` per §2a; return a ranked list of what your area can actually show today so the `sponsor` cut is built from real material), **research questions** (one or two for S99, with the evidence you already have), and **hand-back** (which files to edit, how to run `build.sh` to check).

### BRIEF_trajectory_following.md
Owns 6.2, 6.5, 7.1-7.4, 7.7. Found locally: `~/f1tenth_ws/src/trajectory_following_ros2` (stale, Jan 2025) with `coupled_kinematic_{acados,casadi,do_mpc}.py`, `ackermann_purepursuit.py`, `purepursuit/`, `waypoint_recorder.py`, `waypoint_loader.py`, `twist_to_ackermann_drive.py`, `simulator/`, `launch/mpc.launch.py`, `launch/trajectory.launch.py`. The owning chat works from the current branch (`refactor/unify-backends` carries the 2026 steering fix). The MPC is the flagship: it does waypoint following from recorded waypoints, go-to-goal, and obstacle avoidance, and the brief asks for all three to be shown with their real status. Nav2's MPPI is secondary (one table row, one reference slide). Must say plainly which controllers and modes have driven the car and which have only run in simulation or replay, and where the existing rerun/live recordings are. Steering bound ±0.314 rad and the LUCIO `delta_bound` relationship come from this repo's CLAUDE.md notes.

---

## 1. Slides you own (§3, verbatim)

The rows below are copied unchanged from the plan's slide-by-slide table.
Legend: **T** = talk deck, **R** = reference deck only. "Source" is where the content
is already written down; "Assets" are IDs from §4. Owner defaults to the
f1tenth_launch chat unless stated.

### S60 Planning and Nav2 (T 4, R 2)

*Section file: `slides/60_planning_nav2.md`*

| # | T/R | Slide | Content | Source | Assets | Owner |
|---|---|---|---|---|---|---|
| 6.2 | T | Waypoint recording and loading | `waypoint_recorder.py` / `waypoint_loader.py`: format, frame, how a figure-8 was recorded | trajectory_following_ros2 | FIG-WAYPOINTS | control |
| 6.5 | R | Go-to-goal: Nav2 vs the MPC node | The two go-to-goal paths (Nav2 `NavigateToPose`; the MPC node's goal mode), their interfaces, and which has driven the car (the Nav2 run in 6.4 is the only one documented in this repo; the control chat states the MPC node's status) | trajectory_following_ros2, CLAUDE.md | — | control |

### S70 Hierarchical control (T 5, R 3) — owner: control chat; 7.5-7.6 vesc chat

*Section file: `slides/70_control.md`*

| # | T/R | Slide | Content | Source | Assets |
|---|---|---|---|---|---|
| 7.1 | T | Control architecture | Excalidraw: high-level controller, lateral/longitudinal low-level, rates 100/200/16000 Hz | Obsidian `4_Control` | FIG-ARCH-05 |
| 7.2 | T | High-level controllers | Table: **MPC from `trajectory_following_ros2`** (acados / casadi / do_mpc backends, coupled kinematic model; modes: waypoint following, go-to-goal, obstacle avoidance), pure pursuit (same repo), Nav2 RPP, Nav2 MPPI (secondary, evaluated), joystick. Columns: rate, horizon, model, constraints (delta ±0.314 rad, speed bounds), obstacle-aware, has driven the car | trajectory_following_ros2, `nav2_params.yaml` | TABLE-CONTROLLERS |
| 7.3 | T | MPC: waypoint following | Offline figure-8 and loop runs (rerun recordings) and live drives; tracking error chart; acceptance `k ≈ 0.96`; LUCIO ego-MPC one line | control chat, `SYSID_RESULTS.md` | VID-MPC-RERUN, VID-MPC-LIVE, CHART-MPC-TRACKING |
| 7.4 | T | MPC: go-to-goal and obstacle avoidance | How obstacles enter the problem (constraints or cost), what the obstacle source is today (`autodriver_fake_obstacle_publisher` vs perception), RViz and live placeholders with and without obstacles | control chat | VID-MPC-OBST-RVIZ, VID-MPC-OBST-LIVE |
| 7.4b | R | Nav2 MPPI | Configuration tried, result, why it is secondary | control chat, `nav2_params.yaml` | — |
| 7.7 | R | Pure pursuit details | lookahead, gains | control chat | — |

> **Known plan discrepancy (left uncorrected on purpose, 2026-09-02).** S70's header reads "T 5, R 3" but the section lists **four** R slides (7.4b, 7.6, 7.7, 7.8).
> The scaffold built from the **rows**, which is what the table above shows.
> Phase G arbitrates; do not renumber anything to make the header add up.


7.5 (low-level actuation chain) and 7.6 (actuation lag) belong to the
**vesc / f1tenth_system chat**, and 7.8 (joystick as a controller) to the
f1tenth_launch chat. They sit in the same section file, `slides/70_control.md` —
edit only your own rows, and coordinate through the integration chat if a claim
of yours depends on theirs.

**The MPC is the flagship of this section.** All three of its modes — waypoint
following, go-to-goal, obstacle avoidance — get shown with their real status,
and the brief asks you to be explicit about which of them has driven the car.

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
| TABLE-CONTROLLERS | table | OWNER | control | inline | — |
| CHART-MPC-TRACKING | chart | OWNER | control | `assets/figures/mpc_tracking.png` | Cross-track and speed error from the figure-8 / loop runs. |
| FIG-WAYPOINTS | figure | OWNER | control | `assets/figures/waypoints.png` | — |
| VID-MPC-RERUN | video | OWNER | control | `assets/video/VID-MPC-RERUN.mp4` | Control chat lists what already exists. |
| VID-MPC-LIVE | video | OWNER | control | `assets/video/VID-MPC-LIVE.mp4` | — |
| VID-MPC-OBST-RVIZ | video | OWNER | control | `assets/video/VID-MPC-OBST-RVIZ.mp4` | — |
| VID-MPC-OBST-LIVE | video | OWNER | control | `assets/video/VID-MPC-OBST-LIVE.mp4` | — |

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
