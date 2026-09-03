# Brief: perception (autodriver_*)

**Read this first, then `PRESENTATION_PLAN.md` if you need the whole picture.**
Everything below that is marked *verbatim* is copied unchanged out of the plan —
it is not a paraphrase, because paraphrase is where constraints get dropped. If
this file and the plan ever disagree, the plan wins and this file is the bug.

Your working directory is `docs/presentation/` in `f1tenth_launch`.

---

## 0. What the plan says about this brief (§5, verbatim)

Every brief has the same shape: **slides you own** (IDs from §3), **what exists in your repo** that answers them (the brief lists what this scan found so the chat starts from evidence), **assets you must produce** (IDs and target paths), **conventions** (§1), **what you must not do** (fabricate media, quote numbers without a source, reference agent files on slides), **cut tagging** (tag each slide with `lab`/`sponsor`/`research` per §2a; return a ranked list of what your area can actually show today so the `sponsor` cut is built from real material), **research questions** (one or two for S99, with the evidence you already have), and **hand-back** (which files to edit, how to run `build.sh` to check).

### BRIEF_perception.md
Owns S50. Found locally: `autodriver_image_object_detection` (launch `bringup`, `components/stream`, `components/object_detection`; branch `batch_inference_and_tracking`), `autodriver_pointcloud_object_detection`, `autodriver_pointcloud_preprocessor`, `autodriver_fake_obstacle_publisher`, `autodriver_icp_localizer`; `ros_multi_object_tracker` under `Projects/autodriver/autodriver_perception/`. `autodriver_laser_segmentation` **is on this machine but not under `~/f1tenth_ws/src`**; its path (and the current paths of the other autodriver repos, which may be newer than the workspace copies) come from the user, who keeps them as aliases at the bottom of `~/.bashrc`. The brief must record the resolved paths once given. Must state for each node: input topics, output type, measured rate on the Orin (or TBD), and whether it is wired into any launch file in `f1tenth_launch` (today: none are in bringup; `nav2_perception.launch.py` is standalone).

---

## 1. Slides you own (§3, verbatim)

The rows below are copied unchanged from the plan's slide-by-slide table.
Legend: **T** = talk deck, **R** = reference deck only. "Source" is where the content
is already written down; "Assets" are IDs from §4. Owner defaults to the
f1tenth_launch chat unless stated.

### S50 Perception (T 4, R 2) — owner: perception chat

*Section file: `slides/50_perception.md`*

| # | T/R | Slide | Content | Source | Assets |
|---|---|---|---|---|---|
| 5.1 | T | Obstacle detection pipeline | Excalidraw: 2D detection + depth projection, pointcloud detection, laser detection, merger, tracker | Obsidian `2B_ObstacleDetection` | FIG-ARCH-03 |
| 5.2 | T | LaserScan-based detection | `autodriver_laser_segmentation` (repo location TBD, D4): method, rate, output type | owning repo | FIG-PERC-LASER |
| 5.3 | T | Pointcloud pipeline | `autodriver_pointcloud_preprocessor` filters then `autodriver_pointcloud_object_detection`; rates on Orin | owning repos | FIG-PERC-CLOUD |
| 5.4 | T | Image and 3D | `autodriver_image_object_detection`: YOLOv8 (ultralytics 8.3.86, onnxruntime-gpu), batch inference + tracking; 2D box to 3D via aligned depth; `ros_multi_object_tracker` | owning repos | FIG-PERC-IMAGE |
| 5.5 | R | Costmaps as perception | Nav2 obstacle + inflation layers, `nonpersistent_voxel_layer`; sample costmap | `nav2_params.yaml`, `data/maps/sample_costmap.png` | FIG-COSTMAP |
| 5.6 | R | Perception demo | 2024 clip if allowed (D2); otherwise placeholder | `docs/perception_and_gotogoal_planning.mp4` | VID-PERC-2024 |

**One path is resolved; get the rest from the user before you start.** The
workspace copies under `~/f1tenth_ws/src` may be older than the trees the user
actually works in — they keep the current ones as aliases at the bottom of
`~/.bashrc`. **Record each path in the table below as you get it**, so the next
chat does not have to ask again.

Note the name: the plan calls it `autodriver_laser_segmentation`, but the repo
on disk is **`autodriver_laser_object_segmentation`**. The path below is
authoritative; use the real directory name on the slide.

Resolved paths:

| Repo | Path | Branch | Date checked |
|---|---|---|---|
| `autodriver_laser_object_segmentation` | `/mnt/c/Users/boluo/OneDrive - Florida State University/Projects/autodriver/autodriver_perception/autodriver_laser_object_segmentation` (given by the user 2026-09-02) | | |
| `autodriver_image_object_detection` | | | |
| `autodriver_pointcloud_object_detection` | | | |
| `autodriver_pointcloud_preprocessor` | | | |
| `autodriver_fake_obstacle_publisher` | | | |
| `ros_multi_object_tracker` | | | |

For **each** node, the slide must state: input topics, output message type,
measured rate on the Orin (or `TBD` with the measurement that would settle it),
and whether it is wired into any launch file in `f1tenth_launch`. Today none are
in bringup and `nav2_perception.launch.py` is standalone — say so plainly rather
than implying integration that does not exist.

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
| FIG-COSTMAP | figure | EXISTS | perception | `assets/figures/costmap.png` | `data/maps/sample_costmap.png` — check its date; if stale it becomes ROBOT. |
| FIG-PERC-LASER | figure | OWNER | perception | `assets/figures/perc_laser.png` | — |
| FIG-PERC-CLOUD | figure | OWNER | perception | `assets/figures/perc_cloud.png` | — |
| FIG-PERC-IMAGE | figure | OWNER | perception | `assets/figures/perc_image.png` | — |
| VID-PERC-2024 | video | EXISTS-2024 | perception | `docs/perception_and_gotogoal_planning.mp4` | The perception chat decides whether it still represents the pipeline. Label it 2024. |

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
