# Asset register

Every figure, chart, table, photo and video that any slide uses. **A slide may
only show a placeholder card for an ID listed here**, and `build.sh` fails if it
finds a card whose ID is missing from this file, or a card for an asset marked
`DONE`.

Two parts:

1. **The enumerated table** below (between the `assets-table` markers) is what
   `deck.py` parses. One row per ID, with the target path a section chat should
   write the file to.
2. **§4 of the plan, copied verbatim** at the bottom. It is the authority on how
   each asset is produced. Where the enumerated table had to split a grouped
   row into individual IDs, or pick an owner, the row says so in Notes — the
   verbatim text wins if the two ever disagree.

## Open items

Unticked on purpose. None of these blocks a section chat from writing slides.

- [ ] **Replace the seven Excalidraw architecture figures.** The PNGs in
      `assets/figures/arch/` are *references the planning chat worked from*, not
      deliverables — their `@Hz` annotations are design targets, not measurements
      (§4 note (a)). They are embedded in slides 0.3, 2.1, 2.3, 3.1, 5.1, 6.1 and
      7.1 so the deck reads end-to-end today. Replace with accurate SVG / mermaid
      / TikZ; the user will supply SVG exports separately.
- [ ] **`FIG-ELEC` has no source at all.** Nothing to export from — it must be
      drawn. Mermaid, per the 2026-09-02 decision.
- [ ] **Cut counts are over the §2a targets** (lab 43 / sponsor 36 / research 41
      against 40-42 / 30-34 / 36-40). Deliberately deferred to Phase G, which
      trims once the slides carry real content.
- [ ] **Plan §3's per-section T/R headers disagree with their own rows** in two
      places: S20 says "T 5" and lists six T slides; S70 says "R 3" and lists
      four R slides. Noted, not corrected — the rows are what the scaffold built
      from. Left for Phase G.
- [ ] **Write slides from the config files, not from the prose summary.** Four
      claims in `CLAUDE.md` were stale against the tree when the f1tenth_launch
      sections were written on 2026-09-02, and each one would have put a wrong
      number on a slide. They are logged in `.wolf/buglog.json` as `bug-274`
      to `bug-277`:
      - `bug-274` — **not a doc problem, a live defect.** `min_turning_r` is
        described as inert because only the MPPI copy was checked. The *active*
        planner, `SmacPlannerHybrid` under `planner_server/GridBased`, carries
        its own `minimum_turning_radius: 0.462` (`config/nav2_params.yaml:620`)
        plus `analytic_expansion_max_length: 3.465` derived from it. At the car's
        measured ±0.314 rad the true minimum is **0.788 m**, and the driven
        full-lock radius is **0.814 m** — so the planner routinely plans turns
        the car cannot execute, which matches the 12 % steering saturation seen
        on a real drive. **Left unchanged on purpose**: it alters planner
        behaviour and needs a drive to validate. Do not "fix" it from a desk.
      - `bug-275` — the rf2o rejection gates in `ekf_odom.yaml` are **3.0**, not
        the 5.0 the prose records (and VSLAM carries an unrecorded 2.0 gate).
      - `bug-276` — the RealSense **decimation filter is not enabled**; the whole
        post-processing block is commented out and no launch file sets it.
      - `bug-277` — `Grid/RayTracing` is **never passed** to RTABMap; that
        argument is commented out in `3d_mapping.launch.py`.
- [ ] **Run `./check_overflow.mjs` before handing a section back.** Marp clips a
      slide at 720 px silently — no warning, no error, the PDF just loses the
      bottom. 21 of the 70 slides overflowed on first write. Usage is in the
      script's header. Sizing rule that avoids the loop: usable content height is
      about 616 px, minus ~72 for a one-line `h2` or ~113 for two lines; size
      figures with `h:` rather than `w:`, and put a near-square diagram in
      `<div class="split">` instead of shrinking it.

## Status codes

From plan §4:

- `EXISTS` — file is in the repo or on OneDrive (may still need cropping/copying)
- `OFFLINE` — renderable on this machine, no robot needed
- `ROBOT` — needs gosling1 and the lab
- `USER` — only the user can produce it
- `OWNER` — another chat's repo produces it
- `TBD` — the measurement is not yet designed

Added by the scaffold, because the build needs them:

- `DONE` — the final file is in place at the target path and must be embedded,
  not carded. `build.sh` **fails** on a card for a `DONE` asset. **No asset carries
  it today**: the arch PNGs were `DONE` until 2026-09-02, when they were reclassified
  as reference exports (see Open items), so nothing is currently frozen as final.
- `EXISTS-2024` — a February 2024 file that may be reused, and which **must**
  carry a visible "2024 stack" label on the slide (plan §7.6, D2). Use the
  theme's `<span class="y2024">2024 stack</span>`.

## How a slide uses an asset

```markdown
<!-- not produced yet: card, with the ID -->
> [!PLACEHOLDER CHART-CLOSURE]
> Bar chart of each estimator's error after a loop that returns to start.

<!-- produced: embed it, and drop the card -->
![w:900](../assets/figures/localization/closure.png)
```

Paths in slides are relative to `out/`, because that is where the assembled
deck is written — so an asset at `assets/figures/x.png` is `../assets/figures/x.png`
from a slide. Video is never committed; see `assets/video/README.md`.

<!-- assets-table:begin -->

| ID | Kind | Status | Owner | Target path | Notes |
|---|---|---|---|---|---|
| FIG-ARCH-00 | figure | USER | user | `assets/figures/arch/0_SystemArchitecture.png` | **Reference export, not final** (see Open items). PNG received 2026-09-02. Review notes (a)-(f) in §4 apply. |
| FIG-ARCH-01 | figure | USER | user | `assets/figures/arch/1_Sensing.png` | **Reference export, not final** (see Open items). Portrait 932x1998; **cropped above the dashed line to `1_Sensing_crop.png` on 2026-09-02**, which is what slide 2.1 embeds. Original untouched. See §4 notes (b), (f). |
| FIG-ARCH-01A | figure | USER | user | `assets/figures/arch/1A_StereoVision.png` | **Reference export, not final** (see Open items). Embeds a MathWorks figure: attribute or crop. See §4 note (c). |
| FIG-ARCH-02 | figure | USER | user | `assets/figures/arch/2A_Localization.png` | **Reference export, not final** (see Open items). SVG export still wanted. |
| FIG-ARCH-03 | figure | USER | user | `assets/figures/arch/2B_ObstacleDetection.png` | **Reference export, not final** (see Open items). SVG export still wanted. |
| FIG-ARCH-04 | figure | USER | user | `assets/figures/arch/3_Planning.png` | **Reference export, not final** (see Open items). SVG export still wanted. |
| FIG-ARCH-05 | figure | USER | user | `assets/figures/arch/4_Control.png` | **Reference export, not final** (see Open items). Shows design intent (steering PID, 200 Hz velocity PID) and no mux/gate. See §4 notes (d), (e). |
| PHOTO-CAR | photo | DONE | B1 | `assets/photos/car.jpg` | Produced 2026-09-02 from the release doc's assembled-car photo. Current hardware, so no 2024 label. |
| PHOTO-COMPONENTS | photo | DONE | B1 | `assets/photos/components/` | Produced 2026-09-02: six component shots (computer, ESC, camera, LiDAR, chassis, controller), laid out inline on slide 1.1. |
| PHOTO-WIRING | photo | DONE | B1 | `assets/photos/wiring.jpg` | Produced 2026-09-02 as one image: the computer supply and the ESC XT90 connection side by side, from two current-hardware stills. |
| PHOTO-ASSEMBLY | photo | DONE | B1 | `assets/photos/assembly/` | Produced 2026-09-02: the seven numbered assembly steps, downscaled. |
| PHOTO-DUALSENSE | photo | **DONE** | B1 | `assets/photos/dualsense_labelled.png` | **File produced 2026-09-02 and embedded on 1.6 and 8.4.** Flipped EXISTS -> DONE 2026-09-03: the only remaining card was slide 7.8, and that plan row is retired as a duplicate of 8.4 (Phase-G arbitration; see the note at the end of `slides/70_control.md`). |
| FIG-ELEC | figure | TBD | user | `assets/figures/electrical.svg` | Does not exist. **Draw in mermaid** (SVG or TikZ if it outgrows mermaid), from Release doc §Wiring — decided 2026-09-02, plan §8 open item. Card until then. |
| FIG-TF-TREE | figure | DONE | capture / B1 | `assets/figures/tf_tree.svg` | Produced 2026-09-02 as the sanctioned offline alternative: mermaid from static_transformations.launch.py + the URDF. Source `assets/figures/src/tf_tree.mmd`. A `view_frames` capture may still replace it. |
| FIG-LAUNCHTREE | diagram | DONE | B1 | `assets/figures/launch_tree.svg` | Produced 2026-09-02. Source `assets/figures/src/launch_tree.mmd`; re-render with `assets/figures/src/render.sh`. |
| FIG-CMDPATH | diagram | DONE | B1 | `assets/figures/cmd_path.svg` | Produced 2026-09-02. Source `assets/figures/src/cmd_path.mmd`. **Still wants the vesc chat's review.** |
| FIG-EKF-INPUTS | diagram | DONE | B2 | `assets/figures/ekf_inputs.svg` | Produced 2026-09-02 from ekf_odom.yaml as read, not from the prose summary (the rf2o gate is 3.0, not 5.0). Source `assets/figures/src/ekf_inputs.mmd`. |
| FIG-LOWLEVEL | diagram | OWNER | vesc | `assets/figures/lowlevel.svg` | **Mermaid** (decided 2026-09-02). Drawn from the vesc fork's code, not the template. |
| FIG-NAV2-BT | diagram | DONE | B2 | `assets/figures/nav2_bt.svg` | Produced 2026-09-02 from config/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml. Source `assets/figures/src/nav2_bt.mmd`. |
| TABLE-REPOS | table | DONE | B1 | inline, slide 0.4 | Written 2026-09-02 from f1tenth.repos (21 entries, 10 rows shown). |
| TABLE-CAMERA | table | DONE | B1 | inline, slide 2.2 | Written 2026-09-02. Measured column filled 2026-09-03 from a 60 s live window (`assets/data/rates_live_20260903_nocloud_baseline.json`), not from a chart; the CHART-RATES card on 2.2 is retired. Cloud row corrected: it is the RealSense SDK cloud and it is OFF by default. |
| TABLE-IMU | table | DONE | B1 | inline, slide 2.5 | Written 2026-09-02. |
| TABLE-ODOM | table | DONE | B2 | inline, slide 3.2 | Written 2026-09-02. |
| TABLE-CONTROLLERS | table | DONE | control | inline, slide 7.2 | Written 2026-09-03 from the config files as read, not from a prose summary. The two Nav2 rows are filled from `config/nav2_params.yaml`; the f1tenth_launch chat owns them and should confirm. |
| TABLE-SAFETY-TIMING | table | DONE | B1 | inline, slide 8.2 | Written 2026-09-02. **Still wants the vesc chat's review.** |
| TABLE-CALIB-STATUS | table | DONE | B2 | inline, slide 9.1 | Written 2026-09-02, inherited vs measured per constant with dates. |
| TABLE-DEADBAND | table | DONE | B2 | inline, slide 9.4 | Written 2026-09-02, carried in ERPM. |
| TABLE-TAPE | table | DONE | B2 | inline, slide 3.4 | Written 2026-09-02. Also reproduced on 9.4. |
| TABLE-BANDWIDTH | table | DONE | B1 | inline, slide 1.5 | Written 2026-09-02. |
| TABLE-PACKAGES | table | ROBOT | capture | inline | In the container on gosling1: `ros2 pkg list`, `apt list --installed 'ros-humble-*'`, `pip list`. Feeds 0.4. |
| CHART-RATES | chart | DONE | B1 | embedded, slide 2.7 | Built 2026-09-03 by `scripts/analysis/plot_rates.py` from the five `assets/data/rates_live_20260903_*.json` conditions measured live on gosling1. Not from a bag: no bag on the SSD carries a clean-load camera stream, so `bag_stats.py` on `armA_loop2` could not produce it. Carded once, on 2.7; the 2.2 card is retired (see TABLE-CAMERA). |
| CHART-CLOSURE | chart | ROBOT | capture | `assets/figures/localization/closure.png` | `scripts/analysis/plot_localization.py <armA_loop2> --map data/maps/20260805/rtabmap_2d_final.yaml --out docs/presentation/assets/figures/localization/`; also yields `summary.md`. |
| CHART-STEER | chart | ROBOT | capture | `assets/figures/steer.png` | `fit_actuators.py` on `armA_loop`, `armA_loop2`, `armA_steer_sweep`; add `--save-fig` if absent. |
| CHART-SPEED | chart | ROBOT | capture | `assets/figures/speed.png` | `fit_actuators.py` on `armA_straight_5m` + the tape run. |
| CHART-LAG | chart | ROBOT | capture | `assets/figures/lag.png` | Step response; feeds both 7.6 and 9.5. |
| CHART-NAV2-APPROACH | chart | ROBOT | capture | `assets/figures/nav2_approach.png` | Distance-to-goal and commanded speed vs time from the `nav2_drive4` bag. |
| CHART-MPC-TRACKING | chart | DONE | control | `assets/figures/mpc_tracking.png` | Produced 2026-09-03 from the two gosling1 hardware legs of 2026-08-06. Cross-track error and reference-vs-measured speed, over route-acquired ticks only. |
| FIG-MAP-2D | figure | DONE | B2 | `assets/figures/map_2d.png` | Produced 2026-09-02: rtabmap_2d_final.pgm rendered in metres with the map origin and the start-pose seed marked. |
| FIG-MAP-3D | figure | DONE | B2 | `assets/figures/map_3d.png` | Produced 2026-09-02: top-down and oblique renders of cloud_voxel_0p05.pcd, **colour preserved**, over the occupancy grid. |
| FIG-MAP-COMPARE | figure | ROBOT | capture | `assets/figures/map_compare.png` | Needs the SLAM Toolbox grid from `40_build_map_offline.sh --mode both`. |
| FIG-COSTMAP | figure | EXISTS | perception | `assets/figures/costmap.png` | `data/maps/sample_costmap.png` — check its date; if stale it becomes ROBOT. |
| FIG-WAYPOINTS | figure | DONE | control | `assets/figures/waypoints.png` | Produced 2026-09-03 from the two map-frame route CSVs, forward vs reverse coloured by the signed `vx` column. |
| FIG-PERC-LASER | figure | OWNER | perception | `assets/figures/perc_laser.png` | — |
| FIG-PERC-CLOUD | figure | OWNER | perception | `assets/figures/perc_cloud.png` | — |
| FIG-PERC-IMAGE | figure | OWNER | perception | `assets/figures/perc_image.png` | — |
| FIG-SYSID-METHOD | figure | DONE | B2 | `assets/figures/sysid_method.svg` | Produced 2026-09-02. Source `assets/figures/src/sysid_method.mmd`. |
| FIG-BENCH-SWEEP | figure | TBD | B2 | `assets/figures/bench_sweep.png` | Split from the grouped §4 row. The bench sweep has not been run; `BENCH_SWEEP_SHEET.md` + `bench_servo_sweep.py` define it. |
| VID-SENS-RGB | video | ROBOT | capture | `assets/video/VID-SENS-RGB.mp4` | `docs/demo_shot_list.md` §1, 10-20 s. |
| VID-SENS-DEPTH | video | ROBOT | capture | `assets/video/VID-SENS-DEPTH.mp4` | shot list §1. |
| VID-SENS-CLOUD | video | ROBOT | capture | `assets/video/VID-SENS-CLOUD.mp4` | shot list §1. |
| VID-SENS-LIDAR | video | ROBOT | capture | `assets/video/VID-SENS-LIDAR.mp4` | shot list §1. The one sensor clip the talk shows. |
| VID-SENS-IMU | video | ROBOT | capture | `assets/video/VID-SENS-IMU.mp4` | shot list §1. |
| VID-LOC-LOOP | video | ROBOT | capture | `assets/video/VID-LOC-LOOP.mp4` | shot list §2. Needs a driving session. |
| VID-PF-CONVERGE | video | ROBOT | capture | `assets/video/VID-PF-CONVERGE.mp4` | shot list §2. |
| VID-MAP-2D-TIMELAPSE | video | EXISTS-2024 | B2 | `docs/2d_mapping.mp4` | Usable now **with a 2024 label**; shot list §3 replaces it if the lab session has time. |
| VID-MAP-3D-ORBIT | video | EXISTS-2024 | B2 | `docs/3d_mapping.mp4` | As above. |
| VID-NAV2-LIVE | video | ROBOT | capture | `assets/video/VID-NAV2-LIVE.mp4` | shot list §4.3; requires a driving session. |
| VID-SAFETY-MUX | video | ROBOT | capture | `assets/video/VID-SAFETY-MUX.mp4` | shot list §5. |
| VID-MPC-RERUN | video | DONE | control | `assets/video/VID-MPC-RERUN.mp4` | The 2026-08-10 figure-8 leg replayed from its recording, frame-corrected. Already H.264; copied 2026-09-03. Video is gitignored - re-copy from the control repo `data/fig8_video/figure8_leg1_drive.mp4` for a build. |
| VID-MPC-LIVE | video | DONE | control | `assets/video/VID-MPC-LIVE.mp4` | Two-camera overhead of the same 2026-08-10 figure-8 leg. Transcoded to H.264 720p 2026-09-03 from the capture on OneDrive. |
| VID-MPC-OBST-RVIZ | video | ROBOT | control | `assets/video/VID-MPC-OBST-RVIZ.mp4` | **Stays a card.** No MPC obstacle run exists on this car. `rviz_obstacle_detection.mp4` was offered for this slot and is *not* it - it shows clustered-pointcloud detection with no MPC path and no keep-out; it belongs to perception. |
| VID-MPC-OBST-LIVE | video | ROBOT | control | `assets/video/VID-MPC-OBST-LIVE.mp4` | **Stays a card.** Needs a driving session with an obstacle feed; obstacle avoidance has never run on the car. |
| VID-PERC-2024 | video | EXISTS-2024 | perception | `docs/perception_and_gotogoal_planning.mp4` | The perception chat decides whether it still represents the pipeline. Label it 2024. |
| LAT-E2E | measurement | TBD | B2 | inline | End-to-end joystick-to-servo latency has never been measured. Define it (timestamp `joy` vs `commands/servo/position`) before any number is claimed; slide 9.5 says `TBD` until then. |

<!-- assets-table:end -->

---

## Plan §4, verbatim

The text below is copied unchanged from `PRESENTATION_PLAN.md` §4. Do not
paraphrase it into the table above — if the two disagree, this wins and the
table is the bug.

## 4. Asset register (seed for `ASSETS.md`)

Status codes: `EXISTS` (file in repo/OneDrive), `OFFLINE` (renderable on this machine, no robot), `ROBOT` (needs gosling1 and the lab), `USER` (only you can produce it), `OWNER` (another chat's repo), `TBD` (measurement not yet designed).

| ID | Kind | Status | How to produce / where it is |
|---|---|---|---|
| FIG-ARCH-00 .. 05, 01A | figure | USER (PNG received 2026-09-02; SVG still wanted) | Export the 7 Excalidraw notes from Obsidian as **SVG** (for the deck: crisp at any zoom, small) **and 2x PNG** (for review and the PPTX export) into `assets/figures/arch/`. Keep the GPU/CPU legend. Review notes from the received PNGs, to be applied by whoever places them: **(a) the `@Hz` annotations are design targets, not measurements** (stereo 90 Hz, RGB 60 Hz, fusion 90 Hz, wheel 50 Hz vs configured 30 fps camera, 30 Hz EKF, ~8 Hz LiDAR); every slide that shows a diagram pairs it with the measured table and says so in one line. **(b) `1_Sensing` carries a URL block and a filter list below the dashed line**: crop the slide version above the line; the filter list can become its own reference slide; the URLs go to speaker notes. **(c) `1A_StereoVision` embeds a MathWorks figure**: attribute it on the slide or crop it out; also note on 2.3 that the D435i computes disparity on-device, so the drawn pipeline is conceptual. **(d) `4_Control` shows a steering PID and a 200 Hz velocity PID**: that is design intent; slide 7.5 draws the as-built chain from the vesc fork (servo position command, closed-loop speed optional and default off). **(e) `4_Control` has no mux/gate**: FIG-CMDPATH covers that. **(f) `1_Sensing` is portrait (932x1998)**: it needs a two-column slide or the crop in (b). |
| PHOTO-CAR, -COMPONENTS, -WIRING, -ASSEMBLY, -DUALSENSE | photo | EXISTS | `docs/figures/teleop/*`, `docs/attachments/F1tenthDocumentation_v1_1_Release/media/*`; crop/downscale into `assets/photos/` (several are 12-22 MB). |
| FIG-ELEC | figure | USER or TBD | Electrical/power diagram does not exist. Draw (Excalidraw or mermaid) from Release doc §Wiring; card until then. |
| FIG-TF-TREE | figure | ROBOT (or OFFLINE) | `ros2 run tf2_tools view_frames` on a running stack; offline alternative: draw from `static_transformations.launch.py` + URDF with mermaid. |
| FIG-LAUNCHTREE, FIG-CMDPATH, FIG-EKF-INPUTS, FIG-LOWLEVEL, FIG-NAV2-BT | diagram | OFFLINE | Mermaid inside Marp (marp-core renders mermaid via plugin, or pre-render to SVG in `build.sh`). |
| TABLE-REPOS, -CAMERA, -IMU, -ODOM, -CONTROLLERS, -SAFETY-TIMING, -CALIB-STATUS, -DEADBAND, -TAPE, -BANDWIDTH | table | OFFLINE | Values from config YAML, CLAUDE.md and `SYSID_RESULTS.md`; each cell dated in speaker notes. |
| CHART-RATES | chart | DONE | Built offline from the committed `assets/data/rates_live_20260903_*.json` by `scripts/analysis/plot_rates.py`; no robot time needed to rebuild it. |
| CHART-CLOSURE | chart | ROBOT | `scripts/analysis/plot_localization.py <armA_loop2 bag> --map data/maps/20260805/rtabmap_2d_final.yaml --out docs/presentation/assets/figures/localization/`; also yields `summary.md`. |
| CHART-STEER, CHART-SPEED, CHART-LAG | chart | ROBOT | `fit_actuators.py` on `armA_loop`, `armA_loop2`, `armA_straight_5m`, `armA_steer_sweep`; add a `--save-fig` path if the script lacks one. |
| CHART-NAV2-APPROACH | chart | ROBOT | Distance-to-goal and commanded speed vs time from the `nav2_drive4` bag. |
| CHART-MPC-TRACKING | chart | OWNER (control) | Cross-track and speed error from the figure-8/loop runs. |
| FIG-MAP-2D | figure | OFFLINE | Render `data/maps/20260805/rtabmap_2d_final.pgm` with origin/axes and the seed pose marked. |
| FIG-MAP-3D | figure | OFFLINE | Top-down and oblique render of `cloud_voxel_0p05.pcd` (colour preserved) over the grid; open3d or matplotlib. |
| FIG-MAP-COMPARE | figure | ROBOT | Needs the SLAM Toolbox grid from `40_build_map_offline.sh --mode both` (bags on the SSD). |
| FIG-COSTMAP | figure | EXISTS | `data/maps/sample_costmap.png` (check date, else ROBOT). |
| FIG-WAYPOINTS, FIG-PERC-LASER/-CLOUD/-IMAGE, FIG-SYSID-METHOD, FIG-BENCH-SWEEP | figure | OWNER / OFFLINE | Owning chats. |
| VID-SENS-RGB/-DEPTH/-CLOUD/-LIDAR/-IMU | video | ROBOT | `docs/demo_shot_list.md` §1, 10-20 s each. |
| VID-LOC-LOOP, VID-PF-CONVERGE | video | ROBOT | shot list §2. |
| VID-MAP-2D-TIMELAPSE, VID-MAP-3D-ORBIT | video | EXISTS-2024, re-capture optional | `docs/2d_mapping.mp4`, `docs/3d_mapping.mp4` (Feb 2024) are usable now with a "2024" label; shot list §3 replaces them with current-stack clips if the lab session has time. |
| VID-NAV2-LIVE | video | ROBOT | shot list §4.3, requires a driving session. |
| VID-SAFETY-MUX | video | ROBOT | shot list §5. |
| VID-MPC-RERUN, VID-MPC-LIVE, VID-MPC-OBST-RVIZ, VID-MPC-OBST-LIVE | video | OWNER (control) | Control chat lists what already exists (rerun recordings of figure-8 and loops, live drives, obstacle runs); anything missing becomes a card. |
| VID-PERC-2024 | video | EXISTS-2024 | `docs/perception_and_gotogoal_planning.mp4` (Feb 2024), labelled; the perception chat decides whether it still represents the pipeline. |
| PHOTO-* from the 2024 doc | photo | EXISTS-2024 | Hardware photos and wiring stills from `docs/attachments/.../media` are current hardware and need no label; only *software* text from the 2024 doc is treated as stale. |
| TABLE-PACKAGES | table | ROBOT | Inside the container on gosling1: `ros2 pkg list`, `apt list --installed 'ros-humble-*'`, `pip list`; feeds 0.4. |
| LAT-E2E | measurement | TBD | End-to-end joystick-to-servo latency has never been measured; define (timestamp `joy` vs `commands/servo/position`) before claiming a number. |

Video files are never committed. `assets/video/` holds a `README` listing the canonical path on gosling1 (`/mnt/shared_dir/videos/<date>/`) and the OneDrive copy. Marp embeds video via `<video>` tags pointing at relative paths, so the HTML export plays them when the folder is present, and the PDF shows the poster frame.
