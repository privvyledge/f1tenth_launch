# Brief: f1tenth_launch sections

**Read this first, then `PRESENTATION_PLAN.md` if you need the whole picture.**
Everything below that is marked *verbatim* is copied unchanged out of the plan —
it is not a paraphrase, because paraphrase is where constraints get dropped. If
this file and the plan ever disagree, the plan wins and this file is the bug.

Your working directory is `docs/presentation/` in `f1tenth_launch`.

---

## 0. What the plan says about this brief (§5, verbatim)

Every brief has the same shape: **slides you own** (IDs from §3), **what exists in your repo** that answers them (the brief lists what this scan found so the chat starts from evidence), **assets you must produce** (IDs and target paths), **conventions** (§1), **what you must not do** (fabricate media, quote numbers without a source, reference agent files on slides), **cut tagging** (tag each slide with `lab`/`sponsor`/`research` per §2a; return a ranked list of what your area can actually show today so the `sponsor` cut is built from real material), **research questions** (one or two for S99, with the evidence you already have), and **hand-back** (which files to edit, how to run `build.sh` to check).

### BRIEF_f1tenth_launch.md
Owns S00, S10, S20, S30, S40, 6.1, 6.3, 6.4, 6.6, S80, S90, S95. Sources are all in this repo (`config/`, `launch/`, CLAUDE.md, `scripts/live_runs/*.md`, `docs/F1tenthDocumentation_v1_1_Release.md`). Probably two chats: **B1** = S00, S10, S20, S80 (hardware/sensing/safety, mostly config and the 2024 doc) and **B2** = S30, S40, S60 parts, S90, S95 (localization/mapping/Nav2/sysid, mostly measured results).

---

## 1. Slides you own (§3, verbatim)

The rows below are copied unchanged from the plan's slide-by-slide table.
Legend: **T** = talk deck, **R** = reference deck only. "Source" is where the content
is already written down; "Assets" are IDs from §4. Owner defaults to the
f1tenth_launch chat unless stated.

### S00 Overview (T 4, R 1)

*Section file: `slides/00_overview.md`*

| # | T/R | Slide (title is the claim) | Content | Source | Assets |
|---|---|---|---|---|---|
| 0.1 | T | Title | Project name, lab, date, repos, one photo of the car | photos | PHOTO-CAR |
| 0.2 | T | What the car does today | Status board: teleop, 2D/3D mapping, AMCL+EKF localization, Nav2 has driven the car, MPC live drives, perception nodes exist but are not integrated in bringup. Each row: status, date last verified | CLAUDE.md, live_runs docs | — |
| 0.3 | T | The system is six layers | Excalidraw system architecture (sensing, perception, localization, planning, control, vehicle, offline database) with GPU/CPU legend | Obsidian `0_SystemArchitecture` | FIG-ARCH-00 |
| 0.4 | T | Software stack | ROS 2 Humble on Jetson Orin Nano Super (JetPack 6.2, L4T r36.4.3), CycloneDDS, docker image `humble-devel-08302026`; table of the repos in `f1tenth.repos` with one line each | f1tenth.repos, memory notes | TABLE-REPOS |
| 0.5 | R | Launch tree and two-phase startup | `bringup.launch.py` tree; delays 6 s camera, 2 s LiDAR, 10 s localization, 15 s mapping/Nav2 and why (TF races, USB bandwidth) | CLAUDE.md §Architecture | FIG-LAUNCHTREE (mermaid) |

### S10 Hardware (T 3, R 3)

*Section file: `slides/10_hardware.md`*

| # | T/R | Slide | Content | Source | Assets |
|---|---|---|---|---|---|
| 1.1 | T | Platform and components | Table: Jetson Orin Nano Super 8 GB, 1 TB NVMe; VESC 6 MkVI; RealSense D435i; YDLidar X4; Traxxas 4-Tec 2.0 VXL chassis and its steering servo; DualSense. Photo grid | Release doc §Hardware Components, `docs/attachments/.../media` | PHOTO-COMPONENTS |
| 1.2 | T | Where every sensor sits: the static frame tree | `base_link` = rear axle; offsets from CLAUDE.md frame tree; camera frames from URDF; rendered `view_frames` output | CLAUDE.md §Static TF Frame Tree, `static_transformations.launch.py`, `urdf/f1tenth.urdf.xacro` | FIG-TF-TREE |
| 1.3 | T | Two batteries, two failure modes | Jetson pack vs VESC drive pack; XT90; powerbank; VESC `voltage_input` is the drive pack only, the Jetson supply has no topic. Electrical diagram placeholder | Release doc §Wiring, memory `jetson-and-vesc-are-on-separate-batteries` | PHOTO-WIRING, FIG-ELEC |
| 1.4 | R | Assembly | Rollcage, body shell, NPF adapter, sensor mount photos | `docs/figures/teleop/*` | PHOTO-ASSEMBLY |
| 1.5 | R | Network | Robot WiFi/wired, velox1 as remote RViz host, DDS profiles, remote bandwidth (RViz set ~99 kB/s, one image stream 31 MB/s) | Release doc §Network, CLAUDE.md §CycloneDDS | TABLE-BANDWIDTH |
| 1.6 | R | Controller pairing and button map | BR/EDR pairing; SDL mapping L1=9, R1=10, PS=5; labelled DualSense figure already exists | Release doc §DualSense, `docs/figures/teleop/dualsense_top_with_arrows_and_labels.png` | PHOTO-DUALSENSE |

### S20 Sensing (T 5, R 2)

*Section file: `slides/20_sensing.md`*

| # | T/R | Slide | Content | Source | Assets |
|---|---|---|---|---|---|
| 2.1 | T | Sensing pipeline | Excalidraw sensing diagram: stereo, LiDAR, wheel/steer, IMU, preprocessing filters, rates | Obsidian `1_Sensing` | FIG-ARCH-01 |
| 2.2 | T | Camera streams | Table per topic: RGB, infra1/2, depth, aligned depth, colored pointcloud, IMU. Columns: resolution, configured fps, **measured** Hz, QoS, on/off by default. Emitter-off trade-off (VSLAM vs depth quality), decimation 2, `ordered_pc: false` | `config/sensors/realsense_config.yaml`, `realsense_d435i.launch.py`, `bag_stats.py` output | TABLE-CAMERA, CHART-RATES |
| 2.3 | R | Stereo to depth to pointcloud | Excalidraw stereo pipeline (rectification, census/cost, disparity, projection) | Obsidian `1A_StereoVision` | FIG-ARCH-01A |
| 2.4 | T | 2D LiDAR | X4: 0.12-10 m (12 m spec caused phantoms), ~625 beams, 12 Hz configured / ~8 Hz observed, no intensity, `inf` for out-of-range; speckle filter `lidar/scan` to `lidar/scan_filtered` | `config/sensors/ydlidar_X4.yaml`, `config/filters/laser_filter.yaml`, CLAUDE.md | VID-SENS-LIDAR |
| 2.5 | T | Two IMUs, two treatments | VESC IMU ~100 Hz, no magnetometer, onboard Madgwick, yaw not fused; D435i IMU 200 Hz through `imu_filter_madgwick` with `constant_dt` 0.005 and why (first samples have bogus stamps) | CLAUDE.md §ekf_odom, `imu_filter.launch.py` | TABLE-IMU |
| 2.6 | T | Wheel odometry | `vesc_to_odom`: speed = erpm/3750, angle = (servo-0.56)/-1.1448, yaw rate = v tan(delta)/0.256; silent until first servo command | `config/vehicle/vesc.yaml`, CLAUDE.md | — |
| 2.7 | T | Measured rates and bandwidth | One chart of measured topic rates from a drive bag, with per-stream bandwidth (the image set is ~125 MB/s; the IMU-only `sysid` set is ~0.5 MB/s) | `bag_stats.py`, CLAUDE.md §Debugging Caveats | CHART-RATES |

### S30 Localization (T 5, R 3)

*Section file: `slides/30_localization.md`*

| # | T/R | Slide | Content | Source | Assets |
|---|---|---|---|---|---|
| 3.1 | T | Localization pipeline | Excalidraw: dead reckoning, VO, ICP/scan odometry, fusion, local vs global | Obsidian `2A_Localization` | FIG-ARCH-02 |
| 3.2 | T | Odometry sources | Table: `vehicle/vesc_odom`, `odom/rf2o` 10 Hz, Isaac cuVSLAM `visual_slam/tracking/odometry` 30 Hz GPU, RTABMap stereo/RGB-D/ICP (CPU alternatives, ICP off), kiss-icp optional. Columns: sensor, rate, frame, fused fields, on by default | `ekf_odom.yaml`, `localization.launch.py`, CLAUDE.md | TABLE-ODOM |
| 3.3 | T | Local EKF design, and why each choice | 30 Hz; which fields from each source; differential/relative choices; rejection gates; init covariance 0.5; `odometry/local` is what Nav2 consumes. **Each non-default choice gets its one-line reason on the slide**: VESC yaw not fused (no magnetometer, onboard quaternion free-runs -14 deg/min while advertising tight covariance; parked drift -13.94 to +0.04 deg/min after), camera IMU acceleration not fused (attitude error at stream start leaves a ~1 g residual), rf2o gated at 5.0 Mahalanobis (timestamp spikes), `use_control` off (12 cm divergence on replay, no benefit) | `config/localization/ekf_odom.yaml`, CLAUDE.md | FIG-EKF-INPUTS |
| 3.4 | T | Why fusion: the closure-error figure | Bar chart of each estimator's error after a loop that returns to start; table: over 5.50 m tape, VSLAM +1.3 %, rf2o -6.1 %, EKF -3.1 %; yaw drift parked -13.94 to +0.04 deg/min after disabling VESC yaw | `plot_localization.py` on `armA_loop2`, `SYSID_RESULTS.md`, CLAUDE.md | CHART-CLOSURE, TABLE-TAPE |
| 3.5 | T | Global localization | map_server + AMCL (likelihood field, motion-gated) and `ekf_map` 10 Hz; `map->odom` owned by `ekf_map` via bringup; seed (0.445, -0.575, -84.5 deg) published to `initialpose` at +20 s; 6/6 cold launches within 8 mm / 0.2 deg | `localizer_amcl.yaml`, `ekf_map.yaml`, CLAUDE.md | VID-LOC-LOOP, VID-PF-CONVERGE (R) |
| 3.6 | R | Alternatives evaluated | particle_filter (range_libc GPU, tuned params), slam_toolbox localizer, RTABMap localization mode; status of each | `localizer_pf.yaml`, `BRIEF_PARTICLE_FILTER.md` | — |
| 3.7 | R | Frames | map -> odom -> base_link, who publishes which TF and how `odom_tf_publisher` / `map_tf_publisher` select owners | CLAUDE.md | FIG-TF-TREE |
| 3.8 | R | Seeding and start pose | Why the parking spot is not the map origin; how the heading was measured from raw scans | CLAUDE.md, `DEMO_RUNBOOK_20260810.md` §5b | FIG-MAP-2D |

### S40 Mapping (T 2, R 2)

*Section file: `slides/40_mapping.md`*

| # | T/R | Slide | Content | Source | Assets |
|---|---|---|---|---|---|
| 4.1 | T | 2D mapping | SLAM Toolbox (0.05 m, 12 m) vs RTABMap grid (LiDAR `Grid/Sensor 0`, ray tracing); the current `20260805` grid rendered | `2d_mapping_*.yaml`, `data/maps/20260805/rtabmap_2d_final.pgm` | FIG-MAP-2D, VID-MAP-2D-TIMELAPSE |
| 4.2 | T | 3D mapping | RTABMap RGB-D (CPU) or nvblox (GPU); render of `cloud_voxel_0p05.pcd` in the grid frame | `3d_mapping.launch.py`, `data/maps/20260805/cloud_voxel_0p05.pcd` | FIG-MAP-3D, VID-MAP-3D-ORBIT |
| 4.3 | R | One database, three artifacts | Grid, cloud and AMCL seed must come from the same RTABMap database; save commands | CLAUDE.md, `COMMANDs.md` | — |
| 4.4 | R | RTABMap vs SLAM Toolbox grids | Side-by-side stills | shot list 3.2 | FIG-MAP-COMPARE |

### S60 Planning and Nav2 (T 4, R 2)

*Section file: `slides/60_planning_nav2.md`*

| # | T/R | Slide | Content | Source | Assets | Owner |
|---|---|---|---|---|---|---|
| 6.1 | T | Planning pipeline | Excalidraw: route planning 1 Hz, behavior/local planning 10 Hz, sparse vs dense waypoints | Obsidian `3_Planning` | FIG-ARCH-04 | launch |
| 6.3 | T | Nav2 as configured | bt_navigator + custom BT XMLs, planner (name from `nav2_params.yaml`), RPP controller 10 Hz, costmaps 5 Hz / 1 Hz, velocity smoother, `twist_to_ackermann`; per-server toggles; MPC can replace `controller_server` through `FollowPath` | `nav2_params.yaml`, `nav2_navigation.launch.py`, CLAUDE.md | FIG-NAV2-BT | launch |
| 6.4 | T | Nav2 on the car | 2026-08-27: 5.78 m path, 203/203 commands reached the VESC, stopped 0.379 m from goal (tol 0.25 m), heading 4.8 deg (tol 14.3 deg); last command 0.269 m/s vs breakaway 0.20-0.26 m/s; progress checker and recovery ladder observed; obstacle avoidance untested | CLAUDE.md §Nav2, `nav2_drive4` bag on SSD | CHART-NAV2-APPROACH, VID-NAV2-LIVE | launch |
| 6.6 | R | Offline Nav2 replay results | goal accept 1-2 ms, first plan <0.05 s, cmd_vel 20 Hz bounded 0.5 m/s | `NAV2_OFFLINE_RESULTS.md` | — | launch |

### S80 Vehicle interface and safety (T 4, R 2)

*Section file: `slides/80_vehicle_safety.md`*

| # | T/R | Slide | Content | Source | Assets |
|---|---|---|---|---|---|
| 8.1 | T | Every command passes two gates | Diagram: `teleop`/`drive`/`estop` -> `ackermann_mux` (255/100/10/1, timeouts 0.5/0.3/0.2/0.05 s) -> `ackermann_drive` -> `command_gate` -> `vehicle/ackermann_cmd` -> `ackermann_to_vesc` -> VESC | `mux.yaml`, `command_gate.yaml`, CLAUDE.md | FIG-CMDPATH (mermaid) |
| 8.2 | T | Deadman and handover | Heartbeat on `command_gate/heartbeat`, 0.5 s timeout; L1 manual, R1 autonomous handover; gate closes 0.5-0.63 s after joystick loss; commanded motion bounded to 0.2-0.35 s; never use PS in a deadman set. **Why the heartbeat has its own topic** (one line): the R1 handover silences `teleop` by design, so a `teleop`-based heartbeat would close the gate 1 s into every autonomous run | CLAUDE.md, `joy_teleop.yaml` | TABLE-SAFETY-TIMING, VID-SAFETY-MUX (R) |
| 8.3 | T | Operating procedures | Pre-op checklist, power-up order, connection, normal operation, shutdown, emergency stop; battery management | Release doc §Operating Procedures, §Battery Management | — |
| 8.4 | T | Teleop controls | Labelled DualSense figure with speed/steer scales | `dualsense_top_with_arrows_and_labels.png`, `joystick.launch.py` | PHOTO-DUALSENSE |
| 8.5 | R | Three gate configurations | full safety / passthrough / gate removed, and what each implies | CLAUDE.md | — |
| 8.6 | R | Teardown and orphans | why `kill -INT` is not enough; `stop_launch_tree()` | CLAUDE.md §Script teardown | — |

### S90 System identification and calibration (T 5, R 2)

*Section file: `slides/90_sysid_calibration.md`*

| # | T/R | Slide | Content | Source | Assets |
|---|---|---|---|---|---|
| 9.1 | T | Everything in `vesc.yaml` was inherited from another car | Status board: each constant, inherited vs measured, date, method | CLAUDE.md §vesc.yaml, `SYSID_RESULTS.md` | TABLE-CALIB-STATUS |
| 9.2 | T | Method | Driven bags with structured excitation; `fit_actuators.py` scores against TF-corrected gyro-z in servo units; three references: rf2o, VSLAM, tape measure | `SYSID_RESULTS.md`, `scripts/analysis/fit_actuators.py` | FIG-SYSID-METHOD |
| 9.3 | T | Steering | Gain -1.4 to -1.1448 (over-steer 18-23 %), confirmed k = 0.999 / 0.991; offset 0.56 vs fitted 0.5508-0.5583 (contested, bench sweep pending); bounds driven: 17.45 deg L / 17.47 deg R, R_min 0.814 m, 97 % of commanded; `max_steering` 0.314 is the binding limit; left headroom to 24 deg | CLAUDE.md, `SYSID_RESULTS.md` | CHART-STEER |
| 9.4 | T | Speed and deadband | 3750 erpm per m/s stands; rf2o -6.1 %, VSLAM +1.3 % over 5.50 m; ground breakaway 0.20-0.26 m/s, deadband table carried in ERPM | CLAUDE.md, `SYSID_RESULTS.md` §Deadband | CHART-SPEED, TABLE-DEADBAND |
| 9.5 | T | Latency | Actuation 60 ms + 40 ms first-order; throttle transport <20 ms; end-to-end joystick-to-servo latency TBD (measurement defined in ASSETS) | `SYSID_RESULTS.md` | CHART-LAG |
| 9.6 | R | Wheelbase and yaw rate | 0.25 -> 0.256 alignment; effect on kinematic yaw | CLAUDE.md | — |
| 9.7 | R | Bench servo sweep | Planned measurement of centring and `servo_min`/`servo_max` | `BENCH_SWEEP_SHEET.md`, `bench_servo_sweep.py` | FIG-BENCH-SWEEP |

### S95 Next steps and lessons (T 1, R 1)

*Section file: `slides/95_lessons_next.md`*

| # | T/R | Slide | Content | Source | Assets |
|---|---|---|---|---|---|
| 10.1 | T | What is next | Nav2 goal reach (0.38 m stall), servo offset bench sweep, obstacle avoidance test on the car, MPC go-to-goal on the car, perception integration into bringup, `servo_min/max` measurement, closed-loop speed enable after recalibration | CLAUDE.md, `LOCALIZER_FOLLOWUPS.md` | — |
| 10.2 | R | Lessons for measuring on this car | RealSense IMU first stamps and `constant_dt` (attitude jump 80-167 deg to 0.19 deg); rf2o scale error, so never calibrate against one reference; check `odometry/local` twist before trusting a pose; verify streams by message count, not rate | CLAUDE.md | — |

> **Known plan discrepancy (left uncorrected on purpose, 2026-09-02).** S20's header reads "T 5, R 2" but the section lists **six** T slides (2.1, 2.2, 2.4, 2.5, 2.6, 2.7) and one R slide.
> The scaffold built from the **rows**, which is what the table above shows.
> Phase G arbitrates; do not renumber anything to make the header add up.


**This brief is for two chats working in parallel, on disjoint files:**

| Chat | Sections | Character of the work |
|---|---|---|
| **B1** | S00, S10, S20, S80 | hardware, sensing, safety — mostly config files and the 2024 release doc |
| **B2** | S30, S40, 6.1/6.3/6.4/6.6, S90, S95 | localization, mapping, Nav2, sysid — mostly measured results |

6.2 and 6.5 belong to the control chat; 7.5-7.6 to the vesc chat. Do not write
them, but read them before you claim anything about the actuation chain.

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
| PHOTO-CAR | photo | EXISTS | B1 | `assets/photos/car.jpg` | Crop/downscale from `docs/attachments/F1tenthDocumentation_v1_1_Release/media/`. |
| PHOTO-COMPONENTS | photo | EXISTS | B1 | `assets/photos/components/` | Photo grid; sources as above and `docs/figures/teleop/sensors/`. |
| PHOTO-WIRING | photo | EXISTS | B1 | `assets/photos/wiring.jpg` | Source stills are current hardware and need no 2024 label. |
| PHOTO-ASSEMBLY | photo | EXISTS | B1 | `assets/photos/assembly/` | `docs/figures/teleop/assembly/`, `rollcage.png`, `body_shell.png`, `npf_adapter.png`. |
| PHOTO-DUALSENSE | photo | EXISTS | B1 | `assets/photos/dualsense_labelled.png` | `docs/figures/teleop/dualsense_top_with_arrows_and_labels.png`. |
| FIG-LAUNCHTREE | diagram | OFFLINE | B1 | `assets/figures/launch_tree.svg` | Mermaid; pre-render to SVG. |
| FIG-CMDPATH | diagram | OFFLINE | B1 | `assets/figures/cmd_path.svg` | Mermaid; reviewed by the vesc chat. |
| FIG-EKF-INPUTS | diagram | OFFLINE | B2 | `assets/figures/ekf_inputs.svg` | Mermaid. |
| FIG-NAV2-BT | diagram | OFFLINE | B2 | `assets/figures/nav2_bt.svg` | From `config/behavior_trees/*.xml`. |
| TABLE-REPOS | table | OFFLINE | B1 | inline | From `f1tenth.repos`. Date every cell in speaker notes. |
| TABLE-CAMERA | table | OFFLINE | B1 | inline | Configured values from `config/sensors/realsense_config.yaml`; measured Hz needs CHART-RATES data. |
| TABLE-IMU | table | OFFLINE | B1 | inline | — |
| TABLE-ODOM | table | OFFLINE | B2 | inline | — |
| TABLE-SAFETY-TIMING | table | OFFLINE | B1 | inline | Reviewed by the vesc chat. |
| TABLE-CALIB-STATUS | table | OFFLINE | B2 | inline | Inherited vs measured, per constant, with dates. |
| TABLE-DEADBAND | table | OFFLINE | B2 | inline | Carry the numbers in ERPM, not m/s. |
| TABLE-TAPE | table | OFFLINE | B2 | inline | The 5.50 m tape run vs VSLAM / rf2o / EKF. |
| TABLE-BANDWIDTH | table | OFFLINE | B1 | inline | Remote-link measurements. |
| FIG-MAP-2D | figure | OFFLINE | B2 | `assets/figures/map_2d.png` | Render `data/maps/20260805/rtabmap_2d_final.pgm` with origin, axes and the seed pose marked. |
| FIG-MAP-3D | figure | OFFLINE | B2 | `assets/figures/map_3d.png` | Top-down and oblique render of `cloud_voxel_0p05.pcd` (**colour preserved**) over the grid. |
| FIG-SYSID-METHOD | figure | OFFLINE | B2 | `assets/figures/sysid_method.svg` | Split from the grouped §4 row by the scaffold; drawn from `SYSID_RESULTS.md`. |
| FIG-BENCH-SWEEP | figure | TBD | B2 | `assets/figures/bench_sweep.png` | Split from the grouped §4 row. The bench sweep has not been run; `BENCH_SWEEP_SHEET.md` + `bench_servo_sweep.py` define it. |
| VID-MAP-2D-TIMELAPSE | video | EXISTS-2024 | B2 | `docs/2d_mapping.mp4` | Usable now **with a 2024 label**; shot list §3 replaces it if the lab session has time. |
| VID-MAP-3D-ORBIT | video | EXISTS-2024 | B2 | `docs/3d_mapping.mp4` | As above. |
| LAT-E2E | measurement | TBD | B2 | inline | End-to-end joystick-to-servo latency has never been measured. Define it (timestamp `joy` vs `commands/servo/position`) before any number is claimed; slide 9.5 says `TBD` until then. |

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
