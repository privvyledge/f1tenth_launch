# F1TENTH presentation plan

Status: **APPROVED 2026-09-02** (revision 3). Saved as
`docs/presentation/PRESENTATION_PLAN.md`; it is the contract every slide-writing
chat works from. Nothing beyond this file and the seven diagram PNGs has been implemented; Phase A (scaffold) is the next step.

---

## 0. Decisions already taken (from the 2026-09-02 Q&A)

| Question | Decision |
|---|---|
| Audience / length | **Talk deck first** (target 40-45 slides, ~25 min). Authored so that a long reference deck falls out of the same source: every slide lives in the section files, talk slides carry a `<!-- talk -->` tag, and the build emits both `deck_talk` and `deck_full`. |
| Format | **Marp Markdown**, one file per section, built with `marp-cli` (v4.5.0 works via `npx` on this machine) to PDF, HTML and PPTX. |
| Engineering journey | **Sysid and calibration: in.** Design choices are explained **where they live** (the VESC-yaw exclusion inside the fusion slides, the dedicated heartbeat topic inside the safety slides) rather than on a "bugs" slide. A short lessons slide exists in the reference deck only. Explicitly **out**: the map-frame rotation, the executor abort, the mux-timeout twitch, the image-recording VSLAM starvation. Infra: **safety and operating procedures in**; everything else infra is reference-only or out. |
| Cross-repo material | **One brief per owning repo/chat** (§5). This plan fixes structure, template and asset IDs; the owning chat fills the slides. |
| Where it lives | **`docs/presentation/`** in `f1tenth_launch` (this repo already pools the others). Committed, except video. |
| Media honesty | **No fabricated media.** Anything not yet captured is a placeholder card with an asset ID, and the register (§4) says how to capture it. 2024 photos, hardware text and videos **may be reused where needed**, always with a visible "2024" label on media; the 2024 *software* descriptions are assumed stale and are not copied. |
| Controllers | The **`trajectory_following_ros2` MPC is the flagship high-level controller**: waypoint following from recorded waypoints, go-to-goal, and obstacle avoidance. Nav2's MPPI plugin is secondary and gets a row in the comparison table, not its own slide. |
| Talk length | ~25 min, adjustable as the deck firms up. In person, slides only, so every demo is a video or figure in the deck and the PDF must be self-contained. |
| Audience | Three audiences, **one source, three cuts** (§2a): `lab` (advisor and lab group, the main target), `sponsor` (external collaborators, fused with lab material but demo-led), `research` (committee-style: contributions, evidence, open questions for a paper). A slide is tagged with the cuts it belongs to; ~30 core slides are in all three. Building three cuts costs nothing extra once slides are tagged, and it avoids one 25-minute deck trying to be a status report, a showcase and a rationale record at once. |
| Purpose | Core is the **honest status report** with one-line rationale on each design slide. Showcase material (videos, results) is concentrated in the `sponsor` cut and depends on the robot capture session. Deeper rationale lives in the `lab` cut and the reference deck. |
| Depth | ROS-literate: topic names, Hz, frames and parameter values on slides are expected. |

---

## 1. Deliverables and directory layout

```
docs/presentation/
├── PRESENTATION_PLAN.md          this file
├── ASSETS.md                     asset register (§4): every figure/video/chart ID, status, how to capture
├── build.sh                      concat sections -> deck_full.md / deck_talk.md -> pdf/html/pptx
├── theme/
│   └── f1tenth.css               Marp theme: 16:9, dark-on-light, code/table sizes, placeholder card style
├── slides/                       one Marp file per section, numbered so `cat` gives deck order
│   ├── 00_overview.md
│   ├── 10_hardware.md
│   ├── 20_sensing.md
│   ├── 30_localization.md
│   ├── 40_mapping.md
│   ├── 50_perception.md          (owner: perception chat)
│   ├── 60_planning_nav2.md       (6.2 and 6.5 owner: control chat)
│   ├── 70_control.md             (owner: control chat; 7.5-7.6 owner: f1tenth_system/vesc chat)
│   ├── 80_vehicle_safety.md
│   ├── 90_sysid_calibration.md
│   └── 95_lessons_next.md
├── briefs/
│   ├── BRIEF_f1tenth_launch.md            sections 00,10,20,30,40,60(.3-.4),80,90,95
│   ├── BRIEF_trajectory_following.md      70 (high level), 60.2, 60.5
│   ├── BRIEF_perception.md                50
│   ├── BRIEF_f1tenth_system_vesc.md       70.5-70.6, review of 80.1-80.2
│   └── BRIEF_robot_capture.md             the lab session: clips, charts, inventories that need gosling1
├── assets/
│   ├── figures/arch/             Excalidraw exports (SVG + PNG), user-exported from Obsidian
│   ├── figures/                  rendered charts, TF tree, maps, diagrams
│   ├── photos/                   copied/cropped from docs/figures/teleop and the 2024 doc media
│   └── video/                    .gitignored; real files stay on gosling1 /mnt/shared_dir/videos and OneDrive
└── out/                          .gitignored build output
```

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

---

## 2. Story of the talk deck

1. **What the car is and what it does today** (status board, honest).
2. **Hardware and how it is wired** so the rest makes sense.
3. **Sensing**: what streams exist, at what rates, and what we do to them.
4. **Localization**: many odometry sources, one fused state, and the single figure that justifies fusion.
5. **Mapping**: the 2D and 3D maps we drive in and the rule that keeps them consistent.
6. **Perception**: how obstacles become costmap cells and tracked objects.
7. **Planning and Nav2**: waypoints, goals, and what Nav2 did on the real car.
8. **Hierarchical control**: high-level controllers (MPC, MPPI, pure pursuit, RPP, joystick) over a low-level actuation chain.
9. **Vehicle interface and safety**: mux, gate, deadman, procedures.
10. **System identification**: what was inherited, what was measured, what changed.
11. **Lessons and next steps.**
12. **Research questions** (research cut only): what this platform lets us ask that a paper could answer.

### 2a. The three cuts

| Cut | Audience | Gets the core plus | Drops from the core | Target |
|---|---|---|---|---|
| `lab` | advisor and lab group (main) | design-choice slides with their reasons (3.3, 3.5, 8.1-8.2), sysid detail (9.3-9.5), Nav2 on the car (6.4) | nothing | 40-42 slides, 25 min |
| `sponsor` | external collaborators | demo videos (VID-*), status board up front (0.2), controller results (7.3-7.4), roadmap (10.1) | config tables (2.2, 2.5 detail, 3.3 detail), procedures (8.3), sysid method (9.2) | 30-34 slides, 20-25 min; **not presentable until the robot capture session (Phase F) is done** |
| `research` | committee-style | contributions framing on 0.2, the evaluation figures (3.4, 6.4, 9.3-9.5), plus section S99 | hardware photos beyond 1.1, procedures (8.3-8.4), software-stack table (0.4) | 36-40 slides |

**Core (all three cuts)**: 0.1, 0.2, 0.3, 1.1, 1.2, 2.1, 2.4, 2.6, 2.7, 3.1, 3.2, 3.4, 4.1, 4.2, 5.1, 5.3, 5.4, 6.1, 6.3, 7.1, 7.2, 7.3, 7.5, 8.1, 9.1, 10.1. Everything else tagged T in §3 belongs to one or two cuts as the table says; the section owner tags it, the integration chat arbitrates the counts.

**Why cuts rather than one deck**: a single 25-minute talk cannot be a status report, a showcase and a rationale record at once. The core *is* the honest status report; the showcase and the rationale are what the `sponsor` and `lab` cuts add. **Pushback recorded**: I only know this repo, so which controller and perception results deserve showcase slots is for the owning chats to say; their briefs ask for a ranked list of what they can actually show today.

---

## 3. Slide-by-slide plan

Legend: **T** = talk deck, **R** = reference deck only. "Source" is where the content
is already written down; "Assets" are IDs from §4. Owner defaults to the
f1tenth_launch chat unless stated.

### S00 Overview (T 4, R 1)

| # | T/R | Slide (title is the claim) | Content | Source | Assets |
|---|---|---|---|---|---|
| 0.1 | T | Title | Project name, lab, date, repos, one photo of the car | photos | PHOTO-CAR |
| 0.2 | T | What the car does today | Status board: teleop, 2D/3D mapping, AMCL+EKF localization, Nav2 has driven the car, MPC live drives, perception nodes exist but are not integrated in bringup. Each row: status, date last verified | CLAUDE.md, live_runs docs | — |
| 0.3 | T | The system is six layers | Excalidraw system architecture (sensing, perception, localization, planning, control, vehicle, offline database) with GPU/CPU legend | Obsidian `0_SystemArchitecture` | FIG-ARCH-00 |
| 0.4 | T | Software stack | ROS 2 Humble on Jetson Orin Nano Super (JetPack 6.2, L4T r36.4.3), CycloneDDS, docker image `humble-devel-08302026`; table of the repos in `f1tenth.repos` with one line each | f1tenth.repos, memory notes | TABLE-REPOS |
| 0.5 | R | Launch tree and two-phase startup | `bringup.launch.py` tree; delays 6 s camera, 2 s LiDAR, 10 s localization, 15 s mapping/Nav2 and why (TF races, USB bandwidth) | CLAUDE.md §Architecture | FIG-LAUNCHTREE (mermaid) |

### S10 Hardware (T 3, R 3)

| # | T/R | Slide | Content | Source | Assets |
|---|---|---|---|---|---|
| 1.1 | T | Platform and components | Table: Jetson Orin Nano Super 8 GB, 1 TB NVMe; VESC 6 MkVI; RealSense D435i; YDLidar X4; Traxxas 4-Tec 2.0 VXL chassis and its steering servo; DualSense. Photo grid | Release doc §Hardware Components, `docs/attachments/.../media` | PHOTO-COMPONENTS |
| 1.2 | T | Where every sensor sits: the static frame tree | `base_link` = rear axle; offsets from CLAUDE.md frame tree; camera frames from URDF; rendered `view_frames` output | CLAUDE.md §Static TF Frame Tree, `static_transformations.launch.py`, `urdf/f1tenth.urdf.xacro` | FIG-TF-TREE |
| 1.3 | T | Two batteries, two failure modes | Jetson pack vs VESC drive pack; XT90; powerbank; VESC `voltage_input` is the drive pack only, the Jetson supply has no topic. Electrical diagram placeholder | Release doc §Wiring, memory `jetson-and-vesc-are-on-separate-batteries` | PHOTO-WIRING, FIG-ELEC |
| 1.4 | R | Assembly | Rollcage, body shell, NPF adapter, sensor mount photos | `docs/figures/teleop/*` | PHOTO-ASSEMBLY |
| 1.5 | R | Network | Robot WiFi/wired, velox1 as remote RViz host, DDS profiles, remote bandwidth (RViz set ~99 kB/s, one image stream 31 MB/s) | Release doc §Network, CLAUDE.md §CycloneDDS | TABLE-BANDWIDTH |
| 1.6 | R | Controller pairing and button map | BR/EDR pairing; SDL mapping L1=9, R1=10, PS=5; labelled DualSense figure already exists | Release doc §DualSense, `docs/figures/teleop/dualsense_top_with_arrows_and_labels.png` | PHOTO-DUALSENSE |

### S20 Sensing (T 5, R 2)

| # | T/R | Slide | Content | Source | Assets |
|---|---|---|---|---|---|
| 2.1 | T | Sensing pipeline | Excalidraw sensing diagram: stereo, LiDAR, wheel/steer, IMU, preprocessing filters, rates | Obsidian `1_Sensing` | FIG-ARCH-01 |
| 2.2 | T | Camera streams | Table per topic: RGB, infra1/2, depth, aligned depth, colored pointcloud, IMU. Columns: resolution, configured fps, **measured** Hz, QoS, on/off by default. Emitter-off trade-off (VSLAM vs depth quality), decimation 2, `ordered_pc: false` | `config/sensors/realsense_config.yaml`, `realsense_d435i.launch.py`, `bag_stats.py` output | TABLE-CAMERA, CHART-RATES |
| 2.3 | R | Stereo to depth to pointcloud | Excalidraw stereo pipeline (rectification, census/cost, disparity, projection) | Obsidian `1A_StereoVision` | FIG-ARCH-01A |
| 2.4 | T | 2D LiDAR | X4: 0.12-10 m (12 m spec caused phantoms), ~625 beams, 12 Hz configured / ~8 Hz observed, no intensity, `inf` for out-of-range; speckle filter `lidar/scan` to `lidar/scan_filtered` | `config/sensors/ydlidar_X4.yaml`, `config/filters/laser_filter.yaml`, CLAUDE.md | VID-SENS-LIDAR |
| 2.5 | T | Two IMUs, two treatments | VESC IMU ~100 Hz, no magnetometer, onboard Madgwick, yaw not fused; D435i IMU 200 Hz through `imu_filter_madgwick` with `constant_dt` 0.005 and why (first samples have bogus stamps) | CLAUDE.md §ekf_odom, `imu_filter.launch.py` | TABLE-IMU |
| 2.6 | T | Wheel odometry | `vesc_to_odom`: speed = erpm/3750, angle = (servo-0.56)/-1.1448, yaw rate = v tan(delta)/0.256; silent until first servo command | `config/vehicle/vesc.yaml`, CLAUDE.md | — |
| 2.7 | T | Measured rates and bandwidth | One chart of measured topic rates from a drive bag, with per-stream bandwidth (the image set is ~125 MB/s; the IMU-only `sysid` set is ~0.5 MB/s) | `bag_stats.py`, CLAUDE.md §Debugging Caveats | CHART-RATES |

Sensor demo clips (VID-SENS-RGB, -DEPTH, -CLOUD, -LIDAR, -IMU) are R except LiDAR; the talk shows one.

### S30 Localization (T 5, R 3)

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

| # | T/R | Slide | Content | Source | Assets |
|---|---|---|---|---|---|
| 4.1 | T | 2D mapping | SLAM Toolbox (0.05 m, 12 m) vs RTABMap grid (LiDAR `Grid/Sensor 0`, ray tracing); the current `20260805` grid rendered | `2d_mapping_*.yaml`, `data/maps/20260805/rtabmap_2d_final.pgm` | FIG-MAP-2D, VID-MAP-2D-TIMELAPSE |
| 4.2 | T | 3D mapping | RTABMap RGB-D (CPU) or nvblox (GPU); render of `cloud_voxel_0p05.pcd` in the grid frame | `3d_mapping.launch.py`, `data/maps/20260805/cloud_voxel_0p05.pcd` | FIG-MAP-3D, VID-MAP-3D-ORBIT |
| 4.3 | R | One database, three artifacts | Grid, cloud and AMCL seed must come from the same RTABMap database; save commands | CLAUDE.md, `COMMANDs.md` | — |
| 4.4 | R | RTABMap vs SLAM Toolbox grids | Side-by-side stills | shot list 3.2 | FIG-MAP-COMPARE |

### S50 Perception (T 4, R 2) — owner: perception chat

| # | T/R | Slide | Content | Source | Assets |
|---|---|---|---|---|---|
| 5.1 | T | Obstacle detection pipeline | Excalidraw: 2D detection + depth projection, pointcloud detection, laser detection, merger, tracker | Obsidian `2B_ObstacleDetection` | FIG-ARCH-03 |
| 5.2 | T | LaserScan-based detection | `autodriver_laser_segmentation` (repo location TBD, D4): method, rate, output type | owning repo | FIG-PERC-LASER |
| 5.3 | T | Pointcloud pipeline | `autodriver_pointcloud_preprocessor` filters then `autodriver_pointcloud_object_detection`; rates on Orin | owning repos | FIG-PERC-CLOUD |
| 5.4 | T | Image and 3D | `autodriver_image_object_detection`: YOLOv8 (ultralytics 8.3.86, onnxruntime-gpu), batch inference + tracking; 2D box to 3D via aligned depth; `ros_multi_object_tracker` | owning repos | FIG-PERC-IMAGE |
| 5.5 | R | Costmaps as perception | Nav2 obstacle + inflation layers, `nonpersistent_voxel_layer`; sample costmap | `nav2_params.yaml`, `data/maps/sample_costmap.png` | FIG-COSTMAP |
| 5.6 | R | Perception demo | 2024 clip if allowed (D2); otherwise placeholder | `docs/perception_and_gotogoal_planning.mp4` | VID-PERC-2024 |

### S60 Planning and Nav2 (T 4, R 2)

| # | T/R | Slide | Content | Source | Assets | Owner |
|---|---|---|---|---|---|---|
| 6.1 | T | Planning pipeline | Excalidraw: route planning 1 Hz, behavior/local planning 10 Hz, sparse vs dense waypoints | Obsidian `3_Planning` | FIG-ARCH-04 | launch |
| 6.2 | T | Waypoint recording and loading | `waypoint_recorder.py` / `waypoint_loader.py`: format, frame, how a figure-8 was recorded | trajectory_following_ros2 | FIG-WAYPOINTS | control |
| 6.3 | T | Nav2 as configured | bt_navigator + custom BT XMLs, planner (name from `nav2_params.yaml`), RPP controller 10 Hz, costmaps 5 Hz / 1 Hz, velocity smoother, `twist_to_ackermann`; per-server toggles; MPC can replace `controller_server` through `FollowPath` | `nav2_params.yaml`, `nav2_navigation.launch.py`, CLAUDE.md | FIG-NAV2-BT | launch |
| 6.4 | T | Nav2 on the car | 2026-08-27: 5.78 m path, 203/203 commands reached the VESC, stopped 0.379 m from goal (tol 0.25 m), heading 4.8 deg (tol 14.3 deg); last command 0.269 m/s vs breakaway 0.20-0.26 m/s; progress checker and recovery ladder observed; obstacle avoidance untested | CLAUDE.md §Nav2, `nav2_drive4` bag on SSD | CHART-NAV2-APPROACH, VID-NAV2-LIVE | launch |
| 6.5 | R | Go-to-goal: Nav2 vs the MPC node | The two go-to-goal paths (Nav2 `NavigateToPose`; the MPC node's goal mode), their interfaces, and which has driven the car (the Nav2 run in 6.4 is the only one documented in this repo; the control chat states the MPC node's status) | trajectory_following_ros2, CLAUDE.md | — | control |
| 6.6 | R | Offline Nav2 replay results | goal accept 1-2 ms, first plan <0.05 s, cmd_vel 20 Hz bounded 0.5 m/s | `NAV2_OFFLINE_RESULTS.md` | — | launch |

### S70 Hierarchical control (T 5, R 3) — owner: control chat; 7.5-7.6 vesc chat

| # | T/R | Slide | Content | Source | Assets |
|---|---|---|---|---|---|
| 7.1 | T | Control architecture | Excalidraw: high-level controller, lateral/longitudinal low-level, rates 100/200/16000 Hz | Obsidian `4_Control` | FIG-ARCH-05 |
| 7.2 | T | High-level controllers | Table: **MPC from `trajectory_following_ros2`** (acados / casadi / do_mpc backends, coupled kinematic model; modes: waypoint following, go-to-goal, obstacle avoidance), pure pursuit (same repo), Nav2 RPP, Nav2 MPPI (secondary, evaluated), joystick. Columns: rate, horizon, model, constraints (delta ±0.314 rad, speed bounds), obstacle-aware, has driven the car | trajectory_following_ros2, `nav2_params.yaml` | TABLE-CONTROLLERS |
| 7.3 | T | MPC: waypoint following | Offline figure-8 and loop runs (rerun recordings) and live drives; tracking error chart; acceptance `k ≈ 0.96`; LUCIO ego-MPC one line | control chat, `SYSID_RESULTS.md` | VID-MPC-RERUN, VID-MPC-LIVE, CHART-MPC-TRACKING |
| | | | **CORRECTED 2026-09-03 (§9)** — this row carried two errors: `k ≈ 0.96` is a *steering-calibration* acceptance band, not a tracking metric (it belongs on 9.3), and the CTE p95 of ~4.6 cm was measured over all logged ticks, ~90 % of them parked. Written numbers are the route-acquired figures: loop 3.0 / **13.1** cm, figure-8 5.0 / **18.8** cm. | | |
| 7.4 | T | MPC: go-to-goal and obstacle avoidance | How obstacles enter the problem (constraints or cost), what the obstacle source is today (`autodriver_fake_obstacle_publisher` vs perception), RViz and live placeholders with and without obstacles | control chat | VID-MPC-OBST-RVIZ, VID-MPC-OBST-LIVE |
| 7.4b | R | Nav2 MPPI | Configuration tried, result, why it is secondary | control chat, `nav2_params.yaml` | — |
| 7.5 | T | Low-level actuation chain | `ackermann_to_vesc`: erpm = 3750 v, servo = -1.1448 delta + 0.56, clamp [0.08, 0.92]; saturator; optional closed-loop speed PI (`speed_kp/ki`, anti-windup), adaptive FF, accel FF, rate limit, all default off; `throttle_interpolator` (fixed, optional). Owner confirms whether a steering PID exists (template: "confirm from my vesc fork") | `vesc.yaml`, `bringup.launch.py`, vesc fork | FIG-LOWLEVEL |
| 7.6 | R | Actuation lag | 60 ms delay + 40 ms first-order; throttle transport <20 ms; step-response plot | `SYSID_RESULTS.md` | CHART-LAG |
| 7.7 | R | Pure pursuit details | lookahead, gains | control chat | — |
| ~~7.8~~ | — | ~~Joystick as a controller~~ | **RETIRED 2026-09-03 (§9): a duplicate of 8.4**, which already carries the axis mappings, `max_speed`, `max_steering` 0.314 and PHOTO-DUALSENSE from the same two files. Not reassigned — dropped. | — | — |

### S80 Vehicle interface and safety (T 4, R 2)

| # | T/R | Slide | Content | Source | Assets |
|---|---|---|---|---|---|
| 8.1 | T | Every command passes two gates | Diagram: `teleop`/`drive`/`estop` -> `ackermann_mux` (255/100/10/1, timeouts 0.5/0.3/0.2/0.05 s) -> `ackermann_drive` -> `command_gate` -> `vehicle/ackermann_cmd` -> `ackermann_to_vesc` -> VESC | `mux.yaml`, `command_gate.yaml`, CLAUDE.md | FIG-CMDPATH (mermaid) |
| 8.2 | T | Deadman and handover | Heartbeat on `command_gate/heartbeat`, 0.5 s timeout; L1 manual, R1 autonomous handover; gate closes 0.5-0.63 s after joystick loss; commanded motion bounded to 0.2-0.35 s; never use PS in a deadman set. **Why the heartbeat has its own topic** (one line): the R1 handover silences `teleop` by design, so a `teleop`-based heartbeat would close the gate 1 s into every autonomous run | CLAUDE.md, `joy_teleop.yaml` | TABLE-SAFETY-TIMING, VID-SAFETY-MUX (R) |
| 8.3 | T | Operating procedures | Pre-op checklist, power-up order, connection, normal operation, shutdown, emergency stop; battery management | Release doc §Operating Procedures, §Battery Management | — |
| 8.4 | T | Teleop controls | Labelled DualSense figure with speed/steer scales | `dualsense_top_with_arrows_and_labels.png`, `joystick.launch.py` | PHOTO-DUALSENSE |
| 8.5 | R | Three gate configurations | full safety / passthrough / gate removed, and what each implies | CLAUDE.md | — |
| 8.6 | R | Teardown and orphans | why `kill -INT` is not enough; `stop_launch_tree()` | CLAUDE.md §Script teardown | — |

### S90 System identification and calibration (T 5, R 2)

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

| # | T/R | Slide | Content | Source | Assets |
|---|---|---|---|---|---|
| 10.1 | T | What is next | Nav2 goal reach (0.38 m stall), servo offset bench sweep, obstacle avoidance test on the car, MPC go-to-goal on the car, perception integration into bringup, `servo_min/max` measurement, closed-loop speed enable after recalibration | CLAUDE.md, `LOCALIZER_FOLLOWUPS.md` | — |
| 10.2 | R | Lessons for measuring on this car | RealSense IMU first stamps and `constant_dt` (attitude jump 80-167 deg to 0.19 deg); rf2o scale error, so never calibrate against one reference; check `odometry/local` twist before trusting a pose; verify streams by message count, not rate | CLAUDE.md | — |

Design choices themselves are explained on the slides where they apply (3.3 fusion, 8.2 heartbeat, 2.5 IMU treatment), not collected here.

### S99 Research questions (research cut only; T 3-4)

Each owning brief contributes one or two candidate questions from its area, phrased as a question the platform can answer with an experiment we can run. Seeds from this repo, for the owners to accept, reject or replace:

| # | Slide | Candidate question | Evidence already in hand |
|---|---|---|---|
| 11.1 | What this platform can measure | One slide: the sensor set, the three independent ground-truth references (tape, VSLAM, RTABMap loop closure), the bag tooling, and the cost of a run | S90, `scripts/analysis/` |
| 11.2 | Estimation | Can actuator calibration (gain, offset, deadband) be identified from ordinary driving with enough accuracy to replace bench measurement, given that three driven fits disagreed with the bench value of the servo offset by up to 0.009 servo units (47 cm of drift over 5.5 m)? Which odometry reference is trustworthy for scale, given rf2o at -6 % and VSLAM at +1 % against tape? | 9.3, 9.4, 3.4 |
| 11.3 | Control | **Sharpened 2026-09-03 from the control hand-back §9:** the actuator's floor sits *above* the controller's ceiling — the plant cannot move below 0.20-0.26 m/s, while an MPC publishing the one-step state `v + a·dt` can command at most `max_accel·dt` = 0.15 m/s from rest. How should terminal and standstill behaviour be formulated when the plant has a hard breakaway speed, and what does ~100 ms of actuation lag then force on the horizon and the rate? Evidence in hand: the 2026-08-07 speed staircase (0.18 m/s on stands), the 2026-08-08 ground A/B (un-floored creep pass-through 21.6 % -> 0 % over 16 838 qualifying ticks), and the Nav2 approach on 6.4 that stalled at 0.269 m/s, 0.379 m short. Runnable as it stands: a commanded-speed staircase scored against tape on the ground, then the same terminal approach re-driven with and without a floor. | 6.4, 7.2b, 7.6, control brief |
| 11.4 | Perception and fusion | Which detector (laser, pointcloud, image + depth) gives obstacle states good enough for the MPC at what rate on the Orin, and does fusing them help? | perception brief |

**Cut totals (estimates, integration chat confirms from `build.sh`)**: `lab` 42, `sponsor` 32, `research` 38. Reference-only: 26. Full deck: 72.

---

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
| CHART-RATES | chart | ROBOT | `scripts/analysis/bag_stats.py` on a `sysid` bag on the SSD (`armA_loop2`), output CSV, plot with matplotlib. |
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

---

## 5. Cross-repo briefs (what each file in `briefs/` must say)

Every brief has the same shape: **slides you own** (IDs from §3), **what exists in your repo** that answers them (the brief lists what this scan found so the chat starts from evidence), **assets you must produce** (IDs and target paths), **conventions** (§1), **what you must not do** (fabricate media, quote numbers without a source, reference agent files on slides), **cut tagging** (tag each slide with `lab`/`sponsor`/`research` per §2a; return a ranked list of what your area can actually show today so the `sponsor` cut is built from real material), **research questions** (one or two for S99, with the evidence you already have), and **hand-back** (which files to edit, how to run `build.sh` to check).

### BRIEF_f1tenth_launch.md
Owns S00, S10, S20, S30, S40, 6.1, 6.3, 6.4, 6.6, S80, S90, S95. Sources are all in this repo (`config/`, `launch/`, CLAUDE.md, `scripts/live_runs/*.md`, `docs/F1tenthDocumentation_v1_1_Release.md`). Probably two chats: **B1** = S00, S10, S20, S80 (hardware/sensing/safety, mostly config and the 2024 doc) and **B2** = S30, S40, S60 parts, S90, S95 (localization/mapping/Nav2/sysid, mostly measured results).

### BRIEF_trajectory_following.md
Owns 6.2, 6.5, 7.1-7.4, 7.7. Found locally: `~/f1tenth_ws/src/trajectory_following_ros2` (stale, Jan 2025) with `coupled_kinematic_{acados,casadi,do_mpc}.py`, `ackermann_purepursuit.py`, `purepursuit/`, `waypoint_recorder.py`, `waypoint_loader.py`, `twist_to_ackermann_drive.py`, `simulator/`, `launch/mpc.launch.py`, `launch/trajectory.launch.py`. The owning chat works from the current branch (`refactor/unify-backends` carries the 2026 steering fix). The MPC is the flagship: it does waypoint following from recorded waypoints, go-to-goal, and obstacle avoidance, and the brief asks for all three to be shown with their real status. Nav2's MPPI is secondary (one table row, one reference slide). Must say plainly which controllers and modes have driven the car and which have only run in simulation or replay, and where the existing rerun/live recordings are. Steering bound ±0.314 rad and the LUCIO `delta_bound` relationship come from this repo's CLAUDE.md notes.

### BRIEF_perception.md
Owns S50. Found locally: `autodriver_image_object_detection` (launch `bringup`, `components/stream`, `components/object_detection`; branch `batch_inference_and_tracking`), `autodriver_pointcloud_object_detection`, `autodriver_pointcloud_preprocessor`, `autodriver_fake_obstacle_publisher`, `autodriver_icp_localizer`; `ros_multi_object_tracker` under `Projects/autodriver/autodriver_perception/`. `autodriver_laser_segmentation` **is on this machine but not under `~/f1tenth_ws/src`**; its path (and the current paths of the other autodriver repos, which may be newer than the workspace copies) come from the user, who keeps them as aliases at the bottom of `~/.bashrc`. The brief must record the resolved paths once given. Must state for each node: input topics, output type, measured rate on the Orin (or TBD), and whether it is wired into any launch file in `f1tenth_launch` (today: none are in bringup; `nav2_perception.launch.py` is standalone).

### BRIEF_f1tenth_system_vesc.md
Owns 7.5, 7.6 and reviews 8.1-8.2 for accuracy. Found: `f1tenth_system` (`vesc_driver`, `vesc_ackermann` with `ackermann_to_vesc.cpp` and `vesc_to_odom.cpp`, `ackermann_mux`, `teleop_tools/joy_teleop` fork with the `default` pseudo-button and heartbeat, `f1tenth_stack/throttle_interpolator.py`), and the `vesc` fork (`humble-devel`) carrying `use_closed_loop_speed`, `speed_kp/ki`, `use_adaptive_ff`, `use_accel_ff`, `accel_to_erpm_gain`, `use_cmd_accel_rate_limit`. Must confirm or refute the template's "cascade PIDs: long_vel, steering -> ERPM, PWM" and draw FIG-LOWLEVEL from the code, not the template. Must state defaults and saturation limits.

### BRIEF_robot_capture.md
Not a repo, a lab session. Lists every `ROBOT` asset in §4 with the exact command (from `demo_shot_list.md` and `scripts/analysis/`), the bags to use (`armA_loop`, `armA_loop2`, `armA_straight_5m`, `armA_steer_sweep`, `nav2_drive4` on the SSD), the RViz configs (`rviz/demo_*.rviz`), and the operator rules already learned (operator launches bringup, agent checks health; record with `BAG_TOPIC_SET=sysid`; leave 2 s still at clip ends). Also TABLE-PACKAGES. Order: offline charts from existing bags first (no driving), then parked clips, then the one driving session for VID-NAV2-LIVE and VID-LOC-LOOP.

---

## 6. Work split and order

| Phase | Who | Does | Depends on |
|---|---|---|---|
| 0 | **You** | Export the 7 Excalidraw diagrams (SVG + 2x PNG) into `assets/figures/arch/`; give the perception chat the `autodriver_laser_segmentation` path (and current autodriver repo paths) | — |
| A | **Scaffold chat** (one, short) | Create `docs/presentation/` per §1: theme, `build.sh` (concat, `talk` filter, placeholder-ID check, marp to pdf/html/pptx), `ASSETS.md` from §4, the five briefs from §5, empty section files with slide titles and `<!-- talk -->` tags from §3, `.gitignore` for `assets/video/` and `out/`. Verify `./build.sh talk` renders a skeleton PDF | 0 |
| B1, B2 | **f1tenth_launch chats** (two, parallel) | Fill their sections; produce OFFLINE assets (maps renders, mermaid diagrams, tables) | A |
| C | **Control chat** | S70 high-level, 6.2, 6.5; list which MPC/MPPI videos exist | A, D1 |
| D | **Perception chat** | S50 | A, D4 |
| E | **vesc/f1tenth_system chat** | 7.5, 7.6; review 8.1-8.2 | A |
| ~~F~~ | | **SPLIT 2026-09-03 into F1 and F2** — the one phase bundled two jobs needing different things, and only one needs the lab. | |
| F1 | **Offline chart chat** (`briefs/BRIEF_charts.md`) | The 7 `ROBOT` assets that are processing of bags **already recorded**: CHART-RATES, CHART-CLOSURE, CHART-STEER, CHART-SPEED, CHART-NAV2-APPROACH, FIG-MAP-COMPARE, TABLE-PACKAGES. Plus the plotting path `fit_actuators.py` does not have. No car, no battery, no driving. Inputs verified present on the gosling1 SSD 2026-09-03. | A |
| F2 | **Robot capture session** (you + one chat on gosling1) | The 11 `VID-*` clips. `VID-MPC-OBST-RVIZ` and `VID-MPC-OBST-LIVE` are **not capturable** — that mode has never run on the car — and stay cards (§9 G4). | A; the lab |
| G | **Integration chat** | Merge, enforce conventions (source notes, placeholder IDs, cut tags, no agent-file references on slides), cross-check every number against CLAUDE.md, arbitrate cut membership to the §2a targets, produce `out/deck_lab.pdf`, `out/deck_sponsor.pdf`, `out/deck_research.pdf`, `out/deck_full.pdf` | B-F (`sponsor` also needs F complete) |

B, C, D, E and F1 can run at the same time and touch disjoint files, so merges are trivial. F2 can happen any day the lab is free; the deck is presentable with cards before it.

---

## 7. Success criteria

1. `./build.sh lab`, `sponsor`, `research` and `full` exit 0 and produce PDF, HTML and PPTX; the PDF is self-contained for a slides-only room.
2. `lab` cut: 40-42 slides, every slide title is a claim, dry-run reads in 25 min or less. `sponsor` and `research` cuts within their §2a targets. The `sponsor` cut contains no placeholder cards for video once Phase F is done.
3. Every number on a slide has a `src:` speaker note with file and date; the integration grep finds none without.
4. Every media element is either a real file listed in `ASSETS.md` with its provenance, or a placeholder card whose ID the register explains. No stock, staged or synthetic media.
5. Every quantity the template asked for (rates, latency, resolution, defaults, min, max, pipelines) appears in a table or diagram for each subsystem, or is marked `TBD` with the measurement that would settle it.
6. The 2024 material appears only with a visible "2024 stack" label, or not at all (D2).
7. No slide references `CLAUDE.md`, `PLAN.md`, bug IDs or other agent files.
8. `docs/presentation/` is committed; `assets/video/` and `out/` are not.

---

## 8. Decisions log (resolved 2026-09-02)

| # | Question | Resolution |
|---|---|---|
| D1 | Which MPPI | Nav2's plugin, and it is secondary. The `trajectory_following_ros2` MPC (waypoint following, go-to-goal, obstacle avoidance) is the flagship and owns 7.3-7.4. |
| D2 | 2024 media | Reuse what is needed, labelled "2024" on media. Hardware content is current; 2024 software descriptions are treated as stale. |
| D3 | Lessons | No bugs slide in the talk. Design choices are explained in place (3.3, 8.2, 2.5). Image-recording starvation is out. A measuring-lessons slide is reference-only (10.2). |
| D4 | Laser segmentation path | Exists on this machine; user supplies the path from the `~/.bashrc` aliases when the perception chat starts. |
| D5 | Audience, purpose, setting, depth | Three audiences served by three cuts of one source (§2a): `lab` main, `sponsor` fused/demo-led, `research` for paper questions. Purpose: honest status core, showcase in `sponsor`, rationale in `lab`. In person, slides only. ROS-literate depth. ~25 min. |
| D6 | ESC model | VESC 6 MkVI; chassis Traxxas 4-Tec 2.0 VXL (2024 doc, Hardware Components). |
| D7 | Diagram export format | SVG for the deck, 2x PNG alongside for review and PPTX. PNGs of all seven received 2026-09-02 and parked in the planning scratchpad; SVGs still to export. |

Still open: whether the new diagrams (FIG-CMDPATH, FIG-ELEC, FIG-LOWLEVEL) are drawn in Excalidraw to match the existing seven or in mermaid inside Marp. Default is mermaid unless an Excalidraw authoring tool is set up.


---

## 9. Phase-G arbitration log

Decisions taken by the integration chat on material escalated in the hand-backs.
Each entry names the hand-back that raised it. Entries here **override §3**.

### 2026-09-03 — `briefs/HANDBACK_trajectory_following.md` (control chat)

| # | Item | Decision |
|---|---|---|
| G1 | Nav2 rows of TABLE-CONTROLLERS, filled by the control chat and flagged "please confirm" | **Confirmed.** All seven values re-read from `config/nav2_params.yaml` on 2026-09-03 and correct as written. Recorded on the slide. Two notes carried to 7.4b rather than the table: MPPI has its own `vx_max: 0.75`, and its `min_turning_r: 0.462` is derived from 30° of steering against this car's 18° (0.788 m) — inert only because RPP is the selected plugin. |
| G2 | 7.8 ownership: scaffold said control chat, `BRIEF_trajectory_following` §1 said f1tenth_launch chat, `BRIEF_f1tenth_launch` §1 listed it nowhere | **Row retired, not reassigned** — it is a duplicate of 8.4. See the §3 row and the closing note in `slides/70_control.md`. |
| G3 | PHOTO-DUALSENSE held at `EXISTS` because 7.8 still carded it | **Flipped to DONE.** Nothing cards it any more. |
| G4 | `rviz_obstacle_detection.mp4` rejected as VID-MPC-OBST-RVIZ (no MPC path, no keep-out — it is detection footage) | **Upheld and re-routed to the perception brief**, with the condition that no slide may imply those detections reached a controller. Both obstacle videos stay cards. |
| G5 | `k ≈ 0.96` quoted on plan row 7.3 as a tracking acceptance | **Upheld: the plan was wrong.** It is the steering-calibration band agreed with the downstream group on 2026-08-08 (accepted 0.95–1.02). It belongs on 9.3 and is not quoted on 7.3. |
| G6 | Plan row 7.3's cross-track p95 of ~4.6 cm | **Upheld: the plan was wrong.** Measured over all 14 322 ticks, ~90 % of them parked at the route start with the deadman released. The route-acquired figures (13.1 / 18.8 cm) stand. |
| G7 | Plan's preferred figure-8 CSV `data/fig8_video/solver_155008.csv` | **Upheld: the plan was wrong.** It is a breakaway-floor bench run (1.12 m of travel, `ref_idx` never leaves 5). The chart is built from the two 2026-08-06 hardware legs, whose logs are in the repo. |
| G8 | New slide 7.2b (MPC formulation), reference-only, added by the control chat | **Kept.** The section had no place where the optimal-control problem was stated. Nothing renumbered. S70's header count ("T 5, R 3") was already known to disagree with its rows; with 7.2b added and 7.8 retired it reads T 5, R 4 — the rows are authoritative, the header is not. |
| G9 | 6.2 cut tag widened `lab` -> `lab research`, deliberately not into `sponsor` | **Accepted.** The reasoning (§2a drops format/config detail from `sponsor`; that cut is already over target) is the right one. |
| G10 | S99 research question for 11.3 | **Accepted and folded into the §3 row 11.3** verbatim in substance. |

| G11 | Phase F is one phase covering 29 assets, 7 of which need no lab | **Split into F1 (offline charts, `briefs/BRIEF_charts.md`, written 2026-09-03) and F2 (video capture).** F1's inputs were verified present on the gosling1 SSD the same day. |

Build state after these edits: `STRICT=1 ./build.sh full md` clean, 71 slides;
`check_overflow.mjs` reports one overflow, slide 31 (perception's), unchanged.
