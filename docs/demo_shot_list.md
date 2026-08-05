# Demo Shot List

What to launch, what to show, and how long, for each clip in the presentation.
Clips are short on purpose — one idea each, so they can be reordered freely.

Record with `scripts/live_runs/record_screen.sh <name> --secs <n>`, or any
desktop recorder against `DISPLAY=:0`. Output lands in
`/mnt/shared_dir/videos/<date>/`.

Filming notes that apply throughout:
- Full-screen RViz, dark background, grid off unless the grid is the point.
- Move the car (or the object) *slowly* — motion blur reads badly compressed.
- Leave 2 s of still frame at the start and end of each clip for editing room.

---

## 1. Sensors

| # | Clip | Setup | What to do | Length |
|---|---|---|---|---|
| 1.1 | RGB stream | `./20_sensor_bag.sh --launch-only`, RViz `rviz/demo_sensors.rviz` | Image panel full width. Wave a hand, walk an object through frame. | 15 s |
| 1.2 | Depth | same | Switch the image panel to `camera/aligned_depth_to_color/image_raw`. Move an object toward the camera so the colour ramp changes. | 15 s |
| 1.3 | Colored pointcloud | `./20_sensor_bag.sh --pointcloud --launch-only` | Orbit the 3D view around the cloud. This is the money shot — do a slow, smooth orbit. | 20 s |
| 1.4 | LiDAR | same | `lidar/scan_filtered` as spheres, top-down view. Walk around the car so the scan silhouette changes. | 15 s |
| 1.5 | IMU | same | RViz IMU display on `camera/imu/filtered`. Tilt the car by hand (battery disconnected). | 10 s |

**Talking point for 1.3:** the cloud is off by default; we measured whether
publishing it costs camera frames before deciding — the numbers come from
`./90_inspect_bag.sh` comparing the two bags.

---

## 2. Localization

| # | Clip | Setup | What to do | Length |
|---|---|---|---|---|
| 2.1 | Particle convergence | `./50_localization_test.sh --launch-only --map <map>` | Start with a deliberately vague initial pose. Rotate the car; film the particle cloud tightening. | 20 s |
| 2.2 | Live loop | full `./50_localization_test.sh` | Trailing paths for `odometry/local` (blue) and `odometry/global` (green) over the map, following the car for a full lap. | 40 s |
| 2.3 | TF tree | any localization run | `ros2 run tf2_tools view_frames --ros-args -r __ns:=/gosling1`, then show the PDF. Static image, not video. | still |

**The centrepiece is not a video.** It's `docs/figures/localization/closure_error.png`
from `scripts/analysis/plot_localization.py` — the bar chart of how far each
estimator thinks it moved when the car physically returned to its start. That
single figure is the argument for sensor fusion. `summary.md` next to it has
the numbers in a table ready to paste onto a slide.

---

## 3. Mapping

| # | Clip | Setup | What to do | Length |
|---|---|---|---|---|
| 3.1 | 2D map building | `./40_build_map_offline.sh --mode slamtoolbox --rate 0.5` | Time-lapse of the occupancy grid filling in during replay. Higher `--rate` makes a better time-lapse; the map quality doesn't matter for the clip. | 30 s |
| 3.2 | RTABMap 2D vs SLAM Toolbox | after `--mode both` | Side-by-side still of the two `.pgm` files. Shows the comparison was actually done. | still |
| 3.3 | 3D map | `rtabmap-databaseViewer <db>` | Slow orbit of the exported cloud. Voxelize at 0.02–0.05 m first or the Orin will stutter. | 25 s |
| 3.4 | Map + live scan | localization run | Map underlay with live `scan_filtered` on top, showing the scan aligning to the walls. | 15 s |

---

## 4. Nav2

| # | Clip | Setup | What to do | Length |
|---|---|---|---|---|
| 4.1 | Costmaps | `./60_nav2_test.sh --dry-run --map <map>`, `rviz/demo_nav2.rviz` | Show global then local costmap, inflation visible around obstacles. | 15 s |
| 4.2 | Planning | dry run | Send a goal with the 2D Goal Pose tool. Film the global plan appearing. Car does not move — say so on camera. | 20 s |
| 4.3 | Live navigation | `./60_nav2_test.sh --map <map>` | Full goal-to-arrival run, RViz and the car in the same frame if you can manage two cameras; otherwise RViz with the car audible. | 45 s |
| 4.4 | Dynamic obstacle | live | Step into the planned path; film the local costmap reacting and the plan re-routing. | 20 s |

---

## 5. Optional: vehicle & safety

Worth 30 s if the audience cares about safety architecture.

| # | Clip | What to show |
|---|---|---|
| 5.1 | Mux priority | `ros2 topic hz` on `teleop`, `drive`, `ackermann_drive` side by side while you press and release the deadman. Shows arbitration live. |
| 5.2 | Command gate | Release the joystick entirely; `vehicle/ackermann_cmd` goes to zero. The car stops. |
| 5.3 | Autonomous handover | Press button 5; `ackermann_drive` switches from mirroring `teleop` to mirroring `drive`. |

---

## Assembly order for the deck

1. Hardware still (from `docs/figures/`)
2. Sensors: 1.1 → 1.2 → 1.3 → 1.4
3. Localization: 2.1, then the closure-error figure, then 2.2
4. Mapping: 3.1 → 3.3
5. Nav2: 4.2 → 4.3 → 4.4
6. Safety: 5.1–5.3 if time

The `docs/Presentation.md` outline maps onto this; filling it in and building
the slide deck is the next session's work.
