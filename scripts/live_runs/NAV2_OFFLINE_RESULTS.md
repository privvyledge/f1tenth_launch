# Nav2, driven from a bag — first goals ever sent

**2026-08-06, 15:07–15:35 EDT, `gosling1`, `ROS_DOMAIN_ID=51`, no vehicle.**
Commit under test: `perf/config-tuning` HEAD plus the two fixes below.
Container `f1t_nav2_0806` (`privvyledge/f1tenth:humble-devel-08052026`, HEAD
staged over `src/`, `--init`).

Nav2 had never been given a goal on this vehicle. The 2026-08-06 bench bringup
established only that the eight servers come up. This run puts a goal through
the whole stack — BT, planner, controller, recoveries — using
`mapping_drive_170025` to supply the pose, the scan and the odometry.

**Verdict: the Nav2 stack works.** Goals are accepted, plans come back in
0.00–0.05 s, the controller emits continuous bounded velocity at 20 Hz, the
recovery subtree fires and completes, and nothing crashed, hung or pegged a
core. Two real defects came out of it, both fixed and both re-measured.

## How to reproduce

```bash
export ROS_DOMAIN_ID=51          # NOT 42 — the offline localization scripts use it
export CYCLONEDDS_URI=file:///mnt/shared_dir/cyclonedds_offline_lo.xml
export MAP_ROOT=/mnt/shared_dir/maps/20260805
export BAG_ROOT=/mnt/shared_dir/bags/20260805
cd /workspaces/f1tenth/src/f1tenth_launch/scripts/live_runs
./61_nav2_offline.sh --bag $BAG_ROOT/mapping_drive_170025 --goal-timeout 30
```

`61_nav2_offline.sh` composes `localization.launch.py` (map_server + AMCL, the
same seeded configuration as `51_localize_offline.sh`) with
`nav2_navigation.launch.py`, replays the bag, and runs `nav2_goal_probe.py`,
which sends goals and measures what each server did. Goals come from
`goal_poses_from_bag.py` — poses the car actually occupied, so a failed plan
can never be blamed on a goal inside a wall.

Runs kept: `bags/20260805/nav2bag{2,3,4,5}_mapping_drive_170025`, logs in
`/mnt/shared_dir/run/nav2_offline_*`.

## What a bag test can and cannot prove

The bag drives the pose, so **the loop is open**. The controller steers toward
the path from wherever the bag has put the car; it cannot close the error. Two
consequences that look like results and are not:

- **`SUCCEEDED` does not mean Nav2 drove there.** Goals 0 and 1 succeeded
  because the bag's own trajectory passed within the 0.25 m goal tolerance. What
  it does prove is that the BT recognised arrival and terminated cleanly.
- **`RegulatedPurePursuitController detected collision ahead!` is expected
  here.** The commanded path and the replayed pose diverge, so the controller
  projects itself into a wall and aborts `follow_path` — which is the correct
  response, and it is what exercised the recovery subtree. It is *not* evidence
  of a controller defect, and it says nothing about closed-loop behaviour.

Closed-loop goal reaching still needs the car.

## Measured, run 4 (`nav2_offline_20260806_152715`)

| | goal 0 | goal 1 | goal 2 |
|---|---|---|---|
| straight-line distance | 1.51 m | 1.48 m | 3.23 m |
| accepted | yes, 2 ms | yes, 1 ms | yes, 1 ms |
| first plan | **0.05 s** | **0.00 s** | **0.00 s** |
| plan poses / length | 22 / 1.50 m | 22 / 1.50 m | 50 / 3.50 m |
| replans | 12 | 4 | 25 |
| `cmd_vel_nav2` | 235 msgs, **20.1 Hz** | 61, 20.3 Hz | 540, 17.4 Hz |
| max gap | 0.06 s | 0.06 s | **4.10 s** (the `Wait` recovery) |
| vx range | 0 … +0.5 | 0 … +0.5 | −0.5 … +0.5 |
| \|wz\| max | 0.19 | 0.46 | 1.63 |
| recoveries | 0 | 1 | 6 |
| outcome | SUCCEEDED | SUCCEEDED | still navigating at 30 s |

`max_planning_time` is 5 s and nothing came close: the planner answered in one
control period or less, every time. Velocity is bounded by
`desired_linear_vel: 0.5` exactly as configured, and the 4.1 s gap on goal 2 is
the `Wait wait_duration="5"` step of the recovery subtree, not a stall.

**CPU** (`top`, second-iteration sampling, zombies excluded):

| | peak | while planning |
|---|---|---|
| planner_server | 101 % | **2.5 – 5.5 %** |
| amcl | 11 % | ~6 % |
| bt_navigator | 9 % | ~2 % |
| controller_server | 8 % | ~2.5 % |
| behavior_server | 6 % | ~1 % |

The 101 % is a single sample **during `on_configure`**, when planner_server
builds the global costmap from the map — it is over before the first goal. This
is the explanation for the "planner_server at 94 % with no goal pending" note in
`60_nav2_test.sh`: it was a startup transient, measured with a tool that reports
lifetime averages. Steady-state planning is cheap.

`local_plan` never publishes: that topic is a DWB concept, and the configured
controller is `RegulatedPurePursuitController`. Not a defect; the topic stays in
the recorded set for whoever swaps in DWB or MPPI.

## Defect 1 (bug-125) — a dry run was not dry

`nav2_behaviors` creates its velocity publisher on the **relative name
`cmd_vel`** (`timed_behavior.hpp:130`), and neither this repo nor upstream
`navigation_launch.py` remapped it. Only the velocity smoother honoured
`cmd_vel_topic`. So with the smoother diverted to `cmd_vel_nav2` for a dry run,
a recovery still published straight onto `cmd_vel` — which on the car, with
`twist_to_ackermann` running, reaches `drive` → mux → command_gate → VESC.

**A `--dry-run` would have moved the vehicle the moment a recovery fired.**

Proved rather than argued: `nav2_goal_probe.py --exercise-backup` calls the
`BackUp` action directly (waiting for the BT to reach it does not work — the
RoundRobin clears both costmaps and waits 5 s first, so a 30 s goal window ends
before `BackUp` is ever ticked).

| | on `cmd_vel_nav2` | on `cmd_vel` | \|vx\| max |
|---|---|---|---|
| before | 0 | **6 non-zero** | **0.5 m/s** |
| after | **6** | 0 | — |

Fix: remap `('cmd_vel', cmd_vel_topic)` on `behavior_server` in both the
composable and non-composable paths of `nav2_navigation.launch.py`. In live mode
`cmd_vel_topic` **is** `cmd_vel`, so this is a no-op there; the only thing it
changes is that a diverted run is actually diverted.

## Defect 2 — `60_nav2_test.sh` could never have moved the car

Known and carried in the handoff, now fixed: the script never passed
`launch_twist_to_ackermann:=True`, so Nav2's `cmd_vel` had no route to `drive`.
It now passes `True` in live mode and `False` in `--dry-run`.

Verified hardware-free that the argument actually arrives — `bringup.launch.py`
does not declare it, and it reaches `vehicle.launch.py` only by launch-config
inheritance. A bringup with `launch_twist_to_ackermann:=True` shows
`/gosling1/twist_to_ackermann_converter` in `ros2 node list`. The node's own
`cmd_vel → drive` wiring is still untested on hardware.

## Two traps that cost a run each

- **Goals must be poses, not `truth_<bag>.csv`** (bug-128). That file is the
  `map->odom` transform over time; on this bag it never leaves a 0.3 m ball
  around the origin, so every goal landed on top of the robot and Nav2 reported
  success without planning. Use `goal_poses_from_bag.py`.
- **Wait for ACTIVE, not for topics** (bug-126). Costmap topics and action
  servers exist from CONFIGURE, but `nav2_util::SimpleActionServer` rejects
  every goal while inactive **and logs nothing**. The first run had all three
  goals rejected 1.5 s before the lifecycle manager activated the servers, which
  reads exactly like a Nav2 defect. The probe now gates on
  `<node>/get_state`.

And one measurement trap (bug-127): `ps -o pcpu` is a lifetime average, and a
container whose PID 1 is `sleep infinity` reaps nothing, so `pkill`'d nodes
linger as `<defunct>` zombies with frozen names and CPU figures. Three runs'
leftovers looked precisely like three concurrent nav2 stacks. Use
`docker run --init`, sample with `top -b -n2`, and skip state `Z`.

## What is still untested

- **Everything closed-loop.** The car has still never driven under Nav2.
- `twist_to_ackermann`'s `cmd_vel → drive` conversion, live.
- The recovery subtree's `BackUp` *in context* — it was exercised directly, but
  the BT has never reached it on its own inside a goal window.
- Costmap behaviour against a live LiDAR rather than a replayed one.
- The live run should pass `map_frequency:=30.0` if it wants the map EKF rate
  section 6 of `LOCALIZER_FOLLOWUPS.md` was measured at; the launch default is
  10.0 Hz. Note `60_nav2_test.sh` sets `map_tf_publisher:=ekf`, so that choice
  is already made in that script.
