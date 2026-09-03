# Hand-back: trajectory_following_ros2 (control)

Written 2026-09-03. Answers §8 of `BRIEF_trajectory_following.md`.

---

## 1. Files edited

| File | What changed |
|---|---|
| `slides/60_planning_nav2.md` | rows **6.2**, **6.5** written. Rows 6.1/6.3/6.4/6.6 untouched. |
| `slides/70_control.md` | rows **7.1, 7.2, 7.2b (new), 7.3, 7.4, 7.7** written. **7.4b, 7.5, 7.6, 7.8 untouched.** |
| `ASSETS.md` | status + notes for the seven control-owned asset rows. Nothing else. |
| `assets/figures/waypoints.png` | new — FIG-WAYPOINTS |
| `assets/figures/mpc_tracking.png` | new — CHART-MPC-TRACKING |
| `assets/figures/control/vid_mpc_{live,rerun}_poster.jpg` | new — poster frames, so the two videos are not black boxes in the PDF export |
| `assets/video/VID-MPC-{RERUN,LIVE}.mp4` | placed locally (gitignored by design; re-copy per `assets/video/README.md`) |

## 2. Build output

```
$ ./build.sh full md
cut 'full':
  00_overview.md             5 / 5
  10_hardware.md             6 / 6
  20_sensing.md              7 / 7
  30_localization.md         8 / 8
  40_mapping.md              4 / 4
  50_perception.md           6 / 6
  60_planning_nav2.md        6 / 6
  70_control.md             10 / 10
  80_vehicle_safety.md       6 / 6
  90_sysid_calibration.md    7 / 7
  95_lessons_next.md         6 / 6
  TOTAL                     71 slides -> out/deck_full.md
done: out/deck_full.md

$ ./build.sh lab
cut 'lab':
  ...
  60_planning_nav2.md        4 / 6
  70_control.md              5 / 10
  ...
  TOTAL                     43 slides -> out/deck_lab.md
done: out/deck_lab.md (+3 rendered file(s) in out/)

$ STRICT=1 ./build.sh full md
  TOTAL                     71 slides -> out/deck_full.md
done: out/deck_full.md
```

No errors, no warnings, and `STRICT=1` passes — every number on a control slide carries a
`src:` note.

```
$ ./check_overflow.mjs out/deck_full.md
OVERFLOW  slide 31  1280x847  Obstacle detection pipeline
checked 71 slides, 1 overflowing
```

**All eight control slides fit.** The one remaining overflow is slide 31, the perception
chat's. Two of mine did overflow on first write (6.2 at 803 px, 7.3 at 891 px) and were
resized down.

## 3. Slides filled, added, renumbered

Filled: **6.2, 6.5, 7.1, 7.2, 7.3, 7.4, 7.7**.
Added: **7.2b, "Obstacles enter the MPC as slacked keep-out rows, not as a cost term"** —
reference-only, immediately after 7.2, at the user's request. The plan has no formulation
slide and the section had no place where the actual optimal-control problem was stated.
Phase G arbitrates the numbering; nothing was renumbered.
Left alone: **7.4b** (per the plan), **7.5, 7.6, 7.8** (other owners).

## 4. Cut tags changed

| Slide | Was | Now | Why |
|---|---|---|---|
| 6.2 | `lab` (scaffold NOTE asked the owner to confirm) | `lab research` | Widened, but **not** into `sponsor`: §2a drops format/config detail from that cut, and the sponsor count already sits over its target. |
| 7.2b | — | none (reference-only) | New slide; formulation depth belongs off the talk deck. |

Confirmed unchanged: 7.1, 7.2, 7.3, 7.4 = `lab sponsor research`; 6.5, 7.4b, 7.7 = reference-only.

## 5. Assets

| ID | Status | Note |
|---|---|---|
| TABLE-CONTROLLERS | **DONE** | inline on 7.2, seven rows, from the config files as read. |
| CHART-MPC-TRACKING | **DONE** | cross-track error and reference-vs-measured speed for both hardware legs. |
| FIG-WAYPOINTS | **DONE** | both map-frame routes, forward vs reverse coloured by the signed `vx`. |
| VID-MPC-RERUN | **DONE** | the 2026-08-10 figure-8 leg, frame-corrected replay. |
| VID-MPC-LIVE | **DONE** | two-camera overhead of that same leg, transcoded to 720p H.264. |
| VID-MPC-OBST-RVIZ | **card** | no MPC obstacle run exists on this car — see §7.2 below. |
| VID-MPC-OBST-LIVE | **card** | needs a driving session with an obstacle feed. |

## 6. Which claims have driven the real car

| Claim | Status |
|---|---|
| MPC waypoint following, acados backend | **Driven the real car.** Both recorded routes end to end, goal latched, 100 % optimal solves at 20.0 Hz — 2026-08-06 (loop and figure-8) and 2026-08-10 (figure-8 under the measured steering cap). |
| MPC, CasADi backend | **Bench only.** Holds 20 Hz on the Orin with 100 % optimal solves; no live drive on record. |
| MPC, do_mpc backend | **Bench only, and infeasible there.** 9.65 Hz against a 20 Hz target on the Orin Nano, 69 % of ticks over budget, never completed the course. |
| MPC go-to-goal | **Simulation only** (2026-08-08). Never on the car. |
| MPC obstacle avoidance | **Simulation and replay only.** Never on the car — no live detection has ever reached this controller. |
| Nav2 `NavigateToPose` go-to-goal | **Driven the real car** (2026-08-27), and 6.5 says so. |
| Pure pursuit | **Simulation and the earlier stack only.** Has not driven the 2026 car. |
| Joystick | **Driven the real car.** |

Said plainly on the slides, including in the `sponsor` cut: two of the MPC's three modes
have driven the car; obstacle avoidance is simulation and replay.

## 7. Pushback and corrections

**7.1 — the brief's repo scan is stale.** It found `~/f1tenth_ws/src/trajectory_following_ros2`
dated Jan 2025. The live repo is the OneDrive checkout on branch `refactor/unify-backends`;
every 2026 result — the hardware legs, the steering recalibration, the breakaway work — is
there, much of it uncommitted. Everything on these slides was read from that tree on 2026-09-03.

**7.2 — `rviz_obstacle_detection.mp4` is a perception asset, not a control one.** It shows
clustered-pointcloud detection with red boxes in `base_link`: no MPC path, no keep-out ring,
no avoidance. It must not be used for VID-MPC-OBST-RVIZ. Offer it to the perception brief.

**7.3 — the plan's preferred figure-8 CSV is not a drive.** The plan says to prefer
`data/fig8_video/solver_155008.csv` "since chart and video then describe the same run".
It is a **breakaway-floor bench run**: `ref_idx` never leaves 5, total travel is 1.12 m, and
the breakaway floor fires on 1974 of its 1993 ticks. The leg the video actually shows is
`solver_104311.csv`, which lives on the car and not in the repo. The chart is therefore
built from the two 2026-08-06 hardware legs, whose logs *are* in the repo, and the slide
says which run each number came from.

**7.4 — the previously circulated cross-track figures were measured over the wrong tick set,
and the plan repeats one of them.** The plan quotes "CTE p95 ≈ 4.6 cm" for the loop leg.
That is the p95 over *all 14 322 logged ticks*, ~90 % of which are parked at the route start
with the deadman released — it flatters the controller. Filtering instead to ticks where the
reference index has advanced gives:

| Leg | ticks | CTE p50 | CTE p95 | max |
|---|---|---|---|---|
| loop, 2026-08-06 | 1039 | 3.0 cm | **13.1 cm** | 26.5 cm |
| figure-8, 2026-08-06 | 1176 | 5.0 cm | **18.8 cm** | 24.4 cm |
| figure-8, 2026-08-10 (±18° cap) | 270 samples | 4.4 cm | 12.7 cm | 17.0 cm |

The slides carry the filtered numbers, and the filter is named in the speaker notes. The
2026-08-10 row is recovered from `figure8_leg1_mapfixed.rrd`; its 12.7 cm p95 corroborates
the 14.4 cm recorded live for that leg from an independent script.

**7.5 — `k ≈ 0.96` is not an MPC tracking metric.** The plan lists it for 7.3. Read at source
(`scripts/live_runs/SYSID_RESULTS.md`), it is the **steering-calibration** acceptance band
agreed with the downstream group on 2026-08-08 — accepted range 0.95–1.02, and the note is
explicit that chasing the last 4 % is wrong. It belongs on the sysid slides (9.x), not on a
tracking-results slide, so it is not quoted on 7.3; the reasoning is in a comment there.

**7.6 — the Nav2 rows of TABLE-CONTROLLERS are filled, not left as holes.** The plan said to
leave them to the f1tenth_launch chat. On a table that is core to all three cuts, two blank
rows read worse than sourced ones, so both are filled from `config/nav2_params.yaml` as read
(RPP 10 Hz, lookahead 0.6 m clamped 0.3–3.0, 0.5 m/s, 3.0 rad/s²; MPPI 56 × 0.05 s, 2000
rollouts) with an ownership comment on the slide. **Please confirm them.** The one thing this
repo genuinely cannot supply is an MPPI *result*: it is configured but is not the selected
plugin, and no run of it exists anywhere here. That gap is slide **7.4b**, which is left as
the untouched skeleton per the plan.

**7.7 — a scaffold/plan ownership disagreement, left uncorrected.** `slides/70_control.md`
marks row **7.8** as `owner: control chat`; the brief §1 assigns it to the f1tenth_launch
chat. I followed the brief and did not touch it. Phase G should settle which is right —
note that `PHOTO-DUALSENSE` is held at `EXISTS` rather than `DONE` purely because 7.8 still
cards it.

## 8. Ranked list — what this area can actually show today

1. **Live MPC figure-8 on the car**, two-camera overhead, 2026-08-10 — completed route, goal latched. *(VID-MPC-LIVE, in place)*
2. **The matching rerun replay of that same leg** — predicted horizon, reference window, live cross-track trace. *(VID-MPC-RERUN, in place)*
3. **First-person 4K of the same session** — the best "real car, real lab" shot. Not placed: 185 MB, needs a downscale first.
4. **Live MPC loop route**, 2026-08-08 — supporting footage, same rig.
5. **Nav2 go-to-goal on the car**, 2026-08-06 — belongs to 6.4/6.5, not to 7.x.
6. **The tracking-error chart** from the two live legs. *(CHART-MPC-TRACKING, in place)*
7. **Nothing for MPC obstacle avoidance on hardware.** Simulation stills only, and they must be labelled as simulation.

## 9. S99 research question (for the integration chat — 11.3 is theirs, not edited)

> The vehicle cannot move below 0.20–0.26 m/s, while an MPC that publishes its one-step
> state `v + a·dt` can command at most `max_accel·dt` = 0.15 m/s from rest — **the actuator's
> floor sits above the controller's ceiling.** How should terminal and standstill behaviour be
> formulated when the plant has a hard breakaway speed, and what does ~100 ms of actuation
> lag then force on the horizon and the rate?

Evidence in hand: the speed-staircase measurements of 2026-08-07 (breakaway 0.18 m/s forward
on stands, nothing turns below 0.06); the ground A/B of 2026-08-08 (pass-through of
un-floored creep commands 21.6 % → 0 % over 16 838 qualifying ticks); and the Nav2 goal
approach on 6.4, which stalled at 0.269 m/s and stopped 0.379 m short.

The experiment is runnable as it stands: a commanded-speed staircase scored against tape,
on the ground rather than on stands, then the same terminal approach re-driven with and
without a floor.
