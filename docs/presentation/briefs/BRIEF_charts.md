# Brief: offline charts from existing bags (Phase F1)

**Written 2026-09-03 by the integration chat.** Read this first, then
`PRESENTATION_PLAN.md` §1 (conventions), §3 (the slide rows your figures land on)
and §9 (arbitration log — it overrides §3).

Your working directory is `docs/presentation/` in `f1tenth_launch`, branch
`perf/config-tuning`.

---

## 0. Why this brief exists

The plan has one Phase F, "robot capture session (you + one chat on gosling1)",
covering all 29 `ROBOT` assets. That bundles two jobs that need completely
different things, and only one of them needs the lab:

- **F1 — this brief.** Seven assets that are *processing of bags already
  recorded*. No car, no battery, no driving, no operator. Every input already
  exists on the gosling1 SSD, verified present 2026-09-03.
- **F2 — still unscheduled.** Eleven video clips that need the car running and
  the operator driving. Two of those (`VID-MPC-OBST-RVIZ`, `VID-MPC-OBST-LIVE`)
  can never be captured: MPC obstacle avoidance has never run on this car, and
  they stay cards permanently (plan §9 G4).

You are F1. **Do not start anything that needs the car to move.**

---

## 1. What you own

| Asset | Lands on | Input, verified present 2026-09-03 |
|---|---|---|
| `CHART-RATES` | **2.2 and 2.7** (carded twice) | `bag_stats.py` on `armA_loop2` |
| `CHART-CLOSURE` | **3.x** (`30_localization.md:117`) | `plot_localization.py` on `armA_loop2` + the 20260805 grid |
| `CHART-STEER` | **9.3** (`90_sysid_calibration.md:87`) | `fit_actuators.py` on `armA_loop`, `armA_loop2`, `armA_steer_sweep` |
| `CHART-SPEED` | **nowhere yet** — see §5 | `fit_actuators.py` on `armA_straight_5m` |
| `CHART-NAV2-APPROACH` | **6.4** (`60_planning_nav2.md:138`) | the `nav2_drive4` bag |
| `FIG-MAP-COMPARE` | **4.x** (`40_mapping.md:101`) | needs `40_build_map_offline.sh --mode both` |
| `TABLE-PACKAGES` | **nowhere yet** — see §5 | `ros2 pkg list` etc. inside the container |

**`CHART-LAG` is not yours.** It is carded on 7.6 (vesc chat) and 9.5 (mine), and
it needs a step-response measurement that does not exist in any bag here. Leave it.

Plus one piece of tooling, which is the reason this is a chat and not a script run:

**`scripts/analysis/fit_actuators.py` has no plotting code at all** — zero calls to
anything matplotlib. `ASSETS.md` guessed it "may need a `--save-fig`"; it needs the
whole plotting path. **Write a separate script that consumes its output rather than
adding matplotlib to the fitting script.** `fit_actuators.py` is the sysid tool the
calibration record depends on and it should stay one; a plotting module is also the
kind of thing that later wants to serve `bench_servo_sweep.py` too. If that means
teaching `fit_actuators.py` to emit machine-readable results it currently only
prints, do that — a `--json` output is a smaller, safer change than a plotting
branch inside it.

---

## 2. Where everything is

gosling1 was reachable by plain `ssh gosling1` on 2026-09-03 14:03 EDT, clock
correct, root fs 915 G with 746 G free.

```
/mnt/f1tenth_ssd/shared_dir/bags/20260901/armA_loop2_214423        47 M
/mnt/f1tenth_ssd/shared_dir/bags/20260901/armA_steer_sweep_214921  70 M
/mnt/f1tenth_ssd/shared_dir/bags/20260901/armA_straight_5m_215624  28 M
/mnt/f1tenth_ssd/shared_dir/bags/20260901/armA_loop_212826         27 G   <- see §3
/mnt/f1tenth_ssd/shared_dir/claude_bringup_0827/nav2_drive4
/mnt/f1tenth_ssd/shared_dir/repo_sync/scripts/analysis/            bag_stats.py, plot_localization.py
```

`fit_actuators.py` is in **this** repo (`scripts/analysis/`) and is *not* in the
robot's `repo_sync` copy — stage it if you run it there. Note `scripts/` is not
installed by colcon: it runs from the source tree, so check paths against `$S`,
not the install space, or you will chase spurious MISSINGs.

---

## 3. Traps, all of them already paid for once

**`armA_loop` is the 27 GB image-starved bag (bug-272).** Its camera streams were
recorded and starved librealsense's USB thread; Isaac VSLAM got 107 messages in
214 s while reading a healthy 30 Hz throughout. **Prefer `armA_loop2`**, which is
the `sysid` topic set. If you must use `armA_loop`, do not draw any conclusion from
its VSLAM stream, and verify by message **count** in `ros2 bag info` — never by rate.

**`fit_actuators.py`'s `CFG_STEER_GAIN` is stale at −1.4.** The configured value is
**−1.1448**. Its printed "k vs configured" is therefore against the wrong baseline;
recompute before quoting. Fixing that constant is in scope and welcome — but say so
in the hand-back, because published numbers were computed against it.

**Never calibrate speed against rf2o alone.** It under-reports distance by 5–8 %
(read 5.163 m over a 5.50 m tape run), which inflates its fitted ERPM-per-m/s to
3973 against VSLAM's 3737. The tape measure settles it and the configured 3750
stands. Slide 9.4 already tells this story with both references and the tape — any
CHART-SPEED must agree with that slide, not re-litigate it.

**Do not run bringup, and do not start the car.** The operator launches the robot;
an agent SSH path has killed the RealSense before. Reading bags and running
analysis scripts is fine.

**Do not delete anything on the SSD.** Other sessions' logs live there, and several
bags are the only surviving evidence for closed bugs.

---

## 4. Conventions the build enforces (plan §1)

Read them in the plan, but the three that will bounce your work:

1. **Every number on a slide needs `src: FILE; measured DATE`** in an HTML comment
   on that slide. `STRICT=1 ./build.sh full md` fails otherwise — run it before
   handing back.
2. **A placeholder card's ID must be in `ASSETS.md`, and an asset marked `DONE`
   there must be embedded, not carded.** So flipping a row to `DONE` and forgetting
   to replace the card is a build error, by design.
3. **No slide body may name an agent file or a bug ID** (speaker notes may).

And one that is not enforced but is the whole point: **no stock, staged or
synthetic media, and no number without a source.** If a chart cannot be built from
data that exists, it stays a card and you say why.

Also: `./check_overflow.mjs out/deck_full.md` must not gain a new overflow. It
currently reports exactly one — slide 31, the perception chat's. A figure that
pushes a slide over is your figure to resize.

---

## 5. Two rows to decide, not just produce

`CHART-SPEED` and `TABLE-PACKAGES` are registered in `ASSETS.md` but **carded on no
slide**, because the slides that would have used them were written with tables
instead:

- **9.4** already carries the three-reference speed table *and* the ERPM deadband
  table, sourced and complete. A chart may add nothing.
- **0.4** already lists the repos from `f1tenth.repos`. A `ros2 pkg list` dump is a
  different, less interesting thing.

So for each: either produce it and place it (which means editing a slide that is
already written — say so in the hand-back), or **retire the register row** with a
one-line reason. Retiring is a perfectly good outcome; a register full of assets
nobody wants is worse than a short one.

---

## 6. Hand-back

Write `briefs/HANDBACK_charts.md` and say:

- files edited, and the build output of `STRICT=1 ./build.sh full md`,
  `./build.sh lab`, and `./check_overflow.mjs out/deck_full.md`;
- which assets are now `DONE`, which stayed cards **and why**;
- what you changed in `scripts/analysis/` and whether any previously published
  number moves as a result — the `CFG_STEER_GAIN` question especially;
- **which claims have driven the real car** versus replay, bench or simulation —
  in those words;
- anything in the plan or in `ASSETS.md` you believe is wrong. Push back here
  rather than working around it silently. The last hand-back found three plan
  errors and all three were upheld; that is the expected yield, not an exception.
