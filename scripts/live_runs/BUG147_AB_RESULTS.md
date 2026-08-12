# bug-147 — Isaac VSLAM SIGABRT vs CycloneDDS config: A/B result

**Run 2026-08-07, 15:06–16:44 EDT on gosling1.** 10 launches of
`71_mpc_stack.sh`, arms interleaved A,B,A,B,… so drift over the ~1 h (thermal,
USB, battery) loads both arms equally. Raw data:
`/mnt/shared_dir/bug147_ab_20260807_150630/` (trials 1–4) and
`bug147_ab_20260807_161511/` (trials 1–6). Harness: `/mnt/shared_dir/bug147_ab.sh`.

- **arm A** — `cyclonedds_config_static.xml` (WiFi + lo, static peers)
- **arm B** — `cyclonedds_offline_lo.xml` (lo only, single localhost peer)

Per trial: clean slate → launch → wait for cuVSLAM init → observe 180 s → count
deaths **before** teardown → tear down. Deaths counted from the launch log;
liveness from `pgrep -f __node:=visual_slam_container`; rate from `ros2 topic hz`.

## Result

| | arm A (static) | arm B (lo-only) |
|---|---|---|
| launches | 5 | 5 |
| cuVSLAM reached init | 5/5, all at 30 s | 5/5, all at 30 s |
| `visual_slam_container` aborts | **0** | **0** |
| VSLAM process alive at +180 s | 5/5 | 5/5 |
| VSLAM odometry rate at +180 s | 29.95–29.99 Hz | 29.96–30.03 Hz |
| other container deaths (pre-teardown) | 0 | 0 |

## What this does and does not support

**It does not rule out DDS config.** With zero events in both arms there is
nothing to compare — Fisher's exact on 0/5 vs 0/5 gives p = 1.0. The experiment
produced no discriminating information in either direction. "The rate is
unchanged, so DDS is ruled out and the suspect is cuVSLAM" (the disposition
suggested in the handoff) is **not** a conclusion this data licenses.

**The 1-in-3 baseline was never a rate.** It was 1 abort in 3 launches — a
Clopper-Pearson 95% CI of roughly [0.8 %, 90 %]. Today's 0-in-10 is unlikely
under a true 1/3 (P = (2/3)¹⁰ = 1.7 %), so the real rate is probably much lower.
Pooling everything known: **1 abort in 13 launches ≈ 7.7 %, 95% CI [0.2 %, 36 %]**.

**At that rate this design has no power.** 5 launches per arm at ~8 % expects
0.4 events per arm. Seeing zero in both is the *expected* outcome whether or not
DDS config matters. Detecting a difference by counting would need roughly 20
launches per arm just to reliably observe one event in the worse arm, and
~60+ per arm for a comparison — 6–10 hours of robot time.

**A confound, stated plainly:** `192.168.2.193` was commented out of
`cyclonedds_config_static.xml` *before* trial 1, so today's arm A is not the
configuration under which the original abort was seen. A second difference: no
external MPC node was subscribed during any of these 10 launches, whereas the
abort session had one — one fewer DDS participant on the VSLAM odometry topic.

## The founding observation is real — verified, not assumed

Worth recording because it nearly went the other way. `mpc_stack_20260807_112512.log`
contains six `process has died` lines, which looks like an ordinary teardown.
Locating each against the surrounding timestamps separates them cleanly:

| event | position | time |
|---|---|---|
| cuVSLAM init | — | t+24.2 s |
| **`visual_slam_container` exit −6** | **24.7 % through the log** | **t+49.2 s** |
| `ackermann_to_twist` exit 1 | 97.4 % | t+236.8 s |
| `ros2-5` exit 15 | 97.6 % | t+236.8 s |
| `localization_container` exit −11 | 97.8 % | t+236.8 s |
| `joy_teleop` exit 1 | 99.2 % | t+237.0 s |
| `sensing_container` exit −6 | 99.4 % | t+237.0 s |

VSLAM died **alone, 25 s after init**, with the rest of the stack running for
another three minutes. The other five are the teardown signature that appears in
every session log. So bug-147 is a genuine spontaneous abort, and the 180 s
observation window used here covers its latency with a wide margin.

**Corollary trap:** component containers exiting **−6 / −11 during teardown is
normal here** and appears in every log. An exit code alone is not evidence of a
crash; check whether the death is solitary and where it sits in the run.

## Recommendation: stop counting, start catching

Chasing a ~8 % event by launch count is a poor trade against robot time. One
captured abort yields a cause; a hundred launches yield a rate. Next step:

- enable core dumps for the container process (`ulimit -c unlimited`, a writable
  `/proc/sys/kernel/core_pattern` target on the SSD) and get a backtrace from
  the SIGABRT — cuVSLAM aborts usually carry a message on stderr immediately
  before, which the peer-spam volume may currently be burying;
- run the stack under normal working sessions with the death-grep in place
  rather than dedicating robot time to it, since the event shows up in ordinary
  use anyway.

## Two harness defects found and fixed during the run

- **bug-148**: the first 4 trials recorded `vslam_hz=0` for a demonstrably live
  node. The harness called `ros2 topic hz` from a `docker exec bash -c` shell
  with no ROS on `PATH`; the command failed and the empty result defaulted to 0.
  Fixed by sourcing ROS and aborting loudly if `ros2` is still missing;
  unmeasured now records `NA`, since a missing measurement and a measured zero
  mean opposite things. Liveness/death columns were never affected.
- **The handoff's grep pattern never matches.** The real line is
  `[component_container_mt-29]: process has died [pid …, exit code -6, cmd '… __node:=visual_slam_container …']`
  — the node name comes *after* the phrase, so
  `visual_slam_container.*process has died` always returns 0. Use
  `process has died.*visual_slam_container`.
