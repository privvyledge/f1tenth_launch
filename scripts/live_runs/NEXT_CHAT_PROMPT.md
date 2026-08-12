# Prompt for the next chat

Paste this into a fresh session.

---

Read `scripts/live_runs/SYSID_HANDOFF.md` (the **attempt 2** section first),
`scripts/live_runs/SYSID_RESULTS.md`, `scripts/live_runs/BENCH_SWEEP_SHEET.md`
and `scripts/live_runs/LUCIO_REPLY.md` in the f1tenth_launch repo (branch
`perf/config-tuning`). Then pick up the actuator system-identification work.

**Where it stands.** Stages 1 and 2 are done offline. Stage 3 is partially
applied — `steering_angle_to_servo_gain` is now −1.1448 (was −1.4, over-steering
~22 %), the offset stays at 0.56 because it measured 0.5591/0.5610, and
`speed_to_erpm_gain` stays at 3750. Deployed and md5-verified on gosling1; the
install tree is `--symlink-install`, so a file copy into
`/workspaces/f1tenth/src/f1tenth_launch/config/vehicle/vesc.yaml` is the whole
deployment. Re-grep it anyway before trusting a run.

## Job 1 — Stage 0, which is now a short job

The overhead-photo method was validated on 2026-08-07 with one real trial photo.
**Extraction is not the problem** — a ruler edge fits to 0.1° and three tile
grout lines in one frame agreed to 0.03°. Mounting was the problem, and the fix
turned out to make the procedure smaller rather than larger. The handoff's
attempt-2 section has the numbers; the short version:

- The ruler must be **rigid and never touched again**, not *aligned*. A constant
  per-wheel offset cancels in a slope, so the slope, the travel limits, the
  backlash and the left/right asymmetry all survive a mis-seated pointer.
- **There is no rear reference photo.** Tape two light straightedges to the front
  wheels and shoot the sweep. `s₀` comes from the bags (0.5591/0.5610), not the
  bench.
- **Toe** is the one casualty; measure it separately by the differential gap
  method, which needs no camera.
- Do not retry fitting the tire silhouette or the tread grooves. Both were tried
  on the trial photo and both failed badly (5.7–16.4° and 3.5–10.7° spread).

Blocking item, and it is trivial: the 12 in wooden ruler sags around its taped
pivot under its own weight. The operator is bringing a lighter straightedge.

Three things to do before commanding anything:

1. **Ask which axle the rulers are on.** Front and rear are not distinguishable
   from overhead and it decides whether they move.
2. **Update `bench_sweep_card.html`** — it is the operator-facing field card and
   it predates the simplification above (still asks for a rear reference photo,
   still implies alignment matters). Republish it to the same Artifact URL.
3. Add a **per-wheel slope** output to `reduce_bench_sweep.py`; the combined
   bicycle angle does not let each wheel's offset cancel independently.

Then: wheelbase both sides, walk-out (**right first** toward 0.92, which is
empirically survivable; then left toward 0.08, which is unexplored — `servo_min`
has never been reached on this car), staircase, backlash repeats.

The servo-horn re-centring decision is **the operator's**. Present it with the
numbers; do not decide it. Re-centring invalidates every archived bag for
calibration.

Robot state: no ROS processes, `/dev/sensors/vesc` free, all three containers
warm. Use `ROS_DOMAIN_ID=7` — domains 0 and 42 belong to other agents.
`/mnt/f1tenth_ssd/shared_dir/analysis/servo_hold.sh <value>` holds the servo and
`servo_hold.sh stop` releases it; start `vesc_driver_node` first (command line in
the handoff). `pf_sweep_claude_0807` and `mpc_claude_0806` are other agents'
containers; do not stop them, and check running launches from the **host**.

## Job 2 — send the LUCIO reply

`LUCIO_REPLY.md` is **drafted**. It answers the gain (18–23 % wrong, now
−1.1448), reproduces their `k = 1.177` at 1.216–1.230, settles the `gz` sign as
a consumer-side TF issue, gives the ~5.7 % left-biased asymmetry, and shows the
throttle path contributes < 20 ms of their 80 ms. Two numbers are marked pending
on Stage 0: the measured wheelbase and the post-recalibration steering limit.

It was written against a summary of their asks, **not** against the briefing
document — get the briefing, check every §6 ask and §9 question is actually
covered, then fill the two pending numbers if Stage 0 has run and send it either
way if it has not.

## Do not do

Do not write the VESC driver-side deadband brief. That gate (Stage 3b) still has
unverified items — item 3 (the MPC's actual `max_accel` and control period,
versus the second-hand 3.0/0.05) and item 4 (VESC firmware versus driver code)
are both untouched — and the plan says ask the operator first.

## After Stage 0 — the rest of the exercise

- **Stage 4a**, a confirmation drive set. `k → 1.0` is the acceptance test for
  the whole exercise; right now it is 1.216–1.230 with the *old* gain, and
  nothing has been driven since the new one was deployed.
- **Stage 4b**, deliberate excitation (`26_sysid_drive.sh`): an ERPM staircase to
  pin `speed_to_erpm_gain` better than the current ~7 %, and a throttle step to
  settle the transport delay on every bag rather than the two that happened to
  have sharp transients.
- **Stage 5**, the final fit.

## Also open, not part of sysid

The robot container sets `CYCLONEDDS_URI` but not `RMW_IMPLEMENTATION`, so it
runs FastRTPS and the CycloneDDS config is read by nothing (bug-166). This may
mean the VSLAM frame-jitter fix was never active. It belongs to whoever owns the
container image. Raise it; don't fold it into this work.
