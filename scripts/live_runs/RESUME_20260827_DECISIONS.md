# Resume — four open items, and three of them are decisions rather than measurements

**Repo:** `f1tenth_launch` · branch `perf/config-tuning`
**gosling1:** `192.168.2.195`, direct `ssh gosling1@192.168.2.195` (no jump host)

Written 2026-08-26 ~18:15 EDT. **Replaces `RESUME_20260827_STATIONARY.md`**, whose three items are
all done — recover it from git history for the original wording.

---

## 0 · How the operator wants this session run

**Interview style.** Do not pick these for them and start implementing. For each item below: ask,
lay out the options with their consequences, give a recommendation and say why, then wait. Three of
the four are judgement calls where the measurement work is already finished and the remaining
question is what the project wants — that is the operator's call, not the agent's.

The one thing worth doing *before* asking anything: **verify the named blockers**. This handoff
family has a track record of carrying stale ones — see §5.

---

## 1 · What closed 2026-08-26 evening. Do not re-open

Two commits, **local, not pushed**: `e1ae908`, `aedbb42`.

**bug-248 — second verification, PASS.** `run18_constdt`, largest attitude step over 217 s
**0.420 deg** (run17: 0.187; pre-fix 63–167). The stronger point is that this is not a
non-recurrence: **the driver defect is still present** — a **+865.55 s** header discontinuity at
filtered sample 8 — and the attitude moved **0.0121 deg** through the first 5 s, the window the
pre-fix jump landed in on 4 of 4 launches. madgwick absorbs the provocation. `constant_dt`'s
fingerprint is readable in the data: the filtered stream's header dt is a metronomic
0.005000/0.005001 while its raw input drifts at 0.005528. Tool: `early_stamps.py`.

**bug-244 — a fourth clean cold launch** (`odometry/local` 1.4e-05 m/s at t+30 s).

**`imu_bias_remover` — verified offline.** `BIAS_REMOVER_OFFLINE_20260826.md` has everything. The
estimate reproduces the measured −0.00214 rad/s to **3.1e-05**; the moving-branch subtraction is
**bit-exact**; the staleness hazard is **confirmed** (bug-251). `remove_imu_bias` is unchanged at
`'False'`.

**Doc consolidation — committed** (`e1ae908`), with `LUCIO_MAP_HEADING_ANSWER.md` added to it
because it was untracked while four files cited it.

---

## 2 · Decision 1 — the `imu_bias_remover` watchdog (bug-251). This blocks the whole track

**The finding, so the decision is made against the real thing.** The node's stationary test is
`twist_is_zero_ || odom_is_zero_` and **neither flag has a staleness timeout** — read in the
shipping 0.5.2 source, not inferred. Measured: with the velocity source stopped while the flag read
`true`, the node published `angular_velocity.z == 0.0` for **3996 consecutive samples** while the
real gyro underneath read up to 0.037 rad/s. It never left the zeroing branch and had no way to.

On this car that path is realistic, not hypothetical: the VESC driver aborts on serial EIO and goes
dead-stick while every command topic still looks healthy. `vesc_odom` dying while parked would pin
the corrected gyro at zero **from then on, including once the car moves**, feeding a confidently
wrong yaw rate into `ekf_odom`.

Everything *else* about the node passed. This is the only thing in the way.

### Options to put to the operator

| # | option | cost | what it leaves |
|---|---|---|---|
| A | **Patch `imu_processors` to add a `stationary_timeout`** — if the velocity source has been silent longer than N, drop out of the zeroing branch. The image already builds 0.5.2 from source, so there is a build path, and it is a candidate upstream PR. | small, C++, one param | keeps the EMA estimator and its stationary zeroing benefit; removes the hazard at the source |
| B | **External watchdog** on `vehicle/vesc_odom`; stop trusting `imu_biased` when it goes stale. | a new node — and per CLAUDE.md rule 6 that probably wants its own repo, not inlining here | keeps upstream unforked; adds a moving part and a second failure mode |
| C | **Gate on liveness at the EKF** instead — let `ekf_odom` stop fusing `imu1` when `vesc_odom` is stale. | config/launch, no new node | does not fix the wrong signal, only who listens to it; other consumers (Isaac VSLAM) still get it |
| D | **Drop the zero-velocity coupling** and write the ~40-line static offset subtractor the spec anticipates (§4), as its own small repo. | small, but a new repo to own | simplest possible failure mode; loses the online estimate and the stationary zeroing |
| E | **Do nothing** — leave `remove_imu_bias: 'False'`. | free | the −0.00214 rad/s bias stays uncorrected, outvoted by VSLAM and rf2o. That is today's working configuration, and it is only fragile when VSLAM aborts (~1 launch in 3) |

**Recommendation: A**, with D as the fallback if upstream's shape resists. The spec already
anticipated D as the contingency "if the zero-velocity coupling turns out to be unwanted" — but the
coupling is not unwanted, it is *untimed*, and a timeout is a smaller change than replacing the
estimator. Note E is a legitimate answer: nothing is currently broken by the bias, and the cost of
being wrong here is a silently wrong yaw rate during autonomous driving.

**Whatever is chosen, `remove_imu_bias` stays `'False'` until it lands.**

---

## 3 · Decision 2 — `max_steering` 0.34 → 0.314, and an untrue delivered claim

**This does NOT need LUCIO.** There are three different steering limits in this repo and they are
routinely conflated:

| what | value | who reads it |
|---|---|---|
| `max_steering` | **0.34** at all four entry points | `joystick.launch.py` only — scales the joy_teleop steering axis. **Manual driving.** |
| `vesc_max_steering_angle` | `0.0` (off) | the VESC driver nodes |
| `max_steering_angle` | **0.25**, hardcoded `vehicle.launch.py:320` | `twist_to_ackermann` — the **Nav2** path |

LUCIO's ego-MPC publishes `drive` directly and goes through none of them; its limit is its own
`delta_bound`, ±18 deg = ±0.314 rad, landed in `LUSCIO_ROS` 2026-08-09. So the coupling is that both
sides must agree with the same **measured mechanical constant**, not that they share this argument.
Changing `max_steering` touches joystick teleop scaling and nothing else.

Measured travel: **+0.419 rad (24.0°) left / −0.314 rad (18.0°) right**. At 0.34 the right side
clips (servo 0.9492 vs the 0.92 bound) and the VESC logs "above maximum limit"; at 0.314 neither
side clips.

**A related stale comment, found while verifying the above (2026-08-26).** The justification block
above `vehicle.launch.py:320` still argues from the **old gain −1.4**: "servo = −1.4*0.4 + 0.56 =
0.0, below the 0.08 minimum … 0.25 < 0.257 rad keeps both locks inside the servo range". With the
calibrated **−1.1448**, 0.4 rad gives servo 0.1022, which is *above* 0.08, and the real no-clip
range is +0.419 / −0.314. So **0.25 is now more conservative than it needs to be on the Nav2 path**,
and its stated reason is arithmetic that no longer holds. The value is still safe — this is a
correctness-of-record issue, and a candidate to raise to 0.314 alongside §3 so all three limits
agree with the measured mechanics. Worth asking the operator about in the same breath.

**And `LUCIO_REPLY.md` told them on 2026-08-08 that this was already applied.** It was not — all
four entry points still read `0.34` and the string `0.314` appears nowhere in `launch/` or
`config/`. Either it never landed or a staged tarball reverted it (that has happened before).

### Options

- **A — apply 0.314 at all four entry points, and correct `LUCIO_REPLY.md`.** Recommended. It is
  local, it removes right-turn clipping, and it moves us *into* agreement with the box LUCIO
  already has. Their constraint does not change, so this is our bookkeeping error, not a wrong
  number on their side.
- **B — apply it and also send LUCIO a correction.** Only if the operator wants the record straight
  with them; nothing on their side is currently wrong.
- **C — leave 0.34, correct the record only.** Defensible if the operator likes the extra left
  authority for manual driving and accepts right clipping. Say so explicitly in `LUCIO_REPLY.md`
  rather than leaving the claim standing.

**What genuinely obliges telling LUCIO is re-measuring** the limit — recentring the servo horn or
re-running the gain calibration — because that would make their box constraint stale. Note the
recentring decision was already made: **not before the Stage 4a re-record**, since it voids the
archived 2026-08-05 bags as a calibration baseline and destroys the agreed `k ≈ 0.96` acceptance
test.

---

## 4 · Decision 3 — the live parked wiring test, and when to do it

Spec §6 step 3: bring the stack up with the bias remover actually in the chain and confirm
`odometry/local` does not regress from its **+0.04 / +0.01 / +0.17 deg/min** band
(`yaw_drift.py 60`, parked, twice).

It is stationary and safe, but it needs a **source edit** to flip `remove_imu_bias`, which is why it
was not done unilaterally.

- **A — do it now** with a temporary, uncommitted flip. De-risks the eventual change.
- **B — do it once, after the §2 watchdog lands**, so the thing tested is the final wiring.
  **Recommended** — the watchdog will change the wiring, so A means testing it twice.

**Remember what this test can and cannot say.** A parked yaw-drift figure reads ~0 *whether or not
the correction works*, because the zeroing path is active and the subtraction path is not. This
test checks **wiring and non-regression** — topics, QoS, composition — nothing more. The
discriminating evidence is already in hand (§1). **Stop and report if `odometry/local` regresses.**

---

## 5 · Verify these before planning around them

This handoff family has repeatedly carried blockers that were stale or wrong, and each cost a
session. Two examples from 2026-08-26 alone: *"`imu_processors` is at 0.4.1, re-derive every claim"*
(the robot never used apt — the image carries **0.5.2 built from source**, so apt would be a
**downgrade**) and *"the driving half of the bias check is owed to a drive session"* (it was
closable parked with a synthetic velocity source). Earlier: *"the Jetson has no internet"* and
*"root fs at 96 %"*, both retired by the 2026-08-24 reflash.

Spend the first few minutes checking each named blocker against the machine — `apt-cache policy`,
`docker run --rm … ls`, `df -h`, read the shipping source — then correct the doc in place, in the
same commit as the work.

---

## 6 · The moving-odometry check — still needs battery and a driven leg

Not in scope for a stationary session. `scripts/live_runs/odom_moving_check.py` is the tool,
procedure in `DEMO_RUNBOOK_20260810.md` §3. Get all of this onto one drive:

- **`odometry/local` yaw drift while moving** — no parked test reaches it. Parked is +0.04 deg/min.
- **The wheelbase change 0.25 → 0.256 m** (2026-08-07), still unverified on hardware. It biased
  `vesc_odom`'s kinematic yaw rate ~2.4 % against the frames it fuses into.
- **Does the bias correction improve real driven heading** — the one thing a synthetic source
  cannot answer. (The staleness hazard is already closed offline; that half is *not* owed.)
- **Drive into the steering limit both ways** — the only way `servo_min` is ever exercised. Nothing
  is known empirically about the left bound; all observed saturation was right-turn.
- **Coordinate with Stage 4a sysid.** Do not let a fusion change ride along with a Stage 4a bag —
  `k → 1.0` must be measured against a fusion configuration that is not simultaneously changing.
  Sequence: Stage 4a first on today's fusion, then the bias work.

**Record `camera/imu` AND `vehicle/vesc_odom` in the same bag.** A sweep of all 66 bags on gosling1
found only one (April 2025) carrying both: each campaign recorded for its own investigation, so
run14–18 have 43k IMU messages and zero `vesc_odom`, while the 08-25 and 08-06 sets have the
reverse. That is why the offline test needed a synthetic source.

---

## 7 · Getting the robot back

The container is **not** persistent, and neither is anything in `/workspaces` — that is a container
layer. **The container dies with the terminal that started it** (`jetson-containers run … -it --rm`),
so an SSH/MobaXterm session ending — including a laptop dying — deletes it. This happened three
times on 2026-08-26. Budget ~8.5 min bring-up plus a re-stage each time, or start it detached with
the FIFO-pty recipe.

**The operator starts it**, from the Jetson desktop session, not an agent SSH shell:

```bash
bash ~/bolus_ws/f1tenth_launch.sh
```

Then re-stage, from the host:

```bash
/mnt/f1tenth_ssd/shared_dir/stage_0826.sh <container-name>
```

- Tarball **`f1tenth_stage_20260826b.tgz`**, md5 `84e4258ed4a4fa2a7bc9c837ff8f96f5`. It carries
  **both** fixes. The `0825` tarball predates bug-244's and the first `20260826.tgz` predates
  bug-248's — re-staging either silently reverts one.
- The script verifies six things in the **installed** tree; check the last two on the **live node**:
  ```bash
  ros2 param get /ekf_odom_node imu1_config          # entries 12,13,14 must be False
  ros2 param get /realsense_imu_filter constant_dt    # must read: Double value is: 0.005
  ```

`/mnt/shared_dir` inside the container is `/mnt/f1tenth_ssd/shared_dir` on the host.
**`/tmp` inside the container is not the host's `/tmp`** — stage scripts through the shared mount.

---

## 8 · Environment notes still current

- **X11 works through the operator's SSH forwarding**, not `:0`. On 2026-08-26 the container had
  `DISPLAY=localhost:10.0` (MobaXterm) and `xdpyinfo` succeeded there while `:0` refused. Probe with
  `xdpyinfo -display <d>` before assuming a display problem.
- **DualSense** `10:18:49:9D:72:FC` is paired and trusted but normally left **disconnected** (a PS
  press reconnects). Connected, it publishes the heartbeat, `command_gate` opens, `vesc_odom` starts
  at ~50 Hz — **and the car can drive**. Mind the desk.
- **`command_gate_require_heartbeat:=False` does NOT hold the gate shut** — it collapses the logic
  to always-open. To keep it closed, leave the default `True` and disconnect the joystick.
- Bags: `claude_bringup_0826/run1{4,5,6}` (pre-bug-248-fix, raw **and** filtered camera IMU),
  `run17_constdt` and `run18_constdt` (post-fix). `claude_bringup_0825/ekfdiag_run12_control` is the
  bug-244 onset but has **no** raw `/camera/imu`. Offline bias test data:
  `biastest_0826/bias_test.csv` (plus `bias_test_run1_FLAWED_no_phaseA.csv`, kept as a worked
  example of the trap in §9).
- **The 2026-08-05 source bags are not on gosling1** after the reflash — only the derived `pose_map`
  deliverables. If `figure8_172338` is needed, it is on velox1.
- `/mnt/data` inside the container is a container layer despite looking persistent.
- Docker's default **bridge** network is broken on this kernel — `docker run` needs `--network host`,
  which is what jetson-containers uses. A bare `docker run` fails with an iptables `raw` table error.
- `jetson-containers` bind-mounts `/tmp/argus_socket`, a *socket* on R36.4.3 but a *directory* in the
  image. `run.sh` is patched; **a fresh checkout reintroduces this** (bug-242).
- CycloneDDS spams `ddsi_udp_conn_write … retcode -3` at absent peers. Harmless, but it will drown a
  `ros2 param get` — pipe through `grep -v ddsi`.

---

## 9 · Traps that have each cost a run

- **`ros2 launch` does not reliably die on a SIGINT to its process group here** (bug-249, hit again
  on run18). The parent exits and the script reports "done" while `component_container_isolated`
  keeps publishing — so back-to-back runs are **not** independent trials. After every teardown,
  list, kill, and confirm empty before the next launch.
- **`pkill -f <pattern>` inside `docker exec bash -lc "…"` kills its own shell**, because the
  shell's command line contains the pattern — and a following `pgrep -f` matches it too. List with
  `ps -eo pid,args --no-headers | grep "[c]omponent_container"` and kill by PID.
- **Never put `set -u` in a script that sources `/opt/ros/humble/setup.bash`** (bug-250) — it reads
  an unbound `AMENT_TRACE_SETUP_FILES`, exits before launching anything, and **exits 0**, so it
  looks like a completed run.
- **Do not key a phased test clock off node start when the data comes from a bag.** The first bias
  test lost its whole stationary window to the bag's ~23 s lead-in (the recorder starts before the
  launch; the camera needs ~9.5 s), so the moving phase subtracted a bias of exactly `0.0` and
  passed degenerately. Key off the **first received message**. The tell is a green moving-phase
  check with `bias_z == 0.0`.
- **When scoring against phase labels, print what the guard band excluded.** Scored raw, two checks
  failed — 3 samples each, within 10 ms of a boundary, one tick of a 50 Hz flag against a 200 Hz
  stream. A guard band that hides its exclusions is indistinguishable from fudging a failure.

---

## 10 · Housekeeping

- **Two commits are local and unpushed** (`e1ae908`, `aedbb42`). Ask before pushing.
- `docs/build_repo_requirements.md` is **untracked** and was edited today (its `apt install
  ros-humble-imu-pipeline` ask is a downgrade and now says so). It is referenced by
  `docs/imu_bias_removal_spec.md` §8. Decide whether it should be tracked — CLAUDE.md says
  build-repo material is not maintained here, which may be why it is not.
