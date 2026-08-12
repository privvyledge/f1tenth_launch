# Notice to the LUCIO side — the raslab map-frame start heading moved 4.7°

**gosling1 side, 2026-08-11.** Not a reply to a round — this is an outbound notification about a
calibration constant you consume, sent because the number changed and because one value you may be
seeding from does not fit the map at all.

Short version: **the parking-spot heading in the map frame is −84.5°, not −79.8°. We changed it.
And figure-8 waypoint 0 (−92.08°) is the worst of every candidate we scored — anything that seeds
its start pose from waypoint 0 begins ~7.7° wrong, in a way that waypoint 0's own self-check cannot
detect.** Nothing about the vehicle changed; the frame did.

---

## 1. What changed

`config/localization/localizer_amcl.yaml`, `initial_pose.yaw`: **−1.3928 → −1.4748 rad**
(−79.80° → −84.4998°). Position (x, y) = (0.445, −0.575) is **unchanged** — see §5.

This is the AMCL seed for the car's physical parking spot on the `20260805/rtabmap_2d_final` map.
It is the pose the global localizer starts from on every live bringup, so it sets where the map
frame believes the car is before any motion.

## 2. Why the old value was wrong

−79.80° was inherited from the RTABMap ground-truth start pose of the 2026-08-05 bags
`(+0.445, −0.575, −79.82°)`, read out of the database's optimized poses. It was never checked
against the map geometry — it was an estimator output being used as truth.

Three values had claimed this same physical spot for weeks:

| claim | source |
|---|---|
| −79.80° | the AMCL seed (from the RTABMap ground truth above) |
| −86.50° | live AMCL settle, 2026-08-06 |
| −92.08° | figure-8 route CSV, waypoint 0 |

**All three are estimator outputs**, so they inherit whatever bias they share and cannot arbitrate
each other. Every previous attempt to settle this compared filters against filters. There was also
a standing hypothesis that a rectangular lab is symmetric to a planar LiDAR, which discouraged
treating scan matching as decisive.

## 3. How the new number was measured — no filter in the loop

`scripts/live_runs/heading_from_scan.py` takes the **raw** `LaserScan` and grid-searches
(x, y, yaw) against the occupancy grid's likelihood field, scoring how well the beam endpoints
explain the walls. No AMCL, no EKF, no VSLAM, no odometry. It is a geometric measurement of where
the car must be for the scan to fit the map.

Five samples, two separate cold launches, with the car rolled off the spot and back on between
recordings B and C:

| sample | x | y | yaw | score |
|---|---|---|---|---|
| 2026-08-10 | +0.385 | −0.515 | **−84.50°** | 0.8566 |
| 2026-08-11 A — as parked | +0.415 | −0.515 | **−85.00°** | 0.9162 |
| 2026-08-11 B — rolled off, re-parked on the marks | +0.415 | −0.515 | **−85.00°** | 0.9128 |
| 2026-08-11 C — rolled off, re-parked deliberately off-mark | +0.385 | −0.515 | **−84.25°** | 0.8933 |
| 2026-08-11 — live AMCL settle | +0.516 | −0.472 | **−83.97°** | — |

**Total spread 1.03°.** The four scan fits span 0.75° with mean −84.69°; we kept −84.50° rather
than the mean because the 0.19° difference is inside the 0.25° search step.

Three points that make this stronger than five numbers in a column:

- **The peak is single and unambiguous.** The room-symmetry hypothesis is dead. It was never the
  reason for the recurring ~90° confusion here.
- **AMCL corroborates by a different algorithm.** It was seeded at −79.80° and settled at −83.97°,
  i.e. it walked 4.17° *toward* the geometry on its own.
- **Runs A and B landed on the identical search cell** (0.03 m / 0.25°) from different data, so the
  method's own noise is below its resolution. We verified the car genuinely moved between them
  rather than trusting the coincidence: `odom→base_link` went `(−0.025, 0.002, −0.30°)` →
  `(0.318, 0.225, −5.40°)` across the out-and-back.

## 4. What this means for you — the waypoint 0 problem

**Figure-8 waypoint 0 (−92.08°) scores 62–64 % of best in every single sample, 7.6–7.8° from the
map.** It is the worst of all candidates, worse than the value we just replaced.

This matters more than the seed change itself, because of how it fails. The check we have seen used
on that side — "seeded from waypoint 0, verified 19 mm from waypoint 0" — is **circular**. It
measures agreement between a seed and the thing it was seeded from, so it returns a clean pass for
*any* waypoint 0, correct or not. It cannot detect this class of error, and it will keep reporting
healthy while the run starts 7.7° rotated in the map frame.

**Our ask:** if the ego-MPC (or anything else on your side) initializes its map-frame pose from
figure-8 waypoint 0, that initialization is ~7.7° off and should move to the measured spot pose, or
to whatever the localizer reports once seeded. If you have an independent reason to believe
waypoint 0 — e.g. it was recorded rather than assumed — we would like to hear it, because that
would mean the route CSV and the map disagree about the room and that is a different problem.

## 5. What we deliberately did *not* change

**x/y stays at (0.445, −0.575).** Every fit prefers `y = −0.515` (identical in all four) and
`x = 0.385–0.415`, i.e. ~6 cm away. That is a consistent bias, not noise. We left it because
position sits in a much shallower basin than yaw at this grid resolution, 6 cm is well inside
AMCL's convergence, and the offline replay seeds share those numbers. It wants its own measurement,
not a drive-by edit. **If your pipeline is sensitive at the few-centimetre level, tell us and we
will prioritise it.**

**The offline replay seeds still say −79.82°.** `51_localize_offline.sh`, `check_map_frame.py` and
the map-build handoffs seed replays of the archived 2026-08-05 bags from the RTABMap ground truth.
Either those bags really started 4.7° from where the car parks today, or that ground truth carries
the same error the AMCL seed did. It is decidable in one command against an archived bag and we
have not run it yet. **This is the one open item that could move numbers you already have** — if
the RTABMap ground truth is biased, every map-frame quantity derived from those bags inherits it.
We will report either way.

## 6. Status and caveats

- **Applied in the repo, not yet on the robot.** gosling1 gets `f1tenth_launch` from a staged
  tarball rather than git, so the running stack still carries −1.3928 until the package is
  re-staged and rebuilt. Check with `ros2 param get /amcl initial_pose.yaw` rather than assuming.
- **One spot, one map.** This is the raslab `20260805/rtabmap_2d_final` grid and the one parking
  spot. It says nothing about any other start pose.
- **Everything above is reproducible** in about 40 s with the stack up and the car parked:
  ```
  ros2 bag record -o heading_check /lidar/scan_filtered      # 15-25 s
  python3 scripts/live_runs/heading_from_scan.py --bag heading_check \
    --map <map>/rtabmap_2d_final.yaml --xy-range 0.15
  ```
  The bags behind the table above are kept at `/mnt/shared_dir/claude_heading_0811/heading_{A,B,C}`.

Full method, the raw score profiles and the reasoning for each "not changed" decision:
`scripts/live_runs/DEMO_RUNBOOK_20260810.md` §5b. Buglog `bug-234`.
