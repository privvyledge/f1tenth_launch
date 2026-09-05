# Fleet state, 2026-09-04 — image 09042026 pushed; gosling2/5 blocked on disk

## What is done

`privvyledge/f1tenth:humble-devel-09042026` is **on Docker Hub**, digest
`sha256:fcffbba7dfee19ee8f568ddcdbe23ba7bbf0622bd814687bf8ab4f57e58e0176`
(verified through the registry API, matches the local image). It is a
`docker commit` of a container staged with `stage_0904.sh`, all 30 checks
passing. `~/bolus_ws/f1tenth_launch.sh` on gosling1 now points at it
(previous line kept commented; backup `f1tenth_launch.sh.bak-20260904`).

Repo: four commits on `perf/config-tuning`, pushed
(`18850da`, `44ed0a0`, `e33e5bc`, `594268a`).

The push was **24 MB, not 55 GB** — 14 of 16 layers were already on Hub via
`humble-devel-08092026`. Note `humble-devel-08302026` was never pushed on its
own; its layer went up as part of this one.

## The fleet, as measured (not as assumed)

| | gosling1 | gosling2 | gosling5 |
|---|---|---|---|
| address | 192.168.2.195 | 192.168.2.156 | 192.168.2.120 |
| ssh alias | yes | yes (added today) | yes (added today) |
| login user | gosling1 | gosling2 | gosling5 |
| **hostname** | gosling1 | gosling2 | **`ubuntu`** |
| board | Orin Nano Super | Orin Nano Super | Orin Nano Super |
| L4T | R36.4.3 | R36.4.3 | R36.4.3 |
| **root device** | **915 G NVMe** | **116 G eMMC** | **116 G eMMC** |
| free | 746 G | 3.3 G | 0.06 G |
| `docker compose` | v2.36.2 (installed today) | v2.36.2 | v2.35.1 |

`gosling5`'s hostname is still the L4T default `ubuntu`, which is why
`gosling5.local` does not resolve and mDNS discovery finds only gosling2. The
ssh alias works regardless. Renaming was **not** done — ask first.

`192.168.2.111` and `.173` are `prediss-edge-2` / `prediss-edge-1`: NVIDIA-OUI
Jetsons on the same subnet belonging to a different project. Not goslings.

## The blocker — this is a storage problem, not a network one

**Neither gosling2 nor gosling5 can hold the image.** The BSP assumption
("they have the same setup") holds for board and L4T; it does **not** hold for
storage. gosling1 works because of its NVMe.

The killer detail: **the new image shares ZERO of its 16 layers with
`humble-latest`** (checked by layer digest on both machines). So a pull there is
the full 55.4 GB on disk, not a delta — unlike the 24 MB push from gosling1.

Best case after removing everything safely removable:

* **gosling2: ~25 G free. Does not fit.** Must keep the three compose images
  (`humble-latest` 37.1 G + `noetic-ros-core` + `foxy-ros1-bridge` = 41.8 G).
  Only ~11.6 G of docker is reclaimable (`f1tenth-f1tenth_stack:latest` and a
  dangling twin, which share layers — the nominal 22.4 G is not real).
* **gosling5: ~57 G free. Fits with 1–2 G spare**, which is not operable: no
  room for bags, logs or a colcon build. ~49.4 G is reclaimable there
  (`humble-devel` and `humble-devel-05182026` are the SAME image under two tags).

`/mnt` is 17 G on both — a 16 GB **swapfile**. Not reclaimable.
`du` cannot see `/var/lib/docker` without root, which is why a naive `du /`
appears to lose ~51 G; that is the image store, and `docker system df` agrees.

**Nothing was deleted on either machine.**

## What the operator decided

Pause. Run the coworkers' test with the new image on gosling1 and the current
image on gosling2/5; if there is no behavioural difference, a later chat
overwrites gosling2/5. The coworkers only need teleop — mux, vehicle, joystick,
and `vehicle/vesc_odom` — not the full stack.

## Running the coworkers' test on gosling1

Their compose service is exactly:

```
ros2 launch f1tenth_launch teleop.launch.py launch_localization:=False \
    launch_vehicle:=True launch_sensors:=False use_f1tenth_namespace:=True
```

gosling1 had **no compose at all** (neither v1 nor the v2 plugin). The v2
plugin was copied from gosling2 into `~/.docker/cli-plugins/docker-compose`
(user-local, no sudo, reversible by deleting that one file).

**Why it was missing, per the operator: gosling1 was recently reflashed and
only the home directory was backed up.** That one fact explains three separate
things this session tripped over -- no compose plugin, no `humble-latest`
image, and a `~/shared_dir` still holding the April-2025 DDS config. Expect the
same shape after any future reflash: anything outside `$HOME` is gone, and the
`iptables raw` gap below is almost certainly the same cause.

Their `docker-compose.yml` is **unmodified**. Run it against the new image with
the additive override written today:

```bash
export VEHICLE_NAME=gosling1 DISPLAY=:0        # compose warns if unset
cd ~/bolus_ws/ros1_to_ros2_communication/docker
docker compose -f docker-compose.yml -f docker-compose.image-09042026.yml up ros2
```

`up ros2` alone avoids pulling `noetic-ros-core` and `foxy-ros1-bridge`, which
gosling1 does not have.

**Confound to state, not to fix:** the compose service mounts
`${HOME}/shared_dir`, so it reads the **April-2025** `cyclonedds_config_static.xml`
— no `lo` entry, NIC at `priority="default"` — not the bug-267-fixed copy that
lives on the SSD and that `f1tenth_launch.sh` mounts. gosling2 and gosling5
mount equally old configs, so the confound is **matched** across the three
machines and does not skew the comparison. But do not read these runs as
testing the DDS fix, and note the coworkers' stack has never had it.

## Two defects found today, both fixed

* **`stage_*.sh` aborted early, silently, for its whole history.** `grep -c`
  exits 1 on zero matches and several checks *pass* on zero (`bug-272 no
  images` is the point of that bug). Under `set -e` a passing check killed the
  script, so `stage_0902.sh` never reached its own `staged (0902).` line.
  Everything before it passed, so nobody noticed. 0904 routes every check
  through `cnt()`/`has()`.
* **gosling1 cannot start a container on the default bridge network.**
  `iptables --wait -t raw` fails — the `raw` table module is not loaded,
  apparently since the reflash. Use `--network host`, which the stack uses
  anyway. This will bite any `docker run` here that omits it.

Also: the Docker Hub PAT stored in gosling1's `~/.docker/config.json` had
**expired**; the operator re-logged in. It is stored base64 in plaintext there,
as Docker does by default.
