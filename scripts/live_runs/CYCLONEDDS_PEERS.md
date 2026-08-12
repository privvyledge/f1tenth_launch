# CycloneDDS static peers — log noise and the fix

**Status: applied on the robot 2026-08-07, NOT in any repo.**
`cyclonedds_config_static.xml` is owned by the separate workspace/build repo,
not by `f1tenth_launch`. The live copy at
`/mnt/f1tenth_ssd/shared_dir/cyclonedds_config_static.xml` on gosling1 (selected
via `CYCLONEDDS_URI`) now has `192.168.2.193` commented out; backup of the
previous contents is `cyclonedds_config_static.bak08072026.xml` beside it. **The
same edit still has to be made in the build repo at merge time**, or the next
deploy silently reverts it.

Profile selection is wired into these scripts as `DDS_PROFILE`
(`static` | `lo` | `none`) in `00_env.sh`; the four offline scripts
(`40`, `51`, `52`, `61`) default to `lo`, everything else to `static`.

Written 2026-08-05 after the noise buried a real YDLidar failure during a live
recording session.

## Operator decision, 2026-08-07 — apply a PARTIAL Fix A

- Comment out **`192.168.2.193` (gosling3) only**. Do not delete the line.
- **Leave `192.168.2.194` (DigitalStorm) active** even though it is down.
- Adopt `cyclonedds_offline_lo.xml` for **local-only** runs (see below).
- Leave the `lo` interface entry alone.

**Expect reduced, not eliminated, noise.** `.194` is the address observed
flooding the logs on 2026-08-07 and it stays in by instruction, so the `tev`
thread keeps retrying it. Do not report this change as "the spam is fixed".

**Reachability re-measured 2026-08-07 12:15 EDT** — the table below is stale in
one row: `.140` (gosling5) is now **UP**, not down. Only `.193` and `.194` are
unreachable today. The list changed on its own within two days, which is the
argument for Fix B (render from a reachability probe) over hand-editing.

**Local-only runs:** `/mnt/f1tenth_ssd/shared_dir/cyclonedds_offline_lo.xml` is
lo-only with a single `localhost` peer and produces zero peer noise. Containers
run on host networking, so same-host processes (e.g. an external MPC node) still
discover each other. The cost is the static-peer trap in its strongest form:
anything not in the list is invisible with no error, so remote RViz sees
nothing. Use it only when nothing off-robot needs the topics.

## Correction, measured 2026-08-07 13:53 — reachability is NOT the trigger

Counting the spam by destination address in two full stack logs:

| Log | `.140` | `.141` | `.193` | `.194` |
|---|---|---|---|---|
| `mpc_stack_20260807_112934.log` (before the edit, 5,176,774 lines) | 1,294,000 | 1,294,000 | 1,294,000 | 1,294,000 |
| `mpc_stack_20260807_135313.log` (after the edit) | 34,800 | 34,800 | — | 34,800 |

**Every remote peer produces an identical count, whether it is up or down.** ICMP
at the same moment: `.140` UP, `.141` UP, `.193` DOWN, `.194` DOWN, `.195` (self)
UP. So the "peers that are powered off" explanation below is wrong as a *cause* —
`retcode -3` also fires for peers that answer ping.

The likely mechanism is the second interface: gosling1 has `lo` and `wlP1p1s0`
in `<Interfaces>`, and the `tev` thread announces to every peer on every
interface. A `192.168.2.x` address is unroutable via `lo`, so each remote peer
costs exactly one `EHOSTUNREACH` per announcement regardless of its state.

Consequences for planning:
- The spam scales with **peer count**, not with how many are down. Commenting out
  `.193` removed exactly one of four equal shares: **~25 % less noise, and only
  that.** `.194` staying in is not what makes the remaining 75 %.
- **Fix B (render the list from a reachability probe) would not have helped
  either** — a probe keeps `.140` and `.141`, which flood at the same rate.
- The lever that would actually silence it is the `lo` entry, and that is off the
  table by operator decision (it took VSLAM frame stalls 0.1373/s → 0.0294/s).
  `cyclonedds_offline_lo.xml` sidesteps it instead, by having no remote peers.

## Symptom

Every ROS process writes a continuous stream of:

```
1785934315.165424 [77]  tev: ddsi_udp_conn_write to udp/192.168.2.140:26660 failed with retcode -3
1785934315.165455 [77]  tev: ddsi_udp_conn_write to udp/192.168.2.140:26662 failed with retcode -3
...
```

One `--launch-only` run of the sensor stack produced a **381,976-line** log. The
YDLidar's actual failure — `Failed to start scan mode -1`, four lines total —
was invisible without targeted greps.

## Cause

`<Discovery><Peers>` lists five machines. `retcode -3` is `EHOSTUNREACH`: the
`tev` (timed-event) thread periodically sends SPDP participant announcements to
every configured peer, ARP fails for the ones that are powered off, and each
failure is logged. With `<AllowMulticast>false</AllowMulticast>` there is no
fallback path, so this repeats forever at roughly **200 lines/second**.

Peer status measured on 2026-08-05 09:55 from gosling1:

| Address | Machine | State |
|---|---|---|
| `localhost`, `192.168.2.195` | gosling1 (self) | — |
| `192.168.2.141` | gosling2 | **UP** |
| `192.168.2.193` | gosling3 | down |
| `192.168.2.140` | gosling5 | down |
| `192.168.2.194` | DigitalStorm | down |

## Measurement

20 s of `ros2 run demo_nodes_cpp talker`, counting all stderr lines:

| Config | Lines |
|---|---|
| Current (5 peers, 3 unreachable) | **4020** |
| Peers trimmed to `localhost` + `.195` + `.141` | **20** |
| Current + `<Tracing><OutputFile>` | **4020** (plus 4000 duplicated into the file) |

## What does NOT work

**`<Tracing><OutputFile>` does not redirect these.** Tested. CycloneDDS has two
sinks: the *trace* sink (`OutputFile`, filtered by `<Verbosity>`) and the *log*
sink for errors and warnings, which always goes to stderr. `ddsi_udp_conn_write`
failures are warnings on the log sink, so `OutputFile` merely duplicates them —
you get the noise on the console *and* a 4000-line file. There is no XML knob
that silences them.

**Lowering `<Verbosity>` does not help either**, for the same reason: it filters
the trace sink, not the log sink.

## Fix A — trim the peer list (simple, needs discipline)

Remove the unreachable peers, but leave them commented so re-adding is a
one-line uncomment rather than an archaeology exercise:

```xml
<Discovery>
    <Peers>
        <Peer address="localhost"/>       <!-- same-host, over lo -->
        <Peer address="192.168.2.195"/>   <!-- gosling1 -->
        <Peer address="192.168.2.141"/>   <!-- gosling2 -->
        <!-- Uncomment a line below ONLY while that machine is powered on.
             A peer that is configured but unreachable costs ~200 failed
             sendto() calls per second, per process, and buries real errors.
             After uncommenting, restart the ROS processes that need to see it. -->
        <!-- <Peer address="192.168.2.193"/>  gosling3 -->
        <!-- <Peer address="192.168.2.140"/>  gosling5 -->
        <!-- <Peer address="192.168.2.194"/>  DigitalStorm (viz desktop) -->
    </Peers>
    <ParticipantIndex>auto</ParticipantIndex>
    <MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>
</Discovery>
```

**The trap this introduces:** with static peers and multicast disabled, a
machine that is not in the list is *invisible* — no error, no warning, it simply
never discovers the robot. If RViz on DigitalStorm shows nothing, check this
file before debugging anything else.

## Fix B — generate the peer list from the environment (preferred)

Better suited to a lab where machines come and go: keep a template and render it
at container start, so the list always matches who is actually on the network
and nobody hand-edits XML.

`cyclonedds_config_static.xml.in`, with `@PEERS@` where the block goes, plus:

```bash
#!/usr/bin/env bash
# render_dds_config.sh — run at container start, before any ROS process.
#   DDS_PEERS="192.168.2.141 192.168.2.194" ./render_dds_config.sh
# With DDS_PEERS unset, defaults to same-host only, which is correct for a
# robot running standalone and produces zero peer noise.
set -euo pipefail
TEMPLATE="${DDS_TEMPLATE:-/opt/config/cyclonedds_config_static.xml.in}"
OUT="${DDS_CONFIG_OUT:-/tmp/cyclonedds_config.xml}"
SELF_IP="$(hostname -I | awk '{print $1}')"

peers=$'        <Peer address="localhost"/>\n'
peers+="        <Peer address=\"${SELF_IP}\"/>"$'\n'
for p in ${DDS_PEERS:-}; do
  # Skip peers that are not answering: a configured-but-dead peer costs
  # ~200 failed sendto() per second per process and buries real errors.
  if ping -c1 -W1 "$p" >/dev/null 2>&1; then
    peers+="        <Peer address=\"${p}\"/>"$'\n'
  else
    echo "render_dds_config: SKIPPING unreachable peer ${p}" >&2
  fi
done

python3 - "$TEMPLATE" "$OUT" <<PY
import sys
tpl, out = sys.argv[1], sys.argv[2]
open(out, "w").write(open(tpl).read().replace("@PEERS@", """${peers}"""))
PY
echo "export CYCLONEDDS_URI=file://${OUT}"
```

The reachability probe is the part that matters: it makes the common failure
(someone left a dead machine in the list) self-correcting, while still logging
loudly which peer was dropped so an intentional peer going missing is visible.

Trade-off to be aware of: a machine booted *after* the container starts will not
be in the rendered list. Either re-render and restart, or add it to
`DDS_PEERS` and accept its noise until it comes up.

## Fix C — multicast

If the lab router ever gains working multicast, the whole static-peer mechanism
becomes unnecessary: drop `<Peers>`, set `<AllowMulticast>true</AllowMulticast>`,
and re-enable `autodetermine`. The current config exists specifically because
multicast was unreliable on this network. Worth re-testing when the network
changes; it removes this entire class of problem.

## Unrelated but adjacent

Do not remove the `lo` interface entry. It was added on 2026-08-04 to keep
same-host traffic (848x480 stereo into `visual_slam_node`) off the WiFi stack,
and it measurably fixed VSLAM frame stalls: 0.1373/s down to 0.0294/s, a 4.7x
improvement. That change is unrelated to the peer list and must survive any edit
here.
