#!/usr/bin/env python3
"""Per-topic rate and gap analysis for a rosbag2 recording.

`ros2 bag info` gives message counts and a duration, which is enough for an
average rate but says nothing about *drops* — a stream that stalls for two
seconds mid-run and then catches up has a healthy-looking average. This walks
the actual message timestamps and reports the gap distribution, which is what
distinguishes "recorded fine" from "the recorder could not keep up".

Usage:
    python3 bag_stats.py <bag_dir_or_file> [--expect topic=hz ...] [--json]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

try:
    import rosbag2_py
except ImportError:  # pragma: no cover - depends on the ROS environment
    sys.exit("rosbag2_py not importable — source the ROS 2 overlay first")


def read_timestamps(bag_path: str) -> tuple[dict[str, list[int]], dict[str, str]]:
    """Return {topic: [recv_timestamp_ns, ...]} and {topic: type}."""
    reader = rosbag2_py.SequentialReader()

    storage_id = ""
    path = Path(bag_path)
    if path.is_dir():
        # Infer storage from the data files present; rosbag2 can usually
        # auto-detect, but being explicit avoids a confusing plugin error.
        if list(path.glob("*.mcap")):
            storage_id = "mcap"
        elif list(path.glob("*.db3")):
            storage_id = "sqlite3"

    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )

    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    stamps: dict[str, list[int]] = {name: [] for name in types}

    while reader.has_next():
        topic, _data, t_ns = reader.read_next()
        stamps.setdefault(topic, []).append(t_ns)

    return stamps, types


def summarize(stamps: list[int]) -> dict:
    """Rate and gap statistics for one topic's timestamp series."""
    n = len(stamps)
    if n < 2:
        return {"count": n, "hz": 0.0, "duration_s": 0.0,
                "gap_p50_ms": 0.0, "gap_p99_ms": 0.0, "gap_max_ms": 0.0,
                "stalls": 0}

    stamps = sorted(stamps)
    duration_s = (stamps[-1] - stamps[0]) / 1e9
    hz = (n - 1) / duration_s if duration_s > 0 else 0.0

    gaps_ms = [(b - a) / 1e6 for a, b in zip(stamps, stamps[1:])]
    gaps_ms.sort()

    def pct(p: float) -> float:
        if not gaps_ms:
            return 0.0
        idx = min(len(gaps_ms) - 1, int(p * len(gaps_ms)))
        return gaps_ms[idx]

    median = pct(0.50)
    # A "stall" is a gap more than 5x the median inter-message interval — long
    # enough that it is a dropout rather than ordinary jitter.
    stalls = sum(1 for g in gaps_ms if median > 0 and g > 5 * median)

    return {
        "count": n,
        "hz": hz,
        "duration_s": duration_s,
        "gap_p50_ms": median,
        "gap_p99_ms": pct(0.99),
        "gap_max_ms": gaps_ms[-1],
        "stalls": stalls,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="bag directory or file")
    ap.add_argument("--expect", action="append", default=[], metavar="TOPIC=HZ",
                    help="minimum expected rate; repeatable. Substring match on "
                         "the topic name, so 'scan_filtered=5' matches the "
                         "namespaced topic.")
    ap.add_argument("--json", action="store_true", help="machine-readable output")
    args = ap.parse_args()

    expectations: list[tuple[str, float]] = []
    for item in args.expect:
        if "=" not in item:
            return _fail(f"bad --expect {item!r}, want TOPIC=HZ")
        key, _, val = item.partition("=")
        try:
            expectations.append((key, float(val)))
        except ValueError:
            return _fail(f"bad rate in --expect {item!r}")

    stamps, types = read_timestamps(args.bag)
    if not stamps:
        return _fail(f"no messages found in {args.bag}")

    results = {topic: summarize(ts) for topic, ts in stamps.items()}

    if args.json:
        print(json.dumps({"bag": args.bag, "topics": results}, indent=2))
        return 0

    total_msgs = sum(r["count"] for r in results.values())
    span = max((r["duration_s"] for r in results.values()), default=0.0)
    print(f"\nBag: {args.bag}")
    print(f"{len(results)} topics, {total_msgs} messages, {span:.1f} s\n")

    header = f"{'topic':<52}{'count':>8}{'Hz':>9}{'p50 ms':>9}{'max ms':>9}{'stalls':>8}"
    print(header)
    print("-" * len(header))

    for topic in sorted(results):
        r = results[topic]
        flag = "  <-- STALLS" if r["stalls"] else ""
        print(f"{topic:<52}{r['count']:>8}{r['hz']:>9.2f}"
              f"{r['gap_p50_ms']:>9.1f}{r['gap_max_ms']:>9.1f}"
              f"{r['stalls']:>8}{flag}")

    # Empty topics are the failure mode that hid the namespace bugs: the topic
    # is in the bag, so it looks recorded, but it carries nothing.
    empty = [t for t, r in results.items() if r["count"] == 0]
    if empty:
        print("\nEMPTY topics (recorded but never published):")
        for t in sorted(empty):
            print(f"  {t}   [{types.get(t, '?')}]")

    failures = 0
    if expectations:
        print("\nRate expectations:")
        for pattern, min_hz in expectations:
            matches = [t for t in results if pattern in t]
            if not matches:
                print(f"  MISSING  {pattern}  (no topic matched)")
                failures += 1
                continue
            for t in matches:
                hz = results[t]["hz"]
                ok = hz >= min_hz
                failures += 0 if ok else 1
                print(f"  {'ok ' if ok else 'FAIL'}     {t:<48}"
                      f"{hz:>8.2f} Hz  (want >= {min_hz})")

    if failures:
        print(f"\n{failures} expectation(s) failed")
        return 1

    stalled = [t for t, r in results.items() if r["stalls"]]
    if stalled:
        print(f"\n{len(stalled)} topic(s) show stalls — the recorder or the "
              f"publisher could not keep up:")
        for t in sorted(stalled):
            print(f"  {t}  ({results[t]['stalls']} gaps > 5x median)")

    return 0


def _fail(msg: str) -> int:
    print(f"error: {msg}", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
