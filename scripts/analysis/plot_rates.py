#!/usr/bin/env python3
"""Plot achieved-vs-configured publish rates across live_rates.py conditions.

Consumes the JSON that scripts/live_runs/live_rates.py writes -- one file per
measured condition -- and draws a grouped bar chart of achieved rate as a
percentage of each stream's configured rate.

Why percent and not Hz: the streams being compared run at 30 Hz and 200 Hz, so
absolute Hz puts the camera IMU an order of magnitude above everything else and
hides the thing worth seeing.  What matters is which streams hold 100 % and
which do not, under the same load.

Why this is a separate script: live_rates.py runs on the robot inside the ROS
container and must stay importable there without matplotlib.  This runs on a
workstation against the committed JSON.

Usage:
  plot_rates.py --out FIG.png LABEL=FILE.json [LABEL=FILE.json ...]
"""
import argparse
import json
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# Deck theme (docs/presentation/theme/f1tenth.css).
FG = "#14181d"
MUTED = "#5a6572"
RULE = "#d4dae1"
ACCENT = "#b3282d"          # FSU garnet -- reserved for the streams that fail
OK = "#1c7c46"

# (logical topic, legend label, colour).  Order is the plot order.
SERIES = [
    ("camera/infra1/image_rect_raw", "IR stereo pair (infra1/infra2)", ACCENT),
    ("visual_slam/tracking/odometry", "Visual SLAM odometry", "#d97176"),
    ("camera/color/image_raw", "Colour image", MUTED),
    ("camera/imu/filtered", "Camera IMU (filtered)", "#8c99a8"),
    ("odometry/local", "Fused odometry (EKF)", OK),
]


def load(path):
    with open(path) as fh:
        return json.load(fh)


def pct(doc, topic):
    """Achieved rate as a percentage of configured, or None if unmeasured."""
    entry = doc["topics"].get(topic)
    if entry is None or not entry.get("advertised"):
        return None
    target = entry.get("configured_hz")
    if not target:
        return None
    return 100.0 * entry["hz"] / target


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("conditions", nargs="+", metavar="LABEL=FILE",
                    help="condition label and its live_rates.py JSON; labels "
                         "may use \\n for a line break")
    ap.add_argument("--out", required=True)
    ap.add_argument("--title", default="")
    args = ap.parse_args()

    labels, docs = [], []
    for spec in args.conditions:
        if "=" not in spec:
            print(f"error: expected LABEL=FILE, got {spec!r}", file=sys.stderr)
            return 2
        label, path = spec.split("=", 1)
        labels.append(label.replace("\\n", "\n"))
        docs.append(load(path))

    n_cond, n_series = len(docs), len(SERIES)
    width = 0.8 / n_series
    x = np.arange(n_cond)

    fig, ax = plt.subplots(figsize=(11.0, 3.5), dpi=200)
    for i, (topic, label, colour) in enumerate(SERIES):
        vals = [pct(d, topic) for d in docs]
        pos = x - 0.4 + width * (i + 0.5)
        heights = [0.0 if v is None else v for v in vals]
        ax.bar(pos, heights, width * 0.92, label=label, color=colour,
               edgecolor="white", linewidth=0.4)
        for px, v in zip(pos, vals):
            if v is None:
                continue
            # Only annotate the shortfalls -- labelling every 100 % bar is noise.
            if v < 97.0:
                ax.text(px, v + 1.5, f"{v:.0f}%", ha="center", va="bottom",
                        fontsize=7.5, color=FG,
                        fontweight="bold" if v < 60 else "normal")

    ax.set_ylim(0, 118)
    ax.set_yticks([0, 25, 50, 75, 100])
    ax.set_ylabel("achieved rate, % of configured", fontsize=9, color=FG)
    ax.axhline(100, color=RULE, linewidth=0.9, zorder=0)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=8.5, color=FG)
    ax.tick_params(axis="y", labelsize=8, colors=MUTED)
    ax.tick_params(axis="x", length=0)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.spines["left"].set_color(RULE)
    ax.spines["bottom"].set_color(RULE)
    ax.set_axisbelow(True)
    ax.yaxis.grid(True, color=RULE, linewidth=0.5, alpha=0.7)
    if args.title:
        ax.set_title(args.title, fontsize=10.5, color=FG, loc="left", pad=10)
    ax.legend(fontsize=8, ncol=5, frameon=False, loc="upper center",
              bbox_to_anchor=(0.5, 1.14), labelcolor=FG, columnspacing=1.2,
              handlelength=1.1, handletextpad=0.5)

    fig.tight_layout()
    fig.savefig(args.out, bbox_inches="tight", facecolor="white")
    print(f"wrote {args.out}")

    # Echo the numbers so the caption can be checked against the figure.
    for label, doc in zip(labels, docs):
        flat = label.replace("\n", " ")
        parts = []
        for topic, name, _ in SERIES:
            v = pct(doc, topic)
            parts.append(f"{name}={'n/a' if v is None else f'{v:.1f}%'}")
        print(f"  {flat}: " + ", ".join(parts))
    return 0


if __name__ == "__main__":
    sys.exit(main())
