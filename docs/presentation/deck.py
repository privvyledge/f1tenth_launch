#!/usr/bin/env python3
"""Assemble the Marp source for one cut of the deck, and enforce the conventions.

Called by build.sh.  Reads slides/*.md in name order, keeps the slides that
belong to the requested cut, checks them against ASSETS.md, and writes
out/deck_<cut>.md.

Cuts: lab | sponsor | research  (a slide opts in with `<!-- cut: lab sponsor -->`)
      talk                      (union of the three: every slide in at least one cut)
      full                      (everything, including reference-only slides)
"""
from __future__ import annotations

import argparse
import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent
SLIDES = HERE / "slides"
ASSETS_MD = HERE / "ASSETS.md"
OUT = HERE / "out"

CUTS = ("lab", "sponsor", "research")

CUT_RE = re.compile(r"<!--\s*cut:\s*([^>]*?)-->", re.I)
CARD_RE = re.compile(r"^\s*>\s*\[!PLACEHOLDER\s+([A-Z0-9][A-Z0-9\-]*)\s*\]", re.M)
SRC_RE = re.compile(r"<!--\s*src:", re.I)
COMMENT_RE = re.compile(r"<!--.*?-->", re.S)
AGENT_RE = re.compile(r"CLAUDE\.md|PLAN\.md|HANDOFF|\bbug-\d+", re.I)
# a bare number: not inside an identifier, and not the "2" of "2D"/"3D"
DIGIT_RE = re.compile(r"(?<![\w./-])\d(?![A-Za-z])")
ASSET_ROW_RE = re.compile(r"^\|\s*`?([A-Z][A-Z0-9\-]{2,})`?\s*\|\s*([^|]*?)\s*\|\s*([^|]*?)\s*\|")


def read_assets() -> dict[str, str]:
    """ID -> status, from the machine-readable table in ASSETS.md."""
    if not ASSETS_MD.exists():
        sys.exit("ASSETS.md is missing; the build cannot check placeholder IDs.")
    table: dict[str, str] = {}
    inside = False
    for line in ASSETS_MD.read_text(encoding="utf-8").splitlines():
        if line.startswith("<!-- assets-table:begin -->"):
            inside = True
            continue
        if line.startswith("<!-- assets-table:end -->"):
            inside = False
            continue
        if not inside:
            continue
        m = ASSET_ROW_RE.match(line)
        if m and m.group(1) != "ID":
            table[m.group(1)] = m.group(3).strip().strip("`")
    return table


def split_slides(text: str) -> tuple[str, list[str]]:
    """Return (front_matter, slides).  Splits on `---` at column 0, outside fences."""
    lines = text.splitlines()
    front: list[str] = []
    i = 0
    if lines and lines[0].strip() == "---":
        front.append(lines[0])
        i = 1
        while i < len(lines) and lines[i].strip() != "---":
            front.append(lines[i])
            i += 1
        if i < len(lines):
            front.append(lines[i])
            i += 1
    slides: list[list[str]] = [[]]
    fence = False
    for line in lines[i:]:
        if line.lstrip().startswith("```"):
            fence = not fence
        if not fence and line.strip() == "---":
            slides.append([])
            continue
        slides[-1].append(line)
    return "\n".join(front), ["\n".join(s).strip("\n") for s in slides]


def slide_cuts(slide: str) -> set[str]:
    m = CUT_RE.search(slide)
    if not m:
        return set()
    return {t for t in m.group(1).split() if t in CUTS}


def body_of(slide: str) -> str:
    """Slide text with HTML comments removed - what the audience actually sees."""
    return COMMENT_RE.sub("", slide)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("cut", choices=(*CUTS, "talk", "full"))
    ap.add_argument("--strict", action="store_true",
                    help="treat convention warnings (unsourced numbers) as errors")
    args = ap.parse_args()

    files = sorted(SLIDES.glob("*.md"))
    if not files:
        sys.exit(f"no section files in {SLIDES}")

    assets = read_assets()
    front = ""
    kept: list[str] = []
    errors: list[str] = []
    warnings: list[str] = []
    per_section: list[tuple[str, int, int]] = []

    for path in files:
        text = path.read_text(encoding="utf-8")
        fm, slides = split_slides(text)
        if fm and not front:
            front = fm
        elif fm:
            errors.append(f"{path.name}: only 00_overview.md may carry Marp front-matter")
        n_kept = 0
        for idx, slide in enumerate(slides):
            if not slide.strip():
                continue
            if not body_of(slide).strip():
                continue  # comment-only block (section header): never a slide
            cuts = slide_cuts(slide)
            if args.cut == "full":
                keep = True
            elif args.cut == "talk":
                keep = bool(cuts)
            else:
                keep = args.cut in cuts
            where = f"{path.name} slide {idx + 1}"

            for aid in CARD_RE.findall(slide):
                if aid not in assets:
                    errors.append(f"{where}: placeholder {aid} is not listed in ASSETS.md")
                elif assets[aid].upper() == "DONE":
                    errors.append(
                        f"{where}: {aid} is DONE in ASSETS.md but still shown as a card")

            body = body_of(slide)
            if AGENT_RE.search(body):
                errors.append(f"{where}: references an agent file or bug ID on the slide")
            visible = "\n".join(
                l for l in body.splitlines()
                if not l.lstrip().startswith(">") and not l.lstrip().startswith("![")
            )
            if DIGIT_RE.search(visible) and not SRC_RE.search(slide):
                warnings.append(f"{where}: has numbers but no `<!-- src: ... -->` note")

            if keep:
                kept.append(slide)
                n_kept += 1
        per_section.append((path.name, n_kept, len([s for s in slides if s.strip()])))

    for w in warnings:
        print(f"warn:  {w}", file=sys.stderr)
    for e in errors:
        print(f"ERROR: {e}", file=sys.stderr)
    if errors or (warnings and args.strict):
        return 1

    OUT.mkdir(exist_ok=True)
    dest = OUT / f"deck_{args.cut}.md"
    parts = [front] if front else []
    parts.append("\n\n---\n\n".join(kept))
    dest.write_text("\n\n".join(parts).rstrip() + "\n", encoding="utf-8")

    width = max(len(n) for n, _, _ in per_section)
    print(f"cut '{args.cut}':")
    for name, k, total in per_section:
        print(f"  {name:<{width}}  {k:>3} / {total}")
    print(f"  {'TOTAL':<{width}}  {len(kept):>3} slides -> {dest.relative_to(HERE)}")
    if warnings:
        print(f"  ({len(warnings)} convention warning(s); --strict makes them fatal)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
