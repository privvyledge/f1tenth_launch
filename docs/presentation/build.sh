#!/usr/bin/env bash
# Build one cut of the F1TENTH deck.
#
#   ./build.sh                     # talk cut, all formats
#   ./build.sh lab                 # lab cut, all formats
#   ./build.sh research pdf        # one format
#   ./build.sh full md             # assemble the Markdown only, no renderer
#   STRICT=1 ./build.sh lab        # convention warnings become errors
#
# Cuts: lab | sponsor | research | talk (union of the three) | full (everything).
# Formats: pdf html pptx md   (default: pdf html pptx)
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

CUT="${1:-talk}"
shift || true
FORMATS=("$@")
[ ${#FORMATS[@]} -eq 0 ] && FORMATS=(pdf html pptx)

MARP_VERSION="4.5.0"
MARP=(npx --yes "@marp-team/marp-cli@${MARP_VERSION}")

# 1. assemble + check conventions
STRICT_FLAG=()
[ -n "${STRICT:-}" ] && STRICT_FLAG=(--strict)
python3 deck.py "$CUT" "${STRICT_FLAG[@]}"

SRC="out/deck_${CUT}.md"

# 2. render
rendered=0
for fmt in "${FORMATS[@]}"; do
  case "$fmt" in
    md) continue ;;
    pdf|html|pptx) ;;
    *) echo "unknown format: $fmt" >&2; exit 2 ;;
  esac
  echo "  rendering $fmt ..."
  if ! "${MARP[@]}" --allow-local-files --theme-set theme/f1tenth.css \
        "--$fmt" -o "out/deck_${CUT}.${fmt}" "$SRC"; then
    echo "  !! marp failed for $fmt (needs npx + a Chromium for pdf/pptx)" >&2
    exit 1
  fi
  rendered=$((rendered + 1))
done

echo "done: $SRC${rendered:+ (+$rendered rendered file(s) in out/)}"
