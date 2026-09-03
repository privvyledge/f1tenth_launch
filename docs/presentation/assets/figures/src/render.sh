#!/usr/bin/env bash
# Render every .mmd in this directory to ../<name>.svg.
#
# Needs a Chromium for puppeteer.  Point CHROME at one if the default is wrong:
#   CHROME=/path/to/chrome ./render.sh
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

CHROME="${CHROME:-$(command -v chromium || command -v chromium-browser || command -v google-chrome || true)}"
[ -n "$CHROME" ] && export PUPPETEER_EXECUTABLE_PATH="$CHROME"

cat > .puppeteer.json <<'JSON'
{"args":["--no-sandbox","--disable-gpu","--disable-dev-shm-usage"]}
JSON

for f in *.mmd; do
  out="../${f%.mmd}.svg"
  echo "  $f -> $out"
  npx --yes @mermaid-js/mermaid-cli@11.4.2 \
      -i "$f" -o "$out" -b transparent -p .puppeteer.json
done
rm -f .puppeteer.json
