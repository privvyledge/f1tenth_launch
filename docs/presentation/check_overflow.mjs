#!/usr/bin/env node
//
// Find slides whose content is taller than the 720 px Marp gives them.
//
// Marp clips silently: no warning, no error, the PDF simply loses the bottom of
// the slide.  21 of 70 slides overflowed the first time this deck was written,
// and none of them showed up in `build.sh`.  Run this before handing a section
// back.
//
//   ./build.sh full md                       # assemble out/deck_full.md first
//   ./check_overflow.mjs                     # then check it
//   ./check_overflow.mjs out/deck_lab.md     # or any other cut
//
// Exit status is 1 if anything overflows, so it can gate a hand-back.
//
// Sizing rules that avoid the fix-and-remeasure loop:
//   - usable content height is ~616 px  (720 minus 48 top and 56 bottom padding)
//   - subtract ~72 px for a one-line `h2`, ~113 px for a two-line one
//   - size figures with `h:` rather than `w:` - the height is the constraint
//   - put a near-square diagram in `<div class="split">` instead of shrinking it
//
// Needs node, a Chromium, and puppeteer-core.  Both paths are discovered from
// the environment when possible; override either explicitly:
//
//   CHROME=/path/to/chrome PUPPETEER=/path/to/puppeteer-core ./check_overflow.mjs
//
import { execFileSync } from 'node:child_process';
import { existsSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const HERE = dirname(fileURLToPath(import.meta.url));
const SRC = resolve(HERE, process.argv[2] ?? 'out/deck_full.md');
const BARE = resolve(HERE, 'out/_overflow_check.html');

// 720 px is the slide; allow 3 px of rounding before calling it an overflow.
const LIMIT_H = 723;
const LIMIT_W = 1283;

function findChrome() {
  if (process.env.CHROME) return process.env.CHROME;
  if (process.env.PUPPETEER_EXECUTABLE_PATH) return process.env.PUPPETEER_EXECUTABLE_PATH;
  const guesses = [
    `${process.env.HOME}/.cache/ms-playwright`,
    `${process.env.HOME}/.cache/puppeteer`,
  ];
  for (const root of guesses) {
    if (!existsSync(root)) continue;
    try {
      const hit = execFileSync('find', [root, '-type', 'f', '-name', 'chrome', '-perm', '-u+x'],
                               {encoding: 'utf8'}).split('\n').filter(Boolean).pop();
      if (hit) return hit;
    } catch { /* keep looking */ }
  }
  for (const p of ['/usr/bin/chromium', '/usr/bin/chromium-browser',
                   '/usr/bin/google-chrome', '/snap/bin/chromium']) {
    if (existsSync(p)) return p;
  }
  return null;
}

function findPuppeteer() {
  if (process.env.PUPPETEER) return process.env.PUPPETEER;
  try {
    // puppeteer-core ships inside the npx cache that mermaid-cli or marp populated
    const hit = execFileSync('find', [`${process.env.HOME}/.npm/_npx`, '-maxdepth', '4',
                                      '-type', 'd', '-name', 'puppeteer-core'],
                             {encoding: 'utf8'}).split('\n').filter(Boolean)[0];
    if (hit) return `${hit}/lib/esm/puppeteer/puppeteer-core.js`;
  } catch { /* fall through */ }
  return 'puppeteer';
}

const chrome = findChrome();
if (!chrome) {
  console.error('no Chromium found; set CHROME=/path/to/chrome');
  process.exit(2);
}

// `--template bare` stacks every slide as its own <section>, which is what makes
// the per-slide measurement possible.  The default template shows one at a time.
const marp = process.env.MARP ?? (() => {
  try {
    return execFileSync('find', [`${process.env.HOME}/.npm/_npx`, '-maxdepth', '4',
                                 '-name', 'marp'],
                        {encoding: 'utf8'}).split('\n').filter(Boolean)[0];
  } catch { return null; }
})();
if (!marp) {
  console.error('no marp binary found; run ./build.sh once, or set MARP=/path/to/marp');
  process.exit(2);
}
execFileSync(marp, ['--allow-local-files', '--theme-set', 'theme/f1tenth.css',
                    '--template', 'bare', '--html', '-o', BARE, SRC],
             {cwd: HERE, stdio: 'ignore', env: {...process.env, PUPPETEER_EXECUTABLE_PATH: chrome}});

const puppeteer = (await import(findPuppeteer())).default;
const browser = await puppeteer.launch({
  executablePath: chrome,
  args: ['--no-sandbox', '--disable-gpu', '--disable-dev-shm-usage'],
});
const page = await browser.newPage();
await page.setViewport({width: 1400, height: 900});
await page.goto(pathToFileURL(BARE).href, {waitUntil: 'networkidle0'});
const slides = await page.evaluate(() =>
  [...document.querySelectorAll('section')].map((s, i) => {
    const h = s.querySelector('h2') ?? s.querySelector('h1');
    return {n: i + 1, h: s.scrollHeight, w: s.scrollWidth,
            title: (h ? h.textContent : '(untitled)').slice(0, 58)};
  }));
await browser.close();

const over = slides.filter(s => s.h > LIMIT_H || s.w > LIMIT_W);
for (const s of over) console.log(`OVERFLOW  slide ${s.n}  ${s.w}x${s.h}  ${s.title}`);
console.log(`checked ${slides.length} slides, ${over.length} overflowing`);
process.exit(over.length ? 1 : 0);
