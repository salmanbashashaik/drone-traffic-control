# Review: decision-log viewers (utm-viz-rerun, utm-viz-cesium), PR #27

- Reviewer model: Sonnet 5.5 (claude-sonnet-5-5). The author differs, but this is not the strongest model the project asks for on safety PRs. These are viewer-only changes.
- Commit: b1ee602 (branch demo/decision-viewer), diff against origin/main.
- Scope:
  - `viz_rerun.py`
  - `viz_cesium.py` (Python and the embedded page)
  - `tests/test_viz.py`
  - `docs/demo/VIEWERS.md`
  - the `--decision-log` additions in `cli.py`
  - `pyproject.toml`
  - `decision_log.py` and `filter.py` were read only to understand the log format.
- I did not open the page in a browser. The JS findings come from reading the code.

## Verified by running

- `pytest tests/test_viz.py -n0`: 5 passed (29 s).
- Built Cesium output from `runs/px4/cross3-filter-on`. The log has `scenario_sha256 = None` and a real `script_sha256`. The build works, and it has 120 steps of 100 ms, 3 drones in every step, no gaps.
- Started `--serve` with fake keys on 127.0.0.1:8010, probed it, then killed it. No listener remained.
  - `keys.js` is served with the keys to a request with `Referer: http://evil.example/` and `Sec-Fetch-Site: cross-site` (F1).
  - `keys.js` is also served with `Host: evil.example:8010` (F1).
  - Traversal attempts all failed: `/../secret.txt`, `%2f`, `%2e%2e` and `\` variants, a drive path and a UNC path. The `%5c` variants return a 301 to a directory, which then serves the out directory itself. No file outside `--out` was served.
  - No key appears on disk: the tests and the files written to `--out` show none. `keys.js` is never written.
  - Only the default `/keys.js` path is special-cased. `HEAD /keys.js` returns 404.
- Built a Cesium page from two logs with `script_sha256 = None` (synthetic copy of cross4). It is accepted as "same script" and the toggle is enabled (F2).
- `--origin -12.9,77` fails with argparse "expected one argument". `--origin=-12.9,77,nan` is accepted (F6).
- Checked `separation_rgb` against the docs and the page legend: red below d_eff, yellow (230,220,40) at 2 d_eff, green (40,200,90) from 4 d_eff. They match, and the function is continuous at 4 d_eff.
- Docs commands match the actual CLI:
  - the `utm-run` flags `--variant`, `--out` and `--decision-log`;
  - `utm-decision-log <dir>`;
  - `utm-viz-rerun <dir> [--save]`;
  - `utm-viz-cesium A B --out ... --serve`, with default port 8002.

## Findings

### HIGH

**F1. `keys.js` leaks the keys to any web page (`viz_cesium.py:185-196`, `:241-246`).**
- The handler answers `/keys.js` with `window.UTM_KEYS = {...}` for any request. It does not check Host, Origin, Referer or `Sec-Fetch-*`.
- A script tag is exempt from CORS, so any site open in the same browser can run `<script src="http://127.0.0.1:8002/keys.js">` while `--serve` is up. It then reads `window.UTM_KEYS`, which holds the Google key and the Cesium ion token. The default port is guessable.
- A DNS-rebinding page gets the same result, because Host is not checked.
- I confirmed the server returns the keys for a cross-site Referer and a foreign Host.
- This breaks the requirement that keys are served from memory on 127.0.0.1 and not leaked. Binding to 127.0.0.1 does not protect against the user's own browser.
- Fix, either of these:
  - Make the key URL unguessable: pick a random token at startup, serve `/keys-<token>.js`, and inject that name into `index.html` in the handler from memory. `index.html` on disk then holds only a placeholder, not a key.
  - Or reject the request unless `Host` is `127.0.0.1:<port>` or `localhost:<port>` and `Sec-Fetch-Site` is absent or `same-origin`/`none`. Add `Cross-Origin-Resource-Policy: same-origin`. This is weaker than the token, because old browsers send no Sec-Fetch headers.
- Add a test that sends a foreign Host header and expects a non-200.

### MEDIUM

**F2. A `None` script hash enables the on/off toggle and crashes the page (`viz_cesium.py:158`, `:301`).**
- `len({h["script_sha256"] ...}) != 1` treats two missing hashes (`None == None`) as the same script, so logs made before this PR or without a hash get the toggle. This breaks "toggle only for the same script hash".
- The page code is `r.script_sha256.slice(0, 12)`. For a single log whose hash is null, this throws a TypeError inside the async IIFE. The page stops before the clock and camera are set, and the runs list is left half built. I did not run this in a browser. The code path is clear.
- Today's PX4 manifest does set `script_sha256` (via `px4_demo.py`), so the current PX4 log works. `scenario_sha256` is null there. Rerun handles null (`viz_rerun.py:50`).
- Fix:
  - in `build`, raise a ValueError if any `script_sha256` is falsy when two logs are given;
  - in JS, use `(r.script_sha256 || "none").slice(0, 12)`.
- Add a test for each.

**F3. The default basemap with a token puts the drones underground (`viz_cesium.py:40`, `:270`; `VIEWERS.md:95`).**
- With `CESIUM_ION_TOKEN` set, the default (auto) mode loads World Terrain and OSM Buildings. The world z = 0 height H defaults to 0 m on the ellipsoid.
- The docs say the Bengaluru ground is several hundred metres above the ellipsoid. In the default configuration for a token holder, the traffic is therefore inside the terrain.
- This is documented, but the default is wrong for the mode it selects.
- Fix, either of these:
  - make `auto` pick OSM unless `--origin` has a height, or
  - warn in the page and the CLI when ion or Google mode is used with H = 0.

**F4. Strings from the log or the command line go into the page unescaped (`viz_cesium.py:180`, `:299-301`).**
- `json.dumps(manifest)` is placed inside `<script>`. A `</script>` in `scenario_id`, `episode_id` or the log path ends the script block.
- The same fields are then put into `innerHTML` without escaping.
- The manifest also stores `str(log)`, so the page and `manifest.json` carry a local filesystem path, which includes the user name.
- The data is local, so the severity is limited. If someone shares the output folder or a crafted scenario, it is HTML injection.
- Fix:
  - replace `<` with `<` in the embedded JSON;
  - set values with `textContent`;
  - store only the log's file name in the manifest.

### LOW

**F5. Positions end one step early (`viz_cesium.py:111-118`).**
- Availability runs to `times[-1] + dt_s`, but the sampled positions end at `times[-1]`, and Cesium's default extrapolation is NONE. Each drone, its arrows and its sphere disappear during the last 0.1 s.
- Fix: add `"forwardExtrapolationType": "HOLD"` and `"forwardExtrapolationDuration": dt_s` to the position packet.
- If a drone were ever missing from some steps, LINEAR interpolation would draw it crossing the gap. Not seen in any log here, where every step has all drones.

**F6. `--origin` parsing (`viz_cesium.py:222-228`).**
- A negative latitude (southern hemisphere) fails with the argparse error "expected one argument" unless written `--origin=-12.9,77`. The usage text and docs do not say so.
- `nan` or `inf` is accepted for the height (`--origin=-12.9,77,nan` built a page).
- Fix: use `math.isfinite` on all three values, and document the `=` form.

**F7. Rerun backup paths start at the true position but are computed from the observed state (`viz_rerun.py:82`).**
- `strips.append([drones[aid]["pos"], *path_m])` prefixes the truth position to a path computed from the observed (noisy) state in `decision_steps`.
- With T2 disturbances (`--profile nominal` or `realistic`) the first segment jumps. The drawn pair `backup_min_m` is also from the observed state, while `sep_m` and the sphere colours use truth.
- Not wrong for T1 and PX4, but nothing says so. Fix: note it in the docs, or start the strip at the first path point.

**F8. The Rerun view does not say it is a simulation replay (`viz_rerun.py:51-53`).**
- The Cesium page says it. The Rerun "Run" text panel shows `backend` and `truth_source`, but has no explicit "simulation replay, not live data" line. The docs do say it.
- Fix: add one line to the `about` document.

**F9. Tests and CI.**
- CI installs `.[dev,plot]` only (`.github/workflows/tests.yml:18,30`), so `test_rerun_recording_is_written` is always skipped by `importorskip`. Add `viz` to the CI install, or accept that Rerun is untested there.
- No tests cover: a null script hash (F2), the Host/cross-site check (F1), the CZML interval structure (`bounds`, `held`, last interval) or the page JS.
- The "no key on disk" test only scans `--out`. That is fine, because no other path writes.
- The `--help` and docs of `--serve` do not mention that the server also lists the output directory (it serves `manifest.json` and the CZML files). Acceptable.

**F10. Docs.**
- `VIEWERS.md:68` says a single log gives "a page without the toggle". The page still shows both radio buttons, with the missing one disabled. It also says the toggle is by hash, but the code allows None hashes (F2).
- The `FALLBACK` suffix in labels is not documented.
- The CDN scripts (`cesium@1.123.0`) have no SRI hash. This is a minor supply-chain note.

## Requirements check

| Requirement | Result |
|---|---|
| Values come from the log; page says simulation replay | Pass for Cesium. Rerun lacks the explicit line (F8). |
| Keys from env only, never on disk | Pass for disk. Fail for exposure to other web pages (F1). |
| Local server: path traversal / files outside `--out` | Pass (probed). Binds 127.0.0.1. |
| Google `showCreditsOnScreen`, OSM attribution | Pass. The option is set on both the creation call and the tileset. The OSM credit has `showOnScreen = true`. |
| Fixed origin in manifest, distances kept | Pass. The test checks distances to 1e-6 and the up vector. The default origin is wrong for terrain (F3). |
| Toggle only for the same script hash | Fail for null hashes (F2). |
| Rerun content | Pass. The recording builds. Colour thresholds match the docs. Backup caveat in F7. |
| Docs commands match the CLI | Pass. Minor wording in F10, negative-latitude form in F6. |

## Verdict

Do not merge until F1 is fixed. F2 is also a required change, because it breaks a stated requirement and can crash the page. F3 and F4 are strongly recommended. F5 to F10 can follow in a later PR.

Not checked: the page JS in a real browser, the Google/ion tile loading with real keys, and a full tier run.

## Fixes applied (author, 2026-10-24, after this review)

- F1: `--serve` answers the keys only at a random path (`/keys-<token>.js`, new each start) that only the served `index.html` names, with `Cache-Control: no-store`, and refuses any request whose `Host` is not `127.0.0.1:<port>` or `localhost:<port>` (DNS rebinding). `/keys.js` returns 404. Test: `test_keys_are_served_from_the_environment_to_our_page_only`.
- F2: `build` refuses two logs unless both carry a `script_sha256`; the page uses `(script_sha256 || "none")`. Test: `test_cesium_refuses_a_toggle_without_script_hashes`.
- F3: the page warns when a terrain basemap is used with origin height 0; `VIEWERS.md` says to pass the ground height as `H`.
- F4: run info is built with `textContent`; the embedded manifest escapes `<` as `\u003c`. Test: `test_cesium_page_escapes_log_fields`.
- F5: sampled positions hold for one step past the last sample (`forwardExtrapolationType: HOLD`).
- F6: `--origin` rejects non-finite values; help and docs show the `--origin=-12.9,77` form.
- F7: `VIEWERS.md` says backup paths come from the observed state and can jump from the true position under noise.
- F8: the Rerun run panel starts with "Simulation replay, not live data."
- F9: CI installs the `viz` extra.
- F10: `VIEWERS.md` now says the other toggle option is disabled with a single log and documents the `FALLBACK` label. No SRI hash was added to the Cesium CDN scripts (not done).
