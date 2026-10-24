# Browser 3D viewer, building scenery, six-drone and swap suites on top of the MAPPO policy artifacts (PR #20, pass 2)

Of the 11 findings from the first review, 10 are fixed or explicitly accepted and one (no tests) remains open. The six-drone, swap-mirror and city-clearance tables recompute exactly from the committed rows, and the first 4 six and 3 swap episodes re-run at HEAD with identical results. The problems are wording and omitted numbers, not wrong tables: a safety-filter warning is documented as "normal", the headline building clearance (21.8 m) is from the one suite where the filter deflects least, and the viewer/PyBullet outputs contain a few vacuous or misleading readouts.

Watch for: (1) QUICKSTART tells users that `filter: barrier violated` lines are normal; in `filter.py` that is a WARNING emitted when the HOCBF slack is positive, i.e. the barrier could not be satisfied for a pair (confirmed); (2) building clearance is headlined as 21.8 m while the same `head-on-40` scenario gives 10.6 m (point mass, disclosed), 6.8 m in PyBullet (not disclosed) and 4.2 m with another city seed, because the filter deflects drones up to 55 m against a 35 m layout margin (confirmed); (3) `--pybullet` without `--snapshots`/`--gui` never builds the buildings in the physics world but still prints `building contacts (physics) 0` (confirmed); (4) "within 0.35 m of the shielded reference" leaves out the +0.43 m T2-ideal comparison that sits in the committed summary (confirmed); (5) the QUICKSTART "tested in a fresh venv" statement in DECISIONS is not scoped to sections 2, 3, 5, 6 (confirmed).

**Verdict**: NEEDS_CHANGES

## High-level view

Prior findings: seed-reuse wording, the confounded shield-in-loop comparison (now a full 300-iteration ablation with rows), T2 metadata, dev-seed rows, same-seed T1-vs-T2 table, T2 manifests, plot labelling, safety wording, numeric normalisation fingerprint and `weights_file` check, and resume-log duplication are all fixed and consistent with the committed files. Finding 10 (training-loop quirks) is documented under Known gaps and accepted. Finding 11 (no tests) is still open and now covers much more untested code (scenario generators, payload builder, spec parser); the author declined it as a standing instruction.

The building story is honest in intent ("scenery, not avoided") but the number carrying it is optimistic. The layout keeps buildings 35 m from straight routes; the filter's lateral escape moves drones much further than that in head-on cases, so clearance is luck of the layout.

In the viewer payload, aborted drones are labelled "arrived", the last frame stops one step before arrival, and a drone that completed is still drawn although the simulator has removed it from separation checks.

<details>
<summary>Issues (17)</summary>

1. **Filter warning documented as normal** (confirmed, MEDIUM) — `QUICKSTART.md:60,173` call `filter:` log lines and `barrier violated` "normal ... when it is working on a conflict". `filter.py:1421-1425` logs it as a WARNING when pair slack > tolerance (barrier violated). Reword: it means the filter could not satisfy its constraint for that pair at that step; do not tell users to ignore it.
2. **Building clearance headline omits worse runs** (confirmed, MEDIUM) — `MARL_TRAINING.md:34`, `DECISIONS.md:50` quote 21.8 m (50 heldout scenarios) and 10.6 m (point-mass `head-on-40`). PyBullet `head-on-40` shielded gives 6.8 m (reproduced, same doc's PyBullet section) and other city seeds give 4.2 m (head-on-40), 7.6 m (crossing-90), 7.9 m (sandwich-3) in my probes; drones deviate 53-55 m off route there. Report the shipped-scenario and PyBullet minima next to 21.8 m and say the 35 m margin is smaller than the filter's deflection.
3. **Vacuous PyBullet building-contact count** (confirmed, MEDIUM) — `pybullet_scene.py:68,113` builds buildings only in `_init_view`, which runs only for `--gui` or `--snapshots>0`; `replay.py:277` prints `building contacts (physics) 0` for any `--pybullet` run. Print "n/a (no scene)" when the scene was not built. Contacts are also sampled once per 100 ms control step (`:82`), not per physics substep, so brief contacts can be missed (likely).
4. **"Within 0.35 m" bound is selective** (confirmed, LOW) — `MARL_TRAINING.md:163` lists four deltas; the T2-ideal held-out shielded learned 8.29 m vs shielded reference 7.86 m (+0.43 m, in `fresh-ideal.summary.md`) is outside the bound. Include it or reword to "within about 0.4 m".
5. **"Worst of 50" is by violation pair-steps, not separation** (confirmed, LOW) — `QUICKSTART.md:94`, `MARL_TRAINING.md:124`, `DECISIONS.md` say `heldout:18000044:6` is the worst unshielded case; its 3.83 m is not the suite minimum (2.80 m at seed 18000010). Say "most violation pair-steps (6)".
6. **Fresh-seed range 24,000,000+ claim** (confirmed, LOW) — `MARL_TRAINING.md:128` says seeds 24,000,000+ were unused, then says probe seeds 24,900,000+ were used; the probe range lies inside the first. Say "24,000,000-24,000,049 unused; 24,900,000-24,900,019 probes".
7. **"Fresh clone" test statement not scoped** (confirmed, LOW) — `DECISIONS.md:54` says QUICKSTART commands were run in a brand-new venv; QUICKSTART itself says section 4 was not. Scope it. I could not verify the venv claim itself.
8. **Quickstart "flying through a city"** (confirmed, LOW) — `QUICKSTART.md:3` implies urban flight; the scenery caveat appears 60 lines later. Reword to "over scenery buildings the controller cannot see". The viewer HUD ("nearest building to any drone", `template.html`) and the replay.png building-clearance panel read like a monitored quantity; label them "measured, not controlled".
9. **Viewer labels aborted drones as arrived** (confirmed, LOW) — `viewer3d.py:40` treats any drone absent from `rec["states"]` as arrived; `env.states()` lists only ACTIVE drones, so aborted drones (one occurred in the T1-under-T2 runs) also get the green "arrived" chip. Use `infos`/summary status or an abort chip.
10. **Viewer last frame stops one step early** (confirmed, LOW) — records hold the pre-step state, so arrived drones are held 2.0-2.5 m from the goal (goal radius 2.0 m) and one drone (d2 in all three probed replays) never shows "arrived" although the summary says 6/6. Append the final state or note it.
11. **Completed drones remain drawn but are not checked** (confirmed, LOW) — the simulator drops completed drones from separation checks; the viewer keeps drawing them hovering with no halo, so another drone passing through one is invisible to the metrics. Hide them or note it.
12. **Follow-camera tween fights the follow lock** (confirmed from code, LOW) — `template.html` follow code adds the drone delta every frame (`:380`) while the 0.7 s tween overwrites camera and target with a pose computed at click time; at 4x speed the camera snaps by tens of metres when the tween ends. Cosmetic.
13. **Halo size is not d_safe** (confirmed, LOW) — `template.html:361` scales the halo with the drone-size slider (~34 m diameter by default) while its colour uses the 5 m threshold; touching halos do not mean a violation. Say so in the help text. Also "20x real size" (`QUICKSTART.md:68`, `MARL_TRAINING.md:32`) conflicts with `pybullet_scene.py` (22 m drawn vs a 9 cm Crazyflie, about 240x).
14. **Spec parsing has no validation** (confirmed, LOW) — `replay.py:48` accepts `swap` gaps of 0 or negative (identical start/goal, gap 3 m starts inside d_safe); `random:60`/`swap:..:60` fail with a bare `RuntimeError` although QUICKSTART:42 says "any number of drones" (works to at least 40). Validate gap >= ~10 m and bound n.
15. **`--seed-base` silently moves every suite** (confirmed, LOW) — `evaluate.py:148-149` help says it applies to heldout/dense16, but `six` and `swap` also use it. The MARL_TRAINING example with `--suite six --seed-base 12000000` evaluates six on 12,000,000+, not the 18,000,000+ in the six table. Fix the help text or the example. The manifests for `six`/`swap` also record the artifact's training commit (`6f4c636`), which predates those suites, and `seed_base: null`.
16. **QUICKSTART accuracy nits** (confirmed, LOW) — section 5 uses `python -m ...city_eval` although `utm-city-check` is registered; eval time is overstated (about 5.5 s per episode here, so 50 six episodes is under 5 minutes, not 10-15); "3.11 or newer" is untested above 3.12 and `torch==2.5.1` has no Intel-macOS wheels (likely; Apple silicon only is stated); `gym-pybullet-drones` is pinned by a 7-character SHA from GitHub, use the full SHA. The `.venv` in the working tree has no `utm-replay` script (installed before pyproject changed); reinstall with `pip install -e`.
17. **Vendored three.js provenance and tests** (possible/confirmed, LOW) — `LICENSE-three.txt` says "unmodified" but records no source URL or SHA-256 (I did not fetch upstream to compare); the MIT text ships in the wheel (`viewer/*` package-data verified by building a wheel), while generated HTML carries only the SPDX header. Add the URL and hash. Still no tests for `swap_scenario`, `build_payload`, `load_scenario`, `generate_city` (prior finding 11, still open).

</details>

<details>
<summary>Details</summary>

### Status of the 11 findings from the first review

Fixed and verified against files: 1 (seed reuse wording at `MARL_TRAINING.md:68`, `DECISIONS.md:41`, `scenarios.py:15`), 2 (a full 300-iteration shield-in-loop artifact, log and rows; recomputed 8 vs 8 violation pair-steps in 3 vs 2 episodes, min 3.35 vs 1.93 m, 213 vs 204 interventions, completion 100% vs 99.33%), 3 (`mappo.py:369-373` derives the limitation from the profile; shipped it220 metadata now says T2 nominal), 4 (dev-seed rows committed; 77.78% and 84.44% completion match "78% / 84%"; sandwich-3 13 violation pair-steps for both shielded variants), 5 (same-seed T1-under-T2 table recomputes: 75.2/77.2/83.2/83.9% completion, min 2.59/7.79/2.53/6.97 m), 6 (manifests for both T2 fresh runs), 7 (plot paragraph says worst case), 8 (safety phrases replaced; the T2-realistic dense16 shielded episode inside d_eff is stated), 9 (`learned.py:99-105` fingerprint hashes the numeric constants; `weights_file` must be `actor.npz` at `:231`), 10 (accepted; resume now drops log rows at or past the checkpoint iteration, `mappo.py:405-407`, and the stored `it` is the next iteration so the cut is right). 11 remains open by author decision.

New fresh-seed use not listed in the "consulted more than once" paragraph: `city_eval.py` defaults to seed base 12,000,000, and the plots and PyBullet check use 12000008. Add them to the list.

### Numbers vs committed files

Six-drone table: reference 61 violation pair-steps in 24 episodes (min 0.94 m), T1 unshielded 13/4/2.80, T1 shielded 0/0/7.81 with 486 interventions, T2 unshielded 6/3/0.80, T2 shielded 0/0/8.00 with 558, shielded reference 0/0/8.04 with 2036. All exact. Swap-mirror: 338/50, 344/50 (min 0.33), 0 (min 6.05, 2222 interventions), 0 (min 5.81, 3185), arrived 300/300 each. Exact. City clearance: 21.77 m minimum, 30.67 5th percentile, 39.22 median, 0 episodes inside, identical for both variants to the third decimal (the filter changes the track by under a millimetre in that suite, which fits its small deflections there). All exact. QUICKSTART's swap example reproduces: unshielded d0/d1 1.2/0.7 m off route, min 1.70 m, 7 violation pair-steps; shielded filter acts on d0 and d1 only, 19 steps each (0.9-2.7 s), 5.2/2.4 m off route, min 6.68 m, 38 interventions. PyBullet claims reproduce: `heldout:12000008:4` 4/4, 8.38 m, 46 interventions; `head-on-40` 2/2, 8.93 m.

"Violation in 50/50 swap episodes is the swapping pair" is likely but not shown by the rows (they carry no pair id); the unshielded minimum separations (0.05-1.8 m) are consistent with it.

### Building scenery and what the clearance number means

`generate_city` places boxes at least 35 m (3-D, to samples every 2 m) from each straight start-goal segment. Because the 50-scenario check uses the heldout family where drones deviate at most tens of metres, the 0/50 "inside a building" and 21.8 m minimum follow from the construction. Outside that family the margin is smaller than the deflection: PyBullet `head-on-40` reaches 6.8 m with 53-55 m of deviation; point-mass `head-on-40` with city seeds 1-7 reaches 4.2 m; `crossing-90` 7.6 m; `sandwich-3` 7.9 m. No track entered a building in my probes (25 swap and 25 six episodes: minimum 29.3 m and 21.2 m; shipped scenarios x 8 city seeds), but nothing in the controller prevents it, and a different seed or density would. The doc already says "a scenario where a building sits in the way is not handled"; the numbers should say how close that is.

### PyBullet scene

`SceneEnv` bypasses `PyBulletDroneEnv.step` (drawing and sleeping are re-implemented) and re-implements `_check_collisions` with a retry wrapper. The override matches the base logic today, but any change to the base collision accounting will silently diverge; a comment or a hook in the base would be safer. Buildings exist in the physics world only when a scene is requested (GUI or snapshots), so the contact counter is vacuous otherwise.

### Viewer payload and JS

Payload minimum separation per frame equals the summary minimum, the `near` flags equal the violation pair-step count (7 and 6 in the two unshielded probes), and filter flags match the interventions. The weaknesses are the arrived/aborted conflation, the missing final state and the hold-last-position of completed drones listed above. `tr.innerHTML` interpolates `agent_id` (any non-blank string in `ScenarioConfig`) and the title replaces only `<`; a hostile scenario JSON passed to `utm-replay` could inject markup into a local page (low; the user supplies the file). I could not run the page in headless Chrome here (the run hung), so interaction claims (orbit, wheel, follow) are checked by reading the code only; the `#selftest` hook and URL-hash parameters are undocumented shipped surface.

### Swap generator

Checked over 70 seeds (50 suite, 20 probe): mirroring exact, pair distance equals the gap, all points inside the world box (300 seeds); observed minima among the extras are 31.3 m start-to-goal, 30.9 m goal-goal and 60.02 m trips. The 25 m pair spacing and trip are below the 30 m start spacing and 60 m trip length of the training family, so the scenario is outside the training distribution by construction (stated in the docs). Seeds: swap 24,000,000-24,000,049, probes 24,900,000-24,900,019, `swap:random` 25,000,000+, `random:` 20,000,000+, six 18,000,000+, train 100,000,000+, validation 5,000,000+, test 9,000,000+; no overlaps.

### Wording

Remaining softness: the filter-warning line (issue 1), "flying through a city", the `avoidance:` label printed beside building clearance in `replay.py:218` (it measures drone deviation from the straight route, not obstacle avoidance), and the figure name `viewer-3d-swap-avoidance.jpg`.

### Not checked

Fresh-clone venv install on macOS, Linux and Windows; the PyBullet install workarounds (section 4) beyond running the existing venv's PyBullet; the T2 artifact retrain; the three.js bytes against upstream.

</details>

<details>
<summary>File map</summary>

- `src/utm_services/training/viewer3d.py`, `viewer/template.html`, `viewer/three.min.js`, `viewer/OrbitControls.js`, `viewer/LICENSE-three.txt` — standalone 3D replay page and vendored r128
- `src/utm_services/training/city.py`, `city_eval.py` — scenery buildings and clearance measurement
- `src/utm_services/training/pybullet_scene.py` — PyBullet scene subclass with buildings and snapshots
- `src/utm_services/training/replay.py` — `swap:` and `random:` specs, 3D export, avoidance summary
- `src/utm_services/training/scenarios.py`, `evaluate.py` — `swap_scenario`, `six` and `swap` suites
- `docs/training/QUICKSTART.md`, `MARL_TRAINING.md`, `docs/DECISIONS.md`, `README.md` — docs and results text
- `docs/results/training/*` — six-drone, swap-mirror, city-clearance, dev-seed and ablation rows
- `pyproject.toml` — `utm-city-check` script, `viewer/*` package-data

Full diff: `git diff origin/main...HEAD` on `salman/marl-safety` (HEAD 74445dd).

</details>
