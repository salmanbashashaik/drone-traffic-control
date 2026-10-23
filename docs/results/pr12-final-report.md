# PR #12 final verification report (2026-10-23)

## Final state at merge (B1 round)

This section supersedes the numbers below, which describe 489a2a0. Per-seed results are in `docs/results/pr12-b1-2026-10-23.jsonl`; decisions are in `docs/DECISIONS.md` under "2026-10-23 (PR #12, B1 fix)".

- `fuzz.py 5000 backup 100 8`: 0 runs inside d_eff (worst 5.534 m), 0 fallbacks, 0 solver errors, 0 exceptions, 0 rollout cap hits.
- `HARD=1 fuzz.py 5000 backup 100 8`: 13 runs inside d_eff against the bar of 14 (founder-approved, replaces ≤ 21), 0 fallbacks, 0 solver errors, 0 exceptions, 0 rollout cap hits.
- The latch is a required argument of `apply_filter` (UTM-077), so no caller gets the per-step behaviour described at the end of this report.
- Sandwich tests are non-regression floors 0.05 m below the measured values, rounded down to the centimetre (not 0.01 m as below).
- Known limits accepted by the founder: the seeded colouring keeps a squeezed drone on one side only in collinear geometry (bent sandwich: contradictory latches, worst 5.993 m, never inside d_eff; BL-10, superseded by BL-03), and the 45° staged sandwich dropped from 6.169 m to 5.751 m (BL-11). LOW findings of the re-review are BL-12. Review report: `docs/results/pr12-b1-review-2026-10-23.md`.
- No full run after the re-review (founder decision).

## Summary at 489a2a0

1. **Acceptance bar met.** Hard fuzz 5000 has 14 runs inside d_eff (bar ≤ 21; same seed list as v2), 0 fallbacks, 0 solver errors, 0 exceptions. Fuzz 5000 has 0 runs inside d_eff and 0 fallbacks, solver errors or exceptions. The head-on and crossing tests are unchanged against HEAD.
2. **Full suite:** 252 passed, 1 skipped, 1 xfailed, 1 failed. The failure was the strict xfail on `keeps_one_sideways_direction[1.5-braking]` passing (XPASS). Per the founder's rule that xfail was removed; the reversal tests then gave 5 passed, 1 xfailed (backup σ 1.5 m stays a strict xfail).
3. **`probe.py`:** fixed per the founder (valid margins 0.5 and 5 m stay in the sweep; 50 and 90 m are asserted to raise `ValueError`) and re-run: rc=0.
4. **Invariance:** `invariance.py` (seed 7, 3000 trials) at 883b9c2 gives 1634/216640 violating steps, worst 0.26 m, 1/2708 pairs entering d_eff; the current tree gives 3659/216640, worst 2.06 m, 3/2708. Cause: the script's own pre-latch h model and its missing latch, not the filter (see below). Documented in P2-SA-02 §5.4.
5. **Hard-fuzz seed 3810** (2 drones, 3.568 m) is not recoverable (`recov.py`: best joint 3.735 m), so it does not block the merge.
6. **Known, outside the bar (founder decision):** `squeeze2.py` constant-wind floor rows leave the world box by 0.228 m (backup) and 0.099 m (braking, with 1 fallback and 1 solver error). The world barrier has no wind budget (BL-07, disturbance work BL-06).
7. **Also known:** `probe.py` wall rows that start at 15 m/s on or near a wall leave the box (up to 1.48 m); they are the same or smaller than at 883b9c2 (for example 1.188 m against 1.499 m for the z = 45 m wind row). The pair rows with one drone hovering 1 m above the floor or below the ceiling leave the box by 0.048 m.
8. **Backlog:** BL-05 (actuator lag in the backup rows and world barrier) added; BL-03 to BL-06 assigned to Salman.
9. **Process:** `CLAUDE.md` now has the Decision authority section; decisions are logged in `docs/DECISIONS.md`.
10. **Per-seed results:** `docs/results/pr12-final-2026-10-23.jsonl` (fuzz 5000 and hard fuzz 5000).

## Code state (branch `aman/sprint1-safety-core`)

- v2 latch plus the turn-right rule: at conflict entry the 2-colouring orients the latch only in clusters of three or more drones; a two-drone conflict with no clear miss keeps `ẑ × n` (`_update_latches`, `_escape_signs(..., colouring=False)`).
- Sandwich tests are non-regression floors 0.01 m below the measured values, with the comment "Unrecoverable by any controller (best achievable 4.18-4.45 m). Non-regression floor, not a safety claim. See BL-06."
- Test fakes updated for the new `frames` and `latch` kwargs.
- Docs updated: `INTERFACES.md`, `docs/formulation/P2-SA-02-lateral-escape.md` §5.1 and §5.4, `TASKS.md` (BL-05, BL-06, BL-08, owners).

## Results

| Check | Result | Bar |
|---|---|---|
| `fuzz.py 5000 backup 100 8` | 0 inside d_eff, 0 fallbacks, 0 solver errors, 0 exceptions; max rollout 66, 0 cap hits | — |
| `HARD=1 fuzz.py 5000 backup 100 8` | 14 inside d_eff (10 below 5 m), 0 fallbacks, 0 solver errors, 0 exceptions | ≤ 21 inside, 0 / 0 / 0 |
| Head-on, crossing tests | unchanged vs HEAD; QP head-on passes | unchanged |
| Full suite | 252 passed, 1 skipped, 1 xfailed, 1 XPASS(strict) → xfail removed; affected test 5 passed, 1 xfailed | pass |
| `probe.py` | rc=0 | rc=0 |
| All other verify scripts | rc=0 | rc=0 |

Hard fuzz runs inside d_eff (seed, n, min_sep m): 3810 2 3.568; 3614 6 3.933; 3947 6 4.289; 3932 5 4.351; 3146 6 4.381; 2704 6 4.480; 252 6 4.726; 1177 5 4.765; 370 3 4.832; 4582 5 4.908; 2876 5 5.208; 4828 6 5.369; 4239 5 5.377; 977 6 5.435.

Sandwiches (`targeted.py` and exact runs, backup / braking): mid 3.740 / 3.740 m, floor 3.063 / 3.061 m, ceiling 3.201 / 3.201 m, vertical 2.668 / 2.744 m; all 0 fallbacks, 0 aborts. Fallback-only sandwich 2.98 m.

Noise (20 seeds per row; worst min_sep m, runs below d_eff, max reversals per run):

| Row | σ 0 | σ 0.5 | σ 1.5 |
|---|---|---|---|
| vnoise backup | 6.707, 0, 0 | 6.474, 0, 0 | 5.509, 0, 1 |
| vnoise braking | 10.858, 0, 0 | 10.374, 0, 0 | 9.970, 0, 0 |
| hnoise backup | 7.241, 0, 0 | 6.992, 0, 0 | 5.989, 0, 0 |
| hnoise braking | 10.858, 0, 0 | 10.267, 0, 0 | 9.408, 0, 0 |

Other scripts: `vertical.py` backup 8.195 m, braking 11.423 m, 0 fallbacks; `floor.py` 0 aborts at z0 30.5/35/40; rings n=3–6 all above d_eff, 0 fallbacks; `horizon.py` 0 of 4000 rows changed; `inv2.py 3000` 3632 of 215200 violating steps (worst 1.75 m), 1/2690 entering d_eff (same cause as `invariance.py`); `squeeze2.py` all non-wind rows above d_eff with 0 fallbacks, wind rows as in summary item 6.

## Invariance comparison

The first PR #12 review measured 639/86036 slack-free steps (0.74 %) violating `h_next ≥ (1 − γ dt) h`, worst 0.12 m, 0 of 1086 entering d_eff, on an older filter and a smaller sample. Same script, seed and sample now:

| `invariance.py`, seed 7, 3000 trials | Violating steps | Worst gap | Entered d_eff |
|---|---|---|---|
| 883b9c2 (before the latch work) | 1634/216640 (0.75 %) | 0.2618 m | 1/2708 (trial 1260, 5.499 m) |
| Current tree | 3659/216640 (1.69 %) | 2.0591 m | 3/2708 (trials 207, 1260, 1872: 5.477, 5.498, 5.499 m) |

Ablation on the current tree, first 600 trials (546 pairs, 43680 slack-free steps):

| Latch | h measured by | Violating steps | Worst gap | Entered d_eff |
|---|---|---|---|---|
| none (fresh every step, as the script) | script (pre-latch escape) | 747 | 1.568 m | 1 |
| none | filter rollout, frame the QP used | 409 | 0.127 m | 1 |
| persistent (as the runner) | script | 458 | 0.611 m | 0 |
| persistent | filter rollout | 414 | 0.137 m | 0 |

Trials 207, 1260 and 1872 re-run with a persistent latch: 0 of 3 enter d_eff. The larger worst gap is a model mismatch (the latch deliberately flies a different escape than the script's copy), and the entries come from calling `apply_filter` without a latch. The production runner always passes one. Callers that do not pass a latch get the weaker per-step behaviour. The ablation script is `scripts/verify/inv_ablate.py`; the existing verify scripts are unchanged.
