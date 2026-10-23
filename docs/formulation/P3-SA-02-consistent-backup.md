# P3-SA-02 — Consistent Backup (CBK): One Backup per Drone, Chosen Jointly

| Field | Value |
|---|---|
| Task | Research lane, backlog BL-03 (with BL-04, BL-08, BL-10, BL-11) |
| Status | Prototype and measurements for review by Salman. Nothing here changes `src/utm_services/safety/`. |
| Date | 2026-10-23 |
| Builds on | [`P3-SA-01-literature.md`](./P3-SA-01-literature.md), [`P2-SA-02-lateral-escape.md`](./P2-SA-02-lateral-escape.md) §4–§5, `docs/results/pr12-final-report.md`, `docs/results/pr12-b1-review-2026-10-23.md` |
| Code | `research/consistent_backup/cbk.py` (filter), `harness.py` (closed-loop runners), `evaluate.py` (cases and table) |
| Results | `research/consistent_backup/results/runs_main_cbk.jsonl`, `table_main_cbk.md`, `rerun_fallbacks.txt`, `scale_main_cbk_cbk-lite.md` |

## 1. Problem

Main's filter builds its backup pair by pair. `_backup_rollout` flies both drones of a pair along an evasive direction chosen for that pair, with relative-position feedback. A drone in two pairs therefore gets two backup trajectories, and they can point in opposite directions. Each pair row is satisfiable on its own. The rows of one drone together may not be: they credit the drone with escapes it cannot fly at the same time (BL-03). The PR #12 latches, the 2-colouring and the escape stage choose signs for these pairwise directions. The B1 re-review showed that sign choices cannot repair the structure: the bent sandwich gives the squeezed drone contradictory directions (HIGH-1, BL-10), the axis normalisation flips at x = ±y (MEDIUM-1, BL-11), and seeds 1177 and 4828 still enter `d_eff` (BL-08).

## 2. Design

The design follows Chen, Singletary and Ames 2021 (CSA, [DOI 10.1109/LCSYS.2020.3000748](https://doi.org/10.1109/LCSYS.2020.3000748)) for the structure and Singletary et al. 2022 (TBC, [arXiv 2207.05220](https://arxiv.org/abs/2207.05220)) for the form of each backup and its switching rule. P3-SA-01 §2.1–§2.2 records what each paper proves and under which assumptions.

### 2.1 One backup per drone

Each drone `i` owns one backup `π_i`. It depends only on the drone's own state `(p_i, v_i)`, never on another drone. Every pair row of drone `i` reads the same trajectory `Φ_i(k)`, `k = 1..K`. This is the CSA structure: "every drone flies its backup action" is then one joint action that satisfies every pair row and every world-box limit at once (CSA Theorem 1; P3-SA-01 §3 item 1).

### 2.2 The backup library

Each backup is a time-varying backup controller in the TBC sense (TBC eq. (9)): a manoeuvre for a fixed time, then a stop.

- Candidate 0 brakes every axis to hover.
- Candidates 1–52 are the 26 non-zero world-frame sign vectors `s ∈ {−1, 0, 1}³`, each flown for 10 or 20 steps (1 s or 2 s). During the manoeuvre, each axis with `s ≠ 0` accelerates at `u = 3.5 m/s²` in the direction of `s`, and each axis with `s = 0` brakes. After the manoeuvre every axis brakes to hover.
- Every rollout step is clipped by main's `_world_accel_limits` (world box) and the speed cap, with the simulator's trapezoid update. A candidate that pushes into a wall is therefore flown as the clipped manoeuvre, and the world box holds along every rollout.
- The horizon is the longest manoeuvre plus the braking time from `v_max` plus the lag allowance: 20 + 43 + 5 = 68 steps. It extends until every rollout hovers, up to main's `_backup_max_steps` (109). Rollouts still moving at the cap are flagged in `backup_rollout_cap_pairs`, as in main.

The directions are in the world frame, not in a line-of-sight frame. There is no axis normalisation, so the flip at x = ±y (BL-11) cannot occur. This is our inference from the own-state structure of CSA, not a result of either paper (P3-SA-01 §3 item 5).

### 2.3 Joint selection

The latch stores, per drone, its current candidate and the number of manoeuvre steps already flown (the TBC clock `τ0`, TBC eqs. (14)–(16)). At each step:

1. **Options.** Drone `i` has 54 options. Option 0 is its current candidate at its current clock. Options 1–53 are every candidate started fresh.
2. **Margins.** For each pair `(i, j)` inside the range gate, the margin of every option pair is `m_ij(o_i, o_j) = min_k |Φ_i^{o_i}(k) − Φ_j^{o_j}(k)| − d_eff`, a 54 × 54 matrix.
3. **Coordinate ascent.** Starting from option 0 for every drone, drones are visited in id order, for up to 3 sweeps. Drone `i` scores each option by its own pairs only: `own_min + 0.01 · own_sum`, with every margin clipped at 3 m, so pairs well clear do not steer the choice. A drone moves to a better option only if its own minimum does not drop. Because pairs without drone `i` are unchanged, the global minimum of the clipped margins never drops below that of the warm start.
4. **Hysteresis.** A drone still inside its manoeuvre keeps it while its own minimum is at least 0 (TBC Algorithm 1: the clock runs on while the manoeuvre is certified). Leaving option 0 needs 0.5 m of own score. Ties go to the lowest option index. This departs from TBC Algorithm 2: the 0.5 m band also applies when option 0 is no longer certified (own minimum below 0), so a certified option that scores less than 0.5 m better is rejected (`cbk.py` `_select`). TBC would switch. This may contribute to the noise regression in §6.2; it is not yet tested.
5. **Latch update.** The chosen candidate's clock advances by one step. When the manoeuvre ends, the latch returns to candidate 0 (brake).

The selection is the centralised counterpart of CSA §IV Algorithm 1. CSA lets each agent choose, in turn, a candidate compatible with those already broadcast by its neighbours. Here one process makes the same choices in id order, so it needs no broadcast. Section 5 covers the decentralised version.

### 2.4 Filter rows and QP

The QP keeps main's form: minimise `|a − a_nom|²` plus an L1 slack with weight `1e4`, inside the world-box acceleration limits. Only the rows change:

- Per-drone sensitivities `J_i(k) = ∂Φ_i(k)/∂a_i(0)`, from central differences of ±0.1 m/s² on the first action of the chosen backup (6 extra rollouts per drone).
- For each pair, rows at the closest-approach step `k*` of the chosen backups and 2 steps either side (BL-04 window):
  `m_kᵀ (J_i(k)(a_i − a_i^b) − J_j(k)(a_j − a_j^b)) ≥ −γ dt · max(d_k − d_eff, 0)`,
  normalised by `(k + 0.5) dt²` as in main.
- **Deviation from main.** Main does not clamp the barrier term: its rhs is `−γ dt (d* − d_eff)` (`filter.py` `_backup_constraints`), so when a backup is already inside `d_eff` main asks the QP to recover. CBK clamps at 0: when the chosen joint backup is not certified, the rows ask only that the drones do no worse than the backup. The clamp is what makes `a = a^b` feasible for every row. How often the chosen backup was not certified was not recorded in the evaluation (`min_margin_m` is computed but not saved by the harness). Here `m_k` is the unit vector between the two backup positions at step `k`, and `a^b` is the backup's first action.
- `a = a^b` satisfies every row with zero slack, by construction. The code asserts this.

Failure handling:

- **Slack above the intervention tolerance.** The rows are linear in `a` and `a^b` satisfies them with zero slack, so slack above the tolerance (1e-4 m/s²) can come only from OSQP's accuracy (eps 1e-5) or from the L1 penalty not being exact. It occurred in 5 runs of the evaluation without a solver error (hard seeds 3614, 3947, 3932, 3146 and one hnoise σ = 1.5 run; 9 takeover steps). These steps are not counted as fallbacks. The drones of the affected rows fly their backup action. The takeover then grows over any row that now mixes a backup with a QP action and is violated, as `_fallback_cluster` does in main.
- **Solver error.** OSQP is tried at rho 0.1 (alpha 1.6, then 1.0), then at rho 10 warm-started at the backup action (§6.2). If all four attempts fail, every drone in a row flies its backup action, and `fallback_used` is set.

Both fallbacks are the certified joint backup itself, not a separate evasive heuristic.

### 2.5 What the design keeps

- **World box:** every rollout step and the QP bounds use main's `_world_accel_limits`.
- **Deterministic output:** the drones are sorted by id, there is no random draw, ties go to the lowest index, and OSQP uses main's settings. The same inputs and latch give the same actions.
- **Reversals under noise (intended, not achieved):** a drone's direction is a world-frame candidate held in the latch. It changes only when the current one is no longer certified, or when another option is better by 0.5 m. The intent was that the band filters noise. The data says it does not at σ = 1.5: CBK has 16 reversals against main's 1 (vertical) and 10 against 0 (horizontal), §6.2.
- **Contract:** same signature and return types as `apply_filter`; `latch` is required (UTM-077). The diagnostics carry an extra attribute `cbk` for the research runner. `FilterConfig.barrier` and `tie_break_mps2` are ignored.

## 3. What CBK changes in each failing case

This section states the design intent. §6.2 has the measured outcome; where they differ, §6.2 wins.

| Case | Cause in main | What CBK changes |
|---|---|---|
| Sandwich (mid, floor, ceiling, vertical) | The squeezed drone has one backup per neighbour; the rows credit it with two escapes. | The squeezed drone has one trajectory, read by both of its rows. The selection picks a joint escape that is consistent for the three drones. The physical bound (about 4.18 m mid, P2-SA-02 §3) still applies: CBK can only approach it. |
| Staged and bent sandwiches (HIGH-1, BL-10) | The colouring orders far outer pairs before inner pairs and fixes the squeezed drone's side from the wrong pair. | There is no per-pair side to colour. A drone has one direction, so a contradictory step cannot occur; the metric does not apply to CBK. Measured: staged and bent 45°/60° better than main, bent 10° and 20° worse (−0.32 and −0.12 m, both above `d_eff`). |
| 45° axis flip (MEDIUM-1, BL-11) | `_escape_axes` normalises the sign by the largest component, which flips at x = ±y. | World-frame candidates with no frame normalisation. |
| Hard seeds 1177 and 4828 (BL-08) | Pairwise backups of 5–6 drones conflict; the QP rows are mutually optimistic. | Consistent joint backup; the selection sees every pair of the drone at once. |
| BL-03 root issue | Pairwise trajectories per drone. | CSA structure (§2.1). |

## 4. Compute cost

Per step, with `N` drones, `P ≤ N(N−1)/2` pairs in range, `O = 54` options and `K ≤ 109` steps:

- Rollouts: `N·O` plus `6N` finite-difference rollouts of `K` steps, vectorised (324 + 36 rollouts at `N = 6`).
- Pair margins: `P·K·O²` distances, as one batched matrix product per pair, about 4.8 million at `N = 6`. This dominates.
- Selection: at most `3·N` scans of an `O × deg(i)` table.
- QP: `3N + 5P` variables (actions and one slack per row) and `5P` rows plus the box, the same size class as main.

Two levers reduce the margin cost:

- **Exact prune (in the prototype, always on).** Let `reach_i` be the largest distance any option of drone `i` travels from `p_i` over the horizon. If `|p_i − p_j| − reach_i − reach_j − d_eff ≥ 3 m`, every option pair of `(i, j)` has a margin of at least 3 m, the clipping value of the selection. The pair's margin matrix is then the constant 3 m without computing it, and the selection is unchanged. Rows are still built for every pair in range. The prune removes no option and changes no selection in exact arithmetic; it only skips pairs that are far apart, so it matters at large `N`. In floating point, a computed margin of a far pair can land about 1e-9 below 3 m, so a tie could in principle resolve differently. The §6.2 evaluation ran without the prune; the four re-runs and §6.3 run with it.
- **Lite library (`cbk-lite`, a trade-off).** 11 candidates instead of 53: brake, the 8 horizontal directions and straight up and down, each flown for 20 steps only (12 options per drone instead of 54). The margin cost falls by about `(54/12)² ≈ 20×`. The lite library removes the diagonal climbs and descents and the 1 s manoeuvres, so it can choose a worse joint backup. Its trade-off in minimum separation is measured in §6.3.

Measured latency is in §6.3.

## 5. Decentralised operation

The structure supports it, following CSA eq. (10) and Theorem 1 and TBC Theorem 2:

- Each drone rolls out its own options from its own state. That part needs no neighbour data.
- The selection becomes CSA Algorithm 1: drones choose in a fixed order (here, by id), each given the candidates and clocks already broadcast by its neighbours. A single sweep of §2.3 is exactly that.
- Each drone solves its own QP and takes half of the `γ dt (d − d_eff)` term of every pair row (CSA eq. (10)). If every drone flies its backup, all rows hold, so the split QPs stay feasible together.
- **Protocol requirement for the architecture decision.** Every drone must broadcast its chosen backup candidate and its manoeuvre clock, every step. Without that broadcast the decentralised version has no guarantee. TBC Theorem 2 states that, with several manoeuvres, every agent must know the others' current manoeuvre. With noisy, differing observations, local recomputation can disagree on the selection; the broadcast removes that disagreement. The ADR that chooses centralised or decentralised operation must include this message (candidate index, clock, timestamp) in the contract.
- Not covered: communication delay and loss, and a drone whose broadcast is missing. A missing broadcast could default to candidate 0 (brake), which every other drone can compute from the observed state. This is untested.

## 6. Results

### 6.1 Acceptance bar for "adopt"

Set by the founder on 2026-10-23. CBK is recommended for adoption only if all of these hold:

1. No case is worse than main on minimum separation by more than 0.05 m.
2. The staged, bent and BL-08 (hard-seed) cases are at least as good as main.
3. 0 fallbacks.
4. Latency at N = 16 is within 2× of main's.

### 6.2 Case table

Source: `research/consistent_backup/results/runs_main_cbk.jsonl` (one line per run, `evaluate.py`, 2 processes). That file holds the first evaluation; the four CBK runs re-run after the solver retry (below) still show `fallbacks: 1` there, and their re-run values are in `rerun_fallbacks.txt`. Grouped cases report the smallest minimum separation over their runs, total time inside `d_eff`, total fallbacks, and the largest reversal count of any one run. "Contradictory steps" counts main's steps where the squeezed drone's two pair escapes point in opposite directions (reviewer metric, cos < −0.7); CBK has one backup per drone, so it has none by construction. `d_eff` = 5.5 m.

**Solver retry.** The first evaluation had 4 CBK fallbacks, all OSQP solver errors, not infeasibility: sandwich floor, ceiling and vertical, and vnoise σ = 1.5 seed 14. OSQP stopped at "solved inaccurate" and then at the iteration limit, with rho 0.1, on QPs where the box is saturated or the rows leave little more than the backup action feasible. `cbk._solve` now retries at rho 10 (main's setting for a saturated box), warm-started at the backup action, after the two existing attempts. The retry runs only where both earlier attempts fail, so runs without a solver error are unchanged. The four affected runs were re-run (`rerun_fallbacks.py`, output in `results/rerun_fallbacks.txt`). All four solve at every step, with the same minimum separation. The table uses the re-run values.

| Case | Runs | Min sep main / CBK (m) | Δ (m) | Inside d_eff main / CBK (s) | Fallbacks main / CBK | Contradictory steps (main) | Reversals max/run main / CBK | Bar |
|---|---|---|---|---|---|---|---|---|
| sandwich mid | 1 | 3.740 / 3.986 | +0.246 | 0.7 / 0.7 | 0 / 0 | 0 | 0 / 0 | ok |
| sandwich floor | 1 | 3.063 / 3.205 | +0.142 | 1.1 / 1.2 | 0 / 0 | 0 | 0 / 0 | ok |
| sandwich ceiling | 1 | 3.201 / 3.205 | +0.004 | 1.2 / 1.2 | 0 / 0 | 0 | 1 / 0 | ok |
| sandwich vertical | 1 | 2.668 / 3.986 | +1.318 | 1.4 / 0.7 | 0 / 0 | 0 | 1 / 0 | ok |
| vertical head-on | 1 | 8.195 / 7.425 | -0.770 | 0.0 / 0.0 | 0 / 0 | 0 | 0 / 0 | fails item 1 |
| ring n=3 r=40 v=10 | 1 | 7.770 / 6.948 | -0.822 | 0.0 / 0.0 | 0 / 0 | 0 | 0 / 0 | fails item 1 |
| ring n=3 r=60 v=15 | 1 | 8.338 / 7.051 | -1.287 | 0.0 / 0.0 | 0 / 0 | 0 | 0 / 0 | fails item 1 |
| ring n=4 r=40 v=10 | 1 | 6.599 / 6.835 | +0.236 | 0.0 / 0.0 | 0 / 0 | 0 | 0 / 0 | ok |
| ring n=4 r=60 v=15 | 1 | 7.147 / 7.312 | +0.165 | 0.0 / 0.0 | 0 / 0 | 0 | 0 / 0 | ok |
| ring n=5 r=40 v=10 | 1 | 7.145 / 5.887 | -1.258 | 0.0 / 0.0 | 0 / 0 | 0 | 0 / 0 | fails item 1 |
| ring n=5 r=60 v=15 | 1 | 6.628 / 6.345 | -0.283 | 0.0 / 0.0 | 0 / 0 | 3 | 0 / 0 | fails item 1 |
| ring n=6 r=40 v=10 | 1 | 6.519 / 6.026 | -0.493 | 0.0 / 0.0 | 0 / 0 | 0 | 0 / 0 | fails item 1 |
| ring n=6 r=60 v=15 | 1 | 6.532 / 6.831 | +0.299 | 0.0 / 0.0 | 0 / 0 | 1 | 0 / 0 | ok |
| simultaneous theta=0 | 1 | 6.432 / 5.879 | -0.553 | 0.0 / 0.0 | 0 / 0 | 0 | 0 / 0 | fails item 1 |
| simultaneous theta=45 | 1 | 6.092 / 5.886 | -0.206 | 0.0 / 0.0 | 0 / 0 | 54 | 0 / 0 | fails item 1 |
| simultaneous theta=225 | 1 | 6.050 / 5.921 | -0.129 | 0.0 / 0.0 | 0 / 0 | 53 | 2 / 0 | fails item 1 |
| staged theta=0 | 1 | 6.308 / 7.257 | +0.949 | 0.0 / 0.0 | 0 / 0 | 0 | 0 / 1 | ok |
| staged theta=45 | 1 | 5.674 / 5.995 | +0.321 | 0.0 / 0.0 | 0 / 0 | 24 | 0 / 1 | ok |
| staged theta=225 | 1 | 5.674 / 5.691 | +0.017 | 0.0 / 0.0 | 0 / 0 | 24 | 0 / 1 | ok |
| bent 10 (15 bearings) | 15 | 6.062 / 5.746 | -0.316 | 0.0 / 0.0 | 0 / 0 | 344 | 0 / 1 | fails item 2 |
| bent 20 (15 bearings) | 15 | 6.088 / 5.966 | -0.122 | 0.0 / 0.0 | 0 / 0 | 314 | 0 / 1 | fails item 2 |
| bent 45 (15 bearings) | 15 | 5.993 / 6.240 | +0.247 | 0.0 / 0.0 | 0 / 0 | 114 | 0 / 1 | ok |
| bent 60 (15 bearings) | 15 | 5.967 / 6.099 | +0.132 | 0.0 / 0.0 | 0 / 0 | 0 | 0 / 1 | ok |
| hard seed 3810 n=2 | 1 | 3.568 / 3.695 | +0.127 | 0.8 / 0.8 | 0 / 0 | 0 | 0 / 0 | ok |
| hard seed 3614 n=6 | 1 | 3.933 / 4.466 | +0.533 | 0.7 / 0.7 | 0 / 0 | 16 | 2 / 0 | ok |
| hard seed 3947 n=6 | 1 | 4.289 / 4.879 | +0.590 | 0.6 / 0.6 | 0 / 0 | 16 | 0 / 0 | ok |
| hard seed 3932 n=5 | 1 | 4.351 / 4.999 | +0.648 | 0.6 / 0.5 | 0 / 0 | 8 | 3 / 1 | ok |
| hard seed 2704 n=6 | 1 | 4.480 / 4.663 | +0.183 | 0.5 / 0.5 | 0 / 0 | 0 | 2 / 0 | ok |
| hard seed 252 n=6 | 1 | 4.726 / 4.930 | +0.204 | 0.5 / 0.4 | 0 / 0 | 14 | 2 / 2 | ok |
| hard seed 1177 n=5 | 1 | 4.765 / 5.533 | +0.768 | 0.5 / 0.0 | 0 / 0 | 0 | 1 / 1 | ok |
| hard seed 3146 n=6 | 1 | 4.832 / 4.877 | +0.045 | 0.3 / 0.3 | 0 / 0 | 26 | 0 / 2 | ok |
| hard seed 370 n=3 | 1 | 4.895 / 5.618 | +0.723 | 0.4 / 0.0 | 0 / 0 | 0 | 0 / 1 | ok |
| hard seed 2876 n=5 | 1 | 5.208 / 5.546 | +0.338 | 0.4 / 0.0 | 0 / 0 | 5 | 1 / 0 | ok |
| hard seed 4828 n=6 | 1 | 5.369 / 5.721 | +0.352 | 0.4 / 0.0 | 0 / 0 | 28 | 1 / 0 | ok |
| hard seed 4239 n=5 | 1 | 5.377 / 5.919 | +0.542 | 0.3 / 0.0 | 0 / 0 | 0 | 0 / 4 | ok |
| hard seed 977 n=6 | 1 | 5.435 / 5.775 | +0.340 | 0.2 / 0.0 | 0 / 0 | 2 | 0 / 0 | ok |
| vnoise sigma=0.0 | 1 | 6.707 / 6.855 | +0.148 | 0.0 / 0.0 | 0 / 0 | 0 | 0 / 0 | ok |
| vnoise sigma=0.5 | 20 | 6.474 / 6.711 | +0.237 | 0.0 / 0.0 | 0 / 0 | 0 | 0 / 0 | ok |
| vnoise sigma=1.5 | 20 | 5.509 / 5.094 | -0.415 | 0.0 / 0.8 | 0 / 0 | 0 | 1 / 4 | fails item 1 |
| hnoise sigma=0.0 | 1 | 7.241 / 6.855 | -0.386 | 0.0 / 0.0 | 0 / 0 | 0 | 0 / 0 | fails item 1 |
| hnoise sigma=0.5 | 20 | 6.992 / 6.609 | -0.383 | 0.0 / 0.0 | 0 / 0 | 0 | 0 / 1 | fails item 1 |
| hnoise sigma=1.5 | 20 | 5.989 / 5.752 | -0.237 | 0.0 / 0.0 | 0 / 0 | 0 | 0 / 3 | fails item 1 |

**Against the acceptance bar (§6.1):**

1. **No case worse than main by more than 0.05 m: fails, 15 cases.** Vertical head-on (−0.77 m), five rings (−0.28 to −1.29 m), the three simultaneous sandwiches (−0.13 to −0.55 m), bent 10° and 20° (−0.32 and −0.12 m), hnoise at σ 0, 0.5 and 1.5 (−0.24 to −0.39 m), and vnoise σ = 1.5 (−0.42 m). The Bar column shows the stricter item 2 for the bent cases. In 14 of the 15, CBK stays above `d_eff` (lowest 5.746 m, bent 10°). Our hypothesis, not verified: a CBK backup is a world-frame manoeuvre at up to 3.5 m/s² per axis, so a diagonal candidate escapes faster than main's line-of-sight escape. The certified set is larger, and the filter lets drones come closer before it intervenes.
2. **Staged, bent and BL-08 cases at least as good as main: fails on bent 10° (−0.32 m) and bent 20° (−0.12 m).** It holds for the three staged cases (+0.02 to +0.95 m), bent 45° and 60°, and all 13 hard seeds (+0.05 to +0.77 m). Six hard seeds that enter `d_eff` on main stay outside it on CBK (1177, 370, 2876, 4828, 4239, 977). Total time inside `d_eff` over all cases is 8.4 s for CBK against 10.6 s for main.
3. **0 fallbacks: holds after the solver retry** (0 for both filters; before the retry, CBK had 4).

**Selection bias in the hard seeds.** The 13 hard seeds were chosen because main enters `d_eff` on them. CBK has not run the 5000-run normal and hard fuzz sweeps, so its own failing seeds are unknown. The hard-seed gains do not show that CBK enters `d_eff` less often than main overall; on every case not chosen from main's failures (rings, simultaneous, hnoise), CBK is lower than main.

**The one safety regression.** Under vnoise σ = 1.5, 2 of 20 CBK runs enter `d_eff` (seed 2 at 5.183 m with 3 reversals, seed 18 at 5.094 m with 2 reversals); main keeps all 20 outside (lowest 5.509 m). The reversals point to candidate switching under noise: the 0.5 m switching band (§2.3) is not enough at σ = 1.5. Per the project rules, a result worse on a safety metric goes to the founder.

**Reversals at σ 0.5 and 1.5** (20 seeds each; sum over runs, runs with at least one reversal, largest count in one run):

| Case | main | CBK |
|---|---|---|
| vnoise σ = 0.5 | 0, 0 runs, max 0 | 0, 0 runs, max 0 |
| vnoise σ = 1.5 | 1, 1 run, max 1 | 16, 9 runs, max 4 |
| hnoise σ = 0.5 | 0, 0 runs, max 0 | 2, 2 runs, max 1 |
| hnoise σ = 1.5 | 0, 0 runs, max 0 | 10, 7 runs, max 3 |

CBK has more reversals than main under noise. Main's pair latch holds the escape frame for the pair; CBK re-scores 54 options per drone every step from noisy states and switches when an option is 0.5 m better. In the staged, bent and most hard-seed cases, CBK shows one or more reversals per run where main often shows none. There the nominal is zero, so the harness's deviation is the action itself, and a CBK backup accelerates and then brakes: the sign change is built into the manoeuvre. These counts are not comparable with main's and are not evidence of chattering. Only the noise cases use the pair metric.

Latencies in `table_main_cbk.md` were measured on a shared CPU and are not timing-grade; §6.3 has the timing run.

### 6.3 Latency at N = 6, 16 and 64

Pending: `scale.py` is a timing run and waits for an idle machine.

## 7. Limits

- The rows are first-order rollout rows at the sample times `k* ± 2`, like main's. CSA imposes the constraint at every sample time on the horizon. There is no inter-sample guarantee (P3-SA-01 §4, BL-04).
- The selection is a heuristic local search, not the exact joint optimum. Its only guarantee is that the global minimum of the clipped margins never drops below the warm start's.
- Disturbance (BL-06), lag (BL-05) and the world-box wind budget (BL-07) are not modelled beyond main's lag allowance in the horizon.
- The margins come from observed states. Under noise, a drone may switch candidates when the band of 0.5 m is not enough.
