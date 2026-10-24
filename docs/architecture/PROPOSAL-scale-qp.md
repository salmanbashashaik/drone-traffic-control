# Proposal: a solver-independent QP so the filter can scale (needs the founder)

Status: **proposed, not built.** Written by the scale lane on perf/scale (2026-10-24) because the N = 64 step-time target needs a change that the exactness rule does not allow. Measurements: `docs/results/scale-2026-10-24.md`.

## The problem

Target: p95 step under 100 ms at N = 64 on a dense encounter, with the per-seed results unchanged (`compare_results.py --tolerance 1e-4` on the full tier). After the bit-exact changes of perf/scale, N = 64 dense (groups of 8) is p50 196 / p95 338 ms. Of the mean, the joint OSQP solve is 162 ms: every pair of the fleet (2016 at N = 64) is a row of one QP, and that solve alone is above the budget.

Every way to make that QP smaller changes the results by more than 1e-4 m:

- **Per-cluster QPs (built and rejected, change C).** Dropping rows that every acceleration in the box meets with zero slack, and solving each connected component separately, has the same optimum as the joint QP. Mathematically the solutions are equal; OSQP's are not. On the quick tier 118 of 413 rows moved min_sep by more than 1e-4 m (max 0.163 m; 52 worse, 66 better), with all integer fields and the d_eff seeds unchanged. It was fast: N = 64 dense p50 70 / p95 170 ms, city p95 44 ms, N = 128 dense p95 404 ms.
- **A braking-distance range gate** (derivation below) drops rows from the joint QP, which is the same kind of change; at N = 64 dense it drops nothing anyway.
- **Warm starts and factorisation reuse** change OSQP's iterates (warm start) or save only the setup, 5 ms of 338 (reuse).

The reason is solver accuracy, not the decomposition. OSQP stops at eps 1e-5 and then "polishes": it guesses the active constraints from the ADMM iterate and solves that equality system (Stellato et al., OSQP, *Math. Prog. Comp.* 12(4), 2020). When a coordinate is close to a box bound, the guess can snap it onto the bound. In a perf/scale test a drone whose nominal x acceleration was -3.4849 m/s² (box -3.5) got -3.5 from its 2-drone QP and -3.4849 from the joint QP, although no row bound it in either: a 0.015 m/s² difference from the batching alone. The comment in `_solve_hocbf_qp` already notes that without polish pass-through steps are about 0.05 m/s² off nominal. So the current results contain solver errors of order 1e-2 m/s², and those errors depend on which other drones share the QP. Any change to the problem's composition moves them.

The optimum itself is unique in the accelerations (the objective is 2 I in a, strictly convex), so a solver that returns the optimum to 1e-9 would give the same accelerations however the problem is batched.

## Options

1. **Make the solution solver-independent, regenerate the reference once, then decompose (recommended).** Solve each QP to KKT accuracy: OSQP with eps 1e-9 and a KKT check of the polished solution (falling back to the unpolished one, or a tighter re-solve, when it is worse), or an exact active-set solver for the small per-cluster QPs (qpOASES, DAQP, or proxsuite's dense solver; all handle the 3c + p variable problems of a cluster of c drones in microseconds to milliseconds). This changes the per-seed results once. That is a founder decision: the change is not round-off, and some seeds get worse while others get better. After that, per-cluster QPs, the range gate, warm starts and decentralised variants all reproduce the new reference to solver round-off, so every later scale change can be held to a 1e-6 m rule. Expected N = 64 dense: per-cluster solves were 2.7 ms mean in C, but the per-problem scipy build cost 26-38 ms (8-13 small problems per step). Building the block matrices directly, plus the measured latch (14 ms), rollout (29 ms on 4 threads) and other (8 ms) costs, puts the mean near 60 ms. That is an estimate, not a measurement; the p95 depends on steps with many entering pairs.
2. **Accept C's results as the new reference without changing the solver.** Fastest to ship (C is written and reverted in fae327c). But the reference would again hold batching-dependent solver errors, so the next scale change would hit the same wall.
3. **Decentralised per-drone QPs** (each drone keeps its share of each pair constraint, as in Wang, Ames and Egerstedt, "Safety barrier certificates for collisions-free multirobot systems", *IEEE T-RO* 33(3), 2017). Scales to any N with a fixed per-drone cost, but changes the safety argument: the joint-solve assumption A9 of the formulation note and the pairwise backup of BL-03. It needs an ADR and a contract change. Not recommended before BL-03 is settled.

None of the three makes a single converging cluster of 64 drones fit in 100 ms (main 2021 ms, C 1571 ms p95). That state is not recoverable (min sep 0.9 m on main, 24 of 50 steps solver errors and fallback), and its cost is the solver hitting `max_iter` twice per step. A step-time bound there needs an iteration cap with a defined fallback, which is a safety-behaviour decision of its own.

## Range gate derivation (for option 1)

A backup row coef . (a_i - a_j) + s >= rhs, with |coef| = 1, rhs = m . a_first - gamma (d* - d_eff) / ((k* + 0.5) dt), can be dropped when every acceleration in the box meets it with s = 0. Since every component of a is in [-u, u], m . (a_i - a_j) >= -2 sqrt(3) u and m . a_first <= 2 sqrt(3) u, so it suffices that gamma (d* - d_eff) >= 4 sqrt(3) u (k* + 0.5) dt. Each drone moves at most v_max dt per rollout step: the speed cap projects the velocity onto the v_max ball and the trapezoid update averages two capped velocities. So d* >= d0 - 2 v_max (k* + 1) dt. With k* <= K - 1 = 108 (`_backup_max_steps`), the worst case over k* gives

    d0 >= d_eff + max over k of [2 v_max (k + 1) dt + 4 sqrt(3) u (k + 0.5) dt / gamma] = 595.6 m

at the defaults (u = 3.5 m/s², v_max = 15 m/s, dt = 0.1 s, gamma = 1, d_eff = 5.5 m). Beyond that distance a pair needs neither its rollout nor its row. Measured on perf/scale with `range_gate_m = 596`: actions bit-identical to main at N = 64 and N = 128 city (the dropped rows happened not to move OSQP there), N = 128 city p95 987 → 728 ms, and no change at N = 64 dense (no pair is 596 m apart). It is not part of perf/scale for three reasons. It does nothing at the target. It changes `n_pairs_in_qp`, `backup_rollout_steps` and the episode `p99_backup_rollout_steps`. And because it removes rows from OSQP's problem, the fuzz tier (all pairs within about 100 m) cannot check it. Under option 1 it becomes exact by construction.

## Decision needed

Which option, and whether a one-time change of the per-seed reference (with the per-seed better/worse table, as for PR #12) is acceptable.
