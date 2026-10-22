# P2-SA-02 — Lateral Escape: Is the Braking-Only Barrier Too Conservative?

| Field | Value |
|---|---|
| Task | Sprint 1 PR B, item 4 |
| Status | Reviewed. Decision recorded in §4.3: backup-set CBF, prototyped in PR B2 (§5) and made the default barrier (contract v1.3). |
| Date | 2026-10-21 |
| Builds on | [`P0-SA-01-formulation.md`](./P0-SA-01-formulation.md) §5–§8 |

This note answers one question: does the braking-distance barrier in `src/utm_services/safety/filter.py` demand much more separation than the drones actually need? It then proposes a replacement barrier for review. It also records two results from PR B that the proposal depends on: why the old barrier blew up inside `d_eff`, and what the three-drone sandwich can and cannot achieve.

All numbers use the v1 contract: per-axis acceleration bound `u = 3.5 m/s²`, `d_safe = 5 m`, `margin = 0.5 m`, so `d_eff = 5.5 m`, and a 100 ms step.

---

## 1. Why the old barrier blew up inside `d_eff`

The barrier from P0-SA-01 §6 is

    h = sqrt(4u (d − d_eff)) + ρ,       ρ = n · v_rel (negative when closing)

and the QP row is the HOCBF condition `ḣ + γ h ≥ 0`. Differentiating the square root gives

    d/dt sqrt(4u s) = 2u ṡ / sqrt(4u s) = 2u ρ / sqrt(4u s),   s = d − d_eff.

The coefficient `2u / sqrt(4u s)` goes to infinity as `s → 0`. For `s < 0` the root is undefined, so the code clamped it at `ε = 1e-3`. Inside `d_eff` the row's right-hand side therefore contained the term

    −2u ρ / ε = −7000 ρ   [m/s²].

At a closing speed of 10 m/s that is 7 × 10⁴ m/s²; at 30 m/s it is 2.1 × 10⁵ m/s². The decision variable on the left is a relative acceleration along `n`, which the box bounds limit to at most `2u√3 ≈ 12 m/s²`. The slack variable has to cover the difference, so it took values of 10³ to 10⁵ m/s². Those numbers measured the clamp, not the danger: a pair 0.1 m inside `d_eff` and a pair 3 m inside reported similar slack.

**Fix (PR B).** The barrier now uses an offset square root with a linear extension:

    B(s) = sqrt(4u s + c²) − c        for s ≥ 0
    B(s) = (2u / c) s                 for s < 0
    h    = B(s) + ρ

with `c = barrier_offset_mps = 1 m/s`. B is continuous and has a continuous first derivative at `s = 0`, and its slope never exceeds `2u / c = 7 s⁻¹`. The right-hand side is therefore bounded everywhere by `γ|h| + 7|ρ|` plus the bounded tangential term. The cost is extra standoff: solving `B(s) = |ρ|` gives `s = (ρ² + 2c|ρ|) / 4u`, which is `c|ρ| / 2u` more than before. At 30 m/s closing that is 4.3 m (74.1 m instead of 69.8 m). The slack metric is replaced by violation depth `d_eff − d` in metres and time inside `d_eff` in seconds, per pair and per episode.

---

## 2. Is braking-only too conservative? Head-on at 30 m/s closing

Two drones fly head-on, each at 15 m/s. The question is the smallest initial gap `D` from which some admissible control keeps them at least `d_eff` apart at every instant.

**Braking only.** The barrier assumes the pair can only brake along the line of sight. Each drone brakes at `u`, so the relative deceleration is `2u = 7 m/s²` and the stopping distance is `v² / (2 · 2u) = 900 / 14 = 64.3 m`. The barrier therefore requires

    D ≥ d_eff + v² / 4u = 69.8 m      (74.1 m with the offset c = 1 m/s).

**Lateral escape.** The drones may instead split sideways. With opposite lateral accelerations on one axis the relative lateral acceleration is `2u = 7 m/s²`; using both y and z it is `2u√2 = 9.9 m/s²` in magnitude. Box bounds are per axis, so braking along x and splitting in y and z can be used at the same time at full strength. I computed the minimum `D` by bisection on the closed-form trajectories (relative x: `D − 30t + ½ a_x t²`, held at its minimum once the closing speed reaches zero; lateral offset `½ a_lat t²`):

| Strategy | Required initial gap `D` |
|---|---|
| Brake only (current barrier, c = 0) | 69.8 m |
| Brake only (current barrier, c = 1 m/s) | 74.1 m |
| Lateral split in y only, no braking | 38.4 m |
| Lateral split in y and z, no braking | 32.5 m |
| Brake + split in y | 33.2 m |
| Brake + split in y and z | 28.9 m |

**Conclusion.** For a 30 m/s head-on the braking-only barrier starts intervening about 2.4 times farther out than necessary (69.8 m against 28.9 m). The gap grows with speed, because the braking requirement is quadratic in closing speed while the time to build a lateral offset is set by `sqrt(2 d_eff / a_lat)`. Conversely, every state the braking barrier calls unsafe between 28.9 m and 69.8 m is in fact recoverable, so the barrier makes the filter intervene early and produces the long detours and slow crossings seen in testing.

There is a second, more practical cost. A braking-only barrier gives the QP no reason to turn. In symmetric encounters the minimum-norm solution is pure braking, and only the deterministic tie-break (P0-SA-01 §6 step 4) breaks the symmetry. PR B measured this on the 90° crossing (both drones 50 m from the crossing point at 15 m/s):

| `tie_break_mps2` | Episode steps (unconstrained: 296) | Time below 6 m | Max path deviation |
|---|---|---|---|
| 0 | 1000 (did not finish) | 94.7 s | 239.9 m |
| 0.5 (old default) | 323 (+9.1 %) | 3.6 s | 77.3 m |
| 1.0 (new default) | 307 (+3.7 %) | 0.3 s | 46.1 m |
| 2.0 | 302 (+2.0 %) | 0 s | 32.7 m |

With a tie-break of 0.5 m/s² there is no oscillation: the sign of the lateral acceleration flips only twice in the episode. The stall is a lock. The braking rows push each drone along the line of sight, which for perpendicular traffic deflects each one toward the other's direction of travel. Both drones end up flying side by side along the diagonal at 5.2–5.9 m for about 3.6 s before the tie-break pulls them apart. PR B raises the default to 1.0 m/s². That is a tuning change. The underlying cause is the braking-only barrier, and it is what this note proposes to replace.

---

## 3. The three-drone sandwich (PR B item 1)

d1 and d3 close on a hovering d2 at 10 m/s from 10 m on each side.

- **The 4.45 m upper bound already includes vertical motion.** The proof in `test_three_drone_sandwich_is_physically_unrecoverable` bounds the lateral offset using 7 m/s² per axis on both y and z (`7√2` in magnitude), so it did not need to be redone in 3D. It bounds each outer pair separately.
- **The bound is not achievable.** Keeping only the two outer pairs gives 4.441 m, but that strategy sends d1 and d3 the same way and they then collide (1.47 m). Optimising all three pairs together (SLSQP over 100 ms piecewise-constant accelerations, 8 starts) gives **4.177 m** best found; the optimum lies between 4.177 and 4.446 m. Its structure: d1 and d3 brake, d2 escapes in y opposite to both, and d1 and d3 split in z.
- **What the filter does now.** When the first QP solve leaves braking slack on a closing pair, the filter re-solves those pairs with a constraint written in metres: the predicted separation at the time of closest approach `t*` (clamped to 0.1–3 s), along a direction that mixes the line of sight with a lateral and a vertical escape axis. The escape slack is penalised quadratically, so the violation is shared across pairs instead of being pushed onto one. Escape directions are chosen by a greedy two-colouring of the conflict graph, separately for the lateral and vertical axes. Result: **3.91 m** minimum separation, against 4.177 m best found and 4.446 m upper bound (PR A: 0.61 m). There were no fallback events.

The escape stage is a heuristic layer on top of the braking barrier. It is only triggered after the braking barrier has already failed. The proposal below would make escape part of the barrier itself.

---

## 4. Proposal: a barrier that knows about lateral escape

Two candidates from the literature fit our setting (double-integrator agents, per-axis input bounds, central QP).

### 4.1 Collision-cone CBF (recommended for evaluation first)

References (each checked against its arXiv page on 2026-10-21):

- M. Tayal, R. Singh, J. Keshavan, S. Kolathaya, "Control Barrier Functions in Dynamic UAVs for Kinematic Obstacle Avoidance: A Collision Cone Approach", arXiv:2303.15871. https://arxiv.org/abs/2303.15871
- P. Thontepu, B. G. Goswami, M. Tayal, et al., "Control Barrier Functions in UGVs for Kinematic Obstacle Avoidance: A Collision Cone Approach", arXiv:2209.11524 (title of v4; earlier versions were titled "Collision Cone Control Barrier Functions for Kinematic Obstacle Avoidance in UGVs"). https://arxiv.org/abs/2209.11524

The safe set is the set of states whose relative velocity points outside the cone of directions that would bring the pair within `d_eff`:

    h = p_rel · v_rel + ‖p_rel‖ ‖v_rel‖ cos φ,     cos φ = sqrt(‖p_rel‖² − d_eff²) / ‖p_rel‖

`h ≥ 0` means the relative velocity is outside the collision cone. `h` already contains velocity, so `ḣ` contains the relative acceleration directly. That makes it a first-order CBF with acceleration input, and the QP row is still linear in the accelerations, as now.

What it would change for us:

- It treats steering and braking alike: any acceleration that moves `v_rel` out of the cone satisfies the constraint. The minimum-norm QP then picks lateral motion when it is cheaper than braking, without a tie-break.
- A head-on pair at 30 m/s only needs to act once the cone condition binds, which is much later than the braking curve.
- The crossing lock should not form in the same way: the braking rows push each drone along the line of sight, while the cone condition is met by any correction that turns `v_rel` out of the cone, which is usually sideways. This is expected, not verified; the 5 % crossing test will check it.

Known limits that must be handled before adoption:

- `cos φ` is undefined for `‖p_rel‖ < d_eff`. It needs the same kind of bounded extension as PR B's B(s), and that must be proven to keep the row bounded.
- The CBF condition alone does not account for the input bounds. A pair can be outside the cone yet unable to stay out of it with ±3.5 m/s² (for example, very close with a large tangential speed). The always-feasible slack QP from PR A still catches this, and the escape stage remains as the second layer.
- `v_rel = 0` makes `h = p_rel · 0 + 0 = 0`, on the boundary. Two hovering drones near each other would always be "active". This needs a small velocity floor or a hybrid with a distance-only barrier at low speed.

### 4.2 Backup-set CBF (alternative)

References (each checked on 2026-10-21):

- T. Gurriet, M. Mote, A. Singletary, P. Nilsson, E. Feron, A. D. Ames, "A Scalable Safety Critical Control Framework for Nonlinear Systems", IEEE Access, vol. 8, pp. 187249–187275, 2020. DOI 10.1109/ACCESS.2020.3025248. https://doi.org/10.1109/ACCESS.2020.3025248
- Y. Chen, M. Jankovic, M. Santillo, A. D. Ames, "Backup Control Barrier Functions: Formulation and Comparative Study", arXiv:2104.11332. https://arxiv.org/abs/2104.11332

Pick a backup policy (for example the evasive brake-and-split already used in the fallback), simulate the pair forward under it for a short horizon, and constrain the QP so that the backup trajectory from the next state stays outside `d_eff`. This respects input bounds by construction and gives a recoverability argument that the collision cone does not.

Its cost is computation: one forward simulation and its sensitivity per active pair per step. At N = 100 with the current dense pair search that is too slow for the 100 ms step until PR C's spatial hash lands. It is the stronger guarantee, so it is the right target if the collision cone fails review or testing.

### 4.3 Recommendation

1. Prototype the collision-cone CBF behind a `FilterConfig` switch, keeping the braking barrier as the default.
2. Use the same evidence as PR B: the head-on gaps (40–80 m), the 90° crossing with the 5 % liveness test, the sandwich (target ≥ 3.91 m), and the dense random-traffic test.
3. Accept it only if it is no worse on minimum separation in every case and better on episode time.
4. Keep the backup-set CBF as the follow-up if the collision cone cannot be bounded inside `d_eff` or loses separation in the sandwich.

**Decision (reviewer, 2026-10-21):** skip the collision cone. Prototype the backup-set CBF (§4.2) in PR B2, with PR A's evasive manoeuvre (`_evasive_accel`) as the backup controller, so the fallback and the barrier become one mechanism. It goes behind a `FilterConfig` switch, and the braking barrier stays the default and the baseline. The evidence in step 2 above still applies. If the first version misses the 100 ms step at N = 64, B2 stops and reports, and optimisation moves to PR C. B2 is the last filter-research PR.

**Follow-up decision (reviewer, 2026-10-21):** keep the escape stage on in backup mode; revise the gate to "backup p95 step time at N = 64 at most 3× the braking baseline, zero fallbacks, no new entries into `d_eff`" (absolute latency is PR C's job); derive the backup horizon from the limits; if all pass, make backup the default and keep braking selectable for one release. Results in §5.

---

## 5. PR B2: the backup-set barrier as built

Code: `_backup_steps`, `_backup_max_steps`, `_evasive_direction`, `_backup_rollout` and `_backup_constraints` in `src/utm_services/safety/filter.py`, selected by `FilterConfig.barrier` (`"backup"`, the default since contract v1.3, or `"braking"`). Tests: `tests/test_filter_backup.py` and `tests/test_filter_backup_horizon.py`.

### 5.1 Backup controller

The backup controller is the evasive manoeuvre of the PR A fallback, now one function (`_evasive_direction`) called by both the fallback and the barrier. For a pair it sums the unit line-of-sight vector away from the other drone, a horizontal lateral unit vector and a vertical unit vector, and scales the sum so its largest component is `u = 3.5 m/s²`. Both drones of a pair fly opposite directions from the same rule.

Since the PR #12 review (contract v1.4) two details changed:

- **Lateral direction near a vertical line of sight.** Away from vertical the lateral vector is `z × n`, normalised. That is undefined when `n` is vertical, and it swings from step to step when `n` is nearly vertical. Within 10° of vertical the filter instead uses the horizontal offset between the drones, projected perpendicular to `n`; an exactly vertical `n` uses +x (−x for a downward `n`). The two rules blend linearly between 10° and 20°. The rule is antisymmetric, so the two drones of a pair always get opposite lateral vectors, and it depends only on the relative position, so it does not rotate while the conflict lasts. The escape stage's vertical axis is `n × lateral`.
- **World bounds.** Each drone's acceleration on each axis is limited by a barrier on its distance to each wall of the world box (x/y 0–1000 m, z 30–200 m), using the same `B(s) = sqrt(2u s + c²) − c` form as the pair barrier. These limits are hard bounds in the QP. The evasive manoeuvre is clipped to them in the backup rollout and in the fallback, so neither can drive a drone out of the box. Near a wall the clipping can reduce the braking component below `u / √2`, the bound the minimum horizon in §5.3 rests on; the rollout is therefore adaptive (§5.3). This closes BL-01.

### 5.2 Integration step: 100 ms zero-order hold

The backup trajectory is simulated with the simulator's own update: the acceleration is held for one 100 ms step, the velocity is capped at 15 m/s, and the position uses the trapezoid of the old and new velocity. With this step the first backup step is exactly what the fallback would fly, so the barrier certifies the manoeuvre the drone actually has, not a continuous-time idealisation of it. A finer step would cost proportionally more time and would no longer match the simulator.

### 5.3 Horizon: derived from the limits, adaptive near walls

Along the line of sight `n` the evasive acceleration of one drone is at least `u / √2`. Proof: `n` and the lateral vector `l` are orthonormal, so for the x and y components `|n_x| + |l_x| ≤ √2` (and the same for y); the z component is `|n_z| + 1`, and the n-component of the unscaled sum is `1 + |n_z|`. After scaling to a largest component of `u`, the n-component is `u (1 + |n_z|) / max(√2, 1 + |n_z|) ≥ u / √2`. A test checks the bound on 10 000 random directions.

A pair closing at the maximum `2 v_max = 30 m/s` therefore stops closing within `2 v_max / (2 u / √2) = √2 v_max / u = 6.06 s`. The horizon is that stopping time plus `FilterConfig.lag_s` plus a 0.5 s margin, rounded up to whole steps: 66 steps (6.6 s) at `lag_s = 0`. The margin keeps the closest approach from being cut off at the horizon end by the 100 ms hold and by the turn of the line of sight during the manoeuvre. `test_backup_horizon_covers_stopping_at_max_closing_speed` fails if the horizon is shorter than the derived value. The first prototype used 5 s, which covers only the axis-aligned head-on (relative deceleration 7 m/s², 4.3 s); that test failed on it.

**The `u / √2` bound does not hold near a wall.** Since v1.4 the evasive manoeuvre is clipped to the drone's world-box limits (§5.1). When the escape direction points into a wall, the clipped component can remove most of the braking along `n`, and when the drone is moving fast towards the wall the wall barrier can even require acceleration towards the neighbour. The 66-step horizon is then not covered by the derivation above.

**Scan.** A 300-step rollout was run on near-wall pairs (one drone within 1 m of the floor, the ceiling, the x = 0 wall or the y = 1000 m wall, the other 10–150 m away, both up to 15 m/s, the far drone aimed at the near one; 3000 draws per wall, both id orders), plus max-closing pairs with random lines of sight and hand-picked vertical cases. No pair was still closing after step 55 (floor 52, ceiling 55, x = 0 41, y = 1000 43, vertical 46), so the 66-step horizon was not observed to be short. The gap was in the derivation, not in a run.

**Adaptive rollout.** The rollout (`_backup_rollout`) now flies every pair at least the 66 derived steps and continues while the pair still closes, up to a hard cap of 109 steps (`_backup_max_steps`). The cap is a loose bound, not a proof: the clipping can force the near-wall drone towards its neighbour for at most `v_max / u = 4.29 s`, until it has braked to rest along the wall normal; after that the pair is assumed to stop closing within a further `√2 v_max / u = 6.06 s`, as in the free case. With `lag_s` and the 0.5 s margin that is 10.85 s, 109 steps. A pair still closing at the cap is treated conservatively: its row uses the closest distance reached so far (the last step, so `k*` is the cap), both drones get `SafeAction.backup_rollout_capped = true`, the pair is listed in `FilterDiagnostics.backup_rollout_cap_pairs`, and the episode summary counts it in `rollout_cap_hits`. The summary also reports `max_backup_rollout_steps` and `p99_backup_rollout_steps`, so it is visible whether real scenarios ever need more than 66 steps. A pair that stops closing at or before step 66 gets exactly the old 66-step row, so the change does not alter any trajectory where the old horizon was enough (the random-traffic runs at N = 16 and N = 64 give the same step counts and minimum separations before and after). Tests: `tests/test_filter_backup_horizon.py` (near-wall max closing at the floor, the ceiling and an x and a y edge; the extension past the minimum; a forced cap hit with a reduced cap).

### 5.4 The constraint row

For each pair, `h = min_k |r_k| − d_eff` over the backup trajectory, reached at step `k*`. The row asks that the trajectory starting with the QP's acceleration instead of the backup's first step keeps `h_next ≥ (1 − γ dt) h` at `k*`, linearised as `m · r_k*(a)` with `m = r_k* / |r_k*|`. Because `m · r ≤ |r|`, the linear row is sufficient. The sensitivity of `r_k*` to the first-step acceleration is `(k* + 0.5) dt²`; the row is divided by it, so it has a unit normal and its slack is in m/s² like the braking row. Left in metres (coefficients near 0.05), OSQP stopped at 4000 iterations and the fallback ran 4–16 times per run. `γ = FilterConfig.gamma = 1`: a sweep over 1, 3 and 10 gave the most margin at 1; at 3 every head-on passed at only 5.5–5.7 m, and at 10 there were fallbacks and time inside `d_eff`.

The per-step condition `h_next ≥ (1 − γ dt) h` is approximate, because of the linearisation and the fixed `k*`. In the PR #12 review fuzz it failed on 0.74 % of steps, by at most 0.12 m. Forward invariance still held in that fuzz: 0 of 1086 runs entered `d_eff`. This is evidence, not a proof (see BL-04).

### 5.5 The escape stage in backup mode

Closing pairs left with slack still go through the PR B escape stage. In the sandwich the backup rows leave the outer pair (d1, d3) without slack, so with the slack pairs alone the escape colouring sent d1 to the same side as d2 and the run reached 3.48 m. In backup mode the escape set therefore also includes closing pairs whose two drones are both in slack pairs. The sandwich then matches braking exactly (3.913 m).

### 5.6 Results

Scenario suite, braking against backup, 600-step limit, reference policy as the stand-in for MARL. The latency is wall-clock time per `apply_filter` call on the development laptop:

| Scenario | Min separation (m) | Max depth (m) | Time inside `d_eff` (s) | Path overhead (%) | Steps | Fallbacks | `apply_filter` p50 / p95 (ms) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Head-on 40 m | 5.58 → 7.09 | 0 → 0 | 0 → 0 | 0.2 → 0.4 | 296 → 297 | 0 → 0 | 2.4 / 3.2 → 10.3 / 12.5 |
| Head-on 50 m | 6.16 → 8.33 | 0 → 0 | 0 → 0 | 0.2 → 0.4 | 296 → 297 | 0 → 0 | 2.4 / 4.8 → 10.1 / 11.9 |
| Head-on 60 m | 6.50 → 9.15 | 0 → 0 | 0 → 0 | 0.2 → 0.3 | 296 → 297 | 0 → 0 | 2.5 / 5.3 → 10.1 / 11.1 |
| Head-on 80 m | 14.57 → 9.76 | 0 → 0 | 0 → 0 | 0.7 → 0.3 | 307 → 296 | 0 → 0 | 2.4 / 3.2 → 10.1 / 11.6 |
| Crossing 90° | 5.81 → 8.13 | 0 → 0 | 0 → 0 | 2.5 → 0.3 | 307 → 297 | 0 → 0 | 2.3 / 2.6 → 10.1 / 12.4 |
| Sandwich | 3.91 → 3.91 | 1.59 → 1.59 | 1.8 → 1.6 | 2.0 → 2.8 | 338 → 347 | 0 → 0 | 2.4 / 3.3 → 10.0 / 12.3 |
| Random 16, seed 0 | 14.46 → 12.20 | 0 → 0 | 0 → 0 | 0.3 → 0.0 | 349 → 348 | 0 → 0 | 4.6 / 9.8 → 12.4 / 16.5 |
| Random 16, seed 1 | 14.96 → 9.39 | 0 → 0 | 0 → 0 | 0.7 → 0.1 | 324 → 323 | 0 → 0 | 5.3 / 9.4 → 13.4 / 16.6 |
| Random 16, seed 2 | 7.20 → 7.35 | 0 → 0 | 0 → 0 | 1.4 → 0.1 | 309 → 306 | 0 → 0 | 6.3 / 9.7 → 13.8 / 17.1 |
| Random 16, seed 3 | 7.84 → 8.45 | 0 → 0 | 0 → 0 | 1.1 → 0.1 | 258 → 257 | 0 → 0 | 7.1 / 10.0 → 14.8 / 20.0 |
| Random 16, seed 4 | 8.89 → 8.63 | 0 → 0 | 0 → 0 | 1.1 → 0.1 | 320 → 315 | 0 → 0 | 5.6 / 9.4 → 13.4 / 17.1 |
| All scenarios | | | | | | | 2.6 / 9.0 → 10.4 / 16.7 |

Path overhead is the flown length over the sum of straight-line distances to the goal radius, minus 1. On a 30 m/s head-on the braking barrier starts acting at about 101 m and the backup barrier at about 68 m. Backup has a smaller minimum separation where braking intervened far out (80 m head-on, random seeds 0, 1 and 4) because it no longer brakes early; all of those stay above `d_eff`. The sandwich takes 9 more steps.

N = 64 gate (random traffic from `tests/test_random_traffic.py` with 64 drones, seeds 0–4):

| Run | Barrier | `apply_filter` p50 / p95 / max (ms) | Steps over 100 ms | Fallbacks | Time inside `d_eff` | All 64 done |
| --- | --- | --- | --- | --- | --- | --- |
| Start at rest | braking | 45.5 / 217.9 / 317.6 | 38.7 % | 0 | 0 s | yes |
| Start at rest | backup | 52.3 / 165.2 / 215.4 | 35.3 % | 0 | 0 s | yes |
| Start at 15 m/s toward goal | braking | 35.5 / 235.3 / 753.5 | 34.4 % | 0 | 2.2 s (seed 2, depth 0.36 m) | yes |
| Start at 15 m/s toward goal | backup | 43.5 / 165.2 / 219.3 | 30.8 % | 0 | 0 s | yes |

The gate passes: backup p95 is below the braking p95 (limit 3×), with zero fallbacks and no entry into `d_eff`. Both barriers miss the absolute 100 ms step at N = 64; that is PR C.

### 5.7 Known limits

Each is also in the `_backup_constraints` docstring or in `TASKS.md`:

- The backup is pairwise: a drone with several neighbours is credited with a different escape against each. Backlog **BL-03**.
- Only the closest-approach step `k*` is constrained. Backlog **BL-04**.
- The sensitivity `(k* + 0.5) dt²` ignores the backup's feedback and the speed cap.
- `lag_s` lengthens the horizon but the row does not model the actuator lag; the braking barrier's lag reserve has no backup equivalent yet.
- One forward simulation per pair per step: about 4× the braking cost at N = 16. Since the adaptive rollout it is computed once per step and shared by the first and the escape solve.
- The 109-step cap rests on a loose argument, not a proof; a pair that hits it is flagged and counted (§5.3).
