# P2-SA-02 — Lateral Escape: Is the Braking-Only Barrier Too Conservative?

| Field | Value |
|---|---|
| Task | Sprint 1 PR B, item 4 |
| Status | DRAFT for review. No code in this PR implements the proposed barrier. |
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
- **The bound is not achievable.** Keeping only the two outer pairs gives 4.441 m, but that strategy sends d1 and d3 the same way and they then collide (1.47 m). Optimising all three pairs together (SLSQP over 100 ms piecewise-constant accelerations, 8 starts) gives **4.168 m**. Its structure: d1 and d3 brake, d2 escapes in y opposite to both, and d1 and d3 split in z.
- **What the filter does now.** When the first QP solve leaves braking slack on a closing pair, the filter re-solves those pairs with a constraint written in metres: the predicted separation at the time of closest approach `t*` (clamped to 0.1–3 s), along a direction that mixes the line of sight with a lateral and a vertical escape axis. The escape slack is penalised quadratically, so the violation is shared across pairs instead of being pushed onto one. Escape directions are chosen by a greedy two-colouring of the conflict graph, separately for the lateral and vertical axes. Result: **3.91 m** minimum separation, against 4.168 m best found and 4.45 m upper bound (PR A: 0.61 m). There were no fallback events.

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

**Decision needed from the reviewer:** approve step 1 (collision cone first), or go directly to the backup-set CBF. No code will be written for either until this note is reviewed.
