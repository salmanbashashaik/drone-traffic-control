# P1-SA-01 — Reward Function Proposal

| Field | Value |
|---|---|
| Task | P1-SA-01 (input to P2-SA-01 training) |
| Owner | Salman (`salman/marl-safety`) |
| Status | DRAFT — answering OI-3 from SIMULATOR_NOTE.md |
| Date | 2026-10-20 |
| Authority | `INTERFACES.md` and `shared/contracts.py` on `main`. All reward terms must use fields that exist in the step result or can be derived from `DroneState` / `SafeAction`. |

---

## 1. Design principles

1. **Reward is not a safety metric.** Training reward measures learning progress.
   It must never be reported as evidence of safety. Safety is measured by
   separation violation counts, filter interventions, and infeasibility rates
   from the evaluation (P3-SA-01).

2. **Mission progress must dominate.** The primary objective is reaching the
   goal. Safety penalties modify behavior around conflicts but must not prevent
   the drone from making progress toward its goal in normal conditions.

3. **All terms must be computable from the step contract.** No term may require
   information outside `ObservationV1`, `SafeAction`, and the episode summary
   fields in `INTERFACES.md`.

4. **Weights are hyperparameters, not safety parameters.** The values below are
   starting points. They will be tuned from training curves and must not be
   cited as safety evidence.

---

## 2. Proposed reward function

For agent `i` at step `t`, the per-step reward is:

```
r_i[t] = r_progress + r_goal + r_separation + r_smooth + r_fallback
```

### Term 1 — Mission progress (positive, dense)

```
r_progress = λ_p · (d_goal[t-1] - d_goal[t])
```

Where `d_goal[t]` is the 3D Euclidean distance to goal at step `t`
(computable from `ObservationV1` indices 6–8: `||goal_disp||`).

- Rewards the drone for getting closer to its goal each step.
- Dense: fires every step, gives a continuous learning signal.
- `λ_p = 1.0` (starting value)

### Term 2 — Goal reached (positive, sparse terminal)

```
r_goal = +C_goal   if agent completed (reached goal_radius_m = 2.0m) this step
r_goal = 0         otherwise
```

- `C_goal = 50.0` — large enough to dominate all other terms over an episode.
- Ensures the policy is incentivized to complete missions, not just loiter.

### Term 3 — Separation (negative, risk-shaped)

```
r_separation = -λ_s · max(0, d_safe - d_nearest[t]) / d_safe
             + -C_violation   if separation_violation == True this step
```

Where `d_nearest[t]` is the nearest active neighbor distance
(`ObservationV1` index 15, or `infos[agent_id]['nearest_separation_m']`).

- The smooth term gives a graduated penalty as drones approach `d_safe = 5m`.
  At exactly `d_safe` distance: penalty = 0. At 0m: penalty = `λ_s`.
- The hard penalty `C_violation` fires when a separation violation is logged.
- `λ_s = 5.0`, `C_violation = 20.0` (starting values)
- When no neighbor exists (`d_nearest = -1`), separation term = 0.

**Why not a pure hard penalty:** a purely binary penalty gives the policy no
gradient signal until it is already in violation. The smooth term shapes
behavior before the boundary is crossed, which aids learning. This follows
the risk-shaping approach in the research report (Section 3).

### Term 4 — Action smoothness (negative, small)

```
r_smooth = -λ_u · ||a_i^safe||²   (magnitude penalty)
         + -λ_Δu · ||a_i^safe[t] - a_i^safe[t-1]||²   (jerk penalty)
```

- Penalizes large accelerations and rapid changes, encouraging smooth flight.
- Prevents reward hacking where the policy oscillates at high acceleration.
- `λ_u = 0.01`, `λ_Δu = 0.01` (small — should not dominate mission progress)
- Uses `SafeAction.safe_accel_mps2` (the filtered action, not the proposal),
  so the policy is penalized for actions that required large filter corrections.

### Term 5 — Fallback penalty (negative, sparse)

```
r_fallback = -C_fallback   if fallback_used == True this step
r_fallback = 0             otherwise
```

- Penalizes episodes where the filter had to fall back to zero acceleration.
- This incentivizes the policy to propose actions that keep the QP feasible.
- `C_fallback = 10.0` (starting value)
- Requires `SafeAction.fallback_used` from `infos`.

---

## 3. Complete reward summary

| Term | Formula | `λ` start | Rationale |
|---|---|---|---|
| Progress | `λ_p · (d[t-1] - d[t])` | 1.0 | Dense goal-seeking signal |
| Goal reached | `+50` (terminal) | — | Mission completion dominates |
| Separation smooth | `-λ_s · max(0, d_safe - d_nearest) / d_safe` | 5.0 | Shaped avoidance gradient |
| Separation violation | `-20` (event) | — | Hard penalty for violations |
| Action magnitude | `-λ_u · \|\|a\|\|²` | 0.01 | Smooth flight |
| Action jerk | `-λ_Δu · \|\|Δa\|\|²` | 0.01 | Smooth flight |
| Fallback | `-10` (event) | — | Keep QP feasible |

Episode return: `R_i = Σ_t γ^t · r_i[t]`  with discount `γ = 0.99`.

---

## 4. What this reward does NOT do

- Does not use average reward as a safety measure. Safety is measured
  separately via separation violation counts and filter metrics.
- Does not penalize filter interventions directly. The filter is a
  safety mechanism, not a reward signal. If the policy frequently
  triggers interventions, that is visible in evaluation, not in reward.
- Does not use `h_min_m2` (barrier value) directly as a reward signal.
  The barrier is a control-theoretic quantity, not a training objective.
- Does not encode any term that requires information outside the v1
  step contract (no ground truth of other agents beyond nearest neighbor,
  no future trajectory information).

---

## 5. Open questions (to resolve before P2-SA-01 training)

| # | Question | Decision needed from |
|---|---|---|
| R1 | Should the separation penalty use `d_nearest` from `ObservationV1` index 15 (policy-visible) or `nearest_separation_m` from `infos` (ground truth)? Training with ground truth is cleaner but the policy won't have it at runtime. | Salman, Anas |
| R2 | Should `r_smooth` penalize the proposed action `a^nom` or the filtered action `a^safe`? Current proposal uses `a^safe` — this trains the policy to avoid triggering the filter. | Salman |
| R3 | Abort penalty: should out-of-bounds abort add a large negative terminal reward? | Salman, Anas |
| R4 | Held agents: no terminal condition defined yet. If held agents exist in v1, their reward must be defined. | Anas |
| R5 | Normalization: should observation inputs be normalized before policy inference? If yes, preprocessing parameters must be versioned with the model artifact. | Salman (P1-SA-01 method note) |

---

## 6. Handoff

**For Anas:** the reward requires `nearest_separation_m` and `separation_violation`
per agent per step from `infos`, and `fallback_used` from `SafeAction`. All
of these are already in the `INTERFACES.md` step contract. No new fields needed.

**For Aman:** training reward must not appear in any demo or product messaging
as a safety measure. The evaluation report (P3) uses the fields from
`INTERFACES.md`, not training reward.
