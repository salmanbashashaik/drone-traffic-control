# P0-SA-01 — Mathematical Formulation and Assumptions

| Field | Value |
|---|---|
| Task | P0-SA-01 |
| Owner | Salman (`salman/marl-safety`) |
| Status | DRAFT — for review by Anas and Aman before implementation begins |
| Date | 2026-10-19 |
| Authority | `INTERFACES.md` and `shared/contracts.py` on `main`. All equations here must match what Anas implements. |

---

## 0. Proposed contract change (requires PR to `main`)

The current contract sets `MAX_ACCEL_COMPONENT_MPS2 = 3.0 m/s²`. This formulation proposes raising it to **3.5 m/s²** to give the filter more authority to maneuver, especially in head-on and high-closing-speed scenarios (see Section 5). This is a shared-contract change:

- `shared/contracts.py`: `MAX_ACCEL_COMPONENT_MPS2 = 3.5`
- `INTERFACES.md`: update the stated bound
- Affected: Anas (dynamics, bound validation), Aman (scenario config validation), Salman (QP constraint set)

**This note is written assuming 3.5 m/s². Do not implement the filter until this PR is merged to `main`.**

---

## 1. Notation

| Symbol | Meaning | Units |
|---|---|---|
| `N` | Number of active agents in the episode | — |
| `i, j` | Agent indices, `i ≠ j` | — |
| `p_i` | Position of agent `i`: `[x, y, z]` | m |
| `v_i` | Velocity of agent `i`: `[vx, vy, vz]` | m/s |
| `a_i` | Control acceleration of agent `i` | m/s² |
| `a_i^nom` | Nominal (policy-proposed) acceleration | m/s² |
| `a_i^safe` | Filter-returned acceleration | m/s² |
| `r_ij` | Relative position: `p_i - p_j` | m |
| `v_ij` | Relative velocity: `v_i - v_j` | m/s |
| `d_safe` | Minimum required separation: `5.0 m` | m |
| `dt` | Simulation step: `0.1 s` | s |
| `u_max` | Max acceleration per component: `3.5 m/s²` | m/s² (proposed) |
| `v_max` | Max 3D speed magnitude: `15.0 m/s` | m/s |
| `h_ij` | Pairwise barrier function | m² |
| `α₁, α₂` | HOCBF class-K coefficients | — |

---

## 2. State and control

Each active drone has state:

```
x_i = [p_i, v_i] ∈ ℝ⁶
```

The filter's only input about world truth is the `DroneState` (position + velocity + goal per agent),
read directly from the simulator ground truth. The policy sees only `ObservationV1` (18 floats).
**The filter must not use the policy observation — it uses `DroneState` exclusively.**

Control input per agent:

```
a_i = [ax, ay, az],   |a_i[k]| ≤ 3.5 m/s²  for k ∈ {x, y, z}
```

The constraint is per-component (a box), not a sphere. This matters for QP constraint setup.

---

## 3. Discrete dynamics (must match Anas's simulator exactly)

```
p_i[t+1] = p_i[t] + v_i[t]·dt + 0.5·a_i[t]·dt²
v_i[t+1] = v_i[t] + a_i[t]·dt
```

With `dt = 0.1 s`, the per-step numbers are:

```
position contribution from accel:   0.5 · a · dt² = 0.5 · a · 0.01 = 0.005·a  meters
velocity contribution from accel:   a · dt = 0.1·a  m/s
```

At max accel 3.5 m/s²:
- Max velocity change per step: `3.5 × 0.1 = 0.35 m/s`
- Max position nudge per step: `0.5 × 3.5 × 0.01 = 0.0175 m`

This is a **discrete double integrator** — no drag, no rotational dynamics, no actuator lag.
The filter's predictions are only as good as this model.

---

## 4. Safety constraint and barrier function

For every active pair `(i, j)`, the safety requirement is:

```
||p_i - p_j||  ≥  d_safe = 5.0 m   at all times
```

Define the **pairwise barrier**:

```
h_ij(p_i, p_j) = ||p_i - p_j||²  -  d_safe²
               = ||r_ij||²  -  25.0
```

- `h_ij ≥ 0` means the pair is at or beyond the safe separation (barrier satisfied)
- `h_ij < 0` means the pair is inside `d_safe` (barrier violated — separation violation)
- `h_ij = 0` is exactly on the boundary

This is what gets logged in `SafeAction.h_min_m2` — the minimum `h_ij` across all active pairs.

---

## 5. Why we need HOCBF — the relative degree problem (and the head-on case)

### The relative degree issue

To use a CBF, we need the control input `a_i` to directly appear in the time derivative (or
difference) of `h_ij`. Let's check:

```
h_ij depends on  p_i, p_j           (positions)
Δh_ij depends on  v_i, v_j          (velocities, one step later)
Δ²h_ij depends on  a_i, a_j         (accelerations, two steps later)
```

`a_i` only appears at the **second discrete difference** of `h_ij`. This means `h_ij` has
**relative degree 2** under acceleration control. A first-order CBF (plain CBF) assumes
relative degree 1 — it would enforce `Δh_ij ≥ -α·h_ij` but `a_i` doesn't appear in `Δh_ij`,
so the constraint would have no decision variable in it. It would be either trivially satisfied
or trivially infeasible, not a useful constraint.

**HOCBF solves this** by creating a chain of auxiliary functions that "propagate" the control
influence up through the derivatives until we reach the level where `a_i` appears.

### The head-on scenario — why this is hard

Two drones flying directly at each other at `15 m/s` each have a **closing speed of 30 m/s**.

```
Time to reach d_safe (5 m) from a given distance D:

If they start 50 m apart:   time = (50 - 5) / 30 = 1.5 seconds = 15 steps
If they start 20 m apart:   time = (20 - 5) / 30 = 0.5 seconds = 5 steps
If they start 10 m apart:   time = (10 - 5) / 30 = 0.17 seconds = 1.7 steps
```

At `d_safe = 5 m` and closing at 30 m/s, you have essentially **one step** of warning.

The maximum deceleration you can apply per step with 3.5 m/s² control authority:
```
Δv per step = 3.5 × 0.1 = 0.35 m/s
```

To stop a 15 m/s drone requires: `15 / 0.35 ≈ 43 steps = 4.3 seconds`

**The problem:** the filter cannot physically prevent a collision if two drones are already
very close and heading directly at each other at full speed. No control authority is large
enough to stop them in one or two steps. This is a fundamental physics constraint, not a
software bug.

### What the PDF says about this

The research report (Section 2) describes this using the **tactical margin**:

```
Mt = tloss - τe2e - tmaneuver
```

Where:
- `tloss` = time until the system crosses the recoverability boundary
- `τe2e` = total pipeline delay (sensing → compute → actuation)
- `tmaneuver` = time needed to execute a physically feasible avoidance maneuver

**If `Mt ≤ 0`, no algorithm can help.** By the time the command has physical effect,
recovery is no longer possible. The report concludes: *"tactical collision avoidance
belongs as close to the aircraft as practical."*

The paper's proposed solution for this is **Hamilton-Jacobi (HJ) reachability** — a method
that precomputes a "recoverability boundary": a set of states from which a collision is still
avoidable. The runtime law switches to a guaranteed escape maneuver *before* the boundary
is crossed, not after.

```
u = { u_MARL/CBF    if V(x) > ε   (still recoverable, let MARL/CBF run)
    { u*_HJ(x)      if V(x) ≤ ε   (near the boundary, switch to escape maneuver)
```

**In v1 we do NOT implement HJ reachability.** That is explicitly a post-v1 item.
What v1 does instead is:

1. The HOCBF filter starts enforcing constraints as early as possible (before the drones
   are already at `d_safe`) because the barrier condition is enforced at every step
2. If the QP becomes infeasible (e.g., the head-on case where no feasible accel exists
   within bounds), the fallback is **zero acceleration** — not a guaranteed escape, just
   the least-bad default we have without HJ
3. The evaluation must report how often this infeasibility occurs under head-on scenarios

### The camera / real-world input note

In real deployment, each drone would have a camera and other onboard sensors as primary
inputs. The MARL policy would receive processed perception outputs (object detections,
depth estimates) rather than clean ground-truth positions. The `ObservationV1` 18-value
vector is a **deliberate simplification** for the simulation prototype — it gives the policy
clean positional information. The safety filter in v1 still uses ground-truth `DroneState`,
which also assumes perfect sensing. In real deployment:

- The filter would need to account for sensor uncertainty (position error bounds)
- Detection range limits the "neighbor" set — drones outside sensor range are invisible
- Non-cooperative objects (birds, other vehicles) are not in the state at all

These are post-v1 concerns (Tier T3–T5 in the environment tier plan). They are noted here
because they affect what the v1 safety claim means: **the filter is only as good as the
state estimate fed into it.**

---

## 6. HOCBF construction — step by step

### Step 1: Define the level-0 function

```
ψ₀_ij[t] = h_ij[t] = ||r_ij[t]||² - d_safe²
```

For HOCBF invariance we need `ψ₀_ij[t] ≥ 0` for all `t`.

### Step 2: Define the level-1 function (forward difference + class-K decay)

The discrete forward difference of `ψ₀`:

```
Δψ₀_ij[t] = ψ₀_ij[t+1] - ψ₀_ij[t]
           = h_ij[t+1] - h_ij[t]
```

The level-1 function with class-K coefficient `α₁ ∈ (0, 1]`:

```
ψ₁_ij[t] = Δψ₀_ij[t] + α₁ · ψ₀_ij[t]
```

We require `ψ₁_ij[t] ≥ 0`.

### Step 3: Enforce via the level-2 condition (this is the QP constraint)

```
Δψ₁_ij[t] + α₂ · ψ₁_ij[t] ≥ 0
```

with `α₂ ∈ (0, 1]`. This is the constraint that goes into the QP. Expanding:

```
ψ₁_ij[t+1] - ψ₁_ij[t] + α₂ · ψ₁_ij[t] ≥ 0
ψ₁_ij[t+1] + (α₂ - 1) · ψ₁_ij[t] ≥ 0
```

After substituting the dynamics, `ψ₁_ij[t+1]` depends on `a_i` and `a_j` — so the constraint
is now linear in the decision variables. That makes it a valid QP constraint.

### Class-K coefficients

`α₁` and `α₂` are tuning parameters controlling how aggressively the filter intervenes:

- **Small α (e.g., 0.1):** filter starts intervening early, when drones are still far apart.
  Fewer infeasible cases, more interventions, potentially more conservative behavior.
- **Large α (e.g., 1.0):** filter waits until drones are closer before intervening.
  Less conservative, but higher risk of infeasibility in fast-closing scenarios.

**Starting values for v1:** `α₁ = α₂ = 0.5`. To be tuned based on evaluation results.
Must be treated as assumptions, not guarantees — the right values depend on scenario
density and initial conditions.

---

## 7. The QP — fully written out

Given all active agents' `DroneState` at time `t`, and nominal actions `{a_i^nom}`:

### Decision variables

```
{a_i^safe}  for all i ∈ active agents,   a_i^safe ∈ ℝ³
```

### Objective — stay as close to the nominal policy as possible

```
minimize  Σᵢ ||a_i^safe - a_i^nom||²
```

This is a **quadratic objective** — minimizing the sum of squared deviations from the
policy's proposed actions across all agents.

### Constraint 1 — HOCBF for all pairs

For every unordered active pair `(i, j)`:

```
ψ₁_ij[t+1](a_i^safe, a_j^safe) + (α₂ - 1) · ψ₁_ij[t] ≥ 0
```

This is **linear** in `{a_i^safe, a_j^safe}` after substituting the dynamics.
The constraint matrix row for pair `(i,j)` has nonzero entries only in the columns for
agents `i` and `j`.

### Constraint 2 — Control bounds (box constraints)

```
-3.5 ≤ a_i^safe[k] ≤ 3.5   for all i, k ∈ {x, y, z}
```

### Summary structure

```
min   xᵀQx + cᵀx          (Q = identity blocks, c = -2·a^nom stacked)
s.t.  A_cbf · x ≥ b_cbf   (HOCBF constraints, one row per active pair)
      lb ≤ x ≤ ub          (box control bounds)
```

This is a standard **convex QP**. Solved with `osqp` (recommended) or `cvxpy` with
the OSQP backend.

### What happens when infeasible

If no `{a_i^safe}` satisfies all HOCBF constraints within the control bounds:

- **All agents in the solve** get `a_i^safe = [0, 0, 0]`
- `SafeAction.qp_feasible = False`
- `SafeAction.fallback_used = True`
- `SafeAction.safe_accel_mps2 = [0, 0, 0]`

Zero accel means velocity is preserved: a drone moving at 15 m/s toward another drone
**keeps moving at 15 m/s**. This is not safe — it is the least-disruptive default given
no feasible solution exists. It must be logged, counted in `qp_infeasible_steps` and
`fallback_events`, and never described as a hover or guaranteed escape.

---

## 8. Expanding Δh_ij in terms of a — the linear constraint derivation

This section shows concretely how the HOCBF constraint becomes linear in `a`.

Define at step `t`:
```
r_ij = p_i - p_j                     (relative position, known)
v_ij = v_i - v_j                     (relative velocity, known)
a_ij = a_i^safe - a_j^safe           (relative acceleration, decision variable)
```

Step 1: Compute `r_ij[t+1]` from the discrete dynamics:
```
r_ij[t+1] = r_ij[t] + v_ij[t]·dt + 0.5·a_ij·dt²
```

Step 2: Compute `h_ij[t+1]`:
```
h_ij[t+1] = ||r_ij[t+1]||²  -  d_safe²
           = ||r_ij + v_ij·dt + 0.5·a_ij·dt²||²  -  d_safe²
```

Expand the square (let `s = r_ij + v_ij·dt` — this is purely known at step `t`):
```
h_ij[t+1] = ||s + 0.5·a_ij·dt²||²  -  d_safe²
           = ||s||² + 2·sᵀ·(0.5·a_ij·dt²) + ||0.5·a_ij·dt²||²  -  d_safe²
           = ||s||² + sᵀ·a_ij·dt² + 0.25·||a_ij||²·dt⁴  -  d_safe²
```

The `||a_ij||²` term is **quadratic** in `a`. For the QP we drop it (it's tiny:
`0.25 × dt⁴ = 0.25 × 10⁻⁴`). This is the standard first-order linearization:

```
h_ij[t+1] ≈ (||s||² - d_safe²)  +  sᵀ·(a_i^safe - a_j^safe)·dt²
           =     known scalar        +   linear in a_i^safe, a_j^safe
```

So:
```
Δh_ij ≈ (||s||² - d_safe²) - h_ij[t]  +  sᵀ·(a_i^safe - a_j^safe)·dt²
```

And the HOCBF level-1 function:
```
ψ₁_ij[t] = Δh_ij[t] + α₁·h_ij[t]
```

The QP constraint `ψ₁_ij[t+1] + (α₂-1)·ψ₁_ij[t] ≥ 0` expands into a row of the form:

```
[coeff for a_i^safe] · a_i^safe  +  [coeff for a_j^safe] · a_j^safe  ≥  scalar_rhs
```

Where:
```
coeff for a_i^safe  =  +s · dt²   (from the a_i^safe part of a_ij)
coeff for a_j^safe  =  -s · dt²   (from the -a_j^safe part of a_ij)
scalar_rhs          =  -(known terms from ψ₁[t] and ψ₁[t+1] that don't involve a)
```

This is exactly one row in the constraint matrix `A_cbf` per pair. With N active agents
there are N(N-1)/2 pairs → N(N-1)/2 rows in `A_cbf`.

---

## 9. Complete assumptions table

Every item below is an **assumption**, not a verified property.

| # | Assumption | Consequence if violated |
|---|---|---|
| A1 | Dynamics are a pure discrete double integrator | Filter predictions wrong when speed cap fires; barrier values inaccurate |
| A2 | No wind or disturbance (T1 only) | Filter is not robust to disturbances; must be redesigned for T2+ |
| A3 | Sensor is perfect — filter gets exact `DroneState` | Filter optimality degrades with position uncertainty; may enforce wrong constraints |
| A4 | Initial conditions satisfy `h_ij[0] ≥ 0` for all pairs | If drones start already inside `d_safe`, HOCBF invariance argument breaks immediately |
| A5 | Control authority (3.5 m/s² per component) is sufficient | Head-on high-speed scenarios may be physically irrecoverable regardless of filter |
| A6 | `α₁, α₂` are chosen appropriately | Wrong values → either over-conservative (frequent infeasibility) or under-conservative (violations) |
| A7 | QP solved to optimality each step | Numerical solver tolerances introduce small errors; OSQP accuracy ~1e-6 acceptable for v1 |
| A8 | Linearization of `h_ij[t+1]` is valid (drop `||a_ij||²·dt⁴` term) | Term is `~1e-4 × ||a_ij||²` which is negligible at v1 scale |
| A9 | Joint solve over all pairs (not decentralized) | Pairwise HOCBF feasibility does not prove simultaneous multi-pair feasibility — the joint solve is required |
| A10 | No uncooperative or non-communicating agents | Filter cannot constrain agents it doesn't know about (birds, other vehicles) |

---

## 10. What this formulation does NOT claim

- **No sampled-data guarantee.** Constraints are enforced at 100 ms boundaries only.
  Between steps, no constraint is active.
- **No disturbance robustness.** In T1 (perfect conditions), wind = 0. With wind the
  filter needs a tightened barrier (robust HOCBF), not implemented in v1.
- **No HJ/recoverability layer.** The report describes HJ reachability as the mechanism
  for handling cases where the CBF is about to become infeasible. V1 does not have this —
  infeasibility falls back to zero accel, which is not a verified escape.
- **No proof from use of CBF/HOCBF.** Using this filter in a simulation does not make
  the system certified, safety-assured, or flight-ready.
- **No guarantee from zero violations in a finite run.** A finite campaign with no
  violations does not prove zero risk.

---

## 11. Handoff requirements

**For Anas (implementation):**
- Confirm discrete dynamics equation exactly matches Section 3 above
- Confirm speed cap behavior (OI-1: log events, evaluate fire rate)
- Confirm `DroneState` is passed to filter as ground truth, not derived from policy obs
- The QP solver needs: stacked `[a_i^safe]` decision vector, `A_cbf` matrix (one row
  per pair), `b_cbf` RHS vector, box bounds `[-3.5, 3.5]^{3N}`, quadratic cost `Q = I`,
  linear cost `c = -2·a^nom_stacked`

**For Aman (product messaging):**
- The filter is not a safety guarantee. It is a best-effort constraint enforcer that
  works when: initial separation is ≥ d_safe, dynamics match the model, control authority
  is sufficient, and the QP is feasible
- Head-on high-speed conflicts can be physically irrecoverable within v1 constraints;
  HJ reachability (post-v1) is the proper solution
- The zero-acceleration fallback must be reported in all demos as an unverified default,
  not as "the drone stopped safely"
- Real deployment requires accounting for sensor uncertainty, non-cooperative objects,
  and onboard camera-based perception — none of which are modeled in v1

---

## 12. Proximity range gate — scalable pair selection

### The proposal and why pure repulsion doesn't work

A natural instinct is to have each drone "maximize distance" from all neighbors within
a radius (e.g. 500 m). This resembles **Velocity Obstacles (VO)** or **ORCA** from
multi-robot and urban air mobility research, and **potential-field repulsion** from
classical robotics. The appeal: drones spread out, reducing conflict density.

**This does not work cleanly for three reasons:**

1. **Conflicts with the mission objective.** "Maximize spread" pushes every drone away
   from every neighbor at all times, even when they're 400 m apart and no conflict is
   possible. This directly fights goal-seeking behavior — a drone detours unnecessarily
   and mission time increases. The HOCBF QP already does the right thing: it only
   deviates from the policy's proposed action when a specific barrier constraint is
   *actually about to be violated*.

2. **Reciprocal oscillation at cluster boundaries.** If two groups of drones approach
   each other, boundary drones get repelled by both clusters simultaneously. Both agents
   independently solve symmetric optimization problems and repeatedly switch direction.
   The research report (Section 2) documents this explicitly as **"reciprocal avoidance
   oscillation"** — observable as left-right-left or altitude oscillations. ORCA systems
   are known to produce this in dense crossing-traffic scenarios.

3. **Local optimality trap.** Repulsion from the nearest neighbor can push a drone
   directly into another conflict. The PDF documents this: *"pairwise resolutions do not
   jointly solve a multi-agent geometry — avoidance of one intruder creates another
   conflict."* The HOCBF joint QP solves all pairs simultaneously, which handles this.

### What to use instead: a range gate on the QP

The correct way to capture the "only care about nearby drones" insight is a **range gate**
on pair inclusion in the QP — not repulsion. Only include pair `(i, j)` in the HOCBF
constraint set if the drones are within a range threshold `R_gate`:

```
Include pair (i,j) in QP  iff  ||p_i - p_j|| < R_gate
```

**Choosing `R_gate`:** A pair outside `R_gate` cannot become dangerous within a K-step
lookahead. At max speed 15 m/s, closing at 30 m/s, over K=10 steps (1 second):

```
Max approach distance in K steps = v_close_max × K × dt = 30 × 10 × 0.1 = 30 m
```

So any pair currently farther than `d_safe + 30 m = 35 m` apart cannot violate `d_safe`
within 10 steps even at full closing speed. A gate of **R_gate = 100–200 m** is already
very conservative. 500 m is fine but includes far more pairs than necessary.

**Scaling benefit:** With N drones uniformly distributed in a 1 km² area, the expected
number of neighbors within R_gate = 200 m is approximately:

```
k ≈ N × π × R_gate² / A_world
  = N × π × 200² / 1,000,000
  = N × 0.126
```

So at N = 20 drones, k ≈ 2.5 neighbors per drone on average. QP constraint count
drops from N(N-1)/2 = 190 pairs to roughly Nk/2 ≈ 25 pairs — a 7× reduction.

The PDF (Section 2) confirms: *"Let G = (V,E) be a proximity graph and bound each
aircraft's relevant neighborhood by k ≪ N. Then |E| = O(Nk), so distributed tactical
processing can approach linear scaling in fleet size if physical density and sensor
horizon keep k bounded."*

### v1 implementation

For v1 with small N (≤ 10 drones), the range gate is optional — all pairs fit in the
QP trivially. Add it as a configurable parameter `qp_range_gate_m` in the filter config,
defaulting to `None` (include all pairs). When set, exclude pairs beyond the gate before
building `A_cbf`. Log the number of pairs included vs. excluded per step.

**This is a computational optimization, not a change to the safety architecture.**
The HOCBF formulation, QP structure, fallback semantics, and barrier definition remain
identical.
