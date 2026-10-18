# salman/ — Your Work
**Branch**: `salman/cbf-safety` | **Role**: Math — CBF Safety Layer

You own the formal safety guarantee. Everything you produce either becomes a runtime filter in the simulation or a theorem in the white paper.

---

## Your 10 tickets

| ID | Week | File to create | What it is |
|----|------|----------------|------------|
| S-01 | 1 | `math/cbf_pairwise.py` | Barrier function `h_ij`, gradient, Lie derivatives `Lf_h`, `Lg_h` |
| S-02 | 1 | `math/rulebook.py` | Three-level priority: Safety > Geofence > Progress |
| S-03 | 1 | `docs/cbf_derivation.ipynb` | Derivation notebook: control-affine → CBF → HOCBF → QP |
| S-04 | 2 | `cbf/qp_solver.py` | CVXPY QP: minimize deviation from MARL actions subject to CBF constraints |
| S-05 | 2 | `cbf/hocbf.py` | High-order CBF for relative degree 2 (position-controlled by acceleration) |
| S-06 | 2 | `cbf/shield_logger.py` | CSV log of every shield intervention — columns fixed in INTERFACES.md |
| S-07 | 3 | `analysis/latency_analysis.py` | Bound: at 2s delay + 15 m/s, how much must `d_safe` expand? |
| S-08 | 3 | `cbf/fallback_trigger.py` | If QP infeasible: hover in place, log as critical intervention |
| S-09 | 4 | `docs/blog_cbf_shield.md` | ~1500-word blog post on shield feasibility for Medium/Substack |
| S-10 | 4 | (review task) | Review and approve all math in Aman's white paper |

---

## The math you need (summary)

**Barrier function** (pairwise separation):
```
h_ij(x) = ||p_i - p_j||² − d_safe²
Safe set: { x : h_ij(x) ≥ 0 }
```

**Lie derivatives** for double integrator (ẍ = u):
```
Lf(h_ij) = 2(p_i − p_j)ᵀ(v_i − v_j)     ← no control input
Lg(h_ij)·u = 2(p_i − p_j)ᵀ(u_i − u_j)   ← control input appears here
```

**First-order CBF condition** (must hold every step):
```
Lf(h_ij) + Lg(h_ij)·u + α·h_ij ≥ 0
```

**Why HOCBF**: Position separation has relative degree 2 — `u` doesn't appear in `Lf(h)` directly. You need the HOCBF construction with auxiliary variable `ψ₁ = Lf(h) + α₁(h)`.

**The QP** (S-04):
```
minimize    Σᵢ ||u_i − û_i||²         (stay close to proposed MARL action)
subject to  for all pairs (i,j): Lf(h_ij) + Lg(h_ij)·u + α·h_ij ≥ 0
            ||u_i||∞ ≤ u_max
```

---

## What you receive and what you produce

**Receive from Anas**: `MARLAction` per drone — see `INTERFACES.md § MARL-ACTION`  
**Produce for Anas + Aman**: `SafeAction` per drone — see `INTERFACES.md § CBF-OUTPUT`

Your public function signature (agreed, do not deviate):
```
solve_cbf_qp(states, proposed_actions, d_safe=5.0, alpha=1.0) → list[SafeAction]
```

---

## LLM shortcut for S-04

Paste this into Claude or ChatGPT:
```
I am implementing a CBF-QP safety filter for a multi-drone simulation.
- N drones with (position, velocity), each 3D
- Barrier: h_ij = ||p_i-p_j||^2 - d_safe^2
- Control input: 3D acceleration u_i
- Lie derivatives: Lf_h = 2*(p_i-p_j)^T*(v_i-v_j), Lg_h_ui = 2*(p_i-p_j)
Write a Python function using CVXPY that:
1. Takes N (pos,vel) pairs and N proposed_accel arrays
2. Solves: min sum ||u_i - u_proposed_i||^2 s.t. all pairwise CBF constraints
3. Returns safe_accels, shield_active flags per agent, h_min value
4. Handles infeasibility gracefully — return zeros (hover) if OSQP fails
Use OSQP solver.
```

---

## Key papers

- Ames et al. (2017) "Control Barrier Functions for Safety Critical Systems" → https://arxiv.org/abs/1609.06408
- Xiao & Belta (2019) "High Order CBFs" → https://arxiv.org/abs/1903.04706

---

## Week 1 done means

- [ ] `cbf_pairwise.py` runs standalone, prints h_ij / Lf_h / Lg_h for two sample drones
- [ ] `rulebook.py` passes 3 hand-crafted scenarios
- [ ] `cbf_derivation.ipynb` renders clean, equations match the PDF
- [ ] PR opened into `dev`
