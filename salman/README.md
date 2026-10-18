# salman/ — Your Work
**Branch**: `salman/cbf-safety` | **Role**: RL + Formal Safety Math

You own both sides of the safety problem: **you train the RL policy** and **you build the CBF shield that makes it provably safe**. This is actually the strongest position in the team — you understand what the policy is trying to do and you design the constraint that keeps it honest. Your work is the core scientific contribution of this project.

---

## Your 13 tickets

| ID | Week | Deliverable | What it is |
|----|------|-------------|------------|
| S-01 | 1 | `math/cbf_pairwise.py` | Barrier function `h_ij`, gradient, Lie derivatives |
| S-02 | 1 | `math/rulebook.py` | Safety > Geofence > Progress priority encoder |
| S-03 | 1 | `docs/cbf_derivation.ipynb` | Full CBF-QP derivation notebook |
| S-04 | 1 | `rl/training_spec.md` | RL architecture + reward design spec (reviewed by Anas) |
| S-05 | 2 | `cbf/qp_solver.py` | CVXPY QP solver — filters unsafe MARL actions |
| S-06 | 2 | `cbf/hocbf.py` | High-order CBF for acceleration-controlled drones |
| S-07 | 2 | `cbf/shield_logger.py` | CSV log of every intervention |
| S-08 | 2 | `rl/train_baseline.py` + checkpoint | Baseline PPO policy, no shield, 500k steps |
| S-09 | 3 | `rl/train_shielded.py` + checkpoint | Shielded policy, reduced intervention rate |
| S-10 | 3 | `analysis/latency_analysis.py` | Bound: how much must d_safe grow under 2s comm delay? |
| S-11 | 3 | `cbf/fallback_trigger.py` | Hover fallback when QP is infeasible |
| S-12 | 4 | `docs/blog_cbf_rl.md` | Blog: combining RL + CBF, the feasibility problem |
| S-13 | 4 | (review task) | Approve all math in Aman's white paper |

---

## Directory layout

```
salman/
├── math/
│   ├── cbf_pairwise.py       ← S-01
│   └── rulebook.py           ← S-02
├── cbf/
│   ├── qp_solver.py          ← S-05
│   ├── hocbf.py              ← S-06
│   ├── shield_logger.py      ← S-07
│   └── fallback_trigger.py   ← S-11
├── rl/
│   ├── training_spec.md      ← S-04
│   ├── train_baseline.py     ← S-08
│   └── train_shielded.py     ← S-09
├── checkpoints/              ← .pt files (gitignored)
├── analysis/
│   └── latency_analysis.py   ← S-10
└── docs/
    ├── cbf_derivation.ipynb  ← S-03
    └── blog_cbf_rl.md        ← S-12
```

---

## The RL side (S-04, S-08, S-09)

You train a **PPO policy** (or IPPO — independent PPO per drone) using the environment Anas builds (`anas/env/drone_env.py`). You own the training script; Anas owns the environment.

**Training spec (S-04)** — write this first, coordinate with Anas:
- Policy network: shared trunk or independent per agent?
- Observation layout: must match `INTERFACES.md § ENV-OUTPUT` exactly (18 dims)
- Reward function: document every term and weight
- Framework: TorchRL or RLlib — pick one, commit to it
- W&B project/run naming convention

**Baseline training (S-08)** — train WITHOUT the shield first. This is intentional. You need to show collisions happening so the shield's value is clear in the demo.

**Shielded training (S-09)** — wrap the env in Anas's `ShieldedEnv`, add a small penalty for triggering the shield (`-0.5 × ||safe_accel - proposed_accel||`), retrain. The policy learns to propose safer actions so the shield fires less. Compare intervention rate on W&B — this comparison is a key result in the paper.

---

## The CBF side (S-01, S-05, S-06, S-11)

**Barrier function** (pairwise separation):
```
h_ij(x) = ||p_i − p_j||² − d_safe²
Safe set: { x : h_ij ≥ 0 }
```

**Lie derivatives** (double integrator, ẍ = u):
```
Lf(h_ij) = 2(p_i − p_j)ᵀ(v_i − v_j)      ← no u here
Lg(h_ij) · u = 2(p_i − p_j)ᵀ(u_i − u_j)  ← u appears here
```

**Why HOCBF**: with acceleration input, position separation is relative degree 2 — `u` doesn't appear in `Lf(h)`. You need auxiliary variable `ψ₁ = Lf(h) + α₁(h)` and constrain `Lf(ψ₁) + Lg(ψ₁)u + α₂(ψ₁) ≥ 0`.

**The QP** (S-05):
```
minimize    Σᵢ ||u_i − û_i||²
subject to  for all pairs (i,j): CBF constraint ≥ 0
            ||u_i||∞ ≤ u_max
```
Use CVXPY + OSQP. If infeasible: return `[0,0,0]` (hover), set `qp_feasible=False`.

---

## What you receive and what you produce

**Receive from Anas**: `DroneState` list (from env), `MARLAction` list (from policy output)  
**Produce for Anas**: `SafeAction` list — see `INTERFACES.md § CBF-OUTPUT`

Your public solver signature (do not deviate):
```
solve_cbf_qp(states, proposed_actions, d_safe=5.0, alpha=1.0) → list[SafeAction]
```

---

## LLM shortcut for the QP (S-05)

```
I am implementing a CBF-QP safety filter for multi-drone control in Python.
- N drones, each with position and velocity (3D)
- Barrier: h_ij = ||p_i - p_j||^2 - d_safe^2
- Control input: 3D acceleration u_i
- Lf_h = 2*(p_i-p_j)^T*(v_i-v_j), Lg_h_ui = 2*(p_i-p_j)
Write a Python function using CVXPY that:
1. Takes N (pos, vel) pairs and N proposed_accel arrays
2. Solves: min Σ||u_i - u_proposed_i||^2 s.t. all pairwise CBF constraints
3. Returns safe_accels, shield_active flags, h_min value
4. Gracefully handles infeasibility (return zeros / hover)
Use OSQP solver.
```

---

## Key references

- Ames et al. 2017 — CBF-QP: https://arxiv.org/abs/1609.06408
- Xiao & Belta 2019 — HOCBF: https://arxiv.org/abs/1903.04706
- CVXPY docs: https://www.cvxpy.org
- TorchRL docs: https://pytorch.org/rl

---

## Week 1 done means

- [ ] `cbf_pairwise.py` runs, prints correct h/Lf/Lg for sample inputs
- [ ] `rulebook.py` passes 3 test cases
- [ ] `cbf_derivation.ipynb` renders clean
- [ ] `training_spec.md` written and reviewed by Anas
- [ ] PR into `dev`
