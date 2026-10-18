# salman/ — CBF Safety Layer
**Owner**: Salman | **Branch**: `salman/cbf-safety`

Your job is the **math core**. Everything you write becomes the formal safety guarantee in the white paper and the runtime filter in the simulation.

## Your tickets

| Ticket | Week | File | Status |
|--------|------|------|--------|
| S-01 | 1 | `math/cbf_pairwise.py` | Starter provided |
| S-02 | 1 | `math/rulebook.py` | Starter provided |
| S-03 | 1 | `docs/cbf_derivation.ipynb` | You create |
| S-04 | 2 | `cbf/qp_solver.py` | You create |
| S-05 | 2 | `cbf/hocbf.py` | You create |
| S-06 | 2 | `cbf/shield_logger.py` | You create |
| S-07 | 3 | `analysis/latency_analysis.py` | You create |
| S-08 | 3 | `cbf/fallback_trigger.py` | You create |
| S-09 | 4 | `docs/blog_cbf_shield.md` | You create |
| S-10 | 4 | Review Aman's white paper math | Review task |

## Directory layout

```
salman/
├── math/
│   ├── cbf_pairwise.py   ← S-01: barrier function h_ij, Lf_h, Lg_h
│   └── rulebook.py       ← S-02: lexicographic safety rules
├── cbf/
│   ├── qp_solver.py      ← S-04: CVXPY-based CBF-QP
│   ├── hocbf.py          ← S-05: high-order CBF for acceleration inputs
│   ├── shield_logger.py  ← S-06: CSV logging of every intervention
│   └── fallback_trigger.py ← S-08: HJ reachability fallback
├── analysis/
│   └── latency_analysis.py ← S-07: delay margin bounds
└── docs/
    ├── cbf_derivation.ipynb ← S-03: derivation notebook
    └── blog_cbf_shield.md   ← S-09: Medium/Substack post
```

## Interface you must honor

**Input you receive** (from Anas): `MARLAction` — see `INTERFACES.md § MARL-ACTION`
**Output you produce** (to Anas + Aman): `SafeAction` — see `INTERFACES.md § CBF-OUTPUT`

The CBF-QP solver signature must be:
```python
def solve_cbf_qp(
    states: list[DroneState],
    proposed_actions: list[MARLAction],
    d_safe: float = 5.0,
    alpha: float = 1.0,
) -> list[SafeAction]:
    ...
```

## Key math references

- CBF paper: https://arxiv.org/abs/1609.06408
- HOCBF paper: https://arxiv.org/abs/1903.04706
- CVXPY docs: https://www.cvxpy.org/

## LLM prompt for S-04

Copy this into Claude/ChatGPT to get a working first draft:

```
Write a Python function using CVXPY that:
1. Takes N DroneState objects and N MARLAction objects
2. Solves a QP: minimize sum of ||u_i - u_proposed_i||² over all agents
3. Subject to: for each pair (i,j), Lf_h_ij + Lg_h_ij @ [u_i, u_j] + alpha * h_ij >= 0
4. Where h_ij = ||p_i - p_j||² - d_safe², Lf and Lg are provided by cbf_pairwise.py
5. Returns list of SafeAction with shield_active=True if any u_i != u_proposed_i
Use CVXPY with SCS or OSQP solver. Handle infeasibility gracefully.
```

## Week 1 definition of done

- [ ] `cbf_pairwise.py` self-test passes (`python -m salman.math.cbf_pairwise`)
- [ ] `rulebook.py` self-test passes (`python -m salman.math.rulebook`)
- [ ] `cbf_derivation.ipynb` renders clean
- [ ] PR opened into `dev` with all three files
