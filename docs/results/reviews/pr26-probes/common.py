"""PR 26 review probe, see docs/results/reviews/pr26-review-2026-10-24.md. Run from this directory with the project installed."""
import numpy as np
from shared.contracts import DroneState, MARLAction
from utm_services.safety import filter as F

def ring(centre, radius, speed, count, prefix, z_jitter=0.05, phase=0.0):
    states, actions = [], []
    for k in range(count):
        ang = phase + 2 * np.pi * k / count
        d = np.array([np.cos(ang), np.sin(ang), z_jitter * ((k % 3) - 1)])
        p = np.array(centre) + radius * d
        v = -speed * d / np.linalg.norm(d)
        aid = f"{prefix}{k:02d}"
        states.append(DroneState(agent_id=aid, position_m=tuple(p), velocity_mps=tuple(v),
                                 goal_m=(900.0, 900.0, 100.0), timestamp_ms=0))
        actions.append(MARLAction(agent_id=aid, proposed_accel_mps2=(0.0, 0.0, 0.0), timestamp_ms=0))
    return states, actions

def qp_of(states, actions, config=None, escape=False):
    config = config or F.FilterConfig(step_deadline=False)
    smap = {s.agent_id: s for s in states}; amap = {a.agent_id: a for a in actions}
    ids = sorted(smap)
    pairs = [(a, b) for i, a in enumerate(ids) for b in ids[i+1:]]
    frames = F._update_latches(smap, pairs, config, {})
    backup = F._backup_constraints(smap, pairs, config, frames)
    signs = None
    if escape:
        signs = F._escape_signs(smap, pairs[: max(1, len(pairs) // 4)], frames)
    P, q, A, l, u, aid = F.build_hocbf_qp_matrices(smap, amap, pairs, config, signs, backup, frames, False)
    return P, q, A, l, u, aid, pairs
