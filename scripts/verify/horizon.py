"""Check the adaptive backup horizon (66-109 steps) against a 300-step minimum: does it ever overestimate the min distance?"""
import numpy as np
from shared.contracts import DroneState
from utm_services.safety import filter as F

rng = np.random.default_rng(1)
cfg = F.FilterConfig()
print("steps:", F._backup_steps(cfg))


def rand_vel():
    v = rng.normal(size=3)
    return v / np.linalg.norm(v) * rng.uniform(0, 15) * 0.999999


bad, worst, n_trunc = 0, 0.0, 0
M = 4000
for trial in range(M):
    p = rng.uniform(-60, 60, 3); p[2] *= 0.3
    vi, vj = rand_vel(), rand_vel()
    if trial % 4 == 0:  # aimed head-on at max speed
        d = -p / np.linalg.norm(p); vi = d * 14.9999; vj = -d * 14.9999
    si = DroneState(agent_id="a", position_m=(500 + p[0], 500 + p[1], 100 + p[2]), velocity_mps=tuple(vi), goal_m=(900, 900, 100), timestamp_ms=0)
    sj = DroneState(agent_id="b", position_m=(500, 500, 100), velocity_mps=tuple(vj), goal_m=(900, 900, 100), timestamp_ms=0)
    st = {"a": si, "b": sj}
    orig = F._backup_steps
    m1, r1, *_ = F._backup_constraints(st, [("a", "b")], cfg)
    F._backup_steps = lambda c: 300
    m2, r2, *_ = F._backup_constraints(st, [("a", "b")], cfg)
    F._backup_steps = orig
    # rhs = m.a_first - g dt (d*-d_eff)/sens ; compare d* via rhs difference is messy -> recompute directly
    if not np.allclose(r1, r2, atol=1e-6) or not np.allclose(m1, m2, atol=1e-6):
        n_trunc += 1
        diff = float(r1[0] - r2[0])
        worst = max(worst, -diff)
        if diff < -1e-6:
            bad += 1  # short horizon gives a looser (smaller) rhs -> overestimates safety
print(f"{M} pairs: horizon-truncation changed the row in {n_trunc}; looser row in {bad}; worst rhs gap {worst:.4f} m/s^2")
