"""Does the backup row deliver h_next >= (1 - gamma dt) h for an isolated pair?

Independent re-implementation of the backup rollout (evasive manoeuvre, both drones,
100 ms ZOH, speed cap, trapezoid), then: apply_filter on a random 2-drone state with a
random nominal -> step the true dynamics -> recompute h. Also: is h >= 0 forward invariant
(does a pair that starts with h >= 0 ever end inside d_eff)?
"""
import logging
import math

import numpy as np
from shared.contracts import DroneState, MARLAction
from utm_services.safety.filter import FilterConfig, apply_filter

logging.disable(logging.CRITICAL)
U, DT, VMAX, D_EFF = 3.5, 0.1, 15.0, 5.5
STEPS = math.ceil((math.sqrt(2) * VMAX / U + 0.5) / DT)


def evasive(n, up):
    lat = np.cross((0, 0, 1.0), n)
    if np.linalg.norm(lat) < 1e-3:
        lat = np.cross((1.0, 0, 0), n)
    lat /= np.linalg.norm(lat)
    d = n + lat
    d[2] += up
    return d * (U / np.abs(d).max())


def cap(v):
    s = np.linalg.norm(v)
    return v * min(1.0, VMAX / max(s, 1e-9))


def h_backup(pi, pj, vi, vj):
    r, vi, vj = pi - pj, vi.copy(), vj.copy()
    best = np.inf
    for _ in range(STEPS):
        n = r / np.linalg.norm(r)
        a = evasive(n, np.sign(n[2]) if abs(n[2]) > 1e-3 else 1.0)  # i is the lower id
        vin, vjn = cap(vi + a * DT), cap(vj - a * DT)
        r = r + 0.5 * (vi + vin - vj - vjn) * DT
        vi, vj = vin, vjn
        best = min(best, np.linalg.norm(r))
    return best - D_EFF


def step(p, v, a):
    vn = v + a * DT
    s = np.linalg.norm(vn)
    if s > VMAX:
        vn *= VMAX / s * (1 - 1e-12)
    return p + 0.5 * (v + vn) * DT, vn


def rvel(rng):
    d = rng.normal(size=3)
    return d / np.linalg.norm(d) * rng.uniform(0, 15) * 0.9999


rng = np.random.default_rng(7)
cfg = FilterConfig()
n_tests, n_viol, worst, n_inv_fail, tested_inv = 0, 0, 0.0, 0, 0
for trial in range(3000):
    rel = rng.normal(size=3); rel[2] *= 0.3
    rel = rel / np.linalg.norm(rel) * rng.uniform(6, 60)
    pi, pj = np.array([500, 500, 100.0]) + rel, np.array([500, 500, 100.0])
    vi, vj = rvel(rng), rvel(rng)
    if trial % 3 == 0:
        u = -rel / np.linalg.norm(rel); vi, vj = u * rng.uniform(5, 15) * 0.9999, -u * rng.uniform(5, 15) * 0.9999
    h0 = h_backup(pi, pj, vi, vj)
    if h0 < 0:
        continue
    # closed loop: 80 steps with a random constant nominal; check h stays >= 0 and per-step decay
    p_, v_ = [pi, pj], [vi, vj]
    nom = [tuple(np.r_[rng.uniform(-3.5, 3.5, 2), rng.uniform(-0.5, 0.5)]), tuple(np.r_[rng.uniform(-3.5, 3.5, 2), rng.uniform(-0.5, 0.5)])]
    h = h0
    tested_inv += 1
    failed = False
    for k in range(80):
        try:
            st = [DroneState(agent_id=f"d{i}", position_m=tuple(p_[i]), velocity_mps=tuple(v_[i]),
                             goal_m=(900, 900, 100), timestamp_ms=k * 100) for i in range(2)]
            ac = [MARLAction(agent_id=f"d{i}", proposed_accel_mps2=nom[i], timestamp_ms=k * 100) for i in range(2)]
            safe, diag = apply_filter(st, ac, cfg)
        except Exception:
            break  # left the world box
        a = [np.array(s.safe_accel_mps2) for s in safe]
        for i in range(2):
            p_[i], v_[i] = step(p_[i], v_[i], a[i])
        h_next = h_backup(p_[0], p_[1], v_[0], v_[1])
        if not diag.pair_slack_mps2:
            n_tests += 1
            gap = (1 - cfg.gamma * DT) * h - h_next
            if gap > 1e-3:
                n_viol += 1
                worst = max(worst, gap)
        if np.linalg.norm(p_[0] - p_[1]) < D_EFF and not failed:
            n_inv_fail += 1
            failed = True
            print(f"  trial {trial}: entered d_eff at step {k}, sep {np.linalg.norm(p_[0]-p_[1]):.3f} (h0={h0:.2f})")
        h = h_next

print(f"per-step decay condition: {n_viol}/{n_tests} slack-free steps violate h_next >= (1-g dt) h; worst gap {worst:.4f} m")
print(f"forward invariance: {n_inv_fail}/{tested_inv} pairs starting with h>=0 entered d_eff")
