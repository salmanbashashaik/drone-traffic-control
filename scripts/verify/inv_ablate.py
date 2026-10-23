"""Ablation of invariance.py: script h vs the filter's own backup h, with and without a latch.

Usage: python inv_ablate.py {none|persist} N_TRIALS   (ONLY=207,1260 limits to those trials)
Same seed (7) and trial sampling as invariance.py. "none" calls apply_filter without a latch, as
invariance.py does; "persist" keeps one latch per trial, as the simulation runner does.
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



import copy, os, sys
from utm_services.safety import filter as F
MODE = sys.argv[1]          # "none" or "persist"
N = int(sys.argv[2])
PAIR = ("d0", "d1")

def h_filter(pi, pj, vi, vj, frames):
    """Filter's own backup margin with the given frames (min |r| - d_eff)."""
    fr = frames or {}
    axis = F._latch_axis(fr, PAIR).reshape(1, 3)
    up = np.array([fr[PAIR].up if PAIR in fr else 0.0])
    traj = F._backup_rollout(pi[None], pj[None], vi[None], vj[None], cfg, axis, up)[0]
    return float(np.nanmin(np.linalg.norm(traj, axis=2))) - D_EFF

rng = np.random.default_rng(7)
cfg = FilterConfig()
stats = {"script": [0, 0, 0.0], "filter": [0, 0, 0.0]}  # tests, viol, worst
n_inv_fail = tested_inv = 0
worst_at = None
for trial in range(N):
    rel = rng.normal(size=3); rel[2] *= 0.3
    rel = rel / np.linalg.norm(rel) * rng.uniform(6, 60)
    pi, pj = np.array([500, 500, 100.0]) + rel, np.array([500, 500, 100.0])
    vi, vj = rvel(rng), rvel(rng)
    if trial % 3 == 0:
        u = -rel / np.linalg.norm(rel); vi, vj = u * rng.uniform(5, 15) * 0.9999, -u * rng.uniform(5, 15) * 0.9999
    h0 = h_backup(pi, pj, vi, vj)
    if h0 < 0:
        continue
    p_, v_ = [pi, pj], [vi, vj]
    nom = [tuple(np.r_[rng.uniform(-3.5, 3.5, 2), rng.uniform(-0.5, 0.5)]), tuple(np.r_[rng.uniform(-3.5, 3.5, 2), rng.uniform(-0.5, 0.5)])]
    if os.environ.get('ONLY') and trial not in map(int, os.environ['ONLY'].split(',')):
        continue
    h = h0
    tested_inv += 1
    failed = False
    latch = {} if MODE == "persist" else None
    for k in range(80):
        try:
            st = [DroneState(agent_id=f"d{i}", position_m=tuple(p_[i]), velocity_mps=tuple(v_[i]),
                             goal_m=(900, 900, 100), timestamp_ms=k * 100) for i in range(2)]
            ac = [MARLAction(agent_id=f"d{i}", proposed_accel_mps2=nom[i], timestamp_ms=k * 100) for i in range(2)]
            smap = {s.agent_id: s for s in st}
            frames = F._update_latches(smap, [PAIR], cfg, copy.deepcopy(latch) if latch is not None else {})
            hf = h_filter(p_[0], p_[1], v_[0], v_[1], frames)
            safe, diag = apply_filter(st, ac, cfg, latch=latch if latch is not None else {})
        except Exception:
            break
        a = [np.array(s.safe_accel_mps2) for s in safe]
        for i in range(2):
            p_[i], v_[i] = step(p_[i], v_[i], a[i])
        h_next = h_backup(p_[0], p_[1], v_[0], v_[1])
        hf_next = h_filter(p_[0], p_[1], v_[0], v_[1], frames)  # same frame the QP used
        if not diag.pair_slack_mps2:
            for key, (hh, hn) in (("script", (h, h_next)), ("filter", (hf, hf_next))):
                s = stats[key]
                s[0] += 1
                gap = (1 - cfg.gamma * DT) * hh - hn
                if gap > 1e-3:
                    s[1] += 1
                    if gap > s[2]:
                        s[2] = gap
                        if key == "filter":
                            worst_at = (trial, k)
        if np.linalg.norm(p_[0] - p_[1]) < D_EFF and not failed:
            n_inv_fail += 1
            failed = True
            print(f"  trial {trial}: entered d_eff at step {k}, sep {np.linalg.norm(p_[0]-p_[1]):.3f}", flush=True)
        h = h_next

for key, (t, v, w) in stats.items():
    print(f"{MODE} h={key}: {v}/{t} slack-free steps violate; worst gap {w:.4f} m")
print(f"{MODE} filter-h worst at {worst_at}")
print(f"{MODE} forward invariance: {n_inv_fail}/{tested_inv} entered d_eff")
