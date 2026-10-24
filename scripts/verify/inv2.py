"""Parallel version of invariance.py: per-step decay and forward invariance of the backup barrier, 2-drone closed loop."""
import logging, os, sys
from multiprocessing import Pool
import numpy as np
src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "invariance.py")).read()
exec(src[:src.index("rng = np.random")])  # U, DT, h_backup, step, rvel, imports


def trial(t):
    rng = np.random.default_rng([7, t])
    rel = rng.normal(size=3); rel[2] *= 0.3
    rel = rel / np.linalg.norm(rel) * rng.uniform(6, 60)
    pi, pj = np.array([500, 500, 100.0]) + rel, np.array([500, 500, 100.0])
    vi, vj = rvel(rng), rvel(rng)
    if t % 3 == 0:
        u = -rel / np.linalg.norm(rel); vi, vj = u * rng.uniform(5, 15) * 0.9999, -u * rng.uniform(5, 15) * 0.9999
    h = h_backup(pi, pj, vi, vj)
    if h < 0:
        return None
    p_, v_ = [pi, pj], [vi, vj]
    nom = [tuple(np.r_[rng.uniform(-3.5, 3.5, 2), rng.uniform(-0.5, 0.5)]) for _ in range(2)]
    cfg = FilterConfig(step_deadline=False)
    n_tests = n_viol = 0; worst = 0.0; entered = None; minsep = 1e9
    latch = {}  # one per trial, as the runner keeps one per episode
    for k in range(80):
        try:
            st = [DroneState(agent_id=f"d{i}", position_m=tuple(p_[i]), velocity_mps=tuple(v_[i]), goal_m=(900, 900, 100), timestamp_ms=k * 100) for i in range(2)]
            ac = [MARLAction(agent_id=f"d{i}", proposed_accel_mps2=nom[i], timestamp_ms=k * 100) for i in range(2)]
            safe, diag = apply_filter(st, ac, cfg, latch=latch)
        except Exception:
            break
        a = [np.array(s.safe_accel_mps2) for s in sorted(safe, key=lambda s: s.agent_id)]
        for i in range(2):
            p_[i], v_[i] = step(p_[i], v_[i], a[i])
        hn = h_backup(p_[0], p_[1], v_[0], v_[1])
        if not diag.pair_slack_mps2 and not diag.fallback_used:
            n_tests += 1
            gap = (1 - cfg.gamma * DT) * h - hn
            if gap > 1e-3:
                n_viol += 1; worst = max(worst, gap)
        d = float(np.linalg.norm(p_[0] - p_[1])); minsep = min(minsep, d)
        if d < D_EFF and entered is None:
            entered = k
        h = hn
    return t, n_tests, n_viol, worst, entered, minsep


if __name__ == "__main__":
    N = int(sys.argv[1])
    with Pool(10) as pool:
        res = [r for r in pool.map(trial, range(N), chunksize=8) if r]
    print(f"{len(res)} of {N} trials start with h>=0")
    print(f"per-step decay: {sum(r[2] for r in res)}/{sum(r[1] for r in res)} slack-free steps violate h_next >= (1-g dt) h; worst gap {max(r[3] for r in res):.4f} m")
    bad = [r for r in res if r[4] is not None]
    print(f"forward invariance: {len(bad)}/{len(res)} entered d_eff", [(r[0], r[4], round(r[5], 3)) for r in bad][:20])
    print("decay violators:", [(r[0], r[2], round(r[3], 3)) for r in res if r[2]][:20])
