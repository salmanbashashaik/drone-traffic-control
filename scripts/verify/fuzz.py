"""Closed-loop fuzz of apply_filter on random 2-6 drone encounters.

Nominal = zero acceleration (hold velocity: the worst case for a collision course).
Dynamics replicate DroneEnv._advance (ZOH, speed cap, trapezoid).
"""
import json
import logging
import os
import sys
HARD = os.environ.get('HARD') == '1'
from multiprocessing import Pool

import numpy as np
from shared.contracts import MAX_SPEED_MPS, DroneState, MARLAction
from utm_services.safety.filter import FilterConfig, apply_filter

logging.disable(logging.CRITICAL)
DT, D_EFF = 0.1, 5.5


def encounter(seed):
    rng = np.random.default_rng(seed)
    n = int(rng.integers(2, 7))
    c = np.array([rng.uniform(250, 750), rng.uniform(250, 750), rng.uniform(70, 160)])
    for _ in range(200):
        p, v = [], []
        for _ in range(n):
            az, el = rng.uniform(0, 2 * np.pi), rng.uniform(-0.5, 0.5)
            d = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
            speed = rng.uniform(2, 15) * 0.9999
            ta = rng.uniform(1.5, 4) if HARD else rng.uniform(3, 7)
            miss = rng.normal(size=3) * rng.uniform(0, 6)
            v.append(d * speed)
            p.append(c + miss - d * speed * ta)
        p, v = np.array(p), np.array(v)
        if all(30.5 < z < 199.5 for z in p[:, 2]):
            dmin = min(np.linalg.norm(p[i] - p[j]) for i in range(n) for j in range(i + 1, n))
            if dmin >= (8.0 if HARD else 15.0):
                return p, v
    return None


def run(args):
    seed, barrier, steps = args
    enc = encounter(seed)
    if enc is None:
        return {"seed": seed, "skipped": True}
    p, v = enc
    n = len(p)
    ids = [f"d{i}" for i in range(n)]
    cfg = FilterConfig(barrier=barrier)
    out = {"seed": seed, "n": n, "fallback_steps": 0, "solver_err_steps": 0, "min_sep": 1e9,
           "exc": None, "first_inside": None, "max_depth": 0.0}
    traj = []
    for k in range(steps):
        ts = k * 100
        try:
            states = [DroneState(agent_id=ids[i], position_m=tuple(p[i]), velocity_mps=tuple(v[i]),
                                 goal_m=(900.0, 900.0, 100.0), timestamp_ms=ts) for i in range(n)]
        except Exception as e:  # noqa: BLE001
            out["exc"] = f"state@{k}: {str(e).splitlines()[-1][:120]}"
            break
        acts = [MARLAction(agent_id=i, proposed_accel_mps2=(0.0, 0.0, 0.0), timestamp_ms=ts) for i in ids]
        try:
            safe, diag = apply_filter(states, acts, cfg)
        except Exception as e:  # noqa: BLE001
            out["exc"] = f"filter@{k}: {type(e).__name__}: {e}"[:200]
            break
        out["fallback_steps"] += int(diag.fallback_used)
        out["solver_err_steps"] += int(not diag.qp_feasible)
        a = np.array([s.safe_accel_mps2 for s in sorted(safe, key=lambda s: s.agent_id)])
        assert np.all(np.abs(a) <= 3.5 + 1e-9)
        vn = v + a * DT
        sp = np.linalg.norm(vn, axis=1)
        vn = np.where((sp > MAX_SPEED_MPS)[:, None], vn * (MAX_SPEED_MPS / sp * (1 - 1e-12))[:, None], vn)
        p = p + 0.5 * (v + vn) * DT
        v = vn
        traj.append(a.tolist())
        d = min(np.linalg.norm(p[i] - p[j]) for i in range(n) for j in range(i + 1, n))
        out["min_sep"] = min(out["min_sep"], d)
        if d < D_EFF and out["first_inside"] is None:
            out["first_inside"] = k + 1
        out["max_depth"] = max(out["max_depth"], D_EFF - d)
    out["hash"] = hash(np.round(np.array(traj), 12).tobytes()) if traj else None
    return out


if __name__ == "__main__":
    n_runs, barrier, steps, procs = int(sys.argv[1]), sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
    start = int(sys.argv[5]) if len(sys.argv) > 5 else 0
    with Pool(procs) as pool:
        res = pool.map(run, [(s, barrier, steps) for s in range(start, start + n_runs)], chunksize=4)
    out_dir = os.environ.get("FUZZ_OUT", ".")  # raw results; default cwd, gitignored
    json.dump(res, open(os.path.join(out_dir, f"fuzz_{barrier}_{start}_{n_runs}{'_hard' if HARD else ''}.json"), "w"))
    ok = [r for r in res if not r.get("skipped")]
    print(f"{barrier}: {len(ok)} encounters ({len(res) - len(ok)} skipped)")
    print(" by n:", {k: sum(r['n'] == k for r in ok) for k in range(2, 7)})
    print(" fallback runs:", [r["seed"] for r in ok if r["fallback_steps"]])
    print(" solver-error runs:", [r["seed"] for r in ok if r["solver_err_steps"]])
    print(" exceptions:", [(r["seed"], r["exc"]) for r in ok if r["exc"]][:20], sum(bool(r["exc"]) for r in ok))
    bad = sorted([r for r in ok if r["min_sep"] < D_EFF], key=lambda r: r["min_sep"])
    print(f" runs inside d_eff: {len(bad)}; below 5 m: {sum(r['min_sep'] < 5.0 for r in bad)}")
    for r in bad[:30]:
        print(f"   seed {r['seed']} n={r['n']} min_sep={r['min_sep']:.3f} first_inside_step={r['first_inside']}")
