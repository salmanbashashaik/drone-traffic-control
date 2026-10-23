"""Recoverability of HARD fuzz runs that entered d_eff: best achievable min separation from the initial state."""
import itertools, logging, os, sys
import numpy as np
os.environ["HARD"] = "1"
logging.disable(logging.CRITICAL)
import fuzz, optbound
T = 30
for seed in map(int, sys.argv[1:]):
    p, v = fuzz.encounter(seed)
    n = len(p)
    allp = list(itertools.combinations(range(n), 2))
    # pairs whose straight-line CPA within 3 s is < 15 m: the only ones that can bind
    near = []
    for i, j in allp:
        r, w = p[i] - p[j], v[i] - v[j]
        t = np.clip(-r @ w / max(w @ w, 1e-9), 0, 3)
        if np.linalg.norm(r + w * t) < 15:
            near.append((i, j))
    pw = min(optbound.solve(p[[i, j]], v[[i, j]], T, [(0, 1)], starts=4)[0][0] for i, j in near)
    (val, x), _, _ = optbound.solve(p, v, T, near, starts=6)
    sp = optbound.speeds(x, v, T, n)
    print(f"seed {seed} n={n} near={len(near)} best pairwise-limited={pw:.3f} best joint={val:.3f} (max speed {sp:.1f}) -> {'RECOVERABLE' if val >= 5.5 else 'not recoverable'}", flush=True)
