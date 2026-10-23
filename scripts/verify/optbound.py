"""Independent max-min-separation optimiser over piecewise-constant accels.

Dynamics = simulator: 100 ms ZOH, trapezoid position (speed cap checked post hoc).
p_K = p0 + K dt v0 + sum_{j<K} (K - j - 0.5) dt^2 a_j
"""
import sys
import numpy as np
from scipy.optimize import minimize

U, DT = 3.5, 0.1


def build(p0, v0, T, sub=1):
    """Return function positions(a) at times K*dt/sub, K=1..T*sub, linear in a (T, N, 3)."""
    times = np.arange(1, T * sub + 1) * DT / sub
    # coefficient of a_j at time t: if t <= j dt: 0; elif t < (j+1)dt: 0.5 (t-j dt)^2; else (t-(j+0.5)dt) dt
    C = np.zeros((len(times), T))
    for i, t in enumerate(times):
        for j in range(T):
            t0 = j * DT
            if t <= t0:
                continue
            C[i, j] = 0.5 * (t - t0) ** 2 if t < t0 + DT else (t - t0 - 0.5 * DT) * DT
    base = p0[None] + times[:, None, None] * v0[None]
    return times, C, base


def solve(p0, v0, T, pairs, starts=8, seed=0, sub=1, init=None):
    N = len(p0)
    times, C, base = build(p0, v0, T, sub)
    nv = T * N * 3

    def pos(x):
        a = x[:nv].reshape(T, N, 3)
        return base + np.einsum("tj,jnc->tnc", C, a)

    def cons(x):
        P = pos(x)
        out = []
        for i, j in pairs:
            out.append(np.linalg.norm(P[:, i] - P[:, j], axis=1) - x[-1])
        return np.concatenate(out)

    def cons_jac(x):
        P = pos(x)
        rows = []
        for i, j in pairs:
            r = P[:, i] - P[:, j]
            d = np.maximum(np.linalg.norm(r, axis=1), 1e-9)
            g = r / d[:, None]  # (Tt,3)
            J = np.zeros((len(times), T, N, 3))
            J[:, :, i, :] = C[:, :, None] * g[:, None, :]
            J[:, :, j, :] = -C[:, :, None] * g[:, None, :]
            rows.append(np.concatenate([J.reshape(len(times), nv), -np.ones((len(times), 1))], axis=1))
        return np.vstack(rows)

    rng = np.random.default_rng(seed)
    best = None
    for s in range(starts):
        if init is not None and s == 0:
            x0 = np.append(init, 0.0)
        else:
            x0 = np.append(rng.uniform(-U, U, nv), 0.0)
        res = minimize(lambda x: -x[-1], x0, jac=lambda x: np.eye(1, nv + 1, nv)[0] * -1,
                       bounds=[(-U, U)] * nv + [(None, None)],
                       constraints=[{"type": "ineq", "fun": cons, "jac": cons_jac}],
                       method="SLSQP", options={"maxiter": 500, "ftol": 1e-9})
        x = res.x
        P = pos(x)
        val = min(np.linalg.norm(P[:, i] - P[:, j], axis=1).min() for i, j in pairs)
        if best is None or val > best[0]:
            best = (val, x)
    return best, pos, times


def speeds(x, v0, T, N):
    a = x[:T * N * 3].reshape(T, N, 3)
    v = v0[None] + np.cumsum(a, axis=0) * DT
    return np.linalg.norm(v, axis=2).max()


def cont_min(x, p0, v0, T, pairs):
    """Min separation evaluated on 20 sub-samples per step (continuous check)."""
    times, C, base = build(p0, v0, T, sub=20)
    a = x[:T * len(p0) * 3].reshape(T, len(p0), 3)
    P = base + np.einsum("tj,jnc->tnc", C, a)
    return min(np.linalg.norm(P[:, i] - P[:, j], axis=1).min() for i, j in pairs)


if __name__ == "__main__":
    what = sys.argv[1]
    if what == "sandwich":
        p0 = np.array([[490.0, 500, 100], [500, 500, 100], [510, 500, 100]])
        v0 = np.array([[10.0, 0, 0], [0, 0, 0], [-10, 0, 0]])
        T = 25
        starts = int(sys.argv[2]) if len(sys.argv) > 2 else 40
        for label, pairs in (("outer pairs only", [(0, 1), (1, 2)]), ("all 3 pairs", [(0, 1), (1, 2), (0, 2)])):
            (val, x), pos, _ = solve(p0, v0, T, pairs, starts=starts)
            allp = [(0, 1), (1, 2), (0, 2)]
            P = pos(x)
            all_min = min(np.linalg.norm(P[:, i] - P[:, j], axis=1).min() for i, j in allp)
            print(f"{label}: best sampled min over its pairs = {val:.4f} m; over all 3 pairs = {all_min:.4f} m; "
                  f"continuous min (its pairs) = {cont_min(x, p0, v0, T, pairs):.4f} m; max speed {speeds(x, v0, T, 3):.2f}")
            a = x[:-1].reshape(T, 3, 3)
            print("  first-step accels:", np.round(a[0], 2).tolist())
    elif what == "headon":
        # minimal D such that max-min separation >= d_eff, 30 m/s closing.
        d_eff = 5.5
        T = 70
        def best_sep(D, axes):
            p0 = np.array([[0.0, 0, 100], [D, 0, 100]])
            v0 = np.array([[15.0, 0, 0], [-15, 0, 0]])
            # restrict axes by bounds trick: zero out disallowed components via penalty -> simpler: project
            (val, x), _, _ = solve(p0, v0, T, [(0, 1)], starts=3, seed=1)
            return val
        lo, hi = 15.0, 80.0
        for _ in range(14):
            mid = 0.5 * (lo + hi)
            if best_sep(mid, None) >= d_eff:
                hi = mid
            else:
                lo = mid
        print(f"head-on 30 m/s: optimised minimal D for d_eff=5.5 (all axes, free control) ~ {hi:.2f} m")
