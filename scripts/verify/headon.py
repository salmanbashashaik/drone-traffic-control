"""Recompute P2-SA-02 §2 head-on gaps (30 m/s closing, d_eff 5.5 m)."""
import numpy as np
from scipy.optimize import brentq, minimize

U, D_EFF, V = 3.5, 5.5, 30.0
t = np.linspace(0, 10, 200001)


def min_sep(D, ax, lat_axes):
    """Closed form: relative x decel ax (held at min once closing stops), lateral 2U per axis."""
    if ax > 0:
        ts = V / ax
        x = np.where(t < ts, D - V * t + 0.5 * ax * t**2, D - V**2 / (2 * ax))
    else:
        x = D - V * t
    lat = 0.5 * 2 * U * np.sqrt(lat_axes) * t**2
    return np.hypot(x, lat).min()


rows = [("brake only", 2 * U, 0), ("split y, no brake", 0, 1), ("split y+z, no brake", 0, 2),
        ("brake + split y", 2 * U, 1), ("brake + split y+z", 2 * U, 2)]
print("brake only analytic: d_eff + V^2/(4U) =", D_EFF + V**2 / (4 * U))
print("brake only c=1:", D_EFF + (V**2 + 2 * 1.0 * V) / (4 * U))
for name, ax, la in rows:
    D = brentq(lambda D: min_sep(D, ax, la) - D_EFF, 5.6, 200)
    print(f"{name:22s} D_min = {D:.2f} m")

# Optimiser: relative dynamics (a_rel = a1 - a2, |component| <= 2U), ZOH 100 ms, free control.
DT, T = 0.1, 50
K = np.arange(1, T + 1)
C = np.array([[max(0.0, (k - j - 0.5)) * DT * DT if k > j else 0.0 for j in range(T)] for k in K])


def best_minsep(D, starts=6):
    base = np.array([-D, 0, 0])[None] + np.outer(K * DT, [V, 0, 0])  # r = p1 - p2
    def pos(x):
        return base + C @ x[:-1].reshape(T, 3)
    def cons(x):
        return np.linalg.norm(pos(x), axis=1) - x[-1]
    def jac(x):
        r = pos(x); g = r / np.linalg.norm(r, axis=1)[:, None]
        J = (C[:, :, None] * g[:, None, :]).reshape(T, 3 * T)
        return np.hstack([J, -np.ones((T, 1))])
    rng = np.random.default_rng(0)
    best = 0
    for _ in range(starts):
        x0 = np.append(rng.uniform(-2 * U, 2 * U, 3 * T), 0)
        r = minimize(lambda x: -x[-1], x0, jac=lambda x: -np.eye(1, 3 * T + 1, 3 * T)[0],
                     bounds=[(-2 * U, 2 * U)] * (3 * T) + [(None, None)],
                     constraints=[{"type": "ineq", "fun": cons, "jac": jac}], method="SLSQP",
                     options={"maxiter": 400})
        best = max(best, np.linalg.norm(pos(r.x), axis=1).min())
    return best


lo, hi = 20.0, 35.0
for _ in range(12):
    mid = 0.5 * (lo + hi)
    lo, hi = (lo, mid) if best_minsep(mid) >= D_EFF else (mid, hi)
print(f"optimiser (free relative control, ZOH 100 ms, sampled): D_min ~ {hi:.2f} m")
