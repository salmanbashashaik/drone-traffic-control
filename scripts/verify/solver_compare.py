"""Compare QP solvers on the QPs the filter builds: accuracy against each other, KKT certificate, time.

    python scripts/verify/solver_compare.py [--fuzz SEEDS] [--hard SEEDS] [--bench N ...]

Records every QP ``build_hocbf_qp_matrices`` returns while fuzz.py runs seeds 0..SEEDS-1 (normal
and hard mode) and, with --bench, while scale_bench.py flies N drones (dense groups of 8; joint
QPs, 10 of the 55 steps). Each QP is solved by:

- osqp 1e-5: the filter's OSQP settings before perf/solver-exact (eps 1e-5, polish);
- osqp 1e-9: OSQP at eps 1e-9 (the filter's default since perf/solver-exact);
- daqp: DAQP, dual active set (exact up to linear algebra);
- clarabel, proxsuite, piqp: interior-point and augmented-Lagrangian references, if installed.

Reported per solver: failures, KKT certificate passes (``filter._kkt_certified``), the largest
acceleration difference to DAQP and the count above 1e-6 m/s², and time per QP. This is the
measurement behind the solver choice in filter.py (``_EXACT_MAX_VARS``).
"""
import argparse
import os
import sys
import time

import numpy as np
import scipy.sparse as sp

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import utm_services.safety.filter as F


def record(run):
    qps, real = [], F.build_hocbf_qp_matrices

    def wrapper(states, actions, pairs, config, escape_signs=None, backup=None, frames=None):
        out = real(states, actions, pairs, config, escape_signs, backup, frames)
        qps.append((*(m.copy() for m in out[:5]), len(out[5]), bool(escape_signs)))
        return out

    F.build_hocbf_qp_matrices = wrapper
    try:
        run()
    finally:
        F.build_hocbf_qp_matrices = real
    return qps


def osqp_at(eps, max_iter):
    def solve(P, q, A, l, u, escape):
        config = F.FilterConfig(solver_eps=eps, solver_max_iter=max_iter)
        return F._solve_osqp(P, q, A, l, u, config, escape)
    return solve


def clarabel_solve(P, q, A, l, u, escape):
    import clarabel
    rows_l, rows_u = np.flatnonzero(np.isfinite(l)), np.flatnonzero(np.isfinite(u))
    A = A.tocsc()
    G = sp.vstack([-A[rows_l], A[rows_u]]).tocsc()
    s = clarabel.DefaultSettings()
    s.verbose = False
    s.tol_gap_abs = s.tol_gap_rel = s.tol_feas = 1e-10
    solver = clarabel.DefaultSolver(sp.triu(P).tocsc(), q, G, np.concatenate([-l[rows_l], u[rows_u]]),
                                    [clarabel.NonnegativeConeT(len(rows_l) + len(rows_u))], s)
    sol = solver.solve()
    if "Solved" not in str(sol.status):
        return None
    z = np.asarray(sol.z)
    y = np.zeros(A.shape[0])
    y[rows_l] -= z[: len(rows_l)]
    y[rows_u] += z[len(rows_l):]
    return np.asarray(sol.x), y


def proxsuite_solve(P, q, A, l, u, escape):
    import proxsuite
    qp = proxsuite.proxqp.sparse.QP(P.shape[0], 0, A.shape[0])
    qp.settings.eps_abs, qp.settings.eps_rel, qp.settings.verbose = 1e-9, 0.0, False
    qp.init(P.tocsc(), q, None, None, A.tocsc(), np.where(np.isfinite(l), l, -1e20), np.where(np.isfinite(u), u, 1e20))
    qp.solve()
    return np.asarray(qp.results.x), np.asarray(qp.results.z)


def piqp_solve(P, q, A, l, u, escape):
    import piqp
    solver = piqp.SparseSolver()
    solver.settings.eps_abs, solver.settings.eps_rel, solver.settings.verbose = 1e-10, 0.0, False
    solver.setup(P=P.tocsc(), c=q, G=A.tocsc(), h_l=l, h_u=u)
    solver.solve()
    r = solver.result
    return np.asarray(r.x), np.asarray(r.z_u) - np.asarray(r.z_l)


SOLVERS = {"osqp 1e-5": osqp_at(1e-5, 4000), "osqp 1e-9": osqp_at(1e-9, 200000),
           "daqp": lambda P, q, A, l, u, escape: F._solve_daqp(P, q, A, l, u)}
for name, fn, module in (("clarabel", clarabel_solve, "clarabel"), ("proxsuite", proxsuite_solve, "proxsuite"),
                         ("piqp", piqp_solve, "piqp")):
    try:
        __import__(module)
        SOLVERS[name] = fn
    except ImportError:
        pass


def compare(label, qps):
    stats = {name: {"t": [], "diff": [], "fail": 0, "cert": 0} for name in SOLVERS}
    for P, q, A, l, u, agents, escape in qps:
        sols = {}
        for name, solve in SOLVERS.items():
            t0 = time.perf_counter()
            try:
                sol = solve(P, q, A, l, u, escape)
            except Exception:  # noqa: BLE001
                sol = None
            stats[name]["t"].append(time.perf_counter() - t0)
            if sol is None or not np.all(np.isfinite(sol[0])):
                stats[name]["fail"] += 1
                continue
            stats[name]["cert"] += F._kkt_certified(P, q, A, l, u, *sol)
            sols[name] = sol[0][: 3 * agents]
        for name, x in sols.items():
            if "daqp" in sols:
                stats[name]["diff"].append(float(np.max(np.abs(x - sols["daqp"]))))
    sizes = [P.shape[0] for P, *_ in qps]
    print(f"\n{label}: {len(qps)} QPs, {min(sizes)}-{max(sizes)} variables")
    print("| solver | failed | KKT certified | max |a - a_daqp| | QPs > 1e-6 | mean ms | p95 ms | max ms |")
    print("| --- | --- | --- | --- | --- | --- | --- | --- |")
    for name, s in stats.items():
        d = np.array(s["diff"] or [np.nan])
        t = np.array(s["t"]) * 1e3
        print(f"| {name} | {s['fail']} | {s['cert']} | {np.nanmax(d):.1e} | {int(np.sum(d > 1e-6))} | {t.mean():.2f} | "
              f"{np.percentile(t, 95):.2f} | {t.max():.2f} |", flush=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--fuzz", type=int, default=150, help="normal fuzz seeds 0..n-1")
    ap.add_argument("--hard", type=int, default=150, help="hard fuzz seeds 0..n-1")
    ap.add_argument("--bench", type=int, nargs="*", default=[], help="drone counts for joint bench QPs")
    a = ap.parse_args(argv)
    import fuzz
    import scale_bench
    print(f"solvers: {', '.join(SOLVERS)}")
    for mode, count in (("fuzz", a.fuzz), ("hard", a.hard)):
        if count:
            compare(f"{mode} seeds 0-{count - 1}", record(lambda m=mode, c=count: [
                fuzz.run((seed, "backup", 100, m == "hard", True)) for seed in range(c)]))
    for n in a.bench:
        qps = record(lambda n=n: scale_bench.worker(n, "dense", 8, 0))
        compare(f"bench N = {n} dense, joint QPs", qps[5::5])
    return 0


if __name__ == "__main__":
    sys.exit(main())
