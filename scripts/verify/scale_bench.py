"""Step-time benchmark of apply_filter at scale: fixed seeded states, p50/p95 and a time split.

    python scripts/verify/scale_bench.py [--n 16 32 64 128] [--density dense city] [--cluster 8]
                                         [--src CHECKOUT] [--label NAME] [--out FILE.md] [--json FILE]

Each config (N, density) runs in its own process: 5 warm-up steps (Numba compile, caches), then 50
timed steps of the closed loop of fuzz.py (zero nominal acceleration, the simulator's trapezoid update
and speed cap, one latch dict per run). Densities:

- dense: groups of --cluster drones, each a fuzz.py-style converging encounter (miss up to about 6 m,
  closest approach 2-4 s after the start, so inside the timed window), group centres on a jittered
  80 m grid at 80-150 m altitude, so neighbouring groups also meet. --cluster N gives one converging
  cluster of all N drones.
- city: drones spread over x, y in [100, 900] m and z in [60, 160] m, at least 30 m apart, 5-15 m/s on
  random, mostly horizontal headings.

The time split is exclusive (a nested timed call is not counted twice) and is measured by wrapping
filter-module functions from outside, so older commits run unchanged (--src points at their checkout):
QP matrix build (build_hocbf_qp_matrices, first solve), OSQP setup and update, OSQP and DAQP solve (+ retries),
escape stage (the escape re-solve's build, setup and solve, and _escape_signs), _update_latches,
backup rollout (_backup_constraints: rollout and row derivation), everything else (pair loop,
diagnostics, contract objects, fallback). The wrappers cost a few microseconds per call.

Each run also records its safety outcome (min separation, fallback and solver-error steps, steps
inside d_eff, steps and agent-steps with a deadline fallback) and a hash of every filtered action, so two code versions can be compared on the same
states. Results are a Markdown table (--out) with the machine details, and JSON (--json).
"""
import argparse
import hashlib
import inspect
import json
import logging
import os
import platform
import subprocess
import sys
import time
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
DT, D_EFF = 0.1, 5.5
WARMUP, TIMED = 5, 50
CATEGORIES = ("build", "setup", "solve", "escape", "latches", "rollout", "other")
LABELS = {"build": "QP build", "setup": "OSQP setup", "solve": "OSQP solve", "escape": "escape stage",
          "latches": "_update_latches", "rollout": "backup rollout", "other": "everything else"}


def dense_states(n, seed, cluster):
    rng = np.random.default_rng(seed)
    groups = -(-n // cluster)
    side = int(np.ceil(np.sqrt(groups)))
    origin = 500.0 - 40.0 * (side - 1)
    p, v = [], []
    for g in range(groups):
        c = np.array([origin + 80.0 * (g % side) + rng.uniform(-10, 10),
                      origin + 80.0 * (g // side) + rng.uniform(-10, 10), rng.uniform(80, 150)])
        for _ in range(min(cluster, n - g * cluster)):
            for attempt in range(10000):
                az, el = rng.uniform(0, 2 * np.pi), rng.uniform(-0.5, 0.5)
                d = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
                speed = rng.uniform(2, 15) * 0.9999
                ta = rng.uniform(2, 4) + attempt / 500.0  # a large cluster spreads further out
                q = c + rng.normal(size=3) * rng.uniform(0, 6) - d * speed * ta
                if 31.0 < q[2] < 199.0 and all(np.linalg.norm(q - o) >= 10.0 for o in p):
                    break
            else:
                raise RuntimeError("could not place a drone")
            p.append(q)
            v.append(d * speed)
    return np.array(p), np.array(v)


def city_states(n, seed):
    rng = np.random.default_rng(seed)
    p, v = [], []
    while len(p) < n:
        q = np.array([*rng.uniform(100, 900, 2), rng.uniform(60, 160)])
        if all(np.linalg.norm(q - o) >= 30.0 for o in p):
            az, el = rng.uniform(0, 2 * np.pi), rng.uniform(-0.15, 0.15)
            p.append(q)
            v.append(np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)]) * rng.uniform(5, 15))
    return np.array(p), np.array(v)


class Split:
    """Exclusive wall time per category, from wrapped filter-module functions."""

    def __init__(self):
        self.acc = defaultdict(float)
        self.counts = defaultdict(int)
        self.stack = []  # [category, time spent in timed children]
        self.stage = "first"

    def timed(self, category, fn):
        def wrapper(*args, **kwargs):
            cat = category(args, kwargs) if callable(category) else category
            if cat is None:
                return fn(*args, **kwargs)
            t0 = time.perf_counter()
            self.stack.append([cat, 0.0])
            try:
                return fn(*args, **kwargs)
            finally:
                _, child = self.stack.pop()
                spent = time.perf_counter() - t0
                self.acc[cat] += spent - child
                if self.stack:
                    self.stack[-1][1] += spent
        return wrapper

    def install(self, filt):
        split = self

        def staged(name):
            return lambda a, k: "escape" if split.stage == "escape" else name

        def solve_hocbf(fn):
            def wrapper(*args, **kwargs):
                escape = args[4] if len(args) > 4 else kwargs.get("escape_signs")
                outer, split.stage = split.stage, ("escape" if escape else split.stage)
                try:
                    return fn(*args, **kwargs)
                finally:
                    split.stage = outer
            return wrapper

        class TimedOSQP:
            def __init__(self, *a, **k):
                self._prob = real.OSQP(*a, **k)

            def setup(self, *a, **k):
                split.counts["setups"] += 1
                return split.timed(staged("setup"), self._prob.setup)(*a, **k)

            def update(self, *a, **k):
                return split.timed(staged("setup"), self._prob.update)(*a, **k)

            def solve(self, *a, **k):
                split.counts["solves"] += 1
                return split.timed(staged("solve"), self._prob.solve)(*a, **k)

            def __getattr__(self, name):
                return getattr(self._prob, name)

        real = filt.osqp
        filt.osqp = type("osqp_shim", (), {"OSQP": TimedOSQP})
        if hasattr(filt, "daqp"):  # KKT-exact solver (perf/solver-exact): DAQP time counts as solve
            daqp_solve = filt.daqp.solve

            def timed_daqp(*a, **k):
                split.counts["solves"] += 1
                return split.timed(staged("solve"), daqp_solve)(*a, **k)
            filt.daqp = type("daqp_shim", (), {"solve": staticmethod(timed_daqp)})
        filt.build_hocbf_qp_matrices = self.timed(staged("build"), filt.build_hocbf_qp_matrices)
        filt._solve_hocbf_qp = solve_hocbf(filt._solve_hocbf_qp)
        if hasattr(filt, "_update_latches"):
            filt._update_latches = self.timed("latches", filt._update_latches)
        filt._backup_constraints = self.timed("rollout", filt._backup_constraints)
        # Nested in _update_latches it is latch time; at the top level it is the escape stage.
        filt._escape_signs = self.timed(lambda a, k: None if split.stack else "escape", filt._escape_signs)


def clusters(latch):
    parent = {}

    def find(a):
        while parent.setdefault(a, a) != a:
            a = parent[a]
        return a
    for i, j in latch:
        parent[find(i)] = find(j)
    sizes = defaultdict(int)
    for a in parent:
        sizes[find(a)] += 1
    return max(sizes.values(), default=0)


def worker(n, density, cluster, seed):
    logging.disable(logging.CRITICAL)
    from shared.contracts import MAX_SPEED_MPS, DroneState, MARLAction

    import utm_services.safety.filter as filt

    p, v = dense_states(n, seed, cluster) if density == "dense" else city_states(n, seed)
    ids = [f"d{i:03d}" for i in range(n)]
    cfg = filt.FilterConfig()
    kwargs = {"latch": {}} if "latch" in inspect.signature(filt.apply_filter).parameters else {}
    split = Split()
    split.install(filt)
    times, rows, digest = [], [], hashlib.sha256()
    out = {"min_sep": float("inf"), "fallback_steps": 0, "solver_err_steps": 0, "inside_steps": 0,
           "escape_steps": 0, "max_cluster": 0, "deadline_steps": 0, "deadline_agent_steps": 0,
           "step_deadline": getattr(cfg, "step_deadline", None)}  # runtime default: on (None: before v1.5)
    for k in range(WARMUP + TIMED):
        ts = k * 100
        states = [DroneState(agent_id=ids[i], position_m=tuple(p[i]), velocity_mps=tuple(v[i]),
                             goal_m=(900.0, 900.0, 100.0), timestamp_ms=ts) for i in range(n)]
        acts = [MARLAction(agent_id=i, proposed_accel_mps2=(0.0, 0.0, 0.0), timestamp_ms=ts) for i in ids]
        split.acc.clear()
        split.counts.clear()
        t0 = time.perf_counter()
        safe, diag = filt.apply_filter(states, acts, cfg, **kwargs)
        total = time.perf_counter() - t0
        a = np.array([s.safe_accel_mps2 for s in sorted(safe, key=lambda s: s.agent_id)])
        digest.update(a.tobytes())
        if k >= WARMUP:
            times.append(total)
            row = {c: split.acc.get(c, 0.0) for c in CATEGORIES[:-1]}
            row["other"] = total - sum(row.values())
            row["solves"] = split.counts["solves"]
            rows.append(row)
            out["fallback_steps"] += int(diag.fallback_used)
            out["solver_err_steps"] += int(not diag.qp_feasible)
            missed = getattr(diag, "deadline_fallback_agents", [])  # contract v1.5
            out["deadline_steps"] += int(bool(missed))
            out["deadline_agent_steps"] += len(missed)
            out["escape_steps"] += int(row["escape"] > 0.0)
            out["max_cluster"] = max(out["max_cluster"], clusters(kwargs.get("latch", {})))
        vn = v + a * DT
        speed = np.linalg.norm(vn, axis=1)
        vn = np.where((speed > MAX_SPEED_MPS)[:, None], vn * (MAX_SPEED_MPS / speed * (1 - 1e-12))[:, None], vn)
        p = p + 0.5 * (v + vn) * DT
        v = vn
        dist = np.linalg.norm(p[:, None] - p[None], axis=2)[np.triu_indices(n, 1)]
        out["min_sep"] = min(out["min_sep"], float(dist.min()))
        out["inside_steps"] += int(dist.min() < D_EFF)
    t = np.array(times) * 1e3
    out.update(n=n, density=density, cluster=cluster if density == "dense" else None, seed=seed,
               pairs=n * (n - 1) // 2, p50_ms=float(np.percentile(t, 50)), p95_ms=float(np.percentile(t, 95)),
               max_ms=float(t.max()), mean_ms=float(t.mean()), actions_sha256=digest.hexdigest()[:16],
               split_ms={c: 1e3 * float(np.mean([r[c] for r in rows])) for c in CATEGORIES},
               solves_per_step=float(np.mean([r["solves"] for r in rows])))
    return out


def machine():
    cpu = platform.processor() or "unknown"
    try:
        with open("/proc/cpuinfo") as f:
            cpu = next(line.split(":", 1)[1].strip() for line in f if line.startswith("model name"))
    except (OSError, StopIteration):
        pass
    import numba
    import numpy
    import osqp
    import scipy
    return (f"{cpu}, {os.cpu_count()} logical cores, {platform.system()} {platform.release()}, "
            f"Python {platform.python_version()}, numpy {numpy.__version__}, scipy {scipy.__version__}, "
            f"osqp {getattr(osqp, '__version__', '?')}, numba {numba.__version__}")


def table(results):
    lines = ["| N | density | pairs | p50 ms | p95 ms | max ms | " + " | ".join(LABELS[c] for c in CATEGORIES)
             + " | solves/step | escape steps | max cluster | min sep m | fallback | deadline (steps / agent-steps) "
             "| actions hash |",
             "|" + " --- |" * (13 + len(CATEGORIES))]
    for r in results:
        dens = r["density"] if r["cluster"] is None else f"{r['density']} (groups of {r['cluster']})"
        split = " | ".join(f"{r['split_ms'][c]:.1f}" for c in CATEGORIES)
        lines.append(f"| {r['n']} | {dens} | {r['pairs']} | {r['p50_ms']:.1f} | {r['p95_ms']:.1f} | "
                     f"{r['max_ms']:.1f} | {split} | {r['solves_per_step']:.2f} | {r['escape_steps']} | "
                     f"{r['max_cluster']} | {r['min_sep']:.3f} | {r['fallback_steps']} | "
                     f"{r.get('deadline_steps', 0)} / {r.get('deadline_agent_steps', 0)} | `{r['actions_sha256']}` |")
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--n", type=int, nargs="+", default=[16, 32, 64, 128])
    ap.add_argument("--density", nargs="+", default=["dense", "city"], choices=("dense", "city"))
    ap.add_argument("--cluster", type=int, nargs="+", default=[8], help="dense group size(s)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--src", default=ROOT, help="checkout whose filter is measured (default: this one)")
    ap.add_argument("--label", default=None, help="name of the measured version in the output")
    ap.add_argument("--out", help="append the Markdown table to this file")
    ap.add_argument("--json", help="append one JSON row per config to this file")
    ap.add_argument("--worker", nargs=4, metavar=("N", "DENSITY", "CLUSTER", "SEED"), help=argparse.SUPPRESS)
    a = ap.parse_args(argv)
    if a.worker:
        n, density, cluster, seed = a.worker
        print(json.dumps(worker(int(n), density, int(cluster), int(seed))))
        return 0

    src = os.path.abspath(a.src)
    env = {**os.environ, "PYTHONPATH": os.pathsep.join([os.path.join(src, "src"), src]),
           "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1"}
    rev = subprocess.run(["git", "-C", src, "rev-parse", "--short", "HEAD"], capture_output=True, text=True, check=False).stdout
    dirty = subprocess.run(["git", "-C", src, "status", "--porcelain", "--", "src"], capture_output=True,
                           text=True, check=False).stdout.strip()
    label = a.label or rev.strip() + (" (uncommitted src changes)" if dirty else "")
    configs = [(n, d, c) for n in a.n for d in a.density for c in (a.cluster if d == "dense" else [0])]
    results = []
    for n, density, cluster in configs:
        t0 = time.perf_counter()
        proc = subprocess.run([sys.executable, os.path.abspath(__file__), "--worker", str(n), density,
                               str(cluster), str(a.seed)], env=env, capture_output=True, text=True, cwd=src,
                              check=False)
        if proc.returncode != 0:
            print(proc.stderr, file=sys.stderr)
            return 1
        r = json.loads(proc.stdout.strip().splitlines()[-1])
        r["version"] = label
        results.append(r)
        print(f"{label} N={n} {density}{'' if density == 'city' else f'/{cluster}'}: p50 {r['p50_ms']:.1f} ms, "
              f"p95 {r['p95_ms']:.1f} ms ({time.perf_counter() - t0:.0f} s wall)", flush=True)
        if a.json:
            with open(a.json, "a") as f:
                f.write(json.dumps(r, sort_keys=True) + "\n")
    text = (f"### {label}\n\nMachine: {machine()}. Seed {a.seed}, {WARMUP} warm-up and {TIMED} timed steps per "
            f"config, one process per config, split in mean ms per timed step. "
            f"FilterConfig defaults, step_deadline {sorted({str(r.get('step_deadline')) for r in results})}."
            f"\n\n{table(results)}\n")
    print(text)
    if a.out:
        with open(a.out, "a") as f:
            f.write(text + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
