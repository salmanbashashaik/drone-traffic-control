"""Where does the time go on a 64-ring plus a far 3-cluster, deadline on?"""
import time, logging
import numpy as np
from common import ring
from utm_services.safety import filter as F
logging.basicConfig(level=logging.WARNING, format="%(message)s")
F._warm_up()
log = []
def timed(name, keep_result=False):
    real = getattr(F, name)
    def w(*a, **k):
        t0 = time.perf_counter(); r = real(*a, **k); dt = time.perf_counter() - t0
        tag = ""
        if name == "_solve_one_qp":
            until = a[7] if len(a) > 7 else k.get("deadline")
            tag = f" agents={len(a[0])} pairs={len(a[2])} left_at_entry={(until - t0)*1e3 if until else float('nan'):.1f}ms -> {'DEADLINE' if r is F._DEADLINE else ('None' if r is None else 'solved')}"
        if name in ("_solve_osqp", "_solve_daqp"):
            tag = f" vars={a[0].shape[0]} -> {'DEADLINE' if r is F._DEADLINE else ('None' if r is None else 'sol')}"
        log.append((name, dt * 1e3, tag)); return r
    setattr(F, name, w)
for n in ("_update_latches", "_backup_constraints", "_independent_problems", "_solve_one_qp", "_solve_osqp", "_solve_daqp", "_kkt_certified", "_evasive_accel", "_fallback_cluster", "_select_escape_pairs", "_escape_constraint"):
    if hasattr(F, n): timed(n)
sa, aa = ring((300, 500, 100), 50.0, 12.0, 64, "a")
sb, ab = ring((800, 800, 100), 20.0, 12.0, 3, "b")
states, actions = sa + sb, aa + ab
for rep in range(2):
    log.clear()
    t0 = time.perf_counter(); safe, diag = F.apply_filter(states, actions, F.FilterConfig(), latch={}); total = (time.perf_counter() - t0) * 1e3
    print(f"\nrun {rep}: total {total:.0f} ms, missed {len(diag.deadline_fallback_agents)}")
    agg = {}
    for name, dt, tag in log:
        if tag: print(f"  {name:22s} {dt:8.1f} ms{tag}")
        agg.setdefault(name, [0, 0.0]); agg[name][0] += 1; agg[name][1] += dt
    for name, (c, s) in agg.items(): print(f"  sum {name:22s} x{c:<3d} {s:8.1f} ms")
