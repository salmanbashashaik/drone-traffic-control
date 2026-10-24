"""Attack 2: do the solver calls stop at their time limit? Measure wall time past a tiny limit."""
import time, sys
import numpy as np
from common import ring, qp_of
from utm_services.safety import filter as F
import daqp, osqp
print("osqp", osqp.__version__)
F._warm_up()
for n in (16, 32, 64):
    states, actions = ring((500, 500, 100), 60.0, 12.0, n, "r")
    P, q, A, l, u, aid, pairs = qp_of(states, actions)
    print(f"\nN={n} vars={P.shape[0]} rows={A.shape[0]}")
    for limit in (0.001, 0.005):
        t0 = time.perf_counter(); r = F._solve_daqp(P, q, A, l, u, limit); dt = time.perf_counter() - t0
        print(f"  DAQP time_limit={limit*1e3:.0f} ms -> wall {dt*1e3:.1f} ms, result {'DEADLINE' if r is F._DEADLINE else ('None' if r is None else 'solved')}")
    for limit in (0.001, 0.005):
        cfg = F.FilterConfig()
        t0 = time.perf_counter(); r = F._solve_osqp(P, q, A, l, u, cfg, False, time.perf_counter() + limit); dt = time.perf_counter() - t0
        print(f"  OSQP deadline in {limit*1e3:.0f} ms -> wall {dt*1e3:.1f} ms, result {'DEADLINE' if r is F._DEADLINE else ('None' if r is None else 'solved')}")
    # setup alone
    t0 = time.perf_counter(); prob = osqp.OSQP(); prob.setup(P, q, A, l, u, verbose=False, eps_abs=1e-9, eps_rel=1e-9, adaptive_rho_interval=25, polish=True); dt = time.perf_counter() - t0
    print(f"  OSQP setup alone {dt*1e3:.1f} ms")
    # certificate cost
    x = np.zeros(P.shape[0]); y = np.zeros(A.shape[0])
    t0 = time.perf_counter(); F._kkt_certified(P, q, A, l, u, x, y); dt = time.perf_counter() - t0
    print(f"  _kkt_certified {dt*1e3:.1f} ms")
    # full _solve_exact with 5 ms left
    t0 = time.perf_counter(); r = F._solve_exact(P, q, A, l, u, F.FilterConfig(), False, time.perf_counter() + 0.005); dt = time.perf_counter() - t0
    print(f"  _solve_exact deadline in 5 ms -> wall {dt*1e3:.1f} ms, {'DEADLINE' if r is F._DEADLINE else ('None' if r is None else 'solved')}")
    # escape QP too
    P, q, A, l, u, aid, pairs = qp_of(states, actions, escape=True)
    t0 = time.perf_counter(); r = F._solve_exact(P, q, A, l, u, F.FilterConfig(), True, time.perf_counter() + 0.005); dt = time.perf_counter() - t0
    print(f"  escape _solve_exact deadline in 5 ms -> wall {dt*1e3:.1f} ms, {'DEADLINE' if r is F._DEADLINE else ('None' if r is None else 'solved')}")
