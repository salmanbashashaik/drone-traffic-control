"""Attack 2, end to end: a big cluster whose OSQP answer fails the certificate goes to DAQP,
whose setup ignores the time limit; the later small cluster then misses without solving."""
import time, logging
import numpy as np
from common import ring, qp_of
from utm_services.safety import filter as F
logging.basicConfig(level=logging.WARNING, format="%(message)s")
F._warm_up()

# How often does OSQP (first solver above 256 vars) fail the certificate on real ring QPs?
for n in (32, 64):
    for escape in (False, True):
        fails = 0; total = 0; times = []
        for radius in (40.0, 60.0, 90.0):
            states, actions = ring((500, 500, 100), radius, 12.0, n, "r")
            P, q, A, l, u, aid, pairs = qp_of(states, actions, escape=escape)
            t0 = time.perf_counter(); r = F._solve_osqp(P, q, A, l, u, F.FilterConfig(), escape); times.append(time.perf_counter() - t0)
            total += 1
            if r is None or not F._kkt_certified(P, q, A, l, u, *r): fails += 1
        print(f"N={n} escape={escape}: OSQP fails certificate on {fails}/{total}, solve times ms {[round(t*1e3,1) for t in times]}")

# End to end. Cluster A: 32-ring at (300,500) (592 vars, OSQP first). Cluster B: 3 drones at (800,800), far, cannot bind with A.
def scene(n_big):
    sa, aa = ring((300, 500, 100), 50.0, 12.0, n_big, "a")
    sb, ab = ring((800, 800, 100), 20.0, 12.0, 3, "b")
    return sa + sb, aa + ab

for n_big in (32, 64):
    states, actions = scene(n_big)
    base, _ = F.apply_filter(states, actions, F.FilterConfig(step_deadline=False), latch={})
    base = {a.agent_id: a for a in base}
    # plain deadline run
    t0 = time.perf_counter(); safe, diag = F.apply_filter(states, actions, F.FilterConfig(), latch={}); dt = time.perf_counter() - t0
    print(f"\nn_big={n_big} plain deadline run: step {dt*1e3:.0f} ms, missed agents {len(diag.deadline_fallback_agents)}, b missed: {[a for a in diag.deadline_fallback_agents if a.startswith('b')]}")
    # force the certificate to reject OSQP's answer on the big cluster (the documented path to DAQP)
    real_cert = F._kkt_certified
    state = {"rejected": 0}
    def reject_osqp_once(P, q, A, l, u, x, y):
        if P.shape[0] > 256 and state["rejected"] == 0:
            state["rejected"] += 1
            return False
        return real_cert(P, q, A, l, u, x, y)
    F._kkt_certified = reject_osqp_once
    try:
        t0 = time.perf_counter(); safe, diag = F.apply_filter(states, actions, F.FilterConfig(), latch={}); dt = time.perf_counter() - t0
    finally:
        F._kkt_certified = real_cert
    safe = {a.agent_id: a for a in safe}
    b_missed = [a for a in diag.deadline_fallback_agents if a.startswith("b")]
    print(f"n_big={n_big} OSQP answer rejected once -> DAQP: step {dt*1e3:.0f} ms, missed agents {len(diag.deadline_fallback_agents)}, cluster b missed: {b_missed}")
    print("  cluster b accel equal to no-deadline run:", all(safe[a].safe_accel_mps2 == base[a].safe_accel_mps2 for a in ("b00", "b01", "b02")))
