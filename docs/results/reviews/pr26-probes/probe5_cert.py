"""Attack 5: does the KKT certificate reject inaccurate solutions? Loose OSQP, a degenerate collinear QP, perturbations."""
import numpy as np, osqp, scipy.sparse as sp
from shared.contracts import DroneState, MARLAction
from utm_services.safety import filter as F
from common import ring, qp_of
F._warm_up()

def solve_osqp_eps(P, q, A, l, u, eps, polish=True):
    prob = osqp.OSQP(); prob.setup(P, q, A, l, u, verbose=False, eps_abs=eps, eps_rel=eps, max_iter=200000, adaptive_rho_interval=25, polish=polish)
    r = prob.solve(); return r.x, r.y, r.info.status

def report(name, P, q, A, l, u, aid):
    ref = F._solve_daqp(P, q, A, l, u); assert ref not in (None, F._DEADLINE)
    x_ref, y_ref = ref
    print(f"\n{name}: vars {P.shape[0]}; DAQP certified: {F._kkt_certified(P, q, A, l, u, x_ref, y_ref)}")
    n = 3 * len(aid)
    for eps in (1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-9):
        for polish in (False, True):
            x, y, st = solve_osqp_eps(P, q, A, l, u, eps, polish)
            err = np.max(np.abs(x[:n] - x_ref[:n])); cert = F._kkt_certified(P, q, A, l, u, x, y)
            flag = "  <-- ACCEPTED though off" if cert and err > 1e-6 else ("  (rejected, off by %.1e)" % err if not cert else "")
            print(f"  OSQP eps={eps:g} polish={polish}: status {st}, max |da| vs DAQP {err:.2e}, certified {cert}{flag}")
    # perturbations of the exact optimum
    for scale in (1e-8, 1e-7, 1e-6, 1e-5, 1e-4):
        x = x_ref.copy(); x[:n] += scale * np.sign(np.random.default_rng(0).normal(size=n))
        x[n:] = np.maximum(x[n:], 0.0)
        # make the perturbed point primal-feasible by raising slack where needed
        Ax = (A @ x); viol = np.maximum(l[:len(x) - n] - Ax[:len(x) - n], 0.0)
        x[n:] += viol
        print(f"  perturb accel by {scale:g} (slack fixed up): certified with DAQP's y: {F._kkt_certified(P, q, A, l, u, x, y_ref)}")

# 1. three-drone conflict
s, a = ring((500, 500, 100), 15.0, 10.0, 3, "t")
report("3-ring", *qp_of(s, a)[:6])
# 2. degenerate: three collinear drones, outer two closing on the middle one at equal speed and distance
smap = [DroneState(agent_id="m", position_m=(500.0, 500.0, 100.0), velocity_mps=(0.0, 0.0, 0.0), goal_m=(900.0, 900.0, 100.0), timestamp_ms=0),
        DroneState(agent_id="l", position_m=(488.0, 500.0, 100.0), velocity_mps=(8.0, 0.0, 0.0), goal_m=(900.0, 900.0, 100.0), timestamp_ms=0),
        DroneState(agent_id="r", position_m=(512.0, 500.0, 100.0), velocity_mps=(-8.0, 0.0, 0.0), goal_m=(900.0, 900.0, 100.0), timestamp_ms=0)]
acts = [MARLAction(agent_id=x.agent_id, proposed_accel_mps2=(0.0, 0.0, 0.0), timestamp_ms=0) for x in smap]
report("collinear squeeze (degenerate duals)", *qp_of(smap, acts)[:6])
report("collinear squeeze, escape rows", *qp_of(smap, acts, escape=True)[:6])
# 3. 16-ring escape QP (OSQP's documented weak spot)
s, a = ring((500, 500, 100), 40.0, 12.0, 16, "e")
report("16-ring escape", *qp_of(s, a, escape=True)[:6])
