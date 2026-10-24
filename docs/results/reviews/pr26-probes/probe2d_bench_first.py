"""scale_bench's own 64-cluster scene: per-step time and phase split for the first 8 steps (the bench drops the first 5)."""
import os, sys, time, logging
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "scripts", "verify"))
import numpy as np
import scale_bench as SB
from shared.contracts import MAX_SPEED_MPS, DroneState, MARLAction
from utm_services.safety import filter as F
logging.disable(logging.CRITICAL)
F._warm_up()
log = {}
def timed(name):
    real = getattr(F, name)
    def w(*a, **k):
        t0 = time.perf_counter(); r = real(*a, **k); log[name] = log.get(name, 0.0) + (time.perf_counter() - t0) * 1e3; return r
    setattr(F, name, w)
for n in ("_update_latches", "_backup_constraints", "_solve_one_qp", "_evasive_accel"):
    timed(n)
for n, cluster in ((64, 64), (64, 32), (64, 8)):
    p, v = SB.dense_states(n, 0, cluster)
    ids = [f"d{i:03d}" for i in range(n)]
    cfg = F.FilterConfig(); latch = {}
    print(f"\nN={n} groups of {cluster}:")
    for k in range(8):
        states = [DroneState(agent_id=ids[i], position_m=tuple(p[i]), velocity_mps=tuple(v[i]), goal_m=(900.0, 900.0, 100.0), timestamp_ms=k * 100) for i in range(n)]
        acts = [MARLAction(agent_id=i, proposed_accel_mps2=(0.0, 0.0, 0.0), timestamp_ms=k * 100) for i in ids]
        log.clear(); t0 = time.perf_counter()
        safe, diag = F.apply_filter(states, acts, cfg, latch=latch)
        total = (time.perf_counter() - t0) * 1e3
        a = np.array([s.safe_accel_mps2 for s in sorted(safe, key=lambda s: s.agent_id)])
        print(f"  step {k}: {total:7.1f} ms  latches {log.get('_update_latches',0):7.1f}  backup {log.get('_backup_constraints',0):5.1f}  solves {log.get('_solve_one_qp',0):5.1f}  evasive {log.get('_evasive_accel',0):5.1f}  missed {len(diag.deadline_fallback_agents):2d}  latch pairs {len(latch)}")
        vn = v + a * SB.DT; sp = np.linalg.norm(vn, axis=1)
        vn = np.where((sp > MAX_SPEED_MPS)[:, None], vn * (MAX_SPEED_MPS / sp * (1 - 1e-12))[:, None], vn)
        p = p + 0.5 * (v + vn) * SB.DT; v = vn
