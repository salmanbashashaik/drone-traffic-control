import logging; logging.disable(logging.CRITICAL)
import numpy as np
from shared.contracts import MAX_SPEED_MPS, DroneState, MARLAction
from utm_services.safety import filter as F
def run(sig, bar, seed):
    rng = np.random.default_rng(seed)
    p = np.array([(500, 500, 70.0), (500, 500, 130.0)]); v = np.array([(0, 0, 10.0), (0, 0, -10.0)])
    cfg = F.FilterConfig(step_deadline=False, barrier=bar); latch = {}; mins = 1e9; flips = 0; prev = None
    for k in range(120):
        obs = p + rng.normal(0, sig, p.shape)
        st = [DroneState(agent_id=f"d{i}", position_m=tuple(obs[i]), velocity_mps=tuple(v[i]), goal_m=(500,500,100.), timestamp_ms=k*100) for i in range(2)]
        acts = [MARLAction(agent_id=f"d{i}", proposed_accel_mps2=(0,0,0), timestamp_ms=k*100) for i in range(2)]
        safe, _ = F.apply_filter(st, acts, cfg, latch=latch)
        a = np.array([s.safe_accel_mps2 for s in sorted(safe, key=lambda s: s.agent_id)])
        h = a[0, :2] - a[1, :2]
        if np.linalg.norm(h) > 0.5:
            if prev is not None and h @ prev < 0: flips += 1
            prev = h
        vn = v + a * 0.1; sp = np.linalg.norm(vn, axis=1, keepdims=True); vn *= np.minimum(1, MAX_SPEED_MPS / np.maximum(sp, 1e-9))
        p = p + 0.5 * (v + vn) * 0.1; v = vn
        mins = min(mins, np.linalg.norm(p[0] - p[1]))
    return mins, flips
for bar in ("backup", "braking"):
    for sig in (0.0, 0.5, 1.5):
        r = [run(sig, bar, s) for s in range(20)]
        print(f"{bar:8s} sigma={sig}: min_sep worst={min(x[0] for x in r):.3f} below 5.5: {sum(x[0]<5.5 for x in r)}/20  below 5.0: {sum(x[0]<5.0 for x in r)}/20  lateral-direction reversals max={max(x[1] for x in r)}")
