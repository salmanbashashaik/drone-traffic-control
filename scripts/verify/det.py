"""Hash per-step filtered actions for fixed scenarios; run in separate processes to check determinism."""
import hashlib, logging, os
import numpy as np
logging.disable(logging.CRITICAL)
from shared.contracts import ScenarioConfig, DroneState, MARLAction
from utm_services.policy.reference import compute_reference_action
from utm_services.simulation.runner import run_episode
from utm_services.safety.filter import FilterConfig, apply_filter
import fuzz

def d(a, p, g, v=(0, 0, 0)):
    return {"agent_id": a, "initial_position_m": tuple(map(float, p)), "initial_velocity_mps": tuple(map(float, v)), "goal_m": tuple(map(float, g))}

SC = {
    "sandwich": [d("d1", (490, 500, 100), (900, 500, 100), (10, 0, 0)), d("d2", (500, 500, 100), (500, 900, 100)), d("d3", (510, 500, 100), (100, 500, 100), (-10, 0, 0))],
    "vertical": [d("a", (500, 500, 60), (500, 500, 190), (0, 0, 10)), d("b", (500, 500, 120), (500, 500, 35), (0, 0, -10))],
    "ring5": [d(f"r{i}", (500 + 40 * np.cos(2 * np.pi * i / 5), 500 + 40 * np.sin(2 * np.pi * i / 5), 100), (500 - 300 * np.cos(2 * np.pi * i / 5), 500 - 300 * np.sin(2 * np.pi * i / 5), 100), (-10 * np.cos(2 * np.pi * i / 5), -10 * np.sin(2 * np.pi * i / 5), 0)) for i in range(5)],
}
out = []
for name, ds in SC.items():
    sc = ScenarioConfig(scenario_id=name, seed=0, episode_limit_steps=200, drones=ds)
    recs = run_episode(sc, "shielded_marl", policy=compute_reference_action, policy_id="ref", keep_records=True).records
    h = hashlib.sha256()
    for r in recs:
        for a in sorted(r["returned_actions"], key=lambda x: x.agent_id if hasattr(x, "agent_id") else x["agent_id"]):
            acc = a.safe_accel_mps2 if hasattr(a, "safe_accel_mps2") else a["safe_accel_mps2"]
            h.update(np.asarray(acc, dtype=np.float64).tobytes())
    out.append(f"{name}:{h.hexdigest()[:16]}")
# fuzz seeds: closed loop, hash every action with hashlib (not hash())
for seed in (3, 17, 101, 9002, 9003):
    p, v = fuzz.encounter(seed)
    n = len(p); ids = [f"d{i}" for i in range(n)]; h = hashlib.sha256()
    for k in range(100):
        st = [DroneState(agent_id=ids[i], position_m=tuple(p[i]), velocity_mps=tuple(v[i]), goal_m=(900.0, 900.0, 100.0), timestamp_ms=k * 100) for i in range(n)]
        ac = [MARLAction(agent_id=i, proposed_accel_mps2=(0.0, 0.0, 0.0), timestamp_ms=k * 100) for i in ids]
        safe, _ = apply_filter(st, ac, FilterConfig())
        a = np.array([s.safe_accel_mps2 for s in sorted(safe, key=lambda s: s.agent_id)])
        h.update(a.tobytes())
        vn = v + a * 0.1; sp = np.linalg.norm(vn, axis=1)
        vn = np.where((sp > 15)[:, None], vn * (15 / sp * (1 - 1e-12))[:, None], vn)
        p = p + 0.5 * (v + vn) * 0.1; v = vn
    out.append(f"fuzz{seed}:{h.hexdigest()[:16]}")
print(" ".join(out))
