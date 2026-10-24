"""Attack 3: deadline on (no miss) vs off, closed loop, bit for bit. Also: does daqp accept time_limit?"""
import json, os, numpy as np, daqp
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
from shared.contracts import ScenarioConfig
from utm_services.policy.reference import compute_reference_action
from utm_services.simulation.runner import run_episode
from utm_services.safety import filter as F
F._warm_up()
# daqp time_limit accepted? (an unknown key would raise -> _solve_exact would fall to OSQP)
H = np.eye(2); f = np.zeros(2); A = np.array([[1.0, 1.0]]); bu = np.array([1.0]); bl = np.array([-1.0])
try:
    x, fval, flag, info = daqp.solve(H, f, A, bu, bl, np.zeros(1, dtype=np.int32), time_limit=0.5)
    print("daqp accepts time_limit, flag", flag)
except Exception as e:
    print("daqp rejected time_limit:", e)
try:
    daqp.solve(H, f, A, bu, bl, np.zeros(1, dtype=np.int32), bogus_setting=1); print("daqp silently accepts unknown keys!")
except Exception as e:
    print("daqp rejects unknown key:", type(e).__name__)

for path in ("scenarios/head-on-40.json",):
    cfg = ScenarioConfig(**json.load(open(os.path.join(ROOT, path))))
    outs = {}
    for name, fc, budget in (("off", F.FilterConfig(step_deadline=False), 0.1), ("on_wide", F.FilterConfig(), 10.0), ("on_std", F.FilterConfig(), 0.1)):
        F._STEP_BUDGET_S = budget
        res = run_episode(cfg, "shielded_marl", policy=compute_reference_action, policy_id="reference-standin", filter_config=fc, keep_records=True)
        F._STEP_BUDGET_S = 0.1
        accs = [tuple(a.safe_accel_mps2) for rec in res.records for a in (rec.get("safe_actions") or rec.get("actions") or [])] if isinstance(res.records[0], dict) else None
        outs[name] = (res.summary, res.records)
        s = res.summary
        print(name, {k: s[k] for k in ("min_separation_m", "fallback_events", "deadline_fallbacks", "steps") if k in s}, "mean step ms", round(s.get("mean_step_time_ms", 0), 1))
    # compare records bit for bit
    def flat(records):
        out = []
        for r in records:
            out.append(json.dumps(r, sort_keys=True, default=str))
        return out
    a, b, c = flat(outs["off"][1]), flat(outs["on_wide"][1]), flat(outs["on_std"][1])
    print("records identical off vs on_wide:", a == b, "; off vs on_std:", a == c, "; n records", len(a))
    if a != c:
        diffs = [i for i, (x, y) in enumerate(zip(a, c)) if x != y]
        print("  first differing record index", diffs[:5])
        print("  sample:", a[diffs[0]][:300]); print("  vs    :", c[diffs[0]][:300])
