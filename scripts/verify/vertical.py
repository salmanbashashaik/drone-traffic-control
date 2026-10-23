import logging, sys; logging.disable(logging.CRITICAL)
import numpy as np
from shared.contracts import ScenarioConfig
from utm_services.policy.reference import compute_reference_action
from utm_services.simulation.runner import run_episode
import utm_services.safety.filter as F
def d(a,p,g,v): return {"agent_id":a,"initial_position_m":p,"initial_velocity_mps":v,"goal_m":g}
has_bar = "barrier" in F.FilterConfig.__dataclass_fields__
for bar in (("backup","braking") if has_bar else ("main",)):
    kw = {"filter_config": F.FilterConfig(barrier=bar)} if has_bar else {}
    sc = ScenarioConfig(scenario_id="v", seed=0, episode_limit_steps=200, drones=[d("a",(500.,500.,60.),(500.,500.,190.),(0.,0.,10.)), d("b",(500.,500.,120.),(500.,500.,35.),(0.,0.,-10.))])
    r = run_episode(sc, "shielded_marl", policy=compute_reference_action, policy_id="ref", keep_records=True, **kw)
    print(bar, "min_sep", round(r.summary["min_separation_m"],3), "fallbacks", r.summary["fallback_events"])
    if len(sys.argv) > 1:
        for rec in r.records[:40]:
            P = {s["agent_id"]: np.array(s["position_m"]) for s in rec["states"]}
            V = {s["agent_id"]: np.array(s["velocity_mps"]) for s in rec["states"]}
            A = {a["agent_id"]: np.round(a["safe_accel_mps2"],2).tolist() for a in rec["returned_actions"]}
            N = {a["agent_id"]: np.round(a["proposed_accel_mps2"],2).tolist() for a in rec["proposed_actions"]}
            dd = np.linalg.norm(P["a"]-P["b"])
            if dd < 40: print(f"  t={rec['timestamp_ms']} d={dd:.2f} closing={(V['a']-V['b'])[2]:.1f} nom={N} safe={A}")
