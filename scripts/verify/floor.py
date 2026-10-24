import logging; logging.disable(logging.CRITICAL)
from shared.contracts import ScenarioConfig
from utm_services.policy.reference import compute_reference_action
from utm_services.safety.filter import FilterConfig
from utm_services.simulation.runner import run_episode
import utm_services.safety.filter as F
def d(a,p,g,v): return {"agent_id":a,"initial_position_m":p,"initial_velocity_mps":v,"goal_m":g}
for z in (30.5, 35.0, 40.0):
  for bar in ("backup","braking"):
    kw = {"filter_config": FilterConfig(step_deadline=False, barrier=bar)} if "barrier" in F.FilterConfig.__dataclass_fields__ else {}
    if not kw and bar=="backup": continue
    sc = ScenarioConfig(scenario_id="x", seed=0, episode_limit_steps=400, drones=[d("a",(480.,500.,z),(900.,500.,z),(15.,0.,0.)), d("b",(520.,500.,z),(100.,500.,z),(-15.,0.,0.))])
    r = run_episode(sc, "shielded_marl", policy=compute_reference_action, policy_id="ref", keep_records=True, **kw)
    zmin = {}; az = {}
    for rec in r.records:
        for s in rec["states"]:
            zmin[s["agent_id"]] = min(zmin.get(s["agent_id"], 1e9), s["position_m"][2])
        for a in rec.get("safe_actions", []):
            az.setdefault(a["agent_id"], []).append(round(a["safe_accel_mps2"][2],2))
    reasons = {k: v for k, v in r.summary.items() if "abort" in k}
    print(f"z0={z} {bar}: aborted={r.summary['n_aborted']} min_z={ {k: round(v,2) for k,v in zmin.items()} } {reasons}")
    print("   az(b) first 12 steps:", az.get("b", [])[:12])
