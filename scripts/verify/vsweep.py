import logging, sys; logging.disable(logging.CRITICAL)
import numpy as np
from shared.contracts import ScenarioConfig
from utm_services.policy.reference import compute_reference_action
from utm_services.simulation.runner import run_episode
import utm_services.safety.filter as F
has_bar = "barrier" in F.FilterConfig.__dataclass_fields__
def d(a,p,g,v): return {"agent_id":a,"initial_position_m":tuple(map(float,p)),"initial_velocity_mps":tuple(map(float,v)),"goal_m":tuple(map(float,g))}
c = np.array([500., 500., 110.])
print("elev_deg offset_m speed | " + " ".join(["backup", "braking"] if has_bar else ["main"]))
for elev in (90, 85, 75, 60):
    for off in (0.0, 0.3, 2.0):
        for spd in (5.0, 10.0):
            e = np.radians(elev); u = np.array([np.cos(e), 0, np.sin(e)])
            pa, pb = c - u*40 + [0, off, 0], c + u*40
            row = []
            for bar in (("backup","braking") if has_bar else ("x",)):
                kw = {"filter_config": F.FilterConfig(step_deadline=False, barrier=bar)} if has_bar else {}
                sc = ScenarioConfig(scenario_id="v", seed=0, episode_limit_steps=250, drones=[d("a",pa,c+u*70+[0,off,0],u*spd), d("b",pb,c-u*70,-u*spd)])
                s = run_episode(sc, "shielded_marl", policy=compute_reference_action, policy_id="ref", keep_records=False, **kw).summary
                row.append(f"{s['min_separation_m']:.2f}{'*' if s['min_separation_m']<5.5 else ' '}{'F' if s['fallback_events'] else ''}")
            print(f"{elev:3d} {off:4.1f} {spd:4.0f} | " + "  ".join(row))
