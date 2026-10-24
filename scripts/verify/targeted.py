"""Targeted break attempts against the 5 risky sections (closed loop through run_episode)."""
import logging

import numpy as np
from shared.contracts import ScenarioConfig

from utm_services.policy.reference import compute_reference_action
from utm_services.safety import filter as F
from utm_services.simulation.runner import run_episode

logging.disable(logging.CRITICAL)


def drone(aid, pos, goal, vel=(0.0, 0.0, 0.0)):
    return {"agent_id": aid, "initial_position_m": tuple(map(float, pos)), "initial_velocity_mps": tuple(map(float, vel)),
            "goal_m": tuple(map(float, goal))}


def run(name, drones, cfg=F.FilterConfig(step_deadline=False), steps=400):
    sc = ScenarioConfig(scenario_id=name, seed=0, episode_limit_steps=steps, drones=drones)
    s = run_episode(sc, "shielded_marl", policy=compute_reference_action, policy_id="ref",
                    filter_config=cfg, keep_records=False).summary
    print(f"{name:34s} min_sep={s['min_separation_m']:.2f} depth={s['max_violation_depth_m']:.2f} "
          f"inside={s['time_inside_d_eff_s']:.1f}s fallbacks={s['fallback_events']} aborted={s.get('n_aborted')} "
          f"done={s.get('n_completed')}/{len(drones)} steps={s['steps']}")
    return s


print("== A. world bounds: head-on and sandwich at the altitude floor / ceiling")
for z in (30.5, 199.5):
    run(f"head-on-40 z={z}", [drone("a", (480, 500, z), (900, 500, z), (15, 0, 0)),
                               drone("b", (520, 500, z), (100, 500, z), (-15, 0, 0))])
    run(f"sandwich z={z}", [drone("d1", (490, 500, z), (900, 500, z), (10, 0, 0)), drone("d2", (500, 500, z), (500, 900, z)),
                            drone("d3", (510, 500, z), (100, 500, z), (-10, 0, 0))])

print("== B. symmetric roundabouts: N drones on a circle fly through the centre (odd cycles in the colouring)")
for n in (3, 4, 5, 6):
    for r, spd in ((40.0, 10.0), (60.0, 14.99)):
        ds = []
        for i in range(n):
            th = 2 * np.pi * i / n
            p = np.array([500 + r * np.cos(th), 500 + r * np.sin(th), 100])
            u = -(p - [500, 500, 100]) / r
            ds.append(drone(f"d{i}", p, np.array([500, 500, 100]) + u * 300, u * spd))
        for bar in ("backup", "braking"):
            run(f"ring n={n} r={r:.0f} v={spd:.0f} {bar}", ds, F.FilterConfig(step_deadline=False, barrier=bar))

print("== C. vertical encounters (flat lateral axis branch)")
run("vertical head-on 10 m/s", [drone("a", (500, 500, 60), (500, 500, 190), (0, 0, 10)),
                                drone("b", (500, 500, 120), (500, 500, 35), (0, 0, -10))])
run("vertical sandwich", [drone("a", (500, 500, 90), (500, 500, 190), (0, 0, 10)), drone("b", (500, 500, 100), (900, 500, 100)),
                          drone("c", (500, 500, 110), (500, 500, 35), (0, 0, -10))])


class Broken:
    def setup(self, *a, **k):
        pass

    def solve(self):
        raise RuntimeError("forced")


print("== D. forced solver error every step: fallback only (BL-01/BL-02)")
F.osqp.OSQP = Broken


def broken_daqp(*a, **k):  # small QPs go to DAQP first
    raise RuntimeError("forced")


F.daqp.solve = broken_daqp
run("fallback head-on-40", [drone("a", (480, 500, 100), (900, 500, 100), (15, 0, 0)),
                            drone("b", (520, 500, 100), (100, 500, 100), (-15, 0, 0))])
run("fallback sandwich", [drone("d1", (490, 500, 100), (900, 500, 100), (10, 0, 0)), drone("d2", (500, 500, 100), (500, 900, 100)),
                          drone("d3", (510, 500, 100), (100, 500, 100), (-10, 0, 0))])
for n in (3, 4, 6):
    ds = []
    for i in range(n):
        th = 2 * np.pi * i / n
        p = np.array([500 + 40 * np.cos(th), 500 + 40 * np.sin(th), 100])
        u = -(p - [500, 500, 100]) / 40
        ds.append(drone(f"d{i}", p, np.array([500, 500, 100]) + u * 300, u * 10))
    run(f"fallback ring n={n}", ds)
run("fallback head-on at floor z=31", [drone("a", (480, 500, 31), (900, 500, 31), (15, 0, 0)),
                                        drone("b", (520, 500, 31), (100, 500, 31), (-15, 0, 0))])
