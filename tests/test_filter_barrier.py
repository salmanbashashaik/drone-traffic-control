"""Sprint 1 PR B: bounded barrier inside d_eff, violation metrics in metres,
escape in all three axes when separation cannot be kept, crossing liveness.
"""

import math

import numpy as np
import pytest
from shared.contracts import (
    MAX_ACCEL_COMPONENT_MPS2,
    SAFE_SEPARATION_M,
    DroneState,
    MARLAction,
    ScenarioConfig,
)

from utm_services.policy.reference import compute_reference_action
from utm_services.safety.filter import FilterConfig, _pair_constraint, apply_filter
from utm_services.simulation.runner import run_episode


def _state(agent_id, pos, vel=(0.0, 0.0, 0.0), goal=(900.0, 900.0, 100.0)):
    return DroneState(agent_id=agent_id, position_m=pos, velocity_mps=vel, goal_m=goal, timestamp_ms=0)


def _drone(agent_id, pos, goal, vel=(0.0, 0.0, 0.0)):
    return {"agent_id": agent_id, "initial_position_m": pos, "initial_velocity_mps": vel, "goal_m": goal}


def _run(scenario_id, drones, variant="shielded_marl", steps=600):
    cfg = ScenarioConfig(scenario_id=scenario_id, seed=0, episode_limit_steps=steps, drones=drones)
    return run_episode(cfg, variant, policy=compute_reference_action,
                       policy_id="reference-standin", keep_records=False).summary


def _sandwich():
    return [
        _drone("d1", (490.0, 500.0, 100.0), (900.0, 500.0, 100.0), vel=(10.0, 0.0, 0.0)),
        _drone("d2", (500.0, 500.0, 100.0), (500.0, 900.0, 100.0)),
        _drone("d3", (510.0, 500.0, 100.0), (100.0, 500.0, 100.0), vel=(-10.0, 0.0, 0.0)),
    ]


# ---------------------------------------------------------------------------
# Barrier stays well-defined and bounded inside d_eff
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("dist", [5.49, 5.0, 3.0, 1.0])
def test_constraint_is_bounded_inside_d_eff(dist) -> None:
    """Closing at 30 m/s inside d_eff (5.5 m) must give a bounded rhs.

    With the clamped square root the rhs was -2 u_max rho / 1e-3 = 2.1e5 m/s^2.
    Bound: |rhs| <= gamma |h| + g_max |rho| + v_t^2 / d, with g_max = 2 u_max / c.
    """
    cfg = FilterConfig()
    s_i = _state("i", (500.0 - dist, 500.0, 100.0), (15.0, 0.0, 0.0))
    s_j = _state("j", (500.0, 500.0, 100.0), (-15.0, 0.0, 0.0))
    _, rhs, h = _pair_constraint(s_i, s_j, cfg)

    assert math.isfinite(rhs) and math.isfinite(h)
    assert h < 0.0
    g_max = 2.0 * MAX_ACCEL_COMPONENT_MPS2 / cfg.barrier_offset_mps
    assert abs(rhs) <= cfg.gamma * abs(h) + g_max * 30.0 + 1e-9


def test_barrier_is_continuous_across_d_eff() -> None:
    cfg = FilterConfig()
    d_eff = SAFE_SEPARATION_M + cfg.margin_m
    values = []
    for dist in (d_eff - 1e-6, d_eff + 1e-6):
        s_i = _state("i", (500.0 - dist, 500.0, 100.0), (5.0, 0.0, 0.0))
        s_j = _state("j", (500.0, 500.0, 100.0))
        values.append(_pair_constraint(s_i, s_j, cfg)[1:])
    (rhs_in, h_in), (rhs_out, h_out) = values

    assert h_in == pytest.approx(h_out, abs=1e-4)
    assert rhs_in == pytest.approx(rhs_out, abs=1e-3)


# ---------------------------------------------------------------------------
# Violation metrics in metres and seconds
# ---------------------------------------------------------------------------

def test_episode_reports_violation_depth_and_time_inside_d_eff() -> None:
    s = _run("sandwich-3", _sandwich(), steps=60)
    d_eff = SAFE_SEPARATION_M + FilterConfig().margin_m

    assert "total_slack_mps2" not in s
    assert 0.0 < s["max_violation_depth_m"] <= d_eff
    assert s["max_violation_depth_m"] == pytest.approx(d_eff - s["min_separation_m"], abs=0.5)
    assert s["time_inside_d_eff_s"] > 0.0
    assert s["time_inside_d_eff_s"] == pytest.approx(round(s["time_inside_d_eff_s"] * 10) / 10)
    pairs = s["pair_violations"]
    assert set(pairs) <= {"d1|d2", "d1|d3", "d2|d3"}
    assert max(p["max_depth_m"] for p in pairs.values()) == s["max_violation_depth_m"]
    assert sum(p["time_inside_s"] for p in pairs.values()) == pytest.approx(s["time_inside_d_eff_s"])


def test_unshielded_variant_reports_the_same_metrics() -> None:
    s = _run("sandwich-3", _sandwich(), variant="unshielded_marl", steps=60)

    assert s["max_violation_depth_m"] > 0.0
    assert s["time_inside_d_eff_s"] > 0.0


def test_filter_reports_violation_depth_per_pair() -> None:
    states = [_state("d1", (497.0, 500.0, 100.0)), _state("d2", (500.0, 500.0, 100.0))]
    actions = [MARLAction(agent_id=s.agent_id, proposed_accel_mps2=(0.0, 0.0, 0.0), timestamp_ms=0)
               for s in states]
    _, diag = apply_filter(states, actions)

    assert diag.pair_violation_depth_m == {("d1", "d2"): pytest.approx(2.5)}


# ---------------------------------------------------------------------------
# Unavoidable violation: minimise it in metres, using all three axes
# ---------------------------------------------------------------------------

# Best min separation over all three pairs found by direct optimisation of
# 100 ms piecewise-constant accelerations (SLSQP, 8 starts; a lower bound on
# the optimum). The analytic upper bound is 4.45 m (test_filter_safety_core).
# Keeping only d1|d2 and d2|d3 gives 4.441 m, but that strategy sends d1 and
# d3 the same way laterally and they then collide; the d1|d3 pair costs 0.27 m.
SANDWICH_BEST_FOUND_M = 4.168


def test_sandwich_escape_uses_all_three_axes() -> None:
    states = [_state(d["agent_id"], d["initial_position_m"], d["initial_velocity_mps"]) for d in _sandwich()]
    actions = [MARLAction(agent_id=s.agent_id, proposed_accel_mps2=(0.0, 0.0, 0.0), timestamp_ms=0)
               for s in states]
    safe, _ = apply_filter(states, actions)
    a = {s.agent_id: np.array(s.safe_accel_mps2) for s in safe}

    # The structure of the optimum: outer drones brake, the middle drone
    # escapes opposite to both in y, and the outer drones split in z.
    assert a["d1"][0] < 0.0 < a["d3"][0]
    assert abs(a["d2"][1]) > 1.0
    assert a["d1"][1] * a["d2"][1] < 0.0
    assert a["d3"][1] * a["d2"][1] < 0.0
    assert a["d1"][2] * a["d3"][2] < 0.0
    assert min(abs(a["d1"][2]), abs(a["d3"][2])) > 1.0


def test_sandwich_approaches_best_possible_separation() -> None:
    s = _run("sandwich-3", _sandwich())
    print(f"\nsandwich-3: min_separation_m={s['min_separation_m']:.2f} "
          f"best found {SANDWICH_BEST_FOUND_M} m, upper bound 4.45 m")

    assert s["min_separation_m"] >= 0.9 * SANDWICH_BEST_FOUND_M
    assert s["fallback_events"] == 0


# ---------------------------------------------------------------------------
# Crossing liveness
# ---------------------------------------------------------------------------

# X = 5 %. A 90-degree crossing at 15 m/s needs one drone delayed by at least
# d_eff * sqrt(2) / 15 = 0.52 s (perpendicular movers offset in time by T pass
# at 15 T / sqrt(2)). That is 1.8 % of the 296-step unconstrained episode. 5 %
# (about 1.5 s) leaves roughly 3x headroom for a one-step filter that is not
# time-optimal, and still catches the side-by-side lock seen in PR A (+17.6 %).
CROSSING_MAX_EXTRA_FRACTION = 0.05


def test_crossing_takes_at_most_x_percent_longer_than_unconstrained() -> None:
    drones = [
        _drone("a", (450.0, 500.0, 100.0), (850.0, 500.0, 100.0), vel=(15.0, 0.0, 0.0)),
        _drone("b", (500.0, 450.0, 100.0), (500.0, 850.0, 100.0), vel=(0.0, 15.0, 0.0)),
    ]
    free = _run("crossing-90", drones, variant="reference", steps=1000)["steps"]
    s = _run("crossing-90", drones, steps=1000)
    print(f"\ncrossing-90: {s['steps']} steps vs {free} unconstrained "
          f"(+{100 * (s['steps'] / free - 1):.1f} %), min_separation_m={s['min_separation_m']:.2f}")

    assert s["n_completed"] == 2
    assert s["min_separation_m"] >= SAFE_SEPARATION_M
    assert s["steps"] <= (1 + CROSSING_MAX_EXTRA_FRACTION) * free
