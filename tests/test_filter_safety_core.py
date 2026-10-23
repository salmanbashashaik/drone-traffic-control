"""Sprint 1 PR A: always-feasible QP, evasive fallback, per-cluster fallback.

Unit tests drive apply_filter directly; the regression scenarios run the
shielded variant end to end and print the minimum separation of each case.
"""

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
from utm_services.safety import filter as filter_module
from utm_services.safety.filter import apply_filter
from utm_services.simulation.runner import run_episode


def _state(agent_id, pos, vel=(0.0, 0.0, 0.0), goal=(900.0, 900.0, 100.0)):
    return DroneState(agent_id=agent_id, position_m=pos, velocity_mps=vel, goal_m=goal, timestamp_ms=0)


def _action(agent_id, accel=(0.0, 0.0, 0.0)):
    return MARLAction(agent_id=agent_id, proposed_accel_mps2=accel, timestamp_ms=0)


def _sandwich():
    states = [
        _state("d1", (490.0, 500.0, 100.0), (10.0, 0.0, 0.0)),
        _state("d2", (500.0, 500.0, 100.0)),
        _state("d3", (510.0, 500.0, 100.0), (-10.0, 0.0, 0.0)),
    ]
    return states, [_action(s.agent_id) for s in states]


# ---------------------------------------------------------------------------
# Slack: the QP is always feasible and reports per-pair violation
# ---------------------------------------------------------------------------

def test_sandwich_is_solved_with_slack_not_fallback() -> None:
    states, actions = _sandwich()
    safe, diag = apply_filter(states, actions)

    assert diag.qp_feasible is True
    assert diag.fallback_used is False
    assert all(not a.fallback_used for a in safe)
    assert set(diag.pair_slack_mps2) <= {("d1", "d2"), ("d1", "d3"), ("d2", "d3")}
    assert diag.pair_slack_mps2


def test_escape_solve_failure_keeps_the_first_solve(monkeypatch) -> None:
    # PR #12 review: a failed escape re-solve must not discard the valid first solve.
    states, actions = _sandwich()
    real_solve = filter_module._solve_hocbf_qp
    first_solves = []

    def escape_fails(state_map, action_map, pairs, config, escape_signs=None, backup=None, frames=None):
        if escape_signs:
            return {}, {}, False
        first_solves.append(real_solve(state_map, action_map, pairs, config, backup=backup, frames=frames))
        return first_solves[-1]

    monkeypatch.setattr(filter_module, "_solve_hocbf_qp", escape_fails)
    safe, diag = apply_filter(states, actions)

    assert first_solves and first_solves[0][2] is True
    assert diag.qp_feasible is True
    assert diag.fallback_used is False
    assert {a.agent_id: a.safe_accel_mps2 for a in safe} == first_solves[0][0]
    # PR #12 re-review: the kept first solve carries slack, so it is flagged.
    assert diag.pair_slack_mps2
    assert diag.escape_failed_slack_pairs == sorted(diag.pair_slack_mps2)
    flagged = {aid for pair in diag.pair_slack_mps2 for aid in pair}
    assert {a.agent_id for a in safe if a.escape_failed_slack} == flagged


def test_escape_failures_with_slack_are_counted_in_the_summary(monkeypatch) -> None:
    drones = [
        _drone("d1", (490.0, 500.0, 100.0), (900.0, 500.0, 100.0), vel=(10.0, 0.0, 0.0)),
        _drone("d2", (500.0, 500.0, 100.0), (500.0, 900.0, 100.0)),
        _drone("d3", (510.0, 500.0, 100.0), (100.0, 500.0, 100.0), vel=(-10.0, 0.0, 0.0)),
    ]
    assert _run("sandwich-3", drones, steps=60)["escape_failures_with_slack"] == 0

    real_solve = filter_module._solve_hocbf_qp

    def escape_fails(state_map, action_map, pairs, config, escape_signs=None, backup=None, frames=None):
        return ({}, {}, False) if escape_signs else real_solve(state_map, action_map, pairs, config, backup=backup, frames=frames)

    monkeypatch.setattr(filter_module, "_solve_hocbf_qp", escape_fails)
    s = _run("sandwich-3", drones, steps=60)

    assert s["escape_failures_with_slack"] > 0
    assert s["fallback_events"] == 0


def test_feasible_step_reports_no_slack() -> None:
    # 100 m apart, closing at 30 m/s: outside the braking curve, no conflict.
    states = [
        _state("d1", (450.0, 500.0, 100.0), (15.0, 0.0, 0.0)),
        _state("d2", (550.0, 500.0, 100.0), (-15.0, 0.0, 0.0)),
    ]
    _, diag = apply_filter(states, [_action("d1"), _action("d2")])

    assert diag.qp_feasible is True
    assert diag.pair_slack_mps2 == {}
    assert diag.pair_violation_depth_m == {}


def test_runner_summary_reports_violation_in_metres() -> None:
    drones = [
        {"agent_id": "a", "initial_position_m": (480.0, 500.0, 100.0),
         "initial_velocity_mps": (15.0, 0.0, 0.0), "goal_m": (900.0, 500.0, 100.0)},
        {"agent_id": "b", "initial_position_m": (520.0, 500.0, 100.0),
         "initial_velocity_mps": (-15.0, 0.0, 0.0), "goal_m": (100.0, 500.0, 100.0)},
    ]
    cfg = ScenarioConfig(scenario_id="head-on-40", seed=0, episode_limit_steps=50, drones=drones)
    s = run_episode(cfg, "shielded_marl", policy=compute_reference_action,
                    policy_id="reference-standin", keep_records=False).summary

    # Head-on 40 m keeps d_eff: nothing to report.
    assert s["max_violation_depth_m"] == 0.0
    assert s["time_inside_d_eff_s"] == 0.0
    assert s["pair_violations"] == {}


# ---------------------------------------------------------------------------
# Fallback: solver errors only, evasive, per conflict cluster
# ---------------------------------------------------------------------------

class _BrokenOSQP:
    def setup(self, *args, **kwargs):
        pass

    def solve(self):
        raise RuntimeError("simulated solver crash")


def _head_on_plus_bystander():
    states = [
        _state("d1", (480.0, 500.0, 100.0), (15.0, 0.0, 0.0)),
        _state("d2", (520.0, 500.0, 100.0), (-15.0, 0.0, 0.0)),
        _state("d3", (900.0, 100.0, 100.0)),
    ]
    actions = [_action("d1"), _action("d2"), _action("d3", (1.0, 0.0, 0.0))]
    return states, actions


def test_solver_error_fallback_is_evasive_and_bounded(monkeypatch) -> None:
    monkeypatch.setattr(filter_module.osqp, "OSQP", _BrokenOSQP)
    states, actions = _head_on_plus_bystander()
    safe, diag = apply_filter(states, actions)
    by_id = {a.agent_id: a for a in safe}
    a1 = np.array(by_id["d1"].safe_accel_mps2)
    a2 = np.array(by_id["d2"].safe_accel_mps2)

    assert diag.qp_feasible is False
    assert diag.fallback_used is True
    assert by_id["d1"].fallback_used and by_id["d2"].fallback_used
    # Brake: each accelerates away from the other along the line of sight.
    assert a1[0] < 0.0 and a2[0] > 0.0
    # Escape: opposite lateral and opposite vertical directions.
    assert a1[1] * a2[1] < 0.0
    assert a1[2] * a2[2] < 0.0
    for a in (a1, a2):
        assert np.all(np.abs(a) <= MAX_ACCEL_COMPONENT_MPS2)
        assert np.max(np.abs(a)) == pytest.approx(MAX_ACCEL_COMPONENT_MPS2)


def test_solver_error_fallback_is_per_cluster(monkeypatch) -> None:
    monkeypatch.setattr(filter_module.osqp, "OSQP", _BrokenOSQP)
    states, actions = _head_on_plus_bystander()
    safe, _ = apply_filter(states, actions)
    bystander = {a.agent_id: a for a in safe}["d3"]

    assert bystander.fallback_used is False
    assert bystander.qp_feasible is True
    assert bystander.safe_accel_mps2 == (1.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# Closed-loop regression scenarios
# ---------------------------------------------------------------------------

def _drone(agent_id, pos, goal, vel=(0.0, 0.0, 0.0)):
    return {"agent_id": agent_id, "initial_position_m": pos, "initial_velocity_mps": vel, "goal_m": goal}


def _run(scenario_id, drones, steps=600):
    cfg = ScenarioConfig(scenario_id=scenario_id, seed=0, episode_limit_steps=steps, drones=drones)
    s = run_episode(cfg, "shielded_marl", policy=compute_reference_action,
                    policy_id="reference-standin", keep_records=False).summary
    print(f"\n{scenario_id}: min_separation_m={s['min_separation_m']:.2f} "
          f"max_violation_depth_m={s['max_violation_depth_m']:.2f} "
          f"time_inside_d_eff_s={s['time_inside_d_eff_s']:.1f} fallback_events={s['fallback_events']}")
    return s


@pytest.mark.parametrize("gap", [40.0, 50.0, 60.0, 80.0])
def test_head_on_regression(gap) -> None:
    x0 = 500.0 - gap / 2
    drones = [
        _drone("a", (x0, 500.0, 100.0), (x0 + 400.0, 500.0, 100.0), vel=(15.0, 0.0, 0.0)),
        _drone("b", (x0 + gap, 500.0, 100.0), (x0 + gap - 400.0, 500.0, 100.0), vel=(-15.0, 0.0, 0.0)),
    ]
    s = _run(f"head-on-{gap:.0f}", drones)

    assert s["min_separation_m"] >= SAFE_SEPARATION_M
    assert s["fallback_events"] == 0


def _crossing():
    # Both 50 m from the crossing point at 15 m/s: they arrive together.
    return [
        _drone("a", (450.0, 500.0, 100.0), (850.0, 500.0, 100.0), vel=(15.0, 0.0, 0.0)),
        _drone("b", (500.0, 450.0, 100.0), (500.0, 850.0, 100.0), vel=(0.0, 15.0, 0.0)),
    ]


def test_crossing_90_degrees_regression() -> None:
    s = _run("crossing-90", _crossing())

    assert s["min_separation_m"] >= SAFE_SEPARATION_M
    assert s["fallback_events"] == 0


def test_same_seed_runs_are_identical() -> None:
    # OSQP's default adaptive-rho interval depends on wall-clock setup time;
    # before it was pinned, this scenario hit a solver error on some runs only.
    keys = ("min_separation_m", "max_violation_depth_m", "time_inside_d_eff_s", "fallback_events", "steps")
    first = _run("crossing-90", _crossing())
    second = _run("crossing-90", _crossing())

    assert {k: first[k] for k in keys} == {k: second[k] for k in keys}


def test_three_drone_sandwich_is_physically_unrecoverable() -> None:
    """d1 and d3 close on a hovering d2 at 10 m/s from 10 m each side.

    No controller can keep 5 m here under the +-3.5 m/s^2 per-axis limit.
    Take the pair that d2's x-acceleration does not help (by symmetry, any x
    action helps one pair and hurts the other, so the worse pair gets at most
    the outer drone's own 3.5 m/s^2 of relative x deceleration; closing, both
    drones can add up to 7 m/s^2 of relative acceleration):

        x(t) >= 10 - 10 t - 3.5 t^2  and  x(t) <= 10 - 10 t + 1.75 t^2
        at t = 0.8 s: -0.24 m <= x <= 10 - 8 + 1.12 = 3.12 m, so |x| <= 3.12 m

    Lateral (y, z) relative acceleration is at most 7 m/s^2 per axis
    (3.5 each drone, opposite signs), so at most 7 * sqrt(2) = 9.90 m/s^2 in
    magnitude. Lateral velocity starts at zero, so the lateral offset is

        L(t) <= 0.5 * 9.90 * t^2  ->  L(0.8 s) <= 3.17 m

    Separation at t = 0.8 s is therefore at most sqrt(3.12^2 + 3.17^2) =
    4.45 m < 5 m for the best possible controller. The test checks this bound
    numerically, checks the filter does not fall back, and prints what the
    filter achieves.
    """
    t, u = 0.8, MAX_ACCEL_COMPONENT_MPS2
    x_bound = 10.0 - 10.0 * t + 0.5 * u * t**2
    lateral_bound = 0.5 * 2 * u * np.sqrt(2.0) * t**2
    best_possible = float(np.hypot(x_bound, lateral_bound))
    assert best_possible < SAFE_SEPARATION_M

    drones = [
        _drone("d1", (490.0, 500.0, 100.0), (900.0, 500.0, 100.0), vel=(10.0, 0.0, 0.0)),
        _drone("d2", (500.0, 500.0, 100.0), (500.0, 900.0, 100.0)),
        _drone("d3", (510.0, 500.0, 100.0), (100.0, 500.0, 100.0), vel=(-10.0, 0.0, 0.0)),
    ]
    s = _run("sandwich-3", drones)
    print(f"sandwich-3: physical upper bound on min separation = {best_possible:.2f} m")

    assert s["min_separation_m"] <= best_possible
    assert s["fallback_events"] == 0
    assert s["time_inside_d_eff_s"] > 0.0
