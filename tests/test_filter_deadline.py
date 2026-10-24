"""Bounded step time: per-cluster solve deadlines and the backup on a missed deadline (contract v1.5).

Two separate encounters in one sky make two conflict clusters. A forced or
real deadline miss on one cluster makes its drones fly their backup manoeuvre
(flagged ``deadline_fallback``); the other cluster's solution is unchanged.
"""

import itertools
import time

import numpy as np
from shared.contracts import DroneState, MARLAction

from utm_services.safety import filter as F


def _two_clusters():
    # Cluster a: three drones converging hard near (300, 500); cluster b: three
    # drones converging less hard near (500, 700). Pairs across them cannot bind.
    p, v = [], []
    for c, r, speed in (((300.0, 500.0, 100.0), 18.0, 12.0), ((500.0, 700.0, 100.0), 25.0, 10.0)):
        for ang in (0.0, 2.1, 4.2):
            d = np.array([np.cos(ang), np.sin(ang), 0.05])
            p.append(np.array(c) + r * d)
            v.append(-speed * d / np.linalg.norm(d))
    ids = ["a0", "a1", "a2", "b0", "b1", "b2"]
    states = [DroneState(agent_id=i, position_m=tuple(p[k]), velocity_mps=tuple(v[k]), goal_m=(900.0, 900.0, 100.0),
                         timestamp_ms=0) for k, i in enumerate(ids)]
    actions = [MARLAction(agent_id=i, proposed_accel_mps2=(0.0, 0.0, 0.0), timestamp_ms=0) for i in ids]
    return states, actions


def _run(states, actions):
    safe, diag = F.apply_filter(states, actions, F.FilterConfig(), latch={})
    return {a.agent_id: a for a in safe}, diag


def test_two_clusters_are_solved_separately_most_urgent_first(monkeypatch) -> None:
    order = []
    real = F._solve_one_qp

    def record(states, *args, **kwargs):
        order.append(tuple(sorted(states)))
        return real(states, *args, **kwargs)

    monkeypatch.setattr(F, "_solve_one_qp", record)
    safe, diag = _run(*_two_clusters())
    assert order[:2] == [("a0", "a1", "a2"), ("b0", "b1", "b2")]  # a is closer: smaller backup margin
    assert not diag.deadline_fallback_agents and not diag.fallback_used
    assert all(not a.deadline_fallback for a in safe.values())


def test_forced_deadline_on_one_cluster_backs_it_up_and_leaves_the_other(monkeypatch) -> None:
    states, actions = _two_clusters()
    normal, _ = _run(states, actions)
    real = F._solve_one_qp

    def a_misses(states_, *args, **kwargs):
        return F._DEADLINE if "a0" in states_ else real(states_, *args, **kwargs)

    monkeypatch.setattr(F, "_solve_one_qp", a_misses)
    safe, diag = _run(states, actions)

    assert diag.qp_feasible is True  # a deadline miss is not a solver error
    assert diag.deadline_fallback_agents == ["a0", "a1", "a2"]
    state_map = {s.agent_id: s for s in states}
    cluster_pairs = [("a0", "a1"), ("a0", "a2"), ("a1", "a2")]
    for aid in ("a0", "a1", "a2"):
        assert safe[aid].deadline_fallback and safe[aid].fallback_used and not safe[aid].qp_feasible
        backup = F._evasive_accel(aid, state_map, cluster_pairs, F.FilterConfig(), {})
        assert np.allclose(safe[aid].safe_accel_mps2, backup, atol=1e-12)
    for aid in ("b0", "b1", "b2"):
        assert not safe[aid].deadline_fallback and not safe[aid].fallback_used
        assert safe[aid].safe_accel_mps2 == normal[aid].safe_accel_mps2  # bit for bit


def test_a_slow_cluster_does_not_delay_the_other(monkeypatch) -> None:
    # Cluster a (solved first, most urgent) gets a solver that runs until its
    # time limit and then gives up, as DAQP and OSQP do at their time limit.
    # Cluster b still gets its own share of the window and its normal solution,
    # and the step ends within the solve window plus the reserve.
    states, actions = _two_clusters()
    normal, _ = _run(states, actions)
    real = F._solve_exact

    def slow_for_a(P, q, A, l, u, config, escape, deadline=None):
        if P.shape[0] == 3 * 3 + 3 and slow_for_a.calls == 0:
            slow_for_a.calls += 1
            while time.perf_counter() < deadline:
                pass
            return F._DEADLINE
        return real(P, q, A, l, u, config, escape, deadline)

    slow_for_a.calls = 0
    monkeypatch.setattr(F, "_solve_exact", slow_for_a)
    t0 = time.perf_counter()
    safe, diag = _run(states, actions)
    elapsed = time.perf_counter() - t0

    assert diag.deadline_fallback_agents == ["a0", "a1", "a2"]
    for aid in ("b0", "b1", "b2"):
        assert not safe[aid].deadline_fallback
        assert safe[aid].safe_accel_mps2 == normal[aid].safe_accel_mps2
    assert elapsed < F._STEP_BUDGET_S + 0.05  # the window plus the backup and diagnostics, with CI margin


def test_cluster_deadlines_guarantee_each_share() -> None:
    now = time.perf_counter()
    end = now + 0.06
    sizes = [10, 30, 20]
    deadlines = F._cluster_deadlines(end, sizes)
    assert deadlines[-1] == end
    assert all(a < b for a, b in itertools.pairwise(deadlines))
    # Starting problem k at the previous deadline (the worst case) leaves it its share.
    window = end - now
    starts = [now, *deadlines[:-1]]
    for k, size in enumerate(sizes):
        assert deadlines[k] - starts[k] >= window * size / sum(sizes) - 1e-3
    assert F._cluster_deadlines(None, sizes) == [None, None, None]
    assert F._cluster_deadlines(end, []) == []


def test_episode_summary_counts_deadline_fallbacks(monkeypatch) -> None:
    from shared.contracts import ScenarioConfig

    from utm_services.policy.reference import compute_reference_action
    from utm_services.simulation.runner import run_episode

    drones = [{"agent_id": "a", "initial_position_m": (480.0, 500.0, 100.0), "initial_velocity_mps": (10.0, 0.0, 0.0),
               "goal_m": (900.0, 500.0, 100.0)},
              {"agent_id": "b", "initial_position_m": (520.0, 500.0, 100.0), "initial_velocity_mps": (-10.0, 0.0, 0.0),
               "goal_m": (100.0, 500.0, 100.0)}]
    cfg = ScenarioConfig(scenario_id="head-on", seed=0, episode_limit_steps=30, drones=drones)
    monkeypatch.setattr(F, "_solve_one_qp", lambda *a, **k: F._DEADLINE)
    s = run_episode(cfg, "shielded_marl", policy=compute_reference_action, policy_id="reference-standin",
                    keep_records=False).summary
    assert s["deadline_fallbacks"] > 0
    assert s["fallback_events"] >= s["deadline_fallbacks"]
