"""Adaptive backup rollout: horizon near walls, the hard cap, and the rollout metrics.

Near a wall the world-box clipping can cut the evasive braking below
u_max / sqrt(2), so the fixed 66-step horizon is not covered by its
derivation there (PR #12 follow-up). The rollout now runs at least 66 steps,
continues while the pair still closes, and stops at a hard cap of 109 steps
(P2-SA-02 §5.3). The near-wall tests passed on the fixed horizon too: the
change closes a gap in the derivation, not an observed failure.
"""

import numpy as np
import pytest
from shared.contracts import SAFE_SEPARATION_M, DroneState, MARLAction, ScenarioConfig

from utm_services.policy.reference import compute_reference_action
from utm_services.safety import filter as F
from utm_services.simulation.runner import run_episode

CFG = F.FilterConfig()
D_EFF = SAFE_SEPARATION_M + CFG.margin_m

# One drone 0.5 m from the wall; its partner 60 m away along u, both at 15 m/s
# towards each other. The near drone's escape (away from the partner) points
# into the wall, so its limits clip the braking.
NEAR_WALL = {
    "floor": ((500.0, 500.0, 30.5), (1.0, 0.3, 0.8)),
    "ceiling": ((500.0, 500.0, 199.5), (1.0, 0.3, -0.8)),
    "x_edge": ((0.5, 500.0, 100.0), (1.0, 0.3, 0.4)),
    "y_edge": ((500.0, 999.5, 100.0), (0.3, -1.0, 0.4)),
}


def _pair(wall: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    p_a, u = (np.array(x) for x in NEAR_WALL[wall])
    u = u / np.linalg.norm(u)
    return p_a, p_a + 60.0 * u, 15.0 * u, -15.0 * u


@pytest.mark.parametrize("wall", NEAR_WALL)
def test_near_wall_max_closing_rollout_stops_closing_within_horizon(wall) -> None:
    p_a, p_b, v_a, v_b = _pair(wall)
    for p_i, p_j, v_i, v_j in ((p_a, p_b, v_a, v_b), (p_b, p_a, v_b, v_a)):  # near drone as i and as j
        traj, _, closing, *_ = F._backup_rollout(p_i[None], p_j[None], v_i[None], v_j[None], CFG)

        assert closing[0] <= 0.0
        assert len(traj) <= 109


@pytest.mark.parametrize("wall", NEAR_WALL)
def test_near_wall_max_closing_episode_stays_outside_d_eff(wall) -> None:
    p_a, p_b, v_a, v_b = _pair(wall)
    drones = [
        {"agent_id": "a", "initial_position_m": tuple(p_a), "initial_velocity_mps": tuple(v_a), "goal_m": tuple(p_b)},
        {"agent_id": "b", "initial_position_m": tuple(p_b), "initial_velocity_mps": tuple(v_b), "goal_m": tuple(p_a)},
    ]
    config = ScenarioConfig(scenario_id=f"wall-{wall}", seed=0, episode_limit_steps=200, drones=drones)
    s = run_episode(config, "shielded_marl", policy=compute_reference_action, policy_id="reference-standin",
                    keep_records=False).summary

    assert s["min_separation_m"] >= D_EFF
    assert s["fallback_events"] == 0
    assert s["rollout_cap_hits"] == 0


def _head_on(gap_m: float = 100.0) -> tuple[list[DroneState], list[MARLAction]]:
    states = [
        DroneState(agent_id="a", position_m=(450.0, 500.0, 100.0), velocity_mps=(15.0, 0.0, 0.0),
                   goal_m=(900.0, 500.0, 100.0), timestamp_ms=0),
        DroneState(agent_id="b", position_m=(450.0 + gap_m, 500.0, 100.0), velocity_mps=(-15.0, 0.0, 0.0),
                   goal_m=(100.0, 500.0, 100.0), timestamp_ms=0),
    ]
    actions = [MARLAction(agent_id=s.agent_id, proposed_accel_mps2=(0.0, 0.0, 0.0), timestamp_ms=0) for s in states]
    return states, actions


def test_rollout_extends_past_the_minimum_while_the_pair_closes(monkeypatch) -> None:
    # Minimum 5 steps: a 30 m/s head-on still closes there, so the rollout
    # continues until it stops closing, below the cap.
    monkeypatch.setattr(F, "_backup_steps", lambda config: 5)
    p_i, p_j = np.array([[450.0, 500.0, 100.0]]), np.array([[550.0, 500.0, 100.0]])
    v_i, v_j = np.array([[15.0, 0.0, 0.0]]), np.array([[-15.0, 0.0, 0.0]])
    traj, _, closing, steps, capped = F._backup_rollout(p_i, p_j, v_i, v_j, CFG)

    assert 5 < steps[0] < F._backup_max_steps(CFG)
    assert len(traj) == steps[0]
    assert closing[0] <= 0.0
    assert not capped[0]


def test_rollout_cap_hit_uses_min_distance_so_far_and_is_flagged(monkeypatch) -> None:
    # Forced: the pair still closes at a 10-step cap.
    monkeypatch.setattr(F, "_backup_steps", lambda config: 5)
    monkeypatch.setattr(F, "_backup_max_steps", lambda config: 10)
    states, actions = _head_on()
    smap = {s.agent_id: s for s in states}

    p_i, p_j = (np.array([smap[k].position_m]) for k in "ab")
    v_i, v_j = (np.array([smap[k].velocity_mps]) for k in "ab")
    traj, a_first, closing, steps, capped = F._backup_rollout(p_i, p_j, v_i, v_j, CFG)
    assert capped[0] and steps[0] == 10 and closing[0] > 0.0

    # The row is built from the closest point reached so far: the last step, k* = 9.
    m, rhs, steps_c, capped_c = F._backup_constraints(smap, [("a", "b")], CFG)
    d_last = float(np.linalg.norm(traj[-1, 0]))
    m_last = traj[-1, 0] / d_last
    dt = 0.1
    expected = float(m_last @ a_first[0]) - CFG.gamma * dt * (d_last - D_EFF) / ((9 + 0.5) * dt * dt)
    assert np.allclose(m[0], m_last)
    assert np.isclose(rhs[0], expected)
    assert capped_c[0] and steps_c[0] == 10

    safe, diag = F.apply_filter(states, actions, CFG, latch={})
    assert all(a.backup_rollout_capped for a in safe)
    assert diag.backup_rollout_cap_pairs == [("a", "b")]
    assert diag.backup_rollout_steps == [10]


def test_episode_summary_counts_cap_hits_and_rollout_steps(monkeypatch) -> None:
    states, _ = _head_on()
    drones = [
        {"agent_id": s.agent_id, "initial_position_m": s.position_m, "initial_velocity_mps": s.velocity_mps,
         "goal_m": s.goal_m}
        for s in states
    ]
    config = ScenarioConfig(scenario_id="head-on", seed=0, episode_limit_steps=5, drones=drones)

    def run(variant: str = "shielded_marl") -> dict:
        return run_episode(config, variant, policy=compute_reference_action, policy_id="reference-standin",
                           keep_records=False).summary

    s = run()
    assert s["rollout_cap_hits"] == 0
    assert s["max_backup_rollout_steps"] >= F._backup_steps(CFG)
    assert s["p99_backup_rollout_steps"] <= s["max_backup_rollout_steps"]

    unshielded = run("unshielded_marl")
    assert unshielded["rollout_cap_hits"] == 0
    assert unshielded["max_backup_rollout_steps"] is None

    monkeypatch.setattr(F, "_backup_steps", lambda config: 5)
    monkeypatch.setattr(F, "_backup_max_steps", lambda config: 10)
    capped = run()
    assert capped["rollout_cap_hits"] == 5  # one pair, capped on each of the 5 steps
    assert capped["max_backup_rollout_steps"] == 10
