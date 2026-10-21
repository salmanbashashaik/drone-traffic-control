"""Closed-loop tests: HOCBF filter inside DroneEnv (PR #5 review).

Single-step QP tests cannot show that separation holds over an episode.
These run the shielded variant end to end with the reference controller.
"""

import pytest
from shared.contracts import SAFE_SEPARATION_M, ScenarioConfig

from utm_services.policy.reference import compute_reference_action
from utm_services.simulation.runner import run_episode


def _drone(agent_id, pos, goal, vel=(0.0, 0.0, 0.0)):
    return {"agent_id": agent_id, "initial_position_m": pos, "initial_velocity_mps": vel, "goal_m": goal}


def _run(config):
    return run_episode(
        config, "shielded_marl", policy=compute_reference_action, policy_id="reference-standin",
        keep_records=False,
    ).summary


def test_four_way_corner_swap_holds_separation() -> None:
    corners = [(400.0, 400.0), (600.0, 400.0), (600.0, 600.0), (400.0, 600.0)]
    drones = [
        _drone(f"d{k}", (*corners[k], 100.0), (*corners[(k + 2) % 4], 100.0)) for k in range(4)
    ]
    s = _run(ScenarioConfig(scenario_id="corner-swap", seed=0, episode_limit_steps=900, drones=drones))

    assert s["min_separation_m"] >= SAFE_SEPARATION_M
    assert s["fallback_events"] == 0
    assert s["n_completed"] == 4


@pytest.mark.parametrize("gap", [80.0, 100.0, 150.0])
def test_head_on_at_max_speed_holds_separation_and_completes(gap) -> None:
    # 30 m/s closing; 80 m is inside the pure-braking envelope (≈64 m + margin).
    x0 = 500.0 - gap / 2
    drones = [
        _drone("a", (x0, 500.0, 100.0), (x0 + 400.0, 500.0, 100.0), vel=(15.0, 0.0, 0.0)),
        _drone("b", (x0 + gap, 500.0, 100.0), (x0 + gap - 400.0, 500.0, 100.0), vel=(-15.0, 0.0, 0.0)),
    ]
    s = _run(ScenarioConfig(scenario_id=f"head-on-{gap}", seed=0, episode_limit_steps=600, drones=drones))

    assert s["min_separation_m"] >= SAFE_SEPARATION_M
    assert s["fallback_events"] == 0
    # Tie-break resolves the symmetric deadlock.
    assert s["n_completed"] == 2
