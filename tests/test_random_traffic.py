"""End-to-end loop on dense random traffic: simulator -> policy -> filter -> simulator.

The closed-loop tests fly 2-4 drones on hand-built conflicts. These fly 16
drones with seeded random starts and goals, so a change in any box of the loop
(contracts, env, policy, filter, runner) that breaks safety at scale fails here.
"""

import numpy as np
import pytest
from shared.contracts import SAFE_SEPARATION_M, ScenarioConfig

from utm_services.policy.reference import compute_reference_action
from utm_services.simulation.runner import run_episode

N_DRONES = 16
SEEDS = range(5)


def _random_traffic(seed: int) -> ScenarioConfig:
    rng = np.random.default_rng(seed)
    points: list[np.ndarray] = []
    while len(points) < 2 * N_DRONES:  # starts and goals at least 30 m apart
        p = np.array([*rng.uniform(300.0, 700.0, 2), rng.uniform(80.0, 120.0)])
        if all(np.linalg.norm(p - q) >= 30.0 for q in points):
            points.append(p)
    drones = [
        {
            "agent_id": f"d{k}",
            "initial_position_m": tuple(points[k]),
            "initial_velocity_mps": (0.0, 0.0, 0.0),
            "goal_m": tuple(points[N_DRONES + k]),
        }
        for k in range(N_DRONES)
    ]
    return ScenarioConfig(scenario_id=f"random-{N_DRONES}", seed=seed, episode_limit_steps=1000, drones=drones)


def _summary(seed: int, variant: str) -> dict:
    return run_episode(
        _random_traffic(seed), variant, policy=compute_reference_action, policy_id="reference-standin",
        keep_records=False,
    ).summary


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.slow
def test_shielded_dense_random_traffic_is_safe_and_completes(seed) -> None:
    s = _summary(seed, "shielded_marl")
    assert s["min_separation_m"] >= SAFE_SEPARATION_M
    assert s["fallback_events"] == 0
    assert s["n_completed"] == s["n_launched"] == N_DRONES


def test_unshielded_dense_random_traffic_loses_separation() -> None:
    # Guards the scenario itself: if it got too easy, the shielded test would prove nothing.
    assert min(_summary(seed, "unshielded_marl")["min_separation_m"] for seed in SEEDS) < SAFE_SEPARATION_M
