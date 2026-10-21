"""Closed-loop tests for the PyBullet quadrotor backend (simulation/pybullet_env.py).

Skipped where pybullet / gym-pybullet-drones are not installed (e.g. Windows).
"""

import pytest

pytest.importorskip("pybullet")
pytest.importorskip("gym_pybullet_drones")

from shared.contracts import SAFE_SEPARATION_M, ScenarioConfig

from utm_services.policy.reference import compute_reference_action
from utm_services.simulation.pybullet_env import PyBulletDroneEnv, _demo_scenarios
from utm_services.simulation.runner import run_episode


def _run(scenario, variant):
    cfg = ScenarioConfig(scenario_id=scenario, seed=0, episode_limit_steps=900, drones=_demo_scenarios()[scenario])
    return run_episode(
        cfg, variant, policy=compute_reference_action, policy_id="reference-standin",
        keep_records=False, make_env=PyBulletDroneEnv,
    )


def test_head_on_unshielded_loses_separation() -> None:
    s = _run("head-on-100", "unshielded_marl").summary
    assert s["min_separation_m"] < SAFE_SEPARATION_M


@pytest.mark.parametrize("scenario", ["head-on-100", "corner-swap"])
def test_shielded_holds_separation_on_quadrotor_physics(scenario) -> None:
    res = _run(scenario, "shielded_marl")
    s = res.summary
    assert res.manifest["env_version"] == "sim-pybullet-v0"
    assert s["min_separation_m"] >= SAFE_SEPARATION_M
    assert s["fallback_events"] == 0 and s["collisions"] == 0
    assert s["n_completed"] == s["n_launched"]
