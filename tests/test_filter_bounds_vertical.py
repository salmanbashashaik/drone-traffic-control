"""PR #12 review B1 and B2: world bounds and vertical line of sight.

B1: the escape, backup and fallback manoeuvres must never push a drone out of
the world box (x, y in [0, 1000] m, z in [30, 200] m).
B2: when the line of sight is vertical or near vertical, the sideways escape
direction must be defined and must not swing from step to step.
"""

import numpy as np
import pytest
from shared.contracts import SAFE_SEPARATION_M, ScenarioConfig

from utm_services.policy.reference import compute_reference_action
from utm_services.safety import filter as filter_module
from utm_services.safety.filter import FilterConfig
from utm_services.simulation.runner import run_episode

BARRIERS = ("backup", "braking")
D_EFF = SAFE_SEPARATION_M + FilterConfig().margin_m


class _BrokenOSQP:
    def setup(self, *args, **kwargs):
        pass

    def solve(self):
        raise RuntimeError("simulated solver crash")


def _drone(agent_id, pos, goal, vel):
    return {"agent_id": agent_id, "initial_position_m": tuple(map(float, pos)),
            "initial_velocity_mps": tuple(map(float, vel)), "goal_m": tuple(map(float, goal))}


def _run(drones, barrier, steps=400):
    cfg = ScenarioConfig(scenario_id="bounds-vertical", seed=0, episode_limit_steps=steps, drones=drones)
    return run_episode(cfg, "shielded_marl", policy=compute_reference_action, policy_id="reference-standin",
                       filter_config=FilterConfig(barrier=barrier), keep_records=False).summary


def _head_on_at(z0):
    return [_drone("a", (480, 500, z0), (900, 500, z0), (15, 0, 0)),
            _drone("b", (520, 500, z0), (100, 500, z0), (-15, 0, 0))]


# ---------------------------------------------------------------------------
# B1: world bounds
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("barrier", BARRIERS)
@pytest.mark.parametrize("z0", [30.5, 35.0, 40.0, 195.0, 199.5])
def test_head_on_near_floor_or_ceiling_stays_in_the_world(z0, barrier) -> None:
    s = _run(_head_on_at(z0), barrier)

    assert s["n_aborted"] == 0
    assert s["min_separation_m"] >= SAFE_SEPARATION_M


@pytest.mark.parametrize("z0", [30.5, 35.0, 40.0, 195.0, 199.5])
def test_forced_fallback_head_on_near_floor_or_ceiling_stays_in_the_world(z0, monkeypatch) -> None:
    monkeypatch.setattr(filter_module.osqp, "OSQP", _BrokenOSQP)
    s = _run(_head_on_at(z0), "backup")

    assert s["fallback_events"] > 0
    assert s["n_aborted"] == 0


@pytest.mark.parametrize("barrier", BARRIERS)
@pytest.mark.parametrize("y0", [0.5, 999.5])
def test_head_on_along_a_world_edge_stays_in_the_world(y0, barrier) -> None:
    drones = [_drone("a", (480, y0, 100), (900, y0, 100), (15, 0, 0)),
              _drone("b", (520, y0, 100), (100, y0, 100), (-15, 0, 0))]
    s = _run(drones, barrier)

    assert s["n_aborted"] == 0


# ---------------------------------------------------------------------------
# B2: vertical and near-vertical line of sight
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("barrier", BARRIERS)
def test_vertical_head_on_keeps_d_eff(barrier) -> None:
    # Braking alone needs about 34 m at 20 m/s closing; the drones start 60 m apart.
    drones = [_drone("a", (500, 500, 60), (500, 500, 190), (0, 0, 10)),
              _drone("b", (500, 500, 120), (500, 500, 35), (0, 0, -10))]
    s = _run(drones, barrier, steps=200)

    assert s["min_separation_m"] >= D_EFF
    assert s["n_aborted"] == 0


@pytest.mark.parametrize("barrier", BARRIERS)
@pytest.mark.parametrize("elevation_deg", np.arange(80.0, 90.01, 0.5).tolist())
def test_near_vertical_head_on_keeps_d_eff(elevation_deg, barrier) -> None:
    # 80 m apart, 20 m/s closing along a line of sight elevation_deg above horizontal: recoverable.
    centre = np.array([500.0, 500.0, 110.0])
    e = np.radians(elevation_deg)
    u = np.array([np.cos(e), 0.0, np.sin(e)])
    drones = [_drone("a", centre - 40 * u, centre + 70 * u, 10 * u),
              _drone("b", centre + 40 * u, centre - 70 * u, -10 * u)]
    s = _run(drones, barrier, steps=250)

    assert s["min_separation_m"] >= D_EFF
    assert s["n_aborted"] == 0
