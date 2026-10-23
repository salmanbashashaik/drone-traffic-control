"""Sprint 1 PR B2: backup-set barrier (FilterConfig(barrier="backup")).

The backup barrier keeps every pair able to escape with PR A's evasive
manoeuvre from the next state, instead of being able to brake to a stop.
"""

import math

import numpy as np
import pytest
from shared.contracts import (
    MAX_ACCEL_COMPONENT_MPS2,
    MAX_SPEED_MPS,
    SAFE_SEPARATION_M,
    SIMULATION_STEP_MS,
    DroneState,
    MARLAction,
    ScenarioConfig,
)

from test_filter_barrier import _sandwich
from utm_services.policy.reference import compute_reference_action
from utm_services.safety.filter import FilterConfig, _backup_steps, _evasive_direction, apply_filter
from utm_services.simulation.runner import run_episode

BACKUP = FilterConfig(barrier="backup")
BRAKING = FilterConfig(barrier="braking")


def _state(agent_id, pos, vel):
    return DroneState(agent_id=agent_id, position_m=pos, velocity_mps=vel, goal_m=(900.0, 900.0, 100.0),
                      timestamp_ms=0)


def _head_on(gap):
    return [
        _state("d1", (500.0 - gap / 2, 500.0, 100.0), (15.0, 0.0, 0.0)),
        _state("d2", (500.0 + gap / 2, 500.0, 100.0), (-15.0, 0.0, 0.0)),
    ]


def _zero(states):
    return [MARLAction(agent_id=s.agent_id, proposed_accel_mps2=(0.0, 0.0, 0.0), timestamp_ms=0) for s in states]


def test_barrier_switch_defaults_to_backup_and_rejects_unknown() -> None:
    assert FilterConfig().barrier == "backup"
    with pytest.raises(ValueError):
        FilterConfig(barrier="cone")


def test_backup_barrier_passes_through_where_braking_intervenes() -> None:
    # 80 m head-on at 30 m/s closing. With gamma = 1 the braking barrier
    # starts acting at about 101 m, the backup barrier at about 68 m: the
    # evasive brake-and-split needs only about 29 m (P2-SA-02 section 2).
    states = _head_on(80.0)
    braking, _ = apply_filter(states, _zero(states), BRAKING, latch={})
    backup, diag = apply_filter(states, _zero(states), BACKUP, latch={})

    assert braking[0].shield_active is True
    assert diag.qp_feasible is True
    assert all(not a.shield_active for a in backup)


def test_backup_barrier_intervenes_near_its_edge() -> None:
    states = _head_on(32.0)
    safe, diag = apply_filter(states, _zero(states), BACKUP, latch={})
    a1, a2 = (np.array(a.safe_accel_mps2) for a in safe)

    assert diag.fallback_used is False
    assert all(a.shield_active for a in safe)
    assert a1[0] < 0.0 < a2[0]


@pytest.mark.parametrize("gap", [40.0, 50.0, 60.0, 80.0])
def test_backup_barrier_head_on_closed_loop(gap) -> None:
    x0 = 500.0 - gap / 2
    drones = [
        {"agent_id": "a", "initial_position_m": (x0, 500.0, 100.0), "initial_velocity_mps": (15.0, 0.0, 0.0),
         "goal_m": (x0 + 400.0, 500.0, 100.0)},
        {"agent_id": "b", "initial_position_m": (x0 + gap, 500.0, 100.0), "initial_velocity_mps": (-15.0, 0.0, 0.0),
         "goal_m": (x0 + gap - 400.0, 500.0, 100.0)},
    ]
    cfg = ScenarioConfig(scenario_id=f"head-on-{gap:.0f}", seed=0, episode_limit_steps=600, drones=drones)
    s = run_episode(cfg, "shielded_marl", policy=compute_reference_action, policy_id="reference-standin",
                    filter_config=BACKUP, keep_records=False).summary
    print(f"\nbackup head-on-{gap:.0f}: min_separation_m={s['min_separation_m']:.2f} steps={s['steps']}")

    assert s["min_separation_m"] >= SAFE_SEPARATION_M
    assert s["fallback_events"] == 0


def test_evasive_direction_brakes_at_least_u_max_over_sqrt2_along_line_of_sight() -> None:
    # The horizon derivation below rests on this bound: n . a >= u_max / sqrt(2).
    n = np.random.default_rng(0).normal(size=(10_000, 3))
    n /= np.linalg.norm(n, axis=1, keepdims=True)
    up = np.where(np.abs(n[:, 2]) > 1e-3, np.sign(n[:, 2]), 1.0)
    along = np.einsum("ij,ij->i", _evasive_direction(n, up), n)

    assert along.min() >= 0.999 * MAX_ACCEL_COMPONENT_MPS2 / math.sqrt(2.0)


@pytest.mark.parametrize("lag_s", [0.0, 0.3])
def test_backup_horizon_covers_stopping_at_max_closing_speed(lag_s) -> None:
    # Two drones closing at 2 v_max, each braking at u_max / sqrt(2) along
    # the line of sight, stop after sqrt(2) v_max / u_max (6.06 s), plus lag.
    derived_s = math.sqrt(2.0) * MAX_SPEED_MPS / MAX_ACCEL_COMPONENT_MPS2 + lag_s
    horizon_s = _backup_steps(FilterConfig(barrier="backup", lag_s=lag_s)) * SIMULATION_STEP_MS / 1000.0

    assert horizon_s >= derived_s


# Unrecoverable by any controller (best achievable 4.18-4.45 m). Non-regression floor,
# not a safety claim. See BL-06.
# Measured 3.740 m at PR #12 (both barriers), floor 0.05 m below, rounded down
# to the centimetre, so a solver or BLAS change cannot break it.
SANDWICH_FLOOR_M = 3.69


def test_backup_sandwich_keeps_its_regression_floor() -> None:
    cfg = ScenarioConfig(scenario_id="sandwich-3", seed=0, episode_limit_steps=600, drones=_sandwich())
    s = run_episode(cfg, "shielded_marl", policy=compute_reference_action, policy_id="reference-standin",
                    filter_config=BACKUP, keep_records=False).summary
    print(f"\nbackup sandwich-3: min_separation_m={s['min_separation_m']:.2f}")

    assert s["min_separation_m"] >= SANDWICH_FLOOR_M
    assert s["fallback_events"] == 0
