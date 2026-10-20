"""Tests for the deterministic reference controller (policy/reference.py)."""

import math

import pytest

from shared.contracts import MAX_ACCEL_COMPONENT_MPS2, MARLAction, ObservationV1
from utm_services.policy.reference import compute_reference_action


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _obs(
    pos=(100.0, 200.0, 80.0),
    vel=(0.0, 0.0, 0.0),
    goal_disp=(10.0, 0.0, 0.0),
    neighbor_rel_pos=(0.0, 0.0, 0.0),
    neighbor_rel_vel=(0.0, 0.0, 0.0),
    neighbor_dist=-1.0,
    shield=0.0,
    barrier=1.0,
) -> ObservationV1:
    values = list(pos) + list(vel) + list(goal_disp) + list(neighbor_rel_pos) + list(neighbor_rel_vel) + [neighbor_dist, shield, barrier]
    return ObservationV1.model_validate(values)


# ---------------------------------------------------------------------------
# Basic output contract
# ---------------------------------------------------------------------------

def test_returns_marl_action() -> None:
    obs = _obs()
    result = compute_reference_action(obs, "drone-001", 0)
    assert isinstance(result, MARLAction)


def test_agent_id_preserved() -> None:
    obs = _obs()
    result = compute_reference_action(obs, "drone-abc", 500)
    assert result.agent_id == "drone-abc"


def test_timestamp_preserved() -> None:
    obs = _obs()
    result = compute_reference_action(obs, "drone-001", 1200)
    assert result.timestamp_ms == 1200


def test_output_within_accel_bounds() -> None:
    obs = _obs(goal_disp=(500.0, 500.0, 100.0))
    result = compute_reference_action(obs, "drone-001", 0)
    for component in result.proposed_accel_mps2:
        assert abs(component) <= MAX_ACCEL_COMPONENT_MPS2 + 1e-9


# ---------------------------------------------------------------------------
# Directional behaviour
# ---------------------------------------------------------------------------

def test_accelerates_toward_goal() -> None:
    # Goal is directly ahead in x; acceleration should be positive x.
    obs = _obs(vel=(0.0, 0.0, 0.0), goal_disp=(20.0, 0.0, 0.0))
    result = compute_reference_action(obs, "drone-001", 0)
    assert result.proposed_accel_mps2[0] > 0.0
    assert abs(result.proposed_accel_mps2[1]) < 1e-9
    assert abs(result.proposed_accel_mps2[2]) < 1e-9


def test_accelerates_in_3d_direction() -> None:
    obs = _obs(goal_disp=(6.0, 6.0, 0.0))
    result = compute_reference_action(obs, "drone-001", 0)
    ax, ay, az = result.proposed_accel_mps2
    assert ax > 0.0
    assert ay > 0.0
    assert abs(az) < 1e-9
    # Should be symmetric in x and y.
    assert abs(ax - ay) < 1e-9


def test_zero_accel_at_goal() -> None:
    obs = _obs(goal_disp=(0.0, 0.0, 0.0))
    result = compute_reference_action(obs, "drone-001", 0)
    assert result.proposed_accel_mps2 == (0.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# Determinism
# ---------------------------------------------------------------------------

def test_deterministic_same_obs_same_result() -> None:
    obs = _obs(goal_disp=(15.0, -5.0, 3.0))
    r1 = compute_reference_action(obs, "drone-001", 100)
    r2 = compute_reference_action(obs, "drone-001", 100)
    assert r1.proposed_accel_mps2 == r2.proposed_accel_mps2


# ---------------------------------------------------------------------------
# Clipping
# ---------------------------------------------------------------------------

def test_large_displacement_clips_to_bound() -> None:
    # A very large goal displacement would produce accel >> 3.5 without clipping.
    obs = _obs(goal_disp=(999.0, 999.0, 100.0))
    result = compute_reference_action(obs, "drone-001", 0)
    for component in result.proposed_accel_mps2:
        assert abs(component) <= MAX_ACCEL_COMPONENT_MPS2 + 1e-9


def test_clipped_action_points_toward_goal() -> None:
    # After clipping, the direction should still point toward the goal.
    obs = _obs(goal_disp=(100.0, 0.0, 0.0))
    result = compute_reference_action(obs, "drone-001", 0)
    ax, ay, az = result.proposed_accel_mps2
    assert ax > 0.0
    assert abs(ay) < 1e-9
    assert abs(az) < 1e-9


# ---------------------------------------------------------------------------
# Braking near goal
# ---------------------------------------------------------------------------

def test_braking_reduces_accel_near_goal() -> None:
    # Same goal displacement but with high velocity toward goal —
    # braking should reduce net forward acceleration vs zero velocity.
    obs_no_vel = _obs(vel=(0.0, 0.0, 0.0), goal_disp=(5.0, 0.0, 0.0))
    obs_with_vel = _obs(vel=(10.0, 0.0, 0.0), goal_disp=(5.0, 0.0, 0.0))
    r_no_vel = compute_reference_action(obs_no_vel, "drone-001", 0)
    r_with_vel = compute_reference_action(obs_with_vel, "drone-001", 0)
    # With velocity toward goal, braking should reduce the x acceleration.
    assert r_with_vel.proposed_accel_mps2[0] < r_no_vel.proposed_accel_mps2[0]
