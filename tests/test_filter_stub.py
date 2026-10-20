"""Tests for the HOCBF QP filter stub (safety/filter.py).

These tests verify the contract interface — input validation, barrier
computation, fallback semantics, and SafeAction structure — not the
correctness of the QP solve (which is a stub in P0).
"""

import pytest

from shared.contracts import (
    SAFE_SEPARATION_M,
    DroneState,
    MARLAction,
    SafeAction,
)
from utm_services.safety.filter import FilterConfig, FilterDiagnostics, apply_filter


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _state(
    agent_id: str,
    pos=(100.0, 100.0, 80.0),
    vel=(0.0, 0.0, 0.0),
    goal=(900.0, 900.0, 80.0),
    ts=0,
) -> DroneState:
    return DroneState(
        agent_id=agent_id,
        position_m=pos,
        velocity_mps=vel,
        goal_m=goal,
        timestamp_ms=ts,
    )


def _action(agent_id: str, accel=(0.5, 0.0, 0.0), ts=0) -> MARLAction:
    return MARLAction(
        agent_id=agent_id,
        proposed_accel_mps2=accel,
        timestamp_ms=ts,
    )


# ---------------------------------------------------------------------------
# Basic output contract
# ---------------------------------------------------------------------------

def test_single_agent_returns_one_safe_action() -> None:
    states = [_state("d1")]
    actions = [_action("d1")]
    result, diag = apply_filter(states, actions)
    assert len(result) == 1
    assert isinstance(result[0], SafeAction)
    assert result[0].agent_id == "d1"


def test_two_agents_return_two_safe_actions() -> None:
    states = [_state("d1", pos=(100.0, 100.0, 80.0)), _state("d2", pos=(200.0, 100.0, 80.0))]
    actions = [_action("d1"), _action("d2")]
    result, diag = apply_filter(states, actions)
    assert len(result) == 2
    agent_ids = {r.agent_id for r in result}
    assert agent_ids == {"d1", "d2"}


def test_safe_action_timestamp_matches_input() -> None:
    states = [_state("d1", ts=500)]
    actions = [_action("d1", ts=500)]
    result, _ = apply_filter(states, actions)
    assert result[0].timestamp_ms == 500


def test_stub_passes_nominal_action_through() -> None:
    # Stub does not modify actions.
    states = [_state("d1")]
    actions = [_action("d1", accel=(1.0, -0.5, 0.2))]
    result, _ = apply_filter(states, actions)
    assert result[0].safe_accel_mps2 == (1.0, -0.5, 0.2)


def test_stub_qp_feasible_true() -> None:
    states = [_state("d1")]
    actions = [_action("d1")]
    result, diag = apply_filter(states, actions)
    assert result[0].qp_feasible is True
    assert result[0].fallback_used is False


# ---------------------------------------------------------------------------
# Barrier computation
# ---------------------------------------------------------------------------

def test_no_barrier_for_single_agent() -> None:
    states = [_state("d1")]
    actions = [_action("d1")]
    _, diag = apply_filter(states, actions)
    assert diag.h_min_m2 is None
    assert diag.n_pairs_total == 0


def test_barrier_value_correct_for_pair() -> None:
    # Two drones 20 m apart in x. h = 20^2 - 5^2 = 400 - 25 = 375 m^2.
    states = [
        _state("d1", pos=(100.0, 100.0, 80.0)),
        _state("d2", pos=(120.0, 100.0, 80.0)),
    ]
    actions = [_action("d1"), _action("d2")]
    _, diag = apply_filter(states, actions)
    assert diag.h_min_m2 is not None
    assert abs(diag.h_min_m2 - 375.0) < 1e-6


def test_barrier_negative_when_inside_dsafe() -> None:
    # Two drones 3 m apart — inside d_safe=5m. h = 9 - 25 = -16 m^2.
    states = [
        _state("d1", pos=(100.0, 100.0, 80.0)),
        _state("d2", pos=(103.0, 100.0, 80.0)),
    ]
    actions = [_action("d1"), _action("d2")]
    _, diag = apply_filter(states, actions)
    assert diag.h_min_m2 is not None
    assert diag.h_min_m2 < 0.0


def test_h_min_reported_in_safe_action() -> None:
    states = [
        _state("d1", pos=(100.0, 100.0, 80.0)),
        _state("d2", pos=(120.0, 100.0, 80.0)),
    ]
    actions = [_action("d1"), _action("d2")]
    result, diag = apply_filter(states, actions)
    for sa in result:
        assert sa.h_min_m2 == diag.h_min_m2


def test_h_min_none_in_safe_action_single_agent() -> None:
    states = [_state("d1")]
    actions = [_action("d1")]
    result, _ = apply_filter(states, actions)
    assert result[0].h_min_m2 is None


# ---------------------------------------------------------------------------
# Input validation
# ---------------------------------------------------------------------------

def test_raises_on_duplicate_state_agent_ids() -> None:
    states = [_state("d1"), _state("d1")]
    actions = [_action("d1"), _action("d2")]
    with pytest.raises(ValueError, match="Duplicate"):
        apply_filter(states, actions)


def test_raises_on_duplicate_action_agent_ids() -> None:
    states = [_state("d1"), _state("d2")]
    actions = [_action("d1"), _action("d1")]
    with pytest.raises(ValueError, match="Duplicate"):
        apply_filter(states, actions)


def test_raises_on_mismatched_agent_ids() -> None:
    states = [_state("d1")]
    actions = [_action("d2")]
    with pytest.raises(ValueError, match="do not match"):
        apply_filter(states, actions)


def test_raises_on_mixed_action_timestamps() -> None:
    states = [_state("d1", ts=0), _state("d2", ts=0)]
    actions = [_action("d1", ts=0), _action("d2", ts=100)]
    with pytest.raises(ValueError, match="timestamp"):
        apply_filter(states, actions)


def test_raises_on_state_action_timestamp_mismatch() -> None:
    states = [_state("d1", ts=100)]
    actions = [_action("d1", ts=200)]
    with pytest.raises(ValueError, match="timestamp"):
        apply_filter(states, actions)


# ---------------------------------------------------------------------------
# Shield intervention detection
# ---------------------------------------------------------------------------

def test_shield_inactive_when_pass_through() -> None:
    # Stub passes through, so shield should never be active.
    states = [_state("d1"), _state("d2", pos=(200.0, 100.0, 80.0))]
    actions = [_action("d1", accel=(1.0, 0.0, 0.0)), _action("d2", accel=(-1.0, 0.0, 0.0))]
    result, _ = apply_filter(states, actions)
    for sa in result:
        assert sa.shield_active is False


# ---------------------------------------------------------------------------
# FilterConfig validation
# ---------------------------------------------------------------------------

def test_filter_config_rejects_zero_alpha() -> None:
    with pytest.raises(ValueError):
        FilterConfig(alpha1=0.0)


def test_filter_config_rejects_negative_range_gate() -> None:
    with pytest.raises(ValueError):
        FilterConfig(range_gate_m=-10.0)


def test_filter_config_accepts_valid_values() -> None:
    cfg = FilterConfig(alpha1=0.3, alpha2=0.8, range_gate_m=200.0)
    assert cfg.alpha1 == 0.3


# ---------------------------------------------------------------------------
# Range gate
# ---------------------------------------------------------------------------

def test_range_gate_excludes_distant_pair() -> None:
    # Two drones 500 m apart; range gate at 100 m.
    states = [
        _state("d1", pos=(0.0, 100.0, 80.0)),
        _state("d2", pos=(500.0, 100.0, 80.0)),
    ]
    actions = [_action("d1"), _action("d2")]
    cfg = FilterConfig(range_gate_m=100.0)
    _, diag = apply_filter(states, actions, config=cfg)
    assert diag.n_pairs_total == 1
    assert diag.n_pairs_in_qp == 0


def test_range_gate_includes_nearby_pair() -> None:
    states = [
        _state("d1", pos=(100.0, 100.0, 80.0)),
        _state("d2", pos=(150.0, 100.0, 80.0)),
    ]
    actions = [_action("d1"), _action("d2")]
    cfg = FilterConfig(range_gate_m=200.0)
    _, diag = apply_filter(states, actions, config=cfg)
    assert diag.n_pairs_in_qp == 1


def test_no_active_agents_returns_empty() -> None:
    safe_actions, diag = apply_filter([], [])
    assert safe_actions == []
    assert diag == FilterDiagnostics()
