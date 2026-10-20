import pytest
from pydantic import ValidationError

from shared.contracts import (
    DroneState,
    MARLAction,
    ObservationV1,
    SafeAction,
    validate_action_pair,
)


def _safe_action(**overrides: object) -> dict[str, object]:
    payload: dict[str, object] = {
        "agent_id": "drone-001",
        "safe_accel_mps2": [0.0, 0.0, 0.0],
        "timestamp_ms": 1200,
        "shield_active": False,
        "h_min_m2": 12.5,
        "qp_feasible": True,
        "fallback_used": False,
    }
    payload.update(overrides)
    return payload


def test_observation_rejects_19_values() -> None:
    with pytest.raises(ValidationError):
        ObservationV1.model_validate([0.0] * 19)


def test_marl_action_rejects_acceleration_component_above_3_5_mps2() -> None:
    with pytest.raises(ValidationError):
        MARLAction.model_validate(
            {
                "agent_id": "drone-001",
                "proposed_accel_mps2": [3.51, 0.0, 0.0],
                "timestamp_ms": 1200,
            }
        )


def test_infeasible_qp_requires_fallback_flag() -> None:
    with pytest.raises(ValidationError):
        SafeAction.model_validate(
            _safe_action(qp_feasible=False, fallback_used=False)
        )


def test_fallback_requires_zero_acceleration() -> None:
    with pytest.raises(ValidationError):
        SafeAction.model_validate(
            _safe_action(
                safe_accel_mps2=[0.1, 0.0, 0.0],
                qp_feasible=False,
                fallback_used=True,
            )
        )


def test_valid_infeasible_qp_fallback_is_accepted() -> None:
    action = SafeAction.model_validate(
        _safe_action(qp_feasible=False, fallback_used=True)
    )
    assert action.safe_accel_mps2 == (0.0, 0.0, 0.0)


def test_drone_state_rejects_timestamp_not_multiple_of_100_ms() -> None:
    with pytest.raises(ValidationError):
        DroneState.model_validate(
            {
                "agent_id": "drone-001",
                "position_m": [100.0, 200.0, 80.0],
                "velocity_mps": [2.0, 0.0, 0.0],
                "goal_m": [300.0, 200.0, 80.0],
                "timestamp_ms": 1250,
            }
        )


def test_shield_flag_uses_shared_intervention_tolerance() -> None:
    proposed = MARLAction.model_validate(
        {
            "agent_id": "drone-001",
            "proposed_accel_mps2": [0.2, 0.0, 0.0],
            "timestamp_ms": 1200,
        }
    )
    applied = SafeAction.model_validate(
        _safe_action(
            safe_accel_mps2=[0.2 + 1.1e-4, 0.0, 0.0],
            shield_active=True,
        )
    )
    validate_action_pair(proposed, applied)



def test_marl_action_accepts_acceleration_component_at_3_5_mps2() -> None:
    action = MARLAction.model_validate(
        {
            "agent_id": "drone-001",
            "proposed_accel_mps2": [3.5, -3.5, 0.0],
            "timestamp_ms": 1200,
        }
    )
    assert action.proposed_accel_mps2 == (3.5, -3.5, 0.0)


def test_acceleration_error_names_current_limit() -> None:
    with pytest.raises(ValidationError, match=r"\[-3\.5, 3\.5\]"):
        MARLAction.model_validate(
            {
                "agent_id": "drone-001",
                "proposed_accel_mps2": [3.6, 0.0, 0.0],
                "timestamp_ms": 1200,
            }
        )
