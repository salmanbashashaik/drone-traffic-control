"""Tests for the local synthetic mock API and the scenario adapter."""

import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError

from shared.contracts import ScenarioConfig
from utm_services.api.app import ScenarioDataUnavailable, create_app
from utm_services.application.scenario_adapter import reset_from_payload

_REGISTRATION = {"agent_id": "drone-001", "operator": "sim-op", "mission_type": "delivery"}


def test_rules_and_notams_return_synthetic_data() -> None:
    client = TestClient(create_app())
    assert client.get("/airspace/rules").json()["separation_minimum_m"] == 5.0
    assert client.get("/airspace/notam").json() == {"notams": []}


def test_register_then_duplicate_returns_409() -> None:
    client = TestClient(create_app())
    first = client.post("/drone/register", json=_REGISTRATION)
    assert first.status_code == 201
    assert first.json()["sim_registration_id"].startswith("sim-reg-")
    second = client.post("/drone/register", json=_REGISTRATION)
    assert second.status_code == 409
    assert second.json()["code"] == "duplicate_agent_id"


def test_invalid_body_returns_422_envelope() -> None:
    response = TestClient(create_app()).post("/drone/register", json={"agent_id": "a"})
    assert response.status_code == 422
    assert response.json()["code"] == "invalid_request"


def test_unknown_route_returns_404_envelope() -> None:
    response = TestClient(create_app()).get("/nope")
    assert response.status_code == 404
    assert response.json()["code"] == "not_found"


def test_unavailable_scenario_data_returns_503() -> None:
    def unavailable():
        raise ScenarioDataUnavailable()

    response = TestClient(create_app(rules_provider=unavailable)).get("/airspace/rules")
    assert response.status_code == 503
    assert response.json()["code"] == "scenario_unavailable"


def test_unexpected_error_returns_500_envelope() -> None:
    def broken():
        raise RuntimeError("boom")

    client = TestClient(create_app(notams_provider=broken), raise_server_exceptions=False)
    response = client.get("/airspace/notam")
    assert response.status_code == 500
    assert response.json()["code"] == "mock_api_error"


class _RecordingSimulator:
    def __init__(self) -> None:
        self.config: ScenarioConfig | None = None

    def reset(self, config: ScenarioConfig) -> str:
        self.config = config
        return "reset-ok"


def test_reset_from_payload_passes_validated_config() -> None:
    simulator = _RecordingSimulator()
    payload = {
        "scenario_id": "s1",
        "seed": 7,
        "drones": [
            {
                "agent_id": "drone-001",
                "initial_position_m": [10.0, 10.0, 50.0],
                "initial_velocity_mps": [0.0, 0.0, 0.0],
                "goal_m": [100.0, 100.0, 50.0],
            }
        ],
    }
    assert reset_from_payload(simulator, payload) == "reset-ok"
    assert isinstance(simulator.config, ScenarioConfig)
    assert simulator.config.drones[0].initial_position_m == (10.0, 10.0, 50.0)


def test_reset_from_payload_rejects_invalid_config() -> None:
    simulator = _RecordingSimulator()
    with pytest.raises(ValidationError):
        reset_from_payload(simulator, {"scenario_id": "s1", "seed": 7, "drones": []})
    assert simulator.config is None
