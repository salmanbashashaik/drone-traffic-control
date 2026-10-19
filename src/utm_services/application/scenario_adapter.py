"""Validated handoff from local run configuration to Anas's simulator."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, Protocol

from shared.contracts import ScenarioConfig


class ScenarioResetter(Protocol):
    """The simulator reset boundary agreed in INTERFACES.md."""

    def reset(self, config: ScenarioConfig) -> Any:
        """Reset the simulator from a validated v1 scenario configuration."""


def validate_scenario_config(payload: Mapping[str, Any]) -> ScenarioConfig:
    """Validate external scenario data before it reaches simulator code."""
    return ScenarioConfig.model_validate(payload)


def reset_from_payload(
    simulator: ScenarioResetter, payload: Mapping[str, Any]
) -> Any:
    """Validate the config and pass that exact canonical model to reset()."""
    return simulator.reset(validate_scenario_config(payload))
