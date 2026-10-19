"""Canonical simulation-only v1 Pydantic contracts.

This file is the single source of truth for all Pydantic models in the
repository. Import these models from every workstream; never duplicate or
redefine them on a branch. Update this file and INTERFACES.md together through
a reviewed change to main.
"""

from __future__ import annotations

from math import hypot
from typing import Annotated, Any, Literal

from pydantic import (
    BaseModel,
    BeforeValidator,
    ConfigDict,
    Field,
    RootModel,
    field_validator,
    model_validator,
)


WORLD_XY_MIN_M = 0.0
WORLD_XY_MAX_M = 1000.0
WORLD_Z_MIN_M = 30.0
WORLD_Z_MAX_M = 200.0
MAX_SPEED_MPS = 15.0
MAX_ACCEL_COMPONENT_MPS2 = 3.0
MAX_EPISODE_STEPS = 1000
SIMULATION_STEP_MS = 100
SAFE_SEPARATION_M = 5.0
SHIELD_INTERVENTION_TOLERANCE_MPS2 = 1e-4


def _as_tuple(value: Any) -> Any:
    """Allow JSON/Python arrays while retaining immutable validated tuples."""
    if isinstance(value, list):
        return tuple(value)
    return value


Vec3 = Annotated[tuple[float, float, float], BeforeValidator(_as_tuple)]
Vec2 = Annotated[tuple[float, float], BeforeValidator(_as_tuple)]
BoundingBox3D = Annotated[tuple[float, float, float, float, float, float], BeforeValidator(_as_tuple)]
ObservationValues = Annotated[
    tuple[float, ...],
    BeforeValidator(_as_tuple),
    Field(min_length=18, max_length=18),
]
TimestampMs = Annotated[int, Field(ge=0, multiple_of=SIMULATION_STEP_MS)]


class ContractModel(BaseModel):
    """Shared strict behavior for structured v1 contract objects."""

    model_config = ConfigDict(
        extra="forbid",
        strict=True,
        frozen=True,
        allow_inf_nan=False,
    )


def _validate_agent_id(value: str) -> str:
    if not value.strip():
        raise ValueError("agent_id must not be blank")
    return value


def _validate_position(position: Vec3) -> Vec3:
    x, y, z = position
    if not WORLD_XY_MIN_M <= x <= WORLD_XY_MAX_M:
        raise ValueError("x must be within [0, 1000] m")
    if not WORLD_XY_MIN_M <= y <= WORLD_XY_MAX_M:
        raise ValueError("y must be within [0, 1000] m")
    if not WORLD_Z_MIN_M <= z <= WORLD_Z_MAX_M:
        raise ValueError("z must be within [30, 200] m")
    return position


def _validate_velocity(velocity: Vec3) -> Vec3:
    if hypot(*velocity) > MAX_SPEED_MPS:
        raise ValueError("3D speed magnitude must not exceed 15 m/s")
    return velocity


def _validate_acceleration(acceleration: Vec3) -> Vec3:
    if any(abs(component) > MAX_ACCEL_COMPONENT_MPS2 for component in acceleration):
        raise ValueError("each acceleration component must be within [-3, 3] m/s²")
    return acceleration


class DroneState(ContractModel):
    """Simulator/shield ground-truth state for one active agent."""

    agent_id: str = Field(min_length=1)
    position_m: Vec3
    velocity_mps: Vec3
    goal_m: Vec3
    timestamp_ms: TimestampMs

    @field_validator("agent_id")
    @classmethod
    def _agent_id_is_nonblank(cls, value: str) -> str:
        return _validate_agent_id(value)

    @field_validator("position_m", "goal_m")
    @classmethod
    def _positions_are_in_world(cls, value: Vec3) -> Vec3:
        return _validate_position(value)

    @field_validator("velocity_mps")
    @classmethod
    def _speed_is_bounded(cls, value: Vec3) -> Vec3:
        return _validate_velocity(value)


class ObservationV1(RootModel[ObservationValues]):
    """The fixed 18-float policy observation, serialized as a JSON array."""

    model_config = ConfigDict(strict=True, frozen=True, allow_inf_nan=False)

    @model_validator(mode="after")
    def _validate_v1_semantics(self) -> ObservationV1:
        values = self.root
        _validate_position((values[0], values[1], values[2]))
        _validate_velocity((values[3], values[4], values[5]))

        if values[15] < 0.0 and values[15] != -1.0:
            raise ValueError("neighbor distance must be non-negative or -1 sentinel")
        if values[15] == -1.0 and any(values[index] != 0.0 for index in range(9, 15)):
            raise ValueError("no-neighbor sentinel requires zero relative position/velocity")
        if values[16] not in (0.0, 1.0):
            raise ValueError("shield indicator at index 16 must be 0.0 or 1.0")
        if not -1.0 <= values[17] <= 1.0:
            raise ValueError("normalized barrier value at index 17 must be in [-1, 1]")
        return self


class MARLAction(ContractModel):
    """Nominal bounded acceleration proposed by a policy."""

    agent_id: str = Field(min_length=1)
    proposed_accel_mps2: Vec3
    timestamp_ms: TimestampMs

    @field_validator("agent_id")
    @classmethod
    def _agent_id_is_nonblank(cls, value: str) -> str:
        return _validate_agent_id(value)

    @field_validator("proposed_accel_mps2")
    @classmethod
    def _acceleration_is_bounded(cls, value: Vec3) -> Vec3:
        return _validate_acceleration(value)


class SafeAction(ContractModel):
    """Action returned by the filter adapter; the name is not a safety claim."""

    agent_id: str = Field(min_length=1)
    safe_accel_mps2: Vec3
    timestamp_ms: TimestampMs
    shield_active: bool
    h_min_m2: float | None
    qp_feasible: bool
    fallback_used: bool

    @field_validator("agent_id")
    @classmethod
    def _agent_id_is_nonblank(cls, value: str) -> str:
        return _validate_agent_id(value)

    @field_validator("safe_accel_mps2")
    @classmethod
    def _acceleration_is_bounded(cls, value: Vec3) -> Vec3:
        return _validate_acceleration(value)

    @model_validator(mode="after")
    def _validate_fallback_contract(self) -> SafeAction:
        if not self.qp_feasible and not self.fallback_used:
            raise ValueError("an infeasible QP requires fallback_used=true")
        if self.qp_feasible and self.fallback_used:
            raise ValueError("fallback_used=true requires qp_feasible=false")
        if self.fallback_used and any(component != 0.0 for component in self.safe_accel_mps2):
            raise ValueError("the v1 fallback action must have zero acceleration")
        return self


class ScenarioDroneConfig(ContractModel):
    """Initial conditions for one drone in a local simulation scenario."""

    agent_id: str = Field(min_length=1)
    initial_position_m: Vec3
    initial_velocity_mps: Vec3
    goal_m: Vec3

    @field_validator("agent_id")
    @classmethod
    def _agent_id_is_nonblank(cls, value: str) -> str:
        return _validate_agent_id(value)

    @field_validator("initial_position_m", "goal_m")
    @classmethod
    def _positions_are_in_world(cls, value: Vec3) -> Vec3:
        return _validate_position(value)

    @field_validator("initial_velocity_mps")
    @classmethod
    def _speed_is_bounded(cls, value: Vec3) -> Vec3:
        return _validate_velocity(value)


class ScenarioConfig(ContractModel):
    """Seeded local reset configuration; this is not an HTTP API payload."""

    scenario_id: str = Field(min_length=1)
    seed: int = Field(ge=0)
    episode_limit_steps: int = Field(default=MAX_EPISODE_STEPS, ge=1, le=MAX_EPISODE_STEPS)
    drones: Annotated[tuple[ScenarioDroneConfig, ...], BeforeValidator(_as_tuple)] = Field(
        min_length=1
    )

    @field_validator("scenario_id")
    @classmethod
    def _scenario_id_is_nonblank(cls, value: str) -> str:
        if not value.strip():
            raise ValueError("scenario_id must not be blank")
        return value

    @model_validator(mode="after")
    def _agent_ids_are_unique(self) -> ScenarioConfig:
        agent_ids = [drone.agent_id for drone in self.drones]
        if len(agent_ids) != len(set(agent_ids)):
            raise ValueError("ScenarioConfig drone agent_id values must be unique")
        return self


class DroneRegisterRequest(ContractModel):
    """Synthetic registration request for POST /drone/register."""

    agent_id: str = Field(min_length=1)
    operator: str = Field(min_length=1)
    mission_type: str = Field(min_length=1)

    @field_validator("agent_id", "operator", "mission_type")
    @classmethod
    def _text_is_nonblank(cls, value: str) -> str:
        if not value.strip():
            raise ValueError("registration text fields must not be blank")
        return value


class DroneRegisterResponse(ContractModel):
    """Synthetic registration response; it does not register a real aircraft."""

    registered: Literal[True]
    sim_registration_id: str = Field(min_length=1)

    @field_validator("sim_registration_id")
    @classmethod
    def _registration_id_is_nonblank(cls, value: str) -> str:
        if not value.strip():
            raise ValueError("sim_registration_id must not be blank")
        return value


class GeofenceV1(ContractModel):
    """Closed-cylinder synthetic geofence used by the local mock API."""

    id: str = Field(min_length=1)
    center_xy_m: Vec2
    radius_m: float = Field(gt=0)
    floor_m: float
    ceiling_m: float
    active_from_ms: TimestampMs
    active_until_ms: TimestampMs

    @field_validator("id")
    @classmethod
    def _id_is_nonblank(cls, value: str) -> str:
        if not value.strip():
            raise ValueError("geofence id must not be blank")
        return value

    @model_validator(mode="after")
    def _bounds_are_ordered(self) -> GeofenceV1:
        if self.floor_m >= self.ceiling_m:
            raise ValueError("geofence floor_m must be below ceiling_m")
        if self.active_from_ms > self.active_until_ms:
            raise ValueError("geofence active_from_ms must not exceed active_until_ms")
        return self


class AltitudeLimitsV1(ContractModel):
    """Synthetic local-coordinate altitude limits."""

    min: float
    max: float

    @model_validator(mode="after")
    def _limits_are_ordered(self) -> AltitudeLimitsV1:
        if self.min >= self.max:
            raise ValueError("altitude min must be below max")
        return self


class AirspaceRulesV1(ContractModel):
    """Versioned synthetic rules response for GET /airspace/rules."""

    version: str = Field(min_length=1)
    geofences: Annotated[tuple[GeofenceV1, ...], BeforeValidator(_as_tuple)]
    altitude_limits_m: AltitudeLimitsV1
    separation_minimum_m: float = Field(gt=0)

    @field_validator("version")
    @classmethod
    def _version_is_nonblank(cls, value: str) -> str:
        if not value.strip():
            raise ValueError("rules version must not be blank")
        return value


class NotamV1(ContractModel):
    """Synthetic local-coordinate notice; not an official NOTAM."""

    id: str = Field(min_length=1)
    description: str = Field(min_length=1)
    bbox_m: BoundingBox3D
    active_from_ms: TimestampMs
    active_until_ms: TimestampMs

    @field_validator("id", "description")
    @classmethod
    def _text_is_nonblank(cls, value: str) -> str:
        if not value.strip():
            raise ValueError("notice text fields must not be blank")
        return value

    @model_validator(mode="after")
    def _bounds_are_ordered(self) -> NotamV1:
        xmin, ymin, zmin, xmax, ymax, zmax = self.bbox_m
        if xmin > xmax or ymin > ymax or zmin > zmax:
            raise ValueError("bbox_m minimum coordinates must not exceed maximum coordinates")
        if self.active_from_ms > self.active_until_ms:
            raise ValueError("notice active_from_ms must not exceed active_until_ms")
        return self


class NotamListV1(ContractModel):
    """Synthetic notices response for GET /airspace/notam."""

    notams: Annotated[tuple[NotamV1, ...], BeforeValidator(_as_tuple)]


class APIError(ContractModel):
    """Stable error envelope for local mock API failures."""

    code: Literal[
        "invalid_request",
        "not_found",
        "duplicate_agent_id",
        "scenario_unavailable",
        "mock_api_error",
    ]
    message: str = Field(min_length=1)

    @field_validator("code", "message")
    @classmethod
    def _error_text_is_nonblank(cls, value: str) -> str:
        if not value.strip():
            raise ValueError("error code and message must not be blank")
        return value


def validate_action_pair(proposed: MARLAction, applied: SafeAction) -> None:
    """Check identity/time alignment and the shared shield-intervention flag."""
    if proposed.agent_id != applied.agent_id:
        raise ValueError("proposed and applied actions must have the same agent_id")
    if proposed.timestamp_ms != applied.timestamp_ms:
        raise ValueError("proposed and applied actions must have the same timestamp_ms")

    changed_beyond_tolerance = any(
        abs(applied_component - proposed_component) > SHIELD_INTERVENTION_TOLERANCE_MPS2
        for proposed_component, applied_component in zip(
            proposed.proposed_accel_mps2, applied.safe_accel_mps2, strict=True
        )
    )
    if applied.shield_active != changed_beyond_tolerance:
        raise ValueError("shield_active does not match the v1 intervention tolerance")

