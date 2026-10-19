"""Local-only simulation API.

The module-level ``app`` is the v1 development entry point. ``create_app``
allows data providers and registration storage to be supplied explicitly so
the HTTP layer does not own scenario, persistence, or controller logic.
"""

from __future__ import annotations

import logging
from hashlib import sha256
from threading import Lock
from typing import Callable, Protocol

from fastapi import FastAPI, HTTPException
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from starlette.exceptions import HTTPException as StarletteHTTPException

from shared.contracts import (
    APIError,
    AirspaceRulesV1,
    DroneRegisterRequest,
    DroneRegisterResponse,
    NotamListV1,
)


logger = logging.getLogger(__name__)


class RegistrationConflict(Exception):
    """Raised when this simulation process already registered an agent ID."""


class ScenarioDataUnavailable(Exception):
    """Raised when a configured scenario has no synthetic API data available."""


class RegistrationStore(Protocol):
    """Storage boundary so HTTP handlers do not own registration semantics."""

    def register(self, agent_id: str) -> str:
        """Register a simulation-only agent and return its synthetic ID."""


class InMemoryRegistrationStore:
    """Single-process v1 store; replace when a real service is selected."""

    def __init__(self) -> None:
        self._agent_ids: set[str] = set()
        self._lock = Lock()

    def register(self, agent_id: str) -> str:
        with self._lock:
            if agent_id in self._agent_ids:
                raise RegistrationConflict(agent_id)
            self._agent_ids.add(agent_id)

        digest = sha256(agent_id.encode("utf-8")).hexdigest()[:12]
        return f"sim-reg-{digest}"


def _default_rules() -> AirspaceRulesV1:
    return AirspaceRulesV1.model_validate(
        {
            "version": "sim-rules-v1",
            "geofences": [
                {
                    "id": "restricted-1",
                    "center_xy_m": [500.0, 500.0],
                    "radius_m": 50.0,
                    "floor_m": 30.0,
                    "ceiling_m": 200.0,
                    "active_from_ms": 0,
                    "active_until_ms": 100000,
                }
            ],
            "altitude_limits_m": {"min": 30.0, "max": 200.0},
            "separation_minimum_m": 5.0,
        }
    )


def _default_notams() -> NotamListV1:
    return NotamListV1(notams=())


def create_app(
    *,
    rules_provider: Callable[[], AirspaceRulesV1] = _default_rules,
    notams_provider: Callable[[], NotamListV1] = _default_notams,
    registration_store: RegistrationStore | None = None,
) -> FastAPI:
    """Build the local API with explicit providers and isolated state."""
    store = (
        registration_store
        if registration_store is not None
        else InMemoryRegistrationStore()
    )
    app = FastAPI(
        title="UTM Simulation Mock API",
        version="v1",
        description=(
            "Synthetic local inputs for the simulation prototype; not a live UTM service."
        ),
    )

    @app.exception_handler(RequestValidationError)
    async def handle_invalid_request(
        _request: object, _exception: RequestValidationError
    ) -> JSONResponse:
        error = APIError(
            code="invalid_request",
            message="Request does not match the documented v1 API contract.",
        )
        return JSONResponse(status_code=422, content=error.model_dump(mode="json"))

    @app.exception_handler(StarletteHTTPException)
    async def handle_http_error(
        _request: object, exception: StarletteHTTPException
    ) -> JSONResponse:
        if exception.status_code == 404:
            code = "not_found"
            message = "The requested mock API route does not exist."
        elif exception.status_code == 409:
            code = "duplicate_agent_id"
            message = "This agent_id is already registered in this mock API process."
        else:
            code = "mock_api_error"
            message = "The mock API could not complete the request."
        error = APIError(code=code, message=message)
        return JSONResponse(
            status_code=exception.status_code,
            content=error.model_dump(mode="json"),
            headers=exception.headers,
        )

    @app.exception_handler(ScenarioDataUnavailable)
    async def handle_unavailable_scenario_data(
        _request: object, _exception: ScenarioDataUnavailable
    ) -> JSONResponse:
        error = APIError(
            code="scenario_unavailable",
            message="Synthetic data is unavailable for the requested scenario.",
        )
        return JSONResponse(status_code=503, content=error.model_dump(mode="json"))

    @app.exception_handler(Exception)
    async def handle_unexpected_error(
        _request: object, exception: Exception
    ) -> JSONResponse:
        logger.error(
            "Unhandled simulation mock API error",
            exc_info=(type(exception), exception, exception.__traceback__),
        )
        error = APIError(
            code="mock_api_error",
            message="The mock API could not complete the request.",
        )
        return JSONResponse(status_code=500, content=error.model_dump(mode="json"))

    @app.get(
        "/airspace/rules",
        response_model=AirspaceRulesV1,
        responses={
            404: {"model": APIError, "description": "Unknown route."},
            500: {"model": APIError, "description": "Mock API failure."},
            503: {"model": APIError, "description": "Scenario data unavailable."},
        },
    )
    def get_airspace_rules() -> AirspaceRulesV1:
        """Return the synthetic ruleset in local Cartesian coordinates."""
        return rules_provider()

    @app.get(
        "/airspace/notam",
        response_model=NotamListV1,
        responses={
            404: {"model": APIError, "description": "Unknown route."},
            500: {"model": APIError, "description": "Mock API failure."},
            503: {"model": APIError, "description": "Scenario data unavailable."},
        },
    )
    def get_synthetic_notams() -> NotamListV1:
        """Return the local demo's synthetic notice list."""
        return notams_provider()

    @app.post(
        "/drone/register",
        response_model=DroneRegisterResponse,
        status_code=201,
        responses={
            404: {"model": APIError, "description": "Unknown route."},
            409: {"model": APIError, "description": "Duplicate simulation agent ID."},
            422: {"model": APIError, "description": "Invalid request body."},
            500: {"model": APIError, "description": "Mock API failure."},
        },
    )
    def register_synthetic_drone(request: DroneRegisterRequest) -> DroneRegisterResponse:
        """Create an in-memory synthetic registration; never contact a registry."""
        try:
            registration_id = store.register(request.agent_id)
        except RegistrationConflict as exc:
            raise HTTPException(status_code=409) from exc
        return DroneRegisterResponse(
            registered=True,
            sim_registration_id=registration_id,
        )

    return app


app = create_app()


def run() -> None:
    """Serve only on loopback for local v1 development."""
    import uvicorn

    uvicorn.run(app, host="127.0.0.1", port=8000, workers=1)
