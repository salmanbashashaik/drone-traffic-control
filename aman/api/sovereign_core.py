"""
aman/api/sovereign_core.py
Ticket: M-05 — Sovereign Core mock API

Holds airspace rules that the simulation fetches at episode start.
Run: uvicorn aman.api.sovereign_core:app --reload --port 8000

See INTERFACES.md § API-AIRSPACE for the full contract.
"""
from __future__ import annotations
from datetime import datetime, timezone
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import json, os

app = FastAPI(title="Sovereign Core — Airspace Authority API", version="1.0")

# ---------------------------------------------------------------------------
# In-memory airspace state (replace with DB later)
# ---------------------------------------------------------------------------

_AIRSPACE_RULES: dict = {
    "version": "1.0",
    "geofences": [
        {
            "id": "gf_001",
            "type": "exclusion",
            "shape": "cylinder",
            "center_xy": [500.0, 500.0],
            "radius_m": 50.0,
            "min_alt_m": 0.0,
            "max_alt_m": 200.0,
            "description": "Central hub no-fly zone",
        }
    ],
    "altitude_limits": {
        "min_m": 30.0,
        "max_m": 200.0,
    },
    "separation_minimum_m": 5.0,
    "max_speed_ms": 15.0,
}

_NOTAMS: list[dict] = [
    {
        "id": "notam_001",
        "description": "Test NOTAM — low altitude restriction in SW quadrant",
        "bbox": [[0, 0], [250, 250]],
        "min_alt_m": 80.0,
        "active_until_ms": 9_999_999_999,
    }
]

_REGISTERED_DRONES: dict[str, dict] = {}

# ---------------------------------------------------------------------------
# Request / Response models
# ---------------------------------------------------------------------------

class DroneRegistration(BaseModel):
    agent_id:     str
    operator:     str
    mission_type: str  # e.g., "delivery", "survey", "inspection"


class RegistrationResponse(BaseModel):
    registered:      bool
    transponder_code: str


# ---------------------------------------------------------------------------
# Endpoints
# ---------------------------------------------------------------------------

@app.get("/health")
def health():
    return {"status": "ok", "timestamp_utc": datetime.now(timezone.utc).isoformat()}


@app.get("/airspace/rules")
def get_airspace_rules():
    """Returns current airspace configuration. Called once per simulation episode."""
    return {
        **_AIRSPACE_RULES,
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
    }


@app.get("/airspace/notam")
def get_notams():
    """Returns active NOTAMs."""
    return {"notams": _NOTAMS}


@app.post("/drone/register", response_model=RegistrationResponse)
def register_drone(reg: DroneRegistration):
    """Registers a drone at episode start. Returns transponder code."""
    if reg.agent_id in _REGISTERED_DRONES:
        # Re-registration is fine — just update
        pass
    code = f"{reg.agent_id[-3:].upper()}{len(_REGISTERED_DRONES):04d}"
    _REGISTERED_DRONES[reg.agent_id] = {
        "operator":       reg.operator,
        "mission_type":   reg.mission_type,
        "transponder":    code,
        "registered_at":  datetime.now(timezone.utc).isoformat(),
    }
    return RegistrationResponse(registered=True, transponder_code=code)


@app.get("/drone/{agent_id}")
def get_drone(agent_id: str):
    """Get registration info for a drone."""
    if agent_id not in _REGISTERED_DRONES:
        raise HTTPException(status_code=404, detail=f"{agent_id} not registered")
    return _REGISTERED_DRONES[agent_id]


@app.get("/drone/")
def list_drones():
    """List all registered drones."""
    return {"drones": list(_REGISTERED_DRONES.keys()), "count": len(_REGISTERED_DRONES)}


# Admin endpoint to update geofences (for stress testing)
@app.post("/admin/geofence")
def add_geofence(gf: dict):
    """Add a geofence at runtime (stress testing use only)."""
    _AIRSPACE_RULES["geofences"].append(gf)
    return {"added": True, "total_geofences": len(_AIRSPACE_RULES["geofences"])}


# ---------------------------------------------------------------------------
# Run directly: python aman/api/sovereign_core.py
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import uvicorn
    uvicorn.run("aman.api.sovereign_core:app", host="0.0.0.0", port=8000, reload=True)
