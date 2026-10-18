# Interface Contracts
> This file is the single source of truth for what each branch exposes to the others.
> **Do not change an interface without opening a PR and getting sign-off from all affected owners.**
> Breaking changes require a version bump (v1 → v2).

---

## § STATE-SCHEMA · Drone State Vector

All modules use this shared state representation. Agreed on Day 1.

```python
from dataclasses import dataclass
import numpy as np

@dataclass
class DroneState:
    agent_id:     str           # unique string, e.g. "drone_003"
    position:     np.ndarray    # shape (3,), meters, [x, y, z], z = altitude
    velocity:     np.ndarray    # shape (3,), m/s
    goal:         np.ndarray    # shape (3,), meters — current mission waypoint
    timestamp_ms: int           # simulation time in milliseconds

# Safe serialization to dict (for API / logging)
def state_to_dict(s: DroneState) -> dict:
    return {
        "agent_id":     s.agent_id,
        "position":     s.position.tolist(),
        "velocity":     s.velocity.tolist(),
        "goal":         s.goal.tolist(),
        "timestamp_ms": s.timestamp_ms,
    }
```

**Owner**: Jointly owned — any change requires all three owners to approve.
**Used by**: `A-01` (env), `S-01` (CBF), `M-05` (API telemetry endpoint).

---

## § MARL-ACTION · Action Output from Policy to Shield

Defined in `anas/env/action_schema.py` (ticket `A-03`).

```python
@dataclass
class MARLAction:
    agent_id:        str           # must match DroneState.agent_id
    proposed_accel:  np.ndarray    # shape (3,), m/s² — unconstrained MARL output
    timestamp_ms:    int           # must match the state timestamp it was computed from

# Batch type used when calling the CBF solver
MARLActionBatch = list[MARLAction]
```

**Producer**: `anas/marl-policy` branch.
**Consumer**: `salman/cbf-safety` branch (`qp_solver.py`).
**Contract**: The CBF solver MUST NOT modify `agent_id` or `timestamp_ms`. It only replaces `proposed_accel` with `safe_accel`.

---

## § CBF-OUTPUT · Safe Action Output from Shield

Defined in `salman/cbf/qp_solver.py` (ticket `S-04`).

```python
@dataclass
class SafeAction:
    agent_id:         str
    safe_accel:       np.ndarray    # shape (3,), m/s² — CBF-filtered action
    timestamp_ms:     int
    shield_active:    bool          # True if CBF modified the proposed action
    h_min:            float         # minimum barrier value across all pairwise constraints
    qp_feasible:      bool          # False triggers fallback controller
    fallback_used:    bool          # True if HJ fallback was triggered (S-08)
```

**Producer**: `salman/cbf-safety` branch.
**Consumer**: `anas/marl-policy` (sim loop), `aman/infra-api` (logging endpoint).
**Contract**: If `qp_feasible == False`, `safe_accel` is set to `[0, 0, 0]` (hover). The simulation layer must handle this.

---

## § ENV-OUTPUT · Episode Step Return

Defined in `anas/env/drone_env.py` (ticket `A-01`). Follows PettingZoo `ParallelEnv` API.

```python
# env.step(actions: dict[agent_id, np.ndarray]) returns:
observations:  dict[str, np.ndarray]   # agent_id → obs vector, shape (obs_dim,)
rewards:       dict[str, float]        # agent_id → scalar reward
terminations:  dict[str, bool]         # True when agent reaches goal or crashes
truncations:   dict[str, bool]         # True when episode time limit hit
infos:         dict[str, dict]         # agent_id → {"shield_active": bool, "h_min": float}
```

**Observation vector layout** (`obs_dim = 18`):
```
[0:3]   own position (x, y, z)
[3:6]   own velocity (vx, vy, vz)
[6:9]   goal vector (gx, gy, gz)
[9:12]  nearest neighbor position (relative)
[12:15] nearest neighbor velocity (relative)
[15]    nearest neighbor distance (scalar)
[16]    shield_active last step (0 or 1)
[17]    h_min last step (clipped to [-1, 1])
```

**Owner**: Anas (primary), Salman reviews `[15:17]` shield fields.

---

## § API-AIRSPACE · Sovereign Core REST API

Defined in `aman/api/sovereign_core.py` (ticket `M-05`). Base URL: `http://localhost:8000`.

### `GET /airspace/rules`
Returns current airspace configuration. Called once per episode.

```json
{
  "version": "1.0",
  "timestamp_utc": "2026-10-18T15:00:00Z",
  "geofences": [
    {
      "id": "gf_001",
      "type": "exclusion",
      "shape": "cylinder",
      "center_xy": [500.0, 500.0],
      "radius_m": 50.0,
      "min_alt_m": 0.0,
      "max_alt_m": 200.0
    }
  ],
  "altitude_limits": {
    "min_m": 30.0,
    "max_m": 200.0
  },
  "separation_minimum_m": 5.0
}
```

### `GET /airspace/notam`
Returns active NOTAMs (temporary restrictions).

```json
{
  "notams": [
    {
      "id": "notam_042",
      "description": "VIP movement — altitude below 80m restricted",
      "bbox": [[200, 200], [400, 400]],
      "active_until_ms": 9999999
    }
  ]
}
```

### `POST /drone/register`
Registers a drone at episode start.

```json
// Request body
{ "agent_id": "drone_003", "operator": "sim", "mission_type": "delivery" }

// Response
{ "registered": true, "transponder_code": "A1B2" }
```

**Owner**: Aman (primary). All endpoints are versioned. Salman's `rulebook.py` reads the JSON from this API.

---

## § SHIELD-LOG · Shield Intervention Log Schema

Defined in `salman/cbf/shield_logger.py` (ticket `S-06`). Written as CSV for analysis.

```
timestep, agent_id, proposed_accel_x, proposed_accel_y, proposed_accel_z,
safe_accel_x, safe_accel_y, safe_accel_z, h_value, constraint_active,
qp_feasible, fallback_used
```

**Consumer**: `aman/analysis/plot_results.py` (ticket `M-08`). The CSV column names are fixed — do not rename without updating `M-08`.

---

## § CHAOS-API · Chaos Injector Interface

Defined in `anas/chaos/chaos_injector.py` (ticket `A-07`).

```python
class ChaosInjector:
    def apply_comm_delay(self, state: DroneState, delay_ms: int) -> DroneState:
        """Returns a stale copy of state with position/velocity extrapolated back by delay_ms."""
        ...

    def apply_byzantine(self, state: DroneState) -> DroneState:
        """Returns state with randomized position/velocity (adversarial telemetry)."""
        ...

    def apply_wind_gust(self, accel: np.ndarray, magnitude: float) -> np.ndarray:
        """Adds bounded random noise to an acceleration command."""
        ...
```

**Owner**: Anas. Salman reviews that `delay_ms` values used in `S-07` analysis match what's actually injected here.

---

## Interface Change Protocol

1. Open a PR on the affected branch with the proposed change.
2. Tag all three owners as reviewers.
3. Do not merge until all affected owners approve.
4. Bump the version comment at the top of this file.
5. Update all `Inputs needed` references in `TASKS.md`.

**Current version**: v1.0 — Oct 18, 2026
