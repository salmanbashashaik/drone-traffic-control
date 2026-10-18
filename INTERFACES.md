# INTERFACES.md — Cross-Branch Contracts v1.0
> The single source of truth for what each branch exposes to the others.
> **Do not change an interface without a PR and sign-off from all affected owners.**
> Breaking changes require a version bump.

---

## § STATE-SCHEMA · Drone State Vector

Used by all three workstreams. Agreed Day 1, do not change unilaterally.

**Fields**:
| Field | Type | Description |
|-------|------|-------------|
| `agent_id` | string | Unique drone ID, e.g. `"drone_003"` |
| `position` | float[3] | [x, y, z] in meters. z = altitude. |
| `velocity` | float[3] | [vx, vy, vz] in m/s |
| `goal` | float[3] | Current mission waypoint in meters |
| `timestamp_ms` | int | Simulation time in milliseconds |

**JSON serialization** (for API / logging):
```json
{
  "agent_id": "drone_003",
  "position": [120.4, 340.1, 85.0],
  "velocity": [3.2, -1.1, 0.0],
  "goal": [500.0, 500.0, 100.0],
  "timestamp_ms": 4200
}
```

**Owner**: Jointly owned. Any change requires all three to approve.  
**Used by**: A-01 (env), S-01 (CBF), M-05 (API telemetry), UI-01 (visualizer).

---

## § MARL-ACTION · Action from Policy to Shield

Defined in ticket A-03. Lives in `shared/schemas.py`.

**Fields**:
| Field | Type | Description |
|-------|------|-------------|
| `agent_id` | string | Must match the `DroneState.agent_id` it was computed from |
| `proposed_accel` | float[3] | Unconstrained MARL acceleration output in m/s² |
| `timestamp_ms` | int | Must match the state timestamp it was computed from |

**Producer**: Salman's RL training scripts output the raw policy actions; Anas's `ShieldedEnv` packages them into `MARLAction` structs  
**Consumer**: Salman (`salman/cbf-safety`, `qp_solver.py`)  
**Rule**: The CBF solver must not modify `agent_id` or `timestamp_ms`. It only replaces `proposed_accel` with `safe_accel` in its output.

---

## § CBF-OUTPUT · Safe Action from Shield

Defined in ticket S-04. Lives in `shared/schemas.py`.

**Fields**:
| Field | Type | Description |
|-------|------|-------------|
| `agent_id` | string | Same as the input `MARLAction.agent_id` |
| `safe_accel` | float[3] | CBF-filtered acceleration in m/s² |
| `timestamp_ms` | int | Same as input |
| `shield_active` | bool | `true` if CBF modified the proposed action |
| `h_min` | float | Minimum barrier value across all pairwise constraints (negative = unsafe) |
| `qp_feasible` | bool | `false` means no safe action existed → fallback triggered |
| `fallback_used` | bool | `true` if HJ hover fallback was activated |

**Fallback rule**: If `qp_feasible == false`, then `safe_accel` must be set to `[0, 0, 0]` (hover). The simulation layer (Anas) must handle this case without crashing.

**Producer**: Salman (`salman/cbf-safety`)  
**Consumer**: Anas (sim loop), Aman (logging endpoint, visualizer color logic)

---

## § ENV-OUTPUT · Simulation Step Return

Follows PettingZoo `ParallelEnv` API. Defined in ticket A-01.

**`env.step(actions)` returns 5 values**:

| Return | Type | Description |
|--------|------|-------------|
| `observations` | dict[agent_id → float[18]] | Per-agent observation vector |
| `rewards` | dict[agent_id → float] | Scalar reward |
| `terminations` | dict[agent_id → bool] | `true` when agent reaches goal or crashes |
| `truncations` | dict[agent_id → bool] | `true` when episode time limit hit |
| `infos` | dict[agent_id → dict] | Must include `shield_active: bool` and `h_min: float` |

**Observation vector layout** (18 dimensions, in order):
```
[0:3]   own position (x, y, z)
[3:6]   own velocity (vx, vy, vz)
[6:9]   goal direction vector (goal - position)
[9:12]  nearest neighbor position (relative to self)
[12:15] nearest neighbor velocity (relative to self)
[15]    nearest neighbor distance (scalar, meters)
[16]    shield_active last step (0.0 or 1.0)
[17]    h_min last step (clipped to [-1, 1])
```

**Owner**: Anas (primary). Salman reviews indices [15:17] (shield fields).  
**Rule**: Indices must not shift. Adding info goes to the `infos` dict, not to the obs vector, unless all owners agree.

---

## § API-AIRSPACE · Sovereign Core REST API

Defined in ticket M-05. Runs at `http://localhost:8000` (dev) / `http://<server-ip>:8000` (cloud).

### `GET /airspace/rules`
Called once per simulation episode.

Response:
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
  "altitude_limits": { "min_m": 30.0, "max_m": 200.0 },
  "separation_minimum_m": 5.0
}
```

### `GET /airspace/notam`
Response:
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
Request body: `{ "agent_id": "drone_003", "operator": "sim", "mission_type": "delivery" }`  
Response: `{ "registered": true, "transponder_code": "A1B2" }`

**Owner**: Aman. All endpoints are versioned. Salman's `rulebook.py` reads the JSON output of `GET /airspace/rules` — do not rename fields without updating S-02.

---

## § SHIELD-LOG · Intervention Log CSV Schema

Written by Salman's `shield_logger.py` (S-06). Read by Aman's `plot_results.py` (M-08).

**CSV columns (order and names are fixed)**:
```
timestep, agent_id,
proposed_accel_x, proposed_accel_y, proposed_accel_z,
safe_accel_x, safe_accel_y, safe_accel_z,
h_value, constraint_active, qp_feasible, fallback_used
```

**Rule**: Do not rename these columns without updating both `shield_logger.py` and `plot_results.py` in the same PR.

---

## § CHAOS-API · Chaos Injector Interface

Defined in ticket A-07. Lives in `anas/chaos/chaos_injector.py`.

Three methods, all callable mid-episode:

| Method | Input | Output | Effect |
|--------|-------|--------|--------|
| `apply_comm_delay(state, delay_ms)` | `DroneState`, int | `DroneState` | Returns stale state with position/velocity extrapolated back by `delay_ms` |
| `apply_byzantine(state)` | `DroneState` | `DroneState` | Returns state with randomized position/velocity (adversarial telemetry) |
| `apply_wind_gust(accel, magnitude)` | float[3], float | float[3] | Adds bounded random noise to acceleration command |

**Owner**: Anas. Salman verifies that `delay_ms` values used in the latency analysis (S-07) match what is actually injected here.

---

## § RERUN-LOG · Visualizer Logging Interface

Defined in ticket UI-01. Lives in `aman/ui/visualizer.py`.

Anas calls one method per simulation step:

```
RerunLogger.log_step(states, safe_actions, step_metrics)
```

| Argument | Type | Description |
|----------|------|-------------|
| `states` | list[DroneState] | Current positions/velocities |
| `safe_actions` | list[SafeAction] | Used to determine drone color (shield_active) |
| `step_metrics` | dict | Keys: `n_collisions`, `n_interventions`, `min_separation`, `chaos_flags` |

**Owner**: Aman writes `RerunLogger`. Anas calls it. The method signature must not change without both agreeing.

---

## Interface change protocol

1. Open a PR on the affected branch with the proposed change.
2. Tag all owners affected by that interface as reviewers.
3. Do not merge until all affected owners approve.
4. Bump the version at the top of this file.
5. Update all "Needs" references in `TASKS.md`.

**Current version**: v1.0 — Oct 18, 2026
