# Shared interfaces — simulation-only v1

This document is the single cross-branch contract for the v1 simulation/API demo—the first engineering milestone on a startup path toward real UTM services. The values below are proposed simulation defaults; they are **not** production service contracts, real aviation rules, a regulatory standard, or the report's complete state-integrity or assurance architecture. Any change requires a pull request to `main` that identifies affected producers and consumers. Production APIs and integrations must be defined separately after the first real service and operating domain are selected.

The canonical Pydantic v2 models are in [`shared/contracts.py`](./shared/contracts.py). Producers and consumers must import and validate with those models; do not create branch-local copies. Keep the code and this document in sync through a shared-contract change.

## Contract status and units

- Coordinates are local Cartesian meters, not latitude/longitude. A position is `[x, y, z]`; velocity is m/s; acceleration is m/s².
- Episode time is integer milliseconds from reset. The nominal simulation step is 100 ms (`dt = 0.1 s`); valid action timestamps identify the current state time. The simulator applies the action to advance the following step.
- Proposed world bounds are `x,y ∈ [0, 1000] m`, `z ∈ [30, 200] m`; maximum 3D speed magnitude is 15 m/s and each acceleration component is bounded to `[-3.5, 3.5] m/s²`. Proposed maximum episode length is 1000 steps.
- The v1 pairwise minimum-separation threshold is `d_safe = 5 m`. This is a simulation parameter, not an aviation separation standard, DAA requirement, or certified well-clear threshold.
- Values are raw simulation units unless a field says otherwise. If policy preprocessing normalizes inputs, the preprocessing parameters and artifact version must travel with the model; the environment contract remains in raw units.

## State and observation

### `DroneState` — simulator/shield state

```json
{
  "agent_id": "drone-001",
  "position_m": [100.0, 200.0, 80.0],
  "velocity_mps": [2.0, 0.0, 0.0],
  "goal_m": [300.0, 200.0, 80.0],
  "timestamp_ms": 1200
}
```

`agent_id` is unique within an episode. `position_m`, `velocity_mps`, and `goal_m` are three-element numeric vectors. `timestamp_ms` is the simulator time of the state. Ground truth is available to the simulator and filter; it is not implicitly available to the policy.

### `ObservationV1` — policy input, exactly 18 float values

| Index | Meaning | Unit/encoding |
| --- | --- | --- |
| 0–2 | Own position `[x, y, z]` | m |
| 3–5 | Own velocity `[vx, vy, vz]` | m/s |
| 6–8 | Goal displacement `goal - own_position` | m |
| 9–11 | Nearest active neighbor relative position `neighbor - own_position` | m; `[0,0,0]` when no neighbor |
| 12–14 | Nearest active neighbor relative velocity `neighbor - own_velocity` | m/s; `[0,0,0]` when no neighbor |
| 15 | Distance to nearest active neighbor | m; `-1` when no neighbor |
| 16 | Whether shield modified the prior action | `0.0` or `1.0` |
| 17 | Prior step minimum pairwise barrier value divided by `d_safe²` and clipped to `[-1,1]` | dimensionless; `1.0` when there was no active pair |

Choose the nearest neighbor by 3D Euclidean distance; break exact ties by lexicographic `agent_id`. For no neighbor, use the listed sentinels. The observation is a compact engineering input, not a belief state: it does not represent sensor confidence, integrity, intent, communication trust, or all neighbors. Shield logic must use all modeled active pairs required by its formulation, not just the one neighbor exposed to the policy.

## Actions and safety-filter result

### `MARLAction` — proposed policy action

```json
{
  "agent_id": "drone-001",
  "proposed_accel_mps2": [0.2, -0.1, 0.0],
  "timestamp_ms": 1200
}
```

There must be exactly one action for each active controlled agent for the current state timestamp. Reject unknown or duplicate IDs, missing actions, stale timestamps, non-finite values, and out-of-bounds accelerations; do not silently clip invalid policy output. The action is a nominal command, not a trajectory or safety claim.

### `SafeAction` — filter output (legacy type name)

```json
{
  "agent_id": "drone-001",
  "safe_accel_mps2": [0.0, 0.0, 0.0],
  "timestamp_ms": 1200,
  "shield_active": false,
  "h_min_m2": 12.5,
  "qp_feasible": true,
  "fallback_used": false
}
```

The type name `SafeAction` means “action returned by the filter adapter”; it does **not** mean the action has a proof or is safe in reality. `h_min_m2` is the minimum modeled pairwise barrier value for the step, or `null` when there is no active pair. A candidate geometric barrier is `h_ij = ||p_i - p_j||² - d_safe²`; Salman owns the exact CBF/HOCBF constraints, derivation, and assumptions in the safety workstream.

**Shield intervention tolerance:** v1 uses `1e-4 m/s²` as the default per-component comparison tolerance. Set `shield_active=true` when `max_k |safe_accel_mps2[k] - proposed_accel_mps2[k]| > 1e-4 m/s²`; otherwise set it false. This is a simulation default and must be updated in the shared contract if Salman’s mathematical review requires a different threshold. Count actions beyond this tolerance as interventions, including fallback-modified actions, and report fallback events separately.

**Contract v1.1 (Sprint 1 PR A).** Each pair constraint carries a non-negative slack variable with a heavy linear penalty, so the QP is always feasible. A step that cannot satisfy every barrier is still solved: `qp_feasible=true`, `fallback_used=false`, and the violated pairs are reported through the filter diagnostics (`pair_slack_mps2`) and the episode slack metrics below. Positive slack means that pair's barrier was violated on that step; it is not a separation violation by itself. (The v1.1 note that slack inside `d_eff` reached 10³–10⁵ m/s² is fixed in v1.2.)

**Contract v1.2 (Sprint 1 PR B).** The barrier stays well-defined inside `d_eff`: it uses `B(s) = sqrt(4u s + c²) − c` for `s ≥ 0` and the linear extension `(2u/c) s` for `s < 0`, where `s` is the distance beyond `d_eff` and `c = FilterConfig.barrier_offset_mps` (default 1 m/s, new). When the first solve leaves barrier slack on a closing pair, those pairs are re-solved with a predicted-separation constraint in metres along a lateral and vertical escape direction; see `docs/formulation/P2-SA-02-lateral-escape.md`. Violation is now reported in metres and seconds: `FilterDiagnostics.pair_violation_depth_m` gives `d_eff − d` for every pair inside `d_eff`, and the episode summary fields are listed below. **Breaking change from v1.1:** `FilterDiagnostics.slack_total_mps2` and `slack_max_mps2` are removed, and the summary fields `total_slack_mps2`, `max_slack_mps2` and `slack_pair_steps` are replaced. `FilterDiagnostics.pair_slack_mps2` remains as raw first-solve solver output. The default `tie_break_mps2` changes from 0.5 to 1.0 m/s².

**Contract v1.3 (Sprint 1 PR B2).** New field `FilterConfig.barrier`: `"backup"` (new default) or `"braking"`. The backup-set barrier keeps each pair able to escape with the evasive fallback manoeuvre, simulated over a horizon derived from the speed and acceleration limits; see `docs/formulation/P2-SA-02-lateral-escape.md` §5. **Behaviour change from v1.2:** with default settings the filter now intervenes later on fast head-ons (about 68 m instead of 101 m at 30 m/s closing) and returns different accelerations. Pass `FilterConfig(barrier="braking")` to keep v1.2 behaviour; that option is kept for one release. No message type or episode field changed. In backup mode `lag_s` lengthens the backup horizon but is not otherwise modelled.

**Contract v1.4 (Sprint 1 PR #12 review).** The filter now keeps every drone inside the world box (x/y 0–1000 m, z 30–200 m). Each drone's QP acceleration bounds are tightened per axis by a barrier on its distance to each wall, and the backup rollout and the evasive fallback clip their manoeuvre to the same limits. The sideways escape direction is now defined when the line of sight is vertical or near vertical. **Breaking change from v1.3:** the episode field `time_inside_d_eff_s` now counts real seconds in which at least one pair is inside `d_eff`. The v1.3 meaning, the sum over pairs, moves to the new field `pair_seconds_inside_d_eff`. The final state after the last step is now also checked for violation depth. No message type changed.

The fallback now runs only on a solver error (non-`solved` OSQP status or an exception). It applies only to the conflict cluster: agents in pairs whose barrier the nominal action violates, grown until every agent outside the cluster satisfies its constraints against the cluster's fallback actions. Agents outside the cluster keep their nominal action with `qp_feasible=true` and `fallback_used=false`. Each cluster agent gets an evasive acceleration away from its most critical neighbour (lowest barrier value): braking along the line of sight, a lateral component, and a vertical component, scaled so the largest component equals the ±3.5 m/s² bound. Cluster agents have `qp_feasible=false` and `fallback_used=true`. **Breaking change from v1.0:** v1.0 required the fallback action to be zero acceleration; v1.1 allows any acceleration within the component bounds. This fallback is a heuristic, not a verified recoverability or backup assurance layer, and it ignores third neighbours. Since v1.4 it respects world bounds.

## Simulator contract

The environment implements a PettingZoo-style parallel step. Its step result has the five mappings `observations`, `rewards`, `terminations`, `truncations`, and `infos`, keyed by active `agent_id`. `infos[agent_id]` includes at least `shield_active`, `h_min_m2`, `qp_feasible`, `fallback_used`, `nearest_separation_m`, and `separation_violation` for the just-completed step. The per-agent `separation_violation` flag is true if that agent was part of any pair below `d_safe` during the step. Episode metric `separation_violation_pair_steps` counts each unordered pair once per step when its separation is below `d_safe`. Neither measure is a certified collision or aviation near-miss measure.

Reset accepts a `ScenarioConfig` (including its seed) and returns initial observations and metadata. The same code, configuration, and seed should reproduce the same scenario and transition sequence. A run manifest records code/artifact version, scenario, seed, controller variant, and episode limit. The nominal transition is a bounded point-mass simulation; implementation details such as integration scheme must be documented with the simulator artifact.

## Mock API contract

The API is local to the demo and returns synthetic simulation inputs. It must not call a real UTM service, USS, registry, or external aircraft system.

The canonical Pydantic response models for these routes are `AirspaceRulesV1`, `NotamListV1`, and `DroneRegisterResponse` in `shared/contracts.py`. `GeofenceV1`, `AltitudeLimitsV1`, and `NotamV1` define their nested records. API failures use `APIError` with exactly `code` and `message`; return a stable code and human-readable message for a given failure class. These schemas are simulation-only v1 defaults.

### `ScenarioConfig` — local simulator reset input

`ScenarioConfig` is passed to the simulator reset/run adapter; it is not a new HTTP route. This is the v1 shape consumed by Anas’s simulator and Aman's API/demo integration:

```json
{
  "scenario_id": "baseline-small-n",
  "seed": 17,
  "episode_limit_steps": 1000,
  "drones": [
    {
      "agent_id": "drone-001",
      "initial_position_m": [100.0, 200.0, 80.0],
      "initial_velocity_mps": [2.0, 0.0, 0.0],
      "goal_m": [300.0, 200.0, 80.0]
    }
  ]
}
```

`scenario_id` is non-empty; `seed` is a non-negative integer; and `episode_limit_steps` is an integer from 1 through 1000 (defaults to 1000 when omitted). `drones` contains at least one entry with a unique `agent_id`. Initial positions and goals use the world bounds above; initial 3D speed magnitude must not exceed 15 m/s. Scenario-specific disturbances are configured by the simulator scenario implementation and logged; do not add undeclared fields to this v1 model. Use `ScenarioConfig` from `shared/contracts.py` rather than defining a branch-local schema.

### `GET /airspace/rules`

Returns this v1 shape; geofence geometry is a closed cylinder in local Cartesian coordinates, active for the whole episode unless a different interval is explicitly provided:

```json
{
  "version": "sim-rules-v1",
  "geofences": [
    {"id": "restricted-1", "center_xy_m": [500.0, 500.0], "radius_m": 50.0,
     "floor_m": 30.0, "ceiling_m": 200.0, "active_from_ms": 0, "active_until_ms": 100000}
  ],
  "altitude_limits_m": {"min": 30.0, "max": 200.0},
  "separation_minimum_m": 5.0
}
```

An implementation may use fewer fields only through a documented, shared contract change. This is a simplified demo geometry; do not imply support for arbitrary regulatory airspace geometry.

### `GET /airspace/notam`

Returns `{ "notams": [...] }`. Each synthetic record contains `id`, `description`, `bbox_m: [xmin, ymin, zmin, xmax, ymax, zmax]` in local coordinates, `active_from_ms`, and `active_until_ms` in episode time. These are demo notices, not official NOTAMs.

### `POST /drone/register`

Accepts `{ "agent_id", "operator", "mission_type" }` and returns `{ "registered": true, "sim_registration_id" }`. The ID is synthetic and scoped to the simulation. This does not register an actual aircraft or operator.

Invalid JSON, missing required fields, and invalid ranges return HTTP 422 with `APIError.code="invalid_request"`. A duplicate `agent_id` in the running mock API process returns HTTP 409 with `code="duplicate_agent_id"`; an unknown route returns HTTP 404 with `code="not_found"`; unavailable synthetic scenario data returns HTTP 503 with `code="scenario_unavailable"`. No request should silently succeed with omitted required data. The local v1 implementation keeps registrations in process memory and generates deterministic synthetic IDs; restarting it clears registrations. Route versioning and any extra response fields must be documented before consumers depend on them.

## Production service boundary — future work

The routes above are mock interfaces for the simulator. They are a starting point for product learning, not endpoints to connect to live traffic or operational users. Before a real service uses data from aircraft, operators, USS/UTM providers, or airspace authorities, Phase 5 in [`TASKS.md`](./TASKS.md) must define the intended user and operating domain, each data source’s authority and provenance, freshness/quality behavior, authenticated access, privacy and security controls, auditability, availability and support needs, service-specific failure/degraded modes, monitoring, and deployment/rollback gates. The exact contract depends on whether the first service provides information, coordination, advice, or control influence. Keep each production contract versioned and distinct from this simulation schema until its requirements and evidence are reviewed.

## Disturbance and stress inputs

Chaos/stress inputs are simulator scenarios, for example: bounded wind acceleration, delayed or dropped policy observations, or one deliberately corrupted synthetic telemetry record. Every injector is seeded/configured and logged. A communication delay affects delivered observations, while the simulator retains its own ground truth. A telemetry corruption affects the named synthetic input only. These injectors test behavior under selected conditions; they do not defend against real attacks, guarantee cyber resilience, or model all adversarial behavior.

## Logging and evaluation fields

Emit a per-step record sufficient to reconstruct and diagnose controller behavior: `episode_id`, `scenario_id`, `seed`, `controller_variant`, `timestamp_ms`, agent states, proposed actions, returned actions, and filter flags. Emit one episode summary with at least:

| Field | Definition |
| --- | --- |
| `n_launched`, `n_completed`, `n_aborted`, `n_held` | Mission accounting; incomplete/held/aborted missions remain visible. |
| `completion_pct` | `100 * n_completed / n_launched`; null if nothing launched. |
| `min_separation_m` | Minimum 3D separation over all modeled active pairs and steps; null if no pair existed. |
| `separation_violation_pair_steps` | Count of pair-step samples with separation `< d_safe`; not unique events or a certified collision count. |
| `shield_interventions` | Count of agent-step returned actions modified beyond the documented tolerance, including fallback-modified actions. |
| `qp_infeasible_steps` | Count of joint filter steps reported infeasible or errored. |
| `fallback_events` | Count of agent-step actions where the evasive fallback was used (solver errors only since v1.1). |
| `max_violation_depth_m` | Largest `d_eff − d` over all pairs and steps, in metres; 0 if no pair entered `d_eff` (v1.2). Reported for every variant. |
| `time_inside_d_eff_s` | Time during which at least one pair is inside `d_eff`, in seconds (one step = 0.1 s) (v1.4; before v1.4 this was the pair sum). |
| `pair_seconds_inside_d_eff` | Sum over pairs of the time each pair spent inside `d_eff`, in pair-seconds (v1.4). |
| `pair_violations` | Per pair: `max_depth_m` and `time_inside_s`, only for pairs that entered `d_eff` (v1.2). The key is the two sorted agent ids joined by a vertical-bar character. |
| `mean_step_time_ms`, `p95_step_time_ms` | Simulator/whole-step latency under a named measurement boundary. |
| `mean_policy_latency_ms`, `p95_policy_latency_ms` | Policy inference latency, measured separately and with timing method stated. |

Keep step latency, policy inference latency, and edge device latency distinct. Compare controller variants using paired scenario seeds and matched demand, dynamics, disturbance, and episode limits. Report sample count and run-to-run variation. Zero observed separation violations in a finite simulation is not proof of zero risk.

## Visualization boundary

Visualization and rerun logging are optional consumers of the above records, for example `RerunLogger.log_step(states, proposed_actions, filtered_actions, metrics)`. Visualization is not in the safety loop and is not an authority for ground-truth state. Any displayed safety/intervention label must match the logged filter result and use the limitations stated here.
