# Simulator and Scenario Note (P0-AN-01) - v1, simulation-only

| Field | Value |
|---|---|
| Task | P0-AN-01 |
| Owner | Anas (branch `anas/simulation-edge`); implemented by Aman while Anas is on exams |
| Status | Accepted (Salman answered Open Items 1–9 on 2026-10-19). T1 implemented in `src/utm_services/simulation/env.py` (`DroneEnv`, `sim-v1.0`) and `runner.py` (controller variants). Sprint 1 PR F adds the T2 injectors (`Disturbances` in `env.py`: steady seeded wind, observation noise and delay, command dropout; profiles `T2_NOMINAL` and `T2_REALISTIC`, both placeholders until real telemetry data exists; the filter receives the same noisy, delayed states as the policy) and degraded command handling (hold the last command for `hold_steps`, then brake and hover). They are configured on `DroneEnv`; the `ScenarioConfig.disturbances` field follows as contract v1.6 (v1.4 is the PR #12 review, v1.5 the solve deadlines). |
| Date | 2026-10-19, revised 2026-10-21 |
| Authority | `INTERFACES.md` and `shared/contracts.py` on `main`. This note only fills gaps those documents leave open. |

## 1. Purpose and scope

This note defines how the v1 simulator behaves, so that the policy and safety filter (Salman) and the run adapter and mock API (Aman) can be built against a stable interface.

v1 is a **generic, simulation-only baseline**. Drones are point masses in a synthetic world. The simulator is the source of modeled ground truth. It is not a real sensor model, flight stack, or aircraft model. Sensing models (for example detection range) are future extensions and are listed in Section 10.

**V1 environment condition: perfect conditions only.** V1 runs under ideal, disturbance-free conditions — clear skies, no wind, no sensor noise, no communication delays, no adversarial inputs. This is the "sunny day" baseline. The goal is to establish that the core policy and safety filter work correctly under the simplest possible conditions before introducing complexity. The disturbance injectors defined in Section 7 are designed for later tiers and are **not implemented in v1**; they need the Section 8 contract change first. Adverse and stress conditions are deferred to later simulation tiers (see Section 13).

## 2. Frames, units and time

- Local Cartesian coordinates in meters, `[x, y, z]`. Velocity in m/s. Acceleration in m/s^2.
- World bounds, speed limit, acceleration limit and step size are those fixed in `shared/contracts.py`. They are imported from that module and not redefined here.
- Reset produces the state at t = 0 ms. Step k produces the state at t = 100 * k ms.
- An episode truncates at `ScenarioConfig.episode_limit_steps` (at most 1000).

## 3. Ground truth and delivered data

- **Ground truth:** the simulator-owned `DroneState` of each active agent. The safety filter reads it.
- **Delivered to the policy:** `ObservationV1` only (18 values), built from ground truth. An observation-layer disturbance (Section 7) may alter or withhold it.
- The policy has no other view of the world. Observation construction is isolated in one function so that future sensing models can replace it without changing the dynamics.
- Indices 16 and 17 have no ground-truth source, so the simulator fixes them:
  - **16 (prior shield flag):** `shield_active` of the `SafeAction` this agent received on the previous step. `0.0` at reset.
  - **17 (normalized barrier):** `h_min / d_safe²` clipped to `[-1, 1]`, where `h_min` is the minimum `||p_i − p_j||² − d_safe²` over active pairs at the state the policy is about to act on. `1.0` when there is no active pair.
- **Airspace rules.** `DroneEnv` optionally takes the mock API's `AirspaceRulesV1`. This meets the P1 requirement that API-provided restrictions are represented in the scenario. Geofence incursions are **logged, not enforced**: `infos[...]["geofence_incursion"]` per agent-step and `geofence_incursion_agent_steps` in the summary. `ObservationV1` has no geofence field, so neither the policy nor the filter can react to them. Enforcing them needs a contract change.

## 4. Dynamics

For each active agent, with applied acceleration `a`, wind acceleration `w` (zero when no wind is configured; always zero in T1) and step `dt = 0.1 s`:

```
a_eff  = a + w
v_next = v + a_eff*dt
v_next = v_next * (v_max / |v_next|)     only if |v_next| > v_max (speed cap)
p_next = p + 0.5*(v + v_next)*dt
```

- Without the cap, the trapezoid update equals `p + v*dt + 0.5*a_eff*dt^2`. It is exact for constant acceleration over one step and matches a discrete double integrator.
- **Speed cap:** if `|v_next|` exceeds the contract speed limit, `v_next` is scaled to the limit and the event is logged (`speed_capped`, `speed_cap_events`). This is a nonlinearity outside a plain double-integrator model (see Open Item 1). Using the capped `v_next` in the position update keeps position consistent with velocity. The scale sits a hair under the limit (factor `1 − 1e-12`) so float rounding never fails `DroneState` validation.
- The applied acceleration is `SafeAction.safe_accel_mps2`. For the reference and unshielded variants it is the proposed action passed through.

## 5. Step lifecycle

1. **Validate inputs.** Exactly one `SafeAction` per active agent; known and unique agent IDs; timestamp equal to the current state time. Finite values and acceleration bounds are enforced by the `SafeAction` contract. Invalid input raises and is never silently clipped.
2. **Apply dynamics** (Section 4).
3. **Bounds check.** If the new position leaves the world box, the agent is aborted (Section 6). The bounds check runs first, so an agent that leaves the world is aborted even if it also reached its goal.
4. **Goal check.** If the agent is within `goal_radius_m` of its goal, it is completed (Section 6).
5. **Separation.** Compute 3D separation over every agent that flew this step, including agents that completed or aborted on it, on the post-step positions. The initial state is also checked at reset.
6. **Outputs.** Build `rewards`, `terminations`, `truncations` and `infos` for every agent that flew this step. `observations` are built only for agents still active. A terminated agent gets no observation: an aborted agent can sit outside the world box, which `ObservationV1` cannot represent.

The step result and `infos` fields follow `INTERFACES.md`. The simulator adds `status`, `abort_reason`, `nearest_separation_m`, `separation_violation`, `speed_capped`, `geofence_incursion` and `h_min_prev_m2` (barrier minimum before the step).

**Rewards** follow `docs/formulation/P1-SA-01-reward.md` (Salman). The manifest records `reward_version = P1-SA-01-reward-v1`. Rewards are a training signal, not a safety metric.

## 6. Agent status and episode end

- **Active:** launched and not completed, aborted or held.
- **Completed or aborted:** the agent leaves the active set. It is excluded from neighbor selection and pair counts afterward but stays in the mission accounting.
- **Abort reason (proposed):** `out_of_bounds`, logged per agent. Positions are never clipped back into the world.
- **Held:** reserved for later use. No v1 trigger is defined.
- `goal_radius_m` is a simulator constant, recorded in the run manifest and not a `ScenarioConfig` field. **Value: 2.0 m (agreed, Open Item 5).**
- The episode ends when no active agents remain (termination) or the step limit is reached (truncation).
- Nearest-neighbor selection for `ObservationV1` follows `INTERFACES.md`: smallest 3D distance among active agents, ties broken by lexicographic `agent_id`.

## 7. Seeds and disturbances

**T1 status.** T1 has no random elements. The seed is recorded in the manifest and in `episode_id` but changes nothing. Same code, config and seed reproduce the episode exactly (tested). The seeding scheme below applies once injectors or generated scenarios exist.

**Seeding.** A single `numpy` `SeedSequence` is created from `ScenarioConfig.seed`. Each injector receives its own spawned stream, so adding or removing one injector does not change another. Initial conditions come from `ScenarioConfig` and are already deterministic. The seed governs disturbances and any generated scenarios. The same code version, configuration and seed must reproduce the same initial conditions and the same transition sequence.

**Injectors.** Each injector is seeded, configured, logged, and attached at one named layer. Injectors are repeatable test conditions. They are not defenses against real attacks.

| Injector | Layer | Effect |
|---|---|---|
| Bounded wind | Dynamics | Adds `w` to the acceleration, each component bounded by `w_max` |
| Observation delay or drop | Observation delivery | The policy receives an older observation or none; ground truth is unchanged |
| Corrupted record | One synthetic observation | One named record is altered |

A corrupted observation must still pass `ObservationV1` validation (see Open Item 6).

## 8. Contract gap

`ScenarioConfig` has no disturbance fields and forbids extra fields, so injectors cannot currently be configured. `INTERFACES.md` already allows simulator-configured disturbances; only the typed field is missing. Approved in Open Item 7; the PR is not open yet.

**Proposal:** add an optional field that defaults to an empty tuple, so existing producers remain valid:

```
disturbances: tuple[DisturbanceSpec, ...] = ()
```

This requires one pull request to `main` that updates `INTERFACES.md` and `shared/contracts.py` together. Affected parties: Anas (scenario producer), Aman (run configuration), Salman (wind bound assumptions).

## 9. Evaluation outputs (summary)

The simulator produces a per-step record and one episode summary with the fields defined in `INTERFACES.md`:

- **Mission accounting:** `n_launched`, `n_completed`, `n_aborted`, `n_held`, `completion_pct`. Aborted, held and incomplete missions stay in the accounting.
- **Separation:** `min_separation_m` and `separation_violation_pair_steps` (pair-step samples below `d_safe`; not a certified collision count).
- **Filter behavior:** `shield_interventions`, `qp_infeasible_steps`, `fallback_events`.
- **Timing:** mean and p95 simulator step time and policy latency, kept separate.

- **Simulator extras:** `speed_cap_events`, `geofence_incursion_agent_steps`, `n_incomplete` (still active at truncation), `steps`.

A run manifest records: `env_version`, `reward_version`, `scenario_id`, seed, `episode_id`, controller variant, `policy_id`, filter config (shielded only), episode limit, `dt_s`, `goal_radius_m`, `condition_tier`, the disturbance list (empty in T1), `rules_version` and the timing boundaries.

**Controller variants** (`runner.run_episode`): `reference` (deterministic P-I-D controller `ReferencePolicy`, `reference-p1-sa-01-pid`), `unshielded_marl` (proposal passed through with `shield_active = False`) and `shielded_marl` (HOCBF filter). Until a trained MARL policy exists, the MARL variants may run the reference controller, labeled through `policy_id`.

## 10. Future extensions (not part of v1)

**Detection layer (optional, proposed for discussion).** In v1 every drone effectively observes its nearest neighbor with no range limit and no identity or broadcast model. A later extension could add a detection layer between ground truth and the observation, with a detection range, cooperative (broadcasting) and non-cooperative drones, and dropout. This would change the meaning of `ObservationV1` indices 9-15 and require a contract change.

Key question for Salman: should the safety filter keep using ground truth for all pairs, or only detected ones? Using information the drones do not have would make the safety result look better than it is.

## 11. What this note does not claim

- No sensor fidelity, aircraft realism, attack defense, or field readiness.
- The zero-acceleration fallback is unverified. It is not a hover controller and does not guarantee separation.
- Zero observed violations in a finite simulation campaign do not demonstrate zero risk.

## 12. Open items

| # | Item | Needs an answer from | Blocks | Response |
|---|---|---|---|---|
| 1 | Is the speed cap (scale velocity to the limit) acceptable for the CBF/HOCBF model, or is another treatment needed? | Salman | Dynamics code | **Salman (2026-10-19):** Acceptable for v1 with a documented caveat. The velocity-scaling cap is a nonlinearity outside the double-integrator model the filter uses. In most cases it is conservative (drone is slower than predicted, so separation is at least as good), but this is not guaranteed in all configurations. No dynamics change needed. Anas: keep the cap and the log event. Evaluate cap-fire frequency in evaluation — if it fires in >~5% of steps on baseline scenarios, flag for reassessment. Salman: document as unmodeled nonlinearity in P0-SA-01. |
| 2 | Wind acceleration bound `w_max` | Salman | Wind injector | **Salman (2026-10-19):** `w_max = 1.0 m/s²` per component as the v1 default. That is ~29% of max control authority (`3.5 m/s²`). Worst-case velocity disturbance per step is `w_max * dt = 0.1 m/s`; worst-case position disturbance is `0.5 * w_max * dt² = 0.005 m` — negligible relative to `d_safe = 5 m`. Optional stress variant at `w_max = 2.0 m/s²` acceptable; do not exceed 2.0 m/s² without revisiting the filter formulation. Anas: record `w_max` in the run manifest. Salman: use 1.0 m/s² in P0-SA-01 robustness analysis. |
| 3 | Reward definition for training | Salman (assumed) | Training, not the step interface | **Resolved:** `docs/formulation/P1-SA-01-reward.md`. Implemented in `DroneEnv` as `P1-SA-01-reward-v1`. |
| 4 | Does the runner pass one `SafeAction` per active agent per step? | Aman | Step interface | **Salman (2026-10-19):** Confirmed required. The HOCBF QP is a joint optimization over all active pairs; if any agent's action is missing the barrier constraints cannot be formed correctly. Partial solves are not a defined fallback. The step validation must reject missing, extra, or duplicate agents before calling the filter. This is already the stated contract in `INTERFACES.md` — no change needed. |
| 5 | Value of `goal_radius_m` | All | Completion accounting | **Salman (2026-10-19):** 2.0 m accepted. The filter does not depend on this value directly — it only needs the goal check to occur after dynamics and before the next filter solve, which the step lifecycle already specifies. Do not go below 1.5 m; smaller values would require impractically precise deceleration with 100 ms steps at up to 15 m/s. Keep as a simulator constant in the manifest, not a `ScenarioConfig` field. |
| 6 | Rule for corrupted observations that must still pass validation | Salman, Aman | Corruption injector | **Salman (2026-10-19):** Add zero-mean Gaussian noise (`σ = 0.5`) to one named index group (own position, own velocity, goal displacement, or neighbor relative state). After adding noise, clamp or rescale to keep the observation within `ObservationV1` valid ranges. Do not corrupt index 15 (neighbor distance sentinel — must be ≥ 0 or exactly −1), index 16 (shield flag — must be exactly 0.0 or 1.0), or the no-neighbor zero block (indices 9–14 when index 15 == −1.0). Log the affected agent and index group each step the injector fires. The filter uses `DroneState` ground truth, so corruption affects only the policy's input. |
| 7 | Add the optional `disturbances` field to `ScenarioConfig` | All | Injectors | **Salman (2026-10-19):** Approved. Needed so that paired experiments (wind vs. no wind) are driven from `ScenarioConfig` and appear in the run manifest. Conditions: (a) `DisturbanceSpec` must be a sealed discriminated union with a `kind` field — start with `wind` (field: `w_max_mps2: float`) and `obs_corrupt` (fields: `agent_id: str`, `index_group: str`, `sigma: float`); (b) field must default to `()` so existing producers remain valid; (c) `INTERFACES.md` and `shared/contracts.py` must be updated in the same PR to `main`. Anas: open the PR and tag all owners. |
| 8 | Add `numpy` to `pyproject.toml` and `requirements.txt` | All | Seeded random streams | **Salman (2026-10-19):** Approved. Add `"numpy>=1.26,<3"` to core dependencies in `pyproject.toml`. `>=1.26` is the first release with full Python 3.11 support; `<3` avoids an unreviewed major-version jump. `requirements.txt` uses `-e .` so it picks up the change automatically. Anas: open the PR to `main` and tag all owners — this is a shared-contract dependency change. **Done:** `numpy>=1.26,<3` is on `main` (with `osqp` and `scipy` for the filter). |
| 9 | Abort (not clip) agents that leave the world box | All | Bounds handling | **Salman (2026-10-19):** Approved — abort is correct. Clipping a position would make the simulator's ground truth inconsistent with the acceleration applied, corrupting barrier value calculations for that agent and its neighbors in the filter. Abort + remove from active set is clean; the filter simply excludes the agent from subsequent solves. Log abort frequency in evaluation; if `n_aborted` is high in baseline scenarios, the barrier construction may need explicit world-boundary constraints in a later phase. No contract change needed. |

## 13. Environment condition tiers (v1 and beyond)

The simulation must eventually reflect real-world operating conditions, not just
ideal ones. Conditions are organized into tiers so each tier can be validated
before the next is introduced. Each tier builds on the last; a later tier must
not be started before the earlier tier's evaluation is complete and its failure
modes are understood.

| Tier | Label | Conditions | v1? | Notes |
|---|---|---|---|---|
| T1 | **Perfect / sunny day** | No wind, no sensor noise, no comm delays, no failures, full observability, deterministic dynamics | **Yes — v1 default** | Establish that policy and filter work under ideal conditions. All disturbance injectors disabled. |
| T2 | **Mild atmospheric** | Bounded steady-state wind (`w_max = 1.0 m/s²`), slight observation noise | Post-v1 | Wind injector from Section 7. Filter robustness analysis from P0-SA-01 must be done first. |
| T3 | **Adverse / degraded comms** | Stronger wind (`w_max = 2.0 m/s²`), observation delays and drops, one corrupted record per episode | Post-v1 | Requires OI-7 contract change and disturbance injectors validated in T2. |
| T4 | **Extreme / stress** | Wind gusts, multiple simultaneous observation failures, high drone density, near-boundary initial conditions, long episodes | Post-v1 | Stress scenarios from Anas's stress evaluation plan. Intended to find failure modes, not claim robustness. |
| T5 | **Real-world proxies** | Replayed or statistically modeled real wind profiles, realistic sensor latency distributions, mixed cooperative/non-cooperative traffic | Future | Requires real data sources, a detection layer (Section 10), and service-definition work from Phase 5. |

**Stress scenarios in v1.** P2/P3 ask for stress scenarios, and T4 is post-v1. These do not conflict. v1 stress scenarios stay in T1 physics (no disturbances) and stress geometry only: high density, head-on and crossing conflicts at up to 30 m/s closing speed, and near-boundary starts. Disturbance-driven stress (wind, observation faults) waits for T2+.

**Known filter envelope (T1).** The shielded variant uses Salman's braking-distance CBF (`docs/formulation/P0-SA-01-formulation.md` §6). Head-on pairs at 30 m/s closing need about 70 m of initial range. Closer starts are outside the envelope: the QP is infeasible and the zero-acceleration fallback applies. Evaluation must report these cases, not hide them.

**Design principle:** the world bounds (1000 m × 1000 m × 170 m altitude band)
are already large enough to represent a realistic urban or suburban airspace
patch for a small fleet. For later tiers, the world size can be increased or
tiled rather than shrinking the safety margins. Any world-size change requires a
contract update to `INTERFACES.md` and `shared/contracts.py`.

**What stays the same across tiers:** the step interface, action contract,
barrier formulation, and logging schema. Tiers change only the disturbance
configuration and scenario parameters — not the core simulator loop or data
models.

---

## 14. Visualization UI

A visual map display is required to make simulation runs inspectable and
demonstrable. The UI must show the full airspace at a scale that reflects
realistic drone operations — not a toy grid, but a proper overhead map with
spatial context.

### Requirements

- Display the full 1000 m × 1000 m world in a top-down (plan view) with an
  altitude dimension accessible via a side panel or layer toggle.
- Show each active drone as a labeled marker with its current position,
  velocity vector, and goal.
- Show geofences (cylinders from the mock API) as overlaid regions.
- Show `shield_active`, `h_min_m2`, and `fallback_used` per drone per step so
  filter behavior is immediately visible.
- Play back a completed run from logged step records, or stream a live run
  in near-real-time.
- Must run in a standard browser or as a local desktop app without requiring
  any game engine, Unity, or C# runtime.

### Technology decision

**Do not use Unity or C#.** The simulation is Python-based and the team does
not have a game engine workflow. Adding a Unity dependency would create a
separate build pipeline, binary artifacts, and a deployment surface that is out
of scope for a simulation/API prototype.

**Recommended stack: Python + Plotly Dash or Folium/Kepler.gl (offline).**

| Option | Fit | Notes |
|---|---|---|
| **Plotly Dash** (Python) | ✅ Best fit for v1 | Pure Python, runs as a local web app on `127.0.0.1`, can animate scatter plots on a 2D canvas, integrates directly with the logged step records, no extra language or runtime. Use `dash` + `plotly` with a `dcc.Interval` for live streaming or a slider for playback. |
| **Kepler.gl** (browser, GeoJSON) | Good for geospatial context | If we later want a real map tile background (satellite imagery, street map) for realism, Kepler.gl accepts GeoJSON and runs entirely in the browser. Requires converting local Cartesian coords to a lat/lon origin. |
| **Rerun.io** (Python SDK) | Good for step-by-step debugging | Already mentioned in the architecture as the optional `RerunLogger`. Rerun has a native 3D viewer, time scrubbing, and a Python log SDK. Useful for development inspection, less polished for a stakeholder demo. |
| **Pygame** | Avoid | Requires a window manager, does not run headless, hard to share as a web demo. |
| **Three.js / Babylon.js** | Future option | Good for a polished 3D web demo in a later phase; requires a JavaScript build step. Out of scope for v1. |

**V1 recommendation: Plotly Dash.** It is already compatible with the Python
stack, requires no new runtime, and can be added as an optional `[viz]`
dependency group in `pyproject.toml` without affecting core simulation
dependencies. The map canvas uses local Cartesian meters directly with no
coordinate conversion needed.

### World scale and map context

The v1 world is 1000 m × 1000 m, which is roughly a 1 km × 1 km patch — about
the footprint of 10–12 city blocks, or a mid-size airport apron. This is a
realistic operating patch for a small urban drone fleet. The UI should render it
at full scale (meters) rather than normalizing to a unit grid, so spatial
intuition about separation distances is preserved (e.g., `d_safe = 5 m` looks
like 5 m on screen, not an arbitrary fraction of the canvas).

For later tiers, if the world is scaled up (e.g., to a 5 km × 5 km region),
Plotly Dash can accommodate this without a stack change. If real map tile
backgrounds are desired for stakeholder demos, a Kepler.gl export path can be
added alongside the Dash UI.

### Open items for UI

| # | Item | Owner |
|---|---|---|
| U1 | Confirm Plotly Dash as v1 visualization stack | Aman (integration), Anas (implementation) |
| U2 | Define minimum playback controls: step slider, play/pause, speed multiplier | Anas |
| U3 | Decide whether live-streaming (during a run) or post-run replay is the v1 priority | Aman, Anas |
| U4 | Agree on which filter fields to surface in the UI per drone (shield active, h_min, fallback) | Salman, Anas |
