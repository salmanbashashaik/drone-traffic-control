# Aman: v1 product workflow and integration boundaries

This is the proposed Aman-owned architecture for **P0-AM-01**. The v1 data-flow diagram in the [root README](../../README.md#v1-system-architecture-data-flow) remains authoritative; the shared contract is [INTERFACES.md](../../INTERFACES.md). This proposal is pending the P0 handoffs from Salman and Anas, especially their mathematical assumptions and simulator reset/step details.

## One user run

1. A developer chooses a controller variant and a seeded `ScenarioConfig` (which includes the episode limit) through the planned local run entry point. No remote user or production service is assumed.
2. Aman-owned run orchestration validates the scenario with `ScenarioConfig` from `shared/contracts.py`, obtains synthetic airspace rules/notices from the local mock API, and prepares a run manifest containing the scenario, seed, controller variant, episode limit, and code/artifact versions.
3. The orchestration passes the validated scenario to Anas's simulator reset boundary. The simulator owns ground truth, applies each returned action, advances time, and emits the next policy observations and per-step metadata.
4. For each active agent, Salman's policy adapter consumes `ObservationV1` and returns a bounded `MARLAction`. The orchestrator coordinates this call; the policy owns inference and its model/preprocessing metadata.
5. Salman's filter receives the proposed actions and all modeled active `DroneState` values required by its formulation. It returns `SafeAction` values and feasibility/intervention data. The QP is always feasible; on a solver error the v1 contract specifies an evasive fallback for the conflict cluster with `qp_feasible=false` and `fallback_used=true`; this fallback is unverified.
6. The simulator applies the returned actions and advances the step. Aman-owned run logging records state, proposed and returned action, filter flags, scenario/run identifiers, and timing fields defined in `INTERFACES.md`.
7. At episode end, the run layer aggregates the shared summary metrics and writes a reproducible result record. The demo may display these records, but a visualization is not in the control or safety loop.

## Component ownership and boundaries

| Component | Owner | Responsibility and boundary |
| --- | --- | --- |
| Local run orchestration | Aman | Validates run input, selects controller variant/artifact, orders component calls, creates run metadata, and coordinates logging. It does not calculate the simulator's state transition or safety constraint. |
| Mock airspace API | Aman | Serves fixed synthetic rules/notices and in-memory synthetic registrations over local routes. It contacts no external provider and is not an authoritative source of real airspace data. |
| Simulator | Anas | Owns scenario generation, ground truth, dynamics, reset/step behavior, termination, and scenario disturbances. |
| Policy and action adapter | Salman | Owns policy method, observation preprocessing, model metadata, and bounded proposed actions. A proposed action is not safety-filtered. |
| CBF/HOCBF filter | Salman | Owns the mathematical formulation, solver setup, assumptions, and filter status. It uses all modeled active pairs required by that formulation. |
| Step/episode logs and demo result view | Aman | Persists and presents the records and metrics in the shared contract. It reports measured outputs and does not certify safety. |
| Edge packaging and benchmark | Anas | Owns policy packaging and device/runtime inference measurements; these are distinct from whole-step and platform throughput. |

The simulator, policy, filter, and run orchestration remain in one local Python process for the first vertical slice. The mock HTTP API runs as a local loopback process on `127.0.0.1`; it is not exposed as a network service. This is a development deployment choice, not a production topology or security design.

## Model loading, configuration, and run metadata

- The run configuration is the canonical `ScenarioConfig`; parse it once at the boundary and pass the validated model onward. Do not add a second branch-local scenario model.
- The run orchestrator selects a named controller variant and artifact reference. The policy adapter owns loading and preprocessing of its artifact; the exact artifact format and loader are decided with Salman and Anas when their P0/P1 handoffs arrive.
- The run manifest is created before reset and records at least `episode_id`, `scenario_id`, `seed`, `controller_variant`, `episode_limit_steps`, code/artifact version, and the configuration. The per-step and episode fields remain those in `INTERFACES.md`.
- Aman coordinates calls, but the simulator owns state evolution and action application; the policy owns proposed action generation; the filter owns filtering. This keeps the orchestrator from becoming a second source of controller or dynamics logic.

## API scope and errors

The local mock API implements only `GET /airspace/rules`, `GET /airspace/notam`, and `POST /drone/register` from [INTERFACES.md](../../INTERFACES.md). Its HTTP layer is in `src/utm_services/api/app.py`; scenario validation is in `src/utm_services/application/scenario_adapter.py`. Responses use canonical Pydantic models in `shared/contracts.py`. Invalid request payloads return the shared `APIError` envelope with HTTP 422; duplicate synthetic registration returns HTTP 409; unavailable scenario data returns HTTP 503; unknown routes return HTTP 404. Mock data and registrations are local simulation data. None of these routes is a real UTM service or aircraft registration service. From the repository root, install the project with `python -m pip install -e .` and start the loopback API with `utm-mock-api`.

The response models and error codes are part of the shared contract on `main`. Treat `INTERFACES.md` and `shared/contracts.py` there as authoritative; propose future changes through a pull request that names affected producers and consumers and updates both together.

## Handoffs and unresolved integration detail

- **To Anas:** scenario inputs are the exact `ScenarioConfig` fields in `shared/contracts.py`. The initial adapter validates the payload and calls `reset(config)`. Confirm the concrete reset return value and the simulator's parallel `step` signature before building the full run loop; this proposal does not invent either shape.
- **To Salman:** the policy boundary is one validated `ObservationV1` per controlled agent to one bounded `MARLAction`; the filter consumes the batch of proposed actions and the modeled states needed by the formulation. Confirm model loading metadata, batch semantics, and filter result details with the method handoff.
- **To both:** report any missing or conflicting field in the shared contract before relying on it. Keep real-service role, data authority, live integrations, and deployment obligations open for P5 service definition.

## Startup path

V1 is a learning prototype on the path toward real UTM services. After evidence and customer/operator discovery, P5 selects the first workflow and operating domain, defines the service's role and trust boundaries, and specifies production requirements. Live data connectors, production security/reliability controls, and deployment gates belong to that reviewed service plan; this local architecture does not imply them.
