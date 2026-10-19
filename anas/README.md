# Anas’s workstream — simulation, evaluation, and edge ML

This brief belongs to the `anas/simulation-edge` branch. **Anas owns the simulator and scenarios, stress evaluation, and edge ML packaging and benchmarking.**

The startup is building toward real UTM services, with a simulation/API prototype as its first engineering milestone. Its product path and current limits are in the [root README](../README.md); the shared task index is [`TASKS.md` on main](https://github.com/salmanbashashaik/drone-traffic-control/blob/main/TASKS.md); v1 state/action, timing, mock API, disturbance, and metric contracts are in [`INTERFACES.md` on main](https://github.com/salmanbashashaik/drone-traffic-control/blob/main/INTERFACES.md).

Your simulator is the source of modeled ground truth, not a real sensor or flight stack. A chaos injector is a selected, reproducible test condition, not a cyber defense. A zero-acceleration fallback is unverified and may preserve unsafe motion. Keep those limits visible in scenarios, results, plots, and benchmark summaries. Your post-prototype work defines how simulator evidence can be extended toward the selected service context; simulation results alone do not establish field readiness.

## Ownership boundary

You own simulation transitions and scenario configuration, deterministic seeding, stress-case execution, evaluation manifests and metrics, simulator-throughput characterization, and policy packaging/edge inference benchmarking. Salman owns MARL algorithm research, policy training, math, and CBF/HOCBF derivation/solver semantics. Aman owns the product workflow, mock API, platform integration, and platform-level scalability. You may wire their outputs into the environment, but shared interface changes require a pull request directly to `main` and coordinated updates by affected consumers.

Simulator scaling and platform scalability are separate claims: you report environment step throughput and fleet/scenario limits; Aman reports integration and platform bottlenecks. Edge inference latency is separate from both. There is no `dev` step, and this branch must not copy the main task plan.

## Your tasks

All tasks are planned and map to your rows in [`TASKS.md`](https://github.com/salmanbashashaik/drone-traffic-control/blob/main/TASKS.md). This brief expands the inputs, outputs, dependencies, handoffs, and acceptance conditions for your branch.

### P0-AN-01 — Simulator and scenario contract

**Inputs:** Proposed world, time step, state/observation, and logging fields in `INTERFACES.md`; product use case and v1 exclusions in the root README.

**Deliver:** A concise simulator note defining state transition and integration scheme, reset/step lifecycle, goals and episode termination, active-agent behavior, scenario configuration, deterministic seed behavior, and supported disturbances/stress injectors. Identify which fields are ground truth and which are delivered observations.

**Dependencies:** Shared v1 contract. Coordinate the actual point-mass transition and any model simplifications with Salman before his filter derivation is treated as final.

**Handoff:** Give Salman the exact state transition and bounded-disturbance model; give Aman the configuration/reset inputs and run metadata needed by the API/demo.

**Acceptance:** Reset and step semantics, time advancement, episode stop conditions, coordinate frame, seed scope, scenario fields, and chaos-injection points are explicit and consistent with the shared contract. No scenario description implies attack defense, sensor fidelity, or aircraft realism that was not implemented.

### P1-AN-01 — Deterministic v1 environment

**Inputs:** P0 scenario note, shared `DroneState` and `ObservationV1`, and mock airspace data from Aman’s API work.

**Deliver:** A parallel multi-agent environment with seeded reset/step, agreed world limits and dynamics, exactly shaped observations, configurable synthetic missions, per-agent termination/truncation/info, and episode accounting. Support the deterministic reference actions that Salman supplies.

**Dependencies:** P0-AN-01. Coordinate configuration fields with Aman and action application/timestamps with Salman before freezing the runner.

**Handoff:** Give Salman a reproducible training/evaluation environment and Aman a stable reset/step/config entry point.

**Acceptance:** The same environment version, scenario config, and seed reproduce initial conditions and transition sequence; observation and action shapes, units, and timestamps match `INTERFACES.md`; bounds and episode accounting are explicit; API-provided restrictions are represented in the scenario as documented. Environment throughput is measured only with its workload and timing boundary stated.

### P2-AN-01 — Controller wrappers and fallback integration

**Inputs:** P1 environment, Salman’s versioned reference/MARL policy and CBF/HOCBF filter, and Aman’s integrated run configuration.

**Deliver:** Comparable runners for deterministic reference, unshielded MARL, and shielded MARL; an explicit infeasible/error path using the shared zero-acceleration fallback; step and episode logs that keep proposed action, returned action, intervention, feasibility, fallback, and separation diagnostics separate. Add seeded stress scenarios such as bounded wind, delayed/dropped observation delivery, or a corrupted synthetic record where supported.

**Dependencies:** P1-AN-01 and P2-SA-01. Freeze the runner/config schema with Aman before evaluation campaigns.

**Handoff:** Give Aman one stable experiment entry point and give the evaluation task identical controller interfaces and complete run manifests.

**Acceptance:** All controller variants receive identical scenario demand, dynamics, disturbances, episode limits, and paired seeds; policy/filter failures are visible in logs; fallback semantics match `INTERFACES.md` and are never described as hover, verified, or guaranteed safe; stress inputs are reproducible, clearly named, and reported as test scenarios only.

### P3-AN-01 — Matched evaluation and stress results

**Inputs:** Frozen P2 runner and config from Aman, versioned policy/filter artifacts from Salman, and shared metric definitions.

**Deliver:** Reproducible evaluation manifests, per-run records, summary tables/plots, and a short methods note comparing the deterministic reference, unshielded MARL, and shielded MARL. Include nominal and selected stress scenarios, failure runs, and a record of any excluded/aborted/held missions.

**Dependencies:** P2-AN-01 and a stable P2-AM-01 experiment interface. Resolve metric mismatches with both consumers before the campaign.

**Handoff:** Give Salman raw summaries and manifests for technical interpretation; give Aman reviewed metrics for product interpretation.

**Acceptance:** At least 10 paired scenario seeds/runs per comparison use matched demand, dynamics, disturbance, and episode limits. Report launch/completion/abort/hold counts, completion percentage, minimum separation, pair-step samples below `d_safe`, shield interventions, infeasible filter steps, fallback agent-step events, simulation-step latency, and policy latency. Include run-to-run variation, sample counts, configs, artifact versions, and failures. Do not call the separation threshold a certified collision/NMAC/LoWC measure or infer zero risk from zero observed violations.

### P4-AN-01 — Policy packaging and edge inference benchmark

**Inputs:** Salman’s final policy artifact and adapter/preprocessing metadata; P3 evaluation configuration; the agreed edge target/runtime, if available.

**Deliver:** A packaged model (ONNX when supported by the chosen model stack) and benchmark record. State target hardware, runtime/library versions, model and preprocessing version, input shape, warm-up, measurement method and sample count; report p50/p95 inference latency and memory use where available, plus output/action agreement with the reference runtime.

**Dependencies:** P2-SA-01 artifact and P3-AN-01 evaluation interface. Agree on the benchmark device and runtime with Salman/Aman before collecting numbers. If a target device is unavailable, use a CPU-only proxy and label it plainly; do not call it an edge-device result.

**Handoff:** Give Aman the versioned artifact, invocation details, resource/latency results, and any unsupported operations or conversion discrepancies; give Salman any numerical differences relevant to method interpretation.

**Acceptance:** Another contributor can identify and invoke the same artifact/preprocessing path; measurements are reproducible from recorded metadata; latency is distinguished from simulator step and platform end-to-end latency; model differences or errors are reported rather than hidden; the result makes no flight-readiness or certification claim.

### P5-AN-01 — Validation path toward the selected real service

**Inputs:** Aman’s first-service workflow, operating domain, data sources, deployment model, and constraints; Salman’s hazard/assurance requirements; v1 scenario coverage, failures, edge benchmark, and model assumptions.

**Deliver:** A staged validation and deployment plan mapping the simulator to the selected service context. Specify additional scenarios and model evidence, recorded-data replay, interface and integration validation, shadow-mode comparison, edge/hardware-in-loop evaluation, field evidence, monitoring, and rollback criteria as applicable. Identify the owner, evidence artifact, dependency, and exit condition for each proposed stage.

**Dependencies:** P5-AM-01 and P5-SA-01, plus P3/P4 evaluation artifacts. The plan depends on the agreed service role and operating domain; do not assume flight control or a particular hardware target.

**Handoff:** Give Aman an implementation sequence and measurable deployment gates; give Salman the scenario-to-hazard traceability and evidence plan. Record partner, data-access, hardware, and external approval dependencies for the team to resolve.

**Acceptance:** The plan maps each relevant v1 model assumption and observed failure mode to additional evidence or an explicit accepted limitation. It distinguishes simulation, replay, shadow, hardware, and field evidence; defines monitoring, failure/rollback criteria, and exit gates; names dependencies and accountable owners; and does not present planned validation as completed or approved.

## Working with AI tools

- Give the agent this brief, the task ID, current `main/INTERFACES.md`, and the exact environment/filter adapter before asking it to change scenario or logging behavior.
- Ask it to preserve seed control, record every injected disturbance, and state which simulator layer receives each fault. Do not let an agent describe an injector as a defense or equate synthetic corruption with real attack coverage.
- For evaluation scripts, require explicit paired-seed matching, controller versions, and abort/hold accounting. Check the manifest and raw run records before accepting generated plots or narrative summaries.
- For model export, ask the agent to report unsupported operators, preprocessing changes, numerical/action deviations, and runtime versions. Verify output agreement before quoting latency.
- Keep simulator throughput, inference latency, and Aman’s platform scalability results separate. In pull requests directly to `main`, list task IDs, evidence, interfaces changed, and handoffs.
- When planning the move toward real services, use AI to map scenario coverage and evidence gaps, then confirm them with Aman’s selected product context and Salman’s assurance requirements. Never turn simulation success into a claim of field readiness.

## AI Starter Prompt (Copy to Cursor)

```text
You are Anas’s simulator, scenario, and evaluation engineering assistant for this startup repository. Start with P0-AN-01 and P1-AN-01 from the shared main/TASKS.md plan: define simulator/scenario semantics, then implement the deterministic v1 simulation environment and its reset/step behavior.

Before editing, read the root README.md, main/TASKS.md, main/INTERFACES.md, shared/contracts.py, this workstream README, and Salman’s current state/action formulation or open questions. The shared contract is authoritative for field names, units, bounds, timestamps, seed/config shape, and logging semantics.

Before writing code, state:
1. Which facts are fixed by the shared contract and which implementation choices remain yours.
2. Your proposed point-mass transition/integration scheme, reset/step lifecycle, episode-end behavior, active-agent handling, and exact random-seed scope.
3. Any assumptions or contract gaps that affect Salman’s math or Aman’s run/API adapter.
4. The files you intend to change and how the work meets P0-AN-01/P1-AN-01 acceptance criteria.

Then proceed with explicit implementation choices that do not change the shared contract. If an unresolved item would require a new field, different unit/bound, or different transition assumption than the owners have agreed, stop that part and propose a coordinated update to main/INTERFACES.md and shared/contracts.py rather than silently inventing the behavior.

Contract rules:
- Import and use the canonical Pydantic v2 models from shared/contracts.py. Validate reset/run inputs with ScenarioConfig; emit/validate DroneState and ObservationV1; consume policy proposals as MARLAction; and pass filter outputs as SafeAction. Do not define duplicate or branch-local Pydantic schemas.
- ScenarioConfig already contains the seed. Do not create a second conflicting seed source. Preserve deterministic initial conditions and transitions for the same code, config, and seed.
- Match all observation indices and sentinels exactly. Preserve 100 ms timestamp increments, the specified world/speed/action bounds, unique agent IDs, and per-agent parallel step results. Reject invalid shapes or action values visibly; do not silently clip them.
- Use the shared 1e-4 m/s² threshold when checking whether the shield changed an action. Keep proposal, returned action, feasibility, fallback, and violation data distinct in logs.
- The v1 ScenarioConfig does not define disturbance fields. Do not add undeclared injector/config keys. If a stress case needs configuration fields, specify the proposed contract extension and affected producers/consumers first. Injectors are test scenarios, not defenses against real attacks.
- Keep the simulator as synthetic modeled ground truth. Do not add real sensor feeds, aircraft/flight interfaces, live UTM/USS connections, or claims that simulation proves field readiness.

Coordinate the chosen transition details with Salman before treating a safety-filter assumption as satisfied, and give Aman a stable reset/config/step boundary. Before finishing, report changed files, determinism evidence or checks, assumptions, known limitations, and handoffs. Do not claim that a task is complete without reproducible evidence.
```
