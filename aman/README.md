# Aman’s workstream — product, architecture, and integration

This brief belongs to the `aman/infra-api` branch. **Aman owns product and system architecture, ML system design, integration, API/platform design, and platform-level scalability.** The branch name understates that scope: this workstream is not limited to infrastructure or API coding.

We are building a startup toward real UTM software services. V1 is a simulation/API prototype that lets the team configure a synthetic airspace scenario, run controller variants, inspect proposed and filtered actions, and compare measured outcomes. It is an early product milestone; the first real service and operating domain are selected after its results and customer discovery. The shared root [README](../README.md) describes the product path; the single shared task index is [`TASKS.md` on main](https://github.com/salmanbashashaik/drone-traffic-control/blob/main/TASKS.md); simulation data and API contracts are [`INTERFACES.md` on main](https://github.com/salmanbashashaik/drone-traffic-control/blob/main/INTERFACES.md).

Treat the report’s layered architecture as research context for product evolution. V1 does not connect to live aircraft, sensors, USS/UTM providers, or operations, and the simple fallback is not verified. Your architecture work should make it possible to evolve from the prototype toward the selected real service while keeping the v1 boundary explicit. Do not present interface names such as `SafeAction` or “Sovereign Core” as evidence of a safety guarantee or confirmed product brand.

## Ownership boundary

You own the product workflow and cross-component integration: mock API and scenario configuration, policy/filter runtime integration, the interface a teammate uses to run and inspect a demo, deployment/run documentation, and system-level scalability characterization. You own scalability at the **platform and integration layer**—for example service boundaries, request/run orchestration, resource use, and bottlenecks across components. Anas owns simulator throughput and policy/edge inference benchmarking. Salman owns the mathematical policy and filter formulation and the limits of its safety assumptions.

Keep shared interface definitions in `main/INTERFACES.md`. A branch implementation may not silently redefine fields, units, error behavior, timestamps, or metrics. If a change is needed, include the contract change in a pull request directly to `main`, explain its effect on producers and consumers, and coordinate with the affected owners. Do not create a `dev` integration step or copy the shared task plan into this branch.

## Your tasks

All tasks below are planned. They map to Aman’s rows in the main [`TASKS.md`](https://github.com/salmanbashashaik/drone-traffic-control/blob/main/TASKS.md). The shared row remains the tracking source; this brief gives your branch the specific working instructions.

### P0-AM-01 — Product workflow and system architecture

**Inputs:** Root product README, `INTERFACES.md`, Anas’s simulator/scenario proposal, and Salman’s candidate policy/filter assumptions.

**Deliver:** A concise product workflow and component diagram showing user inputs, mock API, simulator, MARL policy, filter/fallback, logs/results, and component ownership. State where orchestration, model loading, configuration, and run metadata live. Keep real-world integrations and long-term research components outside the v1 boundary.

**Dependencies:** Shared product scope and interface contract; the initial design can start before implementation. Incorporate the P0 handoffs from Salman and Anas before the architecture is treated as agreed.

**Handoff:** Give Anas the simulator/API boundary and scenario inputs; give Salman the policy/filter runtime boundary; give both the decisions needed for the vertical slice.

**Acceptance:** A teammate can trace one run from configuration through simulation to results, identify which component owns every state/action transition, and name what is mock or simulated. The diagram and API responsibilities agree with the root README and `INTERFACES.md`; no unbuilt component is described as existing.

### P1-AM-01 — Mock API and scenario integration

**Inputs:** P0 architecture, Anas’s v1 reset/step/config needs, and exact routes and fields in `INTERFACES.md`.

**Deliver:** Local mock responses for synthetic rules, notices, and simulated drone registration; an adapter that turns scenario configuration into simulator inputs; documented request/response and error behavior.

**Dependencies:** P0-AM-01 and Anas’s P1 environment/config shape. Coordinate field and timing expectations before consumers bind to them.

**Handoff:** Give Anas deterministic scenario configuration and give Salman a stable run setup for controller integration. Clearly identify all generated IDs and data as simulation-only.

**Acceptance:** The three routes and payloads match the shared contract; coordinate and time units are explicit; invalid or unavailable input produces a visible error; registering a vehicle creates only a synthetic simulation record; execution does not contact a real service.

### P2-AM-01 — Integrated demo and product observability

**Inputs:** P1 API adapter, Anas’s simulator runner and logging, Salman’s versioned policy/filter adapter, and the metric definitions in `INTERFACES.md`.

**Deliver:** An integrated run path with a small, documented configuration surface (CLI, API, or simple UI selected in the architecture), plus a result view that identifies scenario/seed/controller variant, exposes mission accounting, separation, filter interventions, infeasibility, fallbacks, and latency, and lets a reviewer inspect per-step proposed and returned actions. Add a visualization only if it helps inspect those outputs; it is not part of the safety loop.

**Dependencies:** P1-AM-01, P2-AN-01, and P2-SA-01. Keep experiment inputs stable before Anas starts matched evaluation.

**Handoff:** Give Anas a frozen experiment entry point and configuration schema; give the team a runnable demo that can be reviewed against the product workflow.

**Acceptance:** A teammate can choose a scenario, seed, and controller variant, launch a run, inspect its run manifest, proposed and returned actions, filter status, core metrics, and repeat the same configuration. The UI/API labels v1 as simulation-only and never labels the zero-acceleration fallback “hover,” “verified,” or guaranteed safe.

### P3-AM-01 — Product interpretation and evaluation summary

**Inputs:** Anas’s paired multi-run result artifacts and Salman’s technical interpretation and assumptions review.

**Deliver:** A concise product evaluation summary separating measured outcomes, interpretation, unresolved limitations, and next product decisions. Link every quantitative statement to its run/config artifact.

**Dependencies:** P3-AN-01 and P3-SA-01. Do not write comparative conclusions before both handoffs arrive.

**Handoff:** Give all contributors and future readers one evidence-backed account of what the prototype did and did not show.

**Acceptance:** Controller variants, metrics, sample count, test conditions, and uncertainty agree with the logged runs; no vendor capability or report hypothesis is presented as a prototype result; zero observed failures are not called proof of zero risk; any scalability claim names workload and environment.

### P4-AM-01 — Reproducible product demo and platform characterization

**Inputs:** Integrated P2 demo, reviewed P3 evaluation, Anas’s packaged model and benchmark record, and Salman’s final method/assumptions note.

**Deliver:** Run/setup instructions, a versioned artifact/config record, and a concise platform characterization covering integration boundaries, tested workload, resource use, bottlenecks, and unimplemented deployment needs.

**Dependencies:** P2-AM-01, P3-AM-01, P4-AN-01, and P4-SA-01.

**Handoff:** Deliver a reproducible v1 demo to the team and document what would be required before considering future platform work.

**Acceptance:** A new contributor can follow the instructions to run and inspect the documented scenario; every named artifact and runtime has a version or stated limitation; platform scalability is distinguished from simulator throughput and edge inference; no operational or flight-readiness claim is implied.

### P5-AM-01 — Define the first real service

**Inputs:** V1 evaluation and team handoffs, prototype architecture, and customer/operator/partner discovery for candidate workflows.

**Deliver:** A first-service product brief and production architecture. Define the target user and job, operating domain, deployment model, and whether the initial service provides information, coordination, advice, or control influence. Map its data sources and trust boundaries; specify production API/integration needs, identity/access, data provenance, audit, security/privacy, reliability/availability, monitoring, support, and scale requirements.

**Dependencies:** P4-AM-01 and stakeholder/customer discovery. The service scope must be clear before Salman finalizes assurance requirements and Anas finalizes the validation path.

**Handoff:** Give Salman a bounded intended use and service role for hazard/assurance analysis. Give Anas the selected workflow, data sources, deployment environment, and performance constraints for validation planning. Bring unresolved partner and external approval needs to the team.

**Acceptance:** The brief names the first service, user, workflow, operating domain, system boundaries, producers/consumers, data authority/freshness expectations, reliability/support expectations, service failure modes, and product success measures. It separates decisions from assumptions and lists dependencies on discovery. Production implementation work is sequenced behind the review of this specification and assurance/validation plans.

## Working with AI tools

- Give the agent this brief, the relevant task ID, the root README, and the current `main` versions of `TASKS.md` and `INTERFACES.md` before asking it to change code or docs.
- Ask it to trace the affected producer and consumer before changing an API, message, timestamp, unit, or metric. Require it to list assumptions and files changed.
- Keep generated code and architecture proposals within v1 scope. Do not ask an agent to invent live integrations, certification evidence, benchmark results, or completed capabilities.
- For post-v1 service design, use AI to enumerate options and questions, then verify them with the intended users, partners, and technical owners. Do not let it choose an operating domain, authority boundary, or production obligation without evidence and team agreement.
- Review generated schemas and integration code against the shared contract and Salman’s method assumptions. You remain accountable for the product boundary and merged behavior.
- In the pull request to `main`, name task IDs, include reproducible evidence or explain what is still planned, identify consumer handoffs, and call out any interface changes.

## AI Starter Prompt (Copy to Cursor)

```text
You are Aman’s product and systems architecture implementation assistant for this startup repository. Work only on P0-AM-01 and P1-AM-01 from the shared main/TASKS.md plan: product workflow/system architecture, the v1 mock API, and scenario-to-simulator integration.

Before editing, read the root README.md, main/TASKS.md, main/INTERFACES.md, shared/contracts.py, and this workstream README. The shared docs and contracts define the current v1 boundary. Do not infer that a planned component already exists.

Before writing code, state:
1. The assumptions you will rely on, separated into fixed contract facts and implementation choices.
2. The relevant producer/consumer path and the fields each boundary accepts or returns.
3. The files you expect to change and how each change will meet the task acceptance criteria.

Then proceed with the task using the stated implementation choices. If a required field, service behavior, authority boundary, or interface is missing or conflicts across documents, do not invent a contract: report the exact mismatch and propose a coordinated update to main/INTERFACES.md and shared/contracts.py.

Contract rules:
- Import and use the canonical Pydantic v2 models from shared/contracts.py: ScenarioConfig for local run/reset configuration; DroneState, ObservationV1, MARLAction, and SafeAction at their respective boundaries; and DroneRegisterRequest/DroneRegisterResponse for the synthetic registration route.
- Validate incoming JSON with model_validate (or model_validate_json where appropriate). Do not bypass validation with hand-written duplicate models, branch-local Pydantic classes, silent coercion, or action clipping.
- Do not add undeclared fields to ScenarioConfig or reinterpret timestamp, units, IDs, action bounds, fallback flags, or the 1e-4 m/s² intervention tolerance.
- Match the documented GET /airspace/rules, GET /airspace/notam, and POST /drone/register mock contracts. Do not contact a real USS/UTM service, registry, aircraft, or sensor. If a route needs a Pydantic response model that shared/contracts.py does not yet define, identify that gap and propose a shared-contract extension instead of defining a local model.

P0-AM-01 work: make the user/run workflow, component ownership, mock API boundary, scenario configuration path, integration points, logging, and v1 exclusions easy for a teammate to follow. Keep future real-service decisions explicitly open until the product-definition task supplies evidence.

P1-AM-01 work: implement or refine the local mock routes and the adapter from a validated ScenarioConfig to Anas’s simulator. Make errors explicit and stable, keep generated registration IDs synthetic, and keep configuration and run metadata reproducible. Do not build production authentication, live integrations, or a real aircraft registry as part of v1.

Before finishing, summarize changed files, validations performed, contract assumptions, unresolved decisions, and the handoff needed from Anas and Salman. Do not claim a task is complete without pointing to its implementation and evidence. Do not claim the prototype is operational or safety-certified.
```
