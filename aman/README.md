# Aman’s workstream — product, architecture, and integration

This brief belongs to the `aman/infra-api` branch. **Aman owns product and system architecture, ML system design, integration, API/platform design, and platform-level scalability.** The branch name understates that scope: this workstream is not limited to infrastructure or API coding.

The product is a simulation-only UTM assurance prototype. It lets the team configure a synthetic airspace scenario, run controller variants, inspect proposed and filtered actions, and compare measured outcomes. The shared root [README](../README.md) describes the product and limits; the single shared task index is [`TASKS.md` on main](https://github.com/salmanbashashaik/drone-traffic-control/blob/main/TASKS.md); the cross-branch data and API contracts are [`INTERFACES.md` on main](https://github.com/salmanbashashaik/drone-traffic-control/blob/main/INTERFACES.md).

Treat the report’s layered architecture as long-term research context. V1 does not connect to real aircraft, sensors, USS/UTM providers, or flight operations, and the simple fallback is not verified. Do not present interface names such as `SafeAction` or “Sovereign Core” as evidence of a safety guarantee or confirmed product brand.

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

## Working with AI tools

- Give the agent this brief, the relevant task ID, the root README, and the current `main` versions of `TASKS.md` and `INTERFACES.md` before asking it to change code or docs.
- Ask it to trace the affected producer and consumer before changing an API, message, timestamp, unit, or metric. Require it to list assumptions and files changed.
- Keep generated code and architecture proposals within v1 scope. Do not ask an agent to invent live integrations, certification evidence, benchmark results, or completed capabilities.
- Review generated schemas and integration code against the shared contract and Salman’s method assumptions. You remain accountable for the product boundary and merged behavior.
- In the pull request to `main`, name task IDs, include reproducible evidence or explain what is still planned, identify consumer handoffs, and call out any interface changes.
