# Shared task plan

This is the **sole shared task index** for the startup’s product path, from the v1 prototype through definition of the first real service. It assigns one directly responsible owner per task, records dependencies and handoffs, and defines phase gates. Detailed implementation notes stay in the owner's branch brief; do not copy this plan to workstream branches. All tasks are planned until their deliverables and evidence exist. There are no calendar-date commitments in this plan.

## How to use this plan

1. Start with the root [`README.md`](./README.md) for startup product direction and [`INTERFACES.md`](./INTERFACES.md) for simulation-only v1 contracts.
2. Work from the matching owner brief: [Aman](https://github.com/salmanbashashaik/drone-traffic-control/blob/aman/product-platform/aman/README.md), [Salman](https://github.com/salmanbashashaik/drone-traffic-control/blob/salman/marl-safety/salman/README.md), or [Anas](https://github.com/salmanbashashaik/drone-traffic-control/blob/anas/simulation-edge/anas/README.md).
3. Open a pull request from the named workstream branch directly to `main`. Describe evidence against the task acceptance criteria and identify any shared-contract changes.
4. Do not mark a task complete because documentation, generated code, or an AI agent claims it is complete. Link the artifact or reproducible result that meets its criteria.

## Phase 0 — Product and contract baseline

**Gate:** The team agrees on the v1 problem, exclusions, system boundaries, units, action/observation meanings, and measurable comparison. No implementation task depending on an unresolved contract begins.

| ID | Owner | Deliverable | Depends on | Handoff | Acceptance criteria |
| --- | --- | --- | --- | --- | --- |
| P0-AM-01 | Aman | Product workflow, component boundaries, integration/deployment sketch, and mock API responsibilities. | None | Gives the vertical-slice boundaries to Salman and Anas. | Describes user input, simulation loop, visible outputs, API boundary, and explicit non-goals; agrees with root README and `INTERFACES.md`. |
| P0-SA-01 | Salman | Mathematical notation and candidate MARL/CBF/HOCBF formulation with assumptions and known failure conditions. | `INTERFACES.md` v1 dynamics and state/action units | Gives formulas, solver inputs, and proof limitations to Anas for implementation and Aman for accurate product claims. | Defines modeled relative state, barrier condition, control bounds, sample timing assumptions, infeasibility behavior, and what is not proven; no unqualified safety guarantee. |
| P0-AN-01 | Anas | Simulator/scenario configuration and reproducibility plan, including which chaos inputs are injected and where. | `INTERFACES.md` v1 world and logging contracts | Gives state transition, seed, scenario, and metric requirements to Salman and Aman. | Defines deterministic reset/step behavior, episode termination, scenario parameters, disturbance semantics, and the exact shared evaluation outputs. |

## Phase 1 — Simulation and API vertical slice

**Gate:** A seeded small-N scenario can reset, step, emit the agreed state and observation, consume mock airspace configuration, and produce a complete log. This establishes plumbing, not MARL performance or safety.

| ID | Owner | Deliverable | Depends on | Handoff | Acceptance criteria |
| --- | --- | --- | --- | --- | --- |
| P1-AN-01 | Anas | Simulator environment implementing v1 dynamics, state, observation, seeded reset/step, and basic episode accounting. | `P0-AN-01`, `INTERFACES.md` | Supplies a runnable environment to Salman and Aman. | Same seed and config reproduce the same episode; all fields use specified units/shapes; environment has deterministic reference behavior and logs termination/abort/hold counts. |
| P1-AM-01 | Aman | Mock API for synthetic rules, notices, and registration plus scenario-to-simulator integration. | `P0-AM-01`, `P1-AN-01`, `INTERFACES.md` | Supplies configured scenario inputs to Anas and the demo/integration path to Salman. | Routes and payloads match `INTERFACES.md`; generated registration data is synthetic; errors and unavailable fields are explicit; no real service is contacted. |
| P1-SA-01 | Salman | Deterministic reference policy and written MARL policy/action adapter specification. | `P0-SA-01`, `P1-AN-01` | Gives comparable baseline and policy interface to Anas and Aman. | Deterministic policy is reproducible and emits valid bounded v1 actions; policy adapter states observation preprocessing and model metadata; neither is described as a safety controller. |

## Phase 2 — MARL, filter, and integrated demo

**Gate:** One end-to-end episode can run as deterministic reference, unshielded MARL, and shielded MARL; every step records proposed and applied actions, filter status, and fallback use. The fallback remains explicitly unverified.

| ID | Owner | Deliverable | Depends on | Handoff | Acceptance criteria |
| --- | --- | --- | --- | --- | --- |
| P2-SA-01 | Salman | MARL training/evaluation method and CBF/HOCBF filter implementation or implementation-ready package, including solver setup. | `P0-SA-01`, `P1-AN-01`, `P1-SA-01` | Gives Anas a versioned policy/filter artifact and Aman the integration contract. | Reproducible config/checkpoint; action bounds enforced; filter accepts shared state/action types and reports feasibility, intervention, and barrier data; assumptions and known infeasible cases documented. |
| P2-AN-01 | Anas | Simulator wrappers for unshielded and shielded execution, deterministic scenarios, and explicitly simple zero-acceleration fallback wiring. | `P1-AN-01`, `P2-SA-01` | Gives Aman a stable end-to-end runner and evaluation hooks. | Same scenario inputs are used across variants; per-step flags distinguish pass-through, intervention, infeasibility, and fallback; fallback is triggered only as documented and is not called hover, verified, or safe. |
| P2-AM-01 | Aman | Integrated mock API/demo flow, product-facing run configuration and results view, per-step proposed/returned-action inspection, and platform-level integration/scalability notes. | `P1-AM-01`, `P2-AN-01`, `P2-SA-01` | Gives team a reviewable demo and hands the frozen experiment interface to Anas. | A teammate can configure and run a scenario, identify controller variant, inspect proposed and returned actions, core metrics, filter status, and interventions, and reproduce a run from its configuration; demo makes simulation-only status visible. |

## Phase 3 — Matched evaluation and limits

**Gate:** A reproducible report compares all three controller variants under paired scenario conditions and clearly distinguishes measurements from guarantees or hypotheses.

| ID | Owner | Deliverable | Depends on | Handoff | Acceptance criteria |
| --- | --- | --- | --- | --- | --- |
| P3-AN-01 | Anas | Multi-run matched evaluation, stress scenarios, result tables/plots, and reproducible run manifests. | `P2-AN-01`, frozen `P2-AM-01` experiment interface | Gives result artifacts and raw summaries to Salman and Aman. | Runs reference, unshielded MARL, and shielded MARL on identical demand, dynamics, episode limits, and paired scenario seeds across at least 10 runs/seeds; reports completion, abort/hold counts, minimum separation, separation-violation pair-step samples, interventions, infeasible steps, fallbacks, and step/policy latency with run-to-run variation; archives configs and failures. |
| P3-SA-01 | Salman | Technical interpretation of policy/filter behavior and safety-assumption review of results. | `P3-AN-01`, `P0-SA-01` | Gives Aman reviewed claim language and unresolved mathematical limitations. | Separates empirical observations from formal results; discusses infeasible filters, fallback exposure, model/measurement limits, and any assumption violations; makes no inference of zero risk from zero observed failures. |
| P3-AM-01 | Aman | Product interpretation and consolidated prototype evaluation summary. | `P3-AN-01`, `P3-SA-01` | Gives team and future contributors the supported product story and next decisions. | States what the prototype demonstrates and does not demonstrate; findings tie to measured artifacts; metrics and terminology agree with `INTERFACES.md`; platform scalability claims are labeled with tested workload and environment. |

## Phase 4 — Packaging, benchmark, and handoff

**Gate:** A reviewer can reproduce the demo and benchmark from documented instructions and identify exactly what hardware/software was measured.

| ID | Owner | Deliverable | Depends on | Handoff | Acceptance criteria |
| --- | --- | --- | --- | --- | --- |
| P4-AN-01 | Anas | Packaged policy artifact (ONNX if supported by the selected stack) and edge inference benchmark. | `P2-SA-01`, `P3-AN-01` | Gives Aman a versioned artifact and benchmark record for the demo. | Records model/version, input shape, runtime, device/CPU, software versions, warm-up and sample count, p50/p95 latency, memory where available, and numerical/action agreement against the reference; identifies CPU-only emulation as a proxy and makes no flight-readiness claim. |
| P4-AM-01 | Aman | Reproducible demo/run instructions, integrated artifact/version record, and platform-level scalability characterization. | `P2-AM-01`, `P3-AM-01`, `P4-AN-01`, `P4-SA-01` | Delivers the reviewable v1 product slice to the team. | Fresh contributor can configure, run, and inspect a documented scenario; tested scale, bottlenecks, deployment assumptions, and unimplemented platform needs are explicit; no real integration or operational use is implied. |
| P4-SA-01 | Salman | Final method and assumptions note linked to the shipped policy/filter artifacts. | `P3-SA-01`, `P4-AN-01` | Supplies the mathematical caveats alongside the final demo and results. | Exact formulation, artifact/config version, model assumptions, feasible-control conditions, and limitations are recorded; any unresolved proof obligation is visible. |

## Phase 5 — Define the first real service

This is the next product-planning phase after v1. It turns the prototype findings into a scoped real-service specification and an evidence-based delivery plan. It does **not** authorize live integrations, flight operation, or deployment by itself.

**Gate:** The team has selected a first user workflow and operating domain; specified the service’s role, data sources, integrations, reliability/security/support requirements, assurance obligations, and staged validation evidence; and approved a delivery plan for implementation. If decisions depend on customer, partner, or regulator input, record who must supply it and what remains unresolved.

| ID | Owner | Deliverable | Depends on | Handoff | Acceptance criteria |
| --- | --- | --- | --- | --- | --- |
| P5-AM-01 | Aman | First-service product definition and production architecture. | `P4-AM-01`, prototype evaluation, stakeholder/customer discovery | Gives Salman a bounded intended use for assurance analysis and Anas a target workflow/data/edge environment for validation planning. | Names the target user and workflow, operating domain, whether the service provides information, coordination, advice, or control influence, service boundaries, deployment model, authoritative data/integration needs, identity/access, audit, reliability/availability, support, and scale requirements. Records decisions, assumptions, and open stakeholder dependencies. |
| P5-SA-01 | Salman | Assurance and safety requirements for the selected service and operating domain. | `P5-AM-01`, `P3-SA-01`, and the selected service role | Gives Aman assurance requirements and gives Anas validation scenarios and evidence criteria. | Defines hazards, safety objectives, model/operating assumptions, assurance evidence, independent review needs, fallback expectations, runtime monitoring, and any unresolved external approval/standards questions. Makes no claim that the v1 filter or report alone satisfies these requirements. |
| P5-AN-01 | Anas | Validation and deployment transition plan from simulation toward the selected service context. | `P5-AM-01`, `P5-SA-01`, `P3-AN-01`, and `P4-AN-01` | Gives Aman implementation sequencing and Salman the proposed validation evidence path. | Maps simulator assumptions to the selected operating context; identifies required replay/shadow, integration, edge, hardware-in-loop or field evidence as applicable; defines scenario coverage, monitoring, failure/rollback criteria, and gates before each deployment stage. Labels evidence and dependencies that require partner or external approval. |

## Shared evaluation definitions

Use the terms and units in [`INTERFACES.md`](./INTERFACES.md). A separation violation is a **pair-step sample** whose modeled separation is below the v1 threshold; report the number of samples, not as a certified collision or aviation near-miss measure. Minimum separation is over all active modeled pairs during a run. Match episode demand, dynamics, seed, disturbance, and time limit across controller variants. Keep aborted, held, and incomplete missions in the accounting. Report distributions or uncertainty across runs; a single favorable run is not acceptance evidence.

## Ownership and change control

- Aman owns product/system architecture, ML system design, API/platform integration, and platform-level scalability.
- Salman owns MARL/ML algorithm research, mathematical foundations, CBF/HOCBF design, and assumptions behind safety claims.
- Anas owns simulator/scenarios, stress evaluation, and edge ML packaging/benchmarking.
- Phase 5 extends these responsibilities toward real-service requirements and validation planning; it does not imply that production integrations or operational approvals already exist.
- An owner may ask another owner for review, but each task above has one accountable owner. Consumers accept outputs through the stated handoff.
- Shared contracts are changed on a workstream branch and proposed directly to `main`; the pull request names affected producers/consumers and includes coordinated updates. There is no `dev` step.
- AI coding tools may assist implementation. The owner remains responsible for checking generated code, equations, units, experiment reproducibility, and claims against evidence.
