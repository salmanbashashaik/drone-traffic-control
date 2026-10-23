# Shared task plan

This is the **sole shared task index** for the startup’s product path, from the v1 prototype through definition of the first real service. It assigns one directly responsible owner per task, records dependencies and handoffs, and defines phase gates. Detailed implementation notes stay in the owner's branch brief; do not copy this plan to workstream branches. All tasks are planned until their deliverables and evidence exist. There are no calendar-date commitments in this plan.

## How to use this plan

1. Start with the root [`README.md`](./README.md) for startup product direction and [`INTERFACES.md`](./INTERFACES.md) for simulation-only v1 contracts.
2. Work from the matching owner brief: [Aman](https://github.com/salmanbashashaik/drone-traffic-control/blob/aman/product-platform/docs/workstreams/aman.md), [Salman](https://github.com/salmanbashashaik/drone-traffic-control/blob/salman/marl-safety/docs/workstreams/salman.md), or [Anas](https://github.com/salmanbashashaik/drone-traffic-control/blob/anas/simulation-edge/docs/workstreams/anas.md).
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

## Backlog — before any real-drone test

These items come from Sprint 1 review. Each must be closed before any test on a real drone.

| ID | Owner | Item | Found in | Acceptance criteria |
| --- | --- | --- | --- | --- |
| BL-01 | Aman | **Done in PR #12.** The evasive fallback must respect world bounds. It could command a drone through the floor, ceiling or a world edge; it now clips its manoeuvre to per-axis world-box limits, and the QP has the same limits as hard bounds. Tests: `tests/test_filter_bounds_vertical.py`. | Sprint 1 PR A | A fallback action never drives a drone outside the world box within its stopping distance; a test places a drone near each bound and checks it. |
| BL-02 | TBD | The evasive fallback must account for third neighbours. Today each agent escapes only from its most critical neighbour and may move toward another. | Sprint 1 PR A | In a three-drone cluster the fallback does not reduce the separation of any other pair below `d_eff` when an alternative exists; a test covers it. |
| BL-03 | Salman | The backup-set barrier assumes a different evasive manoeuvre for each pair, so a drone with several neighbours is credited with escapes it cannot fly at the same time. Design a consistent per-drone backup: one manoeuvre per drone, shared by all its pairs. | Sprint 1 PR B2 | Every backup row of a drone is computed from the same backup trajectory of that drone; a test with one drone between two neighbours checks that the trajectory used in both rows is identical and that it keeps both pairs outside `d_eff` in closed loop. |
| BL-04 | Salman | The backup-set barrier constrains only the closest-approach step k* of each backup trajectory. A QP step can lower the margin at a step next to k* that is not constrained. Constrain a small window of steps around the minimum, or prove that one step is enough. | Sprint 1 PR B2 | Either the rows cover a window around k* with a test showing the windowed minimum never falls below the constrained value, or the formulation note contains a proof that the single-step row bounds the trajectory minimum, with a test that checks it on random pairs. |
| BL-05 | Salman | The backup barrier models actuator lag only in the horizon length, not in the constraint rows or the world barrier. Fix before any real-drone test. | PR #12 founder review | The backup constraint rows and the world barrier both include the actuator lag, with a test that flies the lagged dynamics and shows neither the pair margin nor the world box is violated. |
| BL-06 | Salman | The safety filter has no disturbance margin. It plans with the commanded acceleration only, ignores wind (up to 1.0 m/s² per axis in T2), takes the noisy, delayed states it receives as exact, and assumes every drone receives its command. Under T2 the shielded variant can lose separation. Fix in the C+D filter PR. | Sprint 1 PR F | Every row of `scripts/disturbance_matrix.py` (every shipped scenario plus the no-wind symmetric crossing, both T2 profiles, seeds 0–4) keeps d_safe with zero fallbacks. Those rows then become tests. Also covers the lateral reversal at σ = 1.5 m with the backup barrier in `tests/test_filter_bounds_vertical.py` (strict xfail): the latch is oriented from one noisy state; tracked state (UTM-012) should remove them. Baseline: `docs/results/bl06-baseline-2026-10-21.md`. |
| BL-07 | TBD | The world-box barrier has no wind or lag budget. `_world_accel_limits` assumes the commanded acceleration is flown exactly, so a drone braking against a wall under wind (up to 1.0 m/s² per axis in T2) or actuator lag can overshoot the wall standoff. Part of the disturbance margin in BL-06. | PR #12 re-review | A drone at maximum speed towards each wall, with constant 1.0 m/s² wind towards that wall and the T2 observation delay, never leaves the world box; a test covers each wall. |
| BL-08 | TBD | Four hard-fuzz starts with 5–6 drones entered d_eff although they were recoverable: seeds 3675, 1177, 3958 and 4828 (`HARD=1 python scripts/verify/fuzz.py 5000 backup`; recoverability from `scripts/verify/recov.py`). With the colouring-oriented latch, 3675 and 3958 keep d_eff; 1177 (4.77 m) and 4828 (5.37 m) still enter it. The likely cause is the pairwise backup of BL-03. | PR #12 re-review | Each of the four seeds keeps d_eff in `scripts/verify/fuzz.py` with `HARD=1`, and they become regression tests once BL-03 is done. |
| BL-09 | TBD | Parallel fuzz runs write their log lines from several processes at once, so lines interleave and the output is garbled. Collect the results in the parent process and print them in order. Fix in PR C. | PR #12 re-review | `scripts/verify/fuzz.py` with 8 processes prints one complete line per run, in seed order. |
| BL-10 | TBD | The seeded latch colouring colours seeded pairs before nearer unseeded ones. In a bent sandwich a far outer pair that latched first from its predicted miss fixes the colours, and the squeezed drone gets contradictory sideways directions (re-review probes: 15 of 15 runs at bends 10°, 20° and 45°, worst 5.993 m, never inside d_eff). Founder-accepted in PR #12; superseded by BL-03, close with it. | PR #12 B1 re-review (HIGH-1) | `test_bent_staged_sandwich_ends_with_consistent_latches` in `tests/test_filter_latch.py` passes and its strict xfail is removed. |
| BL-11 | TBD | `_escape_axes` normalises the escape-axis sign by the largest component, so the axes flip where the line of sight crosses x = ±y. The 45° staged sandwich dropped from 6.169 m (489a2a0) to 5.751 m in PR #12 (founder-accepted regression). Make the normalisation continuous. | PR #12 B1 re-review (MEDIUM-1) | Staged sandwiches at bearings 0–90° in 5° steps all keep d_eff with consistent latches, and the 45° case reaches at least 6.169 m; a parametrised test covers it. |
| BL-12 | TBD | Small findings of the PR #12 B1 re-review. LOW-1: `INTERFACES.md` names only the colouring as the trigger that mirrors a turn-right latch; a clear predicted miss can trigger it too. LOW-2: `_two_colour` silently ignores a seed whose drones are already coloured in one component. LOW-3: `_escape_axes` divides by zero and returns NaN axes for coincident positions. | PR #12 B1 re-review (LOW-1 to LOW-3) | The text matches the code; an ignored seed is counted in the diagnostics; coincident positions give finite axes, with a test for each. |

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
