# Salman’s workstream — MARL methods and CBF/HOCBF safety

This brief belongs to the `salman/cbf-safety` branch. **Salman owns MARL/ML algorithm research, mathematical foundations, CBF/HOCBF design, and the assumptions and limits behind safety claims.** The startup is building toward real UTM services; the simulation/API prototype is its first engineering milestone. The shared root [README](../README.md) describes the product path. The shared task index is [`TASKS.md` on main](https://github.com/salmanbashashaik/drone-traffic-control/blob/main/TASKS.md), and v1 simulation units, state/action fields, timing, and logging semantics live in [`INTERFACES.md` on main](https://github.com/salmanbashashaik/drone-traffic-control/blob/main/INTERFACES.md).

The supporting report motivates a broader layered research direction; it does not report new prototype experiments or prove that the proposed architecture works. V1 studies a learned nominal controller and a model-based filter inside a simulator. Its observation is simplified, its model and disturbance coverage are limited, and its zero-acceleration fallback is unverified. Keep every mathematical or safety statement conditional on the actual model, controller implementation, solver behavior, and experiment evidence. Your post-prototype work helps determine what assurance evidence a selected real service will need; the simulation filter alone does not provide that assurance.

## Ownership boundary

You own the learning method and mathematical safety-filter work: algorithm selection and formulation, reward/objective design, policy training and artifact metadata, deterministic reference behavior, CBF/HOCBF derivation, solver constraints, feasibility behavior, and review of claims made from results. Anas owns simulator transitions, scenario generation, stress execution, and edge packaging/benchmarking. Aman owns product/system architecture, mock API and orchestration, end-to-end integration, and platform-level scalability.

Coordinate equations against Anas’s implemented state transition and the shared action bounds. Do not silently choose different dynamics, observation fields, timestamp rules, or separation thresholds. A proposed shared-contract change goes by pull request directly to `main`, with affected consumers named. There is no `dev` step, and this branch must not copy the shared task index.

## Your tasks

All tasks are planned and map to your rows in [`TASKS.md`](https://github.com/salmanbashashaik/drone-traffic-control/blob/main/TASKS.md). This brief expands the inputs, outputs, dependencies, handoffs, and acceptance conditions for this workstream.

### P0-SA-01 — Mathematical formulation and assumptions

**Inputs:** V1 world, state/action units, timestamp and separation definitions in `INTERFACES.md`; Anas’s proposed transition scheme and uncertainty/disturbance model.

**Deliver:** A concise formulation note defining the controlled state, nominal policy action, admissible acceleration set, pairwise separation barrier, chosen first- or higher-order barrier construction, and safety-filter optimization objective. A candidate geometric barrier is `h_ij = ||p_i - p_j||² - d_safe²`, where `d_safe` is the simulation threshold. For relative-degree-two position separation under acceleration control, evaluate a HOCBF formulation rather than treating a first-order position constraint as automatically enforceable. State every modeling and timing assumption, expected infeasible cases, numerical tolerance, and unsupported claim.

**Dependencies:** Shared interface contract and Anas’s intended dynamics. If the simulator does not meet a mathematical assumption, document and resolve the mismatch with Anas before implementation.

**Handoff:** Give Anas solver variables, constraints, inputs, numerical tolerances, and expected failure reporting; give Aman plain-language limitations for product messaging.

**Acceptance:** The note defines units and notation consistently with `INTERFACES.md`, identifies control authority, initial-condition, model, uncertainty, sample-time, and feasibility assumptions, and distinguishes a nominal barrier condition from a proven sampled-data guarantee. It does not claim that use of CBF/HOCBF alone certifies the simulator or a vehicle.

### P1-SA-01 — Deterministic reference and policy adapter

**Inputs:** Anas’s working environment and 18-value observation; shared action schema and bounds; P0 formulation.

**Deliver:** A deterministic reference policy suitable for matched comparisons and a documented adapter between `ObservationV1`, policy output, and `MARLAction`. Record preprocessing and normalization parameters, policy input/output shapes, model/config versioning, and the policy’s role as a nominal action proposer.

**Dependencies:** P0-SA-01 and P1-AN-01. Confirm that the environment’s reset, step, and timestamp behavior matches the adapter assumptions.

**Handoff:** Give Anas the reference controller for evaluation and give Aman the policy runtime boundary and required model metadata.

**Acceptance:** With a fixed configuration and seed, the reference emits the same valid, finite, bounded actions; adapter output meets the shared schema and timestamp contract; preprocessing is reproducible and versioned; no policy output is labeled inherently safe.

### P2-SA-01 — MARL method and safety-filter artifact

**Inputs:** P0 formulation, Anas’s seeded environment/scenarios, P1 adapter/reference, and agreed evaluation metrics.

**Deliver:** A reproducible MARL training/evaluation configuration and versioned policy artifact; a CBF/HOCBF filter implementation or implementation-ready package with solver/version/config recorded; diagnostics for proposed versus returned actions, barrier values, feasibility, and errors. Explain reward/objective terms, training seeds, and any normalization or curriculum choices.

**Dependencies:** P0-SA-01, P1-AN-01, and P1-SA-01. Coordinate filter invocation and batch semantics with Anas before integration.

**Handoff:** Give Anas loadable policy/filter artifacts, exact observation/action adapters, solver settings, and failure semantics. Give Aman the minimal invocation and metadata needed for the demo.

**Acceptance:** The artifact and configuration can be identified and reproduced; actions obey shared bounds; the filter reports feasible/infeasible/error state and diagnostics rather than silently masking failures; infeasibility and solver exceptions follow the shared zero-acceleration fallback adapter semantics; assumptions and known limitations accompany the artifact. Training reward is not used as a safety metric.

### P3-SA-01 — Interpretation and safety-assumption review

**Inputs:** Anas’s raw summaries, manifests, paired-run tables, stress scenarios, and failure records; P0 formulation and artifact versions.

**Deliver:** A technical interpretation of policy/filter behavior, a review of whether the experiments exercised stated assumptions, and a claim review for the consolidated evaluation.

**Dependencies:** P3-AN-01 and P0-SA-01. Request missing run metadata rather than inferring unrecorded conditions.

**Handoff:** Give Aman evidence-backed language and a list of unresolved mathematical or experimental limits; give Anas any metric or interpretation correction needed before reporting.

**Acceptance:** Clearly separates empirical results from analytical statements; accounts for filter modifications, infeasible steps, fallbacks, and observed violations; records assumption mismatches and evidence gaps; explicitly states that no failures in finite simulation do not establish zero risk.

### P4-SA-01 — Final method and limitation note

**Inputs:** Final policy/filter artifacts, benchmark metadata, reviewed evaluation, and any implementation deviations from the formulation.

**Deliver:** A version-linked method note with exact artifact/config identifiers, state/action mapping, equations, solver behavior, relevant assumptions, known failure modes, and unresolved proof obligations.

**Dependencies:** P3-SA-01, P4-AN-01, and implementation artifacts from P2-SA-01.

**Handoff:** Give Aman and the team an accurate technical appendix for the reproducible demo and future research planning.

**Acceptance:** A reader can tell which formulation and versions were actually used, what was measured, which guarantees are absent, and what would need additional analysis or evidence. No unimplemented HJ, recoverability, verified-backup, sensor-integrity, or certification feature is implied.

### P5-SA-01 — Assurance requirements for the first real service

**Inputs:** Aman’s selected service role, operating domain, system boundaries, and data/integration map; v1 evaluation and failure records; applicable technical and operational requirements identified during product discovery.

**Deliver:** A service-specific hazard and assurance requirements note plus an evidence roadmap. Map hazards and assumptions across the service boundary; define safety objectives, uncertainty/model limits, monitoring and escalation needs, fallback requirements, independent review, and evidence needed before each proposed deployment stage. Identify which standards, authorities, and external approvals need qualified follow-up without claiming compliance.

**Dependencies:** P5-AM-01 and P3-SA-01. The assurance analysis depends on knowing whether the service provides information, coordination, advice, or control influence.

**Handoff:** Give Aman requirements that shape production architecture and product claims; give Anas scenario coverage and validation evidence criteria for simulation, replay, shadow operation, and any later hardware or field stages.

**Acceptance:** Each identified hazard and safety objective is linked to assumptions, proposed controls/monitoring, and evidence or an explicit open item. Prototype evidence is kept distinct from service evidence. The note identifies independent review and external dependencies, states where the prototype formulation is insufficient, and makes no certification, operational approval, or zero-risk claim.

## Working with AI tools

- Provide the agent the relevant task ID, this brief, current `main/INTERFACES.md`, and the exact simulator transition code before asking it to derive or alter a constraint.
- Ask it to show definitions, units, relative-state conventions, assumptions, and a derivation step by step. Verify equations independently and check them against the implemented discretization; generated algebra is not evidence of a guarantee.
- Ask for primary-source citations when the agent makes a literature claim. Distinguish report content, published results, proposed formulations, and this repository’s measured results.
- For service assurance planning, use AI to surface hazards, assumptions, and evidence questions for review. Validate them against the selected service, operating domain, and qualified external requirements; do not treat a generated hazard list or assurance argument as an approved safety case.
- Do not let generated code suppress solver errors, hide infeasibility, modify timestamps, or silently clip actions. Preserve logs that allow the nominal policy and filter output to be compared.
- Review every use of “safe,” “guaranteed,” “verified,” “recoverable,” and “robust” in code, plots, demo text, and PR descriptions. Open pull requests directly to `main`, list task IDs and artifacts, and identify all consumer handoffs.

## AI Starter Prompt (Copy to Cursor)

```text
You are Salman’s mathematical methods and safety-filter research assistant for this startup repository. Work only on P0-SA-01 and P1-SA-01 from the shared main/TASKS.md plan: the mathematical formulation/assumptions note, deterministic reference behavior, and the ObservationV1-to-MARLAction policy adapter. The CBF/HOCBF filter artifact belongs to Phase 2; do not silently expand Phase 1 into an unreviewed filter implementation.

Before editing, read the root README.md, main/TASKS.md, main/INTERFACES.md, shared/contracts.py, Anas’s simulator transition/scenario proposal, and this workstream README. The mathematical formulation must match the transition Anas actually proposes and the shared state/action units.

Before writing code or equations, state:
1. The fixed contract facts and every modeling/math assumption you need.
2. The state, relative-state sign convention, control variables, sample timing, bounds, candidate barrier, objective, and failure cases involved.
3. The files and derivation/implementation steps you propose, mapped to P0-SA-01 or P1-SA-01 acceptance criteria.

Then proceed under those explicit assumptions. If a required dynamics detail, uncertainty bound, initial-condition condition, solver tolerance, or control-authority assumption is missing, mark it unresolved and ask for the owner decision; do not fill it with a plausible-sounding value or claim a guarantee. Show equations step by step with units and check the relative-state convention against the simulator.

Contract rules:
- Import and use the canonical Pydantic v2 models from shared/contracts.py: DroneState for modeled state, ObservationV1 for policy input, and MARLAction for nominal policy output. Use SafeAction only when discussing the filter output contract. Never define duplicate or branch-local Pydantic schemas.
- Do not alter observation indices, action fields, units, timestamps, bounds, fallback semantics, or the shared 1e-4 m/s² intervention tolerance locally. If the mathematical review requires a different tolerance or contract, propose a coordinated change to main/INTERFACES.md and shared/contracts.py and identify affected consumers.
- For P0-SA-01, document the candidate h_ij = ||p_i - p_j||² - d_safe² only as a starting point. Analyze the position-separation relative degree under acceleration control; do not treat a first-order position condition as automatically enforceable. State sampled-data, feasibility, model, initial-state, control-bound, and uncertainty assumptions and limitations.
- For P1-SA-01, provide a deterministic reproducible reference controller and a clearly nominal policy adapter that validates the exact shared observation/action models. Reject invalid output visibly; do not silently clip it or label policy output safe.
- Do not claim CBF/HOCBF use proves real-world safety, and do not claim HJ reachability, recoverability supervision, verified backup behavior, or certification exists in v1.

Before finishing, summarize definitions and assumptions, changed files, evidence or checks, unresolved questions, and the exact handoff Anas needs to implement the simulator/filter boundary and Aman needs for accurate product messaging. Keep published claims distinct from this repository’s measurements.
```
