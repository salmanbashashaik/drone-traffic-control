# Salman’s workstream — MARL methods and CBF/HOCBF safety

This brief belongs to the `salman/cbf-safety` branch. **Salman owns MARL/ML algorithm research, mathematical foundations, CBF/HOCBF design, and the assumptions and limits behind safety claims.** The product is the simulation-only UTM assurance prototype described in the [root README](../README.md). The shared task index is [`TASKS.md` on main](https://github.com/salmanbashashaik/drone-traffic-control/blob/main/TASKS.md), and all cross-branch units, state/action fields, timing, and logging semantics live in [`INTERFACES.md` on main](https://github.com/salmanbashashaik/drone-traffic-control/blob/main/INTERFACES.md).

The supporting report motivates a broader layered research direction; it does not report new prototype experiments or prove that the proposed architecture works. V1 studies a learned nominal controller and a model-based filter inside a simulator. Its observation is simplified, its model and disturbance coverage are limited, and its zero-acceleration fallback is unverified. Keep every mathematical or safety statement conditional on the actual model, controller implementation, solver behavior, and experiment evidence.

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

## Working with AI tools

- Provide the agent the relevant task ID, this brief, current `main/INTERFACES.md`, and the exact simulator transition code before asking it to derive or alter a constraint.
- Ask it to show definitions, units, relative-state conventions, assumptions, and a derivation step by step. Verify equations independently and check them against the implemented discretization; generated algebra is not evidence of a guarantee.
- Ask for primary-source citations when the agent makes a literature claim. Distinguish report content, published results, proposed formulations, and this repository’s measured results.
- Do not let generated code suppress solver errors, hide infeasibility, modify timestamps, or silently clip actions. Preserve logs that allow the nominal policy and filter output to be compared.
- Review every use of “safe,” “guaranteed,” “verified,” “recoverable,” and “robust” in code, plots, demo text, and PR descriptions. Open pull requests directly to `main`, list task IDs and artifacts, and identify all consumer handoffs.
