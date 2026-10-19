# Simulated UTM assurance prototype

This repository coordinates a three-person team building a **simulation-only prototype** for studying how multi-agent reinforcement learning (MARL) and a control barrier function (CBF/HOCBF) action filter can work together in dense drone traffic. It is a research and engineering prototype, not an operational UTM service, a flight controller, or a safety-certified system.

## The problem we are exploring

As more drones share constrained airspace, each vehicle's motion can affect the options available to nearby vehicles. A learned policy may improve mission progress or traffic flow, but a learned action alone does not establish that separation will be maintained. We want to build a repeatable simulation in which a policy proposes actions, a model-based filter can modify those actions, and matched experiments show what the filter changes—including when it cannot produce a feasible action.

The first product-shaped result is a **simulation/API demo**. A user should be able to configure synthetic airspace and missions, run a scenario, inspect the proposed and filtered actions, and compare the measured outcomes across controller variants. The API and registered vehicles in v1 are mock simulation interfaces; they do not connect to real UTM services, aircraft, or sensors.

## V1 workflow

```mermaid
flowchart LR
    A[Mock airspace rules and missions] --> B[Simulator state]
    B --> C[MARL policy proposes acceleration]
    C --> D[CBF/HOCBF filter checks modeled constraints]
    D -->|feasible| E[Filtered acceleration]
    D -->|solver error or infeasible| F[Zero-acceleration fallback]
    E --> G[Simulator advances]
    F --> G
    G --> H[Matched-run metrics and visualization]
    H --> B
```

The simulator owns ground-truth state. The policy receives the documented, deliberately simplified 18-value observation. The filter may alter the policy's proposed acceleration using the simulator's modeled state. If the filter fails or its optimization problem is infeasible, v1 emits a zero-acceleration command. That is an **unverified fallback**: it is not a hover controller and does not guarantee separation, especially when a drone already has velocity.

The shared simulation-only v1 data types, units, API routes, timing, logging, and error behavior are specified in [`INTERFACES.md`](./INTERFACES.md). That document is the contract for all branches. If code and contract differ, record the mismatch and resolve it through a change to `main` before treating either version as authoritative.

## What the prototype includes

- A deterministic, seeded multi-drone simulation with configurable scenarios and disturbances.
- Mock API routes for synthetic airspace rules, notices, and drone registration.
- A deterministic reference controller, an unshielded MARL policy, and the same policy with a CBF/HOCBF filter.
- An explicitly unverified zero-acceleration fallback for solver errors or infeasible filter problems.
- Matched scenario comparisons over multiple seeds, with completion, separation, intervention, infeasibility, fallback, and latency measurements.
- A demo visualization/logging path and a model packaging and edge inference benchmark. Benchmark hardware and runtime must be named in the result; a CPU proxy must be labeled as a proxy.

These are planned deliverables. They are not claims that working implementations, performance numbers, or safety results already exist.

## Long-term research direction

The supporting report describes a broader layered architecture: strategic UTM intent and reservation coordination; tactical MARL or trajectory planning; CBF/HOCBF action filtering; a recoverability supervisor informed by Hamilton–Jacobi reachability; deterministic or independently validated backup behavior; and state-integrity, communication-trust, and conformance monitoring. This is a research direction, not the v1 architecture. V1 does not implement the full report architecture or inherit its possible assurance properties by using a simplified CBF filter.

V1 excludes real sensor feeds, real USS/UTM integration, real drone registration, flight operation, hardware-in-the-loop, certification, MPC, HJ reachability, and a verified backup controller. A simulator chaos injector is a repeatable test input, not a defense against real attacks.

## Evaluation and evidence

Compare the deterministic reference, unshielded MARL, and shielded MARL on the same scenarios, traffic demand, dynamics, disturbance seeds, and episode limits. Report results over multiple paired runs and include uncertainty or run-to-run variation. Record at least mission completion and aborts, minimum pairwise separation, separation-violation samples, shield interventions, infeasible filter steps, fallback events, and simulation/policy latency. Do not use average reward as a safety measure.

Zero observed failures in a finite simulation campaign does not prove zero risk. A CBF/HOCBF safety claim is conditional on the model, initial conditions, control authority, uncertainty bounds, sample timing, and a feasible optimization problem. The v1 interfaces and fallback do not provide the report's full state-integrity or runtime-assurance architecture.

## Repository and ownership

`main` is the shared source of truth. The shared task index is [`TASKS.md`](./TASKS.md), and cross-branch contracts live in [`INTERFACES.md`](./INTERFACES.md). There is no `dev` documentation or integration step. Each workstream keeps its detailed owner brief on its named branch; its path is unique so a pull request does not replace this product README.

| Branch | Owner and responsibility | Owner brief |
| --- | --- | --- |
| `aman/infra-api` | **Aman** — product and system architecture, ML system design, integration, API/platform design, and platform-level scalability. The branch name understates this scope; it is not infrastructure work alone. | [`aman/README.md`](https://github.com/salmanbashashaik/drone-traffic-control/blob/aman/infra-api/aman/README.md) |
| `salman/cbf-safety` | **Salman** — MARL/ML algorithm research, mathematical foundations, CBF/HOCBF design, and the assumptions and limits behind safety claims. | [`salman/README.md`](https://github.com/salmanbashashaik/drone-traffic-control/blob/salman/cbf-safety/salman/README.md) |
| `anas/marl-policy` | **Anas** — simulator and scenario design, stress evaluation, and edge ML packaging and benchmarking. The branch name understates this scope; simulation and edge evaluation are part of the ownership. | [`anas/README.md`](https://github.com/salmanbashashaik/drone-traffic-control/blob/anas/marl-policy/anas/README.md) |

Each workstream sends pull requests directly to `main`. Before using an AI coding agent, read this README, [`TASKS.md`](./TASKS.md), [`INTERFACES.md`](./INTERFACES.md), and the owner brief on your branch. Keep changes within the assigned workstream, use the shared contract, identify assumptions, and summarize changed files, evidence, open issues, and handoff needs in the pull request. A change to a shared contract requires review from the affected owners and an update to every affected consumer.

## Supporting research

The [26-page report](./TLDR_Enterprise_UTM_MARL_Formal_Safety.1.pdf) is background research for the long-term direction. It synthesizes public material and proposed methods; it is not a product specification, implementation record, vendor performance validation, or report of new simulation, benchmark, or flight experiments. Its vendor analysis does not demonstrate this prototype's capabilities.

The report discusses CBF/HOCBF, MARL, reachability, and layered runtime assurance with explicit assumptions and open limitations. Read it as context for design decisions, while using the shared v1 contract and task plan to build the current prototype.
