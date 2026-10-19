# Repository structure and merge workflow

## One repository, three workstreams

`aman/product-platform`, `salman/marl-safety`, and `anas/simulation-edge` are **Git branches in this one repository**, not three separate repositories. `main` is the integrated source of truth. Workstream branches submit pull requests directly to `main`; do not merge one owner's branch into another owner's branch.

## Main code tree

```text
.
├── README.md                         # Product direction and system data flow
├── TASKS.md                          # Shared task index and phase gates
├── INTERFACES.md                     # Shared simulation/API v1 contracts
├── pyproject.toml                    # Package, runtime, and dev dependencies
├── shared/
│   └── contracts.py                  # Single canonical Pydantic contract module
├── src/utm_services/
│   ├── api/                          # Aman: HTTP boundaries and mock API
│   ├── application/                  # Aman: run orchestration and adapters
│   ├── simulation/                   # Anas: environment, dynamics, scenarios
│   ├── evaluation/                   # Anas: paired evaluation and edge tools
│   ├── policy/                       # Salman: reference/MARL policy adapters
│   └── safety/                       # Salman: CBF/HOCBF filter and assumptions
├── tests/                            # Tests mirror contracts and owned modules
├── docs/architecture/                # Shared system and repository architecture
│   ├── HIGH_LEVEL_DESIGN.md          # HLD: V1 boundaries and service evolution
│   ├── HIGH_LEVEL_DESIGN.drawio      # Editable architecture source
│   ├── HIGH_LEVEL_DESIGN.svg         # Rendered architecture diagram
│   ├── PRODUCT_AND_INTEGRATION.md    # Aman: V1 workflow proposal
│   └── REPOSITORY_STRUCTURE.md       # Tree, ownership, and merge workflow
└── docs/workstreams/                 # Owner briefs, one document per workstream
    ├── aman.md
    ├── salman.md
    └── anas.md
```

The package boundaries assign responsibility, not permission to invent local interfaces. All modules import shared types from `shared/contracts.py`. The API does not own simulator state transitions; the simulator does not own policy math; the policy does not own the safety filter; the application layer coordinates these components and records their outputs.

The v1 API and simulator remain explicitly local and simulation-only. This tree gives the startup a durable place to grow service code; it does not imply production integrations, live traffic use, operational approval, or production readiness. Add connectors, storage, identity/security, deployment, and reliability components only when the first real service and its requirements are selected in P5.

## Ownership by path

| Owner | Primary paths | Review/consumer boundary |
| --- | --- | --- |
| Aman | `src/utm_services/api/`, `src/utm_services/application/`, product architecture docs | Consumes simulator and policy/filter interfaces; owns local API, run integration, logs, and demo. |
| Salman | `src/utm_services/policy/`, `src/utm_services/safety/`, method notes | Consumes simulator observations/states; supplies bounded actions and filter results to the application and simulator. |
| Anas | `src/utm_services/simulation/`, `src/utm_services/evaluation/` | Supplies deterministic reset/step outputs to policy and integration; consumes controller adapters for matched evaluation. |
| Shared | `shared/contracts.py`, `INTERFACES.md`, root `TASKS.md` | Any change names affected producers/consumers and receives their review before merge. |

Tests for a component live in `tests/` and should follow the same ownership boundary. A change to a shared contract includes producer and consumer updates in the same reviewed integration sequence.

## Pull request and merge sequence

1. Start from the owner's named workstream branch and keep changes within the owner's paths, except for an explicitly coordinated shared-contract change.
2. Open a pull request from that workstream branch directly to `main`. Include task IDs, behavior/evidence, changed paths, interface impact, and the next owner handoff.
3. Keep `main` buildable. Merge one coherent task or phase slice at a time; do not merge unfinished work just to synchronize branches.
4. A shared contract pull request identifies every producer and consumer, updates `INTERFACES.md` and `shared/contracts.py` together, and receives review from affected owners. Merge the agreed contract before dependent components adopt it.
5. Typical v1 integration order is: shared contract decisions; Anas's simulator reset/step implementation; Salman's policy/filter adapters against the agreed state/action interface; Aman's end-to-end run integration; then Anas's matched evaluation and the team's result interpretation. P0 definition work can proceed in parallel.
6. After a pull request lands, sync the long-lived workstream branch from `origin/main` before the next integration slice. Do not force-push shared workstream branches or use another owner's branch as a merge target.

The branches are collaboration lanes, not release artifacts. A feature is part of the product only after its reviewed pull request lands on `main` with the necessary contract and evidence updates.
