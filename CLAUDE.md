# Project instructions

## Mission
- We build a neutral UTM (unmanned traffic management) service for India's low-altitude airspace: drones from many operators connect to it, and it approves their flights, monitors them live and guides them so they stay separated and organised. The v1 simulation assumes cooperative drones that all follow our guidance.
- Safety comes from a model-based filter that corrects any proposed motion (reference controller or learned MARL policy), backed by an onboard layer on each drone later.
- Today this is a simulation and mock-API prototype. No real aircraft, no live data, no flight use.

## Current state (2026-10-24; details in docs/STATUS.md)
- **Built (on main):** shared contracts (`shared/contracts.py`, `INTERFACES.md`, contract v1.4); mock airspace API; point-mass simulator `DroneEnv` with T1 (ideal) and T2 (wind, noise, delay, command dropout) disturbances; reference controller; the safety filter (backup-set barrier QP solved per conflict cluster to KKT accuracy with DAQP/OSQP, latch, escape stage, world-box limits, evasive fallback on solver error); `utm-run` CLI and three scenarios; MAPPO training (`utm-train`), two trained policy artifacts, 3D replay viewer; verification scripts and tiers in `scripts/verify/`; optional PyBullet quadrotor backend (Linux/WSL only).
- **Simulation only (all of it):** every safety result is a simulation measurement under stated assumptions, not a proof or a flight claim. The trained MARL policy only proposes actions; the filter decides. Geofences are counted, not enforced. Zero observed failures in finite runs is not zero risk.
- **Not built:** drone link, telemetry or command protocol, any autopilot integration (PX4 is the likely first); onboard safety layer; disturbance margin in the filter (BL-06, BL-07); consistent per-drone backup (BL-03); flight plans or strategic deconfliction; persistence and results UI; ONNX edge benchmark (P4); a defined first customer and service (P5). The filter meets the 100 ms step budget at 64 drones on a 4-core machine (p95 49-78 ms, `docs/results/scale-step3-2026-10-24.md`), not on one thread (101-124 ms), at 128 drones (182-234 ms), or for a single converging cluster of 64 (seconds; out of scope, strategic layer).
- Known limits accepted by the founder are listed in `TASKS.md` (backlog BL-01 to BL-12) and `docs/DECISIONS.md`. BL-02 to BL-08 must close before any real-drone test.

## Architecture today and likely direction
- **Today:** one Python process. `DroneEnv` (truth) -> observations -> policy proposes acceleration -> `apply_filter` (all true states, one joint QP) -> `SafeAction` -> `DroneEnv.step`, every 100 ms. The server is the only collision avoider, and the output is the drone's command.
- **Likely direction (not final, needs an ADR before any build):** the server does planning and guidance (routes, deconfliction, advice to each drone's autopilot over a link) and each drone runs an onboard safety layer that still works when the link or the server is lost. Open questions: autopilot and link (PX4/MAVLink first), advice or command, link-loss behaviour, fleet size. See `docs/HANDOVER.md` section 9.
- Packages in `src/utm_services/`: `api`, `application`, `simulation`, `policy`, `safety`, `training`, `evaluation`.

## Decision authority
Decide yourself, log it in docs/DECISIONS.md, and continue:
- implementation details, refactors, test structure, tooling, performance work that reproduces the per-seed results file exactly
- choices that follow an existing rule in this file or the spec

Stop and ask the founder only for:
- changing a safety claim, threshold, acceptance bar or the spec
- contract version changes
- any result that gets worse on a safety metric
- merging, force-pushing, deleting
- architecture decisions (ADRs)

Every safety PR gets an automatic adversarial review by a fresh subagent before it is reported as ready. The reviewer should use a different model from the author, preferably the strongest available (Fable for safety maths and safety PRs), because a reviewer that shares the author's model shares its blind spots.

## Literature first
Before designing any new safety mechanism, search the literature, then write down what exists and what is new, then design. Start with `docs/formulation/P3-SA-01-literature.md` (on branch `research/consistent-backup`; read it with `git show research/consistent-backup:docs/formulation/P3-SA-01-literature.md`). Cite sources in the design note, with what each proves and under which assumptions. A mechanism that a paper already solves is adapted, not reinvented.

## Verification
Verification is change-scoped: run the quick tier and targeted probes while iterating; run the full tier once per PR before merge; reviewers never rerun a tier that already ran on the same commit.
- quick tier: `python scripts/verify/tier.py quick`; full tier: `python scripts/verify/tier.py full`; `invariance.py` only on demand: `python scripts/verify/tier.py ondemand` (see `scripts/verify/README.md`)
- `pytest` runs the fast suite in parallel; `pytest -m slow` runs the slow suite (CI runs it on PRs to main)

## Install, run, test
Python 3.11 or newer. From the repo root:
```
python -m venv .venv && .venv\Scripts\Activate.ps1      # macOS/Linux: source .venv/bin/activate
python -m pip install -e ".[dev,plot,train]"            # extras: physics (Linux/WSL only)
```
- `utm-run scenarios/head-on-40.json --variant shielded_marl --plot --out runs/x`: run one scenario (variants: `reference`, `unshielded_marl`, `shielded_marl`; `--profile ideal|nominal|realistic` picks the disturbance profile). The MARL variants use the reference controller as a labelled stand-in.
- `utm-train --out runs/train1`: train a MAPPO policy (needs `train`).
- `utm-eval-policy artifacts/policies/<name> --out runs/eval1`: evaluate a policy artifact on the held-out suites.
- `utm-replay heldout:18000044:6 --artifact artifacts/policies/<name> --variant shielded_marl --html --out runs/viz`: 2D plot and 3D viewer (`replay3d.html`). Full walkthrough: `docs/training/QUICKSTART.md`.
- `utm-mock-api`: mock airspace API on 127.0.0.1:8000.
- Tests: `pytest` = fast suite (parallel, `-m 'not slow'`, 191 tests, 20-40 s). `pytest -m slow` = slow suite (fuzz, sweeps, long closed loops; CI runs it on PRs to main). `pytest -m ""` = everything. `-n0` = one process for debugging.
- Quick tier / full tier / on-demand: see Verification above. Quick is about 30 s, full about 20 min.
- Timing runs need an idle machine. Other Claude sessions share it.

## Glossary
- **d_safe, d_eff:** `d_safe` = 5 m, the required minimum separation. `d_eff` = `d_safe` + margin (margin default 0.5 m) = 5.5 m, the distance the filter actually enforces.
- **Backup-set barrier:** the filter's constraint. For each pair it simulates both drones flying an evasive manoeuvre (the "backup") and requires that escape to stay outside `d_eff`. Default since contract v1.3. It replaced the braking-distance barrier and the original "HOCBF" (the name is wrong: relative degree is 1). Code names like `build_hocbf_qp_matrices` still carry the old name.
- **Latch:** per-pair memory (required `latch` argument of `apply_filter`) that fixes which way a pair turns, so the escape direction does not flip step to step. Seeded colouring keeps a squeezed drone's latches consistent.
- **Escape stage:** a second solve for pairs the braking row cannot hold, using predicted separation in metres on all three axes.
- **CBK (consistent backup):** research design with one backup per drone chosen jointly with neighbours (answers BL-03). Prototype result: adapt, do not adopt. Lives on `research/consistent-backup`.
- **Adaptive margin:** research design that sets the margin every step from measured GNSS error, delay, wind and lag, plus an envelope monitor (P3-SA-03). Lives on `research/adaptive-margin`.
- **Tiers T1-T5:** disturbance levels of the simulator. T1 ideal (default); T2 mild wind and noise (implemented); T3 degraded comms; T4 stress; T5 real-world proxies (T3-T5 not built). `docs/simulation/SIMULATOR_NOTE.md`.
- **BL items:** backlog entries BL-01 to BL-12 in `TASKS.md`, issues to close before any real-drone test.
- **SafeAction:** the filter's return type. The name does not mean proven safe. **Fallback:** evasive action used only on a solver error, unverified.
- **P-codes:** task ids in `TASKS.md` (P0 formulation ... P5 first real service).

## Map of docs/
Current = keep true. Historical = a record of a past state, do not update, do not trust for the present.
| Path | Status |
|---|---|
| `CLAUDE.md`, `docs/STATUS.md`, `docs/DECISIONS.md` | current (the entry points) |
| `README.md`, `INTERFACES.md`, `TASKS.md` | current; INTERFACES.md is the contract |
| `docs/architecture/HIGH_LEVEL_DESIGN.md`, `REPOSITORY_STRUCTURE.md`, `PRODUCT_AND_INTEGRATION.md` | current (HLD updated 2026-10-24) |
| `docs/formulation/P2-SA-02-lateral-escape.md` | current (backup barrier, latch, escape) |
| `docs/formulation/P0-SA-01-formulation.md`, `P1-SA-01-method.md`, `P1-SA-01-reward.md` | partly historical: the original HOCBF design was replaced; use for notation and assumptions |
| `docs/simulation/SIMULATOR_NOTE.md` | current (tiers, seeding) |
| `docs/training/QUICKSTART.md`, `MARL_TRAINING.md` | current |
| `docs/results/` | historical records of dated runs; `solver-exact-2026-10-24.jsonl` is the per-seed reference of the safety filter (KKT-exact QP, founder-accepted 2026-10-24; `solver-exact-2026-10-24.md` has its better/worse against `pr12-b1-2026-10-23.jsonl`, the reference before) |
| `docs/results/reviews/` | historical adversarial reviews |
| `docs/HANDOVER.md` | historical (2026-10-21 snapshot of main before Sprint 1); its section 9 open questions are still useful |
| `docs/workstreams/*.md` | historical owner briefs |
| `research/*` branches | not on main: `research/consistent-backup` (P3-SA-01 literature, P3-SA-02 CBK), `research/adaptive-margin` (P3-SA-03) |
| `TLDR_Enterprise_UTM_MARL_Formal_Safety.1.pdf` | background research, not a spec |
