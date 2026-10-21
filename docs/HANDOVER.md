# Technical Handover

Prepared 2026-10-21 for the incoming CTO. Repository state: `main` at `b63d8fe` (PR #8 merged), clean working tree.

Rules followed while writing this: every claim points to a file or to a command I ran in this session. Anything I could not verify is marked **unverified**, and anything I inferred is marked **guess**. No code was changed to produce this document. All probe scripts were run from a temporary scratch directory outside the repository.

## Sprint 1 status: make the safety core honest

Sections 1–9 below describe `main` at `b63d8fe`, before Sprint 1. This table tracks what each Sprint 1 PR changed. Where a PR supersedes a finding below, the finding is left in place as the baseline and this table says what is now true.

| PR | Scope | Status | Result |
| --- | --- | --- | --- |
| A | Always-feasible QP, evasive per-cluster fallback | Done, PR open on `aman/sprint1-a-safe-fallback` | See below. |
| B | Bounded barrier inside `d_eff`, violation metrics in metres, 3-axis escape, crossing liveness, lateral-escape analysis | Done, PR open on `aman/sprint1-b-barrier` (stacked on A) | See below. |
| B2 | Backup-set barrier from the evasive manoeuvre; default since contract v1.3 | Done, PR open on `aman/sprint1-b2-backup-barrier` (stacked on B) | See below. |
| C | Default range gate, spatial pair search, QP split, N=100 infeasibility root cause, benchmark, `polish=True` latency | Not started | — |
| D | Honest `h_min`, geofence constraints, `n_held`, unused exception | Not started | — |
| E | `utm-run` CLI, scenario files, replay plots | Not started | — |
| F | Disturbance tier T2 and degraded command handling | Not started | — |
| G | Naming, docs, ruff, Linux PyBullet CI, empty folders | Not started | — |

**PR A — what is now true.**

- Every pair constraint has a non-negative slack variable with penalty weight 1e4 (`FilterConfig.slack_weight`). The QP is always feasible. Violated barriers are reported per pair (`FilterDiagnostics.pair_slack_mps2`) and per episode (`total_slack_mps2`, `max_slack_mps2`, `slack_pair_steps`; replaced in PR B by metres and seconds).
- The fallback runs only on a solver error. It applies only to the conflict cluster, not the fleet, and it is evasive: brake away from the most critical neighbour plus lateral and vertical escape, largest component at ±3.5 m/s². This replaces section 4's "Fallback" paragraph and risks 1 and section 7's "fallback does not brake" finding. Contract bumped to v1.1 in `INTERFACES.md` (fallback acceleration may be non-zero).
- OSQP is now pinned to `adaptive_rho_interval=25` and `polish=True`. Before the pin, the same seed gave different results run to run, because OSQP's default picks the rho interval from wall-clock setup time. The crossing scenario hit a solver error on some runs only. `test_same_seed_runs_are_identical` guards this.
- Regression results (`python -m pytest tests/test_filter_safety_core.py -v -s`, 15 m/s per drone, same altitude, reference controller, shielded):

| Scenario | Min separation | Fallback events |
| --- | --- | --- |
| Head-on, 40 m gap | 6.60 m | 0 |
| Head-on, 50 m gap | 10.46 m | 0 |
| Head-on, 60 m gap | 13.98 m | 0 |
| Head-on, 80 m gap | 10.13 m | 0 |
| 90° crossing, both 50 m from the crossing point | 5.46 m | 0 |
| 3-drone sandwich (10 m/s each side, 10 m gaps) | 0.61 m | 0 |

- The sandwich is physically unrecoverable: any controller within ±3.5 m/s² per axis ends at most 4.45 m apart at t = 0.8 s. The proof is in the docstring of `test_three_drone_sandwich_is_physically_unrecoverable`. The filter reaches 0.61 m, well below that bound, partly because it never uses z here (max |a_z| = 0.0 over the first 20 steps). PR B should address this.

Open issues found in PR A, and where they stand after PR B:

- Fixed in PR B: inside `d_eff` the clamped square root made slack 10³–10⁵ m/s².
- Fixed in PR B: the 90° crossing stall (see below).
- Still open, now in `TASKS.md` as BL-01 and BL-02: the evasive fallback ignores world bounds and third neighbours. Both must be fixed before any real-drone test.

**PR B — what is now true.**

- **Barrier blow-up.** The old row contained `2u ρ / sqrt(4u(d − d_eff))`. With the root clamped at 1e-3 that is `7000 ρ`, about 7 × 10⁴ m/s² at 10 m/s closing, and the slack had to absorb it. The barrier now uses `sqrt(4u s + c²) − c` with a linear extension below `d_eff` (`c = barrier_offset_mps = 1 m/s`). It is continuous with a continuous slope, and its slope is at most 7 s⁻¹, so the row is bounded. Cost: about 4.3 m more standoff at 30 m/s closing. Derivation in `docs/formulation/P2-SA-02-lateral-escape.md` §1.
- **Metrics.** Slack totals are replaced by violation depth `d_eff − d` in metres and time inside `d_eff` in seconds, per pair and per episode, for every controller variant. Contract bumped to v1.2 in `INTERFACES.md`.
- **Sandwich.** The 4.45 m bound already included z. It is not achievable: the best found over all three pairs is 4.168 m. A new escape stage re-solves pairs that the braking barrier cannot hold, with a predicted-separation constraint in metres using all three axes. Minimum separation went from 0.61 m to 3.91 m, with no fallback.
- **Crossing.** The stall was a side-by-side lock, not chatter: the braking rows deflect each drone toward the other's path, and the 0.5 m/s² tie-break was too weak to break the symmetry quickly. With tie-break 0.5: 323 steps against 296 unconstrained (+9.1 %), 3.6 s below 6 m, two sign flips of lateral acceleration. With tie-break 0 it deadlocks. The default is now 1.0 m/s²: 307 steps (+3.7 %), minimum separation 5.81 m. `test_crossing_takes_at_most_x_percent_longer_than_unconstrained` fails above +5 %; the justification for 5 % is in the test.
- **Lateral escape analysis.** For a 30 m/s head-on the braking-only barrier needs 69.8 m; braking plus a y/z split needs 28.9 m. A collision-cone CBF is proposed for evaluation, with a backup-set CBF as the alternative. Doc only, awaiting review: `docs/formulation/P2-SA-02-lateral-escape.md`.
- **Note for PR C.** Measure what `polish=True` (pinned in PR A) adds to step latency.

Regression results after PR B (`python -m pytest tests/ -q -s`):

| Scenario | Min separation | Max violation depth | Time inside `d_eff` | Fallback events |
| --- | --- | --- | --- | --- |
| Head-on, 40 m gap | 5.58 m | 0 m | 0 s | 0 |
| Head-on, 50 m gap | 6.16 m | 0 m | 0 s | 0 |
| Head-on, 60 m gap | 6.50 m | 0 m | 0 s | 0 |
| Head-on, 80 m gap | 14.57 m | 0 m | 0 s | 0 |
| 90° crossing | 5.81 m | 0 m | 0 s | 0 |
| 3-drone sandwich | 3.91 m | 1.59 m | 1.8 s | 0 |

Head-on minimum separations are lower than in PR A (6.60 m to 5.58 m at the 40 m gap) but stay above `d_eff`. **Cause: the escape stage.** At 30 m/s closing a 40 m gap is inside the braking curve (69.8 m), so the first solve leaves slack and the escape stage runs. Its constraint asks for predicted separation `≥ d_eff` at closest approach, so the pair passes just outside 5.5 m instead of braking harder; the episode is 4 steps shorter (296 against 300). Isolated by toggling each change on the 40 m head-on:

| `tie_break_mps2` | `barrier_offset_mps` | Escape stage | Min separation |
| --- | --- | --- | --- |
| 0.5 | 0.001 | off | 6.60 m |
| 0.5 | 0.001 | on | 5.58 m |
| 0.5 | 1.0 | off | 6.61 m |
| 0.5 | 1.0 | on | 5.59 m |
| 1.0 | 0.001 | off | 6.98 m |
| 1.0 | 0.001 | on | 5.57 m |
| 1.0 | 1.0 | off | 7.00 m |
| 1.0 | 1.0 | on | 5.58 m |

Neither the tie-break change nor the offset/linear extension lowers it; the tie-break raises it.

**Escape-stage latency** (wall-clock per call, this Windows laptop, 5 seeds each, random traffic from `tests/test_random_traffic.py`):

| Run | First solve p50 / p95 | Escape solve p50 / p95 (calls) | Whole `apply_filter` p50 / p95 / max |
| --- | --- | --- | --- |
| N=16, drones start at rest | 5.2 / 9.3 ms | never triggered (0 of 1560 steps) | 5.4 / 9.7 / 25.6 ms |
| N=16, start at 15 m/s toward goal, plus one sandwich triple | 4.4 / 8.7 ms | 8.0 / 11.0 ms (50) | 4.5 / 9.5 / 21.6 ms |
| N=64, drones start at rest | 44.4 / 220.9 ms | never triggered (0 of 1768 steps) | 45.5 / 223.1 / 322.4 ms |
| N=64, start at 15 m/s toward goal | 35.3 / 230.2 ms | 290.8 / 351.0 ms (25) | 36.3 / 232.5 / 737.6 ms |

At N=64 the braking baseline already misses the 100 ms step at p95, and a step that needs the escape solve takes up to 738 ms. This is PR C's problem (dense pair search, one joint QP).

**PR B2 — what is now true.**

- **Default barrier.** `FilterConfig.barrier` selects `"backup"` (default) or `"braking"` (kept for one release). The backup barrier simulates each pair flying the fallback's evasive manoeuvre for a horizon derived from the limits (6.6 s) and keeps that escape outside `d_eff` from the next state. The fallback and the barrier now share one function. Contract v1.3 in `INTERFACES.md`; full write-up in `docs/formulation/P2-SA-02-lateral-escape.md` §5.
- **Results.** The head-ons and the crossing gain separation (40 m head-on 5.58 → 7.09 m, crossing 5.81 → 8.13 m) and lose path overhead. The sandwich is unchanged at 3.91 m. At N = 64 the p95 step time is 165 ms, against 218–235 ms for braking, with no fallbacks and no time inside `d_eff`.
- **Open, must be fixed before any real flight.** BL-03 (per-pair backups can be inconsistent for a drone with several neighbours) and BL-04 (only the closest-approach step is constrained), in `TASKS.md`.

---

## 1. Product understanding

### What the product is

The product is an Unmanned Traffic Management (UTM) service for drone fleets. A central service watches every drone in a shared volume of airspace and tells each drone what to do so that the drones reach their goals without coming too close to each other. Sources: `README.md`, `docs/architecture/PRODUCT_AND_INTEGRATION.md`, and the founder's notes.

The founder's framing is "we are like air traffic control": drones have their own autopilots and we send them guidance. The research background is in `TLDR_Enterprise_UTM_MARL_Formal_Safety.1.pdf` (a 26-page report). It is context, not a specification.

### Who uses it

The intended user is an operator of a fleet of drones that share airspace. The founder has stated that all drones in scope are **ours and cooperative**: there are no intruder aircraft, no non-cooperative traffic, and no static obstacles other than geofences. There is no named customer, and no customer-facing interface exists in the code.

### What problem it solves

When many drones fly in the same volume, someone has to keep them separated. The thesis of the project is that a learned multi-agent policy (MARL) can propose efficient motions, and a mathematical safety filter (a control barrier function solved as a quadratic program) can correct any proposal that would break minimum separation. The filter supplies the safety guarantee, and the policy supplies efficiency.

### What "the drone must follow our guidance" means in the current design

In v1, our output **is** the drone's command. Each step (100 ms), the service sends each drone an acceleration vector in local Cartesian metres (`SafeAction.safe_accel_mps2` in `shared/contracts.py`). The simulator applies that acceleration directly (`src/utm_services/simulation/env.py`). There is no negotiation, no acknowledgement, and no option for the drone to refuse. The founder resolved this explicitly: drones obey our output.

This means we are not giving advisories in the ATC sense. We are the outer-loop controller. The drone's own autopilot only tracks our acceleration command. In the PyBullet backend this is literal: a PID controller tracks our acceleration as a feed-forward term.

### Who avoids collisions

**Our server does, alone.** The safety filter in `src/utm_services/safety/filter.py` runs centrally. It reads every drone's true position and velocity and solves one joint optimisation over all drones. The drone does no collision avoidance of its own in the design or in the code. The reference policy (`src/utm_services/policy/reference.py`) ignores other drones entirely, so in the current system the filter is the only thing that keeps drones apart.

A consequence: if the link between the server and a drone fails, nothing keeps that drone separated. Section 7 covers this.

---

## 2. The plan

### Where the plan comes from

The only roadmap is `TASKS.md` in the repository. It defines phases P0 to P5 with tasks assigned to three owners. It has **no dates**. `README.md` also has a coarser four-stage path (V1 prototype, then service definition, then integration/limited deployment, then operational). Owner briefs are in `docs/workstreams/{aman,anas,salman}.md`.

| Phase | Content (from `TASKS.md`) | Status (my assessment from code) |
|---|---|---|
| P0 | Formulation (P0-SA-01), simulator note, product boundaries (P0-AM-01) | Done. Docs exist in `docs/formulation/`, `docs/simulation/`, `docs/architecture/`. |
| P1 | Simulator, mock API, reference policy, MARL method and reward design | Mostly done. Simulator (tier T1 only), mock API and reference policy exist. The MARL method doc `P1-SA-01-method.md` is still marked DRAFT. |
| P2 | P2-SA-01 trained MARL plus filter; P2-AN-01 runner variants; P2-AM-01 integrated demo, results view and per-step inspection | Partial. The filter and the runner with three variants exist. **No MARL is trained.** No results view or inspection UI exists. |
| P3 | Matched evaluation over at least 10 seeds | Not started. `src/utm_services/evaluation/` contains only `__init__.py`. |
| P4 | ONNX edge benchmark, reproducible demo | Not started. |
| P5 | Define the first real service | Not started. |

### Which milestone we are on

**We are in P2.** The safety-filter half and the runner half of P2 are merged. The MARL half and the demo/results-view half are not built.

The order of merged work (from git history and the founder's notes): PR #1 contracts, #2 mock API, #3 scenario adapter, #4 reference policy, #5 OSQP filter, #6 braking-distance CBF plus `DroneEnv` and runner, #7 end-to-end random-traffic test plus CI, #8 optional PyBullet backend.

Still open according to `TASKS.md`, the simulator note and the founder: trained MARL policy, disturbance contract and injectors (simulator tiers T2 to T5), P3 evaluation, P4 ONNX, a Plotly Dash results UI, a run logger with persistence, and a CLI or entry point for runs.

---

## 3. What is built

### How I ran it

```
python --version                         -> Python 3.12.3
python -m pytest -v                      -> 83 passed, 1 skipped in 10.34s
python -m ruff check . --statistics      -> 17 errors
utm-mock-api  (then curl)                -> see the API row below
python <scratch>/probe.py variants|scale|failures   -> see below and section 7
```

Installed versions: osqp 0.6.7.post3, fastapi 0.118.0, pydantic 2.11.10, numpy 1.26.4. CI (`.github/workflows/tests.yml`) runs Python 3.11.

There is **no command in the repository that runs a simulation**. There is no CLI, no script and no notebook. The only ways to run an episode are the tests and the PyBullet GUI demo. I ran episodes by calling `run_episode` from a scratch script.

### Component inventory

| Component | Path | What it does | Status | How verified |
|---|---|---|---|---|
| Contracts | `shared/contracts.py` (401 lines) | Pydantic v2 models, strict and frozen, rejecting extra fields and inf/nan. Holds the world constants: x,y in [0,1000] m, z in [30,200] m, max speed 15 m/s, ±3.5 m/s² per axis, 100 ms step, ≤1000 steps, 5 m separation. Models: `DroneState`, `ObservationV1` (18 floats), `MARLAction`, `SafeAction`, `ScenarioConfig`, registration, geofence, altitude limits, NOTAM and error models. | Working | `tests/test_contracts.py` passes. |
| Mock API | `src/utm_services/api/app.py` (224 lines), script `utm-mock-api` | FastAPI app on 127.0.0.1:8000. `POST /drone/register`, `GET /airspace/rules`, `GET /airspace/notam`. | Working as a **mock** | Ran it. `GET /airspace/rules` returned 200 with the hardcoded geofence. `GET /airspace/notam` returned 200 `{"notams":[]}`. `POST /drone/register` for `d1` returned 201 `{"registered":true,"sim_registration_id":"sim-reg-8b53639f152c"}`. A repeat returned 409 `duplicate_agent_id`. Missing fields returned 422 `invalid_request`. `GET /nope` returned 404 `not_found`. `tests/test_api.py` passes. |
| Scenario adapter | `src/utm_services/application/scenario_adapter.py` (27 lines) | Validates a scenario payload and resets a simulator from it. | Working (thin) | Covered indirectly by the tests. |
| Policy adapter | `src/utm_services/policy/adapter.py` | Validates the observation, calls the policy, validates the action, and checks the agent id and timestamp. | Working | `tests/test_reference_policy.py` passes. |
| Reference policy | `src/utm_services/policy/reference.py` | PD go-to-goal controller: Kp = 0.5, Kd = 2·√0.5 ≈ 1.414 (critically damped). It clips the command so every axis stays within ±3.5 m/s². **It ignores all other drones.** | Working | Tests pass. All drones completed in my runs. |
| MARL policy | none | Planned MAPPO with CTDE via CleanRL (`docs/formulation/P1-SA-01-method.md`, DRAFT). | **Not implemented** | No training code, model files or inference code exist. |
| Safety filter | `src/utm_services/safety/filter.py` (489 lines) | Joint QP over all drones that enforces a braking-distance CBF per pair. Solved by OSQP. Falls back to zero acceleration when the QP fails. Details in section 4. | Working in nominal conditions; **breaks inside its stated envelope** (section 7) | `tests/test_filter_qp.py`, `test_filter_closed_loop.py`, `test_filter_stub.py` and `test_random_traffic.py` pass. Probe runs below. |
| Simulator (point mass) | `src/utm_services/simulation/env.py` (`DroneEnv`, `sim-v1.0`) | Double-integrator point masses, trapezoid integration, speed cap at 15 m/s, goal radius 2 m, out-of-bounds abort. Separation is checked after each step. Geofences are **logged, not enforced**. | Working for tier T1 only | `tests/test_simulation_env.py` passes. Probe runs below. |
| Runner | `src/utm_services/simulation/runner.py` | `run_episode(config, variant, ...)`. Variants: `reference` (policy only), `unshielded_marl` (policy only), `shielded_marl` (policy plus filter). Returns a manifest, a summary and optional per-step records with latency. | Working, **in memory only** | Probe runs below. |
| PyBullet backend | `src/utm_services/simulation/pybullet_env.py` (`PyBulletDroneEnv`, `sim-pybullet-v0`) | Crazyflie CF2X model with the DSL PID controller at 240 Hz, 24 substeps per step. Tracks our acceleration as feed-forward. Aborts on collision or speed-envelope breach. Has a GUI demo: `python -m utm_services.simulation.pybullet_env head-on-100\|corner-swap <variant>`. | **Unverified** | It cannot be installed on Windows. Its test was skipped: `tests\test_pybullet_env.py:8: could not import 'pybullet'`. It works only on Linux/WSL. CI installs only `.[dev]`, so CI skips this test too. |
| Evaluation package | `src/utm_services/evaluation/__init__.py` | Nothing. | **Empty** | Read the directory. |
| CI | `.github/workflows/tests.yml` | `pip install -e ".[dev]"` then `pytest -q` on PRs and on pushes to `main`. | Working | Read the file. The founder's notes say PR #7 and #8 passed CI. I did not open the GitHub run logs. |
| Empty folders | `aman/`, `anas/`, `salman/` at the repository root | Nothing; untracked. | Clutter | `ls`. |

### Episode results I produced

16 drones, seed 0, starts and goals placed at random in a 400 × 400 m box at 80–120 m altitude, at least 30 m apart. Run with the mock API's default rules, which include the geofence at (500, 500):

| Variant | Completed | Min separation | Interventions | Infeasible steps | Fallbacks | Geofence incursion agent-steps | Mean / p95 step |
|---|---|---|---|---|---|---|---|
| `reference` | 16/16 | 10.97 m | 0 | 0 | 0 | 269 | 1.25 / 2.36 ms |
| `unshielded_marl` | 16/16 | 10.97 m | 0 | 0 | 0 | 269 | 1.25 / 2.35 ms |
| `shielded_marl` | 16/16 | 11.84 m | 229 | 0 | 0 | 291 | 3.07 / 6.52 ms |

Each run takes 348 steps. With `keep_records=True` the runner returns 348 records with keys `controller_variant, episode_id, infos, proposed_actions, returned_actions, scenario_id, seed, states, timestamp_ms`.

### Things that look real but are faked, hardcoded or simulated

1. **The "MARL" variants contain no MARL.** `unshielded_marl` and `shielded_marl` run the reference PD controller, labelled `policy_id="reference-standin"` in the tests. The table above proves it: `reference` and `unshielded_marl` produce identical numbers. Any result labelled "MARL" from this repository is a result for the PD controller.
2. **The airspace rules are hardcoded.** `_default_rules()` in `app.py` returns one cylinder geofence, `restricted-1`, at (500, 500) with radius 50 m, valid from 0 to 100000 ms. `_default_notams()` returns an empty list. There is no data source.
3. **Registration is in memory.** The registration ID is `sim-reg-` plus the first 12 hex characters of sha256(agent_id). It is deterministic, not issued. Registrations are lost on restart. There is no authentication.
4. **The API is not connected to the control loop.** The runner takes `rules` as a function argument. Nothing in the run loop calls the API. Only the PyBullet demo imports `_default_rules`.
5. **Geofences do nothing.** They are counted (`geofence_incursion_agent_steps`) but never avoided or enforced. In the table above, drones flew through the restricted zone for 269–291 agent-steps.
6. **The "drone" is a function call.** There is no network link, telemetry ingestion or message protocol. Drone state is read directly from the simulator object every step.
7. **The seed changes nothing.** Tier T1 has no randomness, so `ScenarioConfig.seed` does not affect a run.
8. **`n_held` is always 0.** It is a field in the summary that nothing increments.
9. **`ScenarioDataUnavailable`** is defined in `app.py` but never raised.
10. **The term "HOCBF".** The code and docs call the filter a higher-order CBF. The barrier is relative degree 1 (a braking-distance CBF), not a higher-order one. Function names like `build_hocbf_qp_matrices` and `_solve_hocbf_qp` keep the old name.
11. **A test file name is stale.** `tests/test_filter_stub.py` now tests the real filter, but some test names still say "stub".
12. **The docs are stale.** `README.md` and `docs/architecture/HIGH_LEVEL_DESIGN.md` still say the simulator, the real QP solve and the run orchestrator are "planned", and describe a "pass-through safety-filter stub". All three now exist.

---

## 4. Architecture as it stands

### Data flow

Everything runs in one Python process. There is no network hop in the control loop.

```
ScenarioConfig ──reset──► DroneEnv (or PyBulletDroneEnv)
                              │
         every 100 ms step:   │ observations: ObservationV1 per drone (18 floats:
                              │ own state, goal, nearest neighbour only)
                              ▼
                  run_policy(obs) ──► MARLAction (proposed accel) per drone
                              │
          variant=shielded:   ▼
                  apply_filter(all true states, all proposals)
                              │  one joint OSQP solve over 3N variables
                              ▼
                  SafeAction per drone (safe accel, flags)
                              │
                              ▼
                  env.step(actions) ──► integrate, check bounds/goal/separation
                              │
                              ▼
                  runner accumulates summary (+ records if asked), in memory
```

The filter reads **true states from the simulator**, not the observations. The policy sees only its nearest neighbour. The filter sees everyone.

The mock API is a separate process with no link to this loop.

### Tech stack and reasons

| Piece | Choice | Reason (from docs or commit history) |
|---|---|---|
| Language | Python ≥ 3.11 | ML and robotics ecosystem; the planned CleanRL training is Python. The reason is not written down. **Guess.** |
| Contracts | Pydantic v2, strict, frozen | `INTERFACES.md` makes the contracts the integration boundary between three owners. Strict validation catches bad data at every hand-off. |
| API | FastAPI + uvicorn | Mock of a future UTM service API. Pydantic integration. The reason is not written down. **Guess.** |
| QP solver | OSQP 0.6.7 + scipy sparse | Formulation doc P0-SA-01 §7: a sparse convex QP solved every step. OSQP is a standard real-time choice. |
| Simulator | Custom point-mass `DroneEnv`, PettingZoo-style parallel step | `docs/simulation/SIMULATOR_NOTE.md` (accepted): fast, deterministic tier T1 for policy and filter development. |
| Physics backend | gym-pybullet-drones, optional | Added by me (the AI assistant) in PR #8 to check the filter against a rigid-body quadrotor with a real attitude controller. The founder accepted it as optional. |
| RL (planned) | MAPPO, CTDE, CleanRL | `docs/formulation/P1-SA-01-method.md`, DRAFT. |
| UI (planned) | Plotly Dash | Recommended in the simulator note. Not built. |

### Data models

All are in `shared/contracts.py` and specified in `INTERFACES.md`.

- **Drone:** `DroneState`. Fields: agent id, position (m), velocity (m/s), and timestamp. Registration uses `DroneRegisterRequest` and `DroneRegisterResponse`. There is no model for drone type, capability, battery or operator.
- **Flight plan:** **none.** A drone has only a start position, start velocity and goal point (`ScenarioDroneConfig`). It has no waypoints, time windows, corridors or intent sharing.
- **Airspace:** `AirspaceRulesV1` (a list of cylinder `GeofenceV1` plus `AltitudeLimitsV1`), and `NotamV1` / `NotamListV1`. The API serves them. The simulator only logs them.
- **Telemetry:** **only `DroneState`.** There is no telemetry message, no sequence number, no quality or uncertainty field, and no latency field.
- **Observation:** `ObservationV1`, a root model of 18 floats. It holds own state, the goal, and the **nearest neighbour only**, with semantic checks.
- **Advisories:** there is no advisory model. `SafeAction` plays this role. Fields: `safe_accel_mps2`, `timestamp_ms`, `shield_active`, `h_min_m2`, `qp_feasible`, `fallback_used`. A validator enforces that `qp_feasible=False` implies `fallback_used=True` and that a fallback has zero acceleration.

### Conflict detection and resolution

All of this is in `src/utm_services/safety/filter.py`. The formulation is in `docs/formulation/P0-SA-01-formulation.md` §6–7.

**Detection.** Every step, the filter forms **every pair** of active drones. `range_gate_m` defaults to `None`, so all pairs are included and the work is O(N²). For each pair it builds one braking-distance barrier constraint. There is no separate "conflict" object. A conflict exists implicitly when the nominal action violates a constraint.

**Constraint (per pair i, j).** Definitions:

- `d` is the current separation.
- `n` is the unit vector from j to i.
- `ρ = n · v_rel` is the range rate.
- `u_max = 3.5` m/s².
- `d_eff = d_safe + margin = 5.0 + 0.5 = 5.5` m.

```
root = sqrt( max( 4·u_max·(d − d_eff − lag·max(−ρ, 0)), 1e-6 ) )
h    = root + ρ
n·(a_i − a_j) ≥ −γ·h − 2·u_max·ρ/root − (|v_rel|² − ρ²)/d
```

Read in words: the closing speed must stay below the speed from which both drones, braking together at full deceleration, can still stop before 5.5 m.

**Optimisation.** The filter minimises Σ‖a_i − (a_nom,i + bias_i)‖² subject to the box bounds |a_k| ≤ 3.5 on each axis and every pair constraint. It is one joint QP with 3N variables, solved by OSQP with `eps = 1e-5` and `max_iter = 4000`.

**Tie-break.** When a nominal action violates a pair constraint, both drones receive a lateral bias of 0.5 m/s². The bias turns them right about +z, or about +x for vertical pairs. This breaks the symmetry of head-on encounters.

**Parameters** (`FilterConfig` defaults):

| Parameter | Value |
|---|---|
| `gamma` | 1.0 |
| `margin_m` | 0.5 |
| `tie_break_mps2` | 1.0 |
| `lag_s` | 0.0 |
| `range_gate_m` | None (all pairs) |
| `solver_eps` | 1e-5 |
| `solver_max_iter` | 4000 |

**Fallback.** If OSQP's status is not `solved`, or it raises an exception, **every active drone** gets zero acceleration with `qp_feasible=False` and `fallback_used=True`. Zero acceleration means each drone keeps its current velocity. It does not brake. `INTERFACES.md` and the code both call this fallback unverified.

**Reported diagnostics.** `h_min_m2` reports the geometric barrier d² − 25, not the braking barrier used in the constraint. The other diagnostics are pair counts, feasibility, fallback and the number of interventions (intervention tolerance 1e-4 m/s²).

**Assumptions** (formulation §9): perfect knowledge of every state, a double-integrator model with no speed cap, no delay (`lag_s = 0`), and full cooperation. The stated safe envelope: a 30 m/s closing head-on needs roughly 70 m of initial gap.

---

## 5. Decisions made so far

| Decision | Reason | Source |
|---|---|---|
| Drones obey our output (we are the outer-loop controller, not an advisor) | Simplifies the v1 safety argument | Founder |
| All drones are ours and cooperative; no intruders; no static obstacles | Scope for v1 | Founder |
| v1 MVP supplies both the policy and the filter | Keeps the safety guarantee in our hands | Founder |
| Local Cartesian metres, 1000 × 1000 m world, z 30–200 m, 15 m/s, ±3.5 m/s², 100 ms step, 5 m separation | Contract for all three owners (`INTERFACES.md`) | Team (owners and P0 docs) |
| Centralised joint CBF-QP safety filter solved by OSQP | Formulation P0-SA-01 | Salman (MARL/safety owner) |
| Zero-acceleration fallback for all agents | Simple, deterministic first version; documented as unverified | Team (`INTERFACES.md`) |
| MAPPO with CTDE via CleanRL for the policy | P1-SA-01 method doc, DRAFT | Salman |
| Point-mass simulator first, tiers T1 to T5 | `SIMULATOR_NOTE.md`, accepted; open items answered by Salman on 2026-10-19 | Team |
| Plotly Dash for visualisation | Simulator note recommendation | Team (not built) |
| SANDO safe flight corridors only as a visual reference and future work | Formulation §13 | Founder |
| Replace the original HOCBF with a braking-distance CBF and add the tie-break | Response to the review of PR #5; the original filter did not hold separation in head-on tests | Me (AI assistant), PR #6, merged by the founder |
| Add an optional PyBullet quadrotor backend | Check the filter against rigid-body dynamics | Me, PR #8; founder accepted it as optional |
| Add the 16-drone random-traffic end-to-end test and run pytest in CI on every PR | Regression protection for the filter | Me, PR #7 |
| Do not run ruff in CI | `main` already had 17 lint errors; recorded as a known limit in the workflow file | Me, PR #7 |
| PRs merge directly to `main` with no release branch | Team practice | Founder |

---

## 6. Tests and quality

### Result

```
python -m pytest -v
83 passed, 1 skipped in 10.34s
SKIPPED tests\test_pybullet_env.py:8: could not import 'pybullet'
```

```
python -m ruff check . --statistics
8  I001    unsorted-imports
3  UP035   deprecated-import
3  F401    unused-import
3  RUF059  unused-unpacked-variable
Found 17 errors.
```

### What the tests cover

| File | Covers |
|---|---|
| `test_contracts.py` | Validation rules of every contract model: strictness, bounds, inf/nan, `SafeAction` invariants, unique ids. |
| `test_api.py` | Every mock route, plus error mapping (404, 409, 422). |
| `test_reference_policy.py` | Policy adapter validation; PD controller output bounds and direction. |
| `test_simulation_env.py` | Reset, integration, speed cap, goal completion, out-of-bounds abort, stale and missing action rejection. |
| `test_filter_qp.py` | QP matrix construction; QP infeasibility fallback with a 3-drone "sandwich" case. |
| `test_filter_stub.py` | Filter input validation and pass-through when no constraint is active. Test names still say "stub". |
| `test_filter_closed_loop.py` | 4-drone corner swap; head-on at 15 m/s each with initial gaps of 80, 100 and 150 m. Separation held. |
| `test_random_traffic.py` | 16 drones, 400 × 400 m, seeds 0–4. Shielded: min separation ≥ 5 m, zero fallbacks, all complete. Unshielded: loses separation. |
| `test_pybullet_env.py` | PyBullet backend. **Skipped here.** |

### What is not tested

- More than 16 drones in any test. My probes went to 100 drones (section 7).
- Head-on encounters **inside** the envelope (gap below about 70 m). My probe shows this fails (section 7).
- Any disturbance: wind, sensor noise, delay, dropped messages, or wrong state. None of these exist in the simulator.
- Timing against the 100 ms budget. No test asserts latency.
- The fallback being safe. The only fallback test checks that it triggers, not that it keeps drones apart.
- Geofence avoidance (it does not exist).
- The API and the control loop together (they are not connected).
- Multiple processes, persistence or restart.
- The PyBullet backend on this machine. **Unverified.**
- Lint is not enforced. Coverage is not measured.

---

## 7. Scale and safety

### How many drones it has run with

- The test suite uses at most 16 drones.
- I ran a scale sweep (shielded, reference policy, seed 0). The box grew with N so that density stayed comparable.

| N | Box (x,y) | Completed | Min separation | Interventions | Infeasible steps | Fallback events | Mean / p95 step | Wall time |
|---|---|---|---|---|---|---|---|---|
| 16 | 300–700 m | 16/16 | 11.84 m | 229 | 0 | 0 | 3.17 / 6.74 ms | 1.1 s |
| 32 | 200–800 m | 32/32 | 5.50 m | 1052 | 0 | 0 | 8.56 / 18.53 ms | 4.3 s |
| 64 | 100–900 m | 64/64 | 5.97 m | 3784 | 0 | 0 | 27.71 / 73.89 ms | 18.6 s |
| 100 | 50–950 m | 100/100 | 5.49 m | 9429 | **19** | **479** | **95.65 / 217.74 ms** | 67.2 s |

Filter cost alone for one step (all pairs, random positions in the full 1000 m world):

| N | Pairs in QP | Time |
|---|---|---|
| 16 | 120 | 4.7 ms |
| 64 | 2016 | 50.9 ms |
| 100 | 4950 | 127.3 ms |
| 200 | 19900 | 831.1 ms |

What this shows:

- **At about 100 drones on this laptop, the loop no longer fits the 100 ms step.** p95 is 218 ms. Cost grows roughly with the square of N because the range gate is off.
- At N = 100 the QP was declared primal infeasible on 19 consecutive steps (t = 49.4–51.2 s). Each time the fallback froze all 20–27 drones that were still flying. Separation held in that run (5.49 m minimum). OSQP reported infeasibility while h_min was still positive (about 5.1–5.25 m², geometric barrier). I do not know the cause. **Guess:** several pair constraints in a crowded goal cluster plus the box bounds become jointly infeasible, or OSQP hits a numerical limit. It needs investigation.
- These are single runs on one seed on a Windows laptop. They are not a benchmark.

### Head-on inside the envelope

Two drones at the same altitude, 40 m apart, each flying at 15 m/s toward the other (30 m/s closing). This is inside the roughly 70 m envelope the formulation states.

- The QP was infeasible from the first step (t = 0) to t = 1300 ms: **14 infeasible steps, 28 fallback events.**
- The fallback set zero acceleration, so both drones kept flying at full speed toward each other.
- **Minimum separation was 1.0 m. There were 4 separation-violation pair-steps.** In a real system this is a collision.
- The point-mass simulator does not abort on collision, so the two drones passed through each other. They ended `incomplete` after 300 steps.

This is the most important safety result in this document. **The fallback does not brake. When the filter cannot find a safe action, it keeps the current velocity, which is the worst choice in a closing encounter.**

### Link loss

There is no link in the code, so link loss cannot happen in the current system. What exists is:

- If one drone's command is missing from a step, `DroneEnv.step` raises `ValueError: actions must cover exactly the active agents ['d000', 'd001'], got ['d000']`. The whole episode stops. Verified by probe.
- There is no behaviour for a drone that loses contact. No hold, hover, return-to-home, or last-command timeout is designed or coded.
- Because the server is the only collision avoider, a drone that loses its link has no protection, and nor do the drones around it. **Guess:** in a real deployment this would be the dominant risk.

### Late telemetry

- An action with an old timestamp is rejected: `ValueError: stale action for 'd000': timestamp 0 ms, state time 100 ms`. The episode stops. Verified by probe.
- There is no late-telemetry handling: no extrapolation, no state uncertainty, and no inflated margin for old data. `FilterConfig.lag_s` exists as a margin term, but it defaults to 0 and nothing sets it.

### Server crash

- Everything is in one process and in memory. A crash loses the episode, all records and all registrations. There is no persistence, checkpoint, log file or replay.
- There is no redundancy and no hand-off to the drones. With the current design, a server crash would leave every drone without commands and without collision avoidance. Real drones would then do whatever their autopilot does on a lost link. That behaviour is not specified.

### Known limits

- The filter needs perfect, fresh state for every drone.
- The fallback freezes velocity. It is unsafe inside the braking envelope (verified above).
- Inside the braking envelope (about 70 m at 30 m/s closing), there is no guarantee.
- The cost is O(N²) per step. The 100 ms budget is exceeded around N = 100 on a laptop.
- Only tier T1 exists: no wind, noise, delay or drop.
- The point-mass model has no attitude dynamics. The PyBullet check is unverified on this machine.

---

## 8. Gaps, risks and debt

In rough order of what breaks first.

1. **The fallback is unsafe.** Zero acceleration keeps a closing pair closing. I reproduced a 1.0 m miss at a 40 m head-on start. The fallback applies to every flying drone at once, so one infeasible pair freezes the whole fleet's acceleration.
2. **There is no product yet, only a simulation.** There is no drone link, no telemetry protocol, no command protocol, no acknowledgement and no authentication. The API is a mock and is not connected to the loop. Everything sold as "UTM service" is currently a Python function call.
3. **There is no MARL.** The research thesis is "learned policy plus safety filter". Only the filter half exists. The "MARL" variants run a PD controller, so no result so far says anything about learned policies.
4. **Single point of failure.** One central process does all collision avoidance from perfect state. A crash, a network partition or a stale state removes all protection. There is no on-board fallback.
5. **Scale.** The all-pairs QP misses the 100 ms step at about 100 drones and shows unexplained infeasibility at that size. The range gate exists but is off by default and is not tested.
6. **There are no failure modes in the simulator.** There are no disturbance injectors and no T2–T5 tiers, and the seed changes nothing. Every passing test runs under ideal conditions.
7. **Late or missing data crashes the run.** A stale or missing action raises `ValueError`. There is no degraded mode.
8. **Geofences are not enforced.** In my 16-drone run, drones spent 269–291 agent-steps inside the restricted zone. Airspace rules are hardcoded.
9. **There are no flight plans.** There are only goal points. There is no strategic deconfliction, no intent and no time-based planning. Everything is tactical, 100 ms ahead.
10. **There is no persistence or tooling.** The runner keeps records in memory only. There is no run CLI, run logger, results view or replay. The `evaluation` package is empty, so P3 has nothing to build on.
11. **The docs are wrong in places.** `README.md` and `HIGH_LEVEL_DESIGN.md` describe a stub filter and a planned simulator. "HOCBF" is the wrong name. New readers will be misled.
12. **There is a gap between the filter and its diagnostics.** `h_min_m2` reports the geometric barrier, not the braking barrier used in the constraint, so a positive `h_min` does not mean the state is safe.
13. **Hygiene.** Ruff reports 17 errors and is not run in CI. Empty `aman/`, `anas/` and `salman/` folders sit at the root. Some test names are stale. The roadmap has no dates.
14. **Bus factor.** Anas (simulation/edge) is on exams, and the founder took over that work. Much of PRs #6–#8 was written by me (an AI assistant). The filter rewrite in particular has not been independently reviewed by the safety owner as far as I can tell (**guess**; I saw only the merge, not review comments).

---

## 9. Open questions

1. **What does a real drone integration look like?** Which autopilot (PX4, ArduPilot, a vendor SDK)? Which link (4G/5G, radio, MAVLink)? At what rate? Will the autopilot accept acceleration commands at 10 Hz? None of this is written down. **Guess:** the "drones obey our output" decision assumes a level of control access that real autopilots and regulators may not give.
2. **What should happen on link loss and on server loss?** This needs a decision, and it probably needs on-board avoidance. That would contradict the current design, where only the server avoids collisions.
3. **Is the target "advisory" or "command"?** The founder's ATC framing ("we send advice") and the resolved decision ("drones obey") point to different products with different regulatory paths. I took the code's behaviour (command) as the truth.
4. **Why is the N = 100 QP infeasible while h_min is positive?** Unknown. See section 7.
5. **What is the real fleet size and density target?** No document states one. This decides whether the O(N²) joint QP is acceptable.
6. **Is the safe-envelope figure (about 70 m for a 30 m/s head-on) agreed as a product requirement?** It is a property of the filter, not a stated requirement. Scenarios that start inside it are not prevented by anything.
7. **Who owns the MARL work, and when will it start?** `P1-SA-01-method.md` is still DRAFT. `TASKS.md` has no dates.
8. **Is Anas's GitHub account `skan06-md`?** It is unconfirmed in the founder's notes.
9. **The OI-7 disturbances contract PR** was agreed on 2026-10-19 but has not been opened. Who opens it?
10. **Does the PyBullet backend still pass on Linux?** I could not run it on Windows. CI installs only `.[dev]`, not the physics extras, so the PyBullet test is skipped in CI as well. **Nothing automated checks the PyBullet backend.**
11. **Who is the first customer, and what is the first real service (P5)?** It is not defined anywhere in the repository.
