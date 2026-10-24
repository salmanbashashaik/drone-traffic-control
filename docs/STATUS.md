# Project status

**Dated 2026-10-24.** Written from `main` at `dae9066` (PR #20 merged; PR #22 merged). Update this file when a lane starts or finishes, a backlog item closes, or the next step changes. `CLAUDE.md` holds the rules and the glossary; this file holds what is true now.

## Where we are

A simulation-only prototype of a central drone traffic service. A policy proposes accelerations, a model-based filter corrects them, and a point-mass simulator checks the result. No real aircraft or live data is involved.

Built and merged on `main`:

- Shared contracts (`shared/contracts.py`, `INTERFACES.md` v1.4) and a mock airspace API.
- Simulator `DroneEnv` with ideal (T1) and mild disturbance (T2) profiles; optional PyBullet quadrotor backend (Linux/WSL).
- The safety filter (`src/utm_services/safety/filter.py`): joint QP, backup-set barrier, latch, escape stage, world-box limits, evasive fallback on solver error, Numba rollout kernel (bit-exact with the old one).
- `utm-run` with three scenarios, MAPPO training and evaluation, two trained policy artifacts, 2D and 3D replay.
- Verification: `scripts/verify/` tiers (quick about 30 s, full about 20 min), fast suite 203 tests plus 91 slow tests, CI on every push and PR.

Evidence for the filter (simulation, `docs/results/pr12-final-report.md`): random 2-6 drone fuzz, 5000 runs: 0 runs inside `d_eff`, 0 fallbacks. Hard fuzz, 5000 runs: 13 inside `d_eff` (bar 14). These are measurements of the model under its assumptions. They are not a proof and not a flight claim.

Most important open finding (research/adaptive-margin, P3-SA-03, in-model): with realistic GNSS error, telemetry delay and actuator lag, main's filter as configured today lost separation in 81 of 141 calm-air runs (SBAS 37/47, GPS 37/47, RTK 7/47), while the tracker + adaptive-margin prototype lost none in 282 recoverable runs. Main is therefore not real-world safe until tracking, delay compensation and a disturbance margin are in the filter (BL-05, BL-06).

Known weak points (all in the backlog below): no disturbance margin (under T2 the shielded variant can lose separation), pairwise backups can be inconsistent for a drone with several neighbours, and the filter misses the 100 ms step budget near 64 drones (p95 2136 ms for a dense cluster at `apply_filter` alone, `docs/DECISIONS.md`, 2026-10-24).

## Active lanes

| Lane | Branch / worktree | State | Notes |
|---|---|---|---|
| Adaptive-margin research (P3-SA-03) | `research/adaptive-margin`, `../dtc-margin` | A run is active. Do not touch the worktree. | Margin computed each step from GNSS error, delay, wind and lag, plus an envelope monitor. In-model result: no separation loss in 282 recoverable in-limit runs. The monitor missed a doubled actuator lag. Open founder decisions in P3-SA-03 section 8. Nothing in `src/` changes yet. |
| Consistent backup (CBK) research (P3-SA-02) | `research/consistent-backup`, `../dtc-research` | Finished, recommendation "adapt, do not adopt". | Fails 3 of 4 acceptance bar items (worse than main in 15 cases, 2 of 20 noisy vertical runs enter `d_eff`, full CBK takes 273 ms at N = 16). Next: fix the hysteresis rule, then rerun. Feeds BL-03. |
| Scale | `perf/scale` (cloud session) | Active. | At 64 drones `osqp.solve` (45 %) and `_update_latches` (40 %) dominate (research scale run). The lane builds a standard benchmark, explains the 2136 ms p95, then tries per-cluster QPs, a braking-derived range gate, vectorised latch and matrix build, and OSQP reuse; each change must pass compare_results on the full tier. Target p95 < 100 ms at N = 64. |
| PX4 drone link | `integration/px4-bridge`, `../dtc-px4` | Demo done, PR open to main (not merged). | PX4 v1.17.0 SITL in Gazebo Harmonic: headon2 and cross3 flown live through the filter with 0 violation pair-steps (min separation 8.53 m and 7.14 m; loop p95 7.5 and 10.3 ms; idle machine). Link loss: PX4 flies RTL 1.12 s after the bridge exits, 1.27 s after the last fresh step, because MAVSDK re-sends the last setpoint until exit. Findings filed as BL-13 to BL-15. `docs/integration/PX4_SETUP.md`, design notes in `PROTOCOL_NOTES.md`. The production link still needs an ADR. |

## Open backlog

Source of truth: `TASKS.md`. BL-02 to BL-08 must close before any real-drone test.

| ID | Owner | Item | State |
|---|---|---|---|
| BL-02 | TBD | Fallback ignores third neighbours | Open |
| BL-03 | Salman | One consistent backup per drone | Open; CBK research feeds it |
| BL-04 | Salman | Backup barrier constrains only the closest-approach step | Open |
| BL-05 | Salman | Actuator lag missing from the rows and world barrier | Open |
| BL-06 | Salman | Disturbance margin (wind, noisy delayed states, lost commands) | Open; adaptive-margin research feeds it |
| BL-07 | TBD | World-box barrier has no wind or lag budget | Open; part of BL-06 |
| BL-08 | TBD | Four recoverable hard-fuzz starts enter `d_eff` | Open; likely fixed by BL-03 |
| BL-09 | Aman | Parallel fuzz log interleaving | Done (verification-speed PR 1) |
| BL-10 | TBD | Latch colouring order in a bent sandwich | Open; closes with BL-03 |
| BL-11 | TBD | Continuous escape-axis normalisation | Open |
| BL-12 | TBD | Small findings (text, ignored seed count, coincident positions) | Open |
| BL-13 | TBD | PX4 realises only part of a_safe (measured lag, SITL) | Open; with BL-05 and adaptive-margin research |
| BL-14 | TBD | Stale setpoint re-sent after link loss | Open; design in `PROTOCOL_NOTES.md` |
| BL-15 | TBD | Uncoordinated RTL after link loss | Open; design in `PROTOCOL_NOTES.md` §4 |

BL-01 is done (PR #12). Items marked TBD have no owner: assign them when the lane picks them up.

## What is next

1. Founder decisions pending: the open items in `docs/formulation/P3-SA-03-real-world-envelope.md` section 8 and whether to build the adapted CBK for BL-03.
2. Close BL-03 to BL-08 with the filter changes the two research lanes recommend, as safety PRs with the adversarial review and the full tier.
3. Write the ADR for the drone link (PX4 first): command or advice, what the drone does on link loss, onboard safety layer.
4. Scale the filter to the 100 ms budget at the fleet size the founder names (no target is set).
5. Later (`TASKS.md`): P3 matched evaluation of all variants over at least 10 seeds, P4 edge benchmark, P5 first real service definition.

## Housekeeping

- `docs/HANDOVER.md` is a snapshot from 2026-10-21. Do not read it as current state.
- Stale local branches left for the founder to decide (merged or closed, still on GitHub): `aman/product-platform`, `aman/sprint1-ef-cli-disturbance`, `aman/sprint1-safety-core`, `anas/simulation-edge`, `aman/sprint1-a-safe-fallback`, `aman/sprint1-b-barrier`, `aman/sprint1-b2-backup-barrier`, `aman/verify-speed-tooling` with worktree `../dtc-speed`.
