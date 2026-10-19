# Simulator and Scenario Note (P0-AN-01) - v1, simulation-only

| Field | Value |
|---|---|
| Task | P0-AN-01 |
| Owner | Anas (branch `anas/simulation-edge`) |
| Status | DRAFT for review by Salman and Aman. Nothing described here is implemented yet. |
| Date | 2026-10-19 |
| Authority | `INTERFACES.md` and `shared/contracts.py` on `main`. This note only fills gaps those documents leave open. |

## 1. Purpose and scope

This note defines how the v1 simulator behaves, so that the policy and safety filter (Salman) and the run adapter and mock API (Aman) can be built against a stable interface.

v1 is a **generic, simulation-only baseline**. Drones are point masses in a synthetic world. The simulator is the source of modeled ground truth. It is not a real sensor model, flight stack, or aircraft model. Sensing models (for example detection range) are future extensions and are listed in Section 10.

## 2. Frames, units and time

- Local Cartesian coordinates in meters, `[x, y, z]`. Velocity in m/s. Acceleration in m/s^2.
- World bounds, speed limit, acceleration limit and step size are those fixed in `shared/contracts.py`. They are imported from that module and not redefined here.
- Reset produces the state at t = 0 ms. Step k produces the state at t = 100 * k ms.
- An episode truncates at `ScenarioConfig.episode_limit_steps` (at most 1000).

## 3. Ground truth and delivered data

- **Ground truth:** the simulator-owned `DroneState` of each active agent. The safety filter reads it.
- **Delivered to the policy:** `ObservationV1` only (18 values), built from ground truth. An observation-layer disturbance (Section 7) may alter or withhold it.
- The policy has no other view of the world. Observation construction is isolated in one function so that future sensing models can replace it without changing the dynamics.

## 4. Dynamics (proposed)

For each active agent, with applied acceleration `a`, wind acceleration `w` (zero when no wind is configured) and step `dt = 0.1 s`:

```
a_eff  = a + w
p_next = p + v*dt + 0.5*a_eff*dt^2
v_next = v + a_eff*dt
```

- This is exact for constant acceleration over one step and matches a discrete double integrator.
- **Speed cap:** if `|v_next|` exceeds the contract speed limit, `v_next` is scaled to the limit and the event is logged. This is a nonlinearity outside a plain double-integrator model (see Open Item 1).
- The applied acceleration is `SafeAction.safe_accel_mps2`. For the reference and unshielded variants it is the proposed action passed through.

## 5. Step lifecycle

1. **Validate inputs.** Exactly one action per active agent; known and unique agent IDs; timestamp equal to the current state time; finite values; within acceleration bounds. Invalid input is rejected and never silently clipped.
2. **Apply dynamics** (Section 4).
3. **Bounds check.** If the new position leaves the world box, the agent is aborted (Section 6).
4. **Goal check.** If the agent is within `goal_radius_m` of its goal, it is completed (Section 6).
5. **Separation.** Compute 3D separation over all active pairs on the new state and update metrics.
6. **Outputs.** Build `observations`, `rewards`, `terminations`, `truncations` and `infos`, keyed by active `agent_id`.

The step result and `infos` fields follow `INTERFACES.md` exactly.

## 6. Agent status and episode end

- **Active:** launched and not completed, aborted or held.
- **Completed or aborted:** the agent leaves the active set. It is excluded from neighbor selection and pair counts afterward but stays in the mission accounting.
- **Abort reason (proposed):** `out_of_bounds`, logged per agent. Positions are never clipped back into the world.
- **Held:** reserved for later use. No v1 trigger is defined.
- `goal_radius_m` is a simulator constant, recorded in the run manifest and not a `ScenarioConfig` field. **Placeholder value: 2.0 m, to be agreed.**
- The episode ends when no active agents remain (termination) or the step limit is reached (truncation).
- Nearest-neighbor selection for `ObservationV1` follows `INTERFACES.md`: smallest 3D distance among active agents, ties broken by lexicographic `agent_id`.

## 7. Seeds and disturbances

**Seeding.** A single `numpy` `SeedSequence` is created from `ScenarioConfig.seed`. Each injector receives its own spawned stream, so adding or removing one injector does not change another. Initial conditions come from `ScenarioConfig` and are already deterministic. The seed governs disturbances and any generated scenarios. The same code version, configuration and seed must reproduce the same initial conditions and the same transition sequence.

**Injectors.** Each injector is seeded, configured, logged, and attached at one named layer. Injectors are repeatable test conditions. They are not defenses against real attacks.

| Injector | Layer | Effect |
|---|---|---|
| Bounded wind | Dynamics | Adds `w` to the acceleration, each component bounded by `w_max` |
| Observation delay or drop | Observation delivery | The policy receives an older observation or none; ground truth is unchanged |
| Corrupted record | One synthetic observation | One named record is altered |

A corrupted observation must still pass `ObservationV1` validation (see Open Item 6).

## 8. Contract gap

`ScenarioConfig` has no disturbance fields and forbids extra fields, so injectors cannot currently be configured.

**Proposal:** add an optional field that defaults to an empty tuple, so existing producers remain valid:

```
disturbances: tuple[DisturbanceSpec, ...] = ()
```

This requires one pull request to `main` that updates `INTERFACES.md` and `shared/contracts.py` together. Affected parties: Anas (scenario producer), Aman (run configuration), Salman (wind bound assumptions).

## 9. Evaluation outputs (summary)

The simulator produces a per-step record and one episode summary with the fields defined in `INTERFACES.md`:

- **Mission accounting:** `n_launched`, `n_completed`, `n_aborted`, `n_held`, `completion_pct`. Aborted, held and incomplete missions stay in the accounting.
- **Separation:** `min_separation_m` and `separation_violation_pair_steps` (pair-step samples below `d_safe`; not a certified collision count).
- **Filter behavior:** `shield_interventions`, `qp_infeasible_steps`, `fallback_events`.
- **Timing:** mean and p95 simulator step time and policy latency, kept separate.

A run manifest records: code or artifact version, `scenario_id`, seed, controller variant, episode limit, `goal_radius_m`, `w_max` (if any) and the injector list.

## 10. Future extensions (not part of v1)

**Detection layer (optional, proposed for discussion).** In v1 every drone effectively observes its nearest neighbor with no range limit and no identity or broadcast model. A later extension could add a detection layer between ground truth and the observation, with a detection range, cooperative (broadcasting) and non-cooperative drones, and dropout. This would change the meaning of `ObservationV1` indices 9-15 and require a contract change.

Key question for Salman: should the safety filter keep using ground truth for all pairs, or only detected ones? Using information the drones do not have would make the safety result look better than it is.

## 11. What this note does not claim

- No sensor fidelity, aircraft realism, attack defense, or field readiness.
- The zero-acceleration fallback is unverified. It is not a hover controller and does not guarantee separation.
- Zero observed violations in a finite simulation campaign do not demonstrate zero risk.

## 12. Open items

| # | Item | Needs an answer from | Blocks |
|---|---|---|---|
| 1 | Is the speed cap (scale velocity to the limit) acceptable for the CBF/HOCBF model, or is another treatment needed? | Salman | Dynamics code |
| 2 | Wind acceleration bound `w_max` | Salman | Wind injector |
| 3 | Reward definition for training | Salman (assumed) | Training, not the step interface |
| 4 | Does the runner pass one `SafeAction` per active agent per step? | Aman | Step interface |
| 5 | Value of `goal_radius_m` | All | Completion accounting |
| 6 | Rule for corrupted observations that must still pass validation | Salman, Aman | Corruption injector |
| 7 | Add the optional `disturbances` field to `ScenarioConfig` | All | Injectors |
| 8 | Add `numpy` to `pyproject.toml` and `requirements.txt` | All | Seeded random streams |
| 9 | Abort (not clip) agents that leave the world box | All | Bounds handling |