# P1-SA-01 — MARL Method and Observation Preprocessing Note

| Field | Value |
|---|---|
| Task | P1-SA-01 |
| Owner | Salman (`salman/marl-safety`) |
| Status | Implemented in P2-SA-01 (`src/utm_services/training/`). Deviations from this draft are in section 9; how to run it, and results, are in `docs/training/MARL_TRAINING.md`. |
| Date | 2026-10-20 |
| Authority | `INTERFACES.md` and `shared/contracts.py` on `main`. |

---

## 1. Purpose

This note records:
1. The chosen MARL algorithm family and why.
2. The `ObservationV1` → policy input preprocessing plan.
3. The policy output → `MARLAction` adapter contract.
4. Model versioning and artifact metadata requirements.

It does not describe the filter (P0-SA-01) or the reward function
(P1-SA-01-reward.md). It does not claim that the trained policy is safe.

---

## 2. MARL algorithm choice

### Chosen approach: MAPPO (Multi-Agent PPO)

**Algorithm:** Multi-Agent Proximal Policy Optimization with centralized
training and decentralized execution (CTDE).

- Each agent runs its own actor `π_θ(a | o_i)` at execution time using
  only its local `ObservationV1`.
- During training, a centralized critic `V_φ(o_1, ..., o_N)` has access
  to all agents' observations to reduce variance. At runtime only the
  actor is used — the critic is discarded after training.
- This matches the v1 architecture: the policy adapter takes one
  `ObservationV1` per agent and returns one `MARLAction`. No
  inter-agent communication at runtime.

### Why MAPPO over alternatives

| Algorithm | Considered | Verdict |
|---|---|---|
| MAPPO | ✅ | **Selected.** Strong empirical performance on cooperative tasks, CTDE matches our architecture, well-supported in PettingZoo environments, straightforward to implement and reproduce. |
| IPPO (Independent PPO) | ✅ | Simpler but higher variance — each agent treats others as part of the environment. MAPPO's shared critic gives better sample efficiency for cooperative separation. |
| QMIX | Considered | Designed for discrete action spaces. Our action space is continuous (3D acceleration). Not suitable without significant modification. |
| MADDPG | Considered | Works for continuous actions, but off-policy, harder to tune, and less stable than PPO variants on dense reward signals. |
| SAC-based multi-agent | Considered | Good sample efficiency but adds entropy tuning complexity. MAPPO is a more stable starting point for v1. |

**Reference:** Yu et al. (2022), "The Surprising Effectiveness of PPO in
Cooperative Multi-Agent Games." arXiv:2103.01955.

### Action space

Continuous 3D acceleration vector `a_i ∈ ℝ³`, each component in
`[-3.5, 3.5] m/s²`. The actor outputs a Gaussian `(μ, σ)` per component;
actions are sampled and clipped to the contract bounds before being passed
to the adapter for validation.

The policy must never output `NaN`, `Inf`, or out-of-bound values. The
adapter (`policy/adapter.py`) enforces this at the boundary — any invalid
output is rejected with `ActionValidationError`, never silently clipped.

### Observation space

`ObservationV1` — 18 floats in raw simulation units. See `INTERFACES.md`
for the exact index mapping. The environment contract remains in raw units;
normalization (if any) is applied inside the policy, with parameters
versioned alongside the model artifact.

---

## 3. Observation preprocessing plan

### Approach: per-feature linear normalization

Each of the 18 observation values will be linearly normalized to
approximately `[-1, 1]` before being fed to the network. Normalization
parameters are fixed from the known simulation bounds (not fitted from
data) so they are reproducible and do not require a dataset.

| Index range | Feature | Raw range | Normalization |
|---|---|---|---|
| 0–2 | Own position `[x, y, z]` | x,y: [0,1000] m; z: [30,200] m | x,y: `v/500 - 1`; z: `(v-115)/85` |
| 3–5 | Own velocity `[vx,vy,vz]` | [-15, 15] m/s per component | `v / 15` |
| 6–8 | Goal displacement | [-1000, 1000] m | `v / 1000` |
| 9–11 | Neighbor relative position | [-1000, 1000] m (or 0 sentinel) | `v / 500` |
| 12–14 | Neighbor relative velocity | [-30, 30] m/s (or 0 sentinel) | `v / 15` |
| 15 | Neighbor distance | [-1, 1000] m | `v / 500` (−1 sentinel preserved as −0.002) |
| 16 | Shield indicator | {0.0, 1.0} | Pass through unchanged |
| 17 | Normalized barrier | [-1, 1] | Pass through unchanged (already normalized) |

**Sentinel handling:** index 15 = −1 (no neighbor) is mapped to a small
negative value (−0.002) so it remains distinct from a real distance of 0.
Indices 9–14 are 0 when no neighbor exists; normalizing 0 → 0, so no
special handling needed.

**These parameters must be stored alongside every trained model artifact.**
If the world bounds change (future contract update), the preprocessing
parameters must be updated and the model retrained. Never use different
normalization at training and inference.

### Network architecture (starting point)

- **Actor:** 3-layer MLP, hidden size 256, tanh activations
  - Input: 18 normalized floats
  - Output: 6 floats (μ and log σ for each of 3 acceleration components)
- **Critic (training only):** 3-layer MLP, hidden size 256, tanh
  - Input: concatenated observations from all agents (18·N floats)
  - Output: scalar value estimate

These are starting values. Architecture search is out of scope for v1;
tune hidden size and depth only if training curves show underfitting.

---

## 4. Training setup

### Environment

- PettingZoo parallel environment from Anas's simulator (P1-AN-01).
- N = 4–8 agents for v1 training scenarios (small enough to be tractable,
  large enough to show cooperative behavior).
- `ScenarioConfig` seed controls initial conditions; training uses a
  curriculum of seeds to prevent overfitting to one scenario.

### Training framework

**RLlib (Ray)** with PPO config, or **CleanRL** MAPPO implementation.
Decision between the two: CleanRL is simpler to inspect and reproduce for
a research prototype; RLlib is better if we need distributed training later.

**Starting choice: CleanRL MAPPO.** Single-file, easy to audit, works
with PettingZoo parallel environments. Switch to RLlib if N > 20 agents
or training time becomes prohibitive.

### Seeding and reproducibility

- Training seed set explicitly in config; stored in the run manifest.
- Environment seed and training seed are separate — a fixed training seed
  does not imply a fixed environment outcome (environment seeds are
  cycled through a curriculum).
- Model checkpoint saved every N episodes with version tag.
- Artifact metadata (see Section 5) must accompany every checkpoint.

### Episode and horizon settings

| Parameter | Value | Rationale |
|---|---|---|
| Episode limit | 1000 steps (100s) | From `ScenarioConfig.episode_limit_steps` |
| PPO rollout length | 256 steps | 4–8 agents × 256 = 1024–2048 transitions per update |
| PPO epochs per update | 4 | Standard PPO default |
| GAE λ | 0.95 | Standard; reduces variance |
| Discount γ | 0.99 | From reward note |
| Clip ε | 0.2 | Standard PPO clip |
| Learning rate | 3e-4 (actor), 1e-3 (critic) | Starting; decay on schedule |

---

## 5. Model artifact metadata

Every saved checkpoint must include a metadata file with:

```json
{
  "artifact_id": "marl-v1-YYYYMMDD-HHMMSS",
  "algorithm": "MAPPO",
  "framework": "CleanRL",
  "framework_version": "...",
  "python_version": "3.11.x",
  "obs_contract": "ObservationV1",
  "action_contract": "MARLAction",
  "obs_normalization": {
    "indices_0_2": "x: v/500-1, y: v/500-1, z: (v-115)/85",
    "indices_3_5": "v/15",
    "indices_6_8": "v/1000",
    "indices_9_11": "v/500",
    "indices_12_14": "v/15",
    "index_15": "v/500 (sentinel -1 -> -0.002)",
    "index_16": "unchanged",
    "index_17": "unchanged"
  },
  "world_bounds_assumed": {
    "xy_m": [0, 1000],
    "z_m": [30, 200],
    "max_speed_mps": 15.0,
    "max_accel_mps2": 3.5
  },
  "training_seed": 42,
  "n_agents_trained": 4,
  "episode_limit_steps": 1000,
  "total_env_steps": "...",
  "reward_version": "P1-SA-01-reward-v1",
  "notes": "..."
}
```

This metadata must travel with the checkpoint. Loading the policy without
it is not permitted — there is no way to verify the preprocessing is
correct without the version record.

---

## 6. What this note does NOT claim

- The trained policy is not a safety controller. It is a nominal action
  proposer. The HOCBF filter (P2-SA-01) handles safety constraints.
- Training reward is not a safety metric. Do not report it as evidence
  of separation performance.
- This note does not describe the filter. The filter architecture is in
  `P0-SA-01-formulation.md`.
- The architecture (hidden size, layers, lr) is a starting point, not
  a tuned or validated configuration.

---

## 7. Open items

| # | Item | Blocks |
|---|---|---|
| M1 | Confirm CleanRL vs RLlib with Anas (environment integration effort) | Training setup |
| M2 | Resolve reward open questions R1–R5 from P1-SA-01-reward.md | Training start |
| M3 | Confirm PettingZoo parallel step signature with Anas (P1-AN-01) | Adapter wiring |
| M4 | Decide curriculum: fixed scenarios or procedurally generated | Training start |
| M5 | Confirm N agents for v1 training (4 proposed) | Critic input size |

---

## 8. Handoff

**For Anas:** need the PettingZoo parallel environment (`reset()` /
`step()`) from P1-AN-01 before training can begin. The adapter in
`policy/adapter.py` already accepts the `ObservationV1` contract.
Confirm the `step()` return signature matches `INTERFACES.md`.

**For Aman:** the model artifact metadata (Section 5) is what you need
for the run manifest and demo. The `artifact_id` ties a specific
checkpoint to a specific run record.

---

## 9. Implementation record (P2-SA-01) — deviations from sections 2–7

This section supersedes the draft where they differ.

| Draft | Implemented | Why |
|---|---|---|
| CleanRL MAPPO (M1) | Custom single-process PyTorch MAPPO (`training/mappo.py`), rollouts in `training/rollout.py` without torch | `DroneEnv` is a plain Python class with its own `step` contract; no PettingZoo wrapper was needed (M3). Env step is ~0.1 ms, so no distributed framework is needed. |
| Actor outputs mean and log-sigma (6 floats) | Actor outputs the mean (3 floats); log-sigma is a learned state-independent parameter vector | Standard in PPO/MAPPO implementations; fewer unstable degrees of freedom. Sigma is not part of the shipped artifact (runtime is the deterministic mean). |
| Normalisation `v/1000` for goal displacement (indices 6–8) | `arcsinh(v/20)/arcsinh(50)` (obs-norm-v3) | With `v/1000` a drone 2.8 m from its goal got an input of 0.003 and stopped outside the 2 m goal radius (observed on a held-out seed). |
| No neighbour range limit | Neighbour terms (9–15) set to the no-neighbour values when index 15 > 250 m (obs-norm-v3) | Separations beyond the trained range saturated the inputs; `head-on-40` (neighbours up to 750 m apart) never completed. The filter is not gated by this: it sees every drone. |
| PPO rollout length 256 steps | Whole episodes: 16 per update (about 8–10k agent-steps) | The environment is episodic and agents terminate at different times. |
| N = 4 agents (M5) | 2–4 agents per episode, padded to 4 slots for the critic | Gives the shared actor head-on pairs as well as groups. The actor does not depend on N. |
| Fixed scenarios or procedural (M4) | Procedural family `scenario-family-v2` with a conflict knob and a curriculum over it | See `training/scenarios.py`. |
| Training from scratch | Behaviour-cloning warm start from the deterministic reference, then PPO | Reaches goals within the first iterations; recorded in artifact metadata (`bc_warmstart_from_reference`). The reference is the initialisation, not a teacher during PPO. |
| Reward R1–R5 open | R1: env reward uses ground-truth nearest separation (training only; not an input to the actor). R2: smoothness uses the applied command, as `env.py` implements. R3: no abort penalty beyond lost progress. R4: no held agents exist in v1. R5: normalisation is versioned inside the artifact (`obs_normalization`). | Matches `simulation/env.py` `P1-SA-01-reward-v1`. R3 is a known gap: aborts are not explicitly penalised. |
| Metadata required | `metadata.json` plus `actor.npz`, loader refuses on hash, contract, normalisation or world-bound mismatch (`policy/learned.py`) | Section 5. |

The critic is centralised (own observation, every slot's observation, alive mask) and is not shipped.
