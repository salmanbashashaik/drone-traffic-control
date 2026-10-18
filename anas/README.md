# anas/ — Your Work
**Branch**: `anas/marl-policy` | **Role**: ML — MARL Training & Simulation Engine

You own the simulation environment, the training pipeline, the chaos testing, and the edge export. You are also the integration point — your `ShieldedEnv` is where Salman's math plugs into the running simulation.

---

## Your 10 tickets

| ID | Week | File to create | What it is |
|----|------|----------------|------------|
| A-01 | 1 | `env/drone_env.py` | PettingZoo `ParallelEnv` for N drones in 3D airspace |
| A-02 | 1 | `training/train_baseline.py` + checkpoint | PPO/IPPO unshielded baseline, 500k steps, logged to W&B |
| A-03 | 1 | (contributes to `shared/schemas.py`) | `MARLAction` dataclass — coordinate with Salman |
| A-04 | 2 | `env/shielded_env.py` | Wrapper: MARL output → CBF-QP → physics. Transparent to the policy. |
| A-05 | 2 | `training/train_shielded.py` + checkpoint | Retrain with shield penalty, measure reduced intervention rate |
| A-06 | 3 | `training/train_scale50.py` | Scale to N=50, optimize QP call, target < 200ms/step |
| A-07 | 3 | `chaos/chaos_injector.py` | Comm delay, Byzantine drone, wind gust — all togglable mid-episode |
| A-08 | 3 | `evaluation/run_stress_test.py` + CSV | 100 episodes, N=50, all chaos active, export metrics CSV |
| A-09 | 4 | `edge/export_onnx.py` + benchmark | ONNX export, latency on CPU/GPU/Jetson sim, target < 10ms |
| A-10 | 4 | `docs/blog_edge_marl.md` | Blog post on MARL edge optimization |

---

## Environment spec (A-01)

Build a PettingZoo `ParallelEnv`:

| Parameter | Value |
|-----------|-------|
| Airspace | 1000m × 1000m × 200m |
| Altitude range | 30m – 200m |
| State per drone | 18-dim vector (see INTERFACES.md § ENV-OUTPUT for exact layout) |
| Action per drone | 3D acceleration in m/s², clipped to ±3 m/s² |
| Max speed | 15 m/s (clip velocity) |
| Timestep | 0.1 seconds |
| Max steps/episode | 1000 |
| Reward | +1 reach goal, −100 collision, small negative distance shaping |

---

## ShieldedEnv design (A-04)

This is a **wrapper** around `DroneEnv`. The MARL policy proposes actions; the wrapper passes them through `solve_cbf_qp()` before executing physics. The policy is never aware it was overridden.

Flow:
```
policy output (proposed_accel per agent)
    → build MARLAction list
    → call solve_cbf_qp(states, marl_actions)   ← Salman's function
    → get SafeAction list
    → inject shield info back into env for obs [16:17]
    → call parent step() with safe_accel
```

After integration, also call:
```
self.rerun_logger.log_step(states, safe_actions, metrics)   ← Aman's visualizer hook
```

---

## Chaos injectors (A-07)

Three independent, toggleable disruptions:

**1. CommDelay** — stales the state fed to the policy.  
Take the drone's true state and return a version that's `delay_ms` milliseconds in the past (extrapolate position/velocity backward). Only the policy sees the stale state; physics uses the true state.

**2. ByzantineDrone** — one drone broadcasts false telemetry.  
All other drones' policies see randomized (false) position/velocity for this drone. The Byzantine drone itself behaves normally in physics.

**3. WindGust** — adds noise to physics.  
Add a bounded random vector to the acceleration command before the physics step. Magnitude controllable. Should not be large enough to cause unrecoverable situations on its own.

---

## Stress test metrics (A-08)

Run 100 episodes, N=50, all three chaos injectors active. Collect per-episode:

| Metric | Description |
|--------|-------------|
| `throughput_pct` | % of drones that reached their goal |
| `n_collisions` | Count of drones that collided |
| `n_shield_interventions` | Total times shield modified a proposed action |
| `n_fallbacks` | Times QP was infeasible and hover was triggered |
| `min_separation_m` | Minimum pairwise separation across the episode |
| `mean_step_time_ms` | Average time per simulation step |

Export to `results/stress_test_50drones.csv`. Column names are fixed — Aman's plotting scripts depend on them (see `INTERFACES.md § SHIELD-LOG`).

---

## What you receive and what you produce

**Receive from Salman**: `SafeAction` list — `INTERFACES.md § CBF-OUTPUT`  
**Produce for Salman**: `MARLAction` list — `INTERFACES.md § MARL-ACTION`  
**Produce for Aman**: `results/stress_test_50drones.csv`  
**Visualizer hook**: call `RerunLogger.log_step()` at end of each step — `INTERFACES.md § RERUN-LOG`

---

## Recommended libraries

| Task | Library |
|------|---------|
| Environment | PettingZoo |
| Training | TorchRL (PPO) or RLlib (IPPO) |
| Experiment tracking | Weights & Biases |
| Edge export | ONNX + ONNX Runtime |
| QP solver (Salman's) | CVXPY + OSQP (you just call the function) |

---

## Week 1 done means

- [ ] `drone_env.py` — `reset()` and `step()` work for N=10, random policy runs 100 episodes
- [ ] `train_baseline.py` — W&B run exists, checkpoint saved, reward improving
- [ ] `MARLAction` schema committed to `shared/schemas.py` (coordinate with Salman)
- [ ] PR opened into `dev`
