# anas/ — Your Work
**Branch**: `anas/marl-policy` | **Role**: ML + Edge AI + Simulation Engine

You build the simulation environment that everyone else depends on, run the chaos stress tests, and own edge deployment. You're the infrastructure underneath Salman's RL training — your env is what his policy learns from. You also take the final trained model and make it run fast enough for real hardware.

---

## Your 8 tickets

| ID | Week | Deliverable | What it is |
|----|------|-------------|------------|
| A-01 | 1 | `env/drone_env.py` | PettingZoo `ParallelEnv` for N drones in 3D airspace |
| A-02 | 1 | `shared/schemas.py` | `MARLAction` dataclass (coordinate with Salman) |
| A-03 | 2 | `env/shielded_env.py` | Wrapper: policy output → CBF-QP → physics |
| A-04 | 2 | `env/drone_env.py` (update) + `docs/perf_notes.md` | Profile and optimize for N=50 |
| A-05 | 3 | `chaos/chaos_injector.py` + `training/train_scale50.py` | Chaos injectors + 50-drone scale run |
| A-06 | 3 | `evaluation/run_stress_test.py` + CSV | 100-episode stress test, metrics export |
| A-07 | 4 | `edge/export_onnx.py` + `results/edge_benchmark.md` | ONNX export + latency benchmark |
| A-08 | 4 | `docs/blog_edge_ml.md` | Blog: MARL architecture choices for edge deployment |

---

## Directory layout

```
anas/
├── env/
│   ├── drone_env.py          ← A-01: base environment
│   └── shielded_env.py       ← A-03: CBF wrapper + visualizer hook
├── training/
│   └── train_scale50.py      ← A-05: 50-drone scale run
├── chaos/
│   └── chaos_injector.py     ← A-05: comm delay, Byzantine, wind
├── evaluation/
│   └── run_stress_test.py    ← A-06: stress test runner
├── edge/
│   └── export_onnx.py        ← A-07: ONNX export + benchmark
├── checkpoints/              ← receives Salman's .pt files (gitignored)
└── docs/
    ├── perf_notes.md         ← A-04: profiling results
    └── blog_edge_ml.md       ← A-08
```

---

## Environment spec (A-01)

Build a PettingZoo `ParallelEnv`:

| Parameter | Value |
|-----------|-------|
| Airspace | 1000m × 1000m × 200m |
| Altitude range | 30m – 200m |
| State per drone | 18-dim vector — see `INTERFACES.md § ENV-OUTPUT` for exact layout |
| Action per drone | 3D acceleration, m/s², clipped to ±3 m/s² |
| Max speed | 15 m/s |
| Timestep | 0.1 s |
| Max steps/episode | 1000 |
| Reward | +1 reach goal, −100 collision, small negative distance shaping |

The observation vector layout (18 dims) is **fixed** — Salman's training spec (S-04) depends on it:
```
[0:3]   own position
[3:6]   own velocity
[6:9]   goal direction (goal − position)
[9:12]  nearest neighbor position (relative)
[12:15] nearest neighbor velocity (relative)
[15]    nearest neighbor distance
[16]    shield_active last step (0.0 or 1.0)
[17]    h_min last step (clipped to [−1, 1])
```
Do not change indices without updating `INTERFACES.md` and coordinating with Salman.

---

## ShieldedEnv design (A-03)

Wrapper around `DroneEnv`. The policy outputs actions; the wrapper pipes them through Salman's `solve_cbf_qp()` before the physics step. The policy never knows it was overridden.

Flow per step:
```
1. Get proposed actions from policy
2. Build MARLAction list
3. Call solve_cbf_qp(states, marl_actions)  ← Salman's function
4. Inject SafeAction shield info back into env (for obs [16:17])
5. Call parent step() with safe_accel
6. Call rerun_logger.log_step(states, safe_actions, metrics)  ← Aman's visualizer hook
```

The `log_step` hook signature is in `INTERFACES.md § RERUN-LOG`. Agree it with Aman before Week 3.

---

## Chaos injectors (A-05)

Three independent injectors, all togglable mid-episode:

**CommDelay** — stales the state fed to the policy. Return the drone's state from `delay_ms` milliseconds ago (extrapolate backward). Only the policy sees the stale state; physics uses truth.

**ByzantineDrone** — one drone broadcasts randomized false position/velocity to all other agents' observations. It behaves normally in physics; others just can't trust its telemetry.

**WindGust** — adds a bounded random vector to acceleration before the physics step. Magnitude is configurable. Should not be catastrophically large — it stress-tests the shield, not the environment stability.

---

## Stress test metrics (A-06)

100 episodes, N=50, all chaos injectors active. Collect per-episode:

| Column name (fixed) | Description |
|---------------------|-------------|
| `throughput_pct` | % of drones reaching goal |
| `n_collisions` | Collision count |
| `n_shield_interventions` | Times shield modified a proposed action |
| `n_fallbacks` | Times QP was infeasible → hover triggered |
| `min_separation_m` | Minimum pairwise separation in episode |
| `mean_step_time_ms` | Average step time |

CSV saved to `results/stress_test_50drones.csv`. Column names are fixed — Aman's `plot_results.py` reads them.

---

## Edge export (A-07)

Take Salman's `shielded_v1.pt` checkpoint. Export the policy network (not the CBF solver — that runs separately on-device) to ONNX. Benchmark inference latency:

| Target | How to simulate |
|--------|----------------|
| CPU (laptop) | ONNX Runtime, default settings |
| GPU (server) | ONNX Runtime with CUDA EP |
| Jetson Orin | ONNX Runtime, limit to 4 threads, no GPU |

Target: < 10ms per inference step. Report: model size (MB), latency (mean ± std), throughput (inferences/sec).

---

## What you receive and what you produce

**Receive from Salman**: `solve_cbf_qp()` function (from `salman/cbf/qp_solver.py`), `shielded_v1.pt` checkpoint  
**Produce for Salman**: A running `DroneEnv` he can import and train against  
**Produce for Aman**: `results/stress_test_50drones.csv`, `shielded_v1.onnx`, `log_step()` calls to visualizer

---

## Week 1 done means

- [ ] `drone_env.py` — `reset()` and `step()` work for N=10, random policy runs 100 episodes without crash
- [ ] `shared/schemas.py` — `MARLAction` committed, `DroneState` and `SafeAction` stubs added
- [ ] Reviewed Salman's `training_spec.md` (S-04) and confirmed env matches
- [ ] PR into `dev`
