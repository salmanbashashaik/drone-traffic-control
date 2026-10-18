# anas/ — MARL Policy & Simulation Engine
**Owner**: Anas | **Branch**: `anas/marl-policy`

You own the **simulation environment, the MARL training loop, and the chaos testing**. You are also the integration point — your `ShieldedEnv` is where Salman's math plugs into the running simulation.

## Your tickets

| Ticket | Week | File | Status |
|--------|------|------|--------|
| A-01 | 1 | `env/drone_env.py` | Starter provided |
| A-02 | 1 | `training/train_baseline.py` | You create |
| A-03 | 1 | `env/action_schema.py` | You create (schema in shared/) |
| A-04 | 2 | `env/shielded_env.py` | You create |
| A-05 | 2 | `training/train_shielded.py` | You create |
| A-06 | 3 | `training/train_scale50.py` | You create |
| A-07 | 3 | `chaos/chaos_injector.py` | You create |
| A-08 | 3 | `evaluation/run_stress_test.py` | You create |
| A-09 | 4 | `edge/export_onnx.py` | You create |
| A-10 | 4 | `docs/blog_edge_marl.md` | You create |

## Directory layout

```
anas/
├── env/
│   ├── drone_env.py        ← A-01: PettingZoo base env (10 drones)
│   ├── shielded_env.py     ← A-04: wrapper that calls CBF-QP before physics
│   └── action_schema.py    ← A-03: MARLAction (lives in shared/schemas.py)
├── training/
│   ├── train_baseline.py   ← A-02: unshielded policy, 500k steps
│   ├── train_shielded.py   ← A-05: shield-aware training with penalty
│   └── train_scale50.py    ← A-06: 50-drone scalability run
├── chaos/
│   └── chaos_injector.py   ← A-07: comm delay, Byzantine drone, wind gust
├── evaluation/
│   └── run_stress_test.py  ← A-08: 100 episodes, collect metrics CSV
├── edge/
│   └── export_onnx.py      ← A-09: ONNX export + latency benchmark
├── checkpoints/
│   └── .gitkeep            ← .pt files go here (gitignored)
└── docs/
    └── blog_edge_marl.md   ← A-10: Medium/Substack post
```

## Interface you must honor

**Input from Salman**: `SafeAction` list — see `INTERFACES.md § CBF-OUTPUT`
**Output to Salman's QP**: `MARLAction` list — see `INTERFACES.md § MARL-ACTION`
**Output to Aman's analysis**: `results/stress_test_50drones.csv` — see `INTERFACES.md § SHIELD-LOG`

The `ShieldedEnv` wrapper flow:
```
MARL policy → MARLAction[] → solve_cbf_qp() → SafeAction[] → physics step
```

## ShieldedEnv skeleton (ticket A-04)

```python
from anas.env.drone_env import DroneEnv
from salman.cbf.qp_solver import solve_cbf_qp
from shared.schemas import MARLAction

class ShieldedEnv(DroneEnv):
    def step(self, actions: dict[str, np.ndarray]):
        # 1. Build MARLAction list from raw policy outputs
        states = self.get_states()
        marl_actions = [MARLAction(a, actions[a], self._step_count) for a in actions]

        # 2. Filter through CBF-QP
        safe_actions = solve_cbf_qp(states, marl_actions)

        # 3. Update shield visualization info
        self.inject_shield_info(safe_actions)

        # 4. Replace actions with safe ones, call parent step
        safe_action_dict = {sa.agent_id: sa.safe_accel for sa in safe_actions}
        return super().step(safe_action_dict)
```

## Training setup

Recommended: **TorchRL with PPO** or **RLlib with IPPO** (independent PPO per agent).

```bash
pip install torchrl tensordict ray[rllib]
```

Log everything to W&B:
```python
import wandb
wandb.init(project="drone-traffic-control", name="baseline_v1")
```

## Week 1 definition of done

- [ ] `drone_env.py` smoke test passes (`python -m anas.env.drone_env`)
- [ ] `train_baseline.py` runs, W&B run created, checkpoint saved
- [ ] Action schema committed to `shared/schemas.py` (coordinate with Salman)
- [ ] PR opened into `dev`
