"""Procedural scenario family for MARL training and held-out evaluation (P2-SA-01).

Answers method-note open item M4 (curriculum): scenarios are generated from a
seed, with a ``conflict`` knob in [0, 1] that sets the share of drones whose goal
is placed at another drone's start. Those goals force head-on and crossing paths;
conflict 0 gives independent random start/goal pairs. Initial conditions come
only from the seed, so every scenario is reproducible from ``(seed, n, conflict)``.

Seed ranges are disjoint so training never sees evaluation scenarios:
    train      TRAIN_SEED_BASE + run_seed * 1_000_000 + iteration * 1_000 + episode
    validation VAL_SEED_BASE + i      (model selection during training)
    test       TEST_SEED_BASE + i     (development evaluation; its failures guided the observation
                                       preprocessing, so it is NOT clean held-out for the final design)
    final      FINAL_SEED_BASE + i    (frozen-design evaluation, run once)
"""

from __future__ import annotations

import numpy as np
from shared.contracts import MAX_SPEED_MPS, ScenarioConfig

FAMILY_VERSION = "scenario-family-v2"
TRAIN_SEED_BASE = 100_000_000
VAL_SEED_BASE = 5_000_000
TEST_SEED_BASE = 9_000_000
FINAL_SEED_BASE = 12_000_000   # untouched until the design is frozen; the final report uses it once

_MIN_POINT_SEPARATION_M = 30.0   # between any two starts
_MIN_GOAL_SEPARATION_M = 12.0    # between any two goals (two goals < 5 m apart cannot both be completed)
_MIN_TRIP_M = 60.0               # start-to-goal distance
_GOAL_JITTER_M = 5.0


def sample_scenario(
    seed: int,
    n_agents: int = 4,
    conflict: float = 0.5,
    episode_limit_steps: int = 800,
    arena_half_m: tuple[float, float] = (100.0, 300.0),
    scenario_id: str | None = None,
) -> ScenarioConfig:
    """Seeded scenario: n_agents drones in a square arena centred inside the world."""
    if n_agents < 1:
        raise ValueError("n_agents must be >= 1")
    if not 0.0 <= conflict <= 1.0:
        raise ValueError("conflict must be in [0, 1]")
    rng = np.random.default_rng(seed)
    centre = rng.uniform(400.0, 600.0, 2)
    half = rng.uniform(*arena_half_m)

    def random_point() -> np.ndarray:
        return np.array([*(centre + rng.uniform(-half, half, 2)), rng.uniform(80.0, 120.0)])

    for _ in range(10_000):
        starts = [random_point() for _ in range(n_agents)]
        if all(np.linalg.norm(a - b) >= _MIN_POINT_SEPARATION_M for i, a in enumerate(starts) for b in starts[:i]):
            break
    else:  # pragma: no cover - arena is large enough for the supported n
        raise RuntimeError("could not place starts")

    for _ in range(10_000):
        # Drones in `forced` fly to another forced drone's start (a derangement of the subset).
        forced = [i for i in range(n_agents) if rng.random() < conflict]
        goals = [random_point() for _ in range(n_agents)]
        if len(forced) >= 2:
            targets = forced[:]
            while True:
                rng.shuffle(targets)
                if all(a != b for a, b in zip(forced, targets, strict=True)):
                    break
            for i, j in zip(forced, targets, strict=True):
                goals[i] = starts[j] + np.array([*rng.normal(0.0, _GOAL_JITTER_M, 2), rng.normal(0.0, 2.0)])
                goals[i][2] = float(np.clip(goals[i][2], 70.0, 130.0))
        ok_goals = all(np.linalg.norm(a - b) >= _MIN_GOAL_SEPARATION_M for i, a in enumerate(goals) for b in goals[:i])
        ok_trips = all(np.linalg.norm(s - g) >= _MIN_TRIP_M for s, g in zip(starts, goals, strict=True))
        if ok_goals and ok_trips:
            break
    else:  # pragma: no cover
        raise RuntimeError("could not place goals")

    drones = []
    for k in range(n_agents):
        direction = goals[k] - starts[k]
        direction /= np.linalg.norm(direction)
        speed = float(rng.uniform(0.0, MAX_SPEED_MPS * 0.9)) if rng.random() < 0.5 else 0.0
        drones.append({
            "agent_id": f"d{k}",
            "initial_position_m": tuple(float(v) for v in starts[k]),
            "initial_velocity_mps": tuple(float(v) for v in direction * speed),
            "goal_m": tuple(float(v) for v in goals[k]),
        })
    return ScenarioConfig(
        scenario_id=scenario_id or f"family-n{n_agents}-c{conflict:.2f}",
        seed=int(seed),
        episode_limit_steps=episode_limit_steps,
        drones=drones,
    )


def eval_scenarios(base: int, count: int, max_agents: int = 4, conflict: float = 0.8,
                   episode_limit_steps: int = 800) -> list[ScenarioConfig]:
    """Fixed evaluation set: sizes cycle through 2..max_agents, conflict fixed (hard), seeds base..base+count."""
    sizes = list(range(2, max_agents + 1))
    return [
        sample_scenario(base + i, sizes[i % len(sizes)], conflict, episode_limit_steps,
                        scenario_id=f"eval-n{sizes[i % len(sizes)]}-c{conflict:.2f}")
        for i in range(count)
    ]
