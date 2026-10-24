"""Fly a trained policy artifact in the simulator and render it (needs the ``plot`` extra).

Writes ``replay.png`` (top-down tracks and minimum separation over time, from
``utm-run``'s plotter) and ``replay.gif`` (animated top-down view: drone markers,
trails, goals, and a red ring wherever a pair is closer than d_safe). The
simulation is the project's own ``DroneEnv`` through ``run_episode``; nothing here
is a separate renderer-side physics model.

Scenario spec: a ScenarioConfig JSON path, ``heldout:<seed>:<n>`` (procedural family,
n drones, conflict 0.8) or ``dense16:<seed>``.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

import numpy as np
from shared.contracts import SAFE_SEPARATION_M, ScenarioConfig

from utm_services.evaluation.cli import plot_replay
from utm_services.policy.learned import LearnedPolicy
from utm_services.policy.reference import ReferencePolicy
from utm_services.simulation.env import PROFILES, DroneEnv
from utm_services.simulation.runner import VARIANTS, run_episode
from utm_services.training.evaluate import random_traffic
from utm_services.training.scenarios import sample_scenario


def load_scenario(spec: str) -> ScenarioConfig:
    if spec.startswith("heldout:"):
        _, seed, n = spec.split(":")
        return sample_scenario(int(seed), int(n), 0.8, 800, scenario_id=f"heldout-n{n}")
    if spec.startswith("dense16:"):
        return random_traffic(int(spec.split(":")[1]))
    return ScenarioConfig.model_validate_json(Path(spec).read_text())


def animate(records: list[dict[str, Any]], goals: dict[str, tuple[float, ...]], path: Path, title: str,
            stride: int = 3) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    ids = sorted(goals)
    pos = np.full((len(records), len(ids), 3), np.nan)
    for t, rec in enumerate(records):
        for s in rec["states"]:
            pos[t, ids.index(s["agent_id"])] = s["position_m"]
    xy = pos[..., :2]
    lo, hi = np.nanmin(xy, axis=(0, 1)), np.nanmax(xy, axis=(0, 1))
    g = np.array([goals[i][:2] for i in ids])
    lo, hi = np.minimum(lo, g.min(0)) - 30.0, np.maximum(hi, g.max(0)) + 30.0
    frames = list(range(0, len(records), stride)) + [len(records) - 1]

    fig, ax = plt.subplots(figsize=(7, 7))
    colors = plt.cm.tab10(np.arange(len(ids)) % 10)
    ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), aspect="equal", xlabel="x (m)", ylabel="y (m)")
    ax.scatter(g[:, 0], g[:, 1], marker="*", s=140, c=colors, edgecolors="k", zorder=3, label="goal")
    trails = [ax.plot([], [], "-", color=c, lw=1.2, alpha=0.7)[0] for c in colors]
    dots = ax.scatter(np.zeros(len(ids)), np.zeros(len(ids)), s=60, c=colors, edgecolors="k", zorder=4)
    rings = [ax.plot([], [], "o", mfc="none", mec="red", ms=18, lw=2)[0] for _ in ids]
    label = ax.set_title(title)

    def draw(t: int):
        cur = xy[t]
        dots.set_offsets(np.where(np.isnan(cur), -1e6, cur))
        for k, tr in enumerate(trails):
            tr.set_data(xy[: t + 1, k, 0], xy[: t + 1, k, 1])
        too_close = np.zeros(len(ids), dtype=bool)
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                if (np.isfinite(cur[i]).all() and np.isfinite(cur[j]).all()
                        and np.linalg.norm(pos[t, i] - pos[t, j]) < SAFE_SEPARATION_M):
                    too_close[i] = too_close[j] = True
        for k, ring in enumerate(rings):
            ring.set_data([cur[k, 0]] if too_close[k] else [], [cur[k, 1]] if too_close[k] else [])
        label.set_text(f"{title}  t = {records[t]['timestamp_ms'] / 1000:.1f} s")
        return [*trails, dots, *rings]

    FuncAnimation(fig, draw, frames=frames, blit=False).save(path, writer=PillowWriter(fps=15))
    plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="utm-replay", description=__doc__.splitlines()[0])
    p.add_argument("scenario", help="ScenarioConfig JSON, heldout:<seed>:<n>, or dense16:<seed>")
    p.add_argument("--artifact", type=Path, help="policy artifact directory (omit for the reference controller)")
    p.add_argument("--variant", choices=VARIANTS, default="unshielded_marl")
    p.add_argument("--profile", choices=PROFILES, default="ideal")
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args(argv)

    config = load_scenario(a.scenario)
    if a.artifact:
        policy, policy_id = LearnedPolicy.load(a.artifact), None
        policy_id = policy.artifact_id
    else:
        policy, policy_id = ReferencePolicy(), "reference-p1-sa-01-pid"
    result = run_episode(config, a.variant, policy=policy, policy_id=policy_id,
                         make_env=lambda rules: DroneEnv(rules, disturbances=PROFILES[a.profile]))
    a.out.mkdir(parents=True, exist_ok=True)
    title = f"{config.scenario_id} | {a.variant} | {policy_id}"
    plot_replay(result.records, a.out / "replay.png", title)
    animate(result.records, {d.agent_id: d.goal_m for d in config.drones}, a.out / "replay.gif", title)
    s = result.summary
    print(f"{title}\n  completed {s['n_completed']}/{s['n_launched']}  steps {s['steps']}  "
          f"min_sep {s['min_separation_m']:.2f} m  violation pair-steps {s['separation_violation_pair_steps']}  "
          f"interventions {s['shield_interventions']}  fallbacks {s['fallback_events']}\n  -> {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
