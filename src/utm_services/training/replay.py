"""Fly a trained policy artifact in the simulator and render it (needs the ``plot`` extra).

Writes ``replay.png`` (top-down tracks and minimum separation over time, from
``utm-run``'s plotter), ``replay.gif`` (animated top-down view: drone markers,
trails, goals, and a red ring wherever a pair is closer than d_safe) and, with
``--html``, ``replay3d.html`` (interactive 3D scene in the browser, needs plotly). The
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


def _gui_env_class():
    """PyBulletDroneEnv whose contact query survives PyBullet-GUI's intermittent ``None`` replies.

    With the 3D window on, ``getContactPoints`` sometimes returns ``None`` (the GUI server did not
    answer in time) although the connection is alive. The collision check must never be skipped,
    so this retries the query and raises if it still fails.
    """
    import time

    import pybullet as pb

    from utm_services.simulation.pybullet_env import ABORTED, ACTIVE, PyBulletDroneEnv

    class GuiSafeEnv(PyBulletDroneEnv):
        def _check_collisions(self, agents):
            for agent in agents:
                if agent.status != ACTIVE:
                    continue
                body = self._bodies[self._ids.index(agent.agent_id)]
                contacts = None
                for _ in range(50):
                    contacts = pb.getContactPoints(bodyA=body, physicsClientId=self._client)
                    if contacts is not None:
                        break
                    time.sleep(0.02)
                if contacts is None:
                    raise RuntimeError("PyBullet did not answer getContactPoints after 50 tries; "
                                       "collision check cannot be skipped")
                for c in contacts:
                    if c[2] in self._bodies:
                        other = self._agents[self._ids[self._bodies.index(c[2])]]
                        for a in (agent, other):
                            if a.status == ACTIVE:
                                a.status, a.abort_reason = ABORTED, "collision"
                        self._metrics["collisions"] += 1

    return GuiSafeEnv


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


def animate3d(records: list[dict[str, Any]], goals: dict[str, tuple[float, ...]], path: Path, title: str,
              stride: int = 3) -> None:
    """Interactive 3D replay (plotly): rotate/zoom, play/pause, time slider; red = pair closer than d_safe."""
    import plotly.graph_objects as go
    from plotly.colors import qualitative

    ids = sorted(goals)
    pos = np.full((len(records), len(ids), 3), np.nan)
    for t, rec in enumerate(records):
        for s in rec["states"]:
            pos[t, ids.index(s["agent_id"])] = s["position_m"]
    frames_idx = list(range(0, len(records), stride)) + [len(records) - 1]
    colors = [qualitative.Plotly[k % 10] for k in range(len(ids))]
    g = np.array([goals[i] for i in ids])

    def close_pairs(t: int) -> np.ndarray:
        pts = []
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                a, b = pos[t, i], pos[t, j]
                if np.isfinite(a).all() and np.isfinite(b).all() and np.linalg.norm(a - b) < SAFE_SEPARATION_M:
                    pts += [a, b]
        return np.array(pts).reshape(-1, 3)

    def trace_data(t: int) -> list[go.Scatter3d]:
        out = []
        for k in range(len(ids)):
            trail = pos[: t + 1, k]
            cur = pos[t, k]
            out.append(go.Scatter3d(x=trail[:, 0], y=trail[:, 1], z=trail[:, 2], mode="lines",
                                    line={"color": colors[k], "width": 4}, name=f"{ids[k]} path", showlegend=False))
            out.append(go.Scatter3d(x=[cur[0]], y=[cur[1]], z=[cur[2]], mode="markers",
                                    marker={"size": 7, "color": colors[k], "line": {"color": "black", "width": 2}},
                                    name=ids[k]))
        c = close_pairs(t)
        out.append(go.Scatter3d(x=c[:, 0], y=c[:, 1], z=c[:, 2], mode="markers", name="closer than d_safe",
                                marker={"size": 14, "color": "rgba(255,0,0,0.35)", "line": {"color": "red", "width": 3}}))
        return out

    goal_trace = go.Scatter3d(x=g[:, 0], y=g[:, 1], z=g[:, 2], mode="markers", name="goals",
                              marker={"size": 8, "symbol": "diamond", "color": colors, "line": {"color": "black", "width": 2}})
    fig = go.Figure(data=[goal_trace, *trace_data(0)])
    fig.frames = [go.Frame(data=trace_data(t), traces=list(range(1, 2 * len(ids) + 2)), name=str(t))
                  for t in frames_idx]
    finite = pos[np.isfinite(pos)].reshape(-1, 3)
    lo, hi = np.minimum(finite.min(0), g.min(0)) - 20.0, np.maximum(finite.max(0), g.max(0)) + 20.0
    steps = [{"method": "animate", "label": f"{records[t]['timestamp_ms'] / 1000:.1f}s",
                  "args": [[str(t)], {"mode": "immediate", "frame": {"duration": 0, "redraw": True}, "transition": {"duration": 0}}]}
             for t in frames_idx]
    fig.update_layout(
        title=title, template="plotly_white", height=800,
        scene={"xaxis": {"title": "x (m)", "range": [lo[0], hi[0]]}, "yaxis": {"title": "y (m)", "range": [lo[1], hi[1]]},
                   "zaxis": {"title": "altitude z (m)", "range": [max(30.0, lo[2]), min(200.0, hi[2] + 10)]},
                   "aspectmode": "manual", "aspectratio": {"x": 1, "y": 1, "z": 0.45},
                   "camera": {"eye": {"x": 1.5, "y": -1.6, "z": 0.9}}},
        updatemenus=[{"type": "buttons", "showactive": False, "x": 0.05, "y": 0.02, "buttons": [
            {"label": "Play", "method": "animate", "args": [None, {"frame": {"duration": 60, "redraw": True},
                                                                   "fromcurrent": True, "transition": {"duration": 0}}]},
            {"label": "Pause", "method": "animate", "args": [[None], {"mode": "immediate", "frame": {"duration": 0}}]}]}],
        sliders=[{"active": 0, "steps": steps, "x": 0.15, "len": 0.8, "currentvalue": {"prefix": "t = "}}],
    )
    fig.write_html(path, include_plotlyjs=True, auto_play=False)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="utm-replay", description=__doc__.splitlines()[0])
    p.add_argument("scenario", help="ScenarioConfig JSON, heldout:<seed>:<n>, or dense16:<seed>")
    p.add_argument("--artifact", type=Path, help="policy artifact directory (omit for the reference controller)")
    p.add_argument("--variant", choices=VARIANTS, default="unshielded_marl")
    p.add_argument("--profile", choices=PROFILES, default="ideal")
    p.add_argument("--html", action="store_true", help="also write replay3d.html (interactive 3D; needs plotly)")
    p.add_argument("--pybullet", action="store_true",
                   help="fly in the PyBullet rigid-body quadrotor backend instead of the point-mass sim "
                        "(needs the physics extra; the policy was trained on point-mass dynamics)")
    p.add_argument("--gui", action="store_true", help="with --pybullet: open the 3D window and wait until it is closed")
    p.add_argument("--speed", type=float, default=1.0, help="with --gui: playback speed multiplier")
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args(argv)

    config = load_scenario(a.scenario)
    if a.artifact:
        policy, policy_id = LearnedPolicy.load(a.artifact), None
        policy_id = policy.artifact_id
    else:
        policy, policy_id = ReferencePolicy(), "reference-p1-sa-01-pid"
    holder: list[Any] = []
    if a.pybullet:
        env_class = _gui_env_class() if a.gui else __import__(
            "utm_services.simulation.pybullet_env", fromlist=["PyBulletDroneEnv"]).PyBulletDroneEnv

        def make_env(rules):
            holder.append(env_class(rules, gui=a.gui, playback_speed=a.speed))
            return holder[-1]
    else:
        def make_env(rules):
            return DroneEnv(rules, disturbances=PROFILES[a.profile])
    result = run_episode(config, a.variant, policy=policy, policy_id=policy_id, make_env=make_env)
    a.out.mkdir(parents=True, exist_ok=True)
    title = f"{config.scenario_id} | {a.variant} | {policy_id}" + (" | PyBullet" if a.pybullet else "")
    plot_replay(result.records, a.out / "replay.png", title)
    goals = {d.agent_id: d.goal_m for d in config.drones}
    animate(result.records, goals, a.out / "replay.gif", title)
    if a.html:
        animate3d(result.records, goals, a.out / "replay3d.html", title)
    s = result.summary
    print(f"{title}\n  completed {s['n_completed']}/{s['n_launched']}  steps {s['steps']}  "
          f"min_sep {s['min_separation_m']:.2f} m  violation pair-steps {s['separation_violation_pair_steps']}  "
          f"interventions {s['shield_interventions']}  fallbacks {s['fallback_events']}\n  -> {a.out}")
    if holder:
        if a.gui:
            import time

            import pybullet as pb
            print("Episode done. Close the 3D window to exit.")
            while pb.isConnected(holder[-1]._client):
                time.sleep(0.2)
        holder[-1].close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
