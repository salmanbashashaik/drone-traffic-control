"""Fly a policy in the simulator and render it (needs the ``plot`` extra; ``physics`` for PyBullet).

Outputs in ``--out``:
  replay.png     top-down tracks with buildings, minimum drone separation, minimum building clearance
  replay.gif     animated top-down view
  replay3d.html  with --html: interactive 3D viewer (drag to orbit, wheel to zoom, follow cameras, scrub)
  scene_*.png / scene.gif  with --pybullet --snapshots N: rendered frames of the PyBullet world

The simulation is the project's own ``DroneEnv`` (or its PyBullet backend) through ``run_episode``.
Buildings come from ``city.py``: scenery with a clear nominal route, NOT something the policy or
the filter knows about. Drone glyphs are drawn larger than real size so they can be seen.

Scenario spec: ``random:<n>`` (n drones, new random starts and destinations every run, seed
printed), ``heldout:<seed>:<n>`` (the same generator with a fixed seed), ``dense16:<seed>``, or a
ScenarioConfig JSON path.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import Any

import numpy as np
from shared.contracts import SAFE_SEPARATION_M, ScenarioConfig

from utm_services.policy.learned import LearnedPolicy
from utm_services.policy.reference import ReferencePolicy
from utm_services.simulation.env import PROFILES, DroneEnv
from utm_services.simulation.runner import VARIANTS, run_episode
from utm_services.training.city import Building, clearance_report, generate_city
from utm_services.training.evaluate import random_traffic
from utm_services.training.scenarios import sample_scenario
from utm_services.training.viewer3d import build_payload, write_viewer

PALETTE = ["#1a73e8", "#f57c00", "#2e9e5b", "#d81b60", "#7e57c2", "#00acc1", "#c0a000", "#6d6d6d"]


def load_scenario(spec: str) -> ScenarioConfig:
    if spec.startswith("random:"):
        # random:<n>[:<conflict 0-1>] -> fresh random starts and goals each call; the seed is printed to reproduce it.
        parts = spec.split(":")
        n, conflict = int(parts[1]), float(parts[2]) if len(parts) > 2 else 0.8
        seed = int.from_bytes(os.urandom(4), "little") % 1_000_000 + 20_000_000
        print(f"random scenario: {n} drones, seed {seed} (reproduce with heldout:{seed}:{n} when conflict is 0.8)", flush=True)
        return sample_scenario(seed, n, conflict, 800, scenario_id=f"random-n{n}-s{seed}")
    if spec.startswith("heldout:"):
        _, seed, n = spec.split(":")
        return sample_scenario(int(seed), int(n), 0.8, 800, scenario_id=f"heldout-n{n}")
    if spec.startswith("dense16:"):
        return random_traffic(int(spec.split(":")[1]))
    return ScenarioConfig.model_validate_json(Path(spec).read_text())


def _positions(records: list[dict[str, Any]], ids: list[str]) -> np.ndarray:
    pos = np.full((len(records), len(ids), 3), np.nan)
    for t, rec in enumerate(records):
        for s in rec["states"]:
            pos[t, ids.index(s["agent_id"])] = s["position_m"]
    return pos


def _min_pair_sep(pos: np.ndarray) -> np.ndarray:
    out = np.full(len(pos), np.nan)
    for t in range(len(pos)):
        pts = pos[t][np.isfinite(pos[t]).all(axis=1)]
        if len(pts) > 1:
            d = np.linalg.norm(pts[:, None] - pts[None], axis=-1)
            out[t] = d[np.triu_indices(len(pts), 1)].min()
    return out


def _min_clearance(pos: np.ndarray, buildings: list[Building]) -> np.ndarray:
    out = np.full(len(pos), np.nan)
    if not buildings:
        return out
    for t in range(len(pos)):
        pts = pos[t][np.isfinite(pos[t]).all(axis=1)]
        if len(pts):
            out[t] = min(b.distance_to(pts).min() for b in buildings)
    return out


def _draw_buildings_2d(ax, buildings: list[Building]) -> None:
    from matplotlib.patches import Rectangle

    for b in buildings:
        tone = 0.86 - 0.5 * min(1.0, b.h / 170.0)
        ax.add_patch(Rectangle((b.cx - b.sx / 2, b.cy - b.sy / 2), b.sx, b.sy, facecolor=(tone, tone, tone + 0.06),
                               edgecolor="#333333", lw=1.0, zorder=1))
        ax.text(b.cx, b.cy, f"{b.h:.0f}", ha="center", va="center", fontsize=7, color="white", zorder=2)


def _extent(pos: np.ndarray, goals: np.ndarray, buildings: list[Building], pad: float = 40.0):
    xy = pos[..., :2][np.isfinite(pos[..., :2])].reshape(-1, 2)
    pts = [xy, goals[:, :2]] + [np.array([[b.cx - b.sx / 2, b.cy - b.sy / 2], [b.cx + b.sx / 2, b.cy + b.sy / 2]])
                                 for b in buildings]
    allp = np.concatenate(pts)
    return allp.min(axis=0) - pad, allp.max(axis=0) + pad


def plot_tracks(records, ids, goals, buildings, path: Path, title: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    pos = _positions(records, ids)
    t_s = np.array([r["timestamp_ms"] for r in records]) / 1000.0
    g = np.array([goals[i] for i in ids])
    lo, hi = _extent(pos, g, buildings)
    fig = plt.figure(figsize=(15, 6.2), dpi=130)
    gs = fig.add_gridspec(2, 2, width_ratios=[1.15, 1.0])
    top, sep, clr = fig.add_subplot(gs[:, 0]), fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[1, 1])
    top.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), aspect="equal", xlabel="x (m)", ylabel="y (m)")
    top.set_facecolor("#eef2ec")
    top.grid(color="#cfd8cc", lw=0.6)
    _draw_buildings_2d(top, buildings)
    for k, aid in enumerate(ids):
        col = PALETTE[k % len(PALETTE)]
        top.plot(pos[:, k, 0], pos[:, k, 1], "-", color=col, lw=2.2, zorder=3, label=aid)
        top.scatter(*pos[0, k, :2], marker="o", s=60, color=col, edgecolors="k", zorder=4)
        top.scatter(*g[k, :2], marker="*", s=240, color=col, edgecolors="k", zorder=5)
    top.set_title(title, fontsize=10)
    top.legend(fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.09), ncol=len(ids),
               title="o start    * goal    (number on a building = roof height, m)", title_fontsize=8)
    sep.plot(t_s, _min_pair_sep(pos), color="#1a73e8", lw=2)
    sep.axhline(SAFE_SEPARATION_M, color="red", ls="--", label=f"d_safe = {SAFE_SEPARATION_M:g} m")
    sep.set(ylabel="min drone-drone\nseparation (m)", ylim=(0, None))
    sep.legend(fontsize=8)
    sep.grid(alpha=0.3)
    if buildings:
        clr.plot(t_s, _min_clearance(pos, buildings), color="#8d6e63", lw=2)
        clr.axhline(0, color="red", ls="--", label="0 = touching a building")
        clr.legend(fontsize=8)
    else:
        clr.text(0.5, 0.5, "no buildings", ha="center", va="center", transform=clr.transAxes)
    clr.set(xlabel="t (s)", ylabel="min distance to\na building (m)", ylim=(0, None))
    clr.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def animate(records, ids, goals, buildings, path: Path, title: str, stride: int = 3) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.patches import Circle

    pos = _positions(records, ids)
    g = np.array([goals[i] for i in ids])
    lo, hi = _extent(pos, g, buildings)
    frames = list(range(0, len(records), stride)) + [len(records) - 1]
    fig, ax = plt.subplots(figsize=(8, 8), dpi=110)
    ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), aspect="equal", xlabel="x (m)", ylabel="y (m)")
    ax.set_facecolor("#eef2ec")
    ax.grid(color="#cfd8cc", lw=0.6)
    _draw_buildings_2d(ax, buildings)
    cols = [PALETTE[k % len(PALETTE)] for k in range(len(ids))]
    ax.scatter(g[:, 0], g[:, 1], marker="*", s=260, c=cols, edgecolors="k", zorder=5)
    trails = [ax.plot([], [], "-", color=c, lw=2.2, zorder=3)[0] for c in cols]
    drones = ax.scatter(np.zeros(len(ids)), np.zeros(len(ids)), marker="X", s=330, c=cols, edgecolors="k",
                        linewidths=1.2, zorder=6)
    bubbles = [ax.add_patch(Circle((0, 0), SAFE_SEPARATION_M / 2, fc="none", ec=c, lw=1.2, zorder=4)) for c in cols]
    title_text = ax.set_title(title, fontsize=9)

    def draw(t: int):
        cur = pos[t]
        drones.set_offsets(np.where(np.isfinite(cur[:, :2]), cur[:, :2], -1e6))
        close = np.zeros(len(ids), dtype=bool)
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                if np.isfinite(cur[i]).all() and np.isfinite(cur[j]).all() \
                        and np.linalg.norm(cur[i] - cur[j]) < SAFE_SEPARATION_M:
                    close[i] = close[j] = True
        for k, tr in enumerate(trails):
            tr.set_data(pos[: t + 1, k, 0], pos[: t + 1, k, 1])
            bubbles[k].center = (cur[k, 0], cur[k, 1]) if np.isfinite(cur[k, :2]).all() else (-1e6, -1e6)
            bubbles[k].set(ec="red" if close[k] else cols[k], lw=3 if close[k] else 1.2)
        title_text.set_text(f"{title}\\nt = {records[t]['timestamp_ms'] / 1000:.1f} s")
        return [*trails, drones, *bubbles]

    FuncAnimation(fig, draw, frames=frames, blit=False).save(path, writer=PillowWriter(fps=15))
    plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="utm-replay", description=__doc__.splitlines()[0])
    p.add_argument("scenario", help="random:<n> | heldout:<seed>:<n> | dense16:<seed> | ScenarioConfig JSON path")
    p.add_argument("--artifact", type=Path, help="policy artifact directory (omit for the reference controller)")
    p.add_argument("--variant", choices=VARIANTS, default="unshielded_marl")
    p.add_argument("--profile", choices=PROFILES, default="ideal")
    p.add_argument("--buildings", type=int, default=26, help="number of buildings (0 = none); scenery, see city.py")
    p.add_argument("--html", action="store_true", help="also write replay3d.html: self-contained interactive 3D viewer (orbit/zoom/follow, works offline)")
    p.add_argument("--pybullet", action="store_true",
                   help="fly in the PyBullet rigid-body quadrotor backend (needs the physics extra; the policy "
                        "was trained on point-mass dynamics)")
    p.add_argument("--gui", action="store_true", help="with --pybullet: open the 3D window and wait until it is closed")
    p.add_argument("--speed", type=float, default=1.0, help="with --gui: playback speed multiplier")
    p.add_argument("--snapshots", type=int, default=0, metavar="N",
                   help="with --pybullet: render the 3D world every N steps to scene_*.png and scene.gif")
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args(argv)

    config = load_scenario(a.scenario)
    buildings = generate_city(config, a.buildings) if a.buildings else []
    if a.artifact:
        policy = LearnedPolicy.load(a.artifact)
        policy_id = policy.artifact_id
    else:
        policy, policy_id = ReferencePolicy(), "reference-p1-sa-01-pid"
    holder: list[Any] = []
    if a.pybullet:
        from utm_services.training.pybullet_scene import SceneEnv

        def make_env(rules):
            holder.append(SceneEnv(rules, gui=a.gui, playback_speed=a.speed, buildings=buildings,
                                   snapshot_every=a.snapshots))
            return holder[-1]
    else:
        def make_env(rules):
            return DroneEnv(rules, disturbances=PROFILES[a.profile])
    result = run_episode(config, a.variant, policy=policy, policy_id=policy_id, make_env=make_env)
    a.out.mkdir(parents=True, exist_ok=True)
    title = f"{config.scenario_id} | {a.variant} | {policy_id}" + (" | PyBullet" if a.pybullet else "")
    ids = sorted(d.agent_id for d in config.drones)
    goals = {d.agent_id: d.goal_m for d in config.drones}
    plot_tracks(result.records, ids, goals, buildings, a.out / "replay.png", title)
    animate(result.records, ids, goals, buildings, a.out / "replay.gif", title)
    if a.html:
        note = ("Drones are enlarged for visibility and drawn level. Buildings are scenery: the policy and the filter "
                "do not see them." if buildings else "Drones are enlarged for visibility and drawn level.")
        write_viewer(a.out / "replay3d.html",
                     build_payload(result.records, ids, goals, buildings, result.summary, title, note))
    clear = clearance_report(result.records, buildings)
    s = result.summary
    extra = ""
    if holder:
        env = holder[-1]
        extra = f"  building contacts (physics) {len(env.building_contacts)}"
        if env.snapshots:
            from utm_services.training.pybullet_scene import save_snapshots

            save_snapshots(env.snapshots, a.out)
    print(f"{title}\n  completed {s['n_completed']}/{s['n_launched']}  steps {s['steps']}  "
          f"min_sep {s['min_separation_m']:.2f} m  violation pair-steps {s['separation_violation_pair_steps']}  "
          f"interventions {s['shield_interventions']}  fallbacks {s['fallback_events']}\n"
          f"  buildings {clear['n_buildings']}  min clearance "
          f"{'n/a' if clear['min_clearance_m'] is None else format(clear['min_clearance_m'], '.1f') + ' m'}  "
          f"track samples inside a building {clear['samples_inside']}{extra}\n  -> {a.out}")
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
