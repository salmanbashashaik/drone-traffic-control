"""`utm-viz-rerun`: open a decision log (utm-decision-log) in the Rerun viewer.

    utm-viz-rerun runs/x/decisions.jsonl [--save x.rrd]

3D scene in the world frame (metres, z up): drones with trails, a sphere of radius d_eff/2
around each drone (two spheres touch at d_eff) coloured by the distance to its nearest
neighbour, the proposed (blue) and safe (green) acceleration arrows, and the backup manoeuvre
the filter simulated for each drawn pair (``backup_paths_for`` in the log header). Plots of the
minimum separation against d_safe and d_eff, and of the interventions per step. Scrub with the
timeline at the bottom. Needs the ``viz`` extra (rerun-sdk).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

from utm_services.evaluation.decision_log import read_decision_log, separation_rgb

ACCEL_SCALE_M = 2.0      # arrow length per m/s²
TRAIL_STEPS = 150        # 15 s of trail
PROPOSED_RGB, SAFE_RGB = (70, 130, 255), (40, 200, 90)


def log_to_rerun(path: Path, save: Path | None = None) -> None:
    import rerun as rr
    import rerun.blueprint as rrb

    header, steps = read_decision_log(path)
    d_eff, d_safe = header["d_eff_m"], header["d_safe_m"]
    # Start the camera on the whole traffic, from the south-west and above.
    points = np.array([d["pos"] for step in steps for d in step["drones"].values()] or [[500.0, 500.0, 50.0]])
    centre, extent = (points.min(axis=0) + points.max(axis=0)) / 2, float(np.ptp(points[:, :2], axis=0).max()) + 20.0
    eye = rrb.archetypes.EyeControls3D(position=centre + extent * np.array([-0.5, -0.8, 0.6]), look_target=centre,
                                       eye_up=(0.0, 0.0, 1.0))
    blueprint = rrb.Blueprint(rrb.Horizontal(
        rrb.Spatial3DView(origin="world", name=f"{header['episode_id']} (filter {'on' if header['filter_on'] else 'off'})",
                          eye_controls=eye),
        rrb.Vertical(rrb.TimeSeriesView(origin="plots/separation", name="Minimum separation (m)"),
                     rrb.TimeSeriesView(origin="plots/interventions", name="Interventions per step"),
                     rrb.TextDocumentView(origin="about", name="Run")),
        column_shares=[3, 1]))
    rr.init("utm-decision-log", recording_id=header["episode_id"], spawn=save is None, default_blueprint=blueprint)
    if save is not None:
        rr.save(save, default_blueprint=blueprint)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    short = {k: f"{v[:16]}…" if k.endswith("sha256") and v else v for k, v in header.items()}  # full hashes in the log
    rr.log("about", rr.TextDocument("Simulation replay, not live data. Every value comes from the decision log.\n\n"
                                    + "\n".join(f"- **{k}**: {short[k]}" for k in (
        "episode_id", "backend", "truth_source", "filter_on", "scenario_sha256", "script_sha256", "replay_check",
        "backup_paths_for")), media_type="text/markdown"), static=True)
    for name, colour in (("min_sep", (230, 230, 230)), ("d_eff", (255, 160, 0)), ("d_safe", (230, 40, 40))):
        rr.log(f"plots/separation/{name}", rr.SeriesLines(colors=[colour], names=[name]), static=True)
    rr.log("plots/interventions/count", rr.SeriesLines(colors=[(70, 130, 255)], names=["interventions"]), static=True)
    for aid, goal in header["goals_m"].items():
        rr.log(f"world/goals/{aid}", rr.Points3D([goal], radii=0.8, colors=[(150, 150, 150)], labels=[aid]), static=True)

    trails: dict[str, list] = {aid: [] for aid in header["agent_ids"]}
    for step in steps:
        rr.set_time("time", duration=step["t_ms"] / 1000.0)
        drones = step["drones"]
        ids = sorted(drones)
        for aid in ids:
            trails[aid] = (trails[aid] + [drones[aid]["pos"]])[-TRAIL_STEPS:]
        rgb = [separation_rgb(drones[a]["nearest"]["dist_m"] if drones[a]["nearest"] else None, d_eff) for a in ids]
        pos = [drones[a]["pos"] for a in ids]
        labels = [a + (" *" if drones[a]["intervened"] else "") + (" FALLBACK" if drones[a]["fallback_used"] else "")
                  for a in ids]
        rr.log("world/drones", rr.Points3D(pos, radii=0.5, colors=rgb, labels=labels))
        rr.log("world/spheres", rr.Ellipsoids3D(centers=pos, half_sizes=[[d_eff / 2] * 3] * len(ids), colors=rgb,
                                                fill_mode="MajorWireframe"))
        rr.log("world/trails", rr.LineStrips3D([trails[a] for a in ids], radii=0.15, colors=rgb))
        for kind, colour in (("proposed", PROPOSED_RGB), ("safe", SAFE_RGB)):
            rr.log(f"world/accel/{kind}", rr.Arrows3D(
                origins=pos, vectors=[[c * ACCEL_SCALE_M for c in drones[a][kind]] for a in ids], colors=[colour],
                radii=0.12))
        strips, strip_rgb = [], []
        for pair in step["pairs"]:
            for aid, path_m in pair.get("backup", {}).items():
                strips.append([drones[aid]["pos"], *path_m])
                strip_rgb.append(separation_rgb(pair["backup_min_m"], d_eff))
        rr.log("world/backup", rr.LineStrips3D(strips, radii=0.08, colors=strip_rgb) if strips else rr.Clear(recursive=False))
        if step["min_sep_m"] is not None:
            rr.log("plots/separation/min_sep", rr.Scalars(step["min_sep_m"]))
            rr.log("plots/separation/d_eff", rr.Scalars(d_eff))
            rr.log("plots/separation/d_safe", rr.Scalars(d_safe))
        rr.log("plots/interventions/count", rr.Scalars(step["interventions"]))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="utm-viz-rerun", description=__doc__.splitlines()[0])
    parser.add_argument("log", type=Path, help="decisions.jsonl, or the run directory that holds it")
    parser.add_argument("--save", type=Path, help="write an .rrd file instead of opening the viewer")
    args = parser.parse_args(argv)
    path = args.log / "decisions.jsonl" if args.log.is_dir() else args.log
    try:
        import rerun  # noqa: F401
    except ImportError:
        print('utm-viz-rerun: needs rerun-sdk: python -m pip install -e ".[viz]"', file=sys.stderr)
        return 2
    log_to_rerun(path, args.save)
    return 0


if __name__ == "__main__":
    sys.exit(main())
