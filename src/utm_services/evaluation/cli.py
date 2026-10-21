"""`utm-run`: run one scenario file through a controller variant and save the run.

Writes ``manifest.json``, ``summary.json`` and ``records.jsonl`` (one per-step
record per line) to ``--out``. ``--plot`` also writes ``replay.png``: top-down
tracks and the minimum pairwise separation over time. The MARL variants use the
reference controller as a labelled stand-in until a trained policy exists.
"""

from __future__ import annotations

import argparse
import json
import sys
from itertools import combinations
from pathlib import Path
from typing import Any

import numpy as np
from pydantic import ValidationError
from shared.contracts import SAFE_SEPARATION_M, ScenarioConfig

from utm_services.policy.reference import ReferencePolicy
from utm_services.simulation.env import PROFILES, DroneEnv
from utm_services.simulation.runner import VARIANTS, run_episode


def plot_replay(records: list[dict[str, Any]], path: Path, title: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    tracks: dict[str, list[tuple[float, float]]] = {}
    times, min_sep = [], []
    for rec in records:
        pos = {s["agent_id"]: s["position_m"] for s in rec["states"]}
        for aid, p in pos.items():
            tracks.setdefault(aid, []).append((p[0], p[1]))
        seps = [float(np.linalg.norm(np.subtract(pos[a], pos[b]))) for a, b in combinations(pos, 2)]
        if seps:
            times.append(rec["timestamp_ms"] / 1000.0)
            min_sep.append(min(seps))

    fig, (top, sep) = plt.subplots(1, 2, figsize=(12, 5))
    for aid, xy in tracks.items():
        x, y = zip(*xy, strict=True)
        top.plot(x, y, label=aid)
        top.plot(x[0], y[0], "o", color=top.lines[-1].get_color())
    top.set(xlabel="x (m)", ylabel="y (m)", title=f"{title}: tracks (top-down)", aspect="equal")
    top.legend(fontsize="small")
    sep.plot(times, min_sep)
    sep.axhline(SAFE_SEPARATION_M, color="red", linestyle="--", label=f"d_safe = {SAFE_SEPARATION_M:g} m")
    sep.set(xlabel="t (s)", ylabel="min pairwise separation (m)", title="Minimum separation", ylim=(0, None))
    sep.legend(fontsize="small")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="utm-run", description=__doc__.splitlines()[0])
    parser.add_argument("scenario", type=Path, help="ScenarioConfig JSON file, e.g. scenarios/head-on-40.json")
    parser.add_argument("--variant", choices=VARIANTS, default="shielded_marl")
    parser.add_argument("--profile", choices=PROFILES, default="ideal",
                        help="disturbance profile: ideal (T1), nominal or realistic (T2 placeholders; default: ideal)")
    parser.add_argument("--out", type=Path, default=Path("runs"), help="output directory (default: runs)")
    parser.add_argument("--plot", action="store_true", help="also write replay.png (needs the 'plot' extra)")
    args = parser.parse_args(argv)

    try:
        config = ScenarioConfig.model_validate_json(args.scenario.read_text())
    except (OSError, ValidationError) as exc:
        print(f"utm-run: invalid scenario {args.scenario}: {exc}", file=sys.stderr)
        return 2

    result = run_episode(config, args.variant, policy=ReferencePolicy(), policy_id="reference-standin",
                         make_env=lambda rules: DroneEnv(rules, disturbances=PROFILES[args.profile]))
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "manifest.json").write_text(json.dumps(result.manifest, indent=2, default=str))
    (args.out / "summary.json").write_text(json.dumps(result.summary, indent=2))
    with (args.out / "records.jsonl").open("w") as f:
        for rec in result.records:
            f.write(json.dumps(rec) + "\n")
    if args.plot:
        plot_replay(result.records, args.out / "replay.png", result.manifest["episode_id"])

    print(json.dumps(result.summary, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
