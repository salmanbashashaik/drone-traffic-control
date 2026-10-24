"""How close do flown tracks come to the scenery buildings? (measurement, not a guarantee)

Buildings are placed clear of every drone's *straight* start-goal route (``city.generate_city``,
``--margin`` metres); the policy and the filter do not know about them. A filter manoeuvre can
swing a drone much further off its straight route than the margin, so a track can come close to,
or pass through, a building. This reports how often, so nobody has to guess.

Suites:
  heldout   50 procedural scenarios (2-4 drones), one city layout each
  shipped   the three scenarios in scenarios/ (head-on-40, crossing-90, sandwich-3), every one
            combined with ``--city-seeds`` different layouts; the flown track does not depend on the
            layout (the controller cannot see it), so each scenario is flown once per variant
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from shared.contracts import ScenarioConfig

from utm_services.policy.learned import LearnedPolicy
from utm_services.simulation.runner import run_episode
from utm_services.training.city import clearance_report, generate_city
from utm_services.training.provenance import git_state
from utm_services.training.scenarios import eval_scenarios

VARIANTS = ("unshielded_marl", "shielded_marl")


def _heldout(policy, a) -> list[dict]:
    rows = []
    for sc in eval_scenarios(a.seed_base, a.episodes, 4, 0.8, 800):
        city = generate_city(sc, a.buildings, a.margin)
        for variant in VARIANTS:
            r = run_episode(sc, variant, policy=policy, policy_id=policy.artifact_id)
            c = clearance_report(r.records, city)
            rows.append({"suite": "heldout", "scenario": sc.scenario_id, "seed": sc.seed, "city_seed": sc.seed,
                         "variant": variant, "n_buildings": c["n_buildings"], "min_clearance_m": c["min_clearance_m"],
                         "samples_inside": c["samples_inside"]})
    return rows


def _shipped(policy, a) -> list[dict]:
    rows = []
    for path in sorted(Path("scenarios").glob("*.json")):
        sc = ScenarioConfig.model_validate_json(path.read_text())
        for variant in VARIANTS:
            r = run_episode(sc, variant, policy=policy, policy_id=policy.artifact_id)
            for cs in range(1, a.city_seeds + 1):
                city = generate_city(sc, a.buildings, a.margin, seed=cs)
                c = clearance_report(r.records, city)
                rows.append({"suite": "shipped", "scenario": sc.scenario_id, "seed": sc.seed, "city_seed": cs,
                             "variant": variant, "n_buildings": c["n_buildings"],
                             "min_clearance_m": c["min_clearance_m"], "samples_inside": c["samples_inside"]})
    return rows


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="utm-city-check", description=__doc__.splitlines()[0])
    p.add_argument("artifact", type=Path)
    p.add_argument("--suite", choices=("heldout", "shipped"), default="heldout")
    p.add_argument("--episodes", type=int, default=50, help="heldout suite: number of scenarios")
    p.add_argument("--city-seeds", type=int, default=20, help="shipped suite: number of city layouts per scenario")
    p.add_argument("--seed-base", type=int, default=12_000_000)
    p.add_argument("--buildings", type=int, default=26)
    p.add_argument("--margin", type=float, default=60.0, help="minimum distance (m) of a building from a straight route")
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args(argv)
    policy = LearnedPolicy.load(a.artifact)
    rows = _heldout(policy, a) if a.suite == "heldout" else _shipped(policy, a)
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "rows.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    (a.out / "manifest.json").write_text(json.dumps({
        "artifact_id": policy.artifact_id, "weights_sha256": policy.metadata["weights_sha256"], "suite": a.suite,
        "episodes": a.episodes, "city_seeds": a.city_seeds, "seed_base": a.seed_base, "buildings": a.buildings,
        "margin_m": a.margin, "code_git": git_state()}, indent=2))
    lines = [f"# Building clearance of flown tracks: `{policy.artifact_id}` ({a.suite})", "",
             (f"{a.buildings} buildings per layout, each at least {a.margin:g} m from every straight start-goal route. "
              "Measurement only: the controller does not see buildings, and the filter can move a drone further off its "
              "straight route than the margin."), "",
             "| scenario | variant | layouts | min clearance (m) | 5th pct | median | layouts where a track enters a building |",
             "|---|---|---|---|---|---|---|"]
    keys = sorted({(r["scenario"] if a.suite == "shipped" else "heldout (all)", r["variant"]) for r in rows})
    for scen, variant in keys:
        rs = [r for r in rows if r["variant"] == variant and (a.suite != "shipped" or r["scenario"] == scen)]
        cl = np.array([r["min_clearance_m"] for r in rs if r["min_clearance_m"] is not None])
        lines.append(f"| {scen} | {variant} | {len(rs)} | {cl.min():.1f} | {np.percentile(cl, 5):.1f} | "
                     f"{np.median(cl):.1f} | {sum(r['samples_inside'] > 0 for r in rs)} |")
    (a.out / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
