"""How close do flown tracks come to the scenery buildings over many scenarios? (measurement, not a guarantee)

Buildings are placed clear of every drone's straight route (``city.generate_city``); the policy and
the filter do not know about them. This flies the learned policy through many procedural scenarios
with a city and reports the minimum distance between any drone and any building, so we can see how
far a filter manoeuvre or a policy detour moves a track off its straight route.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from utm_services.policy.learned import LearnedPolicy
from utm_services.simulation.runner import run_episode
from utm_services.training.city import clearance_report, generate_city
from utm_services.training.scenarios import eval_scenarios


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="utm-city-check", description=__doc__.splitlines()[0])
    p.add_argument("artifact", type=Path)
    p.add_argument("--episodes", type=int, default=50)
    p.add_argument("--seed-base", type=int, default=12_000_000)
    p.add_argument("--buildings", type=int, default=26)
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args(argv)
    policy = LearnedPolicy.load(a.artifact)
    rows = []
    for sc in eval_scenarios(a.seed_base, a.episodes, 4, 0.8, 800):
        city = generate_city(sc, a.buildings)
        for variant in ("unshielded_marl", "shielded_marl"):
            r = run_episode(sc, variant, policy=policy, policy_id=policy.artifact_id)
            c = clearance_report(r.records, city)
            rows.append({"seed": sc.seed, "variant": variant, "n_buildings": c["n_buildings"],
                         "min_clearance_m": c["min_clearance_m"], "samples_inside": c["samples_inside"],
                         "completed": r.summary["n_completed"], "launched": r.summary["n_launched"]})
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "rows.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    lines = [f"# Building clearance of flown tracks: `{policy.artifact_id}`", "",
             (f"{a.episodes} scenarios (seeds {a.seed_base}+, 2-4 drones), {a.buildings} buildings each, "
              "placed >= 35 m from every straight start-goal route. Measurement only; the controller does not see buildings."), "",
             "| variant | episodes | buildings placed (min) | min clearance (m) | 5th pct | median | episodes with a track inside a building |",
             "|---|---|---|---|---|---|---|"]
    for v in ("unshielded_marl", "shielded_marl"):
        rs = [r for r in rows if r["variant"] == v]
        cl = np.array([r["min_clearance_m"] for r in rs if r["min_clearance_m"] is not None])
        lines.append(f"| {v} | {len(rs)} | {min(r['n_buildings'] for r in rs)} | {cl.min():.1f} | "
                     f"{np.percentile(cl, 5):.1f} | {np.median(cl):.1f} | {sum(r['samples_inside'] > 0 for r in rs)} |")
    (a.out / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
