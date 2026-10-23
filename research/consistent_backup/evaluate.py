"""Compare main's filter with the CBK prototype on the P3-SA-02 cases.

Run from the repo root with at most two processes (the CPU is shared):

    PYTHONPATH="src;.;scripts/verify" python -m research.consistent_backup.evaluate [procs] [filter ...]

Writes one JSON line per run to research/consistent_backup/results/runs_<filters>.jsonl
and the per-case table to table_<filters>.md, and prints the table. Grouped cases (bent bearings, noise seeds)
report the minimum separation over runs, totals of inside time, fallbacks,
solver errors and contradictory steps, the maximum reversals of any one run,
and latency percentiles over all their steps.
"""

from __future__ import annotations

import json
import os
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

os.environ["HARD"] = "1"  # fuzz.encounter reads it at import
import fuzz  # noqa: E402  (scripts/verify)

from research.consistent_backup.harness import env_episode, kinematic  # noqa: E402

OUT = Path(__file__).parent / "results"
# Hard-fuzz seeds inside d_eff on main (docs/results/pr12-final-report.md, B1 round).
HARD_SEEDS = (3810, 3614, 3947, 3932, 2704, 252, 1177, 3146, 370, 2876, 4828, 4239, 977)


def drone(aid, pos, goal, vel=(0.0, 0.0, 0.0)) -> dict:
    """A drone entry as in scripts/verify/targeted.py."""
    return {"agent_id": aid, "initial_position_m": tuple(map(float, pos)),
            "initial_velocity_mps": tuple(map(float, vel)), "goal_m": tuple(map(float, goal))}


def env_cases() -> dict[str, list[dict]]:
    """targeted.py sandwiches, rings and vertical encounters (reference policy in DroneEnv)."""
    cases = {}
    for tag, z in (("mid", 100.0), ("floor", 30.5), ("ceiling", 199.5)):
        cases[f"sandwich {tag}"] = [drone("d1", (490, 500, z), (900, 500, z), (10, 0, 0)),
                                    drone("d2", (500, 500, z), (500, 900, z)),
                                    drone("d3", (510, 500, z), (100, 500, z), (-10, 0, 0))]
    cases["sandwich vertical"] = [drone("a", (500, 500, 90), (500, 500, 190), (0, 0, 10)),
                                  drone("b", (500, 500, 100), (900, 500, 100)),
                                  drone("c", (500, 500, 110), (500, 500, 35), (0, 0, -10))]
    cases["vertical head-on"] = [drone("a", (500, 500, 60), (500, 500, 190), (0, 0, 10)),
                                 drone("b", (500, 500, 120), (500, 500, 35), (0, 0, -10))]
    for n in (3, 4, 5, 6):
        for r, spd in ((40.0, 10.0), (60.0, 14.99)):
            ds = []
            for i in range(n):
                th = 2 * np.pi * i / n
                p = np.array([500 + r * np.cos(th), 500 + r * np.sin(th), 100])
                u = -(p - [500, 500, 100]) / r
                ds.append(drone(f"d{i}", p, np.array([500, 500, 100]) + u * 300, u * spd))
            cases[f"ring n={n} r={r:.0f} v={spd:.0f}"] = ds
    return cases


def probe(theta: float, bend: float, dc: float, jitter: float) -> tuple[list, list]:
    """Reviewer probes (b1review p1b_diag.py, p1g_bends.py): b hovers, a from 30 m and
    c from dc m at 10 m/s, c's bearing bent by ``bend`` from the line through a and b."""
    t1, t2 = np.radians(theta), np.radians(theta + 180.0 - bend)
    u1, u2 = np.array([np.cos(t1), np.sin(t1), 0.0]), np.array([np.cos(t2), np.sin(t2), 0.0])
    w = np.array([-np.sin(t1), np.cos(t1), 0.0])
    c0 = np.array([500.0, 500.0, 100.0])
    return [c0 + 30 * u1 + jitter * w, c0, c0 + dc * u2 + jitter * w], [-10 * u1, np.zeros(3), -10 * u2]


def jobs(filters: list[str]) -> list[tuple]:
    """(filter, case, kind, kwargs) for every run."""
    out = []
    for name in filters:
        for case, ds in env_cases().items():
            out.append((name, case, "env", {"drones": ds}))
        for kind, dc in (("simultaneous", 30.0), ("staged", 50.0)):
            for th in (0.0, 45.0, 225.0):
                p, v = probe(th, 0.0, dc, 0.05)
                out.append((name, f"{kind} theta={th:.0f}", "kin", {"p": p, "v": v, "steps": 90}))
        for bend in (10.0, 20.0, 45.0, 60.0):
            for th in np.arange(0.0, 180.0, 12.0):
                p, v = probe(float(th), bend, 50.0, 0.0)
                out.append((name, f"bent {bend:.0f} (15 bearings)", "kin", {"p": p, "v": v, "steps": 90}))
        for seed in HARD_SEEDS:
            p, v = fuzz.encounter(seed)
            out.append((name, f"hard seed {seed} n={len(p)}", "kin", {"p": p, "v": v, "steps": 100}))
        for tag, p, v, axes in (("vnoise", [(500, 500, 70.0), (500, 500, 130.0)], [(0, 0, 10.0), (0, 0, -10.0)], (0, 1)),
                                ("hnoise", [(470, 500, 100.0), (530, 500, 100.0)], [(10.0, 0, 0), (-10.0, 0, 0)], (1, 2))):
            for sigma, seeds in ((0.0, (0,)), (0.5, range(20)), (1.5, range(20))):
                for seed in seeds:
                    out.append((name, f"{tag} sigma={sigma}", "kin",
                                {"p": p, "v": v, "steps": 120, "sigma": sigma, "seed": seed, "rev_pair_axes": axes}))
    return out


def run(job: tuple) -> dict:
    name, case, kind, kw = job
    res = env_episode(name, kw["drones"]) if kind == "env" else kinematic(name, **kw)
    return {"filter": name, "case": case, **res}


def table(rows: list[dict]) -> str:
    """Markdown table: one line per (case, filter), grouped runs aggregated."""
    groups: dict[tuple[str, str], list[dict]] = {}
    for r in rows:
        groups.setdefault((r["case"], r["filter"]), []).append(r)
    lines = ["| Case | Filter | Runs | Min sep (m) | Inside d_eff (s) | Fallbacks | Solver errors | "
             "Contradictory steps | Reversals (max/run) | p50 / p95 (ms) |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for (case, name), rs in groups.items():
        lat = np.concatenate([r["_latency_ms"] for r in rs])
        lines.append(
            f"| {case} | {name} | {len(rs)} | {min(r['min_sep'] for r in rs):.3f} | "
            f"{sum(r['inside_s'] for r in rs):.1f} | {sum(r['fallbacks'] for r in rs)} | "
            f"{sum(r['solver_errors'] for r in rs)} | "
            f"{sum(r['contradictory'] for r in rs) if name == 'main' else '0 (one backup per drone)'} | "
            f"{max(r['reversals'] for r in rs)} | {np.percentile(lat, 50):.0f} / {np.percentile(lat, 95):.0f} |")
    return "\n".join(lines)


if __name__ == "__main__":
    procs = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    filters = sys.argv[2:] or ["main", "cbk"]
    assert procs <= 2, "at most two processes: the CPU is shared with timing work"
    with Pool(procs) as pool:
        rows = pool.map(run, jobs(filters), chunksize=1)
    OUT.mkdir(exist_ok=True)
    with open(OUT / f"runs_{'_'.join(filters)}.jsonl", "w") as fh:
        for r in rows:
            fh.write(json.dumps({k: v for k, v in r.items() if k != "_latency_ms"}) + "\n")
    order = list(dict.fromkeys(j[1] for j in jobs(filters[:1])))
    rows.sort(key=lambda r: (order.index(r["case"]), r["filter"]))
    md = table(rows)
    (OUT / f"table_{'_'.join(filters)}.md").write_text(md + "\n")
    print(md)
