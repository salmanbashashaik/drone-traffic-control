"""Run the P3-SA-03 matrix and summarise it.

    python -m research.adaptive_margin.run --shard 0 --of 2 --out results/runs.jsonl
    python -m research.adaptive_margin.run --table results/runs*.jsonl

Shards append one JSON line per run and skip runs already in their file, so
an interrupted shard resumes.
"""

from __future__ import annotations

import argparse
import json
import time
from collections import defaultdict
from pathlib import Path

from research.adaptive_margin import sim

SEEDS = range(8)
BEYOND_FAMILIES = ("headon2", "ring4")


def jobs() -> list[dict]:
    out = [dict(family=f, cond=c.name, seed=s, mode=m) for f in sim.FAMILIES for c in sim.NOMINAL
           for s in SEEDS for m in ("adaptive", "baseline")]
    out += [dict(family=f, cond=c.name, seed=s, mode="adaptive") for c in sim.BEYOND
            for f in BEYOND_FAMILIES for s in SEEDS]
    return out


def _key(j: dict) -> tuple:
    return j["family"], j["cond"], j["seed"], j["mode"]


def run(shard: int, of: int, path: Path) -> None:
    conds = {c.name: c for c in sim.NOMINAL + sim.BEYOND}
    done = {_key(json.loads(x)) for x in path.read_text().splitlines()} if path.exists() else set()
    rec: dict[tuple, float] = {}
    with path.open("a") as fh:
        for j in jobs()[shard::of]:
            if _key(j) in done:
                continue
            p0, v0 = sim.scenario(j["family"], j["seed"])
            sk = (j["family"], j["seed"])
            if sk not in rec:
                rec[sk] = sim.recoverable(p0, v0)
            t0 = time.perf_counter()
            r = sim.episode(p0, v0, conds[j["cond"]], j["mode"], j["seed"])
            r.update(j, n=len(p0), ideal_min_sep=rec[sk], wall_s=round(time.perf_counter() - t0, 1))
            fh.write(json.dumps(r) + "\n")
            fh.flush()


def _silent_los(r: dict) -> bool:
    """LoS with no monitor alarm at or before it."""
    return r["los_s"] is not None and (r["first_alarm_s"] is None or r["first_alarm_s"] > r["los_s"])


def table(paths: list[Path]) -> str:
    rows = [json.loads(x) for p in paths for x in p.read_text().splitlines()]
    beyond = {c.name for c in sim.BEYOND}
    lines = ["### In-limit conditions (recoverable encounters only)", "",
             "| condition | mode | runs | LoS | LoS, monitor silent | runs with alarm | alarms | margin p50 (m) | min sep (m) |",
             "|---|---|---|---|---|---|---|---|---|"]
    groups = defaultdict(list)
    for r in rows:
        if r["cond"] not in beyond and r["ideal_min_sep"] >= sim.SAFE_SEPARATION_M:
            groups[(r["cond"], r["mode"])].append(r)
    for (c, m), rs in sorted(groups.items()):
        kinds = defaultdict(int)
        for r in rs:
            for k in r["alarms"]:
                kinds[k] += 1
        margins = sorted(r["margin_p50"] for r in rs)
        lines.append(
            f"| {c} | {m} | {len(rs)} | {sum(r['los_s'] is not None for r in rs)} | "
            f"{sum(_silent_los(r) for r in rs) if m == 'adaptive' else '-'} | "
            f"{sum(bool(r['alarms']) for r in rs) if m == 'adaptive' else '-'} | "
            f"{', '.join(f'{k} {v}' for k, v in sorted(kinds.items())) or '-'} | "
            f"{margins[len(margins) // 2] if m == 'adaptive' else 0.5} | {min(r['min_sep'] for r in rs):.2f} |")
    n_unrec = len({(r["family"], r["seed"]) for r in rows if r["ideal_min_sep"] < sim.SAFE_SEPARATION_M})
    lines += ["", f"Scenarios excluded as not recoverable (main, ground truth, no delay): {n_unrec}.", "",
              "### Beyond-limit conditions (adaptive)", "",
              "| case | runs | runs with alarm | alarm before any LoS | LoS | LoS, monitor silent | first alarm (s, median) | alarms |",
              "|---|---|---|---|---|---|---|---|"]
    groups = defaultdict(list)
    for r in rows:
        if r["cond"] in beyond:
            groups[r["cond"]].append(r)
    for c, rs in sorted(groups.items()):
        kinds = defaultdict(int)
        for r in rs:
            for k in r["alarms"]:
                kinds[k] += 1
        firsts = sorted(r["first_alarm_s"] for r in rs if r["first_alarm_s"] is not None)
        before = sum(r["first_alarm_s"] is not None and (r["los_s"] is None or r["first_alarm_s"] <= r["los_s"])
                     for r in rs)
        lines.append(
            f"| {c} | {len(rs)} | {len(firsts)} | {before} | {sum(r['los_s'] is not None for r in rs)} | "
            f"{sum(_silent_los(r) for r in rs)} | {firsts[len(firsts) // 2] if firsts else '-'} | "
            f"{', '.join(f'{k} {v}' for k, v in sorted(kinds.items())) or '-'} |")
    return "\n".join(lines)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--shard", type=int, default=0)
    ap.add_argument("--of", type=int, default=1)
    ap.add_argument("--out", type=Path)
    ap.add_argument("--table", nargs="*", type=Path)
    a = ap.parse_args()
    if a.table:
        print(table(a.table))
    else:
        run(a.shard, a.of, a.out)
