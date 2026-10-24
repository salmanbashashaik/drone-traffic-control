"""Run the P3-SA-03 matrix and summarise it.

    python -m research.adaptive_margin.run --shard 0 --of 2 --out results/runs.jsonl
    python -m research.adaptive_margin.run --table results/runs*.jsonl

Shards append one JSON line per run and skip runs already in their file, so
an interrupted shard resumes. Every row carries the git commit it ran at
and whether research/adaptive_margin had uncommitted changes.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import time
from collections import defaultdict
from pathlib import Path

from research.adaptive_margin import sim

SEEDS = range(8)
BEYOND_FAMILIES = ("headon2", "ring4")
MODES = ("adaptive", "baseline", "baseline_lag")
# Reference corridor for capacity: 100 m wide, 30 m tall, lanes on a square
# grid at spacing s = SAFE_SEPARATION_M + margin, drones along each lane at s.
CORRIDOR_W_M, CORRIDOR_H_M = 100.0, 30.0


def jobs() -> list[dict]:
    out = [dict(family=f, cond=c.name, seed=s, mode=m) for f in sim.FAMILIES for c in sim.NOMINAL
           for s in SEEDS for m in MODES]
    out += [dict(family=f, cond=c.name, seed=s, mode="adaptive") for c in sim.BEYOND
            for f in BEYOND_FAMILIES for s in SEEDS]
    out += [dict(family=f, cond=c.name, seed=s, mode="adaptive") for c in sim.TAKEOFF
            for f in sim.FAMILIES for s in SEEDS]
    return sorted(out, key=_priority)


# Founder's run order (2026-10-24): the decision cases first, then the other
# beyond-limit cases, then the in-limit matrix. Calm-air baseline runs with
# lag_s = 0 follow the lag_s = 0.25 s ones, as their comparison.
PRIORITY = ("B6_lag_2x_rtk", "B6_lag_2x", "baseline_lag/calm", "baseline/calm",
            "B8_bias_5sigma_rtk", "B8_bias_5sigma")


def _priority(j: dict) -> int:
    calm_baseline = j["mode"] != "adaptive" and j["cond"].endswith("/calm")
    tag = f"{j['mode']}/calm" if calm_baseline else j["cond"]
    if tag in PRIORITY:
        return PRIORITY.index(tag)
    return len(PRIORITY) + (0 if j["cond"].startswith("B") else 1)


def _git() -> dict:
    def git(*args: str) -> str:
        return subprocess.run(["git", *args], capture_output=True, text=True, check=True).stdout.strip()
    return {"git": git("rev-parse", "--short=12", "HEAD"),
            # The run's own output files are not code changes.
            "dirty": bool(git("status", "--porcelain", "--", "research/adaptive_margin",
                              ":(exclude)research/adaptive_margin/results"))}


def _key(j: dict) -> tuple:
    return j["family"], j["cond"], j["seed"], j["mode"]


def run(shard: int, of: int, path: Path) -> None:
    conds = {c.name: c for c in sim.NOMINAL + sim.BEYOND + sim.TAKEOFF}
    done = {_key(json.loads(x)) for x in path.read_text().splitlines()} if path.exists() else set()
    rev = _git()
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
            r.update(j, n=len(p0), ideal_min_sep=rec[sk], wall_s=round(time.perf_counter() - t0, 1), **rev)
            fh.write(json.dumps(r) + "\n")
            fh.flush()


def _first_alarm(r: dict):
    """First alarm time, not counting a headroom alarm already raised at warm-up (founder, 2026-10-24).

    known limit: rows keep only each alarm's first time, so a headroom alarm
    raised at warm-up and raised again later is not counted either; revisit
    if headroom becomes an alarm the design relies on.
    """
    ts = [t for k, t in r["alarms"].items() if not (k == "headroom" and t <= sim.WARMUP_S)]
    return min(ts) if ts else None


def _silent_los(r: dict) -> bool:
    """LoS with no monitor alarm at or before it."""
    first = _first_alarm(r)
    return r["los_s"] is not None and (first is None or first > r["los_s"])


def _kinds(rs: list[dict]) -> str:
    kinds = defaultdict(int)
    for r in rs:
        for k in r["alarms"]:
            kinds[k] += 1
    return ", ".join(f"{k} {v}" for k, v in sorted(kinds.items())) or "-"


def _median(xs: list[float]):
    xs = sorted(xs)
    return xs[len(xs) // 2] if xs else "-"


def _monitored(rs: list[dict]) -> list[str]:
    """Columns shared by the adaptive tables."""
    t0 = sum("wind" in (r["alarms_t0"] or []) for r in rs)
    return [str(sum(bool(r["alarms"]) for r in rs)), str(sum(_silent_los(r) for r in rs)), _kinds(rs),
            f"{t0}", f"{sum(r['absorb_cycles'] > 0 for r in rs)} ({sum(r['absorb_cycles'] for r in rs)}, "
            f"{max(r['absorb_max_m'] for r in rs):.2f})"]


def table(paths: list[Path]) -> str:
    rows = [json.loads(x) for p in paths for x in p.read_text().splitlines()]
    beyond = {c.name for c in sim.BEYOND}
    takeoff = {c.name for c in sim.TAKEOFF}
    revs = sorted({(r["git"], r["dirty"]) for r in rows})
    lines = [f"Rows: {len(rows)}; commits: {', '.join(f'{g}{' (dirty)' if d else ''}' for g, d in revs)}.", "",
             "Absorb: runs with a margin deficit (cycles, largest deficit m). Wind at t0: runs whose first "
             "cycle raised the wind alarm (prior).", "",
             "### In-limit conditions (recoverable encounters only)", "",
             "| condition | mode | runs | LoS | runs with alarm | LoS, monitor silent | alarms | wind at t0 | absorb "
             "| runs saturated | margin p50 (m) | min sep (m) |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    groups = defaultdict(list)
    for r in rows:
        if r["cond"] not in beyond | takeoff and r["ideal_min_sep"] >= sim.SAFE_SEPARATION_M:
            groups[(r["cond"], r["mode"])].append(r)
    for (c, m), rs in sorted(groups.items()):
        mon = _monitored(rs) if m == "adaptive" else ["-"] * 5
        lines.append(
            f"| {c} | {m} | {len(rs)} | {sum(r['los_s'] is not None for r in rs)} | {' | '.join(mon)} | "
            f"{sum(r['saturated_cycles'] > 0 for r in rs)} | "
            f"{_median([r['margin_p50'] for r in rs]) if m == 'adaptive' else 0.5} | {min(r['min_sep'] for r in rs):.2f} |")
    n_unrec = len({(r["family"], r["seed"]) for r in rows if r["ideal_min_sep"] < sim.SAFE_SEPARATION_M})
    lines += ["", f"Scenarios excluded as not recoverable (main, ground truth, no delay): {n_unrec}.", "",
              "### Beyond-limit conditions (adaptive)", "",
              "| case | runs | alarm before any LoS | LoS | runs with alarm | LoS, monitor silent | alarms | wind at t0 "
              "| absorb | first alarm (s, median) | first lag alarm (s, median) |",
              "|---|---|---|---|---|---|---|---|---|---|---|"]
    groups = defaultdict(list)
    for r in rows:
        if r["cond"] in beyond:
            groups[r["cond"]].append(r)
    for c, rs in sorted(groups.items()):
        firsts = [_first_alarm(r) for r in rs]
        before = sum(f is not None and (r["los_s"] is None or f <= r["los_s"]) for f, r in zip(firsts, rs))
        lines.append(
            f"| {c} | {len(rs)} | {before} | {sum(r['los_s'] is not None for r in rs)} | {' | '.join(_monitored(rs))} | "
            f"{_median([f for f in firsts if f is not None])} | "
            f"{_median([r['alarms']['lag'] for r in rs if 'lag' in r['alarms']])} |")
    lines += ["", f"### Takeoff tracks (adaptive, track starts {sim.PREROLL_S:g} s before the run's t = 0, at hover)", "",
              "| condition | runs | LoS | runs with alarm | LoS, monitor silent | alarms | wind at t0 | absorb "
              "| alarm in the last 5 s before the encounter | margin p50 (m) |",
              "|---|---|---|---|---|---|---|---|---|---|"]
    groups = defaultdict(list)
    for r in rows:
        if r["cond"] in takeoff and r["ideal_min_sep"] >= sim.SAFE_SEPARATION_M:
            groups[r["cond"]].append(r)
    for c, rs in sorted(groups.items()):
        late = sum(any(r["t_encounter_s"] - 5.0 <= s_ <= r["t_encounter_s"] for s_ in r["alarms"].values()) for r in rs)
        lines.append(f"| {c} | {len(rs)} | {sum(r['los_s'] is not None for r in rs)} | {' | '.join(_monitored(rs))} | "
                     f"{late} | {_median([r['margin_p50'] for r in rs])} |")
    lines += ["", f"### Corridor capacity per GNSS grade (India wind, adaptive, in-limit runs; reference corridor "
              f"{CORRIDOR_W_M:g} m x {CORRIDOR_H_M:g} m, square lane grid at s = 5 m + margin)", "",
              "| grade | margin p50 of runs (m) | lane spacing s (m) | lanes | drones per km of corridor |",
              "|---|---|---|---|---|"]
    for g in ("rtk_f9p", "sbas_gagan", "gps_sps"):
        rs = [r for r in rows if r["cond"] == f"{g}/india" and r["mode"] == "adaptive"
              and r["ideal_min_sep"] >= sim.SAFE_SEPARATION_M]
        if not rs:
            continue
        m = _median([r["margin_p50"] for r in rs])
        sp = sim.SAFE_SEPARATION_M + m
        lanes = (math.floor(CORRIDOR_W_M / sp) + 1) * (math.floor(CORRIDOR_H_M / sp) + 1)
        lines.append(f"| {g} | {m} | {sp:.1f} | {lanes} | {lanes * math.floor(1000.0 / sp)} |")
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
