"""Run the P3-SA-03 matrix and summarise it.

    python -m research.adaptive_margin.run --shard 0 --of 2 --out results/runs.jsonl
    python -m research.adaptive_margin.run --table results/runs*.jsonl
    python -m research.adaptive_margin.run --rerecord results/runs*.jsonl   # ideal_min_sep only

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

from research.adaptive_margin import margin as M
from research.adaptive_margin import sim
from research.adaptive_margin.margin import TARGETS, risk

SEEDS = range(8)
BEYOND_FAMILIES = ("headon2", "ring4")
MODES = ("adaptive", "baseline", "baseline_lag")
# Reference corridor for capacity: 100 m wide, 30 m tall, lanes on a square
# grid at spacing s = SAFE_SEPARATION_M + margin, drones along each lane at s.
CORRIDOR_W_M, CORRIDOR_H_M = 100.0, 30.0


def jobs() -> list[dict]:
    out = [dict(family=f, cond=c.name, seed=s, mode=m, target=t) for t in TARGETS for f in sim.FAMILIES
           for c in sim.NOMINAL for s in SEEDS for m in MODES]
    out += [dict(family=f, cond=c.name, seed=s, mode="adaptive", target=t) for t in TARGETS for c in sim.BEYOND
            for f in BEYOND_FAMILIES for s in SEEDS]
    out += [dict(family=f, cond=c.name, seed=s, mode="adaptive", target=t) for t in TARGETS for c in sim.TAKEOFF
            for f in sim.FAMILIES for s in SEEDS]
    return sorted(out, key=_priority)


# Founder's run order (2026-10-24, after the Fable review): in-limit RTK and
# SBAS adaptive at both targets, then the B6 and B8 cases, then the rest.
PRIORITY_GRADES = ("rtk_f9p/", "sbas_gagan/")
PRIORITY_CASES = ("B6_lag_2x", "B6_lag_2x_rtk", "B8_bias_5sigma", "B8_bias_5sigma_rtk")


def _priority(j: dict) -> int:
    if j["mode"] == "adaptive" and j["cond"].startswith(PRIORITY_GRADES):
        return 0
    return 1 if j["cond"] in PRIORITY_CASES else 2


def _git() -> dict:
    def git(*args: str) -> str:
        return subprocess.run(["git", *args], capture_output=True, text=True, check=True).stdout.strip()
    # The filter file the run actually imported, by content (solver variants, founder 2026-10-24).
    return {"git": git("rev-parse", "--short=12", "HEAD"), "filter_sha": git("hash-object", sim.F.__file__)[:12],
            "step_deadline": sim.F.FilterConfig().__dict__.get("step_deadline"),
            # The run's own output files are not code changes.
            "dirty": bool(git("status", "--porcelain", "--", "research/adaptive_margin",
                              ":(exclude)research/adaptive_margin/results"))}


def _key(j: dict) -> tuple:
    return j["family"], j["cond"], j["seed"], j["mode"], j["target"]


def _selected(j: dict, select: list[str]) -> bool:
    """``select`` holds ``key=a|b`` terms; a job is selected when every term matches."""
    for term in select:
        k, vals = term.split("=", 1)
        if str(j[k]) not in vals.split("|"):
            return False
    return True


def run(shard: int, of: int, path: Path, select: list[str] | None = None) -> None:
    conds = {c.name: c for c in sim.NOMINAL + sim.BEYOND + sim.TAKEOFF}
    done = {_key(json.loads(x)) for x in path.read_text().splitlines()} if path.exists() else set()
    rev = _git()
    rec: dict[tuple, float] = {}
    with path.open("a") as fh:
        for j in [j for j in jobs() if _selected(j, select or [])][shard::of]:
            if _key(j) in done:
                continue
            cond = conds[j["cond"]]
            p0, v0, v_clr = sim.cleared(*sim.scenario(j["family"], j["seed"]), cond, risk(j["target"]), j["seed"])
            sk = (j["family"], j["seed"], v0.tobytes())
            if sk not in rec:
                rec[sk] = sim.recoverable(p0, v0)
            t0 = time.perf_counter()
            r = sim.episode(p0, v0, cond, j["mode"], j["seed"], j["target"])
            r.update(j, n=len(p0), ideal_min_sep=rec[sk], v_clr=[round(float(x), 2) for x in v_clr],
                     wall_s=round(time.perf_counter() - t0, 1), **rev)
            fh.write(json.dumps(r) + "\n")
            fh.flush()


def rerecord(paths: list[Path]) -> None:
    """Recompute ``ideal_min_sep`` of existing rows in place, stamped ``ideal_git`` (re-review R1).

    Rows written before the fix ran ``sim.recoverable`` with the previous
    adaptive episode's drones still in ``rollout.CTX``. Only this column
    depended on it; every other field is left as written.
    """
    conds = {c.name: c for c in sim.NOMINAL + sim.BEYOND + sim.TAKEOFF}
    rev = _git()
    assert not rev["dirty"], "re-record from a clean tree"
    rec: dict[tuple, float] = {}
    for path in paths:
        rows = [json.loads(x) for x in path.read_text().splitlines()]
        for r in rows:
            p0, v0, _ = sim.cleared(*sim.scenario(r["family"], r["seed"]), conds[r["cond"]], risk(r["target"]),
                                    r["seed"])
            sk = (r["family"], r["seed"], v0.tobytes())
            if sk not in rec:
                rec[sk] = sim.recoverable(p0, v0)
            r["ideal_min_sep"], r["ideal_git"] = rec[sk], rev["git"]
        path.write_text("".join(json.dumps(r) + "\n" for r in rows))


# Reports that the margin in force is below the requested one: they say the
# claim is not in force, they do not warn of a fault (founder, 2026-10-24).
NOT_WARNINGS = ("absorb", "margin")


def _first_alarm(r: dict):
    """First time of an alarm that warns of a fault; ``absorb`` and ``margin`` do not count."""
    ts = [t for k, t in r["alarms"].items() if k not in NOT_WARNINGS]
    return min(ts) if ts else None


def _admissible(rows: list[dict]) -> set[tuple]:
    """Scenarios whose adaptive run found them admissible at clearance (review B2); baselines share the flag."""
    return {(r["family"], r["cond"], r["seed"], r["target"]) for r in rows if r["mode"] == "adaptive" and r["admissible"]}


def _in_limit(r: dict, adm: set[tuple]) -> bool:
    return r["ideal_min_sep"] >= sim.SAFE_SEPARATION_M and (r["family"], r["cond"], r["seed"], r["target"]) in adm


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


def _lanes(width: float, m: float, sp: float) -> int:
    """Lanes across ``width`` at spacing ``sp``, each at least the margin ``m`` from the corridor's walls."""
    return max(0, math.floor((width - 2.0 * m) / sp) + 1) if width >= 2.0 * m else 0


def table(paths: list[Path]) -> str:
    rows = [json.loads(x) for p in paths for x in p.read_text().splitlines()]
    beyond = {c.name for c in sim.BEYOND}
    takeoff = {c.name for c in sim.TAKEOFF}
    adm = _admissible(rows)
    revs = sorted({(r["git"], r["dirty"]) for r in rows})
    lines = [f"Rows: {len(rows)}; commits: {', '.join(f'{g}{' (dirty)' if d else ''}' for g, d in revs)}.", "",
             "Risk target per pair-hour in each heading. Absorb: runs with a margin deficit (cycles, largest "
             "deficit m). Wind at t0: runs whose first cycle raised the wind alarm (prior). `absorb` and `margin` "
             "are listed but do not count as a warning before LoS.", ""]
    for target in TARGETS:
        rk = risk(target)
        lines += [f"## Target {target:g} per pair-hour (K1 = {rk.k1:.2f}, K2 = {rk.k2:.2f})", "",
                  "### In-limit conditions (recoverable and admissible encounters only)", "",
                  "| condition | mode | runs | LoS | runs with alarm | LoS, monitor silent | alarms | wind at t0 | absorb "
                  "| runs saturated | margin p50 (m) | GNSS / lost-command term p50 (m) | min sep (m) |",
                  "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
        tr = [r for r in rows if r["target"] == target]
        groups = defaultdict(list)
        for r in tr:
            if r["cond"] not in beyond | takeoff and _in_limit(r, adm):
                groups[(r["cond"], r["mode"])].append(r)
        for (c, m), rs in sorted(groups.items()):
            ad = m == "adaptive"
            mon = _monitored(rs) if ad else ["-"] * 5
            lines.append(
                f"| {c} | {m} | {len(rs)} | {sum(r['los_s'] is not None for r in rs)} | {' | '.join(mon)} | "
                f"{sum(r['saturated_cycles'] > 0 for r in rs)} | "
                f"{_median([r['margin_p50'] for r in rs]) if ad else 0.5} | "
                f"{f'{_median([r['gnss_p50'] for r in rs])} / {_median([r['lost_p50'] for r in rs])}' if ad else '-'} | "
                f"{min(r['min_sep'] for r in rs):.2f} |")
        nominal = [r for r in tr if r["cond"] not in beyond | takeoff and r["mode"] == "adaptive"]
        n_unrec = len({(r["family"], r["seed"], r["cond"]) for r in nominal if r["ideal_min_sep"] < sim.SAFE_SEPARATION_M})
        n_inadm = sum(not r["admissible"] for r in nominal if r["ideal_min_sep"] >= sim.SAFE_SEPARATION_M)
        lines += ["", f"Excluded: {n_unrec} adaptive runs not recoverable (main, ground truth, no delay); "
                  f"{n_inadm} recoverable but not admissible at clearance (separation below 5 m + requested margin "
                  "along the backup).", "",
                  "### Beyond-limit conditions (adaptive)", "",
                  "| case | runs | alarm before any LoS | LoS | runs with alarm | LoS, monitor silent | alarms | wind at t0 "
                  "| absorb | first alarm (s, median) | first lag alarm (s, median) |",
                  "|---|---|---|---|---|---|---|---|---|---|---|"]
        groups = defaultdict(list)
        for r in tr:
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
        for r in tr:
            if r["cond"] in takeoff and _in_limit(r, adm):
                groups[r["cond"]].append(r)
        for c, rs in sorted(groups.items()):
            late = sum(any(r["t_encounter_s"] - 5.0 <= s_ <= r["t_encounter_s"] for s_ in r["alarms"].values())
                       for r in rs)
            lines.append(f"| {c} | {len(rs)} | {sum(r['los_s'] is not None for r in rs)} | {' | '.join(_monitored(rs))} | "
                         f"{late} | {_median([r['margin_p50'] for r in rs])} |")
        lines += ["", f"### Corridor capacity per GNSS grade (India wind, adaptive, in-limit runs; reference corridor "
                  f"{CORRIDOR_W_M:g} m x {CORRIDOR_H_M:g} m, square lane grid at s = 5 m + margin, lanes at least "
                  "the margin from the walls)", "",
                  "| grade | margin p50 of runs (m) | lane spacing s (m) | lanes | drones per km of corridor |",
                  "|---|---|---|---|---|"]
        for g in ("rtk_f9p", "sbas_gagan", "gps_sps"):
            rs = [r for r in tr if r["cond"] == f"{g}/india" and r["mode"] == "adaptive" and _in_limit(r, adm)]
            if not rs:
                continue
            m = _median([r["margin_p50"] for r in rs])
            sp = sim.SAFE_SEPARATION_M + m
            lanes = _lanes(CORRIDOR_W_M, m, sp) * _lanes(CORRIDOR_H_M, m, sp)
            lines.append(f"| {g} | {m} | {sp:.1f} | {lanes} | {lanes * math.floor(1000.0 / sp)} |")
        lines.append("")
    return "\n".join(lines)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--shard", type=int, default=0)
    ap.add_argument("--of", type=int, default=1)
    ap.add_argument("--out", type=Path)
    ap.add_argument("--table", nargs="*", type=Path)
    ap.add_argument("--rerecord", nargs="*", type=Path)
    ap.add_argument("--lost", choices=M.LOST_TERMS, default=M.LOST_TERM)
    ap.add_argument("--freeze-margin", action="store_true", help="diagnostic: hold the margin from clearance on")
    ap.add_argument("--select", nargs="*", default=[], help="key=a|b terms, e.g. cond=sbas_gagan/calm family=ring6")
    ap.add_argument("--no-step-deadline", action="store_true",
                    help="filters with a wall-clock step deadline (PR #26): run without it, so rows do not depend on load")
    a = ap.parse_args()
    M.LOST_TERM, sim.FREEZE_MARGIN = a.lost, a.freeze_margin
    if a.no_step_deadline:
        assert "step_deadline" in sim.F.FilterConfig.__dataclass_fields__, "this filter has no step deadline"
        _init = sim.F.FilterConfig.__init__

        def _no_deadline(self, *args, **kw):
            kw.setdefault("step_deadline", False)
            _init(self, *args, **kw)
        sim.F.FilterConfig.__init__ = _no_deadline
    if a.rerecord:
        rerecord(a.rerecord)
    elif a.table:
        print(table(a.table))
    else:
        run(a.shard, a.of, a.out, a.select)
