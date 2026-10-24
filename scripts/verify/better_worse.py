"""Per-seed better/worse report of a fuzz results file against a reference, as Markdown.

    python scripts/verify/better_worse.py CANDIDATE.jsonl [--reference REF.jsonl] [--worse 0.05]

Per mode (normal and hard fuzz): runs inside d_eff, below 5 m, with fallbacks, with solver errors and
with exceptions in each file; seeds entering and leaving d_eff; seeds better and worse by more than
1 cm; the worst runs; and every seed whose min separation drops by more than --worse metres, with its
start geometry from fuzz.encounter (drone count, closest start distance, the closest pair's closing
speed, miss distance at constant velocity and angle between headings, altitude and wall distance).
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from compare_results import D_EFF, REFERENCE, load
from fuzz import encounter

WORLD_MIN = np.array([0.0, 0.0, 30.0])
WORLD_MAX = np.array([1000.0, 1000.0, 200.0])


def geometry(seed, hard):
    p, v = encounter(seed, hard)
    n = len(p)
    pairs = [(i, j) for i in range(n) for j in range(i + 1, n)]
    i, j = min(pairs, key=lambda ij: np.linalg.norm(p[ij[0]] - p[ij[1]]))
    r, w = p[i] - p[j], v[i] - v[j]
    t = max(0.0, -float(r @ w) / max(float(w @ w), 1e-9))
    heading = np.degrees(np.arccos(np.clip(v[i] @ v[j] / max(np.linalg.norm(v[i]) * np.linalg.norm(v[j]), 1e-9), -1, 1)))
    # closest constant-velocity approach over all pairs
    miss = min(np.linalg.norm((p[a] - p[b]) + (v[a] - v[b]) * max(0.0, -float((p[a] - p[b]) @ (v[a] - v[b]))
               / max(float((v[a] - v[b]) @ (v[a] - v[b])), 1e-9))) for a, b in pairs)
    wall = float(np.min(np.minimum(p - WORLD_MIN, WORLD_MAX - p)))
    return (f"n={n}, closest start {np.linalg.norm(r):.1f} m (d{i}-d{j}, closing {-float(r @ w) / np.linalg.norm(r):.1f} m/s, "
            f"headings {heading:.0f}°, CPA in {t:.1f} s), min CV miss {miss:.2f} m, z {p[:, 2].min():.0f}-{p[:, 2].max():.0f} m, "
            f"nearest wall {wall:.0f} m")


def report(cand, ref, worse_m):
    lines = []
    for mode in ("fuzz", "hard"):
        keys = sorted(k for k in ref if k[0] == mode and not ref[k].get("skipped") and k in cand)
        if not keys:
            continue
        r = {k: ref[k] for k in keys}
        c = {k: cand[k] for k in keys}
        lines.append(f"\n### {'normal' if mode == 'fuzz' else 'hard'} fuzz ({len(keys)} runs)\n")
        lines.append("| | reference | candidate |\n| --- | --- | --- |")
        for label, test in (("runs inside d_eff", lambda x: x["min_sep"] < D_EFF), ("runs below 5 m", lambda x: x["min_sep"] < 5.0),
                            ("runs with fallback steps", lambda x: x["fallback_steps"] > 0),
                            ("runs with solver errors", lambda x: x["solver_err_steps"] > 0),
                            ("runs with exceptions", lambda x: bool(x["exc"])),
                            ("runs with rollout cap hits", lambda x: x["cap_hits"] > 0)):
            lines.append(f"| {label} | {sum(map(test, r.values()))} | {sum(map(test, c.values()))} |")
        for label, field in (("fallback steps", "fallback_steps"), ("solver-error steps", "solver_err_steps")):
            lines.append(f"| {label} (total) | {sum(x[field] for x in r.values())} | {sum(x[field] for x in c.values())} |")
        lines.append(f"| worst min sep (m) | {min(x['min_sep'] for x in r.values()):.3f} | {min(x['min_sep'] for x in c.values()):.3f} |")
        lines.append(f"| 1st percentile min sep (m) | {np.percentile([x['min_sep'] for x in r.values()], 1):.3f} | "
                     f"{np.percentile([x['min_sep'] for x in c.values()], 1):.3f} |")
        delta = {k: c[k]["min_sep"] - r[k]["min_sep"] for k in keys}
        lines.append(f"\nPer seed: {sum(d > 0.01 for d in delta.values())} better by more than 1 cm, "
                     f"{sum(d < -0.01 for d in delta.values())} worse by more than 1 cm, "
                     f"{sum(abs(d) <= 0.01 for d in delta.values())} within 1 cm; "
                     f"{sum(abs(d) <= 1e-4 for d in delta.values())} within 1e-4 m. Largest gain {max(delta.values()):+.3f} m, "
                     f"largest loss {min(delta.values()):+.3f} m.")
        entered = [k[1] for k in keys if c[k]["min_sep"] < D_EFF <= r[k]["min_sep"]]
        left = [k[1] for k in keys if r[k]["min_sep"] < D_EFF <= c[k]["min_sep"]]
        lines.append(f"Entering d_eff: {entered or 'none'}. Leaving d_eff: {left or 'none'}.")
        inside = sorted((k for k in keys if min(c[k]["min_sep"], r[k]["min_sep"]) < D_EFF), key=lambda k: c[k]["min_sep"])
        if inside:
            lines.append("\nRuns inside d_eff in either file:\n\n| seed | n | reference m | candidate m | change m |\n| --- | --- | --- | --- | --- |")
            lines += [f"| {k[1]} | {c[k]['n']} | {r[k]['min_sep']:.3f} | {c[k]['min_sep']:.3f} | {delta[k]:+.3f} |" for k in inside]
        bad = sorted((k for k in keys if delta[k] < -worse_m), key=lambda k: delta[k])
        lines.append(f"\nSeeds worse by more than {worse_m} m: {len(bad)}\n")
        if bad:
            lines.append("| seed | reference m | candidate m | change m | first step inside d_eff (ref / cand) | start geometry |\n| --- | --- | --- | --- | --- | --- |")
            lines += [f"| {k[1]} | {r[k]['min_sep']:.3f} | {c[k]['min_sep']:.3f} | {delta[k]:+.3f} | "
                      f"{r[k]['first_inside']} / {c[k]['first_inside']} | {geometry(k[1], mode == 'hard')} |" for k in bad]
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("candidate")
    ap.add_argument("--reference", default=REFERENCE)
    ap.add_argument("--worse", type=float, default=0.05, help="list seeds worse by more than this (m)")
    a = ap.parse_args(argv)
    print(report(load(a.candidate), load(a.reference), a.worse))
    return 0


if __name__ == "__main__":
    sys.exit(main())
