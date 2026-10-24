"""Compare a fuzz results file against the reference, per seed.

    python scripts/verify/compare_results.py CANDIDATE.jsonl [--reference REF.jsonl] [--tolerance M]

Rows are matched on (mode, seed). The candidate may cover a subset of the reference (a quick tier);
a candidate row with no reference row is a mismatch.

Exact mode (default, for tooling changes): every field identical, bit for bit.
Tolerance mode (--tolerance 1e-4, for numerical changes): every field except min_sep identical,
the same seeds inside d_eff, and |min_sep - reference| <= tolerance on every seed.

Exits 1 on any mismatch, 0 otherwise.
"""
import argparse
import json
import os
import sys

D_EFF = 5.5
REFERENCE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "docs", "results",
                         "pr12-b1-2026-10-23.jsonl")


def load(path):
    with open(path) as f:
        rows = [json.loads(line) for line in f if line.strip()]
    keyed = {(r["mode"], r["seed"]): r for r in rows}
    if len(keyed) != len(rows):
        raise ValueError(f"{path}: duplicate (mode, seed) rows")
    return keyed


def compare(candidate, reference, tolerance=None):
    """Return (mismatch lines, largest |delta min_sep|) for the rows of candidate."""
    bad, worst = [], 0.0
    for key, c in sorted(candidate.items()):
        r = reference.get(key)
        if r is None:
            bad.append(f"{key}: not in the reference")
            continue
        if tolerance is None:
            diff = sorted(k for k in c.keys() | r.keys() if c.get(k, "<missing>") != r.get(k, "<missing>"))
            if diff:
                bad.append(f"{key}: " + ", ".join(f"{k} {c.get(k, '<missing>')!r} != {r.get(k, '<missing>')!r}"
                                                  for k in diff))
            if "min_sep" in c and "min_sep" in r:
                worst = max(worst, abs(c["min_sep"] - r["min_sep"]))
            continue
        diff = sorted(k for k in (c.keys() | r.keys()) - {"min_sep"}
                      if c.get(k, "<missing>") != r.get(k, "<missing>"))
        if "min_sep" not in c or "min_sep" not in r:
            diff.append("min_sep missing")
        else:
            delta = abs(c["min_sep"] - r["min_sep"])
            worst = max(worst, delta)
            if (c["min_sep"] < D_EFF) != (r["min_sep"] < D_EFF):
                diff.append(f"inside d_eff {c['min_sep'] < D_EFF} != {r['min_sep'] < D_EFF}")
            if not delta <= tolerance:  # also catches NaN
                diff.append(f"min_sep off by {delta:.3g} m")
        if diff:
            bad.append(f"{key}: " + ", ".join(diff))
    return bad, worst


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("candidate")
    ap.add_argument("--reference", default=REFERENCE)
    ap.add_argument("--tolerance", type=float, help="min_sep tolerance in m (default: exact)")
    a = ap.parse_args(argv)
    candidate, reference = load(a.candidate), load(a.reference)
    bad, worst = compare(candidate, reference, a.tolerance)
    modes = {m: sum(k[0] == m for k in candidate) for m in sorted({k[0] for k in candidate})}
    print(f"{a.candidate}: {len(candidate)} of {len(reference)} reference rows {modes}, "
          f"mode {'exact' if a.tolerance is None else f'tolerance {a.tolerance:g} m'}")
    print(f"max |delta min_sep| = {worst:.3g} m; mismatched rows: {len(bad)}")
    for line in bad[:50]:
        print("  " + line)
    print("PASS" if not bad else "FAIL")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
