"""Per-step filter latency at N = 6, 16 and 64 drones, main against CBK (P3-SA-02 §6).

    PYTHONPATH="src;.;scripts/verify" python -m research.consistent_backup.scale [procs] [filter ...]

Crowd encounters: N drones on straight courses through random points near a
common centre, zero nominal, fuzz.py dynamics. N = 6 uses the hard-fuzz
generator itself. Writes results/scale_<filters>.md and prints it.
"""

from __future__ import annotations

import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from research.consistent_backup.evaluate import fuzz
from research.consistent_backup.harness import kinematic

OUT = Path(__file__).parent / "results"
# (N, seeds, steps, spread of the aim points in m)
SIZES = ((6, (2704, 3146, 4828, 977), 100, None), (16, (0, 1, 2), 80, 20.0), (64, (0,), 40, 60.0))


def crowd(n: int, seed: int, spread: float) -> tuple[np.ndarray, np.ndarray]:
    """N drones that would pass near (500, 500, 115) within 2-6 s, at least 10 m apart at the start."""
    rng = np.random.default_rng(seed)
    c = np.array([500.0, 500.0, 115.0])
    for _ in range(1000):
        az, el = rng.uniform(0, 2 * np.pi, n), rng.uniform(-0.3, 0.3, n)
        d = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], axis=1)
        speed = rng.uniform(5, 15, n)[:, None] * 0.9999
        aim = c + rng.normal(size=(n, 3)) * spread * np.array([1.0, 1.0, 0.3])
        p = aim - d * speed * rng.uniform(2, 6, n)[:, None]
        gaps = np.linalg.norm(p[:, None] - p[None], axis=-1) + np.eye(n) * 1e9
        if np.all((p[:, 2] > 30.5) & (p[:, 2] < 199.5)) and gaps.min() >= 10.0:
            return p, d * speed
    raise RuntimeError(f"no crowd for n={n} seed={seed}")


def run(job: tuple) -> dict:
    name, n, seed, steps, spread = job
    p, v = fuzz.encounter(seed) if spread is None else crowd(n, seed, spread)
    return {"filter": name, "n": n, "seed": seed, **kinematic(name, p, v, steps)}


if __name__ == "__main__":
    procs = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    filters = sys.argv[2:] or ["main", "cbk", "cbk-lite"]
    assert procs <= 2, "at most two processes: the CPU is shared with timing work"
    jobs = [(f, n, s, steps, spread) for n, seeds, steps, spread in SIZES for f in filters for s in seeds]
    with Pool(procs) as pool:
        rows = pool.map(run, jobs, chunksize=1)
    lines = ["| N | Filter | Runs x steps | p50 / p95 (ms) | Min sep (m) | Inside d_eff (s) | Fallbacks | Reversals (max/run) |",
             "|---|---|---|---|---|---|---|---|"]
    for n, seeds, steps, _ in SIZES:
        for f in filters:
            rs = [r for r in rows if r["n"] == n and r["filter"] == f]
            lat = np.concatenate([r["_latency_ms"] for r in rs])
            lines.append(f"| {n} | {f} | {len(rs)} x {steps} | {np.percentile(lat, 50):.0f} / {np.percentile(lat, 95):.0f} | "
                         f"{min(r['min_sep'] for r in rs):.3f} | {sum(r['inside_s'] for r in rs):.1f} | "
                         f"{sum(r['fallbacks'] for r in rs)} | {max(r['reversals'] for r in rs)} |")
    md = "\n".join(lines)
    OUT.mkdir(exist_ok=True)
    (OUT / f"scale_{'_'.join(filters)}.md").write_text(md + "\n")
    print(md)
