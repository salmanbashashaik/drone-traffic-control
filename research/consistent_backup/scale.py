"""Per-step filter latency at N = 6, 16 and 64 drones: main, cbk-lite and CBK (P3-SA-02 §6.3).

    PYTHONPATH="src;.;scripts/verify" python -m research.consistent_backup.scale

One configuration at a time, each in its own process (so at most two
processes run), with a hard limit of 10 minutes per configuration. Each starts
from a dense mid-encounter state, flies 5 warm-up filter steps, then 20 timed
closed-loop steps (fuzz.py dynamics, zero nominal). A configuration that hits
the limit is reported as "> limit" with the timed steps it completed.

Then a per-drone cost probe for the decentralised estimate: one drone's
rollouts, one pair's margin matrix and a one-drone QP, timed separately.
Writes results/scale.md and prints it.
"""

from __future__ import annotations

import multiprocessing as mp
import queue
import time
from pathlib import Path

import numpy as np
from shared.contracts import MAX_SPEED_MPS, DroneState, MARLAction
from utm_services.safety import filter as F

from research.consistent_backup import cbk
from research.consistent_backup.harness import D_EFF, DT, _filter

OUT = Path(__file__).parent / "results"
SIZES = ((6, 10.0), (16, 10.0), (64, 40.0))  # (N, spread of the aim points in m): the densest that keeps every pair 6.5 m apart
FILTERS = ("main", "cbk-lite", "cbk")
WARMUP, TIMED, LIMIT_S = 5, 20, 600.0
NEIGHBOURS = (6, 15)  # assumed neighbours in range per drone for the decentralised estimate
CLOSE_M = 20.0


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


def _gaps(p: np.ndarray) -> np.ndarray:
    return np.linalg.norm(p[:, None] - p[None], axis=-1)[np.triu_indices(len(p), 1)]


def dense_state(n: int, spread: float, seed: int = 0) -> tuple[np.ndarray, np.ndarray]:
    """The crowd in free flight at the time (0-6 s) with the most pairs within 20 m, no pair inside d_eff + 1 m."""
    p, v = crowd(n, seed, spread)
    best_k, best_close = 0, -1
    for k in range(61):
        g = _gaps(p + v * k * DT)
        if g.min() >= D_EFF + 1.0 and int((g < CLOSE_M).sum()) > best_close:
            best_k, best_close = k, int((g < CLOSE_M).sum())
    return p + v * best_k * DT, v


def _child(name: str, n: int, spread: float, out: mp.Queue) -> None:
    filt, cfg = _filter(name), F.FilterConfig()
    p, v = dense_state(n, spread)
    g = _gaps(p)
    out.put(("state", int((g < CLOSE_M).sum()), float(g.min())))
    ids = [f"d{i:02d}" for i in range(n)]
    latch: dict = {}
    for k in range(WARMUP + TIMED):
        st = [DroneState(agent_id=ids[i], position_m=tuple(p[i]), velocity_mps=tuple(v[i]),
                         goal_m=(900.0, 900.0, 100.0), timestamp_ms=k * 100) for i in range(n)]
        acts = [MARLAction(agent_id=i, proposed_accel_mps2=(0.0, 0.0, 0.0), timestamp_ms=k * 100) for i in ids]
        t0 = time.perf_counter()
        safe, diag = filt(st, acts, cfg, latch=latch)
        ms = (time.perf_counter() - t0) * 1000.0
        by_id = {s.agent_id: s.safe_accel_mps2 for s in safe}
        a = np.array([by_id[i] for i in ids])
        vn = v + a * DT
        vn *= np.minimum(1, MAX_SPEED_MPS / np.maximum(np.linalg.norm(vn, axis=1, keepdims=True), 1e-9))
        p, v = p + 0.5 * (v + vn) * DT, vn
        out.put(("step", k, ms, bool(diag.fallback_used), float(_gaps(p).min())))
    out.put(("done",))


def measure(name: str, n: int, spread: float) -> dict:
    """Run one configuration in a child process; stop it at LIMIT_S."""
    q: mp.Queue = mp.Queue()
    proc = mp.Process(target=_child, args=(name, n, spread, q), daemon=True)
    t_start = time.monotonic()
    proc.start()
    res = {"filter": name, "n": n, "lat": [], "warm": 0, "fallbacks": 0, "min_sep": np.inf, "status": "limit"}
    while (left := LIMIT_S - (time.monotonic() - t_start)) > 0:
        try:
            msg = q.get(timeout=min(5.0, left))
        except queue.Empty:
            if not proc.is_alive():
                res["status"] = "error"
                break
            continue
        if msg[0] == "state":
            res["close_pairs"], res["start_gap"] = msg[1], msg[2]
        elif msg[0] == "step":
            if msg[1] < WARMUP:
                res["warm"] += 1
            else:
                res["lat"].append(msg[2])
            res["fallbacks"] += int(msg[3])
            res["min_sep"] = min(res["min_sep"], msg[4])
        else:
            res["status"] = "done"
            break
    if proc.is_alive():
        proc.terminate()
    proc.join()
    res["wall_s"] = time.monotonic() - t_start
    return res


def _median_ms(fn, reps: int) -> float:
    ts = []
    for _ in range(reps):
        t0 = time.perf_counter()
        fn()
        ts.append((time.perf_counter() - t0) * 1000.0)
    return float(np.median(ts))


def probe(library: str) -> dict:
    """Per-drone pieces of CBK at the N = 16 dense state: drone 0's option and
    finite-difference rollouts, one pair's margin matrix, and one-drone QPs."""
    cbk.use_library(library)
    cfg = F.FilterConfig()
    p, v = dense_state(16, 10.0)
    n_opts = 1 + len(cbk.CAND_SIGN)
    cand, clock = np.array([0] + list(range(len(cbk.CAND_SIGN)))), np.zeros(n_opts, dtype=int)

    def rollouts():
        traj, first, _ = cbk._rollout(np.repeat(p[:1], n_opts, 0), np.repeat(v[:1], n_opts, 0), cand, clock, cfg)
        probe_a = np.repeat(first[:1], 6, 0) + np.vstack([np.eye(3), -np.eye(3)]) * cbk.FD_MPS2
        cbk._rollout(np.repeat(p[:1], 6, 0), np.repeat(v[:1], 6, 0), np.zeros(6, dtype=int),
                     np.zeros(6, dtype=int), cfg, first=probe_a, steps=len(traj))
        return traj

    t_roll = _median_ms(rollouts, 20)
    traj = cbk._rollout(np.repeat(p[:2], n_opts, 0), np.repeat(v[:2], n_opts, 0), np.tile(cand, 2),
                        np.tile(clock, 2), cfg)[0].reshape(-1, 2, n_opts, 3)

    def pair_margin():
        mid = 0.5 * (p[0] + p[1])
        x, y = traj[:, 0] - mid, traj[:, 1] - mid
        sq = (x * x).sum(-1)[:, :, None] + (y * y).sum(-1)[:, None, :] - 2.0 * x @ y.transpose(0, 2, 1)
        return np.sqrt(np.maximum(sq.min(axis=0), 0.0))

    t_pair = _median_ms(pair_margin, 50)
    rng = np.random.default_rng(0)
    t_qp = {}
    for k in NEIGHBOURS:
        rows = rng.normal(size=(5 * k, 3))
        rows /= np.linalg.norm(rows, axis=1, keepdims=True)
        a_b = rng.uniform(-3.0, 3.0, 3)
        rhs = rows @ a_b - 0.1
        t_qp[k] = _median_ms(lambda: cbk._solve(np.zeros(3), rows, rhs, -np.full(3, 3.5), np.full(3, 3.5), cfg, a_b), 20)
    return {"library": library, "n_opts": n_opts, "t_roll": t_roll, "t_pair": t_pair, "t_qp": t_qp}


def report(rows: list[dict], probes: list[dict]) -> str:
    lines = [f"Latency per filter step: {WARMUP} warm-up steps, then {TIMED} timed closed-loop steps from a dense "
             f"mid-encounter state; limit {LIMIT_S / 60:.0f} min per configuration.", "",
             "| N | Filter | Pairs within 20 m at start | Timed steps | p50 / p95 (ms) | Mean (ms) | Min sep (m) | "
             "Fallbacks | Wall (s) |", "|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        lat = r["lat"]
        if r["status"] == "done":
            pct = f"{np.percentile(lat, 50):.0f} / {np.percentile(lat, 95):.0f}"
            mean = f"{np.mean(lat):.0f}"
        else:
            tag = "> limit" if r["status"] == "limit" else "error"
            pct = f"{tag} ({r['warm']} warm-up, {len(lat)} timed steps done)"
            mean = f"{np.mean(lat):.0f}" if lat else "-"
        lines.append(f"| {r['n']} | {r['filter']} | {r.get('close_pairs', '-')} | {len(lat)}/{TIMED} | {pct} | {mean} | "
                     + (f"{r['min_sep']:.3f}" if np.isfinite(r['min_sep']) else "-") + f" | {r['fallbacks']} | {r['wall_s']:.0f} |")
    lines += ["", "Decentralised per-drone estimate (ESTIMATE, not a measurement of a decentralised filter): "
              "own rollouts + k pair margin matrices + a one-drone QP with 5k rows; selection and messaging not included.", "",
              "| Library | Options | Own rollouts (ms) | One pair margin (ms) | "
              + " | ".join(f"QP k={k} (ms) | Per drone, k={k} (ms)" for k in NEIGHBOURS) + " |",
              "|---|---|---|---|" + "---|---|" * len(NEIGHBOURS)]
    for pr in probes:
        cells = " | ".join(f"{pr['t_qp'][k]:.1f} | {pr['t_roll'] + k * pr['t_pair'] + pr['t_qp'][k]:.0f}" for k in NEIGHBOURS)
        lines.append(f"| {pr['library']} | {pr['n_opts']} | {pr['t_roll']:.1f} | {pr['t_pair']:.2f} | {cells} |")
    return "\n".join(lines)


if __name__ == "__main__":
    assert len(_gaps(np.zeros((3, 3)))) == 3
    rows = []
    for n, spread in SIZES:
        for name in FILTERS:
            rows.append(measure(name, n, spread))
            print(rows[-1]["filter"], n, rows[-1]["status"], len(rows[-1]["lat"]), f"{rows[-1]['wall_s']:.0f}s", flush=True)
    probes = [probe("full"), probe("lite")]
    md = report(rows, probes)
    OUT.mkdir(exist_ok=True)
    (OUT / "scale.md").write_text(md + "\n", encoding="utf-8")
    print(md)
