"""Closed-loop fuzz of apply_filter on random 2-6 drone encounters.

Nominal = zero acceleration (hold velocity: the worst case for a collision course).
Dynamics replicate DroneEnv._advance (ZOH, speed cap, trapezoid).

Results are collected in the parent process: one line per run in seed order, and one JSONL row per
run appended to the output file as it arrives, so a killed run resumes where it stopped. Rows have
the schema of docs/results/pr12-b1-2026-10-23.jsonl; check them with compare_results.py.
"""
import argparse
import json
import logging
import os
import sys
import time
HARD = os.environ.get('HARD') == '1'
from multiprocessing import Pool

import numpy as np
from shared.contracts import MAX_ACCEL_COMPONENT_MPS2, MAX_SPEED_MPS, DroneState, MARLAction
from utm_services.safety.filter import FilterConfig, _backup_steps, _world_accel_limits, apply_filter

logging.disable(logging.CRITICAL)
DT, D_EFF = 0.1, 5.5
REFERENCE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "docs", "results",
                         "pr12-b1-2026-10-23.jsonl")


def encounter(seed, hard=None):
    hard = HARD if hard is None else hard
    rng = np.random.default_rng(seed)
    n = int(rng.integers(2, 7))
    c = np.array([rng.uniform(250, 750), rng.uniform(250, 750), rng.uniform(70, 160)])
    for _ in range(200):
        p, v = [], []
        for _ in range(n):
            az, el = rng.uniform(0, 2 * np.pi), rng.uniform(-0.5, 0.5)
            d = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
            speed = rng.uniform(2, 15) * 0.9999
            ta = rng.uniform(1.5, 4) if hard else rng.uniform(3, 7)
            miss = rng.normal(size=3) * rng.uniform(0, 6)
            v.append(d * speed)
            p.append(c + miss - d * speed * ta)
        p, v = np.array(p), np.array(v)
        if all(30.5 < z < 199.5 for z in p[:, 2]):
            dmin = min(np.linalg.norm(p[i] - p[j]) for i in range(n) for j in range(i + 1, n))
            if dmin >= (8.0 if hard else 15.0):
                return p, v
    return None


def settled(p, v, latch, diag, steps_left, cfg):
    """Whether the rest of the run can change no recorded field, so it may stop early.

    True when the last filter step left the nominal untouched (no intervention, slack, escape,
    fallback, solver error, world-limit conflict, capped or extended backup rollout), no pair is
    latched, every pair is separating beyond its braking distance under combined braking 2 u_max
    (the filter's braking barrier), and the world-box limits allow zero acceleration at both ends of
    every drone's straight-line path over the remaining steps (they are monotone along it per axis).
    Then every drone flies straight, every pairwise distance grows, and no state leaves the box.
    compare_results.py checks the result against the reference bit for bit.
    """
    if (latch or diag.interventions or diag.fallback_used or not diag.qp_feasible or diag.pair_slack_mps2
            or diag.backup_rollout_cap_pairs or diag.world_limit_conflict_agents
            or max(diag.backup_rollout_steps, default=0) > _backup_steps(cfg)):
        return False
    n = len(p)
    for i in range(n):
        for j in range(i + 1, n):
            r, w = p[i] - p[j], v[i] - v[j]
            if not (r @ w > 0.0 and np.linalg.norm(r) - D_EFF > (w @ w) / (4.0 * MAX_ACCEL_COMPONENT_MPS2)):
                return False
    for q in (p, p + v * steps_left * DT):
        low, high = _world_accel_limits(q, v, cfg)
        if np.any(low > 0.0) or np.any(high < 0.0):
            return False
    return True


def run(args):
    """args = (seed, barrier, steps[, hard[, early_stop]]); hard defaults to $HARD, early_stop to False."""
    seed, barrier, steps = args[:3]
    hard = HARD if len(args) < 4 else args[3]
    early_stop = len(args) > 4 and args[4]
    mode = "hard" if hard else "fuzz"
    enc = encounter(seed, hard)
    if enc is None:
        return {"mode": mode, "seed": seed, "skipped": True}
    p, v = enc
    n = len(p)
    ids = [f"d{i}" for i in range(n)]
    cfg = FilterConfig(barrier=barrier)
    out = {"mode": mode, "seed": seed, "n": n, "fallback_steps": 0, "solver_err_steps": 0, "min_sep": 1e9,
           "exc": None, "first_inside": None, "max_rollout": 0, "cap_hits": 0}
    latch = {}  # one filter memory per run, like the episode runner
    for k in range(steps):
        ts = k * 100
        try:
            states = [DroneState(agent_id=ids[i], position_m=tuple(p[i]), velocity_mps=tuple(v[i]),
                                 goal_m=(900.0, 900.0, 100.0), timestamp_ms=ts) for i in range(n)]
        except Exception as e:  # noqa: BLE001
            out["exc"] = f"state@{k}: {str(e).splitlines()[-1][:120]}"
            break
        acts = [MARLAction(agent_id=i, proposed_accel_mps2=(0.0, 0.0, 0.0), timestamp_ms=ts) for i in ids]
        try:
            safe, diag = apply_filter(states, acts, cfg, latch=latch)
        except Exception as e:  # noqa: BLE001
            out["exc"] = f"filter@{k}: {type(e).__name__}: {e}"[:200]
            break
        out["fallback_steps"] += int(diag.fallback_used)
        out["solver_err_steps"] += int(not diag.qp_feasible)
        out["max_rollout"] = max([out["max_rollout"], *diag.backup_rollout_steps])
        out["cap_hits"] += len(diag.backup_rollout_cap_pairs)
        a = np.array([s.safe_accel_mps2 for s in sorted(safe, key=lambda s: s.agent_id)])
        assert np.all(np.abs(a) <= 3.5 + 1e-9)
        vn = v + a * DT
        sp = np.linalg.norm(vn, axis=1)
        vn = np.where((sp > MAX_SPEED_MPS)[:, None], vn * (MAX_SPEED_MPS / sp * (1 - 1e-12))[:, None], vn)
        p = p + 0.5 * (v + vn) * DT
        v = vn
        d = min(np.linalg.norm(p[i] - p[j]) for i in range(n) for j in range(i + 1, n))
        out["min_sep"] = min(out["min_sep"], d)
        if d < D_EFF and out["first_inside"] is None:
            out["first_inside"] = k + 1
        if early_stop and settled(p, v, latch, diag, steps - k - 1, cfg):
            break
    out["aborts"] = int(out["exc"] is not None)  # fuzz.py has no mission abort (docs/DECISIONS.md)
    return out


def _tasks(tier, n_runs, start, hard, reference):
    if tier is None:
        return [(hard, s) for s in range(start, start + n_runs)]
    if tier == "full":
        return [(h, s) for h in (False, True) for s in range(5000)]
    # quick: seeds 0-199 of both modes, plus the hard seeds that entered d_eff in the reference
    with open(reference) as f:
        inside = sorted({r["seed"] for r in map(json.loads, f) if r["mode"] == "hard" and r["min_sep"] < D_EFF})
    return [(h, s) for h in (False, True) for s in range(200)] + [(True, s) for s in inside if s >= 200]


def _hms(s):
    return time.strftime("%H:%M:%S", time.gmtime(s))


def _summary(rows, barrier):
    for mode in ("fuzz", "hard"):
        res = [r for r in rows if r["mode"] == mode]
        if not res:
            continue
        ok = [r for r in res if not r.get("skipped")]
        print(f"{barrier} {mode}: {len(ok)} encounters ({len(res) - len(ok)} skipped)")
        print(" by n:", {k: sum(r['n'] == k for r in ok) for k in range(2, 7)})
        print(" fallback runs:", [r["seed"] for r in ok if r["fallback_steps"]])
        print(" solver-error runs:", [r["seed"] for r in ok if r["solver_err_steps"]])
        rollouts = [r["max_rollout"] for r in ok]
        print(f" backup rollout steps: max {max(rollouts)}, runs past the minimum: "
              f"{sum(k > _backup_steps(FilterConfig()) for k in rollouts)}, cap hits: {sum(r['cap_hits'] for r in ok)}")
        print(" exceptions:", [(r["seed"], r["exc"]) for r in ok if r["exc"]][:20], sum(bool(r["exc"]) for r in ok))
        bad = sorted([r for r in ok if r["min_sep"] < D_EFF], key=lambda r: r["min_sep"])
        print(f" runs inside d_eff: {len(bad)}; below 5 m: {sum(r['min_sep'] < 5.0 for r in bad)}")
        for r in bad[:30]:
            print(f"   seed {r['seed']} n={r['n']} min_sep={r['min_sep']:.3f} first_inside_step={r['first_inside']}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("runs", nargs="?", type=int, default=5000)
    ap.add_argument("barrier", nargs="?", default="backup")
    ap.add_argument("steps", nargs="?", type=int, default=100)
    ap.add_argument("procs", nargs="?", type=int, default=os.cpu_count())
    ap.add_argument("start", nargs="?", type=int, default=0)
    ap.add_argument("--tier", choices=("quick", "full"),
                    help="quick: seeds 0-199 normal and hard plus the reference's hard seeds inside d_eff; "
                         "full: seeds 0-4999 normal and hard. Overrides runs, start and HARD.")
    ap.add_argument("--out", help="JSONL output file (appended; seeds already in it are skipped)")
    ap.add_argument("--no-early-stop", action="store_true", help="always fly all steps")
    ap.add_argument("--reference", default=REFERENCE, help="results file the quick tier takes its seeds from")
    a = ap.parse_args(argv)

    tasks = _tasks(a.tier, a.runs, a.start, HARD, a.reference)
    name = (f"fuzz_{a.barrier}_{a.tier}.jsonl" if a.tier else
            f"fuzz_{a.barrier}_{a.start}_{a.runs}{'_hard' if HARD else ''}.jsonl")
    path = a.out or os.path.join(os.environ.get("FUZZ_OUT", "."), name)  # default cwd, gitignored
    done = {}
    if os.path.exists(path):
        with open(path) as f:
            for line in f:
                try:
                    r = json.loads(line)
                except json.JSONDecodeError:  # a line cut off by a kill
                    continue
                done[(r["mode"] == "hard", r["seed"])] = r
        # Rewrite without a cut-off tail, so appended rows start on a new line.
        with open(path, "w") as f:
            f.writelines(json.dumps(done[t], sort_keys=True) + "\n" for t in tasks if t in done)
    todo = [t for t in tasks if t not in done]
    print(f"{path}: {len(tasks)} runs, {len(tasks) - len(todo)} already done, {a.procs} processes", flush=True)
    t0 = time.perf_counter()
    with Pool(a.procs) as pool, open(path, "a") as f:
        jobs = [(s, a.barrier, a.steps, h, not a.no_early_stop) for h, s in todo]
        for i, r in enumerate(pool.imap(run, jobs, chunksize=2), 1):
            f.write(json.dumps(r, sort_keys=True) + "\n")
            f.flush()
            done[(r["mode"] == "hard", r["seed"])] = r
            el = time.perf_counter() - t0
            print(f"[{i}/{len(todo)} {_hms(el)} eta {_hms(el / i * (len(todo) - i))}] {r['mode']} seed {r['seed']} "
                  + ("skipped" if r.get("skipped") else
                     f"n={r['n']} min_sep={r['min_sep']:.3f} fallback={r['fallback_steps']} "
                     f"solver_err={r['solver_err_steps']} exc={r['exc']}"), flush=True)
    _summary([done[t] for t in tasks], a.barrier)


if __name__ == "__main__":
    sys.exit(main())
