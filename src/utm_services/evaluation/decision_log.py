"""`utm-decision-log`: the filter's decisions of one run, step by step, for the viewers.

    utm-decision-log runs/x        (a utm-run or utm-px4-demo output directory)

Reads ``manifest.json`` and ``records.jsonl`` and writes ``decisions.jsonl`` next to them:
one header line, then one line per step. Nothing is estimated or smoothed. Positions and
velocities are the recorded truth (simulator, or the PX4 EKF for PX4 runs); proposed and safe
accelerations are the recorded actions. The filter's internals (conflict clusters, latches,
backup manoeuvres) are recovered by replaying ``apply_filter`` with ``capture_internals`` on the
recorded filter inputs, and every replayed action must equal the recorded one exactly, or no
log is written: the internals shown are the ones that produced the flown actions.

known limit: replay needs a deterministic filter; a filter with a wall-clock solve deadline
(contract v1.5, not on main) can differ on replay, and then the log refuses. Capture live then.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from itertools import combinations
from pathlib import Path
from typing import Any

import numpy as np
from shared.contracts import MAX_SPEED_MPS, SAFE_SEPARATION_M, SIMULATION_STEP_MS, DroneState, MARLAction

from utm_services.safety import filter as F
from utm_services.training.provenance import git_state

LOG_NAME = "decisions.jsonl"
FORMAT = "utm-decision-log/1"


class ReplayMismatch(RuntimeError):
    """The replayed filter did not reproduce a recorded action."""


def sha256_json(obj: Any) -> str:
    """sha256 of canonical JSON (sorted keys, no spaces): the same script hashes the same."""
    return hashlib.sha256(json.dumps(obj, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def backup_paths(p_i: np.ndarray, p_j: np.ndarray, v_i: np.ndarray, v_j: np.ndarray, config: F.FilterConfig,
                 axis: np.ndarray, up_latched: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Absolute positions of both drones of each pair along the filter's backup manoeuvre.

    The filter's rollout (``F._backup_rollout``) keeps only p_i - p_j. This is the same
    rollout in numpy (the oracle of tests/test_filter_rollout_kernel.py) keeping p_i and p_j:
    (K, M, 3) each, NaN after a pair's own end, and the steps each pair flew (M,).
    ``decision_steps`` checks p_i - p_j against the filter's rollout, bit for bit.
    """
    dt = SIMULATION_STEP_MS / 1000.0
    min_steps = F._backup_steps(config)
    steps = max(min_steps, F._backup_max_steps(config))
    used = np.full(len(p_i), steps)
    still = np.ones(len(p_i), dtype=bool)
    closing = np.zeros(len(p_i))

    def cap(v):
        speed = np.linalg.norm(v, axis=1, keepdims=True)
        return v * np.minimum(1.0, MAX_SPEED_MPS / np.maximum(speed, 1e-9))

    path_i, path_j = np.full((steps, len(p_i), 3), np.nan), np.full((steps, len(p_i), 3), np.nan)
    for k in range(steps):
        r = p_i - p_j
        n = r / np.maximum(np.linalg.norm(r, axis=1, keepdims=True), 1e-9)
        up = np.where(np.abs(n[:, 2]) > 1e-3, np.sign(n[:, 2]), 1.0)
        up = np.where(up_latched != 0.0, up_latched, up)
        a_i = F._evasive_direction(n, up, F._world_accel_limits(p_i, v_i, config), axis)
        a_j = F._evasive_direction(-n, -up, F._world_accel_limits(p_j, v_j, config), axis)
        v_i_next, v_j_next = cap(v_i + a_i * dt), cap(v_j + a_j * dt)
        p_i = p_i + 0.5 * (v_i + v_i_next) * dt
        p_j = p_j + 0.5 * (v_j + v_j_next) * dt
        v_i, v_j = v_i_next, v_j_next
        path_i[k, still], path_j[k, still] = p_i[still], p_j[still]
        r = p_i - p_j
        rate = -np.einsum("ij,ij->i", r, v_i - v_j) / np.maximum(np.linalg.norm(r, axis=1), 1e-9)
        closing[still] = rate[still]
        if k + 1 >= min_steps:
            done = still & (closing <= 0.0)
            used[done] = k + 1
            still &= ~done
            if not still.any():
                break
    end = used.max(initial=0)
    return path_i[:end], path_j[:end], used


def separation_rgb(dist_m: float | None, d_eff_m: float) -> tuple[int, int, int]:
    """Viewer colour of a distance: red inside d_eff, red to yellow up to 2 d_eff, to green at 4 d_eff."""
    if dist_m is None or dist_m >= 4.0 * d_eff_m:
        return (40, 200, 90)
    if dist_m < d_eff_m:
        return (230, 40, 40)
    if dist_m < 2.0 * d_eff_m:
        return (230, int(40 + 180 * (dist_m - d_eff_m) / d_eff_m), 40)
    f = (dist_m - 2.0 * d_eff_m) / (2.0 * d_eff_m)
    return (int(230 - 190 * f), int(220 - 20 * f), int(40 + 50 * f))


def _clusters(pairs: list[tuple[str, str]]) -> dict[str, int]:
    """Connected components of the conflict pairs, numbered by their lowest agent id."""
    group = {aid: {aid} for pair in pairs for aid in pair}
    for a, b in pairs:
        if group[a] is not group[b]:
            merged = group[a] | group[b]
            for aid in merged:
                group[aid] = merged
    firsts = sorted({min(g) for g in group.values()})
    return {aid: firsts.index(min(g)) for aid, g in group.items()}


def _xyz(a: np.ndarray) -> list[list[float]]:
    return np.round(a, 3).tolist()  # backup paths to the millimetre: display only


def decision_steps(records: list[dict[str, Any]], config: F.FilterConfig | None) -> list[dict[str, Any]]:
    """One log entry per record; ``config`` None means the run flew without the filter."""
    d_eff = SAFE_SEPARATION_M + (config or F.FilterConfig()).margin_m
    latch: dict = {}
    steps = []
    for rec in records:
        truth = {s["agent_id"]: s for s in rec["states"]}
        proposed = {a["agent_id"]: a for a in rec["proposed_actions"]}
        returned = {a["agent_id"]: a for a in rec["returned_actions"]}
        ids = sorted(truth)
        call = None
        if config is not None:
            observed = [DroneState.model_validate(s) for s in rec["observed_states"]]
            with F.capture_internals() as calls:
                replayed, diag = F.apply_filter(
                    observed, [MARLAction.model_validate(a) for a in rec["proposed_actions"]], config, latch=latch)
            if [a.model_dump(mode="json") for a in replayed] != sorted(rec["returned_actions"],
                                                                     key=lambda a: a["agent_id"]):
                raise ReplayMismatch(f"t={rec['timestamp_ms']} ms: the replayed filter differs from the recorded actions")
            call = calls[0]

        pos = {aid: np.array(truth[aid]["position_m"]) for aid in ids}
        dist = {(a, b): float(np.linalg.norm(pos[a] - pos[b])) for a, b in combinations(ids, 2)}
        drones = {}
        for aid in ids:
            others = [(float(dist[tuple(sorted((aid, o)))]), o) for o in ids if o != aid]
            near = min(others) if others else None
            drones[aid] = {
                "pos": truth[aid]["position_m"], "vel": truth[aid]["velocity_mps"],
                "proposed": proposed[aid]["proposed_accel_mps2"], "safe": returned[aid]["safe_accel_mps2"],
                "intervened": returned[aid]["shield_active"], "fallback_used": returned[aid]["fallback_used"],
                "deadline_fallback": None,  # not on main: per-cluster solve deadlines are contract v1.5
                "cluster": None, "latch": [],
                "nearest": None if near is None else {"id": near[1], "dist_m": near[0]},
            }
        pairs = []
        if call is not None:
            for aid, c in _clusters(list(call["latches"])).items():
                drones[aid]["cluster"] = c
            state = {s.agent_id: s for s in observed}
            for (a, b), frame in call["latches"].items():
                r = np.subtract(state[a].position_m, state[b].position_m)
                side = F._frame_lateral(r / max(float(np.linalg.norm(r)), 1e-9), np.array(frame["axis"]))
                # Drone a's evasive turns along +side and climbs along up; b mirrors both.
                drones[a]["latch"].append({"with": b, "side": side.tolist(), "up": frame["up"]})
                drones[b]["latch"].append({"with": a, "side": (-side).tolist(), "up": -frame["up"]})
            qp = call["pairs_in_qp"]
            if qp and config.barrier == "backup":
                p_i, p_j, v_i, v_j = F._pair_states(state, qp)
                axis = np.array([call["latches"][p]["axis"] if p in call["latches"] else (0.0, 0.0, 0.0) for p in qp])
                up = np.array([call["latches"][p]["up"] if p in call["latches"] else 0.0 for p in qp])
                rel, _, _, used, _ = F._backup_rollout(p_i, p_j, v_i, v_j, config, axis, up)
                if used.tolist() != diag.backup_rollout_steps:
                    raise ReplayMismatch(f"t={rec['timestamp_ms']} ms: backup rollout steps differ from the filter's")
                d_min = np.nan_to_num(np.linalg.norm(rel, axis=2), nan=np.inf).min(axis=0)
                drawn = [k for k, p in enumerate(qp)
                         if p in call["latches"] or p in call["escape_pairs"] or p in diag.pair_slack_mps2]
                path_i = path_j = None
                if drawn:
                    path_i, path_j, _ = backup_paths(p_i[drawn], p_j[drawn], v_i[drawn], v_j[drawn], config,
                                                     axis[drawn], up[drawn])
                    if not np.array_equal(path_i - path_j, rel[: len(path_i), drawn], equal_nan=True):
                        raise ReplayMismatch(f"t={rec['timestamp_ms']} ms: backup paths differ from the filter's rollout")
                for k, (a, b) in enumerate(qp):
                    entry = {"pair": [a, b], "sep_m": dist[(a, b)], "d_eff_m": d_eff,
                             "backup_min_m": float(d_min[k]), "backup_steps": int(used[k])}
                    if k in drawn:
                        q = drawn.index(k)
                        n = int(used[k])
                        entry["backup"] = {a: _xyz(path_i[:n, q]), b: _xyz(path_j[:n, q])}
                    pairs.append(entry)
        steps.append({
            "t_ms": rec["timestamp_ms"],
            "min_sep_m": min(dist.values(), default=None),
            "interventions": sum(d["intervened"] for d in drones.values()),
            "drones": drones,
            "pairs": pairs,
        })
    return steps


def write_decision_log(run_dir: Path) -> Path:
    """Build ``decisions.jsonl`` in ``run_dir`` from its manifest and records; returns its path."""
    manifest = json.loads((run_dir / "manifest.json").read_text())
    records = [json.loads(line) for line in (run_dir / "records.jsonl").read_text().splitlines() if line]
    config = F.FilterConfig(**manifest["filter"]) if manifest.get("filter") else None
    steps = decision_steps(records, config)
    ids = sorted(s["agent_id"] for s in records[0]["states"]) if records else []
    goals = {s["agent_id"]: s["goal_m"] for s in records[0]["states"]} if records else {}
    header = {
        "format": FORMAT,
        "episode_id": manifest.get("episode_id"),
        "scenario_id": manifest.get("scenario_id"),
        "controller_variant": manifest.get("controller_variant"),
        "filter_on": config is not None,
        "backend": manifest.get("backend", "point-mass DroneEnv"),
        "truth_source": "PX4 EKF estimate" if manifest.get("backend") == "px4-sitl-gazebo" else "simulator ground truth",
        "scenario_sha256": manifest.get("scenario_sha256"),
        "script_sha256": manifest.get("script_sha256"),
        "run_code": manifest.get("code"),
        "log_code": git_state(),
        "filter": manifest.get("filter"),
        "d_safe_m": SAFE_SEPARATION_M,
        "d_eff_m": SAFE_SEPARATION_M + (config or F.FilterConfig()).margin_m,
        "dt_s": SIMULATION_STEP_MS / 1000.0,
        "agent_ids": ids,
        "goals_m": goals,
        "steps": len(steps),
        "replay_check": (f"all {len(steps)} replayed filter actions equal the recorded ones" if config is not None
                         else "filter off: no replay"),
        "backup_paths_for": "pairs latched in conflict, escape pairs and pairs with slack; every pair in the QP "
                            "has backup_min_m and backup_steps",
    }
    path = run_dir / LOG_NAME
    with path.open("w") as f:
        f.write(json.dumps(header) + "\n")
        for step in steps:
            f.write(json.dumps(step) + "\n")
    return path


def read_decision_log(path: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """(header, steps) of a decision log."""
    lines = [json.loads(line) for line in path.read_text().splitlines() if line]
    if not lines or lines[0].get("format") != FORMAT:
        raise ValueError(f"{path} is not a {FORMAT} log")
    return lines[0], lines[1:]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="utm-decision-log", description=__doc__.splitlines()[0])
    parser.add_argument("run_dir", type=Path, help="a utm-run or utm-px4-demo output directory")
    args = parser.parse_args(argv)
    try:
        path = write_decision_log(args.run_dir)
    except ReplayMismatch as exc:
        print(f"utm-decision-log: {exc}; no log written", file=sys.stderr)
        return 1
    print(path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
