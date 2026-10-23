"""Closed-loop runners for comparing main's ``apply_filter`` with the CBK prototype.

Two loops, both with a pluggable filter of the ``apply_filter`` signature:

- ``kinematic``: the fuzz / vnoise / hnoise dynamics (zero nominal, 100 ms
  zero-order hold, speed cap, trapezoid), optional position noise drawn
  exactly as ``scripts/verify/vnoise.py`` draws it, so main reproduces the
  verify baselines.
- ``env_episode``: ``DroneEnv`` with the reference policy, the same loop as
  ``utm_services.simulation.runner.run_episode`` minus records.

Per run they return: min separation, time inside d_eff, fallback steps,
solver-error steps, contradictory-direction steps (main's latch only; CBK has
one backup per drone, so none by construction), direction reversals and the
per-step filter latencies.
"""

from __future__ import annotations

import logging
import time

import numpy as np
from shared.contracts import (
    MAX_SPEED_MPS,
    SAFE_SEPARATION_M,
    AirspaceRulesV1,
    DroneState,
    MARLAction,
    ScenarioConfig,
)
from utm_services.policy.adapter import run_policy
from utm_services.policy.reference import compute_reference_action
from utm_services.safety import filter as F
from utm_services.simulation.env import DroneEnv

from research.consistent_backup import cbk

logging.disable(logging.CRITICAL)
DT = 0.1
D_EFF = SAFE_SEPARATION_M + F.FilterConfig().margin_m
FILTERS = {"main": F.apply_filter, "cbk": cbk.apply_filter, "cbk-lite": cbk.apply_filter}
REV_MPS2 = 0.5  # deviations below this are not counted as a direction (vnoise.py threshold)


def contradictory(latch: dict, pos: dict[str, np.ndarray]) -> bool:
    """The PR #12 re-review metric on main's latch: a drone squeezed between two
    latched neighbours (cosine of the directions to them below -0.7) whose two
    latches push it in roughly opposite world directions (dot below -0.5)."""
    lateral: dict[str, list[tuple[str, np.ndarray]]] = {}
    for (x, y), frame in latch.items():
        if x not in pos or y not in pos:
            continue
        r = pos[x] - pos[y]
        n = r / max(float(np.linalg.norm(r)), 1e-9)
        side = F._frame_lateral(n, frame.axis)
        lateral.setdefault(x, []).append((y, side))
        lateral.setdefault(y, []).append((x, -side))
    for b, entries in lateral.items():
        for k, (a, la) in enumerate(entries):
            for c, lc in entries[k + 1:]:
                to_a, to_c = pos[a] - pos[b], pos[c] - pos[b]
                cos = float(to_a @ to_c) / max(float(np.linalg.norm(to_a) * np.linalg.norm(to_c)), 1e-9)
                if cos < -0.7 and float(la @ lc) < -0.5:
                    return True
    return False


def _filter(name: str):
    """The filter called ``name``; "cbk-lite" is CBK with the pruned library."""
    if name.startswith("cbk"):
        cbk.use_library("lite" if name == "cbk-lite" else "full")
    return FILTERS[name]


class _Tally:
    """Per-run metrics shared by both loops."""

    def __init__(self, ids: list[str], rev_pair_axes: tuple[int, ...] | None):
        self.ids, self.rev_pair_axes = ids, rev_pair_axes
        self.min_sep, self.inside_steps, self.fallbacks, self.solver_errors = np.inf, 0, 0, 0
        self.contradictory, self.reversals, self.latency_ms, self.takeovers = 0, 0, [], 0
        self.prev: dict = {}

    def separation(self, p: np.ndarray, held: bool) -> None:
        n = len(p)
        if n < 2:
            return
        d = min(float(np.linalg.norm(p[i] - p[j])) for i in range(n) for j in range(i + 1, n))
        self.min_sep = min(self.min_sep, d)
        self.inside_steps += int(held and d < D_EFF)

    def filter_step(self, diag, latch: dict, name: str, dev: np.ndarray, pos: np.ndarray, ms: float) -> None:
        self.latency_ms.append(ms)
        self.fallbacks += int(diag.fallback_used)
        self.solver_errors += int(not diag.qp_feasible)
        if name == "main":
            self.contradictory += int(contradictory(latch, dict(zip(self.ids, pos, strict=True))))
        else:
            self.takeovers += int(bool(diag.cbk["takeover"]))
        # Reversals: vnoise.py / hnoise.py metric for two-drone rows (relative
        # deviation on the given axes); otherwise each drone's own deviation.
        if self.rev_pair_axes is not None:
            vecs = {"pair": dev[0, list(self.rev_pair_axes)] - dev[1, list(self.rev_pair_axes)]}
        else:
            vecs = dict(zip(self.ids, dev, strict=True))
        for key, h in vecs.items():
            if np.linalg.norm(h) > REV_MPS2:
                prev = self.prev.get(key)
                if prev is not None and h @ prev < 0:
                    self.reversals += 1
                self.prev[key] = h

    def result(self) -> dict:
        lat = np.array(self.latency_ms) if self.latency_ms else np.zeros(1)
        return {
            "min_sep": round(float(self.min_sep), 3), "inside_s": round(self.inside_steps * DT, 1),
            "fallbacks": self.fallbacks, "solver_errors": self.solver_errors,
            "contradictory": self.contradictory, "reversals": self.reversals, "takeovers": self.takeovers,
            "p50_ms": round(float(np.percentile(lat, 50)), 2), "p95_ms": round(float(np.percentile(lat, 95)), 2),
            "_latency_ms": self.latency_ms,
        }


def kinematic(name: str, p, v, steps: int, sigma: float | None = None, seed: int = 0,
              rev_pair_axes: tuple[int, ...] | None = None) -> dict:
    """Zero-nominal closed loop with fuzz.py dynamics. With ``sigma`` the filter
    sees p + N(0, sigma) from ``default_rng(seed)``, drawn as in vnoise.py."""
    filt, cfg = _filter(name), F.FilterConfig()
    p, v = np.array(p, dtype=float), np.array(v, dtype=float)
    n = len(p)
    ids = [f"d{i}" for i in range(n)]
    rng = np.random.default_rng(seed)
    latch: dict = {}
    tally = _Tally(ids, rev_pair_axes)
    for k in range(steps):
        obs = p + rng.normal(0, sigma, p.shape) if sigma is not None else p
        st = [DroneState(agent_id=ids[i], position_m=tuple(obs[i]), velocity_mps=tuple(v[i]),
                         goal_m=(900.0, 900.0, 100.0), timestamp_ms=k * 100) for i in range(n)]
        acts = [MARLAction(agent_id=i, proposed_accel_mps2=(0.0, 0.0, 0.0), timestamp_ms=k * 100) for i in ids]
        t0 = time.perf_counter()
        safe, diag = filt(st, acts, cfg, latch=latch)
        ms = (time.perf_counter() - t0) * 1000.0
        by_id = {s.agent_id: s.safe_accel_mps2 for s in safe}
        a = np.array([by_id[i] for i in ids])
        tally.filter_step(diag, latch, name, a, p, ms)
        vn = v + a * DT
        speed = np.linalg.norm(vn, axis=1, keepdims=True)
        vn *= np.minimum(1, MAX_SPEED_MPS / np.maximum(speed, 1e-9))
        p = p + 0.5 * (v + vn) * DT
        v = vn
        tally.separation(p, held=k + 1 < steps)
    return tally.result()


def env_episode(name: str, drones: list[dict], steps: int = 400, rules: AirspaceRulesV1 | None = None) -> dict:
    """DroneEnv with the reference policy and the given filter (runner.run_episode loop)."""
    filt, cfg = _filter(name), F.FilterConfig()
    env = DroneEnv(rules)
    sc = ScenarioConfig(scenario_id="cbk-eval", seed=0, episode_limit_steps=steps, drones=drones)
    observations, _ = env.reset(sc)
    latch: dict = {}
    ids = sorted(d["agent_id"] for d in drones)
    tally = _Tally(ids, None)
    while not env.done:
        ts = env.timestamp_ms
        active = sorted(env.states(), key=lambda s: s.agent_id)
        tally.ids = [s.agent_id for s in active]  # drones that reached their goal leave the env
        truth = np.array([s.position_m for s in active])
        tally.separation(truth, held=True)
        observed = env.observed_states()
        proposed = [run_policy(observations[s.agent_id].root, s.agent_id, ts, compute_reference_action)
                    for s in observed]
        t0 = time.perf_counter()
        safe, diag = filt(observed, proposed, cfg, latch=latch)
        ms = (time.perf_counter() - t0) * 1000.0
        by_id = {s.agent_id: s.safe_accel_mps2 for s in safe}
        a = np.array([by_id[i] for i in tally.ids])
        nom = np.array([x.proposed_accel_mps2 for x in sorted(proposed, key=lambda x: x.agent_id)])
        tally.filter_step(diag, latch, name, a - nom, truth, ms)
        observations = env.step(safe).observations
    final = [s.position_m for s in env.states()]
    if final:
        tally.separation(np.array(final), held=False)
    out = tally.result()
    out["aborted"] = env.summary().get("n_aborted")
    return out
