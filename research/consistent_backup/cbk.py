"""Consistent-backup (CBK) safety filter prototype (docs/formulation/P3-SA-02).

Drop-in replacement for ``utm_services.safety.filter.apply_filter`` with the
same signature and return types. It imports from the main package and does
not modify it. Research code: nothing here is used by ``src/``.

Each drone owns one backup, a time-varying backup controller in the sense of
Singletary et al. (IROS 2022): fly a world-frame manoeuvre for a fixed time,
then brake every axis to hover. The backup depends only on the drone's own
state, as in Chen, Singletary and Ames (IEEE CSL 2021), so every pair row of a
drone reads the same trajectory, and "every drone flies its backup" satisfies
all rows and the world box at once (BL-03). The manoeuvre of each drone is
chosen from a small library by a deterministic coordinate ascent on the
joint pair margins, and kept, with its clock, in the caller's latch.
"""

from __future__ import annotations

import itertools
import logging
import math
from collections.abc import Sequence

import numpy as np
import osqp
import scipy.sparse as sp
from shared.contracts import (
    MAX_ACCEL_COMPONENT_MPS2,
    MAX_SPEED_MPS,
    SAFE_SEPARATION_M,
    SHIELD_INTERVENTION_TOLERANCE_MPS2,
    SIMULATION_STEP_MS,
    DroneState,
    MARLAction,
    SafeAction,
    exceeds_intervention_tolerance,
)
from utm_services.safety.filter import (
    FilterConfig,
    FilterDiagnostics,
    _backup_max_steps,
    _barrier,
    _separation_sq,
    _world_accel_limits,
    _world_limit_conflicts,
)

logger = logging.getLogger(__name__)

DT = SIMULATION_STEP_MS / 1000.0
U_MAX = MAX_ACCEL_COMPONENT_MPS2

# Manoeuvre library: index 0 brakes every axis to hover; the others push each
# non-zero axis of a world-frame sign vector at u_max for a fixed duration
# (zero axes brake), then brake to hover. World-frame directions have no
# line-of-sight frame, so they cannot flip where the line of sight crosses
# x = +-y (BL-11).
MANOEUVRE_STEPS = (10, 20)
_SIGNS = [s for s in itertools.product((-1.0, 0.0, 1.0), repeat=3) if any(s)]
CAND_SIGN = np.array([(0.0, 0.0, 0.0)] + [s for _ in MANOEUVRE_STEPS for s in _SIGNS])
CAND_STEPS = np.array([0] + [k for k in MANOEUVRE_STEPS for _ in _SIGNS])
# Pruned library for large N: the 8 horizontal directions and straight up or
# down, 2 s only (11 candidates instead of 53).
_LITE_SIGNS = [s for s in _SIGNS if s[2] == 0.0] + [(0.0, 0.0, 1.0), (0.0, 0.0, -1.0)]
LIBRARIES = {"full": ((10, 20), _SIGNS), "lite": ((20,), _LITE_SIGNS)}


def use_library(name: str) -> None:
    """Switch the module's candidate library (research runs only; not thread-safe)."""
    global MANOEUVRE_STEPS, CAND_SIGN, CAND_STEPS
    MANOEUVRE_STEPS, signs = LIBRARIES[name]
    CAND_SIGN = np.array([(0.0, 0.0, 0.0)] + [s for _ in MANOEUVRE_STEPS for s in signs])
    CAND_STEPS = np.array([0] + [k for k in MANOEUVRE_STEPS for _ in signs])

# Selection score: min over pairs of min(margin, BAND_M) plus SUM_WEIGHT times
# the sum. Pairs further than BAND_M above d_eff do not steer the selection.
BAND_M = 3.0
SUM_WEIGHT = 0.01
# A drone leaves its current manoeuvre only when that manoeuvre is no longer
# certified (some pair margin below 0) and another option scores this much
# better (Singletary et al. 2022, Algorithms 1 and 2, with a noise band).
SWITCH_M = 0.5
SWEEPS = 3
# Rows at the closest-approach step k* and this many steps either side (BL-04).
WINDOW = 2
# Finite-difference step for the per-drone sensitivities d p(k) / d a_0.
FD_MPS2 = 0.1


def _horizon_steps(config: FilterConfig) -> tuple[int, int]:
    """(minimum, cap) rollout steps: the longest manoeuvre, then braking from v_max to hover."""
    brake = math.ceil(MAX_SPEED_MPS / U_MAX / DT)
    min_steps = max(MANOEUVRE_STEPS) + brake + math.ceil((config.lag_s + 0.5) / DT)
    return min_steps, max(min_steps, _backup_max_steps(config))


def _rollout(
    p: np.ndarray, v: np.ndarray, cand: np.ndarray, clock: np.ndarray, config: FilterConfig,
    first: np.ndarray | None = None, steps: int | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Fly R per-drone backups from (p, v), (R, 3) each, in 100 ms zero-order-hold steps.

    ``cand`` (R,) and ``clock`` (R,) give each rollout's manoeuvre and how many
    of its steps are already flown. ``first`` (R, 3) replaces the first
    acceleration (finite differences). Each step is clipped to the drone's
    world-box limits and the speed cap, with the simulator's trapezoid update.
    Without ``steps`` the rollout runs the minimum horizon, then until every
    rollout hovers, at most the cap. Returns positions after each step
    (K, R, 3), the first accelerations (R, 3) and, per rollout, whether it
    still moved at the end (R,).
    """
    min_steps, cap_steps = _horizon_steps(config)
    total = cap_steps if steps is None else steps
    sign, dur = CAND_SIGN[cand], CAND_STEPS[cand]
    traj = np.empty((total, len(p), 3))
    a_first = None
    for k in range(total):
        brake = -np.sign(v) * np.minimum(U_MAX, np.abs(v) / DT)
        evade = ((clock + k) < dur)[:, None] & (sign != 0.0)
        a = np.clip(np.where(evade, U_MAX * sign, brake), *_world_accel_limits(p, v, config))
        if k == 0:
            if first is not None:
                a = first
            a_first = a
        v_next = v + a * DT
        speed = np.linalg.norm(v_next, axis=1, keepdims=True)
        v_next = v_next * np.minimum(1.0, MAX_SPEED_MPS / np.maximum(speed, 1e-9))
        p = p + 0.5 * (v + v_next) * DT
        v = v_next
        traj[k] = p
        if steps is None and k + 1 >= min_steps and not np.any(np.abs(v) > 1e-9):
            return traj[: k + 1], a_first, np.zeros(len(p), dtype=bool)
    return traj, a_first, np.any(np.abs(v) > 1e-9, axis=1)


def _select(
    margins: dict[tuple[int, int], np.ndarray], n_drones: int, n_opts: int, certified_lock: np.ndarray,
) -> tuple[np.ndarray, float]:
    """Deterministic coordinate ascent over drones in id order; option 0 is each drone's current one.

    ``margins[(i, j)]`` (n_opts, n_opts) is min_k |p_i - p_j| - d_eff for
    every option pair, clipped at BAND_M. A drone moves to the option with
    the best own score (own minimum plus SUM_WEIGHT times own sum), but only
    if its own minimum does not drop. Pairs without the drone are unchanged,
    so the global minimum margin never drops below the warm start's.
    ``certified_lock`` (N,) marks drones still inside a manoeuvre: they keep
    it while its own minimum is at least 0. Leaving the current option needs
    SWITCH_M of own score. Returns the options per drone.
    """
    clipped = {pair: np.minimum(m, BAND_M) for pair, m in margins.items()}
    own_pairs = {i: [(pair, pair[0] == i) for pair in clipped if i in pair] for i in range(n_drones)}
    assign = np.zeros(n_drones, dtype=int)

    def own_values(i: int) -> np.ndarray:
        """(n_opts, own pairs): drone i's pair margins for each of its options."""
        cols = [clipped[pair][:, assign[pair[1]]] if first else clipped[pair][assign[pair[0]], :]
                for pair, first in own_pairs[i]]
        return np.stack(cols, axis=1)

    for _ in range(SWEEPS):
        changed = False
        for i in range(n_drones):
            if not own_pairs[i]:
                continue
            values = own_values(i)
            own_min = values.min(axis=1)
            score = own_min + SUM_WEIGHT * values.sum(axis=1)
            o_now = assign[i]
            if o_now == 0 and certified_lock[i] and own_min[0] >= 0.0:
                continue
            need = SWITCH_M if o_now == 0 else 1e-9
            ok = (own_min >= own_min[o_now]) & (score > score[o_now] + need)
            if ok.any():
                # argmax returns the lowest index among ties: deterministic.
                assign[i] = int(np.argmax(np.where(ok, score, -np.inf)))
                changed = True
        if not changed:
            break
    return assign


def _validate(states: Sequence[DroneState], actions: Sequence[MARLAction]) -> int:
    """The input checks of ``apply_filter``; returns the step timestamp."""
    state_ids = [s.agent_id for s in states]
    action_ids = [a.agent_id for a in actions]
    if len(set(state_ids)) != len(state_ids):
        raise ValueError(f"Duplicate agent IDs in states: {state_ids}")
    if len(set(action_ids)) != len(action_ids):
        raise ValueError(f"Duplicate agent IDs in actions: {action_ids}")
    if set(state_ids) != set(action_ids):
        raise ValueError(
            f"State and action agent ID sets do not match. "
            f"States: {sorted(state_ids)}, Actions: {sorted(action_ids)}"
        )
    action_timestamps = {a.timestamp_ms for a in actions}
    if len(action_timestamps) != 1:
        raise ValueError(f"All actions must share the same timestamp, got: {action_timestamps}")
    step_ts = next(iter(action_timestamps))
    for s in states:
        if s.timestamp_ms != step_ts:
            raise ValueError(
                f"State timestamp {s.timestamp_ms} ms for agent '{s.agent_id}' "
                f"does not match action timestamp {step_ts} ms"
            )
    return step_ts


def _solve(
    nominal: np.ndarray, rows: np.ndarray, rhs: np.ndarray, low: np.ndarray, high: np.ndarray,
    config: FilterConfig, backup: np.ndarray,
) -> tuple[np.ndarray | None, np.ndarray]:
    """min ||a - a_nom||^2 + w sum(s) s.t. rows a + s >= rhs, s >= 0, low <= a <= high.

    Same OSQP settings as ``_solve_hocbf_qp`` (fixed adaptive-rho interval,
    polish, alpha 1.6 then 1.0). If both fail, retry at rho 10, main's setting
    for a saturated box, warm-started at the backup action, which is feasible:
    near the floor and ceiling, and when the rows leave little more than the
    backup feasible, rho 0.1 stops at max_iter.
    Returns (a or None on solver error, slack).
    """
    dim, n_rows = len(nominal), len(rhs)
    P = sp.block_diag([sp.eye(dim) * 2.0, sp.csc_matrix((n_rows, n_rows))], format="csc")
    q = np.concatenate([-2.0 * nominal, np.full(n_rows, config.slack_weight)])
    A = sp.vstack([
        sp.hstack([sp.csc_matrix(rows), sp.eye(n_rows)]),
        sp.hstack([sp.eye(dim), sp.csc_matrix((dim, n_rows))]),
        sp.hstack([sp.csc_matrix((n_rows, dim)), sp.eye(n_rows)]),
    ], format="csc")
    lo = np.concatenate([rhs, low, np.zeros(n_rows)])
    hi = np.concatenate([np.full(n_rows, np.inf), high, np.full(n_rows, np.inf)])
    for rho, alpha in ((0.1, 1.6), (0.1, 1.0), (10.0, 1.6), (10.0, 1.0)):
        prob = osqp.OSQP()
        prob.setup(P, q, A, lo, hi, verbose=False, eps_abs=config.solver_eps, eps_rel=config.solver_eps,
                   max_iter=config.solver_max_iter, adaptive_rho_interval=25, polish=True, rho=rho, alpha=alpha)
        if rho == 10.0:
            prob.warm_start(x=np.concatenate([backup, np.zeros(n_rows)]))
        res = prob.solve()
        if res.info.status == "solved":
            return np.clip(res.x[:dim], -U_MAX, U_MAX), res.x[dim:]
        logger.warning("cbk: OSQP status %s at rho=%.1f alpha=%.1f", res.info.status, rho, alpha)
    return None, np.zeros(n_rows)


def apply_filter(
    states: Sequence[DroneState],
    actions: Sequence[MARLAction],
    config: FilterConfig | None = None,
    *,
    latch: dict,
) -> tuple[list[SafeAction], FilterDiagnostics]:
    """Consistent-backup filter for one step; same contract as the main ``apply_filter``.

    ``latch`` holds, per drone, its current manoeuvre and how many of its steps
    were flown. ``FilterConfig.barrier`` and ``tie_break_mps2`` are ignored.
    The diagnostics carry an extra attribute ``cbk`` (selected manoeuvre,
    clock, switches, slack takeovers, the minimum certified margin) for the
    research runner.
    """
    if not isinstance(latch, dict):
        raise TypeError(f"latch must be the caller's per-episode dict (UTM-077), got {type(latch).__name__}")
    config = config or FilterConfig()
    if not states and not actions:
        return [], FilterDiagnostics()
    step_ts = _validate(states, actions)

    state_map = {s.agent_id: s for s in states}
    action_map = {a.agent_id: a for a in actions}
    ids = sorted(state_map)
    n = len(ids)
    pos = np.array([state_map[a].position_m for a in ids], dtype=np.float64)
    vel = np.array([state_map[a].velocity_mps for a in ids], dtype=np.float64)
    nominal = np.array([action_map[a].proposed_accel_mps2 for a in ids], dtype=np.float64)
    d_eff = SAFE_SEPARATION_M + config.margin_m

    diag = FilterDiagnostics()
    pairs = []
    for i, j in itertools.combinations(range(n), 2):
        diag.n_pairs_total += 1
        h = _barrier(state_map[ids[i]], state_map[ids[j]])
        diag.h_min_m2 = h if diag.h_min_m2 is None else min(diag.h_min_m2, h)
        if config.range_gate_m is None or math.sqrt(_separation_sq(state_map[ids[i]], state_map[ids[j]])) <= config.range_gate_m:
            pairs.append((i, j))
    diag.n_pairs_in_qp = len(pairs)
    conflicts = _world_limit_conflicts(pos, vel, config)
    diag.world_limit_conflict_agents = [aid for aid, c in zip(ids, conflicts, strict=True) if c]
    for i, j in itertools.combinations(range(n), 2):
        dist = float(np.linalg.norm(pos[i] - pos[j]))
        if dist < d_eff:
            diag.pair_violation_depth_m[(ids[i], ids[j])] = d_eff - dist

    for aid in set(latch) - set(ids):
        del latch[aid]
    current = [latch.get(aid, (0, 0)) for aid in ids]
    low, high = _world_accel_limits(pos, vel, config)

    # Options per drone: 0 = its current manoeuvre with its clock, then every
    # library manoeuvre fresh. Rolled out together; one horizon for all.
    n_opts = 1 + len(CAND_SIGN)
    opt_cand = np.array([[c] + list(range(len(CAND_SIGN))) for c, _ in current]).ravel()
    opt_clock = np.array([[k] + [0] * len(CAND_SIGN) for _, k in current]).ravel()
    traj, first, moving = _rollout(np.repeat(pos, n_opts, 0), np.repeat(vel, n_opts, 0), opt_cand, opt_clock, config)
    steps = len(traj)
    traj = traj.reshape(steps, n, n_opts, 3)
    first = first.reshape(n, n_opts, 3)
    moving = moving.reshape(n, n_opts)

    # Exact prune: a pair whose every option pair stays BAND_M clear of d_eff
    # has the constant clipped margin BAND_M, so the selection is unchanged.
    reach = np.linalg.norm(traj - pos[None, :, None, :], axis=-1).max(axis=(0, 2))
    margins = {}
    for i, j in pairs:
        if float(np.linalg.norm(pos[i] - pos[j])) - reach[i] - reach[j] - d_eff >= BAND_M:
            margins[(i, j)] = np.full((n_opts, n_opts), BAND_M)
            continue
        # |x - y|^2 = |x|^2 + |y|^2 - 2 x.y as one batched matmul; centring on
        # the pair's midpoint keeps the cancellation error far below 1 mm.
        mid = 0.5 * (pos[i] + pos[j])
        x, y = traj[:, i] - mid, traj[:, j] - mid
        sq = (x * x).sum(-1)[:, :, None] + (y * y).sum(-1)[:, None, :] - 2.0 * x @ y.transpose(0, 2, 1)
        margins[(i, j)] = np.sqrt(np.maximum(sq.min(axis=0), 0.0)) - d_eff
    lock = np.array([CAND_STEPS[c] > k for c, k in current])
    assign = _select(margins, n, n_opts, lock)

    chosen = [(int(opt_cand.reshape(n, n_opts)[i, o]), int(opt_clock.reshape(n, n_opts)[i, o]))
              for i, o in enumerate(assign)]
    a_backup = first[np.arange(n), assign]
    path = traj[:, np.arange(n), assign]  # (K, N, 3): one trajectory per drone, shared by all its rows

    # Per-drone sensitivities J[k] = d p(k) / d a_0 by central differences of the chosen backup.
    probe = np.repeat(a_backup, 6, 0) + np.tile(np.vstack([np.eye(3), -np.eye(3)]) * FD_MPS2, (n, 1))
    fd, _, _ = _rollout(np.repeat(pos, 6, 0), np.repeat(vel, 6, 0), np.repeat([c for c, _ in chosen], 6),
                        np.repeat([k for _, k in chosen], 6), config, first=probe, steps=steps)
    fd = fd.reshape(steps, n, 6, 3)
    jac = (fd[:, :, :3, :] - fd[:, :, 3:, :]).transpose(0, 1, 3, 2) / (2.0 * FD_MPS2)  # (K, N, 3 pos, 3 accel)

    rows, rhs, row_pair = [], [], []
    min_margin = math.inf
    for i, j in pairs:
        r = path[:, i] - path[:, j]
        dist = np.linalg.norm(r, axis=1)
        k_star = int(np.argmin(dist))
        min_margin = min(min_margin, float(dist[k_star]) - d_eff)
        for k in range(max(0, k_star - WINDOW), min(steps, k_star + WINDOW + 1)):
            m = r[k] / max(float(dist[k]), 1e-9)
            scale = (k + 0.5) * DT * DT
            row = np.zeros(3 * n)
            row[3 * i: 3 * i + 3] = m @ jac[k, i] / scale
            row[3 * j: 3 * j + 3] = -(m @ jac[k, j]) / scale
            rows.append(row)
            rhs.append(float(row[3 * i: 3 * i + 3] @ a_backup[i] + row[3 * j: 3 * j + 3] @ a_backup[j])
                       - config.gamma * DT * max(float(dist[k]) - d_eff, 0.0) / scale)
            row_pair.append((i, j))

    diag.backup_rollout_steps = [steps] * len(pairs)
    capped = moving[np.arange(n), assign]
    diag.backup_rollout_cap_pairs = [(ids[i], ids[j]) for i, j in pairs if capped[i] or capped[j]]

    takeover: set[int] = set()
    if rows:
        rows_a, rhs_a = np.array(rows), np.array(rhs)
        # The joint backup satisfies every row and the box: the QP is feasible by construction.
        assert np.all(rows_a @ a_backup.ravel() >= rhs_a - 1e-6)
        solved, slack = _solve(nominal.ravel(), rows_a, rhs_a, low.ravel(), high.ravel(), config, a_backup.ravel())
        diag.qp_feasible = solved is not None
        if solved is None:
            # Solver error: every drone in a row flies its backup; the rest keep the nominal in the box.
            accel = np.clip(nominal, low, high)
            in_rows = sorted({d for pair in row_pair for d in pair})
            accel[in_rows] = a_backup[in_rows]
            diag.fallback_used = True
            takeover = set(in_rows)
        else:
            accel = solved.reshape(n, 3)
            for (i, j), s in zip(row_pair, slack, strict=True):
                if s > SHIELD_INTERVENTION_TOLERANCE_MPS2:
                    diag.pair_slack_mps2[(ids[i], ids[j])] = max(diag.pair_slack_mps2.get((ids[i], ids[j]), 0.0), float(s))
                    takeover |= {i, j}
            # Linearisation error left slack: those drones fly their certified
            # backup, and the takeover grows until no row mixing a backup and
            # a QP action is violated (like ``_fallback_cluster``).
            while takeover:
                for d in takeover:
                    accel[d] = a_backup[d]
                broken = rows_a @ accel.ravel() < rhs_a - SHIELD_INTERVENTION_TOLERANCE_MPS2
                grow = {d for (i, j), b in zip(row_pair, broken, strict=True) if b for d in (i, j)} - takeover
                if not grow:
                    break
                takeover |= grow
    else:
        accel = np.clip(nominal, low, high)

    switches = 0
    for idx, aid in enumerate(ids):
        cand, clock = chosen[idx]
        switches += int(assign[idx] != 0)
        clock += 1
        latch[aid] = (0, 0) if clock >= CAND_STEPS[cand] else (cand, clock)

    diag.cbk = {
        "cand": {aid: chosen[i][0] for i, aid in enumerate(ids)},
        "switches": switches,
        "takeover": sorted(ids[d] for d in takeover),
        "min_margin_m": min_margin,
    }

    safe_actions = []
    capped_ids = {aid for pair in diag.backup_rollout_cap_pairs for aid in pair}
    for idx, aid in enumerate(ids):
        a = tuple(float(c) for c in accel[idx])
        shield_active = exceeds_intervention_tolerance(action_map[aid].proposed_accel_mps2, a)
        diag.interventions += int(shield_active)
        in_fallback = diag.fallback_used and idx in takeover
        safe_actions.append(SafeAction(
            agent_id=aid, safe_accel_mps2=a, timestamp_ms=step_ts, shield_active=shield_active,
            h_min_m2=diag.h_min_m2, qp_feasible=not in_fallback, fallback_used=in_fallback,
            backup_rollout_capped=aid in capped_ids, world_limit_conflict=aid in diag.world_limit_conflict_agents,
            escape_failed_slack=False,
        ))
    return safe_actions, diag
