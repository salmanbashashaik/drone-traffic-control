"""HOCBF QP safety filter — active OSQP implementation (P2-SA-01).

This module implements the full SafeAction contract interface and active HOCBF
quadratic program solver. When active pairs are close or on a collision path,
the filter modifies the nominal acceleration commands by solving a joint convex
QP over all active agents to enforce the pairwise separation barrier.
If the QP is infeasible or the solver encounters an error, the system applies
the documented zero-acceleration fallback.

Implementation status
---------------------
Active OSQP solver implemented (P2-SA-01).
- Solves joint QP over all active agents.
- Enforces pairwise HOCBF constraints (Section 7-8 of formulation note).
- Enforces box acceleration bounds [-3.5, 3.5] m/s^2 per component.
- Emits zero-acceleration fallback with qp_feasible=False on solver failure or infeasibility.

Mathematical basis
------------------
See docs/formulation/P0-SA-01-formulation.md for the full derivation.

Barrier:    h_ij = ||p_i - p_j||^2 - d_safe^2
HOCBF:      two-level chain, relative degree 2 under acceleration control
QP:         min Σ||a_safe - a_nom||^2  s.t. HOCBF constraints, box bounds
Fallback:   zero acceleration when infeasible or solver error

Assumptions (inherited from P0-SA-01, Section 9)
-------------------------------------------------
A1  Discrete double-integrator dynamics (no speed cap in model).
A3  Perfect state knowledge via DroneState ground truth.
A4  Initial conditions satisfy h_ij >= 0 for all pairs.
A5  Control authority (3.5 m/s²) sufficient — not guaranteed for
    head-on high-speed scenarios.
A9  Joint solve over all active pairs required.
"""

from __future__ import annotations

import logging
import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
import osqp
import scipy.sparse as sp
from shared.contracts import (
    MAX_ACCEL_COMPONENT_MPS2,
    SAFE_SEPARATION_M,
    SIMULATION_STEP_MS,
    DroneState,
    MARLAction,
    SafeAction,
    exceeds_intervention_tolerance,
)

logger = logging.getLogger(__name__)

# Default range gate: None means include all pairs (correct for small N).
# Set to a positive float (metres) to exclude distant pairs from the QP.
_DEFAULT_RANGE_GATE_M: float | None = None


@dataclass(frozen=True)
class FilterConfig:
    """Runtime configuration for the HOCBF filter.

    Attributes
    ----------
    alpha1, alpha2:
        HOCBF class-K decay coefficients (0, 1]. See formulation note
        Section 6. Starting value 0.5 for both.
    range_gate_m:
        If set, pairs further apart than this distance are excluded
        from the QP constraint set. None = include all pairs.
    solver_eps:
        Absolute and relative tolerance for the OSQP solver.
    solver_max_iter:
        Maximum iterations allowed for OSQP.
    """

    alpha1: float = 0.5
    alpha2: float = 0.5
    range_gate_m: float | None = _DEFAULT_RANGE_GATE_M
    solver_eps: float = 1e-5
    solver_max_iter: int = 4000

    def __post_init__(self) -> None:
        if not (0.0 < self.alpha1 <= 1.0):
            raise ValueError(f"alpha1 must be in (0, 1], got {self.alpha1}")
        if not (0.0 < self.alpha2 <= 1.0):
            raise ValueError(f"alpha2 must be in (0, 1], got {self.alpha2}")
        if self.range_gate_m is not None and self.range_gate_m <= 0.0:
            raise ValueError(f"range_gate_m must be positive, got {self.range_gate_m}")
        if self.solver_eps <= 0.0:
            raise ValueError(f"solver_eps must be positive, got {self.solver_eps}")
        if self.solver_max_iter <= 0:
            raise ValueError(f"solver_max_iter must be positive, got {self.solver_max_iter}")


@dataclass
class FilterDiagnostics:
    """Per-step diagnostics emitted by the filter for logging/evaluation.

    Attributes
    ----------
    h_min_m2:
        Minimum pairwise barrier value across all active pairs.
        None when fewer than two active agents exist.
    n_pairs_total:
        Total active pairs considered (before range gate).
    n_pairs_in_qp:
        Pairs included in the QP after range gate filtering.
    qp_feasible:
        Whether the QP solve succeeded.
    fallback_used:
        Whether the zero-acceleration fallback was applied.
    interventions:
        Number of agents whose action was modified beyond tolerance.
    """

    h_min_m2: float | None = None
    n_pairs_total: int = 0
    n_pairs_in_qp: int = 0
    qp_feasible: bool = True
    fallback_used: bool = False
    interventions: int = 0


def _separation_sq(state_i: DroneState, state_j: DroneState) -> float:
    """Squared 3D Euclidean distance between two drone positions."""
    dx = state_i.position_m[0] - state_j.position_m[0]
    dy = state_i.position_m[1] - state_j.position_m[1]
    dz = state_i.position_m[2] - state_j.position_m[2]
    return dx * dx + dy * dy + dz * dz


def _barrier(state_i: DroneState, state_j: DroneState) -> float:
    """h_ij = ||p_i - p_j||^2 - d_safe^2."""
    return _separation_sq(state_i, state_j) - SAFE_SEPARATION_M ** 2


def build_hocbf_qp_matrices(
    states: dict[str, DroneState],
    actions: dict[str, MARLAction],
    pairs_in_qp: list[tuple[str, str]],
    config: FilterConfig,
) -> tuple[sp.csc_matrix, np.ndarray, sp.csc_matrix, np.ndarray, np.ndarray, list[str]]:
    """Build OSQP matrices for the joint HOCBF optimization problem.

    Decision vector x is stacked acceleration [a_0, a_1, ..., a_{N-1}] in R^{3N},
    ordered by agent_ids (sorted).

    Returns
    -------
    P: csc_matrix (3N x 3N)
    q: ndarray (3N,)
    A: csc_matrix ((M + 3N) x 3N)
    l: ndarray (M + 3N,)
    u: ndarray (M + 3N,)
    agent_ids: list[str]
    """
    agent_ids = sorted(states.keys())
    agent_idx = {aid: idx for idx, aid in enumerate(agent_ids)}
    n_agents = len(agent_ids)
    dim = 3 * n_agents

    # Objective: min sum_i ||a_i - a_i^nom||^2 = min 0.5 * x^T P x + q^T x
    P = sp.eye(dim, format="csc") * 2.0
    nom_stacked = np.zeros(dim, dtype=np.float64)
    for aid, idx in agent_idx.items():
        nom_stacked[3 * idx : 3 * idx + 3] = actions[aid].proposed_accel_mps2
    q = -2.0 * nom_stacked

    # Box bounds on control accelerations: -u_max <= a_{i,k} <= u_max
    u_max = MAX_ACCEL_COMPONENT_MPS2
    A_box = sp.eye(dim, format="csc")
    l_box = np.full(dim, -u_max, dtype=np.float64)
    u_box = np.full(dim, u_max, dtype=np.float64)

    dt = SIMULATION_STEP_MS / 1000.0  # 0.1 s
    d_safe = SAFE_SEPARATION_M  # 5.0 m

    # Pairwise HOCBF constraints:
    # s_ij^T * (a_i - a_j) >= rhs
    cbf_rows = []
    l_cbf = []
    u_cbf = []

    for id_i, id_j in pairs_in_qp:
        idx_i = agent_idx[id_i]
        idx_j = agent_idx[id_j]

        s_i = states[id_i]
        s_j = states[id_j]

        r_ij = np.array(s_i.position_m, dtype=np.float64) - np.array(s_j.position_m, dtype=np.float64)
        v_ij = np.array(s_i.velocity_mps, dtype=np.float64) - np.array(s_j.velocity_mps, dtype=np.float64)
        s_vec = r_ij + v_ij * dt

        h_ij = float(np.dot(r_ij, r_ij) - d_safe ** 2)
        r_dot_v = float(np.dot(r_ij, v_ij))
        v_sq = float(np.dot(v_ij, v_ij))

        # Level-2 HOCBF lower bound (P0-SA-01 Section 7-8)
        rhs = - v_sq - (config.alpha1 + config.alpha2) * r_dot_v - 0.5 * config.alpha1 * config.alpha2 * h_ij

        row = np.zeros(dim, dtype=np.float64)
        row[3 * idx_i : 3 * idx_i + 3] = s_vec
        row[3 * idx_j : 3 * idx_j + 3] = -s_vec

        cbf_rows.append(row)
        l_cbf.append(rhs)
        u_cbf.append(np.inf)

    if cbf_rows:
        A_cbf = sp.csc_matrix(np.vstack(cbf_rows))
        A = sp.vstack([A_cbf, A_box], format="csc")
        l = np.concatenate([np.array(l_cbf, dtype=np.float64), l_box])
        u = np.concatenate([np.array(u_cbf, dtype=np.float64), u_box])
    else:
        A = A_box
        l = l_box
        u = u_box

    return P, q, A, l, u, agent_ids


def _solve_hocbf_qp(
    states: dict[str, DroneState],
    actions: dict[str, MARLAction],
    pairs_in_qp: list[tuple[str, str]],
    config: FilterConfig,
) -> tuple[dict[str, tuple[float, float, float]], bool]:
    """Solve the joint HOCBF QP over all active agents using OSQP.

    Returns
    -------
    safe_accels:
        Dict mapping agent_id to (ax, ay, az). Empty if infeasible or error.
    qp_feasible:
        True if OSQP status is 'solved', False otherwise.
    """
    if not states or not actions:
        return {}, True

    if not pairs_in_qp:
        # No active pairs to constrain; nominal actions are already valid
        safe_accels = {
            aid: actions[aid].proposed_accel_mps2
            for aid in actions
        }
        return safe_accels, True

    try:
        P, q, A, l, u, agent_ids = build_hocbf_qp_matrices(
            states, actions, pairs_in_qp, config
        )
        prob = osqp.OSQP()
        prob.setup(
            P, q, A, l, u,
            verbose=False,
            eps_abs=config.solver_eps,
            eps_rel=config.solver_eps,
            max_iter=config.solver_max_iter,
        )
        res = prob.solve()

        if res.info.status == "solved":
            u_max = MAX_ACCEL_COMPONENT_MPS2
            safe_accels = {}
            for idx, aid in enumerate(agent_ids):
                ax = float(np.clip(res.x[3 * idx], -u_max, u_max))
                ay = float(np.clip(res.x[3 * idx + 1], -u_max, u_max))
                az = float(np.clip(res.x[3 * idx + 2], -u_max, u_max))
                safe_accels[aid] = (ax, ay, az)
            return safe_accels, True
        else:
            logger.warning("filter: OSQP non-optimal status: %s", res.info.status)
            return {}, False
    except Exception as exc:  # noqa: BLE001
        logger.error("filter: OSQP solve exception: %s", exc)
        return {}, False


def apply_filter(
    states: Sequence[DroneState],
    actions: Sequence[MARLAction],
    config: FilterConfig | None = None,
) -> tuple[list[SafeAction], FilterDiagnostics]:
    """Apply the HOCBF safety filter to one step.

    Parameters
    ----------
    states:
        Ground-truth DroneState for every active agent this step.
        Must contain exactly the same agent IDs as ``actions``.
    actions:
        Nominal MARLAction for every active agent this step.
        One per agent, timestamps must all match the state timestamps.
    config:
        Filter configuration. Uses defaults if None.

    Returns
    -------
    safe_actions:
        List[SafeAction] with one entry per active agent. On fallback,
        all agents receive zero acceleration.
    diagnostics:
        FilterDiagnostics with barrier values, pair counts, and
        intervention count for this step.

    Raises
    ------
    ValueError
        On any structural input violation: missing agents, duplicates,
        timestamp mismatches, or agent ID mismatches between states
        and actions. These must be caught by the step loop before
        calling this function.
    """
    if config is None:
        config = FilterConfig()

    # No active agents (e.g. all completed or aborted): nothing to filter.
    if not states and not actions:
        return [], FilterDiagnostics()

    # --- Input validation ---
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

    state_map: dict[str, DroneState] = {s.agent_id: s for s in states}
    action_map: dict[str, MARLAction] = {a.agent_id: a for a in actions}

    # Timestamp consistency: all actions must share the same timestamp,
    # which must equal each agent's state timestamp.
    action_timestamps = {a.timestamp_ms for a in actions}
    if len(action_timestamps) != 1:
        raise ValueError(
            f"All actions must share the same timestamp, got: {action_timestamps}"
        )
    step_ts = next(iter(action_timestamps))

    for s in states:
        if s.timestamp_ms != step_ts:
            raise ValueError(
                f"State timestamp {s.timestamp_ms} ms for agent '{s.agent_id}' "
                f"does not match action timestamp {step_ts} ms"
            )

    agent_ids = sorted(state_ids)  # deterministic ordering
    diag = FilterDiagnostics()

    # --- Barrier computation and pair selection ---
    h_min: float | None = None
    n_pairs_total = 0
    pairs_in_qp: list[tuple[str, str]] = []

    for idx, id_i in enumerate(agent_ids):
        for id_j in agent_ids[idx + 1:]:
            n_pairs_total += 1
            h = _barrier(state_map[id_i], state_map[id_j])

            if h_min is None or h < h_min:
                h_min = h

            # Range gate: skip pairs too far apart.
            if config.range_gate_m is not None:
                sep = math.sqrt(_separation_sq(state_map[id_i], state_map[id_j]))
                if sep > config.range_gate_m:
                    continue

            pairs_in_qp.append((id_i, id_j))

    diag.h_min_m2 = h_min
    diag.n_pairs_total = n_pairs_total
    diag.n_pairs_in_qp = len(pairs_in_qp)

    # --- QP solve ---
    try:
        safe_accels, qp_feasible = _solve_hocbf_qp(
            state_map, action_map, pairs_in_qp, config
        )
    except Exception as exc:  # noqa: BLE001
        logger.error(
            "filter: QP solver error at t=%d ms, applying fallback: %s",
            step_ts, exc,
        )
        qp_feasible = False
        safe_accels = {}

    diag.qp_feasible = qp_feasible
    diag.fallback_used = not qp_feasible

    # --- Build SafeAction list ---
    safe_actions: list[SafeAction] = []

    for aid in agent_ids:
        nom = action_map[aid].proposed_accel_mps2

        if not qp_feasible:
            # V1 fallback: zero acceleration for ALL agents in the joint solve.
            accel: tuple[float, float, float] = (0.0, 0.0, 0.0)
        else:
            accel = safe_accels[aid]

        # Determine whether the shield modified the action beyond tolerance.
        shield_active = exceeds_intervention_tolerance(nom, accel)
        if shield_active:
            diag.interventions += 1

        safe_actions.append(
            SafeAction(
                agent_id=aid,
                safe_accel_mps2=accel,
                timestamp_ms=step_ts,
                shield_active=shield_active,
                h_min_m2=diag.h_min_m2,
                qp_feasible=qp_feasible,
                fallback_used=diag.fallback_used,
            )
        )

    if not qp_feasible:
        logger.warning(
            "filter: QP infeasible at t=%d ms, fallback applied to %d agents "
            "(h_min=%.3f m²)",
            step_ts,
            len(safe_actions),
            diag.h_min_m2 if diag.h_min_m2 is not None else float("nan"),
        )

    logger.debug(
        "filter: t=%d ms pairs=%d/%d h_min=%.3f m² interventions=%d fallback=%s",
        step_ts,
        diag.n_pairs_in_qp,
        diag.n_pairs_total,
        diag.h_min_m2 if diag.h_min_m2 is not None else float("nan"),
        diag.interventions,
        diag.fallback_used,
    )

    return safe_actions, diag
