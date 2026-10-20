"""HOCBF QP safety filter — v1 stub.

This module implements the full SafeAction contract interface so that
the simulator, runner, and logging pipeline can be built against it
now. The QP solve is a STUB: it passes the nominal action through
unchanged and computes the barrier value for diagnostics, but does
NOT yet modify actions to enforce constraints.

Implementation status
---------------------
STUB — pass-through only. The full HOCBF QP solver (P2-SA-01) will
replace ``_solve_qp_stub`` once Anas's simulator (P1-AN-01) is
available to validate the dynamics contract end-to-end.

What IS implemented here
------------------------
- Full input/output contract: accepts List[DroneState] + List[MARLAction],
  returns List[SafeAction].
- Input validation: rejects missing, duplicate, or mismatched agents and
  timestamps before doing any computation.
- Barrier value computation: h_ij = ||r_ij||² - d_safe² computed over
  all active pairs; h_min_m2 reported in every SafeAction.
- Shield intervention detection: uses the shared tolerance from contracts.
- Fallback semantics: zero acceleration when qp_feasible=False.
- Range gate: optional pair filtering for QP scalability (always-include
  in v1 default since N is small, documented for later use).

What is NOT implemented (P2-SA-01)
------------------------------------
- The HOCBF constraint matrix A_cbf and RHS b_cbf.
- The actual OSQP solve.
- Action modification to enforce the barrier constraints.
- Robust margins for wind disturbances (T2+).

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
from dataclasses import dataclass, field
from typing import Sequence

from shared.contracts import (
    SAFE_SEPARATION_M,
    SHIELD_INTERVENTION_TOLERANCE_MPS2,
    DroneState,
    MARLAction,
    SafeAction,
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
    """

    alpha1: float = 0.5
    alpha2: float = 0.5
    range_gate_m: float | None = _DEFAULT_RANGE_GATE_M

    def __post_init__(self) -> None:
        if not (0.0 < self.alpha1 <= 1.0):
            raise ValueError(f"alpha1 must be in (0, 1], got {self.alpha1}")
        if not (0.0 < self.alpha2 <= 1.0):
            raise ValueError(f"alpha2 must be in (0, 1], got {self.alpha2}")
        if self.range_gate_m is not None and self.range_gate_m <= 0.0:
            raise ValueError(f"range_gate_m must be positive, got {self.range_gate_m}")


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
        Whether the QP solve succeeded (stub always returns True).
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


def _solve_qp_stub(
    states: dict[str, DroneState],
    actions: dict[str, MARLAction],
    pairs_in_qp: list[tuple[str, str]],
    config: FilterConfig,
) -> tuple[dict[str, tuple[float, float, float]], bool]:
    """STUB: pass nominal actions through unchanged.

    Returns
    -------
    safe_accels:
        Dict mapping agent_id to (ax, ay, az) — identical to nominal
        in this stub.
    qp_feasible:
        Always True in the stub. The full implementation will return
        False when the OSQP solve is infeasible or errors.
    """
    # TODO (P2-SA-01): Replace with OSQP solve.
    # Build A_cbf and b_cbf from HOCBF constraints (formulation note
    # Section 7-8), stack decision variables, call osqp.solve(),
    # handle infeasibility and solver errors, return modified accels.
    _ = pairs_in_qp  # Will be used by the real solver.
    _ = config

    safe_accels = {
        aid: actions[aid].proposed_accel_mps2
        for aid in actions
    }
    return safe_accels, True


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
    d_safe_sq = SAFE_SEPARATION_M ** 2
    h_min: float | None = None
    pairs_total: list[tuple[str, str]] = []
    pairs_in_qp: list[tuple[str, str]] = []

    for idx, id_i in enumerate(agent_ids):
        for id_j in agent_ids[idx + 1:]:
            pairs_total.append((id_i, id_j))
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
    diag.n_pairs_total = len(pairs_total)
    diag.n_pairs_in_qp = len(pairs_in_qp)

    # --- QP solve (stub) ---
    try:
        safe_accels, qp_feasible = _solve_qp_stub(
            state_map, action_map, pairs_in_qp, config
        )
    except Exception as exc:
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
        shield_active = any(
            abs(accel[k] - nom[k]) > SHIELD_INTERVENTION_TOLERANCE_MPS2
            for k in range(3)
        )
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
