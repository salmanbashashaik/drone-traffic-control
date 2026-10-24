"""HOCBF QP safety filter — active OSQP implementation (P2-SA-01).

This module implements the full SafeAction contract interface and active HOCBF
quadratic program solver. When active pairs are close or on a collision path,
the filter modifies the nominal acceleration commands by solving a joint convex
QP over all active agents to enforce the pairwise separation barrier.
Each pair constraint carries a penalised slack, so the QP is always feasible.
When a closing pair needs slack, the step is re-solved with that pair's
braking constraint replaced by a predicted-separation constraint in metres,
so the unavoidable violation is minimised in metres using all three axes.
Only a solver error triggers the fallback: an evasive manoeuvre for the
conflict cluster only. The default barrier ("backup", contract v1.3)
replaces the braking rows with backup-set rows built from that same evasive
manoeuvre; FilterConfig(barrier="braking") keeps the PR B barrier.

Implementation status
---------------------
Active OSQP solver implemented (P2-SA-01).
- Solves joint QP over all active agents.
- Enforces pairwise braking-barrier constraints (Section 6-8 of formulation note).
- Enforces box acceleration bounds [-3.5, 3.5] m/s^2 per component.
- Softens each pair constraint with an L1-penalised slack (always feasible).
- On solver error, applies an evasive manoeuvre to the conflict cluster only.

Mathematical basis
------------------
See docs/formulation/P0-SA-01-formulation.md for the full derivation.

Safe set:   d_ij = ||p_i - p_j|| >= d_safe (reported via h_min_m2 = d^2 - d_safe^2)
Barrier:    h_ij = B(d_ij - d_safe - margin) + n_ij . v_ij, with
            B(s) = sqrt(4 u_max s + c^2) - c for s >= 0 and (2 u_max / c) s
            below (braking-distance CBF, relative degree 1 under acceleration
            control; C1 with bounded gradient through d_eff)
QP:         min Σ||a_safe - a_nom - bias||^2 + w Σ s  s.t.  dh/dt + s >= -gamma h, s >= 0, box bounds
Fallback:   evasive manoeuvre for the conflict cluster on solver error

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
import time
from collections.abc import Sequence
from dataclasses import dataclass, field

import daqp
import numpy as np
import osqp
import scipy.sparse as sp
from numba import njit, prange
from shared.contracts import (
    MAX_ACCEL_COMPONENT_MPS2,
    MAX_SPEED_MPS,
    SAFE_SEPARATION_M,
    SHIELD_INTERVENTION_TOLERANCE_MPS2,
    SIMULATION_STEP_MS,
    WORLD_XY_MAX_M,
    WORLD_XY_MIN_M,
    WORLD_Z_MAX_M,
    WORLD_Z_MIN_M,
    DroneState,
    MARLAction,
    SafeAction,
    exceeds_intervention_tolerance,
)

logger = logging.getLogger(__name__)

# Default range gate: None means the derived gate (``_exact_range_gate``), which
# drops only pairs whose rows cannot bind. A positive float (metres) gates closer.
_DEFAULT_RANGE_GATE_M: float | None = None


@dataclass(frozen=True)
class FilterConfig:
    """Runtime configuration for the HOCBF filter.

    Attributes
    ----------
    gamma:
        Linear class-K gain on the braking barrier (1/s). Larger values let
        the pair approach the braking curve faster. See formulation §6.
        At least 1: below that the world-box limits can cross.
    margin_m:
        Separation buffer added to d_safe inside the barrier. Absorbs the
        100 ms sample-and-hold error and small actuator lag (PR #5 review).
        Reported h_min_m2 still uses d_safe. Also the standoff from each wall
        of the world box; at most half the smallest box extent less the
        stopping distance from v_max (48.57 m at the defaults).
    tie_break_mps2:
        Lateral bias added to the nominal of both agents in a pair whose
        nominal violates the constraint (both turn right about +z). Breaks the
        symmetric head-on / crossing deadlock; 0 disables it. 1.0 rather
        than 0.5: at 0.5 a 90-degree crossing locks side by side for 3.6 s
        (+9 % episode time); at 0 it deadlocks.
    lag_s:
        Actuator time constant to budget for (s). The barrier reserves
        lag_s * closing_speed of extra distance. 0 for the ideal simulator;
        set to the measured tau when actuators lag.
    range_gate_m:
        If set, pairs further apart than this distance are excluded
        from the QP constraint set. None = the derived gate
        ``_exact_range_gate`` (595.6 m at the defaults), beyond which no row of
        a pair can bind, so excluding it leaves the optimum unchanged; a set
        value larger than that gate is capped at it.
    solver_eps:
        Absolute and relative tolerance for OSQP, which solves QPs above
        _EXACT_MAX_VARS variables (and any QP DAQP fails on). Tight enough
        that its solution passes the KKT certificate (``_kkt_certified``).
    solver_max_iter:
        Maximum iterations allowed for OSQP.
    slack_weight:
        Linear penalty per m/s² of pair-constraint slack. Must exceed the
        constraint's dual multiplier for the penalty to be exact; the
        multipliers are bounded by the action-change cost (order 10), so
        1e4 leaves wide headroom. The escape constraint uses the same weight
        per metre of predicted shortfall; its row scale 0.5 t*^2 >= 0.005
        keeps its multiplier below about 3e3.
    barrier_offset_mps:
        Offset c in B(s) = sqrt(4 u_max s + c^2) - c (m/s). Bounds the barrier
        gradient at 2 u_max / c (7 1/s at c = 1), so one 100 ms step cannot
        overshoot d_eff, and keeps the constraint finite inside d_eff. Costs
        about c^2 / (4 u_max) + c |rho| / (2 u_max) of extra standoff.
    barrier:
        "backup" (default since contract v1.3): the backup-set barrier
        (``_backup_constraints``),
        which keeps the evasive fallback manoeuvre able to hold d_eff from
        the next state; no tie-break. Closing pairs left with slack go
        through the same escape stage as "braking". "braking": the PR B
        braking-distance barrier plus the escape stage; kept selectable for
        one release. See docs/formulation/P2-SA-02-lateral-escape.md §4.2, §5.
    step_deadline:
        Bound the step time (contract v1.5, ``_solve_deadline``): each conflict
        cluster's solve has a wall-clock deadline, and a cluster that misses it
        flies its backup manoeuvre. True at runtime. Verification (fuzz, tiers,
        det.py, exact comparisons) runs with False, so its results do not
        depend on machine load and reproduce on any machine; the deadline
        behaviour has its own tests (tests/test_filter_deadline.py).
    """

    gamma: float = 1.0
    margin_m: float = 0.5
    tie_break_mps2: float = 1.0
    lag_s: float = 0.0
    range_gate_m: float | None = _DEFAULT_RANGE_GATE_M
    solver_eps: float = 1e-9
    solver_max_iter: int = 200000
    slack_weight: float = 1e4
    barrier_offset_mps: float = 1.0
    barrier: str = "backup"
    step_deadline: bool = True

    def __post_init__(self) -> None:
        # gamma >= 1 and the margin_m bound keep the world-box limits from
        # crossing (low <= high) for every state in the box at |v| <= v_max:
        # checked on a z / v_z grid for gamma 1 and 10 and c 0.1 to 5, not
        # proven. Below gamma 1 they cross even at margin 0 (gamma 0.1, c 0.1).
        # The bound leaves room for a drone at v_max to stop from mid-box.
        # _world_accel_limits still guards against a crossing at runtime.
        if not self.gamma >= 1.0:
            raise ValueError(f"gamma must be at least 1, got {self.gamma}")
        if not self.margin_m >= 0.0:
            raise ValueError(f"margin_m must be non-negative, got {self.margin_m}")
        c = self.barrier_offset_mps
        max_margin = (min(WORLD_XY_MAX_M - WORLD_XY_MIN_M, WORLD_Z_MAX_M - WORLD_Z_MIN_M) / 2.0
                      - ((MAX_SPEED_MPS + c) ** 2 - c * c) / (2.0 * MAX_ACCEL_COMPONENT_MPS2))
        if self.margin_m > max_margin:
            raise ValueError(
                f"margin_m must be at most {max_margin:.2f} m for barrier_offset_mps {c}, got {self.margin_m}"
            )
        if not 0.0 <= self.tie_break_mps2 <= MAX_ACCEL_COMPONENT_MPS2:
            raise ValueError(f"tie_break_mps2 must be in [0, u_max], got {self.tie_break_mps2}")
        if not self.lag_s >= 0.0:
            raise ValueError(f"lag_s must be non-negative, got {self.lag_s}")
        if self.range_gate_m is not None and self.range_gate_m <= 0.0:
            raise ValueError(f"range_gate_m must be positive, got {self.range_gate_m}")
        if self.solver_eps <= 0.0:
            raise ValueError(f"solver_eps must be positive, got {self.solver_eps}")
        if self.solver_max_iter <= 0:
            raise ValueError(f"solver_max_iter must be positive, got {self.solver_max_iter}")
        if not self.slack_weight > 0.0:
            raise ValueError(f"slack_weight must be positive, got {self.slack_weight}")
        if not self.barrier_offset_mps > 0.0:
            raise ValueError(f"barrier_offset_mps must be positive, got {self.barrier_offset_mps}")
        if self.barrier not in ("braking", "backup"):
            raise ValueError(f"barrier must be 'braking' or 'backup', got {self.barrier!r}")
        if not isinstance(self.step_deadline, bool):
            raise TypeError(f"step_deadline must be a bool, got {self.step_deadline!r}")


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
        Whether the QP solve succeeded. With slack the QP is always
        feasible, so False means a solver error.
    fallback_used:
        Whether the evasive fallback was applied to any agent.
    interventions:
        Number of agents whose action was modified beyond tolerance.
    pair_slack_mps2:
        Raw braking-constraint slack per pair (m/s²) from the first solve,
        for pairs above the intervention tolerance. Positive slack means the
        barrier of that pair could not be kept; closing pairs among them are
        re-solved with the escape constraint. Solver output, not a metric.
    pair_violation_depth_m:
        d_eff - d per pair (m) for pairs closer than d_eff this step.
    backup_rollout_steps:
        Steps each backup rollout flew this step, one per pair in the QP
        (barrier="backup" only; empty otherwise).
    backup_rollout_cap_pairs:
        Pairs whose backup rollout was still closing at the hard cap; their
        rows use the closest distance reached so far.
    world_limit_conflict_agents:
        Agents whose world-box acceleration limits crossed this step; the
        nearer wall's limit was kept (``_world_accel_limits``).
    escape_failed_slack_pairs:
        Pairs with slack in the first solve when the escape re-solve failed
        and that first solve was kept: their barrier is violated and the
        escape constraint did not act this step.
    deadline_fallback_agents:
        Agents of conflict clusters whose certified solve did not finish by
        the cluster's deadline (``_solve_deadline``); they fly their backup
        manoeuvre this step (contract v1.5).
    """

    h_min_m2: float | None = None
    n_pairs_total: int = 0
    n_pairs_in_qp: int = 0
    qp_feasible: bool = True
    fallback_used: bool = False
    interventions: int = 0
    pair_slack_mps2: dict[tuple[str, str], float] = field(default_factory=dict)
    pair_violation_depth_m: dict[tuple[str, str], float] = field(default_factory=dict)
    backup_rollout_steps: list[int] = field(default_factory=list)
    backup_rollout_cap_pairs: list[tuple[str, str]] = field(default_factory=list)
    world_limit_conflict_agents: list[str] = field(default_factory=list)
    escape_failed_slack_pairs: list[tuple[str, str]] = field(default_factory=list)
    deadline_fallback_agents: list[str] = field(default_factory=list)


def _separation_sq(state_i: DroneState, state_j: DroneState) -> float:
    """Squared 3D Euclidean distance between two drone positions."""
    dx = state_i.position_m[0] - state_j.position_m[0]
    dy = state_i.position_m[1] - state_j.position_m[1]
    dz = state_i.position_m[2] - state_j.position_m[2]
    return dx * dx + dy * dy + dz * dz


def _barrier(state_i: DroneState, state_j: DroneState) -> float:
    """h_ij = ||p_i - p_j||^2 - d_safe^2."""
    return _separation_sq(state_i, state_j) - SAFE_SEPARATION_M ** 2


def _pair_constraint(
    s_i: DroneState, s_j: DroneState, config: FilterConfig
) -> tuple[np.ndarray, float, float]:
    """Braking-barrier constraint n_ij . (a_i - a_j) >= rhs for one pair.

    Returns (n_ij, rhs, h_ij); n_ij is the unit vector from j to i.
    """
    u_max = MAX_ACCEL_COMPONENT_MPS2
    d_eff = SAFE_SEPARATION_M + config.margin_m
    r_ij = np.array(s_i.position_m, dtype=np.float64) - np.array(s_j.position_m, dtype=np.float64)
    v_ij = np.array(s_i.velocity_mps, dtype=np.float64) - np.array(s_j.velocity_mps, dtype=np.float64)
    dist = max(float(np.linalg.norm(r_ij)), 1e-9)
    n_ij = r_ij / dist
    rho = float(np.dot(n_ij, v_ij))  # range rate; negative when closing

    # h = B(d - D) + rho: the pair can still stop before d_eff using
    # combined braking 2 u_max along n_ij. B is the braking curve
    # sqrt(4 u_max s) shifted by c so its slope is 2 u_max / c at s = 0,
    # continued linearly inside d_eff: finite, C1, slope <= 2 u_max / c.
    # Lag reserve: distance closed before a lagged actuator starts braking.
    # Treated as constant within the step (its derivative is ignored).
    reserve = config.lag_s * max(-rho, 0.0)
    s = dist - d_eff - reserve
    c = config.barrier_offset_mps
    if s >= 0.0:
        root = math.sqrt(4.0 * u_max * s + c * c)
        b, slope = root - c, 2.0 * u_max / root
    else:
        b, slope = 2.0 * u_max * s / c, 2.0 * u_max / c
    h_ij = b + rho
    tangential = (float(np.dot(v_ij, v_ij)) - rho * rho) / dist
    rhs = -config.gamma * h_ij - slope * rho - tangential
    return n_ij, rhs, h_ij


_ESCAPE_HORIZON_S = 3.0
_MISS_CLEAR_M = 0.5
# Quadratic penalty per m² of escape shortfall. 1e3 keeps the multiplier
# (2 w s) well above the action-change cost (about 14) for s >= 0.1 m and lets
# OSQP converge within solver_max_iter; 1e4 does not converge on the sandwich.
_ESCAPE_WEIGHT = 1e3


def _relative(s_i: DroneState, s_j: DroneState) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """(r, v, n, t*) for a pair: relative position and velocity of i to j, unit
    line of sight, and the closest-approach time clamped to [0.1, 3] s."""
    r = np.array(s_i.position_m, dtype=np.float64) - np.array(s_j.position_m, dtype=np.float64)
    v = np.array(s_i.velocity_mps, dtype=np.float64) - np.array(s_j.velocity_mps, dtype=np.float64)
    n = r / max(float(np.linalg.norm(r)), 1e-9)
    t_star = float(np.clip(-float(r @ v) / max(float(v @ v), 1e-9), 0.1, _ESCAPE_HORIZON_S))
    return r, v, n, t_star


# Line-of-sight angle from vertical (as sin) below which the sideways direction
# is the pair's horizontal offset, and above which it is z x n; blended between.
_NEAR_VERTICAL_SIN = (math.sin(math.radians(10.0)), math.sin(math.radians(20.0)))
# Horizontal offset (m) below which a near-vertical pair starting a conflict
# takes the plumb rule (+x for the upper drone) instead of its offset, and
# the predicted miss beyond d_eff (m) under which a pair is in conflict. The
# pair's relative position error is sqrt(2) sigma per axis; its horizontal
# size is Rayleigh, 99th percentile 3.03 sqrt(2) sigma = 6.4 m at the
# realistic profile (sigma 1.5 m; 2.1 m at nominal 0.5 m). A smaller offset
# is mostly noise there, so its direction would be a coin toss.
_PLUMB_OFFSET_M = 6.5


def _lateral(n: np.ndarray, plumb_sin: np.ndarray | float = 1e-9) -> np.ndarray:
    """Unit sideways direction perpendicular to unit vector(s) n.

    Away from vertical it is z x n (a right turn about +z). Near vertical,
    z x n is ill-conditioned: a tiny horizontal offset sets its direction, so
    it swings from step to step (PR #12 review B2). There it is the pair's
    horizontal offset, projected perpendicular to n: moving along it does not
    rotate it, so it stays fixed while the conflict lasts. An exactly vertical
    n uses +x (-x for downward n). The two are orthogonal and blend between
    10 and 20 degrees from vertical, so the result never vanishes or jumps.
    The plumb rule also replaces an offset with sine below ``plumb_sin``,
    within 10 degrees of vertical only (where the turn has no weight).
    _lateral(-n) = -_lateral(n), so the two drones of a pair move apart.
    Works on one (3,) vector or a (M, 3) stack.
    """
    n = np.asarray(n, dtype=np.float64)
    horizontal = np.linalg.norm(n[..., :2], axis=-1, keepdims=True)
    turn = np.cross((0.0, 0.0, 1.0), n) / np.maximum(horizontal, 1e-12)
    offset = n * (1.0, 1.0, 0.0)
    plumb = np.zeros_like(n)
    plumb[..., 0] = np.where(n[..., 2] >= 0.0, 1.0, -1.0)
    lo, hi = _NEAR_VERTICAL_SIN
    offset = np.where(horizontal < np.minimum(np.maximum(plumb_sin, 1e-9), lo), plumb, offset)
    offset = offset - np.sum(offset * n, axis=-1, keepdims=True) * n
    offset /= np.maximum(np.linalg.norm(offset, axis=-1, keepdims=True), 1e-12)
    weight = np.clip((horizontal - lo) / (hi - lo), 0.0, 1.0)
    lateral = weight * turn + (1.0 - weight) * offset
    return lateral / np.linalg.norm(lateral, axis=-1, keepdims=True)


@dataclass
class _PairLatch:
    """Sideways frame of a pair, fixed when its conflict starts (PR #12 re-review).

    axis: unit rotation axis w = n0 x L0 of the line of sight n0 and the
    sideways direction L0 at the start; the sideways direction is then w x n,
    which stays continuous as the pair turns about w. signs: the
    ``_escape_axes`` sign normalisation of (L, V) at the start. up: the
    evasive z direction of the pair's first (lower id) drone at the start,
    by the same rule as before (away in z; level: the lower id climbs).
    escape: the escape signs (s_L, s_V), fixed the first step the pair needs them.
    origin: "turn_right" when a two-drone conflict without a clear miss chose
    the sideways direction, which may be re-oriented once (``_update_latches``);
    "none" when a 2-colouring left it at turn-right (odd cycle), so it does not
    seed later colourings; else "fixed".
    """

    axis: np.ndarray
    signs: tuple[float, float]
    up: float
    escape: tuple[float, float] | None = None
    origin: str = "fixed"


def _frame_lateral(n: np.ndarray, axis: np.ndarray) -> np.ndarray:
    """Sideways unit vector(s) w x n for rotation axis (axes) w, (3,) or (M, 3).

    A zero axis (no latch), or n within 30 degrees of w, gives ``_lateral(n)``.
    """
    lateral = np.cross(axis, n)
    norm = np.linalg.norm(lateral, axis=-1, keepdims=True)
    return np.where(norm > 0.5, lateral / np.maximum(norm, 1e-12), _lateral(n))


def _new_latch(n: np.ndarray, dist: float, escape: tuple[float, float] = (0.0, 0.0)) -> _PairLatch:
    """Latch for a pair entering conflict along n.

    ``escape`` holds the unlatched escape signs (s_L, s_V) of the pair's
    conflict cluster (``_escape_signs``, miss or 2-colouring) on the axes of
    ``_escape_axes(n)``. A non-zero sign orients the latched sideways
    direction, or the latched up, so the backup and the escape that follow
    the latch move agent i the way the colouring chose. With zero signs the
    latch keeps ``_lateral(n)``: both drones turn right (z x n).
    """
    lateral = _lateral(n, _PLUMB_OFFSET_M / max(dist, 1e-9))
    up = math.copysign(1.0, n[2]) if abs(n[2]) > 1e-3 else 1.0
    lateral_e, vertical_e = _escape_axes(n)
    along = escape[0] * float(lateral_e @ lateral)
    if abs(along) > 0.5:
        lateral = math.copysign(1.0, along) * lateral
    if escape[1] != 0.0 and abs(float(vertical_e[2])) > _NEAR_VERTICAL_SIN[0]:
        up = math.copysign(1.0, escape[1] * float(vertical_e[2]))
    axes = (lateral, np.cross(n, lateral))
    signs = tuple(1.0 if a[int(np.argmax(np.abs(a)))] > 0.0 else -1.0 for a in axes)
    return _PairLatch(axis=np.cross(n, lateral), signs=signs, up=up)


def _pair_states(
    state_map: dict[str, DroneState], pairs: list[tuple[str, str]]
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Positions and velocities of the first and second drone of each pair, (M, 3) each.

    The same float64 values ``_relative`` and ``_backup_constraints`` read from
    the states, gathered with one array per agent instead of one per pair.
    """
    index = {aid: k for k, aid in enumerate(state_map)}
    pos = np.array([s.position_m for s in state_map.values()], dtype=np.float64).reshape(-1, 3)
    vel = np.array([s.velocity_mps for s in state_map.values()], dtype=np.float64).reshape(-1, 3)
    i = np.array([index[p[0]] for p in pairs], dtype=np.intp)
    j = np.array([index[p[1]] for p in pairs], dtype=np.intp)
    return pos[i], pos[j], vel[i], vel[j]


# Relative distance from a threshold within which the vectorised screen defers
# to the scalar code. The two differ by a few ulps (summation order), far below.
_SCREEN_RTOL = 1e-9


def _conflict_screen(
    state_map: dict[str, DroneState], pairs: list[tuple[str, str]], limit: float
) -> tuple[np.ndarray, np.ndarray]:
    """Per pair (M,): whether min(|r|, |r + v t*|) < limit, and whether r . v >= 0.

    The two tests of ``_update_latches``, vectorised. numpy's row sums can
    differ from ``_relative``'s 1-D dot products by a few ulps, so a pair whose
    value lies within _SCREEN_RTOL of a threshold is decided by the scalar code,
    which keeps every decision identical to the per-pair loop.
    """
    if not pairs:
        return np.zeros(0, dtype=bool), np.zeros(0, dtype=bool)
    p_i, p_j, v_i, v_j = _pair_states(state_map, pairs)
    r, v = p_i - p_j, v_i - v_j
    rv = np.einsum("ij,ij->i", r, v)
    vv = np.einsum("ij,ij->i", v, v)
    t_star = np.clip(-rv / np.maximum(vv, 1e-9), 0.1, _ESCAPE_HORIZON_S)
    dist = np.sqrt(np.einsum("ij,ij->i", r, r))
    miss_v = r + v * t_star[:, None]
    miss = np.sqrt(np.einsum("ij,ij->i", miss_v, miss_v))
    speed = np.sqrt(vv)
    scale = _SCREEN_RTOL * (1.0 + dist + speed * _ESCAPE_HORIZON_S)
    in_conflict = np.minimum(dist, miss) < limit
    separating = rv >= 0.0
    near = (np.abs(dist - limit) <= scale) | (np.abs(miss - limit) <= scale) | (np.abs(rv) <= scale * (1.0 + speed))
    for k in np.flatnonzero(near):
        r_k, v_k, _, t_k = _relative(state_map[pairs[k][0]], state_map[pairs[k][1]])
        in_conflict[k] = min(float(np.linalg.norm(r_k)), float(np.linalg.norm(r_k + v_k * t_k))) < limit
        separating[k] = float(r_k @ v_k) >= 0.0
    return in_conflict, separating


def _update_latches(
    state_map: dict[str, DroneState], pairs: list[tuple[str, str]], config: FilterConfig,
    latch: dict[tuple[str, str], _PairLatch],
) -> dict[tuple[str, str], _PairLatch]:
    """Latch the sideways frame of each pair entering conflict; drop it after.

    A pair enters conflict when its distance, or its constant-velocity
    predicted miss at t*, is below d_eff + _PLUMB_OFFSET_M: within the
    realistic profile's noise of d_eff. It leaves only when both are above
    that and it no longer closes, so a noisy miss crossing the limit while
    the pair still closes does not release and re-latch it with another
    frame. Pairs entering conflict take their orientation from the
    unlatched escape signs of the conflict set (``_new_latch``): the
    predicted miss when it is clear, else the 2-colouring, but the
    colouring only in clusters of three or more drones. A two-drone
    conflict with no clear miss keeps the turn-right rule (z x n), which
    needs no knowledge of the cluster. The colouring of a cluster is
    seeded with the sideways directions already latched in it and takes
    every pair of its drones, closest first, so new pairs fit the existing
    latches (PR #12 review B1). A turn-right latch whose escape sign in
    that colouring is opposite is re-oriented, once, the step its cluster
    reaches three drones; after that no latch flips. From then on the
    backup and the escape follow the latch. Returns the latches of
    ``pairs``.
    """
    limit = SAFE_SEPARATION_M + config.margin_m + _PLUMB_OFFSET_M
    entering = []
    in_conflict, separating = _conflict_screen(state_map, pairs, limit)
    for pair, inside, apart in zip(pairs, in_conflict, separating, strict=True):
        if inside:
            if pair not in latch:
                entering.append(pair)
        elif apart:
            latch.pop(pair, None)
    for pair in set(latch) - set(pairs):
        del latch[pair]
    if entering:
        # Colour the whole conflict set, so a drone squeezed between two
        # neighbours gets opposite latches towards them.
        conflict = sorted(set(latch) | set(entering))
        # Without colouring each pair's signs depend on that pair alone, and only
        # the entering pairs read them.
        miss_only = _escape_signs(state_map, entering, colouring=False)
        cluster = {aid: {aid} for pair in conflict for aid in pair}
        for id_i, id_j in conflict:
            if cluster[id_i] is not cluster[id_j]:
                merged = cluster[id_i] | cluster[id_j]
                for aid in merged:
                    cluster[aid] = merged
        coloured = [pair for pair in conflict if len(cluster[pair[0]]) >= 3]
        seed = {}
        seeded = [pair for pair in coloured if pair in latch and latch[pair].origin != "none"]
        for pair, along in zip(seeded, _seed_alongs(state_map, seeded, latch), strict=True):
            if abs(along) > 0.5:
                seed[pair] = (math.copysign(1.0, along), 1 if latch[pair].origin == "turn_right" else 2)
        graph = [pair for pair in pairs if len(cluster.get(pair[0], ())) >= 3 and pair[1] in cluster[pair[0]]]
        escape = _escape_signs(
            state_map, [p for p in coloured if p not in latch or latch[p].origin == "turn_right"],
            graph=graph, seed=seed,
        )
        for pair in coloured:
            frame = latch.get(pair)
            if frame is not None and frame.origin == "turn_right":
                if pair in seed and escape[pair][0] * seed[pair][0] < 0.0:
                    # Mirror the sideways direction; the vertical escape stays.
                    latch[pair] = _PairLatch(axis=-frame.axis, signs=(-frame.signs[0], -frame.signs[1]), up=frame.up)
                else:
                    frame.origin = "fixed"
        for pair in entering:
            r = _relative(state_map[pair[0]], state_map[pair[1]])[0]
            dist = float(np.linalg.norm(r))
            sign = escape[pair] if len(cluster[pair[0]]) >= 3 else miss_only[pair]
            frame = latch[pair] = _new_latch(r / max(dist, 1e-9), dist, sign)
            if sign[0] == 0.0:
                frame.origin = "none" if len(cluster[pair[0]]) >= 3 else "turn_right"
    return dict(latch)


def _seed_along(n: np.ndarray, axis: np.ndarray) -> float:
    """L . (w x n): the latched sideways direction along the unlatched escape axis L of n."""
    return float(_escape_axes(n)[0] @ _frame_lateral(n, axis))


def _seed_alongs(
    state_map: dict[str, DroneState], pairs: list[tuple[str, str]], latch: dict[tuple[str, str], _PairLatch]
) -> np.ndarray:
    """``_seed_along`` of each latched pair (M,), vectorised for the seed loop of ``_update_latches``.

    The loop only compares |along| with 0.5 and takes its sign there. The
    stacked computation differs from the per-pair one by a few ulps (1-D norms
    and dots sum in another order), so a pair near a decision edge is
    recomputed by ``_seed_along``: |along| near 0.5, ``_frame_lateral``'s norm
    near its 0.5 switch, a near tie in the sign normalisation of L, or a line
    of sight within 1e-6 of vertical (the plumb rule of ``_lateral``). Away
    from those edges the decisions are the per-pair loop's.
    """
    if not pairs:
        return np.zeros(0)
    p_i, p_j, _, _ = _pair_states(state_map, pairs)
    r = p_i - p_j
    n = r / np.maximum(np.sqrt(np.einsum("ij,ij->i", r, r)), 1e-9)[:, None]
    lateral = _lateral(n)
    top = np.sort(np.abs(lateral), axis=1)
    sign = np.where(lateral[np.arange(len(pairs)), np.argmax(np.abs(lateral), axis=1)] > 0.0, 1.0, -1.0)
    axes = np.array([latch[pair].axis for pair in pairs], dtype=np.float64).reshape(-1, 3)
    turn = np.cross(axes, n)
    size = np.linalg.norm(turn, axis=-1, keepdims=True)
    frame = np.where(size > 0.5, turn / np.maximum(size, 1e-12), lateral)
    along = sign * np.einsum("ij,ij->i", lateral, frame)
    edge = _SCREEN_RTOL * 10.0
    near = ((np.abs(np.abs(along) - 0.5) <= edge) | (np.abs(size[:, 0] - 0.5) <= edge)
            | (top[:, 2] - top[:, 1] <= edge) | (np.hypot(n[:, 0], n[:, 1]) <= 1e-6))
    for k in np.flatnonzero(near):
        along[k] = _seed_along(_relative(state_map[pairs[k][0]], state_map[pairs[k][1]])[2], latch[pairs[k]].axis)
    return along


def _latch_axis(frames: dict[tuple[str, str], _PairLatch] | None, pair: tuple[str, str]) -> np.ndarray:
    frame = (frames or {}).get(pair)
    return np.zeros(3) if frame is None else frame.axis


def _escape_axes(n: np.ndarray, frame: _PairLatch | None = None) -> tuple[np.ndarray, np.ndarray]:
    """Sideways L and vertical V unit vectors perpendicular to n.

    L is ``_lateral(n)``; V = n x L completes the frame (z minus its n
    component for horizontal n). Each is sign-normalised so its largest
    component is positive, which gives pairs on opposite sides of a drone the
    same axes and does not flip under small drifts. With a latched ``frame``
    L is w x n and both keep the signs chosen when the conflict started.
    """
    if frame is not None:
        lateral = _frame_lateral(n, frame.axis)
        vertical = np.cross(n, lateral)
        return frame.signs[0] * lateral, frame.signs[1] * vertical / np.linalg.norm(vertical)
    lateral = _lateral(n)
    vertical = np.cross(n, lateral)
    axes = []
    for axis in (lateral, vertical):
        axis = axis / np.linalg.norm(axis)
        axes.append(axis if axis[int(np.argmax(np.abs(axis)))] > 0.0 else -axis)
    return axes[0], axes[1]


def _escape_signs(
    state_map: dict[str, DroneState], escape_pairs: list[tuple[str, str]],
    frames: dict[tuple[str, str], _PairLatch] | None = None, colouring: bool = True,
    graph: list[tuple[str, str]] | None = None,
    seed: dict[tuple[str, str], tuple[float, int]] | None = None,
) -> dict[tuple[str, str], tuple[float, float]]:
    """Lateral and vertical escape signs (s_L, s_V) per escape pair.

    Each sign follows the geometric miss of the pair at closest approach
    (the part of r + v t* perpendicular to n) when that miss is clear along
    the axis. Otherwise it comes from a 2-colouring of the escape-pair graph,
    one per axis: agent i of the pair moves towards +axis when its colour is
    +1, so a drone squeezed between two neighbours escapes opposite to both.
    The lateral colouring takes the closest pairs first; a pair left with
    equal colours (odd cycle) gets sign 0 on that axis. The vertical
    colouring takes those pairs first, so they split vertically instead.
    A pair with a latched frame instead escapes along its latched backup
    direction, fixed from its first escape step: s_L makes the sideways part
    +L, and s_V follows the latched up (0 when V is within 10 degrees of
    horizontal). With ``colouring`` false, an axis without a clear miss
    gets sign 0. ``graph`` (default the escape pairs) is the edge set
    coloured, and ``seed`` holds latched lateral colours (``_two_colour``),
    whose edges are coloured first, highest rank first.
    """
    frames = frames or {}
    seed = seed or {}
    by_distance = sorted(
        escape_pairs if graph is None else graph,
        key=lambda p: (-seed.get(p, (0.0, 0))[1], _separation_sq(state_map[p[0]], state_map[p[1]]), p),
    )
    lateral = _two_colour(by_distance, seed)
    vertical = _two_colour(sorted(by_distance, key=lambda p: lateral[p[0]] != lateral[p[1]]))

    signs = {}
    for id_i, id_j in escape_pairs:
        frame = frames.get((id_i, id_j))
        if frame is not None and frame.escape is not None:
            signs[(id_i, id_j)] = frame.escape
            continue
        r, v, n, t_star = _relative(state_map[id_i], state_map[id_j])
        if frame is not None:
            # Follow the latched backup direction (+L sideways, the latched up in
            # z for agent i), so the escape cannot flip against the backup rows
            # and the tie-break on a noisy miss (PR #12 re-review).
            # Near a vertical line of sight V is almost horizontal, and the
            # sign of its small z part is noise: no vertical escape then.
            v_z = frame.up * float(_escape_axes(n, frame)[1][2])
            frame.escape = signs[(id_i, id_j)] = (
                frame.signs[0], math.copysign(1.0, v_z) if abs(v_z) > _NEAR_VERTICAL_SIN[0] else 0.0,
            )
            continue
        miss = r + v * t_star
        miss -= float(miss @ n) * n
        signs[(id_i, id_j)] = tuple(
            math.copysign(1.0, float(miss @ axis)) if abs(float(miss @ axis)) > _MISS_CLEAR_M
            else (colour[id_i] if colouring and colour[id_i] != colour[id_j] else 0.0)
            for axis, colour in zip(_escape_axes(n), (lateral, vertical), strict=True)
        )
    return signs


def _two_colour(
    edges: list[tuple[str, str]], seed: dict[tuple[str, str], tuple[float, int]] | None = None,
) -> dict[str, float]:
    """Greedy +-1 colouring that gives each edge, in order, opposite colours
    when the edges before it allow; the lowest id of each component is +1.

    ``seed`` maps an edge to (colour of its first drone, rank > 0) for a
    latched direction. A seeded edge that starts a component takes that
    colour, a component keeps the highest rank of its edges and is not
    renormalised, and of two components that meet with equal colours the
    one of lower rank flips (equal ranks: the second, but never rank 2)."""
    seed = seed or {}
    colour: dict[str, float] = {}
    component: dict[str, int] = {}
    rank: dict[int, int] = {}
    for edge in edges:
        id_i, id_j = edge
        first, r = seed.get(edge, (1.0, 0))
        if id_i not in colour and id_j not in colour:
            colour[id_i], colour[id_j] = first, -first
            component[id_i] = component[id_j] = len(rank)
            rank[len(rank)] = r
        elif id_j not in colour:
            colour[id_j], component[id_j] = -colour[id_i], component[id_i]
            rank[component[id_i]] = max(rank[component[id_i]], r)
        elif id_i not in colour:
            colour[id_i], component[id_i] = -colour[id_j], component[id_j]
            rank[component[id_j]] = max(rank[component[id_j]], r)
        elif component[id_i] != component[id_j]:
            keep, old = component[id_i], component[id_j]
            if rank[old] > rank[keep]:
                keep, old = old, keep
            flip = colour[id_i] == colour[id_j] and rank[old] < 2
            for aid in [a for a, c in component.items() if c == old]:
                component[aid] = keep
                colour[aid] = -colour[aid] if flip else colour[aid]
            rank[keep] = max(rank[keep], rank[old], r)
    for comp in set(component.values()):
        if rank[comp] > 0:
            continue
        lowest = min(a for a, c in component.items() if c == comp)
        if colour[lowest] < 0.0:
            for aid in [a for a, c in component.items() if c == comp]:
                colour[aid] = -colour[aid]
    return colour


def _escape_constraint(
    s_i: DroneState, s_j: DroneState, sign: tuple[float, float], config: FilterConfig,
    frame: _PairLatch | None = None,
) -> tuple[np.ndarray, float]:
    """Predicted-separation constraint coef . (a_i - a_j) >= rhs, in metres.

    With constant relative acceleration a, the relative position at the
    predicted closest approach t* is p = r + v t* + 0.5 t*^2 a. For a unit
    vector m, m . p <= |p|, so m . p >= d_eff - s keeps the predicted
    separation at t* above d_eff - s, with the slack s in metres. m mixes the
    line of sight n with the lateral L and vertical V axes, signs (s_L, s_V),
    so the escape uses all three axes. Returns (0.5 t*^2 m, rhs).

    known limit: constant acceleration to t*, re-planned every step; the true
    closest approach can differ. Revisit with the backup-set barrier
    (docs/formulation/P2-SA-02-lateral-escape.md).
    """
    r, v, n, t_star = _relative(s_i, s_j)
    lateral, vertical = _escape_axes(n, frame)
    m = n + sign[0] * lateral + sign[1] * vertical
    m /= np.linalg.norm(m)
    d_eff = SAFE_SEPARATION_M + config.margin_m
    return 0.5 * t_star**2 * m, d_eff - float(m @ (r + v * t_star))


# Extra backup horizon past the derived stopping time, so the closest
# approach is not cut off at the horizon end by the 100 ms hold and the
# turn of the line of sight during the manoeuvre.
_BACKUP_HORIZON_MARGIN_S = 0.5


def _backup_steps(config: FilterConfig) -> int:
    """Minimum backup horizon in simulator steps, derived from the speed and accel limits.

    Away from the walls the evasive manoeuvre brakes each drone by at least
    u_max / sqrt(2) along the line of sight (``_evasive_direction``: its x and
    y components are at most sqrt(2), its n component at least 1). A pair
    closing at 2 v_max therefore stops closing within sqrt(2) v_max / u_max
    (6.06 s at 15 m/s and 3.5 m/s²). Add the actuator lag and
    _BACKUP_HORIZON_MARGIN_S. Near a wall the bound does not hold, so the
    rollout continues past this minimum while the pair still closes, up to
    ``_backup_max_steps`` (P2-SA-02 §5.3).
    """
    stop_s = math.sqrt(2.0) * MAX_SPEED_MPS / MAX_ACCEL_COMPONENT_MPS2
    return math.ceil((stop_s + config.lag_s + _BACKUP_HORIZON_MARGIN_S) / (SIMULATION_STEP_MS / 1000.0))


def _backup_max_steps(config: FilterConfig) -> int:
    """Hard cap on the adaptive backup rollout in simulator steps (P2-SA-02 §5.3).

    A loose bound, not a proof: the world-box clipping can force the
    near-wall drone towards its neighbour for at most v_max / u_max (4.29 s),
    until it has braked to rest along the wall normal; after that the pair is
    assumed to stop closing within a further sqrt(2) v_max / u_max (6.06 s),
    as in ``_backup_steps``. With the lag and _BACKUP_HORIZON_MARGIN_S this
    is 109 steps at the defaults. A pair still closing at the cap is flagged
    (``FilterDiagnostics.backup_rollout_cap_pairs``), so a case the bound
    misses is visible.
    """
    stop_s = (1.0 + math.sqrt(2.0)) * MAX_SPEED_MPS / MAX_ACCEL_COMPONENT_MPS2
    return math.ceil((stop_s + config.lag_s + _BACKUP_HORIZON_MARGIN_S) / (SIMULATION_STEP_MS / 1000.0))


_WORLD_MIN_M = np.array((WORLD_XY_MIN_M, WORLD_XY_MIN_M, WORLD_Z_MIN_M))
_WORLD_MAX_M = np.array((WORLD_XY_MAX_M, WORLD_XY_MAX_M, WORLD_Z_MAX_M))


def _raw_world_limits(
    pos: np.ndarray, vel: np.ndarray, config: FilterConfig
) -> tuple[np.ndarray, np.ndarray]:
    """Per-component world-box acceleration limits (low, high) before the guard.

    One braking barrier per axis and wall, like the pair barrier but for one
    drone braking at u_max: h = B(s) - w, with s the distance to the wall
    less margin_m, w the speed towards it and B(s) = sqrt(2 u_max s + c^2) - c
    (linear inside the margin). dh/dt >= -gamma h asks for at least
    -gamma h + B'(s) w of acceleration away from the wall, clipped to the
    box. FilterConfig's margin_m and gamma bounds keep low <= high for every
    state in the box at |v| <= v_max (checked on a grid, not proven).
    """
    u_max = MAX_ACCEL_COMPONENT_MPS2
    c = config.barrier_offset_mps

    def min_away(s: np.ndarray, w: np.ndarray) -> np.ndarray:
        root = np.sqrt(2.0 * u_max * np.maximum(s, 0.0) + c * c)
        b = np.where(s >= 0.0, root - c, u_max * s / c)
        slope = np.where(s >= 0.0, u_max / root, u_max / c)
        return np.clip(-config.gamma * (b - w) + slope * w, -u_max, u_max)

    low = min_away(pos - _WORLD_MIN_M - config.margin_m, -vel)
    high = -min_away(_WORLD_MAX_M - config.margin_m - pos, vel)
    return low, high


def _world_accel_limits(
    pos: np.ndarray, vel: np.ndarray, config: FilterConfig
) -> tuple[np.ndarray, np.ndarray]:
    """Per-component acceleration limits (low, high) that keep a drone in the world box.

    ``_raw_world_limits`` with a runtime guard: if the limits ever cross on
    an axis, the nearer wall's limit wins, so the drone is never driven into
    the closer wall and the QP stays feasible (an OSQP setup error before).
    ``_world_limit_conflicts`` reports where the guard acted. Works on (3,)
    or (N, 3) arrays.
    """
    low, high = _raw_world_limits(pos, vel, config)
    nearer_min = (pos - _WORLD_MIN_M) <= (_WORLD_MAX_M - pos)
    crossed = low > high
    return np.where(crossed & ~nearer_min, high, low), np.where(crossed & nearer_min, low, high)


def _world_limit_conflicts(pos: np.ndarray, vel: np.ndarray, config: FilterConfig) -> np.ndarray:
    """Per drone (N,): whether its raw world limits crossed on any axis."""
    low, high = _raw_world_limits(pos, vel, config)
    return np.any(low > high, axis=-1)


def _evasive_direction(
    n: np.ndarray, up: np.ndarray | float, limits: tuple[np.ndarray, np.ndarray] | None = None,
    axis: np.ndarray | None = None,
) -> np.ndarray:
    """Evasive acceleration for the agent at the head of unit vector(s) n.

    Sums n (away from the neighbour), the sideways unit vector ``_lateral(n)``
    (``_frame_lateral(n, axis)`` with latched axes; zero rows have no latch)
    and ``up`` in z, scaled so the largest component is u_max. With
    ``limits`` (``_world_accel_limits`` of the drone) each component is then
    clipped to them, so near a wall the drone brakes or turns away from it
    instead of towards it (PR #12 review B1). Works on one (3,) vector or a
    (M, 3) stack.

    A clipped component can reduce the braking along n below u_max / sqrt(2),
    the bound ``_backup_steps`` rests on; only near a wall. The backup
    rollout therefore runs adaptively (``_backup_rollout``).
    """
    direction = n + (_lateral(n) if axis is None else _frame_lateral(n, axis))
    direction[..., 2] += up
    direction = direction * (MAX_ACCEL_COMPONENT_MPS2 / np.max(np.abs(direction), axis=-1, keepdims=True))
    return direction if limits is None else np.clip(direction, *limits)


def _backup_rollout(
    p_i: np.ndarray, p_j: np.ndarray, v_i: np.ndarray, v_j: np.ndarray, config: FilterConfig,
    axis: np.ndarray | None = None, up_latched: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Fly the evasive manoeuvre of pairs (i, j) against each other, (M, 3) inputs.

    ``axis`` (M, 3) and ``up_latched`` (M,) hold each pair's latched
    rotation axis and z direction (zero: none).

    Each pair flies at least ``_backup_steps`` steps, then until it stops
    closing, at most ``_backup_max_steps``. Returns the relative positions
    after each step (K, M, 3; K is the longest rollout, and steps after a
    pair's own end are NaN), the first relative acceleration a_i - a_j
    (M, 3), the closing speed after each pair's last step (M,; positive
    while the pair still closes), the steps each pair flew (M,) and whether
    it was still closing at the cap (M,).
    """
    count = len(p_i)
    min_steps = _backup_steps(config)
    steps = max(min_steps, _backup_max_steps(config))
    axis = np.zeros((count, 3)) if axis is None else axis
    up_latched = np.zeros(count) if up_latched is None else up_latched
    arrays = [np.ascontiguousarray(x, dtype=np.float64) for x in (p_i, p_j, v_i, v_j, axis, up_latched)]
    # The kernel neither checks bounds nor propagates NaN as numpy does: reject
    # what the numpy rollout would have raised on or turned into NaN.
    if any(x.shape != (count, 3) for x in arrays[:5]) or arrays[5].shape != (count,):
        raise ValueError("backup rollout needs (M, 3) states and axes and (M,) up_latched")
    if not all(np.isfinite(x).all() for x in arrays):
        raise ValueError("backup rollout inputs must be finite")
    kernel = _rollout_kernel_parallel if count >= _PARALLEL_MIN_PAIRS else _rollout_kernel
    traj, a_first, closing, used, still = kernel(
        *arrays,
        _WORLD_MIN_M, _WORLD_MAX_M, float(config.margin_m), float(config.gamma),
        float(config.barrier_offset_mps), *_NEAR_VERTICAL_SIN, MAX_ACCEL_COMPONENT_MPS2,
        MAX_SPEED_MPS, SIMULATION_STEP_MS / 1000.0, min_steps, steps,
    )
    return traj[: used.max(initial=0)], a_first, closing, used.astype(np.int_), still


# Numba kernel of ``_backup_rollout``, one pair at a time (89 % of fuzz time
# in numpy, docs/results/profile-2026-10-23.md). It repeats the floating-point
# operations of the numpy helpers it inlines (_lateral with the default
# plumb_sin, _frame_lateral, _world_accel_limits, _evasive_direction) in the
# same order: norms and sums add x, y, z left to right, the einsum of the
# closing rate adds x, z, then y, as numpy does; no fastmath. On finite float64
# inputs its results match the numpy rollout bit for bit
# (tests/test_filter_rollout_kernel.py), and those helpers stay the reference
# for every other caller. _max and _min do not propagate NaN, so the wrapper
# rejects non-finite inputs.


@njit(cache=True)
def _max(a, b):
    return a if a >= b else b  # noqa: FURB136 (np.maximum, spelled out)


@njit(cache=True)
def _min(a, b):
    return a if a <= b else b  # noqa: FURB136 (np.minimum, spelled out)


@njit(cache=True)
def _norm3(x0, x1, x2):
    return math.sqrt(x0 * x0 + x1 * x1 + x2 * x2)


@njit(cache=True)
def _lateral_kernel(n0, n1, n2, lo, hi):
    horizontal = math.sqrt(n0 * n0 + n1 * n1)
    scale = _max(horizontal, 1e-12)
    t0 = (0.0 * n2 - 1.0 * n1) / scale
    t1 = (1.0 * n0 - 0.0 * n2) / scale
    t2 = (0.0 * n1 - 0.0 * n0) / scale
    if horizontal < 1e-9:  # min(max(plumb_sin, 1e-9), lo) at the default plumb_sin
        o0, o1, o2 = (1.0 if n2 >= 0.0 else -1.0), 0.0, 0.0
    else:
        o0, o1, o2 = n0 * 1.0, n1 * 1.0, n2 * 0.0
    dot = o0 * n0 + o1 * n1 + o2 * n2
    o0, o1, o2 = o0 - dot * n0, o1 - dot * n1, o2 - dot * n2
    scale = _max(_norm3(o0, o1, o2), 1e-12)
    o0, o1, o2 = o0 / scale, o1 / scale, o2 / scale
    weight = _min(_max((horizontal - lo) / (hi - lo), 0.0), 1.0)
    rest = 1.0 - weight
    l0, l1, l2 = weight * t0 + rest * o0, weight * t1 + rest * o1, weight * t2 + rest * o2
    scale = _norm3(l0, l1, l2)
    return l0 / scale, l1 / scale, l2 / scale


@njit(cache=True)
def _frame_lateral_kernel(n0, n1, n2, w0, w1, w2, lo, hi):
    c0, c1, c2 = w1 * n2 - w2 * n1, w2 * n0 - w0 * n2, w0 * n1 - w1 * n0
    size = _norm3(c0, c1, c2)
    if size > 0.5:
        scale = _max(size, 1e-12)
        return c0 / scale, c1 / scale, c2 / scale
    return _lateral_kernel(n0, n1, n2, lo, hi)


@njit(cache=True)
def _min_away_kernel(s, w, gamma, c, u_max):
    root = math.sqrt(2.0 * u_max * _max(s, 0.0) + c * c)
    if s >= 0.0:
        b, slope = root - c, u_max / root
    else:
        b, slope = u_max * s / c, u_max / c
    return _min(_max(-gamma * (b - w) + slope * w, -u_max), u_max)


@njit(cache=True)
def _limits_kernel(p, v, world_min, world_max, margin, gamma, c, u_max, low, high):
    for d in range(3):
        below = _min_away_kernel(p[d] - world_min[d] - margin, -v[d], gamma, c, u_max)
        above = -_min_away_kernel(world_max[d] - margin - p[d], v[d], gamma, c, u_max)
        nearer_min = (p[d] - world_min[d]) <= (world_max[d] - p[d])
        crossed = below > above
        low[d] = above if crossed and not nearer_min else below
        high[d] = below if crossed and nearer_min else above


@njit(cache=True)
def _evasive_kernel(n, up, low, high, w, lo, hi, u_max, out):
    l0, l1, l2 = _frame_lateral_kernel(n[0], n[1], n[2], w[0], w[1], w[2], lo, hi)
    d0, d1, d2 = n[0] + l0, n[1] + l1, n[2] + l2 + up
    scale = u_max / _max(_max(abs(d0), abs(d1)), abs(d2))
    out[0] = _min(_max(d0 * scale, low[0]), high[0])
    out[1] = _min(_max(d1 * scale, low[1]), high[1])
    out[2] = _min(_max(d2 * scale, low[2]), high[2])


@njit(cache=True)
def _rollout_pair(
    q, p_i, p_j, v_i, v_j, axis, up_latched, world_min, world_max, margin, gamma, c,
    lo, hi, u_max, v_max, dt, min_steps, steps, traj, a_first, closing, used, still,
):
    pi, pj, vi, vj = p_i[q].copy(), p_j[q].copy(), v_i[q].copy(), v_j[q].copy()
    wi, wj, n, m = np.empty(3), np.empty(3), np.empty(3), np.empty(3)
    ai, aj = np.empty(3), np.empty(3)
    low_i, high_i, low_j, high_j = np.empty(3), np.empty(3), np.empty(3), np.empty(3)
    for k in range(steps):
        r0, r1, r2 = pi[0] - pj[0], pi[1] - pj[1], pi[2] - pj[2]
        size = _max(_norm3(r0, r1, r2), 1e-9)
        n[0], n[1], n[2] = r0 / size, r1 / size, r2 / size
        m[0], m[1], m[2] = -n[0], -n[1], -n[2]
        # Pairs are (lower id, higher id); on equal altitude the lower id climbs.
        up = (1.0 if n[2] > 0.0 else -1.0) if abs(n[2]) > 1e-3 else 1.0
        if up_latched[q] != 0.0:
            up = up_latched[q]
        _limits_kernel(pi, vi, world_min, world_max, margin, gamma, c, u_max, low_i, high_i)
        _limits_kernel(pj, vj, world_min, world_max, margin, gamma, c, u_max, low_j, high_j)
        _evasive_kernel(n, up, low_i, high_i, axis[q], lo, hi, u_max, ai)
        _evasive_kernel(m, -up, low_j, high_j, axis[q], lo, hi, u_max, aj)
        if k == 0:
            for d in range(3):
                a_first[q, d] = ai[d] - aj[d]
        for d in range(3):
            wi[d] = vi[d] + ai[d] * dt
            wj[d] = vj[d] + aj[d] * dt
        cap_i = _min(1.0, v_max / _max(_norm3(wi[0], wi[1], wi[2]), 1e-9))
        cap_j = _min(1.0, v_max / _max(_norm3(wj[0], wj[1], wj[2]), 1e-9))
        for d in range(3):
            wi[d] = wi[d] * cap_i
            wj[d] = wj[d] * cap_j
            pi[d] = pi[d] + 0.5 * (vi[d] + wi[d]) * dt
            pj[d] = pj[d] + 0.5 * (vj[d] + wj[d]) * dt
            vi[d], vj[d] = wi[d], wj[d]
        r0, r1, r2 = pi[0] - pj[0], pi[1] - pj[1], pi[2] - pj[2]
        traj[k, q, 0], traj[k, q, 1], traj[k, q, 2] = r0, r1, r2
        u0, u1, u2 = vi[0] - vj[0], vi[1] - vj[1], vi[2] - vj[2]
        closing[q] = -(r0 * u0 + r2 * u2 + r1 * u1) / _max(_norm3(r0, r1, r2), 1e-9)
        if k + 1 >= min_steps and closing[q] <= 0.0:
            used[q] = k + 1
            still[q] = False
            break


@njit(cache=True)
def _rollout_kernel(
    p_i, p_j, v_i, v_j, axis, up_latched, world_min, world_max, margin, gamma, c,
    lo, hi, u_max, v_max, dt, min_steps, steps,
):
    count = p_i.shape[0]
    traj = np.full((steps, count, 3), np.nan)
    a_first, closing = np.zeros((count, 3)), np.zeros(count)
    used, still = np.full(count, steps), np.ones(count, dtype=np.bool_)
    for q in range(count):
        _rollout_pair(q, p_i, p_j, v_i, v_j, axis, up_latched, world_min, world_max, margin, gamma, c,
                      lo, hi, u_max, v_max, dt, min_steps, steps, traj, a_first, closing, used, still)
    return traj, a_first, closing, used, still


# The same kernel with pairs spread over threads. Pairs are independent and each
# writes only its own slots, so the results are those of _rollout_kernel bit for
# bit; used from _PARALLEL_MIN_PAIRS pairs, below which threading costs more.
@njit(cache=True, parallel=True)
def _rollout_kernel_parallel(
    p_i, p_j, v_i, v_j, axis, up_latched, world_min, world_max, margin, gamma, c,
    lo, hi, u_max, v_max, dt, min_steps, steps,
):
    count = p_i.shape[0]
    traj = np.full((steps, count, 3), np.nan)
    a_first, closing = np.zeros((count, 3)), np.zeros(count)
    used, still = np.full(count, steps), np.ones(count, dtype=np.bool_)
    for q in prange(count):
        _rollout_pair(q, p_i, p_j, v_i, v_j, axis, up_latched, world_min, world_max, margin, gamma, c,
                      lo, hi, u_max, v_max, dt, min_steps, steps, traj, a_first, closing, used, still)
    return traj, a_first, closing, used, still


_PARALLEL_MIN_PAIRS = 256


def _exact_range_gate(config: FilterConfig) -> float:
    """Separation (m) beyond which no row of a pair can bind (docs/architecture/PROPOSAL-scale-qp.md).

    Every row reads coef . (a_i - a_j) + s >= rhs with each acceleration
    component in [-u, u], so |a_i - a_j| <= a = 2 sqrt(3) u. A row with
    rhs <= -a |coef| holds for every acceleration in the box with s = 0, so
    its slack is 0 at every optimum and dropping the pair leaves the optimum
    unchanged. The returned distance is the largest of the three row types'
    thresholds, from bounds that hold for any state (speeds <= v_max):

    - backup: rhs = m . a_first - gamma (d* - d_eff) / ((k* + 0.5) dt) with
      m . a_first <= a, k* < ``_backup_max_steps`` and d* >= d0 - 2 v_max (k* + 1) dt
      (each drone moves at most v_max dt per rollout step);
    - escape (t* <= 3 s): coef = 0.5 t*^2 m, rhs = d_eff - m . (r + v t*) <=
      d_eff - d0 / sqrt(3) + 2 v_max t* (m . n >= 1 / sqrt(3));
    - braking: rhs <= -gamma B(s) + (gamma + 2 u / sqrt(4 u s + c^2)) 2 v_max with
      s >= d0 - d_eff - 2 v_max lag_s; then the nominal (also in the box) meets
      the row too, so no tie-break acts.
    """
    u, v_max, dt = MAX_ACCEL_COMPONENT_MPS2, MAX_SPEED_MPS, SIMULATION_STEP_MS / 1000.0
    a, gamma, c = 2.0 * math.sqrt(3.0) * u, config.gamma, config.barrier_offset_mps
    d_eff = SAFE_SEPARATION_M + config.margin_m
    backup = max(d_eff + 2.0 * v_max * (k + 1) * dt + 2.0 * a * (k + 0.5) * dt / gamma
                 for k in range(_backup_max_steps(config)))
    t = _ESCAPE_HORIZON_S
    escape = math.sqrt(3.0) * (d_eff + 2.0 * v_max * t + 0.5 * t * t * a)
    # gamma R - 4 u v_max / R >= gamma c + 2 v_max gamma + a with R = sqrt(4 u s + c^2)
    k = gamma * c + 2.0 * v_max * gamma + a
    root = (k + math.sqrt(k * k + 16.0 * gamma * u * v_max)) / (2.0 * gamma)
    braking = (root * root - c * c) / (4.0 * u) + d_eff + 2.0 * v_max * config.lag_s
    return max(backup, escape, braking)


def _backup_constraints(
    states: dict[str, DroneState], pairs: list[tuple[str, str]], config: FilterConfig,
    frames: dict[tuple[str, str], _PairLatch] | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Backup-set barrier rows coef . (a_i - a_j) >= rhs, one per pair.

    Both drones of a pair fly the evasive manoeuvre against each other from
    the current state in 100 ms zero-order-hold steps with the simulator's
    trapezoid position update and speed cap, each clipped to its own
    world-box limits: for an isolated pair this is what the fallback flies.
    The rollout (``_backup_rollout``) runs at least ``_backup_steps`` steps
    and continues while the pair closes, up to ``_backup_max_steps``. A pair
    still closing at the cap is treated conservatively: its row uses the
    closest distance reached so far, and it is reported as capped. h = min
    |r| - d_eff over that trajectory is the pair's margin, reached at step k*. The row asks
    the trajectory that starts with the QP's acceleration instead of the
    backup's first step to keep h_next >= (1 - gamma dt) h at k*, linearised
    as m . r_k*(a) with m = r_k* / |r_k*| (m . r <= |r|, so the row is
    sufficient) and d r_k* / d a = (k* + 0.5) dt^2. The row is divided by
    that sensitivity, so it has a unit normal m and its slack is in m/s² like
    the braking row (in metres OSQP stalls at 4000 iterations). The backup's
    first acceleration meets the row whenever h >= 0, so an isolated pair
    that is recoverable stays feasible. The per-step condition is only
    approximate (in the PR #12 review fuzz it failed on 0.74 % of steps, by
    at most 0.12 m), while forward invariance held: 0 of 1086 runs entered
    d_eff. Returns (m (M, 3), rhs (M,), steps flown (M,), capped (M,),
    backup margin d* - d_eff (M,)).

    known limit: pairwise backup (a drone in several conflicts gets one
    evasive per pair, not one manoeuvre: TASKS.md BL-03); only k* is
    constrained (BL-04); the sensitivity ignores the backup's feedback and
    the speed cap; lag_s only lengthens the horizon. Re-planned every step.
    All must be resolved before any real flight.
    """
    dt = SIMULATION_STEP_MS / 1000.0
    d_eff = SAFE_SEPARATION_M + config.margin_m

    axis, up = np.zeros((len(pairs), 3)), np.zeros(len(pairs))
    if frames:
        for k, pair in enumerate(pairs):
            frame = frames.get(pair)
            if frame is not None:
                axis[k], up[k] = frame.axis, frame.up
    traj, a_first, _, used, capped = _backup_rollout(*_pair_states(states, pairs), config, axis, up)
    dist = np.nan_to_num(np.linalg.norm(traj, axis=2), nan=np.inf)
    k_star = np.argmin(dist, axis=0)
    cols = np.arange(len(pairs))
    d_star = dist[k_star, cols]
    m = traj[k_star, cols] / np.maximum(d_star, 1e-9)[:, None]
    sensitivity = (k_star + 0.5) * dt * dt
    rhs = np.einsum("ij,ij->i", m, a_first) - config.gamma * dt * (d_star - d_eff) / sensitivity
    return m, rhs, used, capped, d_star - d_eff


def build_hocbf_qp_matrices(
    states: dict[str, DroneState],
    actions: dict[str, MARLAction],
    pairs_in_qp: list[tuple[str, str]],
    config: FilterConfig,
    escape_signs: dict[tuple[str, str], tuple[float, float]] | None = None,
    backup: tuple[np.ndarray, ...] | None = None,
    frames: dict[tuple[str, str], _PairLatch] | None = None,
    dense: bool = False,
) -> tuple[sp.csc_matrix, np.ndarray, sp.csc_matrix, np.ndarray, np.ndarray, list[str]]:
    """Build OSQP matrices for the joint HOCBF optimization problem.

    Decision vector x is stacked acceleration [a_0, a_1, ..., a_{N-1}] in R^{3N},
    ordered by agent_ids (sorted), followed by one slack s_k >= 0 per pair
    constraint (M pairs). Each pair constraint is softened to
    n_ij . (a_i - a_j) + s_k >= rhs, and the objective adds slack_weight * s_k,
    an exact L1 penalty: s_k stays 0 whenever the hard constraints are
    jointly satisfiable, so the QP is always feasible. Pairs in
    ``escape_signs`` use ``_escape_constraint`` instead, with the slack in
    metres, and get no tie-break. With barrier="backup" every other pair
    uses ``_backup_constraints`` instead, with no tie-break; pass its result
    as ``backup`` to reuse it across the solves of one step. ``frames`` holds
    the latched sideways frames of pairs in conflict (``_update_latches``).
    ``dense`` returns P and A as numpy arrays with the same entries (zeros
    +0.0), for the small QPs DAQP solves, without the cost of scipy blocks.

    Returns
    -------
    P: csc_matrix ((3N + M) x (3N + M))
    q: ndarray (3N + M,)
    A: csc_matrix ((M + 3N + M) x (3N + M)); rows: pairs, accel box, slack >= 0
    l: ndarray (M + 3N + M,)
    u: ndarray (M + 3N + M,)
    agent_ids: list[str]
    """
    agent_ids = sorted(states.keys())
    agent_idx = {aid: idx for idx, aid in enumerate(agent_ids)}
    n_agents = len(agent_ids)
    dim = 3 * n_agents
    n_pairs = len(pairs_in_qp)

    # Objective: min sum_i ||a_i - a_i^nom||^2 + w sum_k s_k
    #          = min 0.5 * x^T P x + q^T x (constant dropped)
    # Escape slacks (metres) take a quadratic penalty instead of the linear
    # one: their violation is unavoidable, and squaring spreads it across
    # pairs instead of loading one pair (an approximation of max-min).
    escape_signs = escape_signs or {}
    is_escape = np.array([pair in escape_signs for pair in pairs_in_qp], dtype=bool)
    slack_hessian = np.where(is_escape, 2.0 * _ESCAPE_WEIGHT, 0.0)
    nom_stacked = np.zeros(dim, dtype=np.float64)
    for aid, idx in agent_idx.items():
        nom_stacked[3 * idx : 3 * idx + 3] = actions[aid].proposed_accel_mps2
    target = nom_stacked.copy()

    # Box bounds on control accelerations: -u_max <= a_{i,k} <= u_max,
    # tightened per drone by the world-box barriers (hard: no slack).
    l_box, u_box = (
        limit.ravel()
        for limit in _world_accel_limits(
            np.array([states[aid].position_m for aid in agent_ids], dtype=np.float64),
            np.array([states[aid].velocity_mps for aid in agent_ids], dtype=np.float64),
            config,
        )
    )

    # Pairwise braking-barrier constraints (P0-SA-01 Section 6-8):
    # n_ij^T (a_i - a_j) >= rhs, one row per pair with coef at agent i and
    # -coef at agent j, built straight into sparse form (exact zeros dropped,
    # as a dense row converted to CSC drops them).
    if config.barrier != "backup":
        backup = None
    elif backup is None:
        backup = _backup_constraints(states, pairs_in_qp, config, frames)
    coef = np.zeros((n_pairs, 3), dtype=np.float64)
    l_cbf = np.zeros(n_pairs, dtype=np.float64)
    if backup is not None and n_pairs:
        coef[:], l_cbf[:] = backup[0], backup[1]
    for k, (id_i, id_j) in enumerate(pairs_in_qp):
        if (id_i, id_j) in escape_signs:
            coef[k], l_cbf[k] = _escape_constraint(
                states[id_i], states[id_j], escape_signs[(id_i, id_j)], config, (frames or {}).get((id_i, id_j))
            )
        elif backup is None:
            idx_i, idx_j = agent_idx[id_i], agent_idx[id_j]
            coef[k], l_cbf[k], _ = _pair_constraint(states[id_i], states[id_j], config)
            row = np.zeros(dim, dtype=np.float64)
            row[3 * idx_i : 3 * idx_i + 3] = coef[k]
            row[3 * idx_j : 3 * idx_j + 3] = -coef[k]

            # Symmetric pairs stall at d_eff with no lateral escape. When the
            # nominal would violate the constraint, bias both agents apart
            # sideways (``_lateral``: right turn about +z away from vertical).
            if config.tie_break_mps2 > 0.0 and float(row @ nom_stacked) < l_cbf[k]:
                lateral = config.tie_break_mps2 * _frame_lateral(coef[k], _latch_axis(frames, (id_i, id_j)))
                target[3 * idx_i : 3 * idx_i + 3] += lateral
                target[3 * idx_j : 3 * idx_j + 3] -= lateral

    q = np.concatenate([-2.0 * target, np.where(is_escape, 0.0, config.slack_weight)])
    l = np.concatenate([l_cbf, l_box, np.zeros(n_pairs)])
    u = np.concatenate([np.full(n_pairs, np.inf), u_box, np.full(n_pairs, np.inf)])
    idx_i = np.array([agent_idx[p[0]] for p in pairs_in_qp], dtype=np.intp)
    idx_j = np.array([agent_idx[p[1]] for p in pairs_in_qp], dtype=np.intp)
    cols = np.concatenate([3 * idx_i[:, None] + np.arange(3), 3 * idx_j[:, None] + np.arange(3)], axis=1)
    data = np.concatenate([coef, -coef], axis=1)
    rows = np.repeat(np.arange(n_pairs), 6).reshape(n_pairs, 6)
    keep = data != 0.0

    if dense:
        n = dim + n_pairs
        slack = dim + np.arange(n_pairs)
        P = np.zeros((n, n))
        P[np.arange(dim), np.arange(dim)] = 2.0
        P[slack, slack] = slack_hessian
        A = np.zeros((2 * n_pairs + dim, n))
        A[rows[keep], cols[keep]] = data[keep]
        A[np.arange(n_pairs), slack] = 1.0
        A[n_pairs + np.arange(dim), np.arange(dim)] = 1.0
        A[n_pairs + dim + np.arange(n_pairs), slack] = 1.0
        return P, q, A, l, u, agent_ids

    P = sp.block_diag([sp.eye(dim) * 2.0, sp.diags(slack_hessian)], format="csc")
    A_box = sp.hstack([sp.eye(dim), sp.csc_matrix((dim, n_pairs))])
    if n_pairs:
        A_rows = sp.csc_matrix((data[keep], (rows[keep], cols[keep])), shape=(n_pairs, dim))
        A_cbf = sp.hstack([A_rows, sp.eye(n_pairs)])
        A_slack = sp.hstack([sp.csc_matrix((n_pairs, dim)), sp.eye(n_pairs)])
        A = sp.vstack([A_cbf, A_box, A_slack], format="csc")
    else:
        A = sp.csc_matrix(A_box)

    return P, q, A, l, u, agent_ids


def _solve_hocbf_qp(
    states: dict[str, DroneState],
    actions: dict[str, MARLAction],
    pairs_in_qp: list[tuple[str, str]],
    config: FilterConfig,
    escape_signs: dict[tuple[str, str], tuple[float, float]] | None = None,
    backup: tuple[np.ndarray, ...] | None = None,
    frames: dict[tuple[str, str], _PairLatch] | None = None,
    *,
    deadline: float | None = None,
    missed: list[tuple[list[str], list[tuple[str, str]]]] | None = None,
    reuse: dict | None = None,
    skip: set[str] | None = None,
) -> tuple[dict[str, tuple[float, float, float]], dict[tuple[str, str], float], bool]:
    """Solve the joint HOCBF QP over all active agents, as its independent problems.

    ``_independent_problems`` drops rows no acceleration in the box can
    violate and gives each connected group of agents its own QP, with the
    joint QP's optimum; each is solved to KKT accuracy (``_solve_exact``).
    Any problem's solver error is a solver error of the step.

    With a ``deadline`` (``time.perf_counter()`` seconds, ``_solve_deadline``)
    the problems are solved most urgent first (smallest backup margin, or
    braking barrier value, of their rows), and each gets its own deadline
    (``_cluster_deadlines``): its fair share of the window is guaranteed, so a
    slow problem cannot take time from another. A problem not solved by its
    deadline is appended to ``missed`` as (agents, pairs) and its agents are
    left out of the returned accelerations. ``reuse`` caches the solution of
    each problem without escape rows by (agents, pairs), so the escape stage
    does not solve an unchanged QP twice (deterministic solvers: the same
    result). Problems with an agent in ``skip`` are left out.

    Returns
    -------
    safe_accels:
        Dict mapping agent_id to (ax, ay, az). Empty on solver error.
    pair_slack:
        Slack per pair in m/s², only for pairs above the intervention
        tolerance. Positive slack means that pair's barrier is violated.
    qp_feasible:
        True if OSQP status is 'solved', False otherwise. With slack the
        problem is always feasible, so False means a solver error.
    """
    if not states or not actions:
        return {}, {}, True

    if not pairs_in_qp:
        # No active pairs to constrain; the nominal only needs the world-box
        # limits the QP would apply (PR #12 re-review: a lone drone left the box).
        return _clipped_nominal(states, actions, list(actions), config), {}, True

    try:
        if config.barrier == "backup" and backup is None:
            backup = _backup_constraints(states, pairs_in_qp, config, frames)
        safe_accels, pair_slack = {}, {}
        problems = []
        for agents, rows in _independent_problems(states, actions, pairs_in_qp, config, escape_signs, backup, frames):
            if not len(rows):
                # No row can bind: the minimiser of |a - a_nom|^2 over the box.
                safe_accels.update(_clipped_nominal(states, actions, agents, config))
            elif not (skip and skip.intersection(agents)):
                problems.append((agents, rows))
        if backup is not None:
            margin = backup[4]
        else:
            margin = np.array([_pair_constraint(states[a], states[b], config)[2] for a, b in pairs_in_qp])
        problems.sort(key=lambda prob: (float(np.min(margin[prob[1]])), prob[0]))
        deadlines = _cluster_deadlines(deadline, [3 * len(agents) + len(rows) for agents, rows in problems])
        for (agents, rows), until in zip(problems, deadlines, strict=True):
            pairs = [pairs_in_qp[k] for k in rows]
            signs = {pair: escape_signs[pair] for pair in pairs if pair in escape_signs} if escape_signs else None
            key = (tuple(agents), tuple(pairs))
            if not signs and reuse is not None and key in reuse:
                solved = reuse[key]
            else:
                solved = _solve_one_qp(
                    {aid: states[aid] for aid in agents}, {aid: actions[aid] for aid in agents}, pairs, config,
                    signs, None if backup is None else tuple(x[rows] for x in backup), frames, until,
                )
            if solved is _DEADLINE:
                if missed is not None:
                    missed.append((agents, pairs))
                continue
            if solved is None:
                return {}, {}, False
            if not signs and reuse is not None:
                reuse[key] = solved
            safe_accels.update(solved[0])
            pair_slack.update(solved[1])
        return safe_accels, {pair: pair_slack[pair] for pair in pairs_in_qp if pair in pair_slack}, True
    except Exception as exc:  # noqa: BLE001
        logger.error("filter: QP solve exception: %s", exc)
        return {}, {}, False


# Step-time budget (founder decision 2026-10-24, docs/results/deadline-2026-10-24.md).
# The filter decides once per 100 ms step. After the solves it still selects the
# escape pairs, flies the backup of missed clusters, fills the diagnostics and
# builds the SafeActions: measured at most 3.4 ms at N = 64 and 9.6 ms at N = 128,
# so 10 ms are kept for that. The solves of both stages get what is left of the
# step; when the work before them already used it (N >= 128 on a 4-core machine),
# they still get 25 ms, above the measured 99.9th percentile of a whole solve
# stage at N = 64 (16.5 ms), so the worst step is bounded by the work before the
# solves plus 25 ms plus the reserve.
_STEP_BUDGET_S = SIMULATION_STEP_MS / 1000.0
_POST_SOLVE_RESERVE_S = 0.010
_MIN_SOLVE_WINDOW_S = 0.025


_warmed_up = False


def _warm_up() -> None:
    """Pay the filter's first-call costs once per process, outside any step's clock.

    Measured on the first ``apply_filter`` call of a process, 24 drones and
    276 pairs (docs/results/deadline-2026-10-24.md): the backup rollout took
    7.1 s with a cold Numba cache (compiling the threaded kernel) and 0.28 s
    with a warm one (cache load, thread pool start), against 2-4 ms on later
    steps. That time fell inside step 0's clock, so its solves were left only
    ``_MIN_SOLVE_WINDOW_S``, and under load they missed it. This runs both
    rollout kernels, a latch update and a DAQP and an OSQP solve on a small
    three-drone conflict before the first step's clock starts, and changes no
    result (it uses its own latch dict; the filter keeps no other state).
    Best effort: an error here is left to the step, which handles solver errors.
    """
    global _warmed_up
    _warmed_up = True
    try:
        _warm_up_calls()
    except Exception as exc:  # noqa: BLE001
        logger.debug("filter: warm-up stopped early: %s", exc)


def _warm_up_calls() -> None:
    config = FilterConfig(step_deadline=False)
    p = np.array([[500.0, 500.0, 100.0], [520.0, 500.0, 100.0], [510.0, 515.0, 100.0]])
    v = np.array([[5.0, 0.0, 0.0], [-5.0, 0.0, 0.0], [0.0, -5.0, 0.0]])
    states = {f"w{k}": DroneState(agent_id=f"w{k}", position_m=tuple(p[k]), velocity_mps=tuple(v[k]),
                                  goal_m=(900.0, 900.0, 100.0), timestamp_ms=0) for k in range(3)}
    actions = {aid: MARLAction(agent_id=aid, proposed_accel_mps2=(0.0, 0.0, 0.0), timestamp_ms=0) for aid in states}
    pairs = [("w0", "w1"), ("w0", "w2"), ("w1", "w2")]
    frames = _update_latches(states, pairs, config, {})
    backup = _backup_constraints(states, pairs, config, frames)
    pair_arrays = [np.repeat(x, 2, axis=0) for x in _pair_states(states, pairs[:1])]
    _rollout_kernel_parallel(
        *pair_arrays, np.zeros((2, 3)), np.zeros(2), _WORLD_MIN_M, _WORLD_MAX_M, float(config.margin_m),
        float(config.gamma), float(config.barrier_offset_mps), *_NEAR_VERTICAL_SIN, MAX_ACCEL_COMPONENT_MPS2,
        MAX_SPEED_MPS, SIMULATION_STEP_MS / 1000.0, _backup_steps(config), _backup_max_steps(config),
    )
    for dense in (True, False):
        P, q, A, l, u, _ = build_hocbf_qp_matrices(states, actions, pairs, config, None, backup, frames, dense)
        _solve_daqp(P, q, A, l, u) if dense else _solve_osqp(P, q, A, l, u, config, False)


def _solve_deadline(step_start: float) -> float:
    """Absolute end (``time.perf_counter()`` s) of the solve window of a step that started at ``step_start``."""
    return max(step_start + _STEP_BUDGET_S - _POST_SOLVE_RESERVE_S, time.perf_counter() + _MIN_SOLVE_WINDOW_S)


def _cluster_deadlines(deadline: float | None, sizes: list[int]) -> list[float | None]:
    """Deadline of each problem, in solving order, for a window ending at ``deadline``.

    Each problem's guaranteed share of the window is proportional to its size
    (QP variables). Problem k may run until the window end less the shares of
    the problems after it, so it can use time the earlier ones left, but never
    a later problem's share: however long problem k takes, problem k + 1 still
    starts with at least its own share.
    """
    if deadline is None or not sizes:
        return [None] * len(sizes)
    window = max(deadline - time.perf_counter(), 0.0)
    total = float(sum(sizes)) or 1.0
    after = np.concatenate([np.cumsum(np.asarray(sizes[::-1], dtype=np.float64))[::-1][1:], [0.0]])
    return [deadline - window * float(rest) / total for rest in after]


def _clipped_nominal(
    states: dict[str, DroneState], actions: dict[str, MARLAction], agents: list[str], config: FilterConfig
) -> dict[str, tuple[float, float, float]]:
    """Nominal accelerations clipped to each drone's world-box limits."""
    safe_accels = {}
    for aid in agents:
        state = states[aid]
        limits = _world_accel_limits(
            np.array(state.position_m, dtype=np.float64), np.array(state.velocity_mps, dtype=np.float64), config
        )
        safe_accels[aid] = tuple(float(c) for c in np.clip(actions[aid].proposed_accel_mps2, *limits))
    return safe_accels


def _independent_problems(
    states: dict[str, DroneState],
    actions: dict[str, MARLAction],
    pairs_in_qp: list[tuple[str, str]],
    config: FilterConfig,
    escape_signs: dict[tuple[str, str], tuple[float, float]] | None,
    backup: tuple[np.ndarray, ...] | None,
    frames: dict[tuple[str, str], _PairLatch] | None,
) -> list[tuple[list[str], np.ndarray]]:
    """Split the joint QP into independent problems with the same optimum.

    Returns (agents, row indices into pairs_in_qp) per problem; agents whose
    rows are all dropped come as one problem with no rows. A pair row
    coef . (a_i - a_j) + s >= rhs is dropped when every acceleration in the
    box (``_world_accel_limits``, a hard constraint) meets it with s = 0, that
    is when min over the box of coef . (a_i - a_j) >= rhs. Its slack then
    costs (linear or quadratic, both > 0 for s > 0) and buys nothing, so it is
    0 at every optimum and the row never binds: removing it changes neither
    the feasible set of the accelerations nor the optimum. A braking row is
    kept when its tie-break acts (nominal violates it), since that shifts the
    target. The objective is a sum over agents, so after the drop the QP
    separates over the connected components of the remaining pair graph; a
    component's QP has the joint QP's optimum restricted to its agents, and
    with KKT-certified solves (``_solve_exact``) the solutions agree within
    the certificate whichever way the QP is split. A small margin (1e-9
    relative) keeps rows within round-off of the test.
    """
    agent_ids = sorted(states)
    index = {aid: k for k, aid in enumerate(agent_ids)}
    low, high = _world_accel_limits(
        np.array([states[aid].position_m for aid in agent_ids], dtype=np.float64).reshape(-1, 3),
        np.array([states[aid].velocity_mps for aid in agent_ids], dtype=np.float64).reshape(-1, 3),
        config,
    )
    n_pairs = len(pairs_in_qp)
    escape_signs = escape_signs or {}
    coef = np.zeros((n_pairs, 3))
    rhs = np.zeros(n_pairs)
    keep = np.zeros(n_pairs, dtype=bool)
    if backup is not None:
        coef[:], rhs[:] = backup[0], backup[1]
    for k, (id_i, id_j) in enumerate(pairs_in_qp):
        if (id_i, id_j) in escape_signs:
            coef[k], rhs[k] = _escape_constraint(
                states[id_i], states[id_j], escape_signs[(id_i, id_j)], config, (frames or {}).get((id_i, id_j))
            )
        elif backup is None:
            coef[k], rhs[k], _ = _pair_constraint(states[id_i], states[id_j], config)
            nominal = np.subtract(actions[id_i].proposed_accel_mps2, actions[id_j].proposed_accel_mps2)
            keep[k] = config.tie_break_mps2 > 0.0 and float(coef[k] @ nominal) < rhs[k] + 1e-9 * (1.0 + abs(rhs[k]))
    i = np.array([index[p[0]] for p in pairs_in_qp], dtype=np.intp)
    j = np.array([index[p[1]] for p in pairs_in_qp], dtype=np.intp)
    if n_pairs:
        lowest = (np.minimum(coef * low[i], coef * high[i]) - np.maximum(coef * low[j], coef * high[j])).sum(axis=1)
        keep |= ~(lowest >= rhs + 1e-9 * (1.0 + np.abs(rhs)))
    parent = list(range(len(agent_ids)))

    def find(a: int) -> int:
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for k in np.flatnonzero(keep):
        parent[find(int(i[k]))] = find(int(j[k]))
    groups: dict[int, list[str]] = {}
    for aid in agent_ids:
        groups.setdefault(find(index[aid]), []).append(aid)
    rows: dict[int, list[int]] = {}
    for k in np.flatnonzero(keep):
        rows.setdefault(find(int(i[k])), []).append(int(k))
    problems = [(agents, np.array(rows[root], dtype=np.intp)) for root, agents in groups.items() if root in rows]
    free = [aid for root, agents in groups.items() if root not in rows for aid in agents]
    if free:
        problems.append((free, np.zeros(0, dtype=np.intp)))
    return problems


def _solve_one_qp(
    states: dict[str, DroneState],
    actions: dict[str, MARLAction],
    pairs_in_qp: list[tuple[str, str]],
    config: FilterConfig,
    escape_signs: dict[tuple[str, str], tuple[float, float]] | None,
    backup: tuple[np.ndarray, ...] | None,
    frames: dict[tuple[str, str], _PairLatch] | None,
    deadline: float | None = None,
) -> tuple[dict[str, tuple[float, float, float]], dict[tuple[str, str], float]] | None | object:
    """One QP of ``build_hocbf_qp_matrices``, KKT-certified: (safe_accels, pair_slack).

    None on solver error, _DEADLINE when ``deadline`` passed first."""
    if deadline is not None and time.perf_counter() >= deadline:
        return _DEADLINE
    dense = 3 * len(states) + len(pairs_in_qp) <= _EXACT_MAX_VARS
    P, q, A, l, u, agent_ids = build_hocbf_qp_matrices(
        states, actions, pairs_in_qp, config, escape_signs, backup, frames, dense
    )
    x = _solve_exact(P, q, A, l, u, config, bool(escape_signs), deadline)
    if x is _DEADLINE:
        return _DEADLINE
    if x is None:
        logger.warning("filter: no solver returned a KKT-certified solution")
        return None
    u_max = MAX_ACCEL_COMPONENT_MPS2
    safe_accels = {}
    for idx, aid in enumerate(agent_ids):
        ax = float(np.clip(x[3 * idx], -u_max, u_max))
        ay = float(np.clip(x[3 * idx + 1], -u_max, u_max))
        az = float(np.clip(x[3 * idx + 2], -u_max, u_max))
        safe_accels[aid] = (ax, ay, az)
    slacks = x[3 * len(agent_ids):]
    pair_slack = {
        pair: float(s)
        for pair, s in zip(pairs_in_qp, slacks, strict=True)
        if s > SHIELD_INTERVENTION_TOLERANCE_MPS2
    }
    return safe_accels, pair_slack


# The QP is solved to KKT accuracy (founder decision 2026-10-24,
# docs/architecture/PROPOSAL-scale-qp.md option 1), so its solution is the
# optimum (unique: the objective is strictly convex in the accelerations, and
# each slack is then fixed) and not an artefact of the solver or of which
# drones share the QP. Measured on 21 259 fuzz QPs and joint QPs at N = 16-64
# (docs/results/solver-exact-2026-10-24.md): DAQP (dual active set, dense),
# OSQP at eps 1e-9 and proxsuite agree within 5e-8 m/s² and all pass the
# certificate; DAQP is faster up to about 300 variables (2.3 against 12 ms at
# 168), OSQP above (49 against 117 ms at 592). OSQP at the old eps 1e-5 with
# polish was off by up to 0.05 m/s² on 0.7 % of the fuzz QPs.
_EXACT_MAX_VARS = 256
# A solve stopped by its deadline (``_solve_deadline``), not a solver error.
_DEADLINE = object()
_DAQP_TIME_LIMIT = -7  # DAQP exit flag: time limit reached
# Certificate tolerances, in the form of OSQP's own test: primal feasibility
# per row relative to its bound (backup rows reach |rhs| ~ 1e3), stationarity
# per component against |Px| + |q| + |A|'|y| (multipliers reach the slack
# weight, 1e4), each with an absolute floor. Set from measured residuals:
# DAQP leaves up to 3e-8 of row violation and round-off stationarity; OSQP at
# eps 1e-9 up to 1.4e-8, but up to 7e-7 on some escape QPs, which therefore
# fail and go to DAQP (their accelerations are within 2e-8 m/s² of DAQP's).
# A multiplier counts as nonzero above _KKT_TOL.
_KKT_TOL = 1e-7
_KKT_STATIONARITY_RTOL = 1e-10
_DAQP_INF = 1e30


def _kkt_certified(
    P: sp.csc_matrix, q: np.ndarray, A: sp.csc_matrix, l: np.ndarray, u: np.ndarray, x: np.ndarray, y: np.ndarray
) -> bool:
    """Whether (x, y) satisfies the KKT conditions of min 0.5 x'Px + q'x s.t. l <= Ax <= u.

    OSQP's sign convention: y_i < 0 on an active lower bound, y_i > 0 on an
    active upper bound. Checks primal feasibility, stationarity Px + q + A'y = 0,
    the sign of y (none towards an infinite bound) and complementarity.
    """
    if not (np.all(np.isfinite(x)) and np.all(np.isfinite(y))):
        return False
    Ax = A @ x
    finite_l, finite_u = np.isfinite(l), np.isfinite(u)
    tol_l = _KKT_TOL * (1.0 + np.abs(np.where(finite_l, l, 0.0)))
    tol_u = _KKT_TOL * (1.0 + np.abs(np.where(finite_u, u, 0.0)))
    if np.any((l - Ax)[finite_l] > tol_l[finite_l]) or np.any((Ax - u)[finite_u] > tol_u[finite_u]):
        return False
    size = np.abs(P) @ np.abs(x) + np.abs(q) + np.abs(A).T @ np.abs(y)
    if np.any(np.abs(P @ x + q + A.T @ y) > _KKT_TOL + _KKT_STATIONARITY_RTOL * size):
        return False
    lower, upper = np.maximum(-y, 0.0), np.maximum(y, 0.0)
    if np.any(lower[~finite_l] > _KKT_TOL) or np.any(upper[~finite_u] > _KKT_TOL):
        return False
    # Complementarity: a multiplier on a row the solution does not touch (gap
    # beyond the primal tolerance) must vanish.
    gap_l = (Ax - l)[finite_l] > tol_l[finite_l]
    gap_u = (u - Ax)[finite_u] > tol_u[finite_u]
    return bool(not np.any(lower[finite_l][gap_l] > _KKT_TOL) and not np.any(upper[finite_u][gap_u] > _KKT_TOL))


def _solve_daqp(
    P: sp.csc_matrix, q: np.ndarray, A: sp.csc_matrix, l: np.ndarray, u: np.ndarray, time_limit: float | None = None,
) -> tuple[np.ndarray, np.ndarray] | None | object:
    """DAQP on the QP of ``build_hocbf_qp_matrices``: (x, y in OSQP's convention), None on failure.

    Rows with a single unit entry (the acceleration box and slack >= 0) become
    DAQP's simple bounds, the others its general rows. P is singular in the
    linearly penalised slacks; DAQP's automatic proximal regularisation
    (eps_prox < 0) solves such QPs to the exact optimum.
    """
    n = P.shape[0]
    if sp.issparse(A):
        rows = A.tocsr()
        single = np.flatnonzero(np.diff(rows.indptr) == 1)
        single = single[rows.data[rows.indptr[single]] == 1.0]
        cols = rows.indices[rows.indptr[single]]
        H = P.toarray()
    else:  # dense (``build_hocbf_qp_matrices(dense=True)``): the same rows
        rows, H = A, P
        single = np.flatnonzero(np.count_nonzero(A, axis=1) == 1)
        cols = np.argmax(A[single] != 0.0, axis=1)
        single, cols = single[A[single, cols] == 1.0], cols[A[single, cols] == 1.0]
    low, high = np.full(n, -_DAQP_INF), np.full(n, _DAQP_INF)
    np.maximum.at(low, cols, np.where(np.isfinite(l[single]), l[single], -_DAQP_INF))
    np.minimum.at(high, cols, np.where(np.isfinite(u[single]), u[single], _DAQP_INF))
    general = np.setdiff1d(np.arange(A.shape[0]), single)
    lower = np.concatenate([low, np.where(np.isfinite(l[general]), l[general], -_DAQP_INF)])
    upper = np.concatenate([high, np.where(np.isfinite(u[general]), u[general], _DAQP_INF)])
    general_rows = rows[general].toarray() if sp.issparse(rows) else np.ascontiguousarray(rows[general])
    x, _, flag, info = daqp.solve(
        H, np.ascontiguousarray(q, dtype=np.float64), general_rows, upper, lower,
        np.zeros(len(upper), dtype=np.int32), primal_tol=1e-10,
        **({} if time_limit is None else {"time_limit": max(time_limit, 1e-6)}),
    )
    if flag == _DAQP_TIME_LIMIT:
        return _DEADLINE
    if flag != 1:
        return None
    lam = np.asarray(info["lam"], dtype=np.float64)
    y = np.zeros(A.shape[0])
    first = np.unique(cols, return_index=True)[1]  # one bound row per variable carries its multiplier
    y[single[first]] = lam[cols[first]]
    y[general] = lam[n:]
    return np.asarray(x, dtype=np.float64), y


def _solve_osqp(
    P: sp.csc_matrix, q: np.ndarray, A: sp.csc_matrix, l: np.ndarray, u: np.ndarray, config: FilterConfig,
    escape: bool, deadline: float | None = None,
) -> tuple[np.ndarray, np.ndarray] | None | object:
    """OSQP at config.solver_eps: (x, y), None when it does not converge, _DEADLINE past ``deadline``."""
    if not sp.issparse(A):
        P, A = sp.csc_matrix(P), sp.csc_matrix(A)
    # On non-convergence retry once without over-relaxation (alpha 1.0
    # instead of OSQP's 1.6): fixes HARD fuzz seeds 864 and 977, where
    # rho alone (up to 10) and 20000 iterations did not (PR #12 review).
    for alpha in (1.6, 1.0):
        limit = {} if deadline is None else {"time_limit": deadline - time.perf_counter()}
        if limit and limit["time_limit"] <= 0.0:
            return _DEADLINE
        prob = osqp.OSQP()
        prob.setup(
            P, q, A, l, u,
            verbose=False,
            eps_abs=config.solver_eps,
            eps_rel=config.solver_eps,
            max_iter=config.solver_max_iter,
            # OSQP's default (0) times the setup to pick this interval, which
            # makes same-seed runs differ. A fixed interval keeps them identical.
            adaptive_rho_interval=25,
            polish=True,
            # The escape QP saturates the box with a degenerate active set;
            # at the default rho (0.1) ADMM needs many more iterations there.
            rho=10.0 if escape else 0.1,
            alpha=alpha,
            **limit,
        )
        res = prob.solve()
        if res.info.status == "solved":
            return res.x, res.y
        if res.info.status == "run time limit reached":
            return _DEADLINE
        logger.warning("filter: OSQP status %s at alpha=%.1f", res.info.status, alpha)
    return None


def _solve_exact(
    P: sp.csc_matrix, q: np.ndarray, A: sp.csc_matrix, l: np.ndarray, u: np.ndarray, config: FilterConfig,
    escape: bool, deadline: float | None = None,
) -> np.ndarray | None | object:
    """KKT-certified solution x of the QP; None when no solver returns one; _DEADLINE when time ran out.

    DAQP first up to _EXACT_MAX_VARS variables, OSQP first above; the other
    one when the first fails or its solution fails ``_kkt_certified``. With a
    ``deadline`` (``time.perf_counter()`` seconds) each solver gets the time
    left as its own time limit, and a solver stopped by it, or no time left
    for the next one, makes the result _DEADLINE.
    """
    small = P.shape[0] <= _EXACT_MAX_VARS
    order = (_solve_daqp, _solve_osqp) if small else (_solve_osqp, _solve_daqp)
    for solver in order:
        left = None if deadline is None else deadline - time.perf_counter()
        if left is not None and left <= 0.0:
            return _DEADLINE
        try:
            if solver is _solve_daqp:
                solution = solver(P, q, A, l, u, left)
            else:
                solution = solver(P, q, A, l, u, config, escape, deadline)
        except Exception as exc:  # noqa: BLE001
            logger.warning("filter: %s raised %s", solver.__name__, exc)
            continue
        if solution is _DEADLINE:
            return _DEADLINE
        if solution is not None and _kkt_certified(P, q, A, l, u, *solution):
            return solution[0]
        logger.warning("filter: %s returned no KKT-certified solution", solver.__name__)
    return None


def _evasive_accel(
    aid: str,
    state_map: dict[str, DroneState],
    pairs_in_qp: list[tuple[str, str]],
    config: FilterConfig,
    frames: dict[tuple[str, str], _PairLatch] | None = None,
) -> tuple[float, float, float]:
    """Fallback manoeuvre away from the most critical neighbour.

    Most critical = lowest braking barrier h. The direction sums the unit
    line-of-sight vector away from the neighbour (relative braking), the
    sideways unit vector ``_lateral`` and a vertical unit vector (away in z;
    on equal altitude the lower agent id climbs). Both agents of a pair
    compute opposite directions from the same rule. The direction is scaled
    so its largest component is u_max and clipped to the drone's world-box
    limits, so it never pushes the drone out of the world (BL-01).

    known limit: ignores third neighbours, so it can push a drone towards
    another drone; revisit with the PR B lateral-escape proposal.
    """
    best: tuple[float, str] | None = None
    for id_i, id_j in pairs_in_qp:
        if aid in (id_i, id_j):
            other = id_j if aid == id_i else id_i
            _, _, h = _pair_constraint(state_map[aid], state_map[other], config)
            if best is None or h < best[0]:
                best = (h, other)
    u_max = MAX_ACCEL_COMPONENT_MPS2
    if best is None:
        return (0.0, 0.0, 0.0)
    other = best[1]
    n, _, _ = _pair_constraint(state_map[aid], state_map[other], config)
    frame = (frames or {}).get(tuple(sorted((aid, other))))
    if frame is not None:
        up = frame.up if aid < other else -frame.up
    elif abs(n[2]) > 1e-3:
        up = math.copysign(1.0, n[2])
    else:
        up = 1.0 if aid < other else -1.0
    state = state_map[aid]
    limits = _world_accel_limits(
        np.array(state.position_m, dtype=np.float64), np.array(state.velocity_mps, dtype=np.float64), config
    )
    direction = _evasive_direction(n, up, limits, _latch_axis(frames, tuple(sorted((aid, other)))))
    return tuple(float(c) for c in np.clip(direction, -u_max, u_max))


def _fallback_cluster(
    state_map: dict[str, DroneState],
    action_map: dict[str, MARLAction],
    pairs_in_qp: list[tuple[str, str]],
    config: FilterConfig,
    frames: dict[tuple[str, str], _PairLatch] | None = None,
) -> dict[str, tuple[float, float, float]]:
    """Evasive accelerations for the conflict cluster after a solver error.

    The cluster starts with every agent in a pair whose constraint the
    nominal actions violate. It then grows until every agent outside it
    satisfies all its pair constraints given the cluster's evasive actions.
    Agents outside the cluster keep their nominal action.
    """
    constraints = {
        pair: _pair_constraint(state_map[pair[0]], state_map[pair[1]], config)
        for pair in pairs_in_qp
    }
    cluster: set[str] = set()
    evasive: dict[str, tuple[float, float, float]] = {}

    def accel(aid: str) -> np.ndarray:
        return np.array(evasive[aid] if aid in cluster else action_map[aid].proposed_accel_mps2)

    while True:
        grow = set()
        for (id_i, id_j), (n, rhs, _) in constraints.items():
            if {id_i, id_j} <= cluster:
                continue
            if float(n @ (accel(id_i) - accel(id_j))) < rhs - SHIELD_INTERVENTION_TOLERANCE_MPS2:
                grow |= {id_i, id_j} - cluster
        if not grow:
            return evasive
        cluster |= grow
        for aid in grow:
            evasive[aid] = _evasive_accel(aid, state_map, pairs_in_qp, config, frames)


def apply_filter(
    states: Sequence[DroneState],
    actions: Sequence[MARLAction],
    config: FilterConfig | None = None,
    *,
    latch: dict,
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
    latch:
        The caller's per-episode filter memory: pass the same empty dict on
        every step of one episode. Each pair keeps the sideways escape frame
        and escape signs it had when its conflict started, so observation
        noise cannot flip them. Required (UTM-077): a fresh dict on every
        step does not give the latched behaviour.

    Returns
    -------
    safe_actions:
        List[SafeAction] with one entry per active agent. On solver error,
        agents in the conflict cluster receive the evasive manoeuvre
        (fallback_used=True); all others keep their nominal action.
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
    TypeError
        When ``latch`` is not a dict.
    """
    if not _warmed_up:
        _warm_up()  # first-call costs (Numba kernels, solvers), before the step's clock starts
    step_start = time.perf_counter()
    if not isinstance(latch, dict):
        raise TypeError(f"latch must be the caller's per-episode dict (UTM-077), got {type(latch).__name__}")
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
    # Pairs within the configured gate (all when None) go to _update_latches,
    # as before the derived gate; the rollout and the QP take only those within
    # the derived gate as well, whose rows can bind.
    pairs_in_range: list[tuple[str, str]] = []
    gate_sq = _exact_range_gate(config) ** 2

    for idx, id_i in enumerate(agent_ids):
        for id_j in agent_ids[idx + 1:]:
            n_pairs_total += 1
            h = _barrier(state_map[id_i], state_map[id_j])

            if h_min is None or h < h_min:
                h_min = h

            # Range gate: skip pairs too far apart.
            sep_sq = _separation_sq(state_map[id_i], state_map[id_j])
            if config.range_gate_m is not None and math.sqrt(sep_sq) > config.range_gate_m:
                continue
            pairs_in_range.append((id_i, id_j))
            if sep_sq > gate_sq:
                continue

            pairs_in_qp.append((id_i, id_j))

    diag.h_min_m2 = h_min
    diag.n_pairs_total = n_pairs_total
    diag.n_pairs_in_qp = len(pairs_in_qp)
    conflicts = _world_limit_conflicts(
        np.array([state_map[aid].position_m for aid in agent_ids], dtype=np.float64),
        np.array([state_map[aid].velocity_mps for aid in agent_ids], dtype=np.float64),
        config,
    )
    diag.world_limit_conflict_agents = [aid for aid, c in zip(agent_ids, conflicts, strict=True) if c]
    if diag.world_limit_conflict_agents:
        logger.warning(
            "filter: world-box limits crossed at t=%d ms for %s; kept the nearer wall's limit",
            step_ts, diag.world_limit_conflict_agents,
        )

    frames = _update_latches(state_map, pairs_in_range, config, latch)

    # --- QP solve ---
    backup = None
    deadline = None
    # Conflict clusters whose certified solve missed its deadline: (agents, pairs).
    missed: list[tuple[list[str], list[tuple[str, str]]]] = []
    reuse: dict = {}
    try:
        if config.barrier == "backup" and pairs_in_qp:
            backup = _backup_constraints(state_map, pairs_in_qp, config, frames)
            diag.backup_rollout_steps = [int(k) for k in backup[2]]
            diag.backup_rollout_cap_pairs = [pair for pair, c in zip(pairs_in_qp, backup[3], strict=True) if c]
            if diag.backup_rollout_cap_pairs:
                logger.warning(
                    "filter: backup rollout still closing at the cap at t=%d ms on %d pairs",
                    step_ts, len(diag.backup_rollout_cap_pairs),
                )
        deadline = _solve_deadline(step_start) if config.step_deadline else None
        safe_accels, pair_slack, qp_feasible = _solve_hocbf_qp(
            state_map, action_map, pairs_in_qp, config, backup=backup, frames=frames,
            deadline=deadline, missed=missed, reuse=reuse,
        )
    except Exception as exc:  # noqa: BLE001
        logger.error(
            "filter: QP solver error at t=%d ms, applying fallback: %s",
            step_ts, exc,
        )
        qp_feasible = False
        safe_accels, pair_slack = {}, {}

    # Closing pairs that cannot keep the barrier: re-solve with the escape
    # constraint, minimising the predicted violation in metres.
    # Backup rows can leave a closing pair between two drones that are
    # already escaping without slack (the outer pair of a sandwich); colour
    # it too, so the signs of its drones stay consistent.
    in_slack = {aid for pair in pair_slack for aid in pair}
    escape_pairs = []
    for pair in pairs_in_qp if config.barrier == "backup" else pair_slack:
        if pair not in pair_slack and not (pair[0] in in_slack and pair[1] in in_slack):
            continue
        _, v, n, _ = _relative(state_map[pair[0]], state_map[pair[1]])
        if float(v @ n) < 0.0:
            escape_pairs.append(pair)
    # A failed escape solve keeps the first solve, which is valid; so does an
    # escape problem that missed its deadline (its agents keep the first solve).
    missed_agents = {aid for agents, _ in missed for aid in agents}
    if qp_feasible and escape_pairs:
        escape_missed: list[tuple[list[str], list[tuple[str, str]]]] = []
        try:
            escaped, _, escape_ok = _solve_hocbf_qp(
                state_map, action_map, pairs_in_qp, config, _escape_signs(state_map, escape_pairs, frames), backup,
                frames, deadline=deadline, missed=escape_missed, reuse=reuse, skip=missed_agents,
            )
        except Exception as exc:  # noqa: BLE001
            logger.error("filter: escape solve error at t=%d ms: %s", step_ts, exc)
            escape_ok = False
        if escape_ok:
            safe_accels.update({aid: a for aid, a in escaped.items() if aid not in missed_agents})
            late = {aid for agents, _ in escape_missed for aid in agents}
            diag.escape_failed_slack_pairs = sorted(pair for pair in pair_slack if late.intersection(pair))
            if late:
                logger.warning(
                    "filter: escape solve missed its deadline at t=%d ms for %d agents, keeping their first solve",
                    step_ts, len(late),
                )
        else:
            diag.escape_failed_slack_pairs = sorted(pair_slack)
            logger.warning(
                "filter: escape solve failed at t=%d ms, keeping the first solve with slack on %d pairs",
                step_ts, len(pair_slack),
            )

    diag.qp_feasible = qp_feasible
    diag.pair_slack_mps2 = pair_slack
    d_eff = SAFE_SEPARATION_M + config.margin_m
    for idx, id_i in enumerate(agent_ids):
        for id_j in agent_ids[idx + 1:]:
            dist = math.sqrt(_separation_sq(state_map[id_i], state_map[id_j]))
            if dist < d_eff:
                diag.pair_violation_depth_m[(id_i, id_j)] = d_eff - dist

    # Solver error: evasive fallback for the conflict cluster only; every
    # other agent keeps its nominal, which satisfies all its pair constraints,
    # clipped to its world-box limits (a drone leaving the cluster can still
    # be heading out of the world).
    evasive: dict[str, tuple[float, float, float]] = {}
    if not qp_feasible:
        evasive = _fallback_cluster(state_map, action_map, pairs_in_qp, config, frames)
        safe_accels = {}
        for aid in agent_ids:
            state = state_map[aid]
            limits = _world_accel_limits(
                np.array(state.position_m, dtype=np.float64), np.array(state.velocity_mps, dtype=np.float64), config
            )
            nominal = np.clip(action_map[aid].proposed_accel_mps2, *limits)
            safe_accels[aid] = evasive.get(aid, tuple(float(c) for c in nominal))
    # Deadline missed: the cluster's drones fly their backup manoeuvre (the
    # evasive manoeuvre of the backup rows, away from the most critical
    # neighbour in the cluster), as a backup-CBF filter does when its QP is not
    # available; other clusters keep their solutions.
    for agents, pairs in missed:
        for aid in agents:
            evasive[aid] = safe_accels[aid] = _evasive_accel(aid, state_map, pairs, config, frames)
    diag.deadline_fallback_agents = sorted(missed_agents)
    if missed_agents:
        logger.warning(
            "filter: certified solve missed its deadline at t=%d ms for %d clusters (%d agents), backup applied",
            step_ts, len(missed), len(missed_agents),
        )
    diag.fallback_used = bool(evasive)

    # --- Build SafeAction list ---
    safe_actions: list[SafeAction] = []
    capped_ids = {aid for pair in diag.backup_rollout_cap_pairs for aid in pair}
    escape_failed_ids = {aid for pair in diag.escape_failed_slack_pairs for aid in pair}
    deadline_ids = set(diag.deadline_fallback_agents)

    for aid in agent_ids:
        nom = action_map[aid].proposed_accel_mps2
        accel = safe_accels[aid]
        in_fallback = aid in evasive

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
                qp_feasible=not in_fallback,
                fallback_used=in_fallback,
                backup_rollout_capped=aid in capped_ids,
                world_limit_conflict=aid in diag.world_limit_conflict_agents,
                escape_failed_slack=aid in escape_failed_ids,
                deadline_fallback=aid in deadline_ids,
            )
        )

    if not qp_feasible:
        logger.warning(
            "filter: QP solver error at t=%d ms, evasive fallback applied to %d of %d agents "
            "(h_min=%.3f m²)",
            step_ts,
            len(evasive),
            len(safe_actions),
            diag.h_min_m2 if diag.h_min_m2 is not None else float("nan"),
        )
    elif pair_slack:
        logger.warning(
            "filter: barrier violated at t=%d ms on %d pairs (%d escaping)",
            step_ts, len(pair_slack), len(escape_pairs),
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
