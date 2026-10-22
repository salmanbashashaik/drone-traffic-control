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
from collections.abc import Sequence
from dataclasses import dataclass, field

import numpy as np
import osqp
import scipy.sparse as sp
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

# Default range gate: None means include all pairs (correct for small N).
# Set to a positive float (metres) to exclude distant pairs from the QP.
_DEFAULT_RANGE_GATE_M: float | None = None


@dataclass(frozen=True)
class FilterConfig:
    """Runtime configuration for the HOCBF filter.

    Attributes
    ----------
    gamma:
        Linear class-K gain on the braking barrier (1/s). Larger values let
        the pair approach the braking curve faster. See formulation §6.
    margin_m:
        Separation buffer added to d_safe inside the barrier. Absorbs the
        100 ms sample-and-hold error and small actuator lag (PR #5 review).
        Reported h_min_m2 still uses d_safe.
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
        from the QP constraint set. None = include all pairs.
    solver_eps:
        Absolute and relative tolerance for the OSQP solver.
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
    """

    gamma: float = 1.0
    margin_m: float = 0.5
    tie_break_mps2: float = 1.0
    lag_s: float = 0.0
    range_gate_m: float | None = _DEFAULT_RANGE_GATE_M
    solver_eps: float = 1e-5
    solver_max_iter: int = 4000
    slack_weight: float = 1e4
    barrier_offset_mps: float = 1.0
    barrier: str = "backup"

    def __post_init__(self) -> None:
        if not self.gamma > 0.0:
            raise ValueError(f"gamma must be positive, got {self.gamma}")
        if not self.margin_m >= 0.0:
            raise ValueError(f"margin_m must be non-negative, got {self.margin_m}")
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
    """

    h_min_m2: float | None = None
    n_pairs_total: int = 0
    n_pairs_in_qp: int = 0
    qp_feasible: bool = True
    fallback_used: bool = False
    interventions: int = 0
    pair_slack_mps2: dict[tuple[str, str], float] = field(default_factory=dict)
    pair_violation_depth_m: dict[tuple[str, str], float] = field(default_factory=dict)


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


def _lateral(n: np.ndarray) -> np.ndarray:
    """Unit sideways direction perpendicular to unit vector(s) n.

    Away from vertical it is z x n (a right turn about +z). Near vertical,
    z x n is ill-conditioned: a tiny horizontal offset sets its direction, so
    it swings from step to step (PR #12 review B2). There it is the pair's
    horizontal offset, projected perpendicular to n: moving along it does not
    rotate it, so it stays fixed while the conflict lasts. An exactly vertical
    n uses +x (-x for downward n). The two are orthogonal and blend between
    10 and 20 degrees from vertical, so the result never vanishes or jumps.
    _lateral(-n) = -_lateral(n), so the two drones of a pair move apart.
    Works on one (3,) vector or a (M, 3) stack.
    """
    n = np.asarray(n, dtype=np.float64)
    horizontal = np.linalg.norm(n[..., :2], axis=-1, keepdims=True)
    turn = np.cross((0.0, 0.0, 1.0), n) / np.maximum(horizontal, 1e-12)
    offset = n * (1.0, 1.0, 0.0)
    plumb = np.zeros_like(n)
    plumb[..., 0] = np.where(n[..., 2] >= 0.0, 1.0, -1.0)
    offset = np.where(horizontal < 1e-9, plumb, offset)
    offset = offset - np.sum(offset * n, axis=-1, keepdims=True) * n
    offset /= np.maximum(np.linalg.norm(offset, axis=-1, keepdims=True), 1e-12)
    lo, hi = _NEAR_VERTICAL_SIN
    weight = np.clip((horizontal - lo) / (hi - lo), 0.0, 1.0)
    lateral = weight * turn + (1.0 - weight) * offset
    return lateral / np.linalg.norm(lateral, axis=-1, keepdims=True)


def _escape_axes(n: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Sideways L and vertical V unit vectors perpendicular to n.

    L is ``_lateral(n)``; V = n x L completes the frame (z minus its n
    component for horizontal n). Each is sign-normalised so its largest
    component is positive, which gives pairs on opposite sides of a drone the
    same axes and does not flip under small drifts.
    """
    lateral = _lateral(n)
    vertical = np.cross(n, lateral)
    axes = []
    for axis in (lateral, vertical):
        axis = axis / np.linalg.norm(axis)
        axes.append(axis if axis[int(np.argmax(np.abs(axis)))] > 0.0 else -axis)
    return axes[0], axes[1]


def _escape_signs(
    state_map: dict[str, DroneState], escape_pairs: list[tuple[str, str]]
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
    """
    by_distance = sorted(
        escape_pairs, key=lambda p: (_separation_sq(state_map[p[0]], state_map[p[1]]), p)
    )
    lateral = _two_colour(by_distance)
    vertical = _two_colour(sorted(by_distance, key=lambda p: lateral[p[0]] != lateral[p[1]]))

    signs = {}
    for id_i, id_j in escape_pairs:
        r, v, n, t_star = _relative(state_map[id_i], state_map[id_j])
        miss = r + v * t_star
        miss -= float(miss @ n) * n
        signs[(id_i, id_j)] = tuple(
            math.copysign(1.0, float(miss @ axis)) if abs(float(miss @ axis)) > _MISS_CLEAR_M
            else (colour[id_i] if colour[id_i] != colour[id_j] else 0.0)
            for axis, colour in zip(_escape_axes(n), (lateral, vertical), strict=True)
        )
    return signs


def _two_colour(edges: list[tuple[str, str]]) -> dict[str, float]:
    """Greedy +-1 colouring that gives each edge, in order, opposite colours
    when the edges before it allow; the lowest id of each component is +1."""
    colour: dict[str, float] = {}
    component: dict[str, int] = {}
    for id_i, id_j in edges:
        if id_i not in colour and id_j not in colour:
            colour[id_i], colour[id_j] = 1.0, -1.0
            component[id_i] = component[id_j] = len(component)
        elif id_j not in colour:
            colour[id_j], component[id_j] = -colour[id_i], component[id_i]
        elif id_i not in colour:
            colour[id_i], component[id_i] = -colour[id_j], component[id_j]
        elif component[id_i] != component[id_j]:
            old, flip = component[id_j], colour[id_i] == colour[id_j]
            for aid in [a for a, c in component.items() if c == old]:
                component[aid] = component[id_i]
                colour[aid] = -colour[aid] if flip else colour[aid]
    for comp in set(component.values()):
        lowest = min(a for a, c in component.items() if c == comp)
        if colour[lowest] < 0.0:
            for aid in [a for a, c in component.items() if c == comp]:
                colour[aid] = -colour[aid]
    return colour


def _escape_constraint(
    s_i: DroneState, s_j: DroneState, sign: tuple[float, float], config: FilterConfig
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
    lateral, vertical = _escape_axes(n)
    m = n + sign[0] * lateral + sign[1] * vertical
    m /= np.linalg.norm(m)
    d_eff = SAFE_SEPARATION_M + config.margin_m
    return 0.5 * t_star**2 * m, d_eff - float(m @ (r + v * t_star))


# Extra backup horizon past the derived stopping time, so the closest
# approach is not cut off at the horizon end by the 100 ms hold and the
# turn of the line of sight during the manoeuvre.
_BACKUP_HORIZON_MARGIN_S = 0.5


def _backup_steps(config: FilterConfig) -> int:
    """Backup horizon in simulator steps, derived from the speed and accel limits.

    The evasive manoeuvre brakes each drone by at least u_max / sqrt(2) along
    the line of sight (``_evasive_direction``: its x and y components are at
    most sqrt(2), its n component at least 1). A pair closing at 2 v_max
    therefore stops closing within sqrt(2) v_max / u_max (6.06 s at 15 m/s
    and 3.5 m/s²). Add the actuator lag and _BACKUP_HORIZON_MARGIN_S.
    """
    stop_s = math.sqrt(2.0) * MAX_SPEED_MPS / MAX_ACCEL_COMPONENT_MPS2
    return math.ceil((stop_s + config.lag_s + _BACKUP_HORIZON_MARGIN_S) / (SIMULATION_STEP_MS / 1000.0))


_WORLD_MIN_M = np.array((WORLD_XY_MIN_M, WORLD_XY_MIN_M, WORLD_Z_MIN_M))
_WORLD_MAX_M = np.array((WORLD_XY_MAX_M, WORLD_XY_MAX_M, WORLD_Z_MAX_M))


def _world_accel_limits(
    pos: np.ndarray, vel: np.ndarray, config: FilterConfig
) -> tuple[np.ndarray, np.ndarray]:
    """Per-component acceleration limits (low, high) that keep a drone in the world box.

    One braking barrier per axis and wall, like the pair barrier but for one
    drone braking at u_max: h = B(s) - w, with s the distance to the wall
    less margin_m, w the speed towards it and B(s) = sqrt(2 u_max s + c^2) - c
    (linear inside the margin). dh/dt >= -gamma h asks for at least
    -gamma h + B'(s) w of acceleration away from the wall. Clipped to the
    box, so the limits never make the QP infeasible: low <= high because the
    floor and ceiling (and the x and y walls) are far apart. Works on (3,)
    or (N, 3) arrays.
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


def _evasive_direction(
    n: np.ndarray, up: np.ndarray | float, limits: tuple[np.ndarray, np.ndarray] | None = None
) -> np.ndarray:
    """Evasive acceleration for the agent at the head of unit vector(s) n.

    Sums n (away from the neighbour), the sideways unit vector ``_lateral(n)``
    and ``up`` in z, scaled so the largest component is u_max. With
    ``limits`` (``_world_accel_limits`` of the drone) each component is then
    clipped to them, so near a wall the drone brakes or turns away from it
    instead of towards it (PR #12 review B1). Works on one (3,) vector or a
    (M, 3) stack.

    known limit: a clipped component can reduce the braking along n below
    u_max / sqrt(2), the bound ``_backup_steps`` rests on; only near a wall.
    """
    direction = n + _lateral(n)
    direction[..., 2] += up
    direction = direction * (MAX_ACCEL_COMPONENT_MPS2 / np.max(np.abs(direction), axis=-1, keepdims=True))
    return direction if limits is None else np.clip(direction, *limits)


def _backup_constraints(
    states: dict[str, DroneState], pairs: list[tuple[str, str]], config: FilterConfig
) -> tuple[np.ndarray, np.ndarray]:
    """Backup-set barrier rows coef . (a_i - a_j) >= rhs, one per pair.

    Both drones of a pair fly the evasive manoeuvre against each other from
    the current state for ``_backup_steps`` steps of 100 ms zero-order-hold steps
    with the simulator's trapezoid position update and speed cap, each
    clipped to its own world-box limits: for an isolated pair this is what
    the fallback flies. h = min |r| - d_eff over
    that trajectory is the pair's margin, reached at step k*. The row asks
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
    d_eff. Returns (m (M, 3), rhs (M,)).

    known limit: pairwise backup (a drone in several conflicts gets one
    evasive per pair, not one manoeuvre: TASKS.md BL-03); only k* is
    constrained (BL-04); the sensitivity ignores the backup's feedback and
    the speed cap; lag_s only lengthens the horizon. Re-planned every step.
    All must be resolved before any real flight.
    """
    dt = SIMULATION_STEP_MS / 1000.0
    steps = _backup_steps(config)
    d_eff = SAFE_SEPARATION_M + config.margin_m

    def column(attr: str, k: int) -> np.ndarray:
        return np.array([getattr(states[pair[k]], attr) for pair in pairs], dtype=np.float64)

    def cap(v: np.ndarray) -> np.ndarray:
        speed = np.linalg.norm(v, axis=1, keepdims=True)
        return v * np.minimum(1.0, MAX_SPEED_MPS / np.maximum(speed, 1e-9))

    p_i, p_j = column("position_m", 0), column("position_m", 1)
    v_i, v_j = column("velocity_mps", 0), column("velocity_mps", 1)
    traj = np.empty((steps, len(pairs), 3))
    for k in range(steps):
        r = p_i - p_j
        n = r / np.maximum(np.linalg.norm(r, axis=1, keepdims=True), 1e-9)
        # Pairs are (lower id, higher id); on equal altitude the lower id climbs.
        up = np.where(np.abs(n[:, 2]) > 1e-3, np.sign(n[:, 2]), 1.0)
        a_i = _evasive_direction(n, up, _world_accel_limits(p_i, v_i, config))
        a_j = _evasive_direction(-n, -up, _world_accel_limits(p_j, v_j, config))
        if k == 0:
            a_first = a_i - a_j
        v_i_next, v_j_next = cap(v_i + a_i * dt), cap(v_j + a_j * dt)
        p_i = p_i + 0.5 * (v_i + v_i_next) * dt
        p_j = p_j + 0.5 * (v_j + v_j_next) * dt
        v_i, v_j = v_i_next, v_j_next
        traj[k] = p_i - p_j

    dist = np.linalg.norm(traj, axis=2)
    k_star = np.argmin(dist, axis=0)
    cols = np.arange(len(pairs))
    d_star = dist[k_star, cols]
    m = traj[k_star, cols] / np.maximum(d_star, 1e-9)[:, None]
    sensitivity = (k_star + 0.5) * dt * dt
    rhs = np.einsum("ij,ij->i", m, a_first) - config.gamma * dt * (d_star - d_eff) / sensitivity
    return m, rhs


def build_hocbf_qp_matrices(
    states: dict[str, DroneState],
    actions: dict[str, MARLAction],
    pairs_in_qp: list[tuple[str, str]],
    config: FilterConfig,
    escape_signs: dict[tuple[str, str], tuple[float, float]] | None = None,
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
    uses ``_backup_constraints`` instead, with no tie-break.

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
    P = sp.block_diag(
        [sp.eye(dim) * 2.0, sp.diags(np.where(is_escape, 2.0 * _ESCAPE_WEIGHT, 0.0))], format="csc"
    )
    nom_stacked = np.zeros(dim, dtype=np.float64)
    for aid, idx in agent_idx.items():
        nom_stacked[3 * idx : 3 * idx + 3] = actions[aid].proposed_accel_mps2
    target = nom_stacked.copy()

    # Box bounds on control accelerations: -u_max <= a_{i,k} <= u_max,
    # tightened per drone by the world-box barriers (hard: no slack).
    A_box = sp.hstack([sp.eye(dim), sp.csc_matrix((dim, n_pairs))])
    l_box, u_box = (
        limit.ravel()
        for limit in _world_accel_limits(
            np.array([states[aid].position_m for aid in agent_ids], dtype=np.float64),
            np.array([states[aid].velocity_mps for aid in agent_ids], dtype=np.float64),
            config,
        )
    )

    # Pairwise braking-barrier constraints (P0-SA-01 Section 6-8):
    # n_ij^T (a_i - a_j) >= rhs
    cbf_rows = []
    l_cbf = []
    u_cbf = []

    backup = _backup_constraints(states, pairs_in_qp, config) if config.barrier == "backup" else None
    for k, (id_i, id_j) in enumerate(pairs_in_qp):
        idx_i = agent_idx[id_i]
        idx_j = agent_idx[id_j]
        row = np.zeros(dim, dtype=np.float64)
        if backup is not None or (id_i, id_j) in escape_signs:
            if (id_i, id_j) in escape_signs:
                coef, rhs = _escape_constraint(states[id_i], states[id_j], escape_signs[(id_i, id_j)], config)
            else:
                coef, rhs = backup[0][k], float(backup[1][k])
            row[3 * idx_i : 3 * idx_i + 3] = coef
            row[3 * idx_j : 3 * idx_j + 3] = -coef
            cbf_rows.append(row)
            l_cbf.append(rhs)
            u_cbf.append(np.inf)
            continue

        n_ij, rhs, _ = _pair_constraint(states[id_i], states[id_j], config)
        row[3 * idx_i : 3 * idx_i + 3] = n_ij
        row[3 * idx_j : 3 * idx_j + 3] = -n_ij

        # Symmetric pairs stall at d_eff with no lateral escape. When the
        # nominal would violate the constraint, bias both agents apart
        # sideways (``_lateral``: right turn about +z away from vertical).
        if config.tie_break_mps2 > 0.0 and float(row @ nom_stacked) < rhs:
            lateral = config.tie_break_mps2 * _lateral(n_ij)
            target[3 * idx_i : 3 * idx_i + 3] += lateral
            target[3 * idx_j : 3 * idx_j + 3] -= lateral

        cbf_rows.append(row)
        l_cbf.append(rhs)
        u_cbf.append(np.inf)

    q = np.concatenate([-2.0 * target, np.where(is_escape, 0.0, config.slack_weight)])

    if cbf_rows:
        A_cbf = sp.hstack([sp.csc_matrix(np.vstack(cbf_rows)), sp.eye(n_pairs)])
        A_slack = sp.hstack([sp.csc_matrix((n_pairs, dim)), sp.eye(n_pairs)])
        A = sp.vstack([A_cbf, A_box, A_slack], format="csc")
        l = np.concatenate([np.array(l_cbf, dtype=np.float64), l_box, np.zeros(n_pairs)])
        u = np.concatenate([np.array(u_cbf, dtype=np.float64), u_box, np.full(n_pairs, np.inf)])
    else:
        A = sp.csc_matrix(A_box)
        l = l_box
        u = u_box

    return P, q, A, l, u, agent_ids


def _solve_hocbf_qp(
    states: dict[str, DroneState],
    actions: dict[str, MARLAction],
    pairs_in_qp: list[tuple[str, str]],
    config: FilterConfig,
    escape_signs: dict[tuple[str, str], tuple[float, float]] | None = None,
) -> tuple[dict[str, tuple[float, float, float]], dict[tuple[str, str], float], bool]:
    """Solve the joint HOCBF QP over all active agents using OSQP.

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
        # No active pairs to constrain; nominal actions are already valid
        safe_accels = {
            aid: actions[aid].proposed_accel_mps2
            for aid in actions
        }
        return safe_accels, {}, True

    try:
        P, q, A, l, u, agent_ids = build_hocbf_qp_matrices(
            states, actions, pairs_in_qp, config, escape_signs
        )
        # On non-convergence retry once without over-relaxation (alpha 1.0
        # instead of OSQP's 1.6): fixes HARD fuzz seeds 864 and 977, where
        # rho alone (up to 10) and 20000 iterations did not (PR #12 review).
        for alpha in (1.6, 1.0):
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
                # The 1e4 slack penalty slows ADMM convergence; without polishing,
                # pass-through steps return accelerations ~0.05 m/s² off nominal.
                polish=True,
                # The escape QP saturates the box with a degenerate active set;
                # at the default rho (0.1) ADMM needs > 4000 iterations there.
                rho=10.0 if escape_signs else 0.1,
                alpha=alpha,
            )
            res = prob.solve()
            if res.info.status == "solved":
                break
            logger.warning("filter: OSQP status %s at alpha=%.1f", res.info.status, alpha)

        if res.info.status == "solved":
            u_max = MAX_ACCEL_COMPONENT_MPS2
            safe_accels = {}
            for idx, aid in enumerate(agent_ids):
                ax = float(np.clip(res.x[3 * idx], -u_max, u_max))
                ay = float(np.clip(res.x[3 * idx + 1], -u_max, u_max))
                az = float(np.clip(res.x[3 * idx + 2], -u_max, u_max))
                safe_accels[aid] = (ax, ay, az)
            slacks = res.x[3 * len(agent_ids):]
            pair_slack = {
                pair: float(s)
                for pair, s in zip(pairs_in_qp, slacks, strict=True)
                if s > SHIELD_INTERVENTION_TOLERANCE_MPS2
            }
            return safe_accels, pair_slack, True
        else:
            logger.warning("filter: OSQP non-optimal status: %s", res.info.status)
            return {}, {}, False
    except Exception as exc:  # noqa: BLE001
        logger.error("filter: OSQP solve exception: %s", exc)
        return {}, {}, False


def _evasive_accel(
    aid: str,
    state_map: dict[str, DroneState],
    pairs_in_qp: list[tuple[str, str]],
    config: FilterConfig,
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
    if abs(n[2]) > 1e-3:
        up = math.copysign(1.0, n[2])
    else:
        up = 1.0 if aid < other else -1.0
    state = state_map[aid]
    limits = _world_accel_limits(
        np.array(state.position_m, dtype=np.float64), np.array(state.velocity_mps, dtype=np.float64), config
    )
    direction = _evasive_direction(n, up, limits)
    return tuple(float(c) for c in np.clip(direction, -u_max, u_max))


def _fallback_cluster(
    state_map: dict[str, DroneState],
    action_map: dict[str, MARLAction],
    pairs_in_qp: list[tuple[str, str]],
    config: FilterConfig,
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
            evasive[aid] = _evasive_accel(aid, state_map, pairs_in_qp, config)


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
        safe_accels, pair_slack, qp_feasible = _solve_hocbf_qp(
            state_map, action_map, pairs_in_qp, config
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
    # A failed escape solve keeps the first solve, which is valid.
    if qp_feasible and escape_pairs:
        try:
            escaped, _, escape_ok = _solve_hocbf_qp(
                state_map, action_map, pairs_in_qp, config, _escape_signs(state_map, escape_pairs)
            )
        except Exception as exc:  # noqa: BLE001
            logger.error("filter: escape solve error at t=%d ms: %s", step_ts, exc)
            escape_ok = False
        if escape_ok:
            safe_accels = escaped
        else:
            logger.warning("filter: escape solve failed at t=%d ms, keeping the first solve", step_ts)

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
        evasive = _fallback_cluster(state_map, action_map, pairs_in_qp, config)
        safe_accels = {}
        for aid in agent_ids:
            state = state_map[aid]
            limits = _world_accel_limits(
                np.array(state.position_m, dtype=np.float64), np.array(state.velocity_mps, dtype=np.float64), config
            )
            nominal = np.clip(action_map[aid].proposed_accel_mps2, *limits)
            safe_accels[aid] = evasive.get(aid, tuple(float(c) for c in nominal))
    diag.fallback_used = bool(evasive)

    # --- Build SafeAction list ---
    safe_actions: list[SafeAction] = []

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
