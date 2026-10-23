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
    for pair in pairs:
        r, v, n, t_star = _relative(state_map[pair[0]], state_map[pair[1]])
        if min(float(np.linalg.norm(r)), float(np.linalg.norm(r + v * t_star))) < limit:
            if pair not in latch:
                entering.append(pair)
        elif float(r @ v) >= 0.0:
            latch.pop(pair, None)
    for pair in set(latch) - set(pairs):
        del latch[pair]
    if entering:
        # Colour the whole conflict set, so a drone squeezed between two
        # neighbours gets opposite latches towards them.
        conflict = sorted(set(latch) | set(entering))
        miss_only = _escape_signs(state_map, conflict, colouring=False)
        cluster = {aid: {aid} for pair in conflict for aid in pair}
        for id_i, id_j in conflict:
            if cluster[id_i] is not cluster[id_j]:
                merged = cluster[id_i] | cluster[id_j]
                for aid in merged:
                    cluster[aid] = merged
        coloured = [pair for pair in conflict if len(cluster[pair[0]]) >= 3]
        seed = {}
        for pair in coloured:
            frame = latch.get(pair)
            if frame is not None and frame.origin != "none":
                n = _relative(state_map[pair[0]], state_map[pair[1]])[2]
                along = float(_escape_axes(n)[0] @ _frame_lateral(n, frame.axis))
                if abs(along) > 0.5:
                    seed[pair] = (math.copysign(1.0, along), 1 if frame.origin == "turn_right" else 2)
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
    dt = SIMULATION_STEP_MS / 1000.0
    min_steps = _backup_steps(config)
    steps = max(min_steps, _backup_max_steps(config))
    used = np.full(len(p_i), steps)
    still = np.ones(len(p_i), dtype=bool)
    closing = np.zeros(len(p_i))

    def cap(v: np.ndarray) -> np.ndarray:
        speed = np.linalg.norm(v, axis=1, keepdims=True)
        return v * np.minimum(1.0, MAX_SPEED_MPS / np.maximum(speed, 1e-9))

    traj = np.full((steps, len(p_i), 3), np.nan)
    for k in range(steps):
        r = p_i - p_j
        n = r / np.maximum(np.linalg.norm(r, axis=1, keepdims=True), 1e-9)
        # Pairs are (lower id, higher id); on equal altitude the lower id climbs.
        up = np.where(np.abs(n[:, 2]) > 1e-3, np.sign(n[:, 2]), 1.0)
        if up_latched is not None:
            up = np.where(up_latched != 0.0, up_latched, up)
        a_i = _evasive_direction(n, up, _world_accel_limits(p_i, v_i, config), axis)
        a_j = _evasive_direction(-n, -up, _world_accel_limits(p_j, v_j, config), axis)
        if k == 0:
            a_first = a_i - a_j
        v_i_next, v_j_next = cap(v_i + a_i * dt), cap(v_j + a_j * dt)
        p_i = p_i + 0.5 * (v_i + v_i_next) * dt
        p_j = p_j + 0.5 * (v_j + v_j_next) * dt
        v_i, v_j = v_i_next, v_j_next
        r = p_i - p_j
        traj[k, still] = r[still]
        rate = -np.einsum("ij,ij->i", r, v_i - v_j) / np.maximum(np.linalg.norm(r, axis=1), 1e-9)
        closing[still] = rate[still]
        if k + 1 >= min_steps:
            done = still & (closing <= 0.0)
            used[done] = k + 1
            still &= ~done
            if not still.any():
                break
    return traj[: used.max(initial=0)], a_first, closing, used, still


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
    d_eff. Returns (m (M, 3), rhs (M,), steps flown (M,), capped (M,)).

    known limit: pairwise backup (a drone in several conflicts gets one
    evasive per pair, not one manoeuvre: TASKS.md BL-03); only k* is
    constrained (BL-04); the sensitivity ignores the backup's feedback and
    the speed cap; lag_s only lengthens the horizon. Re-planned every step.
    All must be resolved before any real flight.
    """
    dt = SIMULATION_STEP_MS / 1000.0
    d_eff = SAFE_SEPARATION_M + config.margin_m

    def column(attr: str, k: int) -> np.ndarray:
        return np.array([getattr(states[pair[k]], attr) for pair in pairs], dtype=np.float64)

    traj, a_first, _, used, capped = _backup_rollout(
        column("position_m", 0), column("position_m", 1),
        column("velocity_mps", 0), column("velocity_mps", 1), config,
        np.array([_latch_axis(frames, pair) for pair in pairs]).reshape(-1, 3),
        np.array([(frames or {})[pair].up if pair in (frames or {}) else 0.0 for pair in pairs]),
    )
    dist = np.nan_to_num(np.linalg.norm(traj, axis=2), nan=np.inf)
    k_star = np.argmin(dist, axis=0)
    cols = np.arange(len(pairs))
    d_star = dist[k_star, cols]
    m = traj[k_star, cols] / np.maximum(d_star, 1e-9)[:, None]
    sensitivity = (k_star + 0.5) * dt * dt
    rhs = np.einsum("ij,ij->i", m, a_first) - config.gamma * dt * (d_star - d_eff) / sensitivity
    return m, rhs, used, capped


def build_hocbf_qp_matrices(
    states: dict[str, DroneState],
    actions: dict[str, MARLAction],
    pairs_in_qp: list[tuple[str, str]],
    config: FilterConfig,
    escape_signs: dict[tuple[str, str], tuple[float, float]] | None = None,
    backup: tuple[np.ndarray, ...] | None = None,
    frames: dict[tuple[str, str], _PairLatch] | None = None,
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

    if config.barrier != "backup":
        backup = None
    elif backup is None:
        backup = _backup_constraints(states, pairs_in_qp, config, frames)
    for k, (id_i, id_j) in enumerate(pairs_in_qp):
        idx_i = agent_idx[id_i]
        idx_j = agent_idx[id_j]
        row = np.zeros(dim, dtype=np.float64)
        if backup is not None or (id_i, id_j) in escape_signs:
            if (id_i, id_j) in escape_signs:
                coef, rhs = _escape_constraint(
                    states[id_i], states[id_j], escape_signs[(id_i, id_j)], config, (frames or {}).get((id_i, id_j))
                )
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
            lateral = config.tie_break_mps2 * _frame_lateral(n_ij, _latch_axis(frames, (id_i, id_j)))
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
    backup: tuple[np.ndarray, ...] | None = None,
    frames: dict[tuple[str, str], _PairLatch] | None = None,
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
        # No active pairs to constrain; the nominal only needs the world-box
        # limits the QP would apply (PR #12 re-review: a lone drone left the box).
        safe_accels = {}
        for aid in actions:
            state = states[aid]
            limits = _world_accel_limits(
                np.array(state.position_m, dtype=np.float64), np.array(state.velocity_mps, dtype=np.float64), config
            )
            safe_accels[aid] = tuple(float(c) for c in np.clip(actions[aid].proposed_accel_mps2, *limits))
        return safe_accels, {}, True

    try:
        P, q, A, l, u, agent_ids = build_hocbf_qp_matrices(
            states, actions, pairs_in_qp, config, escape_signs, backup, frames
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

    frames = _update_latches(state_map, pairs_in_qp, config, latch)

    # --- QP solve ---
    backup = None
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
        safe_accels, pair_slack, qp_feasible = _solve_hocbf_qp(
            state_map, action_map, pairs_in_qp, config, backup=backup, frames=frames
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
                state_map, action_map, pairs_in_qp, config, _escape_signs(state_map, escape_pairs, frames), backup,
                frames,
            )
        except Exception as exc:  # noqa: BLE001
            logger.error("filter: escape solve error at t=%d ms: %s", step_ts, exc)
            escape_ok = False
        if escape_ok:
            safe_accels = escaped
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
    diag.fallback_used = bool(evasive)

    # --- Build SafeAction list ---
    safe_actions: list[SafeAction] = []
    capped_ids = {aid for pair in diag.backup_rollout_cap_pairs for aid in pair}
    escape_failed_ids = {aid for pair in diag.escape_failed_slack_pairs for aid in pair}

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
