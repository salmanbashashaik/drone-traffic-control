"""Backup rollout with the actuator lag, drag and thrust authority (P3-SA-03 §3.5, §3.9; review B3, B4).

Main's backup rollout (``filter._rollout_kernel``) flies the evasive
manoeuvre in the contract's drag-free model: the acceleration it chooses is
the acceleration the drone gets, at once. On the drag plant the station sends
the thrust command c = u - D (w_hat - v), clipped to the airframe (horizontal
disc of radius a_phys, |c_z| <= a_phys), and the actuator follows c through
a first-order lag tau_a. This module flies the same manoeuvre (main's
``_limits_kernel`` and ``_evasive_kernel``, the same latch handling and stop
rule) through that chain:

    p' = v,   v' = a + D (w_hat - v),   a' = (c - a) / tau_a

exactly per 100 ms step (matrix exponential, inputs c and w_hat held),
starting from the actuator output a0 the tracker predicts at the action
time. The lag is therefore simulated inside the backup instead of being
bounded by a separate term (review B4), and a manoeuvre the airframe cannot
fly at its airspeed brakes slower here, as on the plant (B3).

``backup_constraints`` replaces ``filter._backup_constraints`` (installed
at import; ``apply_filter`` looks it up as a module global) while ``CTX``
holds the per-drone context; with ``CTX`` empty it calls main's own, so the
baseline modes run main unchanged. The row's sensitivity is the position
response at k* to a unit change of the first step's acceleration through
the lag (no drag), in place of main's (k* + 0.5) dt^2.
"""

from __future__ import annotations

import math
from functools import lru_cache

import numpy as np
from numba import njit
from scipy.linalg import expm
from shared.contracts import MAX_ACCEL_COMPONENT_MPS2, MAX_SPEED_MPS, SAFE_SEPARATION_M
from utm_services.safety import filter as F

from research.adaptive_margin.envelope import TAU_ACT_S

DT = 0.1
# Per drone id: (a0 actuator output (3,), drag D (1/s), estimated wind w_hat (3,), a_phys (m/s²)).
CTX: dict[str, tuple[np.ndarray, float, np.ndarray, float]] = {}
_MAIN_CONSTRAINTS = F._backup_constraints
_limits = F._limits_kernel
_evasive = F._evasive_kernel


@lru_cache(maxsize=64)
def step_matrices(drag: float, tau: float = TAU_ACT_S) -> tuple[np.ndarray, np.ndarray]:
    """Per-axis (Phi 3x3, Gamma 3x2) over DT for x = (p, v, a) with inputs (c, w) held."""
    A = np.array([[0.0, 1.0, 0.0], [0.0, -drag, 1.0], [0.0, 0.0, -1.0 / tau]])
    B = np.array([[0.0, 0.0], [0.0, drag], [1.0 / tau, 0.0]])
    e = expm(np.block([[A, B], [np.zeros((2, 5))]]) * DT)
    return e[:3, :3].copy(), e[:3, 3:].copy()


@lru_cache(maxsize=8)
def sensitivity(steps: int, tau: float = TAU_ACT_S) -> np.ndarray:
    """Position after k + 1 steps per unit thrust change over the first step only (no drag), k < steps."""
    phi, gam = step_matrices(0.0, tau)
    x, out = gam[:, 0].copy(), np.empty(steps)
    for k in range(steps):
        out[k] = x[0]
        x = phi @ x
    return out


def horizon(tau: float = TAU_ACT_S) -> tuple[int, int]:
    """(min_steps, cap): main's horizon formulas with the declared lag."""
    cfg = F.FilterConfig(lag_s=tau)
    min_steps = F._backup_steps(cfg)
    return min_steps, max(min_steps, F._backup_max_steps(cfg))


@njit(cache=True)
def _drone_step(p, v, a, u, w, drag, a_phys, phi, gam, v_max, c):
    for d in range(3):
        c[d] = u[d] - drag * (w[d] - v[d])
    h = math.sqrt(c[0] * c[0] + c[1] * c[1])
    if h > a_phys:
        c[0] = c[0] * a_phys / h
        c[1] = c[1] * a_phys / h
    c[2] = min(max(c[2], -a_phys), a_phys)
    for d in range(3):
        p0, v0, a0 = p[d], v[d], a[d]
        p[d] = phi[0, 0] * p0 + phi[0, 1] * v0 + phi[0, 2] * a0 + gam[0, 0] * c[d] + gam[0, 1] * w[d]
        v[d] = phi[1, 0] * p0 + phi[1, 1] * v0 + phi[1, 2] * a0 + gam[1, 0] * c[d] + gam[1, 1] * w[d]
        a[d] = phi[2, 0] * p0 + phi[2, 1] * v0 + phi[2, 2] * a0 + gam[2, 0] * c[d] + gam[2, 1] * w[d]
    s = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if s > v_max:
        for d in range(3):
            v[d] = v[d] * v_max / s


@njit(cache=True)
def _kernel(p_i, p_j, v_i, v_j, a_i, a_j, w_i, w_j, phi_i, gam_i, phi_j, gam_j, drag_i, drag_j, ap_i, ap_j,
            axis, up_latched, world_min, world_max, margin, gamma, c_off, lo, hi, u_max, v_max, min_steps, steps):
    count = p_i.shape[0]
    traj = np.full((steps, count, 3), np.nan)
    a_first = np.zeros((count, 3))
    closing = np.zeros(count)
    used = np.full(count, steps)
    still = np.ones(count, dtype=np.bool_)
    pi, pj, vi, vj, ai, aj = np.empty(3), np.empty(3), np.empty(3), np.empty(3), np.empty(3), np.empty(3)
    n, m, ui, uj, ci, cj = np.empty(3), np.empty(3), np.empty(3), np.empty(3), np.empty(3), np.empty(3)
    low_i, high_i, low_j, high_j = np.empty(3), np.empty(3), np.empty(3), np.empty(3)
    for q in range(count):
        pi[:], pj[:], vi[:], vj[:], ai[:], aj[:] = p_i[q], p_j[q], v_i[q], v_j[q], a_i[q], a_j[q]
        for k in range(steps):
            r0, r1, r2 = pi[0] - pj[0], pi[1] - pj[1], pi[2] - pj[2]
            size = max(math.sqrt(r0 * r0 + r1 * r1 + r2 * r2), 1e-9)
            n[0], n[1], n[2] = r0 / size, r1 / size, r2 / size
            m[0], m[1], m[2] = -n[0], -n[1], -n[2]
            up = (1.0 if n[2] > 0.0 else -1.0) if abs(n[2]) > 1e-3 else 1.0
            if up_latched[q] != 0.0:
                up = up_latched[q]
            _limits(pi, vi, world_min, world_max, margin, gamma, c_off, u_max, low_i, high_i)
            _limits(pj, vj, world_min, world_max, margin, gamma, c_off, u_max, low_j, high_j)
            _evasive(n, up, low_i, high_i, axis[q], lo, hi, u_max, ui)
            _evasive(m, -up, low_j, high_j, axis[q], lo, hi, u_max, uj)
            if k == 0:
                for d in range(3):
                    a_first[q, d] = ui[d] - uj[d]
            _drone_step(pi, vi, ai, ui, w_i[q], drag_i[q], ap_i[q], phi_i[q], gam_i[q], v_max, ci)
            _drone_step(pj, vj, aj, uj, w_j[q], drag_j[q], ap_j[q], phi_j[q], gam_j[q], v_max, cj)
            r0, r1, r2 = pi[0] - pj[0], pi[1] - pj[1], pi[2] - pj[2]
            traj[k, q, 0], traj[k, q, 1], traj[k, q, 2] = r0, r1, r2
            u0, u1, u2 = vi[0] - vj[0], vi[1] - vj[1], vi[2] - vj[2]
            closing[q] = -(r0 * u0 + r1 * u1 + r2 * u2) / max(math.sqrt(r0 * r0 + r1 * r1 + r2 * r2), 1e-9)
            if k + 1 >= min_steps and closing[q] <= 0.0:
                used[q] = k + 1
                still[q] = False
                break
    return traj, a_first, closing, used, still


def pair_rollout(states: dict, pairs: list[tuple[str, str]], config: F.FilterConfig, frames: dict | None = None):
    """Fly the lagged backup for ``pairs`` from ``states`` (id -> DroneState) with ``CTX``.

    Returns main's tuple: (traj (K, M, 3), a_first (M, 3), closing (M,), steps flown (M,), capped (M,)).
    """
    frames = frames or {}

    def col(k: int, f) -> np.ndarray:
        return np.ascontiguousarray([f(pr[k]) for pr in pairs], dtype=np.float64)

    mats = {aid: step_matrices(float(CTX[aid][1])) for pr in pairs for aid in pr}
    min_steps, steps = horizon()
    traj, a_first, closing, used, still = _kernel(
        col(0, lambda i: states[i].position_m), col(1, lambda i: states[i].position_m),
        col(0, lambda i: states[i].velocity_mps), col(1, lambda i: states[i].velocity_mps),
        col(0, lambda i: CTX[i][0]), col(1, lambda i: CTX[i][0]),
        col(0, lambda i: CTX[i][2]), col(1, lambda i: CTX[i][2]),
        col(0, lambda i: mats[i][0]), col(0, lambda i: mats[i][1]),
        col(1, lambda i: mats[i][0]), col(1, lambda i: mats[i][1]),
        col(0, lambda i: CTX[i][1]), col(1, lambda i: CTX[i][1]),
        col(0, lambda i: CTX[i][3]), col(1, lambda i: CTX[i][3]),
        np.array([F._latch_axis(frames, pr) for pr in pairs], dtype=np.float64).reshape(-1, 3),
        np.array([frames[pr].up if pr in frames else 0.0 for pr in pairs], dtype=np.float64),
        F._WORLD_MIN_M, F._WORLD_MAX_M, float(config.margin_m), float(config.gamma),
        float(config.barrier_offset_mps), *F._NEAR_VERTICAL_SIN, MAX_ACCEL_COMPONENT_MPS2,
        MAX_SPEED_MPS * (1.0 - 1e-9), min_steps, steps,
    )
    return traj[: used.max(initial=0)], a_first, closing, used.astype(np.int_), still


def backup_constraints(states, pairs, config, frames=None):
    """``filter._backup_constraints`` on the lagged rollout; main's own while ``CTX`` is empty."""
    if not CTX:
        return _MAIN_CONSTRAINTS(states, pairs, config, frames)
    traj, a_first, _, used, capped = pair_rollout(states, pairs, config, frames)
    dist = np.nan_to_num(np.linalg.norm(traj, axis=2), nan=np.inf)
    k_star = np.argmin(dist, axis=0)
    cols = np.arange(len(pairs))
    d_star = dist[k_star, cols]
    m = traj[k_star, cols] / np.maximum(d_star, 1e-9)[:, None]
    sens = sensitivity(horizon()[1])[k_star]
    rhs = (np.einsum("ij,ij->i", m, a_first)
           - config.gamma * DT * (d_star - (SAFE_SEPARATION_M + config.margin_m)) / sens)
    return m, rhs, used, capped


F._backup_constraints = backup_constraints
