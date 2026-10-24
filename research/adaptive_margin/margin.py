"""Adaptive separation margin and envelope monitor (P3-SA-03 §3, §5).

Every term is computed each filter cycle from live quantities: the trackers'
covariance, the predicted velocities, the disturbance estimate and the
backup rollout's lines of sight. The only fixed numbers are the risk target
per pair-hour (founder choice, ``Risk``), the contract limits, the link's
playout delay and the measured linearisation gap EPS_LIN_M.

Per pair (i, j), with the filter acting at the next command's playout and T
the pair's worst-case stopping time (constant per pair, review B2):

    m_ij = K1 max_k sqrt(n_k' (S_i(T) + S_j(T))_pp n_k)    estimate error along the backup's lines of sight
         + lost_i + lost_j                                  one lost uplink command each (constant per drone)
                                                            lost_i = dc_i g_i(AGE_S + T) (re-review 3a, 3b)
         + delta_sample(|v_rel| + K1 sigma_vrel)            closest approach between samples
         + EPS_LIN_M                                        filter gap (measured, PR #12)

    T = (2 v_max + K1 sigma_vc) / (sqrt(2) u_max) + tau_a

The actuator lag, the drag and the airframe's thrust limit are simulated
inside the backup rollout (``rollout.py``), so no separate lag term remains
(review B4). The filter takes one margin, the largest over all pairs, and
raises it no faster than the barrier can absorb (``absorbable``, P3-SA-03
§3.9).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from functools import lru_cache

import numpy as np
from scipy.linalg import expm
from scipy.stats import chi2
from shared.contracts import (
    MAX_ACCEL_COMPONENT_MPS2,
    MAX_SPEED_MPS,
    SAFE_SEPARATION_M,
    WORLD_XY_MAX_M,
    WORLD_XY_MIN_M,
    WORLD_Z_MAX_M,
    WORLD_Z_MIN_M,
)
from utm_services.safety import filter as F

from research.adaptive_margin import envelope as E
from research.adaptive_margin.tracker import H, CommandLog, Tracker, transition

U_MAX = MAX_ACCEL_COMPONENT_MPS2
CYCLE = 10                         # ticks per 100 ms filter cycle
CHECKS_PER_HOUR = 36000            # one margin check per pair per 100 ms cycle
TARGETS = (1e-5, 1e-7)             # risk per pair-hour, founder decision B6 (2026-10-24)
# Uplink playout delay: the 99.9 % quantile of the one-way latency, rounded up
# to the tick. A command arriving later is dropped (counted as lost).
P_TICKS = math.ceil(E.LTE_AIR_ONE_WAY_MEDIAN_S * math.exp(E.LTE_AIR_SIGMA_LN * 3.090232) / H - 1e-9)
# Oldest telemetry the lost-command bound allows at the action time: the
# latest message may be lost and the one before it still in flight (latency
# up to P_TICKS), so the station knows the applied commands up to at least
# 2 CYCLE + P_TICKS ticks before now, and acts P_TICKS after now. Older is a
# `link` alarm.
AGE_S = (2 * P_TICKS + 2 * CYCLE) * H
# FilterConfig's margin bound at the default barrier offset c = 1 m/s.
M_MAX_M = (min(WORLD_XY_MAX_M - WORLD_XY_MIN_M, WORLD_Z_MAX_M - WORLD_Z_MIN_M) / 2.0
           - ((MAX_SPEED_MPS + 1.0) ** 2 - 1.0) / (2.0 * U_MAX))
EPS_LIN_M = 0.137   # measured, PR #12 ablation (docs/results/pr12-final-report.md); not proven
# Kalman updates that can fall between the predictions of two consecutive
# cycles: a third needs a message later than the 99.9 % latency quantile.
N_UPD = 2


@dataclass(frozen=True)
class Risk:
    """Risk target per pair-hour and its per-check quantiles (union bound over CHECKS_PER_HOUR)."""

    target: float
    q: float        # per-check risk
    k1: float       # 1-D Gaussian quantile: P(|x| > k1 sigma) = q
    k2: float       # 2-D: P(|x|_S > k2) = q
    k3: float       # 3-D: bound on the norm of a 3-vector, P(|x|_S > k3) = q


@lru_cache(maxsize=8)
def risk(target: float) -> Risk:
    q = target / CHECKS_PER_HOUR
    return Risk(target, q, *(math.sqrt(float(chi2.isf(q, k))) for k in (1, 2, 3)))


def dc_max(af: E.Airframe) -> float:
    """Largest change between two thrust commands: horizontal disc of radius a_phys and |c_z| <= a_phys."""
    return 2.0 * math.sqrt(2.0) * af.a_phys_mps2


@lru_cache(maxsize=256)
def pulse_gain(drag: float, t_s: float, tau: float = E.TAU_ACT_S) -> float:
    """Position at ``t_s`` after a unit thrust error held for one 100 ms segment (re-review 3a).

    The perturbation obeys a' = (dc - a) / tau, v' = a - D v, p' = v: the
    drag is linear in v, so neither the wind nor the state enters. The
    velocity stays >= 0 for a positive pulse, so the result is non-decreasing
    in ``t_s`` and bounds the error over the whole horizon; its limit is 0.1 / D.
    """
    A = np.array([[0.0, 1.0, 0.0], [0.0, -drag, 1.0], [0.0, 0.0, -1.0 / tau]])
    Aa = np.zeros((4, 4))
    Aa[:3, :3], Aa[2, 3] = A, 1.0 / tau
    x = expm(Aa * 0.1)[:3, 3]
    return float((expm(A * (t_s - 0.1)) @ x)[0])


def dc_bound(pred: "Pred", rk: Risk) -> float:
    """Bound on |c_k - c_(k-1)| between consecutive thrust commands to one drone (re-review 3b).

    c = clip(u - D (w_hat - v_hat)); the clip is a projection onto the convex
    thrust set and does not enlarge differences. u lies in main's box
    (2 sqrt(3) u_max apart at most). The trim changes by D times the change
    of v_hat - w_hat between predictions 100 ms apart: propagation at most
    0.1 (sqrt(2) a_phys + D (v_max + wind limit)), corrections at most N_UPD
    updates of K3 sqrt(lambda_max) of their covariance along v - w. Capped at
    the airframe's own bound. The station checks it exactly (``dc`` alarm).
    """
    af, D = pred.airframe, pred.tracker.drag
    Mvw = np.hstack([np.zeros((3, 3)), np.eye(3), -np.eye(3)])
    corr = rk.k3 * math.sqrt(max(_lmax(Mvw @ pred.tracker.corr @ Mvw.T), 0.0))
    prop = 0.1 * (math.sqrt(2.0) * af.a_phys_mps2 + D * (MAX_SPEED_MPS + af.wind_max_mps))
    return min(dc_max(af), 2.0 * math.sqrt(3.0) * U_MAX + D * (prop + N_UPD * corr))


def lost_m(af: E.Airframe, horizon_s: float, dc: float | None = None) -> float:
    """Position error at the end of the horizon if one unconfirmed command was lost (hold-last).

    A lost command leaves its predecessor in force for one 100 ms segment,
    starting at most AGE_S before the action time: a thrust error of at most
    ``dc`` (``dc_bound``; the airframe's dc_max if None), carried through the
    lag and the drag to the end of the horizon (``pulse_gain``, re-review 3a).
    """
    return (dc_max(af) if dc is None else dc) * pulse_gain(af.drag_per_s, round(AGE_S + horizon_s, 6))


def stopping_s(rk: Risk, sigma_vel_i: float, sigma_vel_j: float) -> float:
    """Worst-case stopping time T of a pair: closing at 2 v_max plus K1 sigma, braking at sqrt(2) u_max, plus the lag.

    The cleared cruise speed (``sim.cleared_speed``) keeps the braking the
    airframe can add to its drag at least u_max v / v_max, so sqrt(2) u_max
    is the pair's relative braking at every speed up to v_max.
    """
    return ((2.0 * MAX_SPEED_MPS + rk.k1 * math.hypot(sigma_vel_i, sigma_vel_j)) / (math.sqrt(2.0) * U_MAX)
            + E.TAU_ACT_S)


def delta_sample(v_rel: float) -> float:
    """Separation lost between 100 ms samples at relative speed v_rel (straight-line pass)."""
    return math.hypot(SAFE_SEPARATION_M, v_rel * 0.1 / 2.0) - SAFE_SEPARATION_M


def _lmax(m: np.ndarray) -> float:
    return float(np.linalg.eigvalsh(m)[-1])


@dataclass
class Pred:
    """One drone's prediction at the filter's action time."""

    x: np.ndarray       # (p, v, w)
    P: np.ndarray       # 9 x 9, position floored
    tracker: Tracker
    log: CommandLog
    airframe: Airframe
    a0: np.ndarray      # actuator output
    dc: float | None = None   # command-difference bound, fixed at clearance (``dc_bound``)

    def at(self, horizon_s: float) -> np.ndarray:
        phi, qm = transition(horizon_s, self.tracker.drag, self.tracker.q)
        return phi @ self.P @ phi.T + qm

    @property
    def sigma_vel(self) -> float:
        return math.sqrt(float(self.tracker.R[3, 3]))

    @property
    def drag(self) -> np.ndarray:
        """Drag acceleration D (w - v) at the action time; the feedforward cancels it."""
        return self.tracker.drag * (self.x[6:] - self.x[3:6])

    def drag_cov(self, S: np.ndarray) -> np.ndarray:
        """Covariance of D (w - v) under the state covariance S."""
        M = np.hstack([np.zeros((3, 3)), -np.eye(3), np.eye(3)]) * self.tracker.drag
        return M @ S @ M.T


Airframe = E.Airframe


@dataclass
class Result:
    margin_m: float                 # requested margin, clamped to M_MAX_M
    requested_m: float = 0.0        # before the clamp
    alarms: set[str] = field(default_factory=set)
    worst_pair: tuple[str, str] | None = None
    horizon_s: float = 0.0
    gnss_m: float = 0.0             # terms of the worst pair
    lost_m: float = 0.0


def margin(preds: dict[str, Pred], rk: Risk, los: dict[tuple[str, str], np.ndarray]) -> Result:
    """Margin for this cycle: the largest pair margin, clamped to FilterConfig's bound.

    ``los`` maps each pair (lower id, higher id) to the relative positions
    r_i - r_j along its backup rollout (K, 3), NaN rows after its end.
    """
    res = Result(margin_m=0.0)
    ids = sorted(preds)
    for k, i in enumerate(ids):
        for j in ids[k + 1:]:
            a, b = preds[i], preds[j]
            T = stopping_s(rk, a.sigma_vel, b.sigma_vel)
            S = a.at(T)[:3, :3] + b.at(T)[:3, :3]
            r = np.vstack([a.x[:3] - b.x[:3], los[(i, j)]])
            r = r[np.isfinite(r).all(axis=1)]
            n = r / np.maximum(np.linalg.norm(r, axis=1, keepdims=True), 1e-9)
            gnss = rk.k1 * math.sqrt(float(np.einsum("ki,ij,kj->k", n, S, n).max()))
            lost = lost_m(a.airframe, T, a.dc) + lost_m(b.airframe, T, b.dc)
            v_rel = float(np.linalg.norm(a.x[3:6] - b.x[3:6]))
            s_rel = math.sqrt(_lmax(a.P[3:6, 3:6] + b.P[3:6, 3:6]))
            m = gnss + lost + delta_sample(v_rel + rk.k1 * s_rel) + EPS_LIN_M
            if m > res.requested_m:
                res.requested_m, res.worst_pair, res.horizon_s, res.gnss_m, res.lost_m = m, (i, j), T, gnss, lost
    res.margin_m = min(res.requested_m, M_MAX_M)
    if res.requested_m > M_MAX_M:
        res.alarms.add("margin")
    return res


def monitor(preds: dict[str, Pred], res: Result, tick_a: int, rk: Risk, m_app: float) -> set[str]:
    """Per-drone envelope checks (P3-SA-03 §5 table) at the action tick; adds to ``res.alarms``."""
    cfg = F.FilterConfig(margin_m=m_app)
    for p in preds.values():
        S = p.at(res.horizon_s)
        # Wind: the estimate plus K2 sigma must stay within the declared max wind.
        if float(np.linalg.norm(p.x[6:8])) + rk.k2 * math.sqrt(_lmax(S[6:8, 6:8])) > p.airframe.wind_max_mps:
            res.alarms.add("wind")
        # Headroom (safety alarm, review B3): the thrust left after the drag,
        # with K2 sigma over the horizon, must give the braking the stopping
        # time T assumes, u_max |v| / v_max (``stopping_s``).
        avail = (p.airframe.a_phys_mps2 - float(np.linalg.norm(p.drag[:2]))
                 - rk.k2 * math.sqrt(_lmax(p.drag_cov(S)[:2, :2])))
        if avail < U_MAX * float(np.linalg.norm(p.x[3:5])) / MAX_SPEED_MPS:
            res.alarms.add("headroom")
        if p.tracker.nis_alarm():
            res.alarms.add("nis")
        if p.tracker.lag_alarm():
            res.alarms.add("lag")
        # Link: the drone holds the last command and brakes after OFFBOARD_LOSS_S;
        # two lost commands in a row (1e-6 under the 3GPP rate) break the
        # one-loss bound, and so does telemetry older than AGE_S.
        if ((tick_a - P_TICKS - p.log.delivered) * H > E.OFFBOARD_LOSS_S or p.log.streak >= 2
                or (tick_a - p.log.known) * H > AGE_S):
            res.alarms.add("link")
        # Walls: a_rel = sqrt(2) u_max holds only where no world limit is active.
        low, high = F._world_accel_limits(p.x[:3], p.x[3:6], cfg)
        if np.any(low > -U_MAX + 1e-9) or np.any(high < U_MAX - 1e-9):
            res.alarms.add("wall")
    return res.alarms


def absorbable(traj: np.ndarray) -> float:
    """m*: the largest margin the states still satisfy under the backup barrier (P3-SA-03 §3.9).

    The barrier for a pair is h = min |r| over the backup rollout -
    (SAFE_SEPARATION_M + margin), so h >= 0 for every pair exactly when the
    margin is at most min over pairs of (min |r| - SAFE_SEPARATION_M).
    ``traj`` is ``rollout.pair_rollout``'s (K, M, 3), run with the latched
    frames and the wall standoff of the margin in force.
    """
    if traj.shape[1] == 0:
        return math.inf
    return float(np.nan_to_num(np.linalg.norm(traj, axis=2), nan=np.inf).min()) - SAFE_SEPARATION_M
