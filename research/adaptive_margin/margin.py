"""Adaptive separation margin and envelope monitor (P3-SA-03 §3, §5).

Every term is computed each filter cycle from live quantities: the trackers'
covariance, the predicted velocities, the disturbance estimate, the age of
the unconfirmed commands and the actuator lag. The only fixed numbers are
the risk level P_RISK (founder choice), the contract limits and the
measured linearisation gap EPS_LIN_M.

Per pair (i, j), with t_a the time the filter acts at (``Tracker.predict_delay``)
and T the backup's stopping time:

    m_ij = K sqrt(lmax(S_i(t_a+T) + S_j(t_a+T)))      estimate error, open loop over T
         + du_i + du_j                                  one lost uplink command each
         + EPS_LAG_M                                    lag vs equivalent delay
         + delta_sample(V_rel)                          closest approach between samples
         + EPS_LIN_M                                    filter gap (measured, PR #12)

    T = (V_c + K sigma_vc) / (a_rel - rho),  rho = K sqrt(lmax(P_dd,i(T) + P_dd,j(T)))

The filter takes one margin, the largest over all pairs.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
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

from research.adaptive_margin.envelope import A_PHYS_MPS2, OFFBOARD_LOSS_S, P_RISK, TAU_ACT_S
from research.adaptive_margin.tracker import H, CommandLog, Tracker, transition

U_MAX = MAX_ACCEL_COMPONENT_MPS2
K = math.sqrt(float(chi2.ppf(1.0 - P_RISK, 3)))   # 3-D Gaussian error inside K sqrt(lmax) w.p. 1 - P_RISK
A_REL = math.sqrt(2.0) * U_MAX   # pair braking along the line of sight away from walls (_backup_steps)
T_MAX_S = F._backup_max_steps(F.FilterConfig()) * 0.1
# FilterConfig's margin bound at the default barrier offset c = 1 m/s.
M_MAX_M = (min(WORLD_XY_MAX_M - WORLD_XY_MIN_M, WORLD_Z_MAX_M - WORLD_Z_MIN_M) / 2.0
           - ((MAX_SPEED_MPS + 1.0) ** 2 - 1.0) / (2.0 * U_MAX))
# |u - a0| per drone: the filter's command and the actuator output both lie in
# [-u_max, u_max] per component (feedforward changes slowly, P3-SA-03 §3.5).
DC_MAX = 2.0 * math.sqrt(3.0) * U_MAX
EPS_LAG_M = 0.5 * TAU_ACT_S**2 * 2.0 * DC_MAX
# A command lost in the cycle being sent now is seen by the next cycle's
# margin (it is then unconfirmed); until then it moves the drone by at most
# 0.5 |dc| dt^2 with |dc| up to 2 sqrt(3) a_phys per drone.
DU_NOW_M = 0.5 * 2.0 * math.sqrt(3.0) * A_PHYS_MPS2 * 0.1**2
EPS_LIN_M = 0.137   # measured, PR #12 ablation (docs/results/pr12-final-report.md); not proven
HEADROOM_MPS2 = A_PHYS_MPS2 - U_MAX


def delta_sample(v_rel: float) -> float:
    """Separation lost between 100 ms samples at relative speed v_rel (straight-line pass)."""
    return math.hypot(SAFE_SEPARATION_M, v_rel * 0.1 / 2.0) - SAFE_SEPARATION_M


def lost_bound(log: CommandLog, tick_a: int, horizon_s: float) -> float:
    """Position error at t_a + horizon if one unconfirmed command was lost (hold-last).

    A lost command c_k leaves c_{k-1} in force for its 100 ms segment: a
    velocity error |c_k - c_{k-1}| 0.1 s from the segment's middle on, held
    open loop to the end of the horizon.
    """
    worst = 0.0
    for t, c in log.planned:
        dv = float(np.linalg.norm(c - log.before(t))) * 0.1
        worst = max(worst, dv * ((tick_a - t - 5) * H + horizon_s))
    return worst + DU_NOW_M


def _lmax(m: np.ndarray) -> float:
    return float(np.linalg.eigvalsh(m)[-1])


@dataclass
class Pred:
    """One drone's prediction at the filter's action time."""

    x: np.ndarray       # (p, v, d)
    P: np.ndarray       # 9 x 9, position floored
    tracker: Tracker
    log: CommandLog

    def at(self, horizon_s: float) -> np.ndarray:
        phi, qm = transition(horizon_s, self.tracker.q)
        return phi @ self.P @ phi.T + qm


@dataclass
class Result:
    margin_m: float
    alarms: set[str] = field(default_factory=set)
    worst_pair: tuple[str, str] | None = None
    horizon_s: float = 0.0


def margin(preds: dict[str, Pred], tick_a: int) -> Result:
    """Margin for this cycle: the largest pair margin, clamped to FilterConfig's bound."""
    res = Result(margin_m=EPS_LAG_M + DU_NOW_M * 2 + EPS_LIN_M)
    ids = sorted(preds)
    for k, i in enumerate(ids):
        for j in ids[k + 1:]:
            a, b = preds[i], preds[j]
            r = a.x[:3] - b.x[:3]
            n = r / max(float(np.linalg.norm(r)), 1e-9)
            dv = a.x[3:6] - b.x[3:6]
            vc = max(0.0, -float(n @ dv))
            svc = math.sqrt(float(n @ (a.P[3:6, 3:6] + b.P[3:6, 3:6]) @ n))
            T = 0.0
            for _ in range(3):   # fixed point: rho grows with T
                rho = K * math.sqrt(_lmax(a.P[6:, 6:] + b.P[6:, 6:] + np.diag(a.tracker.q + b.tracker.q) * T))
                if rho >= A_REL:
                    res.alarms.add("authority")
                    T = T_MAX_S
                    break
                T = min((vc + K * svc) / (A_REL - rho), T_MAX_S)
            Sa, Sb = a.at(T), b.at(T)
            m = (K * math.sqrt(_lmax(Sa[:3, :3] + Sb[:3, :3]))
                 + lost_bound(a.log, tick_a, T) + lost_bound(b.log, tick_a, T)
                 + EPS_LAG_M + delta_sample(float(np.linalg.norm(dv))) + EPS_LIN_M)
            if m > res.margin_m:
                res.margin_m, res.worst_pair, res.horizon_s = m, (i, j), T
    if res.margin_m > M_MAX_M:
        res.alarms.add("margin")
        res.margin_m = M_MAX_M
    return res


def monitor(preds: dict[str, Pred], res: Result, tick_now: int) -> set[str]:
    """Per-drone envelope checks (P3-SA-03 §5 table); adds to ``res.alarms``."""
    cfg = F.FilterConfig(margin_m=res.margin_m)
    for p in preds.values():
        S = p.at(res.horizon_s)
        # Wind: feedforward plus K sigma of the disturbance must fit the headroom.
        if np.any(np.abs(p.x[6:]) + K * np.sqrt(np.diag(S)[6:]) > HEADROOM_MPS2):
            res.alarms.add("wind")
        if p.tracker.nis_alarm():
            res.alarms.add("nis")
        # Link: the drone holds the last command and brakes after OFFBOARD_LOSS_S;
        # two lost commands in a row are 1e-6 under the 3GPP rate (single-loss bound broken).
        if (tick_now - p.log.delivered) * H > OFFBOARD_LOSS_S or p.log.streak >= 2:
            res.alarms.add("link")
        # Walls: a_rel = sqrt(2) u_max holds only where no world limit is active.
        low, high = F._world_accel_limits(p.x[:3], p.x[3:6], cfg)
        if np.any(low > -U_MAX + 1e-9) or np.any(high < U_MAX - 1e-9):
            res.alarms.add("wall")
    return res.alarms
