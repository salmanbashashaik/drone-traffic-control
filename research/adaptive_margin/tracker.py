"""Per-drone Kalman tracker over x = (p, v, w), with delayed measurements (P3-SA-03 §4).

Model per axis, w the wind velocity and D the drone's drag per unit mass
(P3-SA-03 §3.3):

    p' = v,   v' = a_act + D (w - v),   w' = white noise of intensity q,
    a_act' = (c - a_act) / tau_a   (actuator lag; c the applied thrust command)

The applied command history is known: up to the time of the latest
telemetry from the drone's echo, after it from the commands sent (assumed
delivered; ``margin.py`` bounds a lost one). a_act is therefore a known input
and is not a filter state. Time is counted in plant ticks of H seconds; every
event (command switch, measurement) lies on the tick grid, so the tracker's
mean model is the plant's own step (``kin_step``).

Measurements z = (p, v) arrive late with their measurement time. The tracker
keeps the posterior at the time of the latest measurement and predicts from
there to any later time (Singletary, Chen, Ames 2020, Algorithm 1).
"""

from __future__ import annotations

import math
from bisect import bisect_right
from collections import deque
from functools import lru_cache

import numpy as np
from scipy.linalg import expm
from scipy.stats import chi2
from scipy.stats import f as f_dist
from shared.contracts import MAX_SPEED_MPS

from research.adaptive_margin.envelope import P_RISK, Gnss

H = 0.01           # plant tick, s
NIS_WINDOW = 10    # messages per NIS test (1 s at 10 Hz)
NIS_LIMIT = float(chi2.ppf(1.0 - P_RISK, 6 * NIS_WINDOW))
LAG_WINDOW = 20                          # acceleration samples per lag test (2 s at 10 Hz)
LAG_GRID = (0.5, 0.75, 1.5, 2.0, 3.0)    # alternative lags, as multiples of the declared one


def kin_step(p, v, a, c, w, drag, tau_a):
    """One tick: first-order actuator lag towards c, airspeed drag D (w - v), speed cap, trapezoid."""
    e = math.exp(-H / tau_a) if tau_a > 0.0 else 0.0
    a_next = c + (a - c) * e
    vn = v + (0.5 * (a + a_next) + drag * (w - v)) * H
    speed = float(np.linalg.norm(vn))
    if speed > MAX_SPEED_MPS:
        vn = vn * (MAX_SPEED_MPS / speed)
    return p + 0.5 * (v + vn) * H, vn, a_next


@lru_cache(maxsize=4096)
def _axis(dt: float, drag: float) -> tuple[np.ndarray, np.ndarray]:
    """Per-axis (Phi, Q at unit intensity) of (p, v, w) by Van Loan's matrix exponential."""
    A = np.array([[0.0, 1.0, 0.0], [0.0, -drag, drag], [0.0, 0.0, 0.0]])
    Qc = np.zeros((3, 3))
    Qc[2, 2] = 1.0
    vl = expm(np.block([[-A, Qc], [np.zeros((3, 3)), A.T]]) * dt)
    phi = vl[3:, 3:].T
    return phi, phi @ vl[:3, 3:]


def transition(dt: float, drag: float, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Exact (Phi, Q) over dt for x = (p, v, w) with white noise of intensity q on w'.

    The drag is linear, so the actuator output and the speed cap enter only
    the mean (``kin_step``); the covariance ignores the cap.
    """
    phi1, q1 = _axis(round(dt, 9), drag)
    return np.kron(phi1, np.eye(3)), np.kron(q1, np.diag(q))


class CommandLog:
    """Applied command history of one drone, in ticks.

    ``applied`` holds (tick, c) switches reported by the drone up to
    ``known``; ``planned`` holds the commands sent and not yet confirmed, at
    their playout ticks. ``sent`` and ``lost`` count the commands whose
    playout tick is confirmed, and those of them the drone never applied.
    """

    def __init__(self, c0: np.ndarray):
        self.applied: list[tuple[int, np.ndarray]] = [(-(10**9), c0)]
        self.planned: list[tuple[int, np.ndarray]] = []
        self.known = -(10**9)
        self.delivered = 0      # playout tick of the latest command the drone confirmed applying
        self.sent = self.lost = self.streak = 0

    def confirm(self, echo: list[tuple[int, np.ndarray]], tick_m: int) -> None:
        """The drone reports its whole switch history up to ``tick_m``.

        shortcut: the echo is the full history (free in simulation); a real
        protocol would acknowledge commands instead.
        """
        if tick_m <= self.known:
            return
        self.applied = [self.applied[0]] + echo
        self.known = tick_m
        # The echo also holds the autopilot's own commands (offboard-loss
        # braking), so only the station's commands count as delivered.
        switched = {t for t, _ in echo}
        for t, _ in self.planned:
            if t > tick_m:
                break
            self.sent += 1
            hit = t not in switched
            self.lost += hit
            self.streak = self.streak + 1 if hit else 0
            if not hit:
                self.delivered = t
        self.planned = [e for e in self.planned if e[0] > tick_m]

    def plan(self, tick: int, c: np.ndarray) -> None:
        self.planned.append((tick, c))

    def at(self, tick: int) -> np.ndarray:
        """Command in force during tick ``tick`` (from its start)."""
        if tick > self.known and self.planned and self.planned[0][0] <= tick:
            return self.planned[bisect_right(self.planned, tick, key=lambda e: e[0]) - 1][1]
        return self.applied[bisect_right(self.applied, tick, key=lambda e: e[0]) - 1][1]

    def before(self, tick: int) -> np.ndarray:
        """Command in force just before ``tick``."""
        return self.at(tick - 1)


class LagMonitor:
    """Identify the actuator lag from the command echo and the measured acceleration (P3-SA-03 §5).

    Each telemetry message carries the drone's acceleration a_meas (IMU,
    noise ``ACC_NOISE_MPS2``). The thrust part a_meas - D (w - v_meas) is
    compared with the actuator output that the echoed thrust commands give
    through a first-order lag, for the declared tau and for each alternative
    in ``LAG_GRID``. The wind is not taken from the tracker: over the last
    ``LAG_WINDOW`` samples each axis gets a free offset and a free linear
    trend, which absorb D w for a wind that is constant or changes at a
    constant rate over the window (review N1: the per-sample wind estimate
    and the wind's own change entered the residual as lag would). An F test
    asks whether any alternative fits better than the declared lag, at
    P_RISK split over the alternatives (Bonferroni). The windows overlap,
    so the false-alarm rate per run is higher than P_RISK per test. Without
    command changes all candidates agree and the test cannot fire.
    """

    def __init__(self, tau_a: float, a0: np.ndarray, tick0: int):
        taus = np.array([1.0, *LAG_GRID]) * tau_a
        self.e = np.exp(-H / taus)[:, None]
        self.a = np.tile(np.asarray(a0, float), (len(taus), 1))
        self.tick = tick0
        self.res: deque[tuple[int, np.ndarray]] = deque(maxlen=LAG_WINDOW)
        self.dof = 3 * LAG_WINDOW - 6 - 1
        self.limit = float(f_dist.ppf(1.0 - P_RISK / len(LAG_GRID), 1, self.dof))

    def add(self, tick_m: int, log: CommandLog, thrust: np.ndarray) -> None:
        """``thrust`` = a_meas + D v_meas: the thrust up to the wind's D w, which the trend absorbs."""
        if tick_m <= self.tick:
            return
        for t in range(self.tick, tick_m):
            c = log.at(t)
            self.a = c + (self.a - c) * self.e
        self.tick = tick_m
        self.res.append((tick_m, thrust - self.a))

    def alarm(self) -> bool:
        if len(self.res) < LAG_WINDOW:
            return False
        t = np.array([k for k, _ in self.res], float)
        X = np.stack([np.ones_like(t), t - t.mean()], axis=1)            # offset and trend
        r = np.stack([e for _, e in self.res], axis=1)                   # (candidates, samples, 3)
        M = np.eye(len(t)) - X @ np.linalg.pinv(X)                         # residual projector
        r = np.einsum("st,ctd->csd", M, r)
        ssr = (r**2).sum(axis=(1, 2))
        best = float(ssr[1:].min())
        return best > 0.0 and (float(ssr[0]) - best) / (best / self.dof) > self.limit


class Tracker:
    """Kalman filter for one drone.

    ``z`` is the first fix (p, v); ``q`` the per-axis wind-velocity
    intensity (m²/s³); ``drag`` the drone's D (1/s); ``p_ww0`` the wind prior.
    """

    def __init__(self, gnss: Gnss, q: np.ndarray, tau_a: float, drag: float, tick0: int, z: np.ndarray,
                 a0: np.ndarray, p_ww0: np.ndarray):
        self.r_p = np.array([gnss.sigma_h_m, gnss.sigma_h_m, gnss.sigma_v_m]) ** 2
        self.R = np.diag(np.concatenate([self.r_p, np.full(3, gnss.sigma_vel_mps**2)]))
        self.q, self.tau_a, self.drag = np.asarray(q, float), tau_a, drag
        self.x = np.concatenate([z, np.zeros(3)])
        self.P = np.diag(np.concatenate([self.R.diagonal(), p_ww0]))
        self.corr = self.P.copy()   # covariance of the latest update's correction K S K'; the prior before any
        self.tick, self.a = tick0, np.asarray(a0, float)
        self.nis: deque[float] = deque(maxlen=NIS_WINDOW)
        self.lag = LagMonitor(tau_a, a0, tick0)

    def _mean(self, x: np.ndarray, a: np.ndarray, t0: int, t1: int, log: CommandLog):
        p, v, w = x[:3], x[3:6], x[6:]
        for t in range(t0, t1):
            p, v, a = kin_step(p, v, a, log.at(t), w, self.drag, self.tau_a)
        return np.concatenate([p, v, w]), a

    def predict(self, tick: int, log: CommandLog, floor: bool = False) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Mean, covariance and actuator output at ``tick`` (no measurement).

        With ``floor`` the posterior position covariance is first raised to
        the receiver's (``floored``); the margin uses that, the filter update not.
        """
        x, a = self._mean(self.x, self.a, self.tick, tick, log)
        phi, qm = transition((tick - self.tick) * H, self.drag, self.q)
        P = self.floored(self.P) if floor else self.P
        return x, phi @ P @ phi.T + qm, a

    def update(self, tick_m: int, z: np.ndarray, log: CommandLog, a_meas: np.ndarray | None = None) -> float | None:
        """Fuse the measurement taken at ``tick_m``; older-than-posterior messages are dropped.

        ``a_meas`` (the drone's measured acceleration) feeds the lag monitor only.
        """
        if tick_m <= self.tick:
            return None
        x, P, self.a = self.predict(tick_m, log)
        nu = z - x[:6]
        S = P[:6, :6] + self.R
        K = np.linalg.solve(S, P[:6, :]).T
        self.x = x + K @ nu
        self.corr = K @ S @ K.T
        IKH = np.eye(9) - K @ np.hstack([np.eye(6), np.zeros((6, 3))])
        self.P = IKH @ P @ IKH.T + K @ self.R @ K.T   # Joseph form
        self.tick = tick_m
        nis = float(nu @ np.linalg.solve(S, nu))
        self.nis.append(nis)
        if a_meas is not None:
            self.lag.add(tick_m, log, a_meas + self.drag * z[3:])
        return nis

    def nis_alarm(self) -> bool:
        """Windowed NIS above the chi-square bound at P_RISK (6 dof per message)."""
        return len(self.nis) == NIS_WINDOW and sum(self.nis) > NIS_LIMIT

    def lag_alarm(self) -> bool:
        return self.lag.alarm()

    def floored(self, P: np.ndarray) -> np.ndarray:
        """No credit for averaging GNSS position: P_pp is raised to at least R_p per axis.

        The sources give no error correlation time (P3-SA-03 §2.1, §4.3), so a
        constant receiver bias cannot be ruled out.
        """
        P = P.copy()
        idx = np.arange(3)
        P[idx, idx] += np.maximum(self.r_p - P[idx, idx], 0.0)
        return P
