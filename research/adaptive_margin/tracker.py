"""Per-drone Kalman tracker over x = (p, v, d), with delayed measurements (P3-SA-03 §4).

Model per axis, d the wind disturbance acceleration (P3-SA-03 §3.3):

    p' = v,   v' = a_act + d,   d' = white noise of intensity q,
    a_act' = (c - a_act) / tau_a   (actuator lag; c the applied command)

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

import numpy as np
from scipy.stats import chi2
from shared.contracts import MAX_SPEED_MPS

from research.adaptive_margin.envelope import P_RISK, Gnss

H = 0.01           # plant tick, s
NIS_WINDOW = 10    # messages per NIS test (1 s at 10 Hz)
NIS_LIMIT = float(chi2.ppf(1.0 - P_RISK, 6 * NIS_WINDOW))


def kin_step(p, v, a, c, d, tau_a):
    """One tick: first-order actuator lag towards c, disturbance d, speed cap, trapezoid."""
    e = math.exp(-H / tau_a) if tau_a > 0.0 else 0.0
    a_next = c + (a - c) * e
    vn = v + (0.5 * (a + a_next) + d) * H
    speed = float(np.linalg.norm(vn))
    if speed > MAX_SPEED_MPS:
        vn = vn * (MAX_SPEED_MPS / speed)
    return p + 0.5 * (v + vn) * H, vn, a_next


def transition(dt: float, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Exact (Phi, Q) over dt for the per-axis triple integrator with white noise on d'."""
    phi1 = np.array([[1.0, dt, dt * dt / 2], [0.0, 1.0, dt], [0.0, 0.0, 1.0]])
    q1 = np.array([[dt**5 / 20, dt**4 / 8, dt**3 / 6], [dt**4 / 8, dt**3 / 3, dt**2 / 2],
                   [dt**3 / 6, dt**2 / 2, dt]])
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
        if echo:
            self.delivered = max(self.delivered, echo[-1][0])
        switched = {t for t, _ in echo}
        for t, _ in self.planned:
            if t > tick_m:
                break
            self.sent += 1
            hit = t not in switched
            self.lost += hit
            self.streak = self.streak + 1 if hit else 0
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


class Tracker:
    """Kalman filter for one drone. ``z`` is the first fix (p, v); ``q`` the per-axis disturbance intensity (m²/s⁵)."""

    def __init__(self, gnss: Gnss, q: np.ndarray, tau_a: float, tick0: int, z: np.ndarray,
                 a0: np.ndarray, p_dd0: np.ndarray):
        self.r_p = np.array([gnss.sigma_h_m, gnss.sigma_h_m, gnss.sigma_v_m]) ** 2
        self.R = np.diag(np.concatenate([self.r_p, np.full(3, gnss.sigma_vel_mps**2)]))
        self.q, self.tau_a = np.asarray(q, float), tau_a
        self.x = np.concatenate([z, np.zeros(3)])
        self.P = np.diag(np.concatenate([self.R.diagonal(), p_dd0]))
        self.tick, self.a = tick0, np.asarray(a0, float)
        self.nis: deque[float] = deque(maxlen=NIS_WINDOW)

    def _mean(self, x: np.ndarray, a: np.ndarray, t0: int, t1: int, log: CommandLog):
        p, v, d = x[:3], x[3:6], x[6:]
        for t in range(t0, t1):
            p, v, a = kin_step(p, v, a, log.at(t), d, self.tau_a)
        return np.concatenate([p, v, d]), a

    def predict(self, tick: int, log: CommandLog, floor: bool = False) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Mean, covariance and actuator output at ``tick`` (no measurement).

        With ``floor`` the posterior position covariance is first raised to
        the receiver's (``floored``); the margin uses that, the filter update not.
        """
        x, a = self._mean(self.x, self.a, self.tick, tick, log)
        phi, qm = transition((tick - self.tick) * H, self.q)
        P = self.floored(self.P) if floor else self.P
        return x, phi @ P @ phi.T + qm, a

    def predict_delay(self, tick: int, log: CommandLog, ticks_a: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """State the filter acts on: at ``tick`` (the next command's playout), then
        ``ticks_a`` more ticks holding the actuator output a0 (P3-SA-03 §3.5).

        With a first-order lag the true path from ``tick`` on differs from this
        pure-delay model by at most tau_a^2 / 2 |u - a0| in position
        (``margin.EPS_LAG_M``). Covariance floored (``floored``).
        """
        x, P, a0 = self.predict(tick, log, floor=True)
        t = ticks_a * H
        acc = a0 + x[6:]
        x = np.concatenate([x[:3] + x[3:6] * t + 0.5 * acc * t * t, x[3:6] + acc * t, x[6:]])
        phi, qm = transition(t, self.q)
        return x, phi @ P @ phi.T + qm, a0

    def update(self, tick_m: int, z: np.ndarray, log: CommandLog) -> float | None:
        """Fuse the measurement taken at ``tick_m``; older-than-posterior messages are dropped."""
        if tick_m <= self.tick:
            return None
        x, P, self.a = self.predict(tick_m, log)
        nu = z - x[:6]
        S = P[:6, :6] + self.R
        K = np.linalg.solve(S, P[:6, :]).T
        self.x = x + K @ nu
        IKH = np.eye(9) - K @ np.hstack([np.eye(6), np.zeros((6, 3))])
        self.P = IKH @ P @ IKH.T + K @ self.R @ K.T   # Joseph form
        self.tick = tick_m
        nis = float(nu @ np.linalg.solve(S, nu))
        self.nis.append(nis)
        return nis

    def nis_alarm(self) -> bool:
        """Windowed NIS above the chi-square bound at P_RISK (6 dof per message)."""
        return len(self.nis) == NIS_WINDOW and sum(self.nis) > NIS_LIMIT

    def floored(self, P: np.ndarray) -> np.ndarray:
        """No credit for averaging GNSS position: P_pp is raised to at least R_p per axis.

        The sources give no error correlation time (P3-SA-03 §3.4), so a
        constant receiver bias cannot be ruled out.
        """
        P = P.copy()
        idx = np.arange(3)
        P[idx, idx] += np.maximum(self.r_p - P[idx, idx], 0.0)
        return P
