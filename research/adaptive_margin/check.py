"""Self-checks for the P3-SA-03 prototype: python -m research.adaptive_margin.check"""

from __future__ import annotations

import math

import numpy as np
from scipy.linalg import expm

from research.adaptive_margin import envelope as E
from research.adaptive_margin import margin as M
from research.adaptive_margin import sim
from research.adaptive_margin.tracker import H, CommandLog, Tracker, kin_step, transition


def check_constants() -> None:
    su, sw, lu, lw = E.dryden(E.W20_MODERATE_MPS, 100.0)
    assert abs(su - 2.13) < 0.02 and abs(lu - 262.0) < 2.0, (su, lu)   # MIL-F-8785C, h = 328 ft
    assert abs(sw - 1.543) < 0.002 and abs(lw - 100.0) < 1e-9
    assert abs(M.K - 4.033) < 0.002 and abs(M.M_MAX_M - 48.57) < 0.01 and abs(M.EPS_LAG_M - 0.758) < 0.002
    assert sim.P_TICKS == 13 and sim.A_TICKS == 25, (sim.P_TICKS, sim.A_TICKS)


def check_transition() -> None:
    """Phi and Q against Van Loan's matrix exponential of the continuous model."""
    dt, q = 0.37, np.array([0.3, 0.2, 0.1])
    A = np.kron(np.array([[0, 1, 0], [0, 0, 1], [0, 0, 0]], float), np.eye(3))
    Qc = np.zeros((9, 9))
    Qc[6:, 6:] = np.diag(q)
    vl = expm(np.block([[-A, Qc], [np.zeros((9, 9)), A.T]]) * dt)
    phi_ref = vl[9:, 9:].T
    phi, qm = transition(dt, q)
    assert np.allclose(phi, phi_ref) and np.allclose(qm, phi_ref @ vl[:9, 9:], atol=1e-12)


def check_lag_bound() -> None:
    """Lag minus equivalent delay stays within [0, tau^2 / 2 |dc|] and tends to its upper end."""
    tau, dc = E.TAU_ACT_S, 3.0
    p = v = a = np.zeros(3)
    worst = 0.0
    c = np.array([dc, 0, 0])
    for k in range(1, 400):
        p, v, a = kin_step(p, v, a, c, np.zeros(3), tau)
        t = k * H
        ref = 0.5 * dc * max(t - tau, 0.0) ** 2
        worst = max(worst, p[0] - ref)
        assert -1e-6 <= p[0] - ref <= 0.5 * tau**2 * dc + 1e-3
    assert worst > 0.49 * tau**2 * dc


def check_command_log() -> None:
    log = CommandLog(np.zeros(3))
    for t in (13, 23, 33):
        log.plan(t, np.full(3, float(t)))
    assert log.at(20)[0] == 13 and log.at(40)[0] == 33 and log.at(5)[0] == 0
    log.confirm([(13, np.full(3, 13.0))], 30)     # 23 lost
    assert log.lost == 1 and log.streak == 1 and log.at(25)[0] == 13 and log.at(40)[0] == 33
    assert log.delivered == 13


def check_nis() -> None:
    """In-model data (white GNSS noise, random-walk disturbance): mean NIS near 6 dof."""
    rng = np.random.default_rng(3)
    g, q = E.SBAS, np.full(3, 0.02)
    sig = np.array([g.sigma_h_m, g.sigma_h_m, g.sigma_v_m])
    p, v, d, a = np.array([500.0, 500, 100]), np.array([5.0, 0, 0]), np.zeros(3), np.zeros(3)
    log = CommandLog(np.zeros(3))
    z = np.concatenate([p + rng.normal(0, sig), v + rng.normal(0, g.sigma_vel_mps, 3)])
    tr = Tracker(g, q, E.TAU_ACT_S, 0, z, np.zeros(3), np.full(3, 0.05))
    nis = []
    for t in range(1, 5001):
        p, v, a = kin_step(p, v, a, np.zeros(3), d, E.TAU_ACT_S)
        d = d + rng.normal(0, np.sqrt(q * H))
        if t % 10 == 0:
            z = np.concatenate([p + rng.normal(0, sig), v + rng.normal(0, g.sigma_vel_mps, 3)])
            nis.append(tr.update(t, z, log))
    m = float(np.mean(nis[50:]))
    assert 5.6 < m < 6.4, m


def check_margin_monotone() -> None:
    log = CommandLog(np.zeros(3))
    tr = Tracker(E.SBAS, np.full(3, 0.01), E.TAU_ACT_S, 0, np.zeros(6), np.zeros(3), np.full(3, 0.01))
    x_a = np.array([500.0, 500, 115, 10, 0, 0, 0, 0, 0])
    x_b = np.array([560.0, 500, 115, -10, 0, 0, 0, 0, 0])
    base = np.diag([1.0, 1, 1, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
    ms = [M.margin({"a": M.Pred(x_a, base * s, tr, log), "b": M.Pred(x_b, base * s, tr, log)}, 0).margin_m
          for s in (1.0, 2.0, 4.0)]
    assert ms[0] < ms[1] < ms[2], ms
    # Closing faster lengthens the horizon and so the margin.
    fast = M.margin({"a": M.Pred(x_a * [1, 1, 1, 1.5, 1, 1, 1, 1, 1], base, tr, log),
                     "b": M.Pred(x_b * [1, 1, 1, 1.5, 1, 1, 1, 1, 1], base, tr, log)}, 0)
    assert fast.margin_m > ms[0] and math.isfinite(fast.horizon_s)


if __name__ == "__main__":
    for f in (check_constants, check_transition, check_lag_bound, check_command_log, check_nis,
              check_margin_monotone):
        f()
        print("ok", f.__name__)
