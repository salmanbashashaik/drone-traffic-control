"""Self-checks for the P3-SA-03 prototype: python -m research.adaptive_margin.check"""

from __future__ import annotations

import math

import numpy as np
from scipy.linalg import expm
from utm_services.safety import filter as F

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
    m350 = next(a for a in E.FLEET if a.name == "dji_m350_rtk")
    assert abs(m350.a_phys_mps2 - 5.662) < 0.002 and abs(m350.drag_per_s - 0.2462) < 0.0005
    assert all(a.a_phys_mps2 > M.U_MAX for a in E.FLEET)


def check_transition() -> None:
    """Phi and Q against Van Loan's matrix exponential of the continuous 9-state model."""
    dt, drag, q = 0.37, 0.25, np.array([0.3, 0.2, 0.1])
    A = np.kron(np.array([[0, 1, 0], [0, -drag, drag], [0, 0, 0]], float), np.eye(3))
    Qc = np.zeros((9, 9))
    Qc[6:, 6:] = np.diag(q)
    vl = expm(np.block([[-A, Qc], [np.zeros((9, 9)), A.T]]) * dt)
    phi_ref = vl[9:, 9:].T
    phi, qm = transition(dt, drag, q)
    assert np.allclose(phi, phi_ref) and np.allclose(qm, phi_ref @ vl[:9, 9:], atol=1e-12)


def check_lag_bound() -> None:
    """Lag minus equivalent delay stays within [0, tau^2 / 2 |dc|] and tends to its upper end."""
    tau, dc = E.TAU_ACT_S, 3.0
    p = v = a = np.zeros(3)
    worst = 0.0
    c = np.array([dc, 0, 0])
    for k in range(1, 400):
        p, v, a = kin_step(p, v, a, c, np.zeros(3), 0.0, tau)
        t = k * H
        ref = 0.5 * dc * max(t - tau, 0.0) ** 2
        worst = max(worst, p[0] - ref)
        assert -1e-6 <= p[0] - ref <= 0.5 * tau**2 * dc + 1e-3
    assert worst > 0.49 * tau**2 * dc


def check_drag() -> None:
    """Constant thrust c in wind w: the speed settles at w + c / D, not at the cap."""
    drag, w, c = 0.25, np.array([3.0, -2.0, 0.0]), np.array([1.0, 0.0, 0.0])
    p = v = a = np.zeros(3)
    for _ in range(6000):
        p, v, a = kin_step(p, v, a, c, w, drag, E.TAU_ACT_S)
    assert np.allclose(v, w + c / drag, atol=1e-3), v


def check_command_log() -> None:
    log = CommandLog(np.zeros(3))
    for t in (13, 23, 33):
        log.plan(t, np.full(3, float(t)))
    assert log.at(20)[0] == 13 and log.at(40)[0] == 33 and log.at(5)[0] == 0
    log.confirm([(13, np.full(3, 13.0))], 30)     # 23 lost
    assert log.lost == 1 and log.streak == 1 and log.at(25)[0] == 13 and log.at(40)[0] == 33
    assert log.delivered == 13


def _fly(tau_true: float, steps: int, seed: int, square: float = 0.0):
    """In-model flight with 10 Hz fixes: random-walk wind through drag, optional x thrust square wave (1 s halves).

    Returns the tracker's NIS values and, per fix, whether the lag monitor alarms.
    """
    rng = np.random.default_rng(seed)
    g, drag, q = E.SBAS, 0.25, np.full(3, 0.02)
    sig = np.array([g.sigma_h_m, g.sigma_h_m, g.sigma_v_m])
    p, v, w, a = np.array([500.0, 500, 100]), np.array([5.0, 0, 0]), np.zeros(3), np.zeros(3)
    log = CommandLog(np.zeros(3))
    z = np.concatenate([p + rng.normal(0, sig), v + rng.normal(0, g.sigma_vel_mps, 3)])
    tr = Tracker(g, q, E.TAU_ACT_S, drag, 0, z, np.zeros(3), np.full(3, 1.0))
    nis, alarms = [], []
    for t in range(1, steps + 1):
        if square and (t - 1) % 100 == 0:
            log.plan(t - 1, np.array([square if (t - 1) // 100 % 2 == 0 else -square, 0.0, 0.0]))
        p, v, a = kin_step(p, v, a, log.at(t - 1), w, drag, tau_true)
        w = w + rng.normal(0, np.sqrt(q * H))
        if t % 10 == 0:
            z = np.concatenate([p + rng.normal(0, sig), v + rng.normal(0, g.sigma_vel_mps, 3)])
            a_meas = a + drag * (w - v) + rng.normal(0, E.ACC_NOISE_MPS2, 3)
            nis.append(tr.update(t, z, log, a_meas))
            alarms.append(tr.lag.alarm())
    return nis, alarms


def check_nis() -> None:
    """In-model data (white GNSS noise, random-walk wind through drag): mean NIS near 6 dof."""
    nis, _ = _fly(E.TAU_ACT_S, 5000, 3)
    m = float(np.mean(nis[50:]))
    assert 5.6 < m < 6.4, m


def check_lag_id() -> None:
    """The lag monitor finds a lag twice the declared one and stays quiet at the declared one.

    It needs command changes: with a steady command it cannot fire.
    """
    _, slow = _fly(2.0 * E.TAU_ACT_S, 3000, 5, square=2.0)
    assert np.mean(slow[30:]) > 0.9, np.mean(slow[30:])
    _, ok = _fly(E.TAU_ACT_S, 20000, 6, square=2.0)
    assert np.mean(ok) < 0.01, np.mean(ok)
    _, flat = _fly(2.0 * E.TAU_ACT_S, 3000, 7)
    assert not any(flat)


def check_margin_monotone() -> None:
    log = CommandLog(np.zeros(3))
    af = E.FLEET[0]
    tr = Tracker(E.SBAS, np.full(3, 0.01), E.TAU_ACT_S, af.drag_per_s, 0, np.zeros(6), np.zeros(3),
                 np.full(3, 0.01))
    x_a = np.array([500.0, 500, 115, 10, 0, 0, 0, 0, 0])
    x_b = np.array([560.0, 500, 115, -10, 0, 0, 0, 0, 0])
    base = np.diag([1.0, 1, 1, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
    ms = [M.margin({"a": M.Pred(x_a, base * s, tr, log, af), "b": M.Pred(x_b, base * s, tr, log, af)}, 0).margin_m
          for s in (1.0, 2.0, 4.0)]
    assert ms[0] < ms[1] < ms[2], ms
    # Closing faster lengthens the horizon and so the margin.
    fast = M.margin({"a": M.Pred(x_a * [1, 1, 1, 1.5, 1, 1, 1, 1, 1], base, tr, log, af),
                     "b": M.Pred(x_b * [1, 1, 1, 1.5, 1, 1, 1, 1, 1], base, tr, log, af)}, 0)
    assert fast.margin_m > ms[0] and math.isfinite(fast.horizon_s)


def check_absorbable() -> None:
    """m* is about |r| - 5 for a receding pair, less for a closing one, and shrinks as the pair closes."""
    cfg = F.FilterConfig(margin_m=2.0)

    def m_star(gap: float, speed: float) -> float:
        a = sim._state("a", np.array([500.0, 500, 115]), np.array([speed, 0, 0]), 0)
        b = sim._state("b", np.array([500.0 + gap, 500, 115]), np.array([-speed, 0, 0]), 0)
        return M.absorbable([a, b], {}, cfg)

    rec = m_star(40.0, -5.0)
    assert 40.0 - 5.0 <= rec <= 40.0 - 5.0 + 2 * sim.MAX_SPEED_MPS * H * 10, rec   # one 0.1 s step
    closing = [m_star(g, 10.0) for g in (60.0, 45.0, 30.0)]
    assert closing[0] < 60.0 - 5.0 and closing[0] > closing[1] > closing[2], closing
    assert M.absorbable([sim._state("a", np.full(3, 500.0), np.zeros(3), 0)], {}, cfg) == math.inf


if __name__ == "__main__":
    for f in (check_constants, check_transition, check_lag_bound, check_drag, check_command_log, check_nis,
              check_lag_id, check_margin_monotone, check_absorbable):
        f()
        print("ok", f.__name__)
