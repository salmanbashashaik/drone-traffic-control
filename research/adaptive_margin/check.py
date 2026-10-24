"""Self-checks for the P3-SA-03 prototype: python -m research.adaptive_margin.check"""

from __future__ import annotations

import math

import numpy as np
from scipy.linalg import expm
from scipy.stats import norm
from utm_services.safety import filter as F

from research.adaptive_margin import envelope as E
from research.adaptive_margin import margin as M
from research.adaptive_margin import rollout as R
from research.adaptive_margin import sim
from research.adaptive_margin.tracker import H, CommandLog, Tracker, kin_step, transition

M350 = next(a for a in E.FLEET if a.name == "dji_m350_rtk")


def check_constants() -> None:
    su, sw, lu, lw = E.dryden(E.W20_MODERATE_MPS, 100.0)
    assert abs(su - 2.13) < 0.02 and abs(lu - 262.0) < 2.0, (su, lu)   # MIL-F-8785C, h = 328 ft
    assert abs(sw - 1.543) < 0.002 and abs(lw - 100.0) < 1e-9
    for target in M.TARGETS:
        rk = M.risk(target)
        assert math.isclose(rk.q, target / 36000.0)
        assert math.isclose(rk.k1, norm.isf(rk.q / 2.0), rel_tol=1e-6)          # two-sided 1-D
        assert math.isclose(rk.k2, math.sqrt(-2.0 * math.log(rk.q)), rel_tol=1e-6)   # chi-square, 2 dof
    assert abs(M.risk(1e-5).k1 - 6.30) < 0.02 and abs(M.risk(1e-5).k2 - 6.63) < 0.01, M.risk(1e-5)
    assert abs(M.M_MAX_M - 48.57) < 0.01
    assert M.P_TICKS == 13 and abs(M.AGE_S - 0.46) < 1e-9, (M.P_TICKS, M.AGE_S)
    assert abs(M350.a_phys_mps2 - 5.662) < 0.002 and abs(M350.drag_per_s - 0.2462) < 0.0005
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


def check_step_matrices() -> None:
    """The rollout's exact 100 ms step agrees with ten plant ticks (lag, drag, no speed cap)."""
    drag, c, w = 0.3, np.array([2.0, -1.0, 0.5]), np.array([4.0, 1.0, 0.0])
    p, v, a = np.array([1.0, 2.0, 3.0]), np.array([5.0, -3.0, 1.0]), np.array([-1.0, 0.5, 0.0])
    phi, gam = R.step_matrices(drag)
    ref = np.stack([phi @ np.array([p[d], v[d], a[d]]) + gam @ np.array([c[d], w[d]]) for d in range(3)], axis=1)
    for _ in range(10):
        p, v, a = kin_step(p, v, a, c, w, drag, E.TAU_ACT_S)
    assert np.allclose(ref, np.stack([p, v, a]), atol=1e-3), (ref, p, v, a)


def check_lagged_rollout() -> None:
    """Through the lag the head-on backup comes closer than main's instant-response backup, never farther."""
    cfg = F.FilterConfig(margin_m=0.5)
    a = sim._state("a", np.array([500.0, 500, 115]), np.array([10.0, 0, 0]), 0)
    b = sim._state("b", np.array([560.0, 500.5, 115]), np.array([-10.0, 0, 0]), 0)
    traj_main, *_ = F._backup_rollout(np.array([a.position_m]), np.array([b.position_m]), np.array([a.velocity_mps]),
                                      np.array([b.velocity_mps]), cfg, np.zeros((1, 3)), np.zeros(1))
    try:
        for s in (a, b):   # trimmed in still air: thrust cancels the drag
            R.CTX[s.agent_id] = (-M350.drag_per_s * -np.array(s.velocity_mps), M350.drag_per_s, np.zeros(3),
                                 M350.a_phys_mps2)
        traj = R.pair_rollout({"a": a, "b": b}, [("a", "b")], cfg)[0]
        rows = R.backup_constraints({"a": a, "b": b}, [("a", "b")], cfg)
    finally:
        R.CTX.clear()
    d_main = float(np.nanmin(np.linalg.norm(traj_main, axis=2)))
    d_lag = float(np.nanmin(np.linalg.norm(traj, axis=2)))
    assert d_lag < d_main, (d_lag, d_main)
    assert M.absorbable(traj) == d_lag - 5.0 and rows[0].shape == (1, 3) and np.isfinite(rows[1]).all()
    assert R.backup_constraints({"a": a, "b": b}, [("a", "b")], cfg)[1].shape == (1,)   # main's own with CTX empty


def check_drag() -> None:
    """Constant thrust c in wind w: the speed settles at w + c / D, not at the cap."""
    drag, w, c = 0.25, np.array([3.0, -2.0, 0.0]), np.array([1.0, 0.0, 0.0])
    p = v = a = np.zeros(3)
    for _ in range(6000):
        p, v, a = kin_step(p, v, a, c, w, drag, E.TAU_ACT_S)
    assert np.allclose(v, w + c / drag, atol=1e-3), v


def check_cleared_speed() -> None:
    """At the cleared speed the thrust left after the drag is exactly the braking the margin assumes."""
    rk = M.risk(1e-5)
    calm = sim.NOMINAL[0]
    assert calm.wind_mps == 0.0 and abs(sim.cleared_speed(M350, calm, rk) - 11.81) < 0.01
    india = next(c for c in sim.NOMINAL if c.name.endswith("/india"))
    W = india.wind_mps + rk.k2 * E.dryden(india.w20_mps, sim.Z_CENTRE_M)[0]
    for af in E.FLEET:
        v = sim.cleared_speed(af, india, rk)
        assert v < M.MAX_SPEED_MPS and math.isclose(af.a_phys_mps2 - af.drag_per_s * (W + v), M.U_MAX * v / 15.0)
    assert sim.cleared_speed(M350, india, M.risk(1e-7)) < sim.cleared_speed(M350, india, rk)
    p0, v0 = sim.scenario("headon2", 0)
    p1, v1, vc = sim.cleared(p0, v0, india, rk, 0)
    assert np.allclose(p0 + v0 * sim.T_C_S, p1 + v1 * sim.T_C_S)              # same meeting point
    assert np.all(np.linalg.norm(v1, axis=1) <= vc + 1e-9)


def check_command_log() -> None:
    log = CommandLog(np.zeros(3))
    for t in (13, 23, 33):
        log.plan(t, np.full(3, float(t)))
    assert log.at(20)[0] == 13 and log.at(40)[0] == 33 and log.at(5)[0] == 0
    log.confirm([(13, np.full(3, 13.0)), (28, np.full(3, -1.0))], 30)     # 23 lost; 28 the autopilot's own
    assert log.lost == 1 and log.streak == 1 and log.at(25)[0] == 13 and log.at(29)[0] == -1 and log.at(40)[0] == 33
    assert log.delivered == 13


def _fly(tau_true: float, steps: int, seed: int, square: float = 0.0, ramp: float = 0.0):
    """In-model flight with 10 Hz fixes: random-walk wind through drag, optional x thrust square wave (1 s halves).

    ``ramp`` adds a steady wind change along x (m/s²). Returns the tracker's
    NIS values and, per fix, whether the lag monitor alarms.
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
        w = w + rng.normal(0, np.sqrt(q * H)) + np.array([ramp * H, 0.0, 0.0])
        if t % 10 == 0:
            z = np.concatenate([p + rng.normal(0, sig), v + rng.normal(0, g.sigma_vel_mps, 3)])
            a_meas = a + drag * (w - v) + rng.normal(0, E.ACC_NOISE_MPS2, 3)
            nis.append(tr.update(t, z, log, a_meas))
            alarms.append(tr.lag_alarm())
    return nis, alarms


def check_nis() -> None:
    """In-model data (white GNSS noise, random-walk wind through drag): mean NIS near 6 dof."""
    nis, _ = _fly(E.TAU_ACT_S, 5000, 3)
    m = float(np.mean(nis[50:]))
    assert 5.6 < m < 6.4, m


def check_lag_id() -> None:
    """The lag monitor finds a lag twice the declared one and stays quiet at the declared one, also in a wind ramp.

    It needs command changes: with a steady command it cannot fire.
    """
    _, slow = _fly(2.0 * E.TAU_ACT_S, 3000, 5, square=2.0)
    assert np.mean(slow[30:]) > 0.9, np.mean(slow[30:])
    _, ok = _fly(E.TAU_ACT_S, 20000, 6, square=2.0)
    assert np.mean(ok) < 0.01, np.mean(ok)
    _, ramp = _fly(E.TAU_ACT_S, 20000, 8, square=2.0, ramp=0.5)
    assert np.mean(ramp) < 0.01, np.mean(ramp)
    _, flat = _fly(2.0 * E.TAU_ACT_S, 3000, 7)
    assert not any(flat)


def _pred(x: np.ndarray, P: np.ndarray) -> M.Pred:
    tr = Tracker(E.SBAS, np.full(3, 0.01), E.TAU_ACT_S, M350.drag_per_s, 0, np.zeros(6), np.zeros(3),
                 np.full(3, 0.01))
    return M.Pred(x, P, tr, CommandLog(np.zeros(3)), M350, np.zeros(3))


def check_covariance_grows() -> None:
    """N2: the predicted position covariance is non-decreasing in the horizon, so S(T) bounds every S(tau <= T)."""
    pr = _pred(np.array([500.0, 500, 115, 10, 0, 0, 3, 0, 0]), np.diag([9.0, 9, 15, 0.01, 0.01, 0.01, 1, 1, 0.1]))
    lm = [float(np.linalg.eigvalsh(pr.at(tau)[:3, :3])[-1]) for tau in np.linspace(0.0, 8.0, 81)]
    assert all(b >= a - 1e-12 for a, b in zip(lm, lm[1:])), lm


def check_margin() -> None:
    """The margin grows with the covariance, uses the error along the line of sight, and keeps the terms."""
    rk = M.risk(1e-5)
    x_a = np.array([500.0, 500, 115, 10, 0, 0, 0, 0, 0])
    x_b = np.array([560.0, 500, 115, -10, 0, 0, 0, 0, 0])
    base = np.diag([1.0, 1, 1, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
    los = {("a", "b"): np.empty((0, 3))}

    def m(P_a, P_b, xa=x_a, xb=x_b):
        return M.margin({"a": _pred(xa, P_a), "b": _pred(xb, P_b)}, rk, los)

    ms = [m(base * s, base * s).margin_m for s in (1.0, 2.0, 4.0)]
    assert ms[0] < ms[1] < ms[2], ms
    # Closing faster raises the sampling term.
    fast = m(base, base, x_a * [1, 1, 1, 1.5, 1, 1, 1, 1, 1], x_b * [1, 1, 1, 1.5, 1, 1, 1, 1, 1])
    assert fast.margin_m > ms[0]
    # Direction-aware (review B1): vertical error does not widen a horizontal line of sight.
    tall, wide = np.diag([1.0, 1, 100, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]), np.diag(
        [100.0, 1, 1, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
    r_tall, r_wide = m(tall, tall), m(wide, wide)
    assert r_tall.gnss_m < r_wide.gnss_m / 5.0, (r_tall.gnss_m, r_wide.gnss_m)
    # A line of sight in the rollout that turns vertical picks the vertical error up.
    turned = M.margin({"a": _pred(x_a, tall), "b": _pred(x_b, tall)}, rk, {("a", "b"): np.array([[0.0, 0.0, 6.0]])})
    assert turned.gnss_m > 5.0 * r_tall.gnss_m
    # Lost-command term: constant per pair, one segment per drone, through the lag and drag (re-review 3a).
    T = M.stopping_s(rk, math.sqrt(_pred(x_a, base).tracker.R[3, 3]), math.sqrt(_pred(x_a, base).tracker.R[3, 3]))
    assert math.isclose(r_tall.lost_m, 2 * M.dc_max(M350) * M.pulse_gain(M350.drag_per_s, round(M.AGE_S + T, 6)))
    assert math.isclose(r_tall.horizon_s, T) and math.isclose(r_tall.requested_m, r_tall.margin_m)


def check_pulse_gain() -> None:
    """The lost segment's position error (re-review 3a) against ten-tick plant steps; monotone; limit 0.1 / D."""
    drag = M350.drag_per_s
    p = v = a = np.zeros(3)
    ts, ps = [], []
    for k in range(1, 1001):
        c = np.array([1.0, 0.0, 0.0]) if k <= 10 else np.zeros(3)
        p, v, a = kin_step(p, v, a, c, np.zeros(3), drag, E.TAU_ACT_S)
        if k % 10 == 0:
            ts.append(k * H)
            ps.append(p[0])
    g = [M.pulse_gain(drag, t) for t in ts]
    assert np.allclose(g, ps, atol=2e-3), max(abs(x - y) for x, y in zip(g, ps))
    assert all(b >= a - 1e-12 for a, b in zip(g, g[1:]))
    assert abs(M.pulse_gain(drag, 200.0) - 0.1 / drag) < 1e-6
    assert M.pulse_gain(drag, 6.9) < 0.5 * 0.1 * 6.9   # at least a factor 2 against the held error


def check_dc_bound() -> None:
    """The thrust clip is non-expansive; the command-difference bound lies between the box and the airframe (3b)."""
    rng = np.random.default_rng(11)
    for _ in range(2000):
        x, y = rng.normal(0.0, 8.0, 3), rng.normal(0.0, 8.0, 3)
        cx, cy = sim._authority(x, M350)[0], sim._authority(y, M350)[0]
        assert np.linalg.norm(cx - cy) <= np.linalg.norm(x - y) + 1e-12
    pr = _pred(np.array([500.0, 500, 115, 10, 0, 0, 3, 0, 0]), np.eye(9) * 0.01)
    box = 2.0 * math.sqrt(3.0) * M.U_MAX
    dc = M.dc_bound(pr, M.risk(1e-5))
    assert box < dc <= M.dc_max(M350) and math.isclose(box, 12.124, rel_tol=1e-4), dc
    pr.tracker.corr = np.eye(9) * 1e4          # a huge correction: the airframe's own bound applies
    assert M.dc_bound(pr, M.risk(1e-5)) == M.dc_max(M350)


def check_absorbable() -> None:
    """m* is the rollout's closest distance minus 5 m; inf without pairs; NaN rows ignored."""
    traj = np.full((4, 2, 3), np.nan)
    traj[:3, 0] = [[20.0, 0, 0], [12.0, 0, 0], [15.0, 0, 0]]
    traj[:2, 1] = [[0.0, 30, 0], [0.0, 0, 9]]
    assert M.absorbable(traj) == 4.0 and M.absorbable(np.empty((0, 0, 3))) == math.inf


def check_job_order() -> None:
    """The priority order keeps every job once: in-limit RTK and SBAS adaptive first, then B6 and B8."""
    from research.adaptive_margin import run
    js = run.jobs()
    keys = [run._key(j) for j in js]
    assert len(js) == 2336 and len(set(keys)) == len(keys), len(js)
    ranks = [run._priority(j) for j in js]
    assert ranks == sorted(ranks)
    first = {(j["cond"], j["mode"], j["target"]) for j in js if ranks[0] == run._priority(j)}
    assert first == {(f"{g}/{w}", "adaptive", t) for g in ("rtk_f9p", "sbas_gagan") for w in ("calm", "india")
                     for t in M.TARGETS}, first
    assert {j["cond"] for j in js if run._priority(j) == 1} == set(run.PRIORITY_CASES)


def check_silent_los() -> None:
    """`absorb` and `margin` are not warnings; every other alarm, `headroom` at warm-up too, is."""
    from research.adaptive_margin import run
    assert run._silent_los({"los_s": 11.76, "alarms": {"absorb": 8.4, "margin": 2.0}})
    assert not run._silent_los({"los_s": 11.76, "alarms": {"headroom": sim.WARMUP_S}})
    assert not run._silent_los({"los_s": 11.0, "alarms": {"headroom": 9.0}})
    assert run._silent_los({"los_s": 11.0, "alarms": {"lag": 11.5}})


if __name__ == "__main__":
    for f in (check_constants, check_transition, check_step_matrices, check_lagged_rollout, check_drag,
              check_cleared_speed, check_command_log, check_nis, check_lag_id, check_covariance_grows, check_margin,
              check_pulse_gain, check_dc_bound, check_absorbable, check_job_order, check_silent_los):
        f()
        print("ok", f.__name__)
