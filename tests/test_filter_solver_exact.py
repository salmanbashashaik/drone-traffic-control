"""The filter QP is solved to KKT accuracy (perf/solver-exact, PROPOSAL-scale-qp.md option 1).

- DAQP returns KKT-certified solutions of the filter's QPs; OSQP at the
  default eps does too except on some escape QPs (stationarity a few 1e-7,
  so the filter takes DAQP's there); where both are certified they agree to
  1e-7 m/s².
- The certificate rejects a solution that is not the optimum.
- A pass-through drone near (not at) a box bound keeps its nominal: OSQP at
  eps 1e-5 with polish snapped it onto the bound (0.015 m/s² off).
"""

import numpy as np
from shared.contracts import DroneState, MARLAction

from utm_services.safety import filter as F


def _maps(p, v, accel):
    ids = [f"d{i:02d}" for i in range(len(p))]
    states = {a: DroneState(agent_id=a, position_m=tuple(p[k]), velocity_mps=tuple(v[k]), goal_m=(900.0, 900.0, 80.0),
                            timestamp_ms=0) for k, a in enumerate(ids)}
    actions = {a: MARLAction(agent_id=a, proposed_accel_mps2=tuple(accel[k]), timestamp_ms=0) for k, a in enumerate(ids)}
    return states, actions, [(a, b) for k, a in enumerate(ids) for b in ids[k + 1:]]


def _random_qps(seed, count):
    rng = np.random.default_rng(seed)
    config = F.FilterConfig()
    for _ in range(count):
        n = int(rng.integers(2, 7))
        p = np.column_stack([rng.uniform(450, 520, n), rng.uniform(450, 520, n), rng.uniform(80, 130, n)])
        v = rng.normal(size=(n, 3))
        v *= rng.uniform(0, 14.9, (n, 1)) / np.linalg.norm(v, axis=1, keepdims=True)
        states, actions, pairs = _maps(p, v, rng.uniform(-3.5, 3.5, (n, 3)))
        frames = F._update_latches(states, pairs, config, {})
        escape = ({pair: (float(rng.choice([-1, 1])), float(rng.choice([-1, 0, 1]))) for pair in pairs
                   if rng.random() < 0.3} if rng.random() < 0.5 else None)
        yield F.build_hocbf_qp_matrices(states, actions, pairs, config, escape, None, frames), bool(escape)


def test_daqp_is_certified_and_agrees_with_tight_osqp() -> None:
    config = F.FilterConfig()
    for (P, q, A, l, u, ids), escape in _random_qps(0, 60):
        x_d, y_d = F._solve_daqp(P, q, A, l, u)
        x_o, y_o = F._solve_osqp(P, q, A, l, u, config, escape)
        assert F._kkt_certified(P, q, A, l, u, x_d, y_d)
        certified = F._kkt_certified(P, q, A, l, u, x_o, y_o)
        assert certified or escape
        if certified:
            assert np.max(np.abs(x_d[:3 * len(ids)] - x_o[:3 * len(ids)])) <= 1e-7


def test_certificate_rejects_a_point_that_is_not_the_optimum() -> None:
    (P, q, A, l, u, _), _ = next(_random_qps(1, 1))
    x, y = F._solve_daqp(P, q, A, l, u)
    moved = x.copy()
    moved[0] += 1e-3  # feasible or not, no longer stationary with these multipliers
    assert not F._kkt_certified(P, q, A, l, u, moved, y)
    assert not F._kkt_certified(P, q, A, l, u, x, np.zeros_like(y))


def test_pass_through_near_the_box_keeps_the_nominal() -> None:
    # Two drones far apart and separating: no pair row binds, so the optimum is the nominal.
    p = np.array([[400.0, 500.0, 100.0], [600.0, 500.0, 100.0]])
    v = np.array([[-5.0, 0.0, 0.0], [5.0, 0.0, 0.0]])
    nominal = np.array([[-3.4849145463935134, 2.167922127639706, 0.715406761949823], [0.3, -0.2, 0.1]])
    states, actions, pairs = _maps(p, v, nominal)
    safe, slack, ok = F._solve_hocbf_qp(states, actions, pairs, F.FilterConfig(), frames={})
    assert ok and not slack
    for k, aid in enumerate(sorted(states)):
        assert np.max(np.abs(np.array(safe[aid]) - nominal[k])) <= 1e-9
