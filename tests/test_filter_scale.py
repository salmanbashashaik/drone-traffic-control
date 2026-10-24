"""The scale changes of the filter (perf/scale) keep its results.

- ``_conflict_screen`` (vectorised ``_update_latches`` tests) decides every
  pair as the per-pair loop it replaced, including pairs on the thresholds.
- ``build_hocbf_qp_matrices`` returns the matrices of the dense-row build it
  replaced, bit for bit (``_dense_row_build`` is that build, kept as the oracle).
- A closed loop of ``apply_filter`` above 256 pairs gives the same actions with
  the threaded rollout as with the serial one (the fuzz tier never reaches it).
- ``_solve_hocbf_qp`` solved as independent problems has the optimum of the one
  joint QP (within the KKT certificate), and every row it drops holds for every
  acceleration in the box.
"""

import numpy as np
import pytest
import scipy.sparse as sp
from shared.contracts import MAX_SPEED_MPS, DroneState, MARLAction

from utm_services.safety import filter as F


def _traffic(rng, n, spread=60.0, speed=14.9):
    p = np.column_stack([rng.uniform(0, spread, n) + 400, rng.uniform(0, spread, n) + 400, rng.uniform(60, 60 + spread, n)])
    v = rng.normal(size=(n, 3))
    v *= rng.uniform(0, speed, (n, 1)) / np.linalg.norm(v, axis=1, keepdims=True)
    return p, v


def _maps(p, v, rng=None):
    ids = [f"d{i:02d}" for i in range(len(p))]
    states = {a: DroneState(agent_id=a, position_m=tuple(p[k]), velocity_mps=tuple(v[k]), goal_m=(900.0, 900.0, 80.0),
                            timestamp_ms=0) for k, a in enumerate(ids)}
    accel = (lambda: tuple(rng.uniform(-3.5, 3.5, 3))) if rng is not None else (lambda: (0.0, 0.0, 0.0))
    actions = {a: MARLAction(agent_id=a, proposed_accel_mps2=accel(), timestamp_ms=0) for a in ids}
    return states, actions, [(a, b) for k, a in enumerate(ids) for b in ids[k + 1:]]


def test_conflict_screen_matches_the_per_pair_loop() -> None:
    rng = np.random.default_rng(0)
    limit = F.SAFE_SEPARATION_M + F.FilterConfig().margin_m + F._PLUMB_OFFSET_M
    for _ in range(200):
        p, v = _traffic(rng, 12)
        # Pairs exactly on (and one ulp around) the distance threshold, some with r . v = 0.
        for k in range(1, 5):
            d = rng.normal(size=3)
            p[k] = p[0] + d / np.linalg.norm(d) * limit * (1.0 + rng.choice([0.0, 2e-16, -2e-16]))
            if k % 2:
                v[k] = v[0]
        # A pair whose predicted miss at t* lies on the threshold: r = m - v t*
        # with m perpendicular to v, so t* is the chosen one and |r + v t*| = |m|.
        w = rng.normal(size=3) * (1.0, 1.0, 0.2)  # mostly level, so the pair stays in the box
        w *= rng.uniform(1.0, 10.0) / np.linalg.norm(w)
        m = np.cross(w, rng.normal(size=3))
        m *= limit * (1.0 + rng.choice([0.0, 2e-16, -2e-16, 4e-16])) / np.linalg.norm(m)
        t = rng.choice([0.1, 3.0, rng.uniform(0.1, 3.0)])
        v[6], v[7] = w / 2.0, -w / 2.0
        p[7] = p[6] - (m - w * t)
        # A pair with r perpendicular to v beyond the limit (the release test r . v >= 0).
        r = np.cross(v[8] - v[9], rng.normal(size=3))
        p[9] = p[8] - r * (limit * 1.5 / np.linalg.norm(r))
        states, _, pairs = _maps(p, v)
        inside, apart = F._conflict_screen(states, pairs, limit)
        for k, (a, b) in enumerate(pairs):
            r, w, _, t = F._relative(states[a], states[b])
            assert inside[k] == (min(float(np.linalg.norm(r)), float(np.linalg.norm(r + w * t))) < limit)
            assert apart[k] == (float(r @ w) >= 0.0)


def _dense_row_build(states, actions, pairs, config, escape_signs, backup, frames):
    """The pair rows of build_hocbf_qp_matrices before perf/scale: one dense row per pair."""
    agent_ids = sorted(states)
    idx = {a: k for k, a in enumerate(agent_ids)}
    dim = 3 * len(agent_ids)
    nom = np.concatenate([actions[a].proposed_accel_mps2 for a in agent_ids]).astype(np.float64)
    target = nom.copy()
    escape_signs = escape_signs or {}
    if config.barrier == "backup" and backup is None:
        backup = F._backup_constraints(states, pairs, config, frames)
    rows, lower = [], []
    for k, (i, j) in enumerate(pairs):
        row = np.zeros(dim)
        if (i, j) in escape_signs:
            coef, rhs = F._escape_constraint(states[i], states[j], escape_signs[(i, j)], config, (frames or {}).get((i, j)))
        elif config.barrier == "backup":
            coef, rhs = backup[0][k], float(backup[1][k])
        else:
            coef, rhs, _ = F._pair_constraint(states[i], states[j], config)
            row[3 * idx[i]:3 * idx[i] + 3], row[3 * idx[j]:3 * idx[j] + 3] = coef, -coef
            if config.tie_break_mps2 > 0.0 and float(row @ nom) < rhs:
                lateral = config.tie_break_mps2 * F._frame_lateral(coef, F._latch_axis(frames, (i, j)))
                target[3 * idx[i]:3 * idx[i] + 3] += lateral
                target[3 * idx[j]:3 * idx[j] + 3] -= lateral
        row[3 * idx[i]:3 * idx[i] + 3], row[3 * idx[j]:3 * idx[j] + 3] = coef, -coef
        rows.append(row)
        lower.append(rhs)
    return sp.csc_matrix(np.vstack(rows)), np.array(lower), target


@pytest.mark.parametrize("barrier", ["backup", "braking"])
def test_sparse_build_matches_the_dense_row_build(barrier) -> None:
    rng = np.random.default_rng(1)
    config = F.FilterConfig(barrier=barrier)
    for trial in range(60):
        n = int(rng.integers(2, 10))
        p, v = _traffic(rng, n)
        if trial % 3 == 0:
            p[1] = p[0] + (0.0, 0.0, 8.0)  # vertical pair: exact zeros in its row
        states, actions, pairs = _maps(p, v, rng)
        frames = F._update_latches(states, pairs, config, {})
        escape = ({pair: (float(rng.choice([-1, 0, 1])), float(rng.choice([-1, 0, 1]))) for pair in pairs
                   if rng.random() < 0.3} if trial % 2 else None)
        _, q, A, l, u, _ = F.build_hocbf_qp_matrices(states, actions, pairs, config, escape, None, frames)
        rows, lower, target = _dense_row_build(states, actions, pairs, config, escape, None, frames)
        m, dim = len(pairs), 3 * n
        # The whole stacked A that OSQP consumes: pair rows | slack, box, slack >= 0.
        want = sp.vstack([sp.hstack([rows, sp.eye(m)]), sp.hstack([sp.eye(dim), sp.csc_matrix((dim, m))]),
                          sp.hstack([sp.csc_matrix((m, dim)), sp.eye(m)])], format="csc")
        assert A.shape == want.shape and A.data.tobytes() == want.data.tobytes()
        assert np.array_equal(A.indices, want.indices) and np.array_equal(A.indptr, want.indptr)
        low, high = (x.ravel() for x in F._world_accel_limits(np.array([states[a].position_m for a in sorted(states)]),
                                                                np.array([states[a].velocity_mps for a in sorted(states)]),
                                                                config))
        assert l.tobytes() == np.concatenate([lower, low, np.zeros(m)]).tobytes()
        assert u.tobytes() == np.concatenate([np.full(m, np.inf), high, np.full(m, np.inf)]).tobytes()
        assert q[:dim].tobytes() == (-2.0 * target).tobytes()


@pytest.mark.parametrize("barrier", ["backup", "braking"])
def test_threaded_rollout_gives_the_same_closed_loop(monkeypatch, barrier) -> None:
    # 24 drones (276 pairs, above _PARALLEL_MIN_PAIRS) converging with random
    # nominals, 10 steps of the fuzz.py loop: actions bit for bit equal.
    def fly(min_pairs):
        monkeypatch.setattr(F, "_PARALLEL_MIN_PAIRS", min_pairs)
        rng = np.random.default_rng(7)
        p, v = _traffic(rng, 24, spread=80.0)
        v = -(p - p.mean(axis=0)) / np.linalg.norm(p - p.mean(axis=0), axis=1, keepdims=True) * rng.uniform(3, 12, (24, 1))
        config, latch, out = F.FilterConfig(barrier=barrier), {}, []
        for k in range(10):
            states, actions, _ = _maps(p, v, rng)
            states = {a: s.model_copy(update={"timestamp_ms": 100 * k}) for a, s in states.items()}
            actions = {a: s.model_copy(update={"timestamp_ms": 100 * k}) for a, s in actions.items()}
            safe, _ = F.apply_filter(list(states.values()), list(actions.values()), config, latch=latch)
            a = np.array([s.safe_accel_mps2 for s in sorted(safe, key=lambda s: s.agent_id)])
            out.append(a)
            w = v + a * 0.1
            speed = np.linalg.norm(w, axis=1, keepdims=True)
            w = np.where(speed > MAX_SPEED_MPS, w * (MAX_SPEED_MPS / speed * (1 - 1e-12)), w)
            p, v = p + 0.5 * (v + w) * 0.1, w
        return np.array(out)

    assert fly(10**9).tobytes() == fly(0).tobytes()


def test_independent_problems_have_the_joint_optimum() -> None:
    # Two or three encounters in one sky: rows across them are dropped, and the
    # split solves agree with the joint QP within the KKT certificate.
    rng = np.random.default_rng(2)
    config = F.FilterConfig()
    split = 0
    for _ in range(40):
        p, v = _traffic(rng, 9, spread=40.0)
        p[3:6] += (150.0, 0.0, 0.0)
        p[6:] += (0.0, 150.0, 0.0)
        states, actions, pairs = _maps(p, v, rng)
        frames = F._update_latches(states, pairs, config, {})
        backup = F._backup_constraints(states, pairs, config, frames)
        problems = F._independent_problems(states, actions, pairs, config, None, backup, frames)
        split += len(problems) > 1
        joint = F._solve_one_qp(states, actions, pairs, config, None, backup, frames)
        parts, slack, ok = F._solve_hocbf_qp(states, actions, pairs, config, backup=backup, frames=frames)
        assert ok and joint is not None
        for aid in states:
            assert np.max(np.abs(np.subtract(parts[aid], joint[0][aid]))) <= 1e-7
        assert set(slack) == set(joint[1])
        # A dropped row holds for every acceleration in the box (sampled at its corners).
        kept = {int(k) for _, rows in problems for k in rows}
        ids = sorted(states)
        low, high = F._world_accel_limits(np.array([states[a].position_m for a in ids]),
                                          np.array([states[a].velocity_mps for a in ids]), config)
        index = {a: k for k, a in enumerate(ids)}
        samples = np.where(rng.uniform(size=(200, len(ids), 3)) < 0.5, low, high)
        for k, (a, b) in enumerate(pairs):
            if k not in kept:
                assert np.all((samples[:, index[a]] - samples[:, index[b]]) @ backup[0][k] >= backup[1][k])
    assert split > 0
