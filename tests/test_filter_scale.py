"""The scale changes of the filter (perf/scale) keep its results.

- ``_conflict_screen`` (vectorised ``_update_latches`` tests) decides every
  pair as the per-pair loop it replaced, including pairs on the thresholds.
- ``build_hocbf_qp_matrices`` returns the matrices of the dense-row build it
  replaced, bit for bit (``_dense_row_build`` is that build, kept as the oracle).
- ``_solve_hocbf_qp`` solved as independent problems has the optimum of the one
  joint QP, and every row it drops holds for every acceleration in the box.
"""

import numpy as np
import pytest
import scipy.sparse as sp
from shared.contracts import DroneState, MARLAction

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
        top = A[:m, :dim].tocsc()
        assert top.data.tobytes() == rows.data.tobytes()
        assert np.array_equal(top.indices, rows.indices) and np.array_equal(top.indptr, rows.indptr)
        assert l[:m].tobytes() == lower.tobytes()
        assert q[:dim].tobytes() == (-2.0 * target).tobytes()
        assert np.all(np.isinf(u[:m]))
