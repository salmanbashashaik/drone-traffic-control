"""The Numba backup rollout matches the numpy rollout it replaced, bit for bit.

``_numpy_rollout`` is the numpy ``_backup_rollout`` as it was before the
kernel (PR 2 of the verification speed-up), kept here as the oracle. The
inputs cover open air, the world-box walls (clipped evasive), near-vertical
and exactly vertical lines of sight (offset and plumb rules), latched axes
(including ones near the line of sight, which fall back to ``_lateral``) and
latched z directions.
"""

import numpy as np
import pytest
from shared.contracts import MAX_SPEED_MPS, SIMULATION_STEP_MS

from utm_services.safety import filter as F


def _numpy_rollout(p_i, p_j, v_i, v_j, config, axis=None, up_latched=None):
    dt = SIMULATION_STEP_MS / 1000.0
    min_steps = F._backup_steps(config)
    steps = max(min_steps, F._backup_max_steps(config))
    used = np.full(len(p_i), steps)
    still = np.ones(len(p_i), dtype=bool)
    closing = np.zeros(len(p_i))

    def cap(v):
        speed = np.linalg.norm(v, axis=1, keepdims=True)
        return v * np.minimum(1.0, MAX_SPEED_MPS / np.maximum(speed, 1e-9))

    traj = np.full((steps, len(p_i), 3), np.nan)
    for k in range(steps):
        r = p_i - p_j
        n = r / np.maximum(np.linalg.norm(r, axis=1, keepdims=True), 1e-9)
        up = np.where(np.abs(n[:, 2]) > 1e-3, np.sign(n[:, 2]), 1.0)
        if up_latched is not None:
            up = np.where(up_latched != 0.0, up_latched, up)
        a_i = F._evasive_direction(n, up, F._world_accel_limits(p_i, v_i, config), axis)
        a_j = F._evasive_direction(-n, -up, F._world_accel_limits(p_j, v_j, config), axis)
        if k == 0:
            a_first = a_i - a_j
        v_i_next, v_j_next = cap(v_i + a_i * dt), cap(v_j + a_j * dt)
        p_i = p_i + 0.5 * (v_i + v_i_next) * dt
        p_j = p_j + 0.5 * (v_j + v_j_next) * dt
        v_i, v_j = v_i_next, v_j_next
        r = p_i - p_j
        traj[k, still] = r[still]
        rate = -np.einsum("ij,ij->i", r, v_i - v_j) / np.maximum(np.linalg.norm(r, axis=1), 1e-9)
        closing[still] = rate[still]
        if k + 1 >= min_steps:
            done = still & (closing <= 0.0)
            used[done] = k + 1
            still &= ~done
            if not still.any():
                break
    return traj[: used.max(initial=0)], a_first, closing, used, still


def _unit(rng, count):
    u = rng.standard_normal((count, 3))
    return u / np.linalg.norm(u, axis=1, keepdims=True)


def _cases(seed, count=400):
    rng = np.random.default_rng(seed)
    lo, hi = F._WORLD_MIN_M, F._WORLD_MAX_M
    p_i = rng.uniform(lo, hi, (count, 3))
    wall = rng.random(count) < 0.4  # put drone i within 3 m of a random wall
    side = rng.integers(0, 2, count)
    dim = rng.integers(0, 3, count)
    gap = rng.uniform(0.0, 3.0, count)
    p_i[wall, dim[wall]] = np.where(side[wall] == 0, lo[dim[wall]] + gap[wall], hi[dim[wall]] - gap[wall])
    u = _unit(rng, count)
    vertical = rng.random(count) < 0.25  # near-vertical lines of sight, some exactly vertical
    u[vertical, :2] *= rng.choice([0.0, 1e-12, 1e-6, 0.1, 0.25, 0.4], (vertical.sum(), 1))
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    p_j = p_i + rng.uniform(1.0, 80.0, (count, 1)) * u
    speed = rng.uniform(0.0, MAX_SPEED_MPS, (count, 1))
    toward = rng.random((count, 1)) < 0.6
    v_i = np.where(toward, -u, _unit(rng, count)) * speed
    v_j = np.where(toward, u, _unit(rng, count)) * rng.uniform(0.0, MAX_SPEED_MPS, (count, 1))
    axis = np.where(rng.random((count, 1)) < 0.5, 0.0, _unit(rng, count))
    near_n = rng.random(count) < 0.1  # axes within 30 degrees of n fall back to _lateral
    axis[near_n] = u[near_n]
    up = rng.choice([0.0, 1.0, -1.0], count)
    return p_i, p_j, v_i, v_j, axis, up


def _assert_same(got, want):
    for g, w in zip(got, want, strict=True):
        assert g.shape == w.shape
        assert g.dtype == w.dtype
        assert np.array_equal(g, w, equal_nan=True)


@pytest.mark.parametrize("config", [F.FilterConfig(), F.FilterConfig(lag_s=0.3)], ids=["default", "lag"])
@pytest.mark.parametrize("seed", range(4))
def test_kernel_matches_numpy_rollout(seed, config) -> None:
    p_i, p_j, v_i, v_j, axis, up = _cases(seed)

    _assert_same(F._backup_rollout(p_i, p_j, v_i, v_j, config, axis, up),
                 _numpy_rollout(p_i, p_j, v_i, v_j, config, axis, up))


@pytest.mark.parametrize("cap", [10, 40])
def test_kernel_matches_numpy_rollout_past_the_minimum_and_at_the_cap(monkeypatch, cap) -> None:
    # At the real 66-step minimum these cases never extend or cap; a short
    # minimum makes many pairs still close there.
    monkeypatch.setattr(F, "_backup_steps", lambda config: 5)
    monkeypatch.setattr(F, "_backup_max_steps", lambda config: cap)
    p_i, p_j, v_i, v_j, axis, up = _cases(20)

    got = F._backup_rollout(p_i, p_j, v_i, v_j, F.FilterConfig(), axis, up)
    _assert_same(got, _numpy_rollout(p_i, p_j, v_i, v_j, F.FilterConfig(), axis, up))
    used, still = got[3], got[4]
    assert ((used > 5) & ~still).any()
    assert still.any() == (cap == 10)  # at 40 every pair stops closing before the cap


def test_kernel_matches_numpy_rollout_without_latches() -> None:
    p_i, p_j, v_i, v_j, *_ = _cases(10)

    _assert_same(F._backup_rollout(p_i, p_j, v_i, v_j, F.FilterConfig()),
                 _numpy_rollout(p_i, p_j, v_i, v_j, F.FilterConfig()))


def test_kernel_matches_numpy_rollout_with_crossed_world_limits() -> None:
    # Far outside the box and fast, the raw limits cross and the nearer-wall
    # guard of _world_accel_limits acts; the in-box cases never reach it.
    rng = np.random.default_rng(30)
    p_i, p_j = rng.uniform(-1000.0, 1000.0, (2, 400, 3))
    v_i, v_j = rng.uniform(-300.0, 300.0, (2, 400, 3))
    for p, v in ((p_i, v_i), (p_j, v_j)):
        low, high = F._raw_world_limits(p, v, F.FilterConfig())
        assert (low > high).any()

    _assert_same(F._backup_rollout(p_i, p_j, v_i, v_j, F.FilterConfig()),
                 _numpy_rollout(p_i, p_j, v_i, v_j, F.FilterConfig()))


def test_rollout_rejects_inputs_the_kernel_cannot_check() -> None:
    p_i, p_j, v_i, v_j, axis, up = _cases(12, count=6)

    with pytest.raises(ValueError, match="needs"):
        F._backup_rollout(p_i, p_j[:5], v_i, v_j, F.FilterConfig())
    with pytest.raises(ValueError, match="needs"):
        F._backup_rollout(p_i, p_j, v_i, v_j, F.FilterConfig(), axis[:1], up)
    with pytest.raises(ValueError, match="needs"):
        F._backup_rollout(p_i[0], p_j[0], v_i[0], v_j[0], F.FilterConfig())
    up[2] = np.nan
    with pytest.raises(ValueError, match="finite"):
        F._backup_rollout(p_i, p_j, v_i, v_j, F.FilterConfig(), axis, up)


@pytest.mark.parametrize("count", [0, 1])
def test_kernel_matches_numpy_rollout_small(count) -> None:
    p_i, p_j, v_i, v_j, axis, up = (x[:count] for x in _cases(11))

    _assert_same(F._backup_rollout(p_i, p_j, v_i, v_j, F.FilterConfig(), axis, up),
                 _numpy_rollout(p_i, p_j, v_i, v_j, F.FilterConfig(), axis, up))


@pytest.mark.parametrize("seed", range(3))
def test_parallel_kernel_matches_serial_kernel(monkeypatch, seed) -> None:
    # The 400-pair cases above run the parallel kernel (_PARALLEL_MIN_PAIRS); this pins both paths.
    p_i, p_j, v_i, v_j, axis, up = _cases(40 + seed)
    monkeypatch.setattr(F, "_PARALLEL_MIN_PAIRS", 10**9)
    serial = F._backup_rollout(p_i, p_j, v_i, v_j, F.FilterConfig(), axis, up)
    monkeypatch.setattr(F, "_PARALLEL_MIN_PAIRS", 0)

    _assert_same(F._backup_rollout(p_i, p_j, v_i, v_j, F.FilterConfig(), axis, up), serial)
