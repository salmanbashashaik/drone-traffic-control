"""PR #12 review B1 and B2: world bounds and vertical line of sight.

B1: the escape, backup and fallback manoeuvres must never push a drone out of
the world box (x, y in [0, 1000] m, z in [30, 200] m).
B2: when the line of sight is vertical or near vertical, the sideways escape
direction must be defined and must not swing from step to step.
"""

import functools

import numpy as np
import pytest
from shared.contracts import MAX_ACCEL_COMPONENT_MPS2, SAFE_SEPARATION_M, DroneState, MARLAction, ScenarioConfig

from utm_services.policy.reference import compute_reference_action
from utm_services.safety import filter as filter_module
from utm_services.safety.filter import FilterConfig
from utm_services.simulation.runner import run_episode

BARRIERS = ("backup", "braking")
D_EFF = SAFE_SEPARATION_M + FilterConfig().margin_m


class _BrokenOSQP:
    def setup(self, *args, **kwargs):
        pass

    def solve(self):
        raise RuntimeError("simulated solver crash")


def _drone(agent_id, pos, goal, vel):
    return {"agent_id": agent_id, "initial_position_m": tuple(map(float, pos)),
            "initial_velocity_mps": tuple(map(float, vel)), "goal_m": tuple(map(float, goal))}


def _run(drones, barrier, steps=400):
    cfg = ScenarioConfig(scenario_id="bounds-vertical", seed=0, episode_limit_steps=steps, drones=drones)
    return run_episode(cfg, "shielded_marl", policy=compute_reference_action, policy_id="reference-standin",
                       filter_config=FilterConfig(barrier=barrier), keep_records=False).summary


def _head_on_at(z0):
    return [_drone("a", (480, 500, z0), (900, 500, z0), (15, 0, 0)),
            _drone("b", (520, 500, z0), (100, 500, z0), (-15, 0, 0))]


# ---------------------------------------------------------------------------
# B1: world bounds
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("barrier", BARRIERS)
@pytest.mark.parametrize("z0", [30.5, 35.0, 40.0, 195.0, 199.5])
def test_head_on_near_floor_or_ceiling_stays_in_the_world(z0, barrier) -> None:
    s = _run(_head_on_at(z0), barrier)

    assert s["n_aborted"] == 0
    assert s["min_separation_m"] >= SAFE_SEPARATION_M


@pytest.mark.parametrize("z0", [30.5, 35.0, 40.0, 195.0, 199.5])
def test_forced_fallback_head_on_near_floor_or_ceiling_stays_in_the_world(z0, monkeypatch) -> None:
    monkeypatch.setattr(filter_module.osqp, "OSQP", _BrokenOSQP)
    s = _run(_head_on_at(z0), "backup")

    assert s["fallback_events"] > 0
    assert s["n_aborted"] == 0


@pytest.mark.parametrize("barrier", BARRIERS)
@pytest.mark.parametrize("y0", [0.5, 999.5])
def test_head_on_along_a_world_edge_stays_in_the_world(y0, barrier) -> None:
    drones = [_drone("a", (480, y0, 100), (900, y0, 100), (15, 0, 0)),
              _drone("b", (520, y0, 100), (100, y0, 100), (-15, 0, 0))]
    s = _run(drones, barrier)

    assert s["n_aborted"] == 0


def _push(direction):
    d = np.array(direction, dtype=float)
    accel = tuple(float(c) for c in d * MAX_ACCEL_COMPONENT_MPS2 / np.max(np.abs(d)))
    return lambda obs, agent_id, timestamp_ms: MARLAction(
        agent_id=agent_id, proposed_accel_mps2=accel, timestamp_ms=timestamp_ms)


@pytest.mark.parametrize("pos, direction", [((500, 500, 80), (0, 0, -1)), ((20, 20, 50), (-1, -1, -1))],
                         ids=["dive-to-floor", "diagonal-into-corner"])
def test_single_drone_pushed_into_a_wall_stays_in_the_world(pos, direction) -> None:
    # PR #12 re-review: with no pair in the QP the nominal skipped the world limits.
    vel = 15.0 * 0.9999 * np.array(direction) / np.linalg.norm(direction)
    cfg = ScenarioConfig(scenario_id="single-wall", seed=0, episode_limit_steps=150,
                         drones=[_drone("a", pos, (500, 500, 100), vel)])
    s = run_episode(cfg, "shielded_marl", policy=_push(direction), policy_id="push-to-wall",
                    keep_records=False).summary

    assert s["n_aborted"] == 0


def test_filter_config_rejects_a_margin_that_crosses_the_world_limits() -> None:
    # 170 m / 2 - ((15 + 1)^2 - 1) / (2 * 3.5) = 48.57 m at the defaults.
    FilterConfig(margin_m=48.5)
    with pytest.raises(ValueError, match="margin_m"):
        FilterConfig(margin_m=48.6)
    with pytest.raises(ValueError, match="margin_m"):
        FilterConfig(margin_m=31.5, barrier_offset_mps=5.0)


def test_filter_config_rejects_gamma_below_one() -> None:
    # Below 1 the world limits cross even at margin 0 (gamma 0.1, c 0.1).
    FilterConfig(gamma=1.0)
    with pytest.raises(ValueError, match="gamma"):
        FilterConfig(gamma=0.99)


def _crossed_config():
    # Bypasses __post_init__: margin 70 m crosses the z limits at z = 75 m climbing at 15 m/s.
    cfg = FilterConfig()
    object.__setattr__(cfg, "margin_m", 70.0)
    return cfg


def test_crossed_world_limits_keep_the_nearer_wall_and_are_flagged() -> None:
    states = [DroneState(agent_id="a", position_m=(500.0, 500.0, 75.0), velocity_mps=(0.0, 0.0, 15.0),
                         goal_m=(500.0, 500.0, 100.0), timestamp_ms=0),
              DroneState(agent_id="b", position_m=(600.0, 500.0, 115.0), velocity_mps=(0.0, 0.0, 0.0),
                         goal_m=(500.0, 500.0, 100.0), timestamp_ms=0)]
    actions = [MARLAction(agent_id=s.agent_id, proposed_accel_mps2=(0.0, 0.0, -3.5), timestamp_ms=0) for s in states]
    safe, diag = filter_module.apply_filter(states, actions, _crossed_config(), latch={})
    by_id = {a.agent_id: a for a in safe}

    assert diag.qp_feasible and not diag.fallback_used
    assert diag.world_limit_conflict_agents == ["a"]
    assert by_id["a"].world_limit_conflict and not by_id["b"].world_limit_conflict
    # The floor (45 m away) is nearer than the ceiling (125 m): its limit wins.
    assert by_id["a"].safe_accel_mps2[2] == pytest.approx(MAX_ACCEL_COMPONENT_MPS2, abs=1e-3)


@pytest.mark.parametrize("n_drones", [1, 2])
def test_crossed_world_limits_keep_the_drone_in_the_world(n_drones) -> None:
    drones = [_drone("a", (500, 500, 75), (500, 500, 190), (0, 0, 15)),
              _drone("b", (600, 500, 115), (600, 500, 40), (0, 0, -15))][:n_drones]
    cfg = ScenarioConfig(scenario_id="crossed-limits", seed=0, episode_limit_steps=300, drones=drones)
    s = run_episode(cfg, "shielded_marl", policy=compute_reference_action, policy_id="reference-standin",
                    filter_config=_crossed_config(), keep_records=False).summary

    assert s["n_aborted"] == 0
    assert s["fallback_events"] == 0
    assert s["world_limit_conflicts"] > 0


# Unrecoverable by any controller (best achievable 4.18-4.45 m). Non-regression floor,
# not a safety claim. See BL-06.
# Measured at PR #12 (backup / braking): floor 3.063 / 3.061 m, ceiling 3.201 /
# 3.201 m, vertical 2.668 / 2.744 m; each floor is 0.05 m below, rounded down to
# the centimetre, so a solver or BLAS change cannot break it.
def _sandwich_at(z0):
    return [_drone("d1", (490, 500, z0), (900, 500, z0), (10, 0, 0)), _drone("d2", (500, 500, z0), (500, 900, z0), (0, 0, 0)),
            _drone("d3", (510, 500, z0), (100, 500, z0), (-10, 0, 0))]


_VERTICAL_SANDWICH = [_drone("a", (500, 500, 90), (500, 500, 190), (0, 0, 10)),
                      _drone("b", (500, 500, 100), (900, 500, 100), (0, 0, 0)),
                      _drone("c", (500, 500, 110), (500, 500, 35), (0, 0, -10))]


@pytest.mark.parametrize(("drones", "barrier", "floor_m"), [
    (_sandwich_at(30.5), "backup", 3.01), (_sandwich_at(30.5), "braking", 3.01),
    (_sandwich_at(199.5), "backup", 3.15), (_sandwich_at(199.5), "braking", 3.15),
    (_VERTICAL_SANDWICH, "backup", 2.61), (_VERTICAL_SANDWICH, "braking", 2.69),
], ids=["floor-backup", "floor-braking", "ceiling-backup", "ceiling-braking", "vertical-backup", "vertical-braking"])
def test_sandwich_keeps_its_regression_floor(drones, barrier, floor_m) -> None:
    s = _run(drones, barrier)

    assert s["min_separation_m"] >= floor_m
    assert s["fallback_events"] == 0
    assert s["n_aborted"] == 0


# ---------------------------------------------------------------------------
# B2: vertical and near-vertical line of sight
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("barrier", BARRIERS)
def test_vertical_head_on_keeps_d_eff(barrier) -> None:
    # Braking alone needs about 34 m at 20 m/s closing; the drones start 60 m apart.
    drones = [_drone("a", (500, 500, 60), (500, 500, 190), (0, 0, 10)),
              _drone("b", (500, 500, 120), (500, 500, 35), (0, 0, -10))]
    s = _run(drones, barrier, steps=200)

    assert s["min_separation_m"] >= D_EFF
    assert s["n_aborted"] == 0


def _noisy_head_on(p0, v0, sigma, barrier, seed, steps=120):
    """Closed loop with position noise on what the filter sees, one latch per run.

    Returns (min separation, sideways direction reversals): a reversal is a
    step whose sideways relative acceleration (perpendicular to the initial
    line of sight, above 0.5 m/s²) points against the last one.
    """
    rng = np.random.default_rng(seed)
    p, v = np.array(p0, dtype=float), np.array(v0, dtype=float)
    axis = (p[0] - p[1]) / np.linalg.norm(p[0] - p[1])
    cfg, latch = FilterConfig(barrier=barrier), {}
    min_sep, reversals, prev = np.inf, 0, None
    for k in range(steps):
        obs = p + rng.normal(0.0, sigma, p.shape)
        states = [DroneState(agent_id=f"d{i}", position_m=tuple(obs[i]), velocity_mps=tuple(v[i]),
                             goal_m=(500.0, 500.0, 100.0), timestamp_ms=k * 100) for i in range(2)]
        actions = [MARLAction(agent_id=f"d{i}", proposed_accel_mps2=(0.0, 0.0, 0.0), timestamp_ms=k * 100)
                   for i in range(2)]
        safe, _ = filter_module.apply_filter(states, actions, cfg, latch=latch)
        a = np.array([s.safe_accel_mps2 for s in sorted(safe, key=lambda s: s.agent_id)])
        side = (a[0] - a[1]) - float((a[0] - a[1]) @ axis) * axis
        if np.linalg.norm(side) > 0.5:
            reversals += prev is not None and float(side @ prev) < 0.0
            prev = side
        v_next = v + a * 0.1
        v_next *= np.minimum(1.0, 15.0 / np.maximum(np.linalg.norm(v_next, axis=1, keepdims=True), 1e-9))
        p, v = p + 0.5 * (v + v_next) * 0.1, v_next
        min_sep = min(min_sep, float(np.linalg.norm(p[0] - p[1])))
    return min_sep, reversals


@functools.cache
def _noise_rows(sigma, barrier):
    # PR #12 re-review: noise on the horizontal offset swung the sideways
    # direction every few steps. Horizontal head-on is the control.
    vertical = [_noisy_head_on([(500, 500, 70), (500, 500, 130)], [(0, 0, 10), (0, 0, -10)], sigma, barrier, s)
                for s in range(20)]
    horizontal = [_noisy_head_on([(470, 500, 100), (530, 500, 100)], [(10, 0, 0), (-10, 0, 0)], sigma, barrier, s)
                  for s in range(20)]
    print(f"\n{barrier} sigma={sigma}: vertical min_sep worst {min(m for m, _ in vertical):.3f} "
          f"below d_eff {sum(m < D_EFF for m, _ in vertical)}/20; horizontal worst "
          f"{min(m for m, _ in horizontal):.3f} below d_eff {sum(m < D_EFF for m, _ in horizontal)}/20")
    return vertical, horizontal


@pytest.mark.parametrize(("sigma", "barrier"), [
    (sigma, barrier) if (sigma, barrier) != (1.5, "backup") else pytest.param(sigma, barrier, marks=pytest.mark.xfail(
        strict=True, reason="BL-06 / UTM-012: at sigma 1.5 the noisy miss sign sets the latch at conflict "
                            "entry; needs tracked (Kalman) state"))
    for sigma in (0.0, 0.5, 1.5) for barrier in BARRIERS
])
def test_vertical_head_on_under_noise_keeps_one_sideways_direction(sigma, barrier) -> None:
    vertical, horizontal = _noise_rows(sigma, barrier)

    assert max(r for _, r in vertical) == 0
    assert max(r for _, r in horizontal) == 0


@pytest.mark.parametrize("barrier", BARRIERS)
@pytest.mark.parametrize("sigma", [0.0, 0.5, 1.5])
def test_vertical_head_on_under_noise_no_worse_than_horizontal(sigma, barrier) -> None:
    # Compared by runs below d_eff: the worst minimum differs at sigma 0 from
    # geometry alone (vertical escapes sideways only), all above d_eff.
    vertical, horizontal = _noise_rows(sigma, barrier)

    assert sum(m < D_EFF for m, _ in vertical) <= sum(m < D_EFF for m, _ in horizontal)


@pytest.mark.parametrize("barrier", BARRIERS)
@pytest.mark.parametrize("elevation_deg", np.arange(80.0, 90.01, 0.5).tolist())
def test_near_vertical_head_on_keeps_d_eff(elevation_deg, barrier) -> None:
    # 80 m apart, 20 m/s closing along a line of sight elevation_deg above horizontal: recoverable.
    centre = np.array([500.0, 500.0, 110.0])
    e = np.radians(elevation_deg)
    u = np.array([np.cos(e), 0.0, np.sin(e)])
    drones = [_drone("a", centre - 40 * u, centre + 70 * u, 10 * u),
              _drone("b", centre + 40 * u, centre - 70 * u, -10 * u)]
    s = _run(drones, barrier, steps=250)

    assert s["min_separation_m"] >= D_EFF
    assert s["n_aborted"] == 0
