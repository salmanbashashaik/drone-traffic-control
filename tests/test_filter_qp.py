"""Unit and property tests for the active HOCBF QP safety filter (safety/filter.py).

Verifies the mathematical correctness of the QP formulation, matrix construction,
active collision interventions and pass-through conditions. Slack and
fallback semantics live in test_filter_safety_core.py.
"""

import math

import numpy as np
from shared.contracts import (
    MAX_ACCEL_COMPONENT_MPS2,
    DroneState,
    MARLAction,
)

from utm_services.safety.filter import (
    FilterConfig,
    apply_filter,
    build_hocbf_qp_matrices,
)

# Tests of the braking-barrier geometry (tie-break, braking curve, lag
# reserve) pin it; the default is the backup barrier since contract v1.3.
BRAKING = FilterConfig(barrier="braking")


def _state(
    agent_id: str,
    pos=(100.0, 100.0, 80.0),
    vel=(0.0, 0.0, 0.0),
    goal=(900.0, 900.0, 80.0),
    ts=0,
) -> DroneState:
    return DroneState(
        agent_id=agent_id,
        position_m=pos,
        velocity_mps=vel,
        goal_m=goal,
        timestamp_ms=ts,
    )


def _action(agent_id: str, accel=(0.0, 0.0, 0.0), ts=0) -> MARLAction:
    return MARLAction(
        agent_id=agent_id,
        proposed_accel_mps2=accel,
        timestamp_ms=ts,
    )


# ---------------------------------------------------------------------------
# Matrix builder tests
# ---------------------------------------------------------------------------

def test_build_matrices_shapes_and_bounds() -> None:
    states = {
        "d1": _state("d1", pos=(100.0, 100.0, 80.0)),
        "d2": _state("d2", pos=(120.0, 100.0, 80.0)),
    }
    actions = {
        "d1": _action("d1", accel=(1.0, 0.0, 0.0)),
        "d2": _action("d2", accel=(-1.0, 0.0, 0.0)),
    }
    pairs = [("d1", "d2")]
    cfg = FilterConfig()

    P, q, A, l, u, agent_ids = build_hocbf_qp_matrices(states, actions, pairs, cfg)

    assert agent_ids == ["d1", "d2"]
    # 6 acceleration variables + 1 slack variable per pair
    assert P.shape == (7, 7)
    assert q.shape == (7,)
    # 1 pair constraint + 6 box bound constraints + 1 slack >= 0 row = 8 rows
    assert A.shape == (8, 7)
    assert len(l) == 8
    assert len(u) == 8

    # Box bounds on acceleration variables:
    assert np.allclose(l[1:7], -MAX_ACCEL_COMPONENT_MPS2)
    assert np.allclose(u[1:7], MAX_ACCEL_COMPONENT_MPS2)
    # Slack is non-negative and unbounded above:
    assert l[7] == 0.0
    assert np.isinf(u[7])


# ---------------------------------------------------------------------------
# Head-on collision avoidance & interventions
# ---------------------------------------------------------------------------

def test_qp_intervenes_on_head_on_converging_pair() -> None:
    # Two drones 20m apart closing at 10 m/s (5 m/s each).
    # Nominal actions want them to maintain speed (zero accel).
    # HOCBF must intervene to push them apart (d1 accelerates in -x, d2 in +x).
    states = [
        _state("d1", pos=(400.0, 500.0, 100.0), vel=(5.0, 0.0, 0.0)),
        _state("d2", pos=(420.0, 500.0, 100.0), vel=(-5.0, 0.0, 0.0)),
    ]
    actions = [
        _action("d1", accel=(0.0, 0.0, 0.0)),
        _action("d2", accel=(0.0, 0.0, 0.0)),
    ]

    safe_actions, diag = apply_filter(states, actions, BRAKING)

    assert diag.qp_feasible is True
    assert diag.fallback_used is False
    assert diag.interventions == 2
    assert len(safe_actions) == 2

    a1 = safe_actions[0].safe_accel_mps2
    a2 = safe_actions[1].safe_accel_mps2

    # d1 must brake (negative x acceleration)
    assert a1[0] < -0.1
    # d2 must brake (positive x acceleration)
    assert a2[0] > 0.1

    # Symmetry in deceleration
    assert abs(a1[0] + a2[0]) < 1e-3
    # Tie-break: both turn right (d1 heading +x -> -y, d2 heading -x -> +y)
    assert a1[1] < -0.1 and abs(a1[1] + a2[1]) < 1e-3
    assert abs(a1[2]) < 1e-4 and abs(a2[2]) < 1e-4

    # Both must be within control authority
    assert abs(a1[0]) <= MAX_ACCEL_COMPONENT_MPS2
    assert abs(a2[0]) <= MAX_ACCEL_COMPONENT_MPS2


def test_qp_passes_through_diverging_pair() -> None:
    # Two drones 20m apart flying away from each other at 5 m/s.
    # Nominal actions should pass through unchanged without intervention.
    states = [
        _state("d1", pos=(400.0, 500.0, 100.0), vel=(-5.0, 0.0, 0.0)),
        _state("d2", pos=(420.0, 500.0, 100.0), vel=(5.0, 0.0, 0.0)),
    ]
    actions = [
        _action("d1", accel=(-0.5, 0.0, 0.0)),
        _action("d2", accel=(0.5, 0.0, 0.0)),
    ]

    safe_actions, diag = apply_filter(states, actions)

    assert diag.qp_feasible is True
    assert diag.fallback_used is False
    assert diag.interventions == 0

    assert np.allclose(safe_actions[0].safe_accel_mps2, (-0.5, 0.0, 0.0), atol=1e-3)
    assert np.allclose(safe_actions[1].safe_accel_mps2, (0.5, 0.0, 0.0), atol=1e-3)


def test_qp_passes_through_distant_stationary_pair() -> None:
    # Two drones 100m apart, zero velocity.
    states = [
        _state("d1", pos=(400.0, 500.0, 100.0), vel=(0.0, 0.0, 0.0)),
        _state("d2", pos=(500.0, 500.0, 100.0), vel=(0.0, 0.0, 0.0)),
    ]
    actions = [
        _action("d1", accel=(1.5, -0.5, 0.0)),
        _action("d2", accel=(-1.5, 0.5, 0.0)),
    ]

    safe_actions, diag = apply_filter(states, actions)

    assert diag.qp_feasible is True
    assert diag.interventions == 0
    assert np.allclose(safe_actions[0].safe_accel_mps2, (1.5, -0.5, 0.0), atol=1e-3)
    assert np.allclose(safe_actions[1].safe_accel_mps2, (-1.5, 0.5, 0.0), atol=1e-3)


# ---------------------------------------------------------------------------
# Multi-agent symmetric convergence
# ---------------------------------------------------------------------------

def test_qp_three_drone_symmetric_convergence() -> None:
    # 3 drones starting 25m from center (500, 500, 100), heading directly inward at 12 m/s
    # (pairwise closing ≈ 20.8 m/s, inside the braking curve so the barrier binds).
    center_x = 500.0
    center_y = 500.0
    r = 25.0
    v = 12.0
    angles = [0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0]

    states = []
    actions = []
    for idx, angle in enumerate(angles):
        aid = f"d{idx+1}"
        pos = (center_x + r * math.cos(angle), center_y + r * math.sin(angle), 100.0)
        vel = (-v * math.cos(angle), -v * math.sin(angle), 0.0)
        states.append(_state(aid, pos=pos, vel=vel))
        actions.append(_action(aid, accel=(0.0, 0.0, 0.0)))

    safe_actions, diag = apply_filter(states, actions)

    assert diag.qp_feasible is True
    assert diag.interventions == 3

    # All drones should experience radial outward acceleration (opposite to inward velocity)
    for idx, sa in enumerate(safe_actions):
        acc = sa.safe_accel_mps2
        angle = angles[idx]
        inward_dir = np.array([-math.cos(angle), -math.sin(angle), 0.0])
        acc_vec = np.array(acc)
        # Dot product with inward direction must be negative (outward braking)
        assert np.dot(acc_vec, inward_dir) < -0.1


def test_lag_reserve_brakes_earlier() -> None:
    # 106 m apart, closing at 30 m/s: just outside the ideal braking curve
    # (the barrier offset c = 1 m/s moves it out to about 104 m).
    states = [
        _state("d1", pos=(447.0, 500.0, 100.0), vel=(15.0, 0.0, 0.0)),
        _state("d2", pos=(553.0, 500.0, 100.0), vel=(-15.0, 0.0, 0.0)),
    ]
    actions = [_action("d1"), _action("d2")]

    ideal, _ = apply_filter(states, actions, BRAKING)
    lagged, diag = apply_filter(states, actions, FilterConfig(barrier="braking", lag_s=0.5))

    assert ideal[0].shield_active is False
    assert diag.qp_feasible is True
    assert lagged[0].safe_accel_mps2[0] < -0.1


# ---------------------------------------------------------------------------
# Range gate behavior
# ---------------------------------------------------------------------------

def test_qp_range_gate_pruning_removes_pair_from_qp() -> None:
    # Two drones 80m apart, range gate is 50m.
    states = [
        _state("d1", pos=(400.0, 500.0, 100.0), vel=(5.0, 0.0, 0.0)),
        _state("d2", pos=(480.0, 500.0, 100.0), vel=(-5.0, 0.0, 0.0)),
    ]
    actions = [
        _action("d1", accel=(0.0, 0.0, 0.0)),
        _action("d2", accel=(0.0, 0.0, 0.0)),
    ]
    cfg = FilterConfig(range_gate_m=50.0)

    safe_actions, diag = apply_filter(states, actions, config=cfg)

    assert diag.n_pairs_total == 1
    assert diag.n_pairs_in_qp == 0
    assert diag.interventions == 0
    assert safe_actions[0].safe_accel_mps2 == (0.0, 0.0, 0.0)
