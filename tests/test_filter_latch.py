"""Tests of the per-pair escape latch (P2-SA-02 §5.1, PR #12 review B1).

A two-drone conflict with no clear miss latches turn-right; a cluster of
three or more drones is 2-coloured, seeded with the directions already
latched, so in collinear geometry a drone squeezed between two neighbours
moves one way and its neighbours the other way. When the neighbours are not
collinear the squeezed drone can still get contradictory latches (BL-10). A turn-right latch that contradicts the colouring
is re-oriented once, when its cluster reaches three drones, and never after.
"""

import math

import numpy as np
import pytest
from shared.contracts import DroneState, MARLAction

from utm_services.safety import filter as F

CFG = F.FilterConfig()
PAIRS_OF = lambda ids: [(a, b) for k, a in enumerate(ids) for b in ids[k + 1:]]


def _states(pos: dict, vel: dict | None = None, ts: int = 0) -> list[DroneState]:
    vel = vel or {}
    return [
        DroneState(
            agent_id=aid, position_m=tuple(p), velocity_mps=tuple(vel.get(aid, (0.0, 0.0, 0.0))),
            goal_m=(900.0, 900.0, 100.0), timestamp_ms=ts,
        )
        for aid, p in sorted(pos.items())
    ]


def _zero(states: list[DroneState]) -> list[MARLAction]:
    return [MARLAction(agent_id=s.agent_id, proposed_accel_mps2=(0.0, 0.0, 0.0), timestamp_ms=s.timestamp_ms)
            for s in states]


def _sideways(states: list[DroneState], latch: dict, pair: tuple[str, str], aid: str) -> np.ndarray:
    """World sideways direction that the latch of ``pair`` gives drone ``aid``."""
    pos = {s.agent_id: np.array(s.position_m) for s in states}
    n = pos[pair[0]] - pos[pair[1]]
    n /= np.linalg.norm(n)
    lateral = F._frame_lateral(n, latch[pair].axis)
    return lateral if aid == pair[0] else -lateral


def _colour(states: list[DroneState], latch: dict, pair: tuple[str, str], aid: str) -> float:
    """Colour of ``aid`` in ``pair``: +1 when its latch moves it along the pair's +L axis."""
    pos = {s.agent_id: np.array(s.position_m) for s in states}
    n = pos[pair[0]] - pos[pair[1]]
    n /= np.linalg.norm(n)
    return math.copysign(1.0, float(F._escape_axes(n)[0] @ _sideways(states, latch, pair, aid)))


def test_apply_filter_requires_a_latch() -> None:
    states = _states({"a": (100.0, 100.0, 80.0), "b": (300.0, 100.0, 80.0)})
    with pytest.raises(TypeError):
        F.apply_filter(states, _zero(states))  # type: ignore[call-arg]
    with pytest.raises(TypeError, match="UTM-077"):
        F.apply_filter(states, _zero(states), latch=None)  # type: ignore[arg-type]


def test_two_drone_head_on_latches_turn_right() -> None:
    states = _states({"a": (470.0, 500.0, 100.0), "b": (500.0, 500.0, 100.0)}, {"a": (10.0, 0.0, 0.0)})
    latch: dict = {}
    F.apply_filter(states, _zero(states), CFG, latch=latch)
    # a flies +x, so turning right is -y; b flies towards a, so its right is +y.
    np.testing.assert_allclose(_sideways(states, latch, ("a", "b"), "a"), (0.0, -1.0, 0.0), atol=1e-9)
    assert latch[("a", "b")].origin == "turn_right"


def test_three_drone_cluster_is_coloured() -> None:
    # a and c close on a hovering b from both sides at once.
    states = _states(
        {"a": (470.0, 500.0, 100.0), "b": (500.0, 500.0, 100.0), "c": (530.0, 500.0, 100.0)},
        {"a": (10.0, 0.0, 0.0), "c": (-10.0, 0.0, 0.0)},
    )
    latch: dict = {}
    F.apply_filter(states, _zero(states), CFG, latch=latch)
    b_ab = _sideways(states, latch, ("a", "b"), "b")
    np.testing.assert_allclose(_sideways(states, latch, ("b", "c"), "b"), b_ab, atol=1e-9)
    assert float(_sideways(states, latch, ("a", "b"), "a") @ b_ab) < -0.99
    assert latch[("a", "b")].origin == latch[("b", "c")].origin == "fixed"


def test_staged_sandwich_ends_with_consistent_latches() -> None:
    # b hovers; a arrives first and latches turn-right, c joins later
    # (scratchpad latch_dump.py of the PR #12 review). The squeezed drone b
    # must move one way towards both neighbours.
    p = np.array([[470.0, 500, 100], [500.0, 500, 100], [550.0, 500, 100]])
    v = np.array([[10.0, 0, 0], [0.0, 0, 0], [-10.0, 0, 0]])
    ids = ["a", "b", "c"]
    latch: dict = {}
    both = 0
    for k in range(60):
        states = _states(dict(zip(ids, p, strict=True)), dict(zip(ids, v, strict=True)), ts=k * 100)
        safe, _ = F.apply_filter(states, _zero(states), CFG, latch=latch)
        los = [(p[i] - p[j]) / np.linalg.norm(p[i] - p[j]) for i, j in ((0, 1), (1, 2))]
        # The side is defined while both lines of sight stay within 45 degrees of x.
        if ("a", "b") in latch and ("b", "c") in latch and min(abs(n[0]) for n in los) > math.sqrt(0.5):
            both += 1
            b_ab = _sideways(states, latch, ("a", "b"), "b")
            b_bc = _sideways(states, latch, ("b", "c"), "b")
            # Same side of the initial x line; the lines of sight turn as the drones move.
            assert b_ab[1] * b_bc[1] > 0.0, f"step {k}: b latched {b_ab} towards a, {b_bc} towards c"
        a = np.array([s.safe_accel_mps2 for s in sorted(safe, key=lambda s: s.agent_id)])
        vn = v + a * 0.1
        p, v = p + 0.5 * (v + vn) * 0.1, vn
    assert both > 0


@pytest.mark.xfail(strict=True, reason="BL-10: a far outer pair seeds the colouring before the inner pair "
                   "(founder-accepted known limit of PR #12, superseded by BL-03)")
@pytest.mark.parametrize("bend_deg", [10.0, 30.0])
def test_bent_staged_sandwich_ends_with_consistent_latches(bend_deg: float) -> None:
    # As the staged sandwich, but c arrives bend_deg off the line through a
    # and b. The outer pair a-c latches first, from its predicted miss, while
    # still about 70 m apart; the inner pair b-c must still decide b's side
    # (PR #12 B1 re-review HIGH-1).
    u_a = np.array([1.0, 0.0, 0.0])
    u_c = np.array([math.cos(math.radians(180.0 - bend_deg)), math.sin(math.radians(180.0 - bend_deg)), 0.0])
    b0 = np.array([500.0, 500.0, 100.0])
    p = np.array([b0 + 30.0 * u_a, b0, b0 + 50.0 * u_c])
    v = np.array([-10.0 * u_a, np.zeros(3), -10.0 * u_c])
    ids = ["a", "b", "c"]
    latch: dict = {}
    squeezed = 0
    min_sep = math.inf
    for k in range(90):
        states = _states(dict(zip(ids, p, strict=True)), dict(zip(ids, v, strict=True)), ts=k * 100)
        safe, _ = F.apply_filter(states, _zero(states), CFG, latch=latch)
        to_a, to_c = p[0] - p[1], p[2] - p[1]
        # b is squeezed while its neighbours are on roughly opposite sides of it.
        if ("a", "b") in latch and ("b", "c") in latch and \
                float(to_a @ to_c) / float(np.linalg.norm(to_a) * np.linalg.norm(to_c)) < -0.7:
            squeezed += 1
            b_ab = _sideways(states, latch, ("a", "b"), "b")
            b_bc = _sideways(states, latch, ("b", "c"), "b")
            assert float(b_ab @ b_bc) > 0.5, f"step {k}: b latched {b_ab} towards a, {b_bc} towards c"
        a = np.array([s.safe_accel_mps2 for s in sorted(safe, key=lambda s: s.agent_id)])
        vn = v + a * 0.1
        p, v = p + 0.5 * (v + vn) * 0.1, vn
        min_sep = min(min_sep, *(float(np.linalg.norm(p[i] - p[j])) for i, j in ((0, 1), (0, 2), (1, 2))))
    assert squeezed > 0
    assert min_sep >= F.SAFE_SEPARATION_M + CFG.margin_m


def test_turn_right_latch_reorients_once_when_its_cluster_reaches_three() -> None:
    # Step 1: two separate two-drone conflicts, both turn-right. a-b and d-c
    # both latch "left drone -y", which gives b and c the same colour.
    far = {"a": (100.0, 100.0, 100.0), "b": (110.0, 100.0, 100.0),
           "c": (510.0, 500.0, 100.0), "d": (500.0, 500.0, 100.0)}
    latch: dict = {}
    states = _states(far)
    F._update_latches({s.agent_id: s for s in states}, PAIRS_OF("abcd"), CFG, latch)
    assert {p: f.origin for p, f in latch.items()} == {("a", "b"): "turn_right", ("c", "d"): "turn_right"}

    # Step 2: the pairs meet in a 10 m square; a-d and b-c enter conflict.
    square = {"a": (100.0, 100.0, 100.0), "b": (110.0, 100.0, 100.0),
              "c": (110.0, 110.0, 100.0), "d": (100.0, 110.0, 100.0)}
    states = _states(square)
    F._update_latches({s.agent_id: s for s in states}, PAIRS_OF("abcd"), CFG, latch)
    latched = [("a", "b"), ("a", "d"), ("b", "c"), ("c", "d")]
    assert set(latch) == set(latched)
    for aid in "abcd":
        colours = {_colour(states, latch, pair, aid) for pair in latched if aid in pair}
        assert len(colours) == 1, f"{aid} has contradictory latches"
    assert all(f.origin == "fixed" for f in latch.values())
    axes = {pair: latch[pair].axis.copy() for pair in latched}

    # Later steps never flip a latch, not even when a fifth drone closes an
    # odd cycle with a and b.
    states = _states({**square, "e": (105.0, 92.0, 100.0)})
    for _ in range(2):
        F._update_latches({s.agent_id: s for s in states}, PAIRS_OF("abcde"), CFG, latch)
    for pair in latched:
        np.testing.assert_array_equal(latch[pair].axis, axes[pair])
