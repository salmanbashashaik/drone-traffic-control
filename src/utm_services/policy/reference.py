"""Deterministic reference controller for v1 matched comparisons.

This controller is intentionally simple: it applies a proportional
go-to-goal acceleration computed from ObservationV1. It is used as
a reproducible, non-learned baseline alongside the unshielded and
shielded MARL variants.

Assumptions / limitations
--------------------------
- Input is ObservationV1 (18 floats, raw simulation units).
- Output is a bounded MARLAction. Acceleration is clipped to the
  contract limit before output — but invalid pre-clip magnitudes are
  logged, not silently ignored.
- This controller does NOT account for neighbors, separation, or
  any safety constraint. It is a nominal policy proposer only.
- Reproducibility: the controller is purely deterministic given the
  observation. No random state is used.
- Do not label output of this controller as "safe".
"""

from __future__ import annotations

import math
import logging
from typing import Sequence

from shared.contracts import (
    MAX_ACCEL_COMPONENT_MPS2,
    MARLAction,
    ObservationV1,
)

logger = logging.getLogger(__name__)

# Proportional gain: acceleration = GAIN * goal_displacement (capped at u_max).
# At gain=0.5 and displacement=6m the raw accel is 3.0 m/s² — within bounds.
# Tuning note: higher gain → faster approach but more overshoot at 100ms steps.
_PROPORTIONAL_GAIN: float = 0.5

# Damping gain: deceleration proportional to own velocity, applied everywhere.
# 2*sqrt(Kp) is critical damping, so the drone reaches the goal without
# overshoot. A braking zone that only starts near the goal cannot stop a
# drone at 15 m/s (stopping distance ~32 m at 3.5 m/s²).
_DAMPING_GAIN: float = 2.0 * math.sqrt(_PROPORTIONAL_GAIN)


def compute_reference_action(
    obs: ObservationV1,
    agent_id: str,
    timestamp_ms: int,
) -> MARLAction:
    """Compute a deterministic proportional go-to-goal acceleration.

    Parameters
    ----------
    obs:
        Validated ObservationV1 for this agent at this step.
    agent_id:
        Agent identifier; passed through to MARLAction unchanged.
    timestamp_ms:
        Current simulator timestamp; passed through to MARLAction.

    Returns
    -------
    MARLAction
        Bounded nominal acceleration proposal. Never raises for valid
        inputs — clipping is applied and logged when the raw value
        would exceed the contract bound.
    """
    values: Sequence[float] = obs.root

    # Own velocity (indices 3-5) — used for damping.
    vx, vy, vz = values[3], values[4], values[5]

    # Goal displacement = goal - own_position (indices 6-8).
    dx, dy, dz = values[6], values[7], values[8]
    dist_to_goal = math.sqrt(dx * dx + dy * dy + dz * dz)

    if dist_to_goal < 1e-6:
        # Already at goal — command zero acceleration.
        return MARLAction(
            agent_id=agent_id,
            proposed_accel_mps2=(0.0, 0.0, 0.0),
            timestamp_ms=timestamp_ms,
        )

    # --- Proportional attraction toward goal, damped by own velocity ---
    ax = _PROPORTIONAL_GAIN * dx - _DAMPING_GAIN * vx
    ay = _PROPORTIONAL_GAIN * dy - _DAMPING_GAIN * vy
    az = _PROPORTIONAL_GAIN * dz - _DAMPING_GAIN * vz

    # --- Clip to contract bounds ---
    raw_max = max(abs(ax), abs(ay), abs(az))
    if raw_max > MAX_ACCEL_COMPONENT_MPS2:
        logger.debug(
            "reference controller: raw accel %.3f m/s² exceeds bound %.3f, clipping",
            raw_max,
            MAX_ACCEL_COMPONENT_MPS2,
        )
        # Subtract tiny epsilon so float rounding never exceeds the bound.
        scale = (MAX_ACCEL_COMPONENT_MPS2 - 1e-9) / raw_max
        ax *= scale
        ay *= scale
        az *= scale

    return MARLAction(
        agent_id=agent_id,
        proposed_accel_mps2=(ax, ay, az),
        timestamp_ms=timestamp_ms,
    )
