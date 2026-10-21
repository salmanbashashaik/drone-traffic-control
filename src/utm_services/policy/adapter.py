"""ObservationV1 → policy → MARLAction adapter for v1.

This module is the only place in Salman's workstream where raw
observation values are unpacked and where policy output is validated
before being handed to the filter or simulator. Nothing outside this
module should unpack observation indices directly.

Responsibilities
----------------
- Accept raw observation data (list/tuple of 18 floats) and an agent id
  / timestamp from the step loop.
- Validate the observation against the shared ObservationV1 contract.
- Pass the validated observation to a policy callable.
- Validate and return the resulting MARLAction.
- Reject invalid input and invalid output visibly — never clip or
  silently coerce.

Policy callable protocol
------------------------
A policy is any callable with the signature::

    def policy(obs: ObservationV1, agent_id: str, timestamp_ms: int) -> MARLAction

The reference controller in ``reference.py`` satisfies this protocol.
A learned MARL actor will satisfy the same protocol once implemented.

What this adapter does NOT do
------------------------------
- Does not normalise or preprocess observation values. If a learned
  policy requires normalisation, that must happen inside the policy
  callable itself, with versioned parameters stored alongside the
  model artifact.
- Does not modify, clip, or label the MARLAction as safe.
- Does not call the safety filter. The adapter boundary ends at a
  validated MARLAction.
"""

from __future__ import annotations

import logging
from typing import Callable, Sequence

from pydantic import ValidationError

from shared.contracts import MARLAction, ObservationV1

logger = logging.getLogger(__name__)

# Type alias for the policy callable accepted by this adapter.
PolicyFn = Callable[[ObservationV1, str, int], MARLAction]


class ObservationValidationError(ValueError):
    """Raised when raw observation data fails ObservationV1 validation."""


class ActionValidationError(ValueError):
    """Raised when the policy returns an action that fails MARLAction validation."""


def run_policy(
    raw_obs: Sequence[float],
    agent_id: str,
    timestamp_ms: int,
    policy: PolicyFn,
) -> MARLAction:
    """Validate observation, run policy, validate and return action.

    Parameters
    ----------
    raw_obs:
        18-element sequence of floats from the simulator step.
        Must satisfy all ObservationV1 semantic constraints.
    agent_id:
        Agent identifier for this step. Must match the returned action.
    timestamp_ms:
        Current simulator timestamp in milliseconds. Must be a
        non-negative multiple of 100.
    policy:
        Callable satisfying the PolicyFn protocol. Must return a
        valid MARLAction without raising.

    Returns
    -------
    MARLAction
        Validated nominal acceleration proposal from the policy.

    Raises
    ------
    ObservationValidationError
        If ``raw_obs`` fails ObservationV1 validation. The step loop
        must not proceed with an invalid observation.
    ActionValidationError
        If the policy returns an action that fails MARLAction validation
        or whose agent_id / timestamp_ms do not match the inputs.
        The step loop must not pass an invalid action to the filter.
    """
    # --- Step 1: validate observation ---
    try:
        obs = ObservationV1.model_validate(list(raw_obs))
    except ValidationError as exc:
        raise ObservationValidationError(
            f"ObservationV1 validation failed for agent '{agent_id}' "
            f"at t={timestamp_ms} ms: {exc}"
        ) from exc

    # --- Step 2: run policy ---
    action = policy(obs, agent_id, timestamp_ms)

    # --- Step 3: validate action ---
    # Check Pydantic contract first.
    try:
        # Re-validate to catch any policy that constructs MARLAction
        # without going through the validator (e.g. model_construct).
        action = MARLAction.model_validate(action.model_dump())
    except ValidationError as exc:
        raise ActionValidationError(
            f"Policy returned invalid MARLAction for agent '{agent_id}' "
            f"at t={timestamp_ms} ms: {exc}"
        ) from exc

    # Check identity alignment separately for a clearer error message.
    if action.agent_id != agent_id:
        raise ActionValidationError(
            f"Policy action agent_id '{action.agent_id}' does not match "
            f"expected '{agent_id}'"
        )
    if action.timestamp_ms != timestamp_ms:
        raise ActionValidationError(
            f"Policy action timestamp_ms {action.timestamp_ms} does not match "
            f"step timestamp {timestamp_ms} ms"
        )

    logger.debug(
        "adapter: agent=%s t=%d ms accel=(%.3f, %.3f, %.3f)",
        agent_id,
        timestamp_ms,
        *action.proposed_accel_mps2,
    )

    return action
