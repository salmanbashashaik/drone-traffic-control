"""Controller-variant runner over DroneEnv (P2-AN-01).

Every variant gets the same scenario, dynamics and episode limit; only the
policy and the presence of the safety filter differ. Unshielded variants wrap
the proposed action unchanged into a SafeAction (shield_active=False). The
shielded variant calls the HOCBF filter, whose infeasible path returns the
zero-acceleration fallback. That fallback is unverified: it is not a hover and
does not guarantee separation.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Literal

import numpy as np
from shared.contracts import AirspaceRulesV1, SafeAction, ScenarioConfig

from utm_services.policy.adapter import PolicyFn, run_policy
from utm_services.policy.reference import compute_reference_action
from utm_services.safety.filter import FilterConfig, apply_filter
from utm_services.simulation.env import DroneEnv

Variant = Literal["reference", "unshielded_marl", "shielded_marl"]
VARIANTS: tuple[Variant, ...] = ("reference", "unshielded_marl", "shielded_marl")


@dataclass(frozen=True)
class EpisodeResult:
    manifest: dict[str, Any]
    summary: dict[str, Any]
    records: list[dict[str, Any]]


def _mean_p95(samples: list[float]) -> tuple[float | None, float | None]:
    if not samples:
        return None, None
    return float(np.mean(samples)), float(np.percentile(samples, 95))


def run_episode(
    config: ScenarioConfig,
    variant: Variant,
    policy: PolicyFn | None = None,
    policy_id: str | None = None,
    filter_config: FilterConfig | None = None,
    rules: AirspaceRulesV1 | None = None,
    keep_records: bool = True,
    make_env: Callable[[AirspaceRulesV1 | None], DroneEnv] = DroneEnv,
) -> EpisodeResult:
    """Run one episode; returns the manifest, episode summary and per-step records.

    ``reference`` always uses the deterministic reference controller. The MARL
    variants need ``policy``; until a trained policy exists, callers may pass the
    reference controller and must label it through ``policy_id``. ``make_env``
    selects the dynamics backend (point-mass ``DroneEnv`` by default).
    """
    if variant not in VARIANTS:
        raise ValueError(f"unknown controller variant '{variant}'")
    if variant == "reference":
        policy, policy_id = compute_reference_action, "reference-p1-sa-01"
    elif policy is None or not policy_id:
        raise ValueError(f"variant '{variant}' requires a policy and a policy_id")
    shielded = variant == "shielded_marl"
    filter_config = filter_config or FilterConfig()

    env = make_env(rules)
    observations, metadata = env.reset(config)
    episode_id = f"{config.scenario_id}-s{config.seed}-{variant}"
    manifest = {
        **metadata,
        "episode_id": episode_id,
        "controller_variant": variant,
        "policy_id": policy_id,
        "filter": vars(filter_config) if shielded else None,
        "step_timing_boundary": "policy + filter + env.step, time.perf_counter",
        "policy_timing_boundary": "per-agent run_policy call incl. validation, time.perf_counter",
    }
    records: list[dict[str, Any]] = []
    step_ms: list[float] = []
    policy_ms: list[float] = []

    while not env.done:
        t0 = time.perf_counter()
        ts = env.timestamp_ms
        states = env.states()
        proposed = []
        for state in states:
            p0 = time.perf_counter()
            proposed.append(run_policy(observations[state.agent_id].root, state.agent_id, ts, policy))
            policy_ms.append((time.perf_counter() - p0) * 1000.0)

        if shielded:
            returned, _ = apply_filter(states, proposed, filter_config)
        else:
            h_min = env.h_min_m2()
            returned = [
                SafeAction(
                    agent_id=a.agent_id,
                    safe_accel_mps2=a.proposed_accel_mps2,
                    timestamp_ms=ts,
                    shield_active=False,
                    h_min_m2=h_min,
                    qp_feasible=True,
                    fallback_used=False,
                )
                for a in proposed
            ]
        result = env.step(returned)
        step_ms.append((time.perf_counter() - t0) * 1000.0)
        observations = result.observations

        if keep_records:
            records.append(
                {
                    "episode_id": episode_id,
                    "scenario_id": config.scenario_id,
                    "seed": config.seed,
                    "controller_variant": variant,
                    "timestamp_ms": ts,
                    "states": [s.model_dump(mode="json") for s in states],
                    "proposed_actions": [a.model_dump(mode="json") for a in proposed],
                    "returned_actions": [a.model_dump(mode="json") for a in returned],
                    "infos": result.infos,
                }
            )

    mean_step, p95_step = _mean_p95(step_ms)
    mean_pol, p95_pol = _mean_p95(policy_ms)
    summary = {
        "episode_id": episode_id,
        **env.summary(),
        "mean_step_time_ms": mean_step,
        "p95_step_time_ms": p95_step,
        "mean_policy_latency_ms": mean_pol,
        "p95_policy_latency_ms": p95_pol,
    }
    return EpisodeResult(manifest, summary, records)
