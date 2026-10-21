"""Deterministic v1 point-mass environment (P1-AN-01).

Implements docs/simulation/SIMULATOR_NOTE.md. The environment is the source of
modeled ground truth for a synthetic world; it is not a sensor, flight-stack or
aircraft model. Field names, units and bounds come from shared/contracts.py.

Lifecycle: ``reset(config)`` then ``step(actions)`` until ``done``. ``step``
takes one returned ``SafeAction`` per active agent; unshielded runners wrap the
proposed action unchanged (see simulation/runner.py).
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from shared.contracts import (
    MAX_SPEED_MPS,
    SAFE_SEPARATION_M,
    SIMULATION_STEP_MS,
    WORLD_XY_MAX_M,
    WORLD_XY_MIN_M,
    WORLD_Z_MAX_M,
    WORLD_Z_MIN_M,
    AirspaceRulesV1,
    DroneState,
    ObservationV1,
    SafeAction,
    ScenarioConfig,
)

ENV_VERSION = "sim-v1.0"
REWARD_VERSION = "P1-SA-01-reward-v1"
GOAL_RADIUS_M = 2.0
DT_S = SIMULATION_STEP_MS / 1000.0

# Reward weights from docs/formulation/P1-SA-01-reward.md (starting values).
_LAMBDA_PROGRESS = 1.0
_C_GOAL = 50.0
_LAMBDA_SEPARATION = 5.0
_C_VIOLATION = 20.0
_LAMBDA_ACCEL = 0.01
_LAMBDA_JERK = 0.01
_C_FALLBACK = 10.0

# Scale capped speeds just under the limit so float rounding never fails DroneState validation.
_SPEED_CAP_SCALE = 1.0 - 1e-12

ACTIVE, COMPLETED, ABORTED = "active", "completed", "aborted"


@dataclass
class _Agent:
    agent_id: str
    position: np.ndarray
    velocity: np.ndarray
    goal: np.ndarray
    status: str = ACTIVE
    abort_reason: str | None = None
    prev_accel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    prev_shield_active: bool = False


@dataclass(frozen=True)
class StepResult:
    """PettingZoo-style parallel step result, keyed by agent_id."""

    observations: dict[str, ObservationV1]
    rewards: dict[str, float]
    terminations: dict[str, bool]
    truncations: dict[str, bool]
    infos: dict[str, dict[str, Any]]


def _in_world(p: np.ndarray) -> bool:
    return (
        WORLD_XY_MIN_M <= p[0] <= WORLD_XY_MAX_M
        and WORLD_XY_MIN_M <= p[1] <= WORLD_XY_MAX_M
        and WORLD_Z_MIN_M <= p[2] <= WORLD_Z_MAX_M
    )


def _pair_distances(agents: Sequence[_Agent]) -> np.ndarray:
    """Symmetric matrix of 3D distances; diagonal is +inf."""
    pos = np.array([a.position for a in agents], dtype=np.float64).reshape(-1, 3)
    dist = np.linalg.norm(pos[:, None, :] - pos[None, :, :], axis=-1)
    np.fill_diagonal(dist, np.inf)
    return dist


def _h_min(agents: Sequence[_Agent]) -> float | None:
    """Minimum pairwise barrier ||p_i - p_j||^2 - d_safe^2, or None without a pair."""
    if len(agents) < 2:
        return None
    return float(_pair_distances(agents).min() ** 2 - SAFE_SEPARATION_M**2)


class DroneEnv:
    """Seeded multi-agent point-mass environment for the v1 contract."""

    def __init__(self, rules: AirspaceRulesV1 | None = None) -> None:
        self._rules = rules
        self._config: ScenarioConfig | None = None
        self._agents: dict[str, _Agent] = {}
        self._step = 0
        self._done = True

    # ------------------------------------------------------------------ API

    @property
    def timestamp_ms(self) -> int:
        return self._step * SIMULATION_STEP_MS

    @property
    def done(self) -> bool:
        return self._done

    @property
    def active_ids(self) -> list[str]:
        return sorted(aid for aid, a in self._agents.items() if a.status == ACTIVE)

    def h_min_m2(self) -> float | None:
        """Minimum barrier over active pairs at the current state."""
        return _h_min(self._active())

    def states(self) -> list[DroneState]:
        """Ground-truth DroneState of every active agent, sorted by agent_id."""
        return [self._drone_state(a) for a in self._active()]

    def reset(self, config: ScenarioConfig) -> tuple[dict[str, ObservationV1], dict[str, Any]]:
        """Reset from a validated ScenarioConfig; returns observations and run metadata.

        T1 (v1) has no random elements: initial conditions come from the config
        and the seed is recorded for the run manifest only.
        """
        if not isinstance(config, ScenarioConfig):
            raise TypeError("reset() requires a validated ScenarioConfig")
        self._config = config
        self._step = 0
        self._done = False
        self._agents = {
            d.agent_id: _Agent(
                agent_id=d.agent_id,
                position=np.array(d.initial_position_m, dtype=np.float64),
                velocity=np.array(d.initial_velocity_mps, dtype=np.float64),
                goal=np.array(d.goal_m, dtype=np.float64),
            )
            for d in config.drones
        }
        self._metrics = {
            "n_launched": len(self._agents),
            "min_separation_m": None,
            "separation_violation_pair_steps": 0,
            "shield_interventions": 0,
            "qp_infeasible_steps": 0,
            "fallback_events": 0,
            "speed_cap_events": 0,
            "geofence_incursion_agent_steps": 0,
        }
        self._record_separation(self._active())
        h_min = self.h_min_m2()
        observations = {a.agent_id: self._observation(a, h_min) for a in self._active()}
        metadata = {
            "env_version": ENV_VERSION,
            "reward_version": REWARD_VERSION,
            "scenario_id": config.scenario_id,
            "seed": config.seed,
            "episode_limit_steps": config.episode_limit_steps,
            "dt_s": DT_S,
            "goal_radius_m": GOAL_RADIUS_M,
            "condition_tier": "T1",
            "disturbances": [],
            "rules_version": self._rules.version if self._rules else None,
            "agent_ids": sorted(self._agents),
        }
        return observations, metadata

    def step(self, actions: Sequence[SafeAction]) -> StepResult:
        """Advance one 100 ms step with exactly one SafeAction per active agent."""
        if self._done:
            raise RuntimeError("step() called on a finished episode; call reset()")
        stepping = self._active()
        self._validate_actions(actions, [a.agent_id for a in stepping])
        by_id = {act.agent_id: act for act in actions}
        h_min_prev = _h_min(stepping)
        prev_goal_dist = {a.agent_id: float(np.linalg.norm(a.goal - a.position)) for a in stepping}

        # Dynamics, bounds and goal checks (SIMULATOR_NOTE section 4-5).
        accels = {aid: np.array(act.safe_accel_mps2, dtype=np.float64) for aid, act in by_id.items()}
        speed_capped = self._advance(stepping, accels)
        for agent in stepping:
            if agent.status != ACTIVE:
                continue  # the dynamics backend already ended it (e.g. physical collision)
            if not _in_world(agent.position):
                agent.status, agent.abort_reason = ABORTED, "out_of_bounds"
            elif np.linalg.norm(agent.goal - agent.position) <= GOAL_RADIUS_M:
                agent.status = COMPLETED
        self._step += 1
        t_next = self.timestamp_ms

        # Separation over every agent that flew this step, on post-step positions.
        dist = _pair_distances(stepping)
        self._record_separation(stepping, dist)
        violating = dist < SAFE_SEPARATION_M

        truncated = self._step >= self._config.episode_limit_steps
        remaining = self._active()
        h_min_now = _h_min(remaining)
        any_infeasible = False

        result = StepResult({}, {}, {}, {}, {})
        for idx, agent in enumerate(stepping):
            aid = agent.agent_id
            act = by_id[aid]
            nearest = float(dist[idx].min()) if len(stepping) > 1 else None
            violation = bool(violating[idx].any())
            geofence = self._in_geofence(agent.position, t_next)
            any_infeasible |= not act.qp_feasible
            self._metrics["shield_interventions"] += act.shield_active
            self._metrics["fallback_events"] += act.fallback_used
            self._metrics["speed_cap_events"] += speed_capped[aid]
            self._metrics["geofence_incursion_agent_steps"] += geofence

            accel = np.array(act.safe_accel_mps2, dtype=np.float64)
            result.rewards[aid] = self._reward(
                agent, prev_goal_dist[aid], nearest, violation, accel, act.fallback_used
            )
            agent.prev_accel = accel
            agent.prev_shield_active = act.shield_active

            result.terminations[aid] = agent.status != ACTIVE
            result.truncations[aid] = truncated and agent.status == ACTIVE
            result.infos[aid] = {
                "timestamp_ms": t_next,
                "status": agent.status,
                "abort_reason": agent.abort_reason,
                "shield_active": act.shield_active,
                "h_min_m2": act.h_min_m2,
                "qp_feasible": act.qp_feasible,
                "fallback_used": act.fallback_used,
                "nearest_separation_m": nearest,
                "separation_violation": violation,
                "speed_capped": speed_capped[aid],
                "geofence_incursion": geofence,
                "h_min_prev_m2": h_min_prev,
            }
            # Terminated agents get no observation: an aborted agent can sit outside
            # the world box, which ObservationV1 cannot represent.
            if agent.status == ACTIVE:
                result.observations[aid] = self._observation(agent, h_min_now)
        self._metrics["qp_infeasible_steps"] += any_infeasible
        self._done = truncated or not remaining
        return result

    def summary(self) -> dict[str, Any]:
        """Episode accounting and metrics (INTERFACES.md 'Logging and evaluation fields').

        Latency fields are measured by the runner, which owns the timing boundary.
        """
        statuses = [a.status for a in self._agents.values()]
        n_launched = self._metrics["n_launched"]
        n_completed = statuses.count(COMPLETED)
        return {
            **self._metrics,
            "n_completed": n_completed,
            "n_aborted": statuses.count(ABORTED),
            "n_held": 0,
            "n_incomplete": statuses.count(ACTIVE),
            "completion_pct": 100.0 * n_completed / n_launched if n_launched else None,
            "steps": self._step,
        }

    # ------------------------------------------------------------- helpers

    def _advance(self, agents: Sequence[_Agent], accels: dict[str, np.ndarray]) -> dict[str, bool]:
        """Point-mass dynamics for one step; returns speed_capped per agent.

        Backends override this to integrate with other dynamics; they update
        ``position``/``velocity`` and may end an agent by setting ``status``.
        """
        speed_capped: dict[str, bool] = {}
        for agent in agents:
            v_next = agent.velocity + accels[agent.agent_id] * DT_S
            speed = float(np.linalg.norm(v_next))
            speed_capped[agent.agent_id] = speed > MAX_SPEED_MPS
            if speed_capped[agent.agent_id]:
                v_next *= MAX_SPEED_MPS / speed * _SPEED_CAP_SCALE
            # Trapezoid: equals p + v*dt + 0.5*a*dt^2 when uncapped, and stays consistent with v_next when capped.
            agent.position = agent.position + 0.5 * (agent.velocity + v_next) * DT_S
            agent.velocity = v_next
        return speed_capped

    def _active(self) -> list[_Agent]:
        return [self._agents[aid] for aid in self.active_ids]

    def _validate_actions(self, actions: Sequence[SafeAction], expected: list[str]) -> None:
        ids = [a.agent_id for a in actions]
        if len(ids) != len(set(ids)):
            raise ValueError(f"duplicate agent_id in actions: {sorted(ids)}")
        if set(ids) != set(expected):
            raise ValueError(f"actions must cover exactly the active agents {expected}, got {sorted(ids)}")
        for act in actions:
            if not isinstance(act, SafeAction):
                raise TypeError("step() requires SafeAction instances")
            if act.timestamp_ms != self.timestamp_ms:
                raise ValueError(
                    f"stale action for '{act.agent_id}': timestamp {act.timestamp_ms} ms, "
                    f"state time {self.timestamp_ms} ms"
                )

    def _drone_state(self, agent: _Agent) -> DroneState:
        return DroneState(
            agent_id=agent.agent_id,
            position_m=tuple(float(x) for x in agent.position),
            velocity_mps=tuple(float(x) for x in agent.velocity),
            goal_m=tuple(float(x) for x in agent.goal),
            timestamp_ms=self.timestamp_ms,
        )

    def _observation(self, agent: _Agent, h_min: float | None) -> ObservationV1:
        """Build ObservationV1; index 17 uses the barrier minimum over active pairs
        at the state the policy is about to act on (initial state at reset)."""
        others = [a for a in self._active() if a.agent_id != agent.agent_id]
        if others:
            # Nearest by 3D distance, ties broken by lexicographic agent_id.
            nbr = min(others, key=lambda o: (float(np.linalg.norm(o.position - agent.position)), o.agent_id))
            rel_p = nbr.position - agent.position
            rel_v = nbr.velocity - agent.velocity
            dist = float(np.linalg.norm(rel_p))
        else:
            rel_p = rel_v = np.zeros(3)
            dist = -1.0
        barrier = 1.0 if h_min is None else max(-1.0, min(1.0, h_min / SAFE_SEPARATION_M**2))
        values = [
            *agent.position,
            *agent.velocity,
            *(agent.goal - agent.position),
            *rel_p,
            *rel_v,
            dist,
            1.0 if agent.prev_shield_active else 0.0,
            barrier,
        ]
        return ObservationV1.model_validate([float(v) for v in values])

    def _reward(
        self,
        agent: _Agent,
        prev_goal_dist: float,
        nearest: float | None,
        violation: bool,
        accel: np.ndarray,
        fallback: bool,
    ) -> float:
        """Training reward from P1-SA-01-reward.md. Not a safety metric."""
        goal_dist = float(np.linalg.norm(agent.goal - agent.position))
        reward = _LAMBDA_PROGRESS * (prev_goal_dist - goal_dist)
        if agent.status == COMPLETED:
            reward += _C_GOAL
        if nearest is not None:
            reward -= _LAMBDA_SEPARATION * max(0.0, SAFE_SEPARATION_M - nearest) / SAFE_SEPARATION_M
        if violation:
            reward -= _C_VIOLATION
        reward -= _LAMBDA_ACCEL * float(accel @ accel)
        jerk = accel - agent.prev_accel
        reward -= _LAMBDA_JERK * float(jerk @ jerk)
        if fallback:
            reward -= _C_FALLBACK
        return reward

    def _record_separation(self, agents: Sequence[_Agent], dist: np.ndarray | None = None) -> None:
        if len(agents) < 2:
            return
        if dist is None:
            dist = _pair_distances(agents)
        d_min = float(dist.min())
        prev = self._metrics["min_separation_m"]
        self._metrics["min_separation_m"] = d_min if prev is None else min(prev, d_min)
        # Each unordered pair once: the matrix is symmetric.
        self._metrics["separation_violation_pair_steps"] += int((dist < SAFE_SEPARATION_M).sum()) // 2

    def _in_geofence(self, p: np.ndarray, t_ms: int) -> bool:
        """True if p is inside an active closed-cylinder geofence from the mock API rules.

        known limit: geofences are logged, not enforced; the policy observation has no
        geofence field. Revisit when a contract change exposes them to policy/filter.
        """
        if self._rules is None:
            return False
        for g in self._rules.geofences:
            if not g.active_from_ms <= t_ms <= g.active_until_ms:
                continue
            if g.floor_m <= p[2] <= g.ceiling_m and math.hypot(
                p[0] - g.center_xy_m[0], p[1] - g.center_xy_m[1]
            ) <= g.radius_m:
                return True
        return False
