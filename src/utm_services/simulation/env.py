"""Deterministic v1 point-mass environment (P1-AN-01).

Implements docs/simulation/SIMULATOR_NOTE.md. The environment is the source of
modeled ground truth for a synthetic world; it is not a sensor, flight-stack or
aircraft model. Field names, units and bounds come from shared/contracts.py.

Lifecycle: ``reset(config)`` then ``step(actions)`` until ``done``. ``step``
takes one returned ``SafeAction`` per active agent; unshielded runners wrap the
proposed action unchanged (see simulation/runner.py).

Degraded commands: a missing, stale or dropped command does not stop the run.
The drone holds its last valid command for ``hold_steps`` steps, then brakes to
zero velocity at the per-axis acceleration limit and hovers. It uses only its
own velocity, never other drones. Every degraded agent-step is logged in
``infos`` and counted in the summary. ``strict=True`` raises instead (tests).
"""

from __future__ import annotations

import math
from collections import deque
from collections.abc import Sequence
from dataclasses import asdict, dataclass, field
from typing import Any

import numpy as np
from shared.contracts import (
    MAX_ACCEL_COMPONENT_MPS2,
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
RECEIVED, HOLD, BRAKE = "received", "hold", "brake"


@dataclass(frozen=True)
class Disturbances:
    """Seeded injectors (SIMULATOR_NOTE section 7). All zero is T1.

    Wind is one steady vector per episode, each component uniform in
    [-wind_max, wind_max], added to every drone's acceleration. Noise and delay
    act on what the server receives: the policy observation and
    ``observed_states()``, which the runner gives the filter. Ground truth stays
    exact and is used only for dynamics and metrics. Dropout loses each drone's
    command with probability ``dropout_p`` per step.
    """

    wind_max_mps2: float = 0.0
    obs_pos_sigma_m: float = 0.0
    obs_vel_sigma_mps: float = 0.0
    obs_delay_steps: int = 0
    dropout_p: float = 0.0


T1 = Disturbances()
# T2 "mild atmospheric" (SIMULATOR_NOTE section 13); w_max from Open Item 2.
# Placeholders: both T2 profiles are guesses until we have real telemetry data.
T2_NOMINAL = Disturbances(wind_max_mps2=1.0, obs_pos_sigma_m=0.5, obs_vel_sigma_mps=0.1, obs_delay_steps=1,
                          dropout_p=0.05)
T2_REALISTIC = Disturbances(wind_max_mps2=1.0, obs_pos_sigma_m=1.5, obs_vel_sigma_mps=0.3, obs_delay_steps=3,
                            dropout_p=0.1)
PROFILES = {"ideal": T1, "nominal": T2_NOMINAL, "realistic": T2_REALISTIC}
_PROFILE_NAMES = {d: name for name, d in PROFILES.items()}

# Filter flags for an agent whose command never reached step(): nothing decided.
_NO_ACTION = SafeAction(agent_id="none", safe_accel_mps2=(0.0, 0.0, 0.0), timestamp_ms=0, shield_active=False,
                        h_min_m2=None, qp_feasible=True, fallback_used=False)


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
    last_command: np.ndarray | None = None
    missed_commands: int = 0


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

    def __init__(
        self,
        rules: AirspaceRulesV1 | None = None,
        disturbances: Disturbances = T1,
        hold_steps: int = 3,
        strict: bool = False,
    ) -> None:
        self._rules = rules
        self._disturbances = disturbances
        self._hold_steps = hold_steps
        self._strict = strict
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

    def observed_states(self) -> list[DroneState]:
        """DroneState of every active agent as the server receives it: the same noisy,
        delayed position and velocity the policy observation uses, sorted by agent_id.

        Stamped with the current time (the receive time), because the filter
        requires state and action timestamps to match; the data is
        ``obs_delay_steps`` steps old.
        """
        received = self._obs_history[0]
        return [
            self._drone_state(a).model_copy(update={"position_m": tuple(map(float, received[a.agent_id][1])),
                                                    "velocity_mps": tuple(map(float, received[a.agent_id][2]))})
            for a in self._active()
        ]

    def reset(self, config: ScenarioConfig) -> tuple[dict[str, ObservationV1], dict[str, Any]]:
        """Reset from a validated ScenarioConfig; returns observations and run metadata.

        Initial conditions come from the config. The seed drives the injectors
        only, one spawned stream each, so T1 (no injectors) ignores it.
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
            "command_dropouts": 0,
            "degraded_agent_steps": 0,
        }
        d = self._disturbances
        wind_rng, self._noise_rng, self._dropout_rng = (
            np.random.default_rng(s) for s in np.random.SeedSequence(config.seed).spawn(3)
        )
        self._wind = wind_rng.uniform(-d.wind_max_mps2, d.wind_max_mps2, 3)
        # Per step: agent_id -> (observation, observed position, observed velocity).
        self._obs_history: deque[dict[str, tuple[ObservationV1, np.ndarray, np.ndarray]]] = deque(
            maxlen=d.obs_delay_steps + 1
        )
        self._record_separation(self._active())
        observations = self._deliver_observations(self.h_min_m2())
        metadata = {
            "env_version": ENV_VERSION,
            "reward_version": REWARD_VERSION,
            "scenario_id": config.scenario_id,
            "seed": config.seed,
            "episode_limit_steps": config.episode_limit_steps,
            "dt_s": DT_S,
            "goal_radius_m": GOAL_RADIUS_M,
            "condition_tier": {T1: "T1", T2_NOMINAL: "T2", T2_REALISTIC: "T2"}.get(d, "custom"),
            "disturbance_profile": _PROFILE_NAMES.get(d, "custom"),
            "disturbances": {**asdict(d), "wind_mps2": self._wind.tolist()},
            "hold_steps": self._hold_steps,
            "rules_version": self._rules.version if self._rules else None,
            "agent_ids": sorted(self._agents),
        }
        return observations, metadata

    def step(self, actions: Sequence[SafeAction]) -> StepResult:
        """Advance one 100 ms step with at most one SafeAction per active agent.

        A missing, stale or dropped command takes the degraded path (module docstring).
        """
        if self._done:
            raise RuntimeError("step() called on a finished episode; call reset()")
        stepping = self._active()
        self._validate_actions(actions, [a.agent_id for a in stepping])
        by_id = {act.agent_id: act for act in actions}
        h_min_prev = _h_min(stepping)
        prev_goal_dist = {a.agent_id: float(np.linalg.norm(a.goal - a.position)) for a in stepping}

        # Command delivery, then dynamics, bounds and goal checks (SIMULATOR_NOTE section 4-5).
        commands, modes, dropped = {}, {}, {}
        for agent in stepping:
            aid = agent.agent_id
            act = by_id.get(aid)
            dropped[aid] = bool(self._dropout_rng.random() < self._disturbances.dropout_p)
            if act is not None and act.timestamp_ms == self.timestamp_ms and not dropped[aid]:
                agent.last_command, agent.missed_commands = np.array(act.safe_accel_mps2, dtype=np.float64), 0
                commands[aid], modes[aid] = agent.last_command, RECEIVED
                continue
            agent.missed_commands += 1
            if agent.last_command is not None and agent.missed_commands <= self._hold_steps:
                commands[aid], modes[aid] = agent.last_command, HOLD
            else:
                brake = np.clip(-agent.velocity / DT_S, -MAX_ACCEL_COMPONENT_MPS2, MAX_ACCEL_COMPONENT_MPS2)
                commands[aid], modes[aid] = brake, BRAKE
        accels = {aid: cmd + self._wind for aid, cmd in commands.items()}
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
            # Filter flags count what the filter decided, even if the command was lost.
            act = by_id.get(aid) or _NO_ACTION
            nearest = float(dist[idx].min()) if len(stepping) > 1 else None
            violation = bool(violating[idx].any())
            geofence = self._in_geofence(agent.position, t_next)
            any_infeasible |= not act.qp_feasible
            self._metrics["shield_interventions"] += act.shield_active
            self._metrics["fallback_events"] += act.fallback_used
            self._metrics["speed_cap_events"] += speed_capped[aid]
            self._metrics["geofence_incursion_agent_steps"] += geofence
            self._metrics["command_dropouts"] += dropped[aid]
            self._metrics["degraded_agent_steps"] += modes[aid] != RECEIVED

            accel = commands[aid]
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
                "command": modes[aid],
                "command_dropped": dropped[aid],
            }
        # Terminated agents get no observation: an aborted agent can sit outside
        # the world box, which ObservationV1 cannot represent.
        result.observations.update(self._deliver_observations(h_min_now))
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
        if not set(ids) <= set(expected) or (self._strict and set(ids) != set(expected)):
            raise ValueError(f"actions must cover exactly the active agents {expected}, got {sorted(ids)}")
        for act in actions:
            if not isinstance(act, SafeAction):
                raise TypeError("step() requires SafeAction instances")
            if self._strict and act.timestamp_ms != self.timestamp_ms:
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

    def _deliver_observations(self, h_min: float | None) -> dict[str, ObservationV1]:
        """Observations of the active agents after noise and delay.

        Each agent's observed position and velocity get one Gaussian draw per step;
        neighbour terms use the neighbour's observed values. The policy receives the
        observation from ``obs_delay_steps`` steps ago (the oldest kept, early on).
        """
        d = self._disturbances
        active = self._active()
        observed = {}
        for a in active:
            # Clipped to the contract bounds so the observation stays a valid ObservationV1.
            pos = np.clip(a.position + self._noise_rng.normal(0.0, d.obs_pos_sigma_m, 3),
                          (WORLD_XY_MIN_M, WORLD_XY_MIN_M, WORLD_Z_MIN_M),
                          (WORLD_XY_MAX_M, WORLD_XY_MAX_M, WORLD_Z_MAX_M))
            vel = a.velocity + self._noise_rng.normal(0.0, d.obs_vel_sigma_mps, 3)
            speed = float(np.linalg.norm(vel))
            if speed > MAX_SPEED_MPS:
                vel *= MAX_SPEED_MPS / speed * _SPEED_CAP_SCALE
            observed[a.agent_id] = (pos, vel)
        self._obs_history.append({a.agent_id: (self._observation(a, h_min, observed), *observed[a.agent_id])
                                  for a in active})
        return {a.agent_id: self._obs_history[0][a.agent_id][0] for a in active}

    def _observation(
        self, agent: _Agent, h_min: float | None, observed: dict[str, tuple[np.ndarray, np.ndarray]]
    ) -> ObservationV1:
        """Build ObservationV1 from observed positions and velocities; index 17 uses the
        ground-truth barrier minimum over active pairs at the state the policy is about
        to act on (initial state at reset)."""
        pos, vel = observed[agent.agent_id]
        others = [aid for aid in observed if aid != agent.agent_id]
        if others:
            # Nearest by 3D distance, ties broken by lexicographic agent_id.
            nbr = min(others, key=lambda o: (float(np.linalg.norm(observed[o][0] - pos)), o))
            rel_p = observed[nbr][0] - pos
            rel_v = observed[nbr][1] - vel
            dist = float(np.linalg.norm(rel_p))
        else:
            rel_p = rel_v = np.zeros(3)
            dist = -1.0
        barrier = 1.0 if h_min is None else max(-1.0, min(1.0, h_min / SAFE_SEPARATION_M**2))
        values = [
            *pos,
            *vel,
            *(agent.goal - pos),
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
