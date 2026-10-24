"""Episode rollouts for MAPPO training (P2-SA-01). No torch: safe to run in worker processes.

One rollout job flies one ScenarioConfig in the unmodified ``DroneEnv`` (T1, ideal)
and records, per step and per agent slot, what the learner needs. Slots are the
agents sorted by id, padded to ``max_agents``; a slot is ``alive`` at a step if its
agent received a command that step. Dead or absent slots hold zeros.

Training sees exactly what the deployed policy sees: raw ``ObservationV1`` values
through ``normalize_obs``. The centralised critic additionally gets every slot's
normalised observation (the CTDE part); it is dropped after training.

Training reward is the environment's ``P1-SA-01-reward-v1``. It is not a safety
metric; the per-episode summaries returned here carry the measured separation
quantities instead.
"""

from __future__ import annotations

from typing import Any

import numpy as np
from shared.contracts import MARLAction, SafeAction, ScenarioConfig

from utm_services.policy.learned import ACTION_SCALE_MPS2, OBS_DIM, actor_mean, normalize_obs
from utm_services.policy.reference import compute_reference_action
from utm_services.safety.filter import FilterConfig, apply_filter
from utm_services.simulation.env import PROFILES, DroneEnv


def _safe_action(agent_id: str, accel: np.ndarray, ts: int) -> SafeAction:
    return SafeAction(
        agent_id=agent_id, safe_accel_mps2=(float(accel[0]), float(accel[1]), float(accel[2])),
        timestamp_ms=ts, shield_active=False, h_min_m2=None, qp_feasible=True, fallback_used=False,
    )


def collect_episode(job: dict[str, Any]) -> dict[str, Any]:
    """Fly one episode and return arrays plus the env summary.

    job keys:
      scenario      ScenarioConfig as a dict (model_dump)
      max_agents    slot count M
      rng_entropy   list[int] seeding the action noise stream
      mode          "policy"  sample from the Gaussian actor (training)
                    "greedy"  deterministic mean action (validation)
                    "reference" reference controller + noise, returns BC labels
      weights       actor weights dict (numpy), for policy/greedy
      log_std       (3,) array, for policy
      bc_noise      noise std in network units, for reference
      shield        bool: run the HOCBF filter on the proposals
      profile       disturbance profile name from simulation.env.PROFILES (default "ideal" = T1)
      filter_config dict | None
      keep_arrays   bool: False skips storing arrays (validation)
    """
    config = ScenarioConfig.model_validate(job["scenario"])
    M, mode, shield = int(job["max_agents"]), job["mode"], bool(job.get("shield", False))
    rng = np.random.default_rng(job["rng_entropy"])
    slot = {d.agent_id: k for k, d in enumerate(sorted(config.drones, key=lambda d: d.agent_id))}
    if len(slot) > M:
        raise ValueError(f"scenario has {len(slot)} agents, more than max_agents={M}")
    weights = job.get("weights")
    std = np.exp(np.asarray(job["log_std"])) if mode == "policy" else None
    filter_config = FilterConfig(**job["filter_config"]) if job.get("filter_config") else FilterConfig()
    latch: dict = {}

    env = DroneEnv(disturbances=PROFILES[job.get("profile", "ideal")])
    observations, _ = env.reset(config)

    obs_t, alive_t, act_t, rew_t, label_t = [], [], [], [], []
    while not env.done:
        ids = env.active_ids
        ts = env.timestamp_ms
        raw = np.array([observations[a].root for a in ids], dtype=np.float64)
        x = normalize_obs(raw)
        if mode == "reference":
            label = np.array([
                np.asarray(compute_reference_action(observations[a], a, ts).proposed_accel_mps2) for a in ids
            ]) / ACTION_SCALE_MPS2
            sample = label + float(job["bc_noise"]) * rng.standard_normal(label.shape)
        else:
            mean = actor_mean(weights, x)
            sample = mean if mode == "greedy" else mean + std * rng.standard_normal(mean.shape)
        accel = np.clip(sample, -1.0, 1.0) * ACTION_SCALE_MPS2

        if shield:
            proposed = [MARLAction(agent_id=a, proposed_accel_mps2=tuple(float(v) for v in accel[k]), timestamp_ms=ts)
                        for k, a in enumerate(ids)]
            returned, _ = apply_filter(env.observed_states(), proposed, filter_config, latch=latch)
        else:
            returned = [_safe_action(a, accel[k], ts) for k, a in enumerate(ids)]
        result = env.step(returned)

        if job.get("keep_arrays", True):
            o = np.zeros((M, OBS_DIM)); al = np.zeros(M, dtype=bool); ac = np.zeros((M, 3)); r = np.zeros(M)
            lb = np.zeros((M, 3))
            for k, a in enumerate(ids):
                s = slot[a]
                o[s], al[s], ac[s], r[s] = x[k], True, sample[k], result.rewards[a]
                if mode == "reference":
                    lb[s] = label[k]
            obs_t.append(o); alive_t.append(al); act_t.append(ac); rew_t.append(r); label_t.append(lb)
        observations = result.observations

    out: dict[str, Any] = {"summary": env.summary(), "steps": env.timestamp_ms // 100}
    if job.get("keep_arrays", True):
        final_obs = np.zeros((M, OBS_DIM)); final_alive = np.zeros(M, dtype=bool)
        for a, obs in observations.items():
            if a in env.active_ids:
                final_obs[slot[a]], final_alive[slot[a]] = normalize_obs(np.asarray(obs.root)), True
        out.update(
            obs=np.stack(obs_t), alive=np.stack(alive_t), act=np.stack(act_t), rew=np.stack(rew_t),
            labels=np.stack(label_t), final_obs=final_obs, final_alive=final_alive,
        )
    return out
