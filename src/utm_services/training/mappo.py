"""MAPPO trainer for the nominal MARL policy (P2-SA-01).

Algorithm (docs/formulation/P1-SA-01-method.md, with the deviations listed in
docs/training/MARL_TRAINING.md): multi-agent PPO with parameter sharing and
centralised training, decentralised execution. One shared actor maps an agent's
own ``ObservationV1`` to a Gaussian over acceleration; a centralised critic sees
the agent's observation, every agent's observation and the alive mask. Only the
actor ships.

The learned policy is a nominal action proposer. Training reward is
``P1-SA-01-reward-v1`` and is not a safety metric; the log carries measured
separation quantities (violation pair-steps, minimum separation) next to it.

Reproducibility: every random draw is seeded from ``(run_seed, iteration,
episode)``; scenarios from disjoint seed ranges (see scenarios.py). Resuming from
``checkpoint.pt`` continues the same stream.
"""

from __future__ import annotations

import json
import multiprocessing as mp
import platform
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch import nn

from utm_services.policy.learned import (
    ACT_DIM,
    OBS_DIM,
    save_policy_artifact,
    weights_sha256,
)
from utm_services.simulation.env import ENV_VERSION, REWARD_VERSION
from utm_services.training.rollout import collect_episode
from utm_services.training.scenarios import (
    FAMILY_VERSION,
    TRAIN_SEED_BASE,
    VAL_SEED_BASE,
    eval_scenarios,
    sample_scenario,
)

TRAINER_VERSION = "mappo-v1"
BC_SEED_BASE = 300_000_000


@dataclass
class TrainConfig:
    run_seed: int = 42
    iterations: int = 300
    episodes_per_iter: int = 16
    min_agents: int = 2
    max_agents: int = 4
    episode_limit_steps: int = 800
    hidden: int = 256
    init_log_std: float = -1.2
    # PPO
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_eps: float = 0.2
    epochs: int = 4
    minibatch: int = 2048
    lr_actor: float = 3e-4
    lr_critic: float = 1e-3
    lr_final_frac: float = 0.1       # linear decay to this fraction of the initial lr
    ent_coef: float = 0.001
    vf_coef: float = 0.5
    max_grad_norm: float = 0.5
    target_kl: float = 0.03
    reward_scale: float = 0.1        # constant, applied to the environment reward before GAE
    # curriculum over the conflict knob (scenarios.py)
    conflict_start: float = 0.3
    conflict_end: float = 1.0
    curriculum_frac: float = 0.5
    # behaviour-cloning warm start from the deterministic reference controller
    bc_warmstart: bool = True
    bc_episodes: int = 96
    bc_epochs: int = 40
    bc_noise: float = 0.15           # exploration noise on executed actions (labels stay clean)
    bc_lr: float = 1e-3
    # shield in the training loop (the unshielded policy is the default)
    shield: bool = False
    # disturbance profile flown in training and validation (simulation.env.PROFILES); ideal = T1
    profile: str = "ideal"
    # validation / model selection (validation seeds only)
    eval_every: int = 10
    val_episodes: int = 60
    val_conflict: float = 0.8
    workers: int = 1
    torch_threads: int = 2


# ----------------------------------------------------------------------------- networks


def _init(layer: nn.Linear, gain: float) -> nn.Linear:
    nn.init.orthogonal_(layer.weight, gain)
    nn.init.zeros_(layer.bias)
    return layer


class Actor(nn.Module):
    def __init__(self, hidden: int, init_log_std: float) -> None:
        super().__init__()
        self.l0 = _init(nn.Linear(OBS_DIM, hidden), np.sqrt(2))
        self.l1 = _init(nn.Linear(hidden, hidden), np.sqrt(2))
        self.l2 = _init(nn.Linear(hidden, ACT_DIM), 0.01)
        self.log_std = nn.Parameter(torch.full((ACT_DIM,), float(init_log_std)))

    def mean(self, x: torch.Tensor) -> torch.Tensor:
        return self.l2(torch.tanh(self.l1(torch.tanh(self.l0(x)))))

    def log_prob(self, x: torch.Tensor, a: torch.Tensor) -> torch.Tensor:
        z = (a - self.mean(x)) / self.log_std.exp()
        return (-0.5 * z * z - self.log_std - 0.5 * np.log(2 * np.pi)).sum(-1)

    def entropy(self) -> torch.Tensor:
        return (self.log_std + 0.5 * np.log(2 * np.pi * np.e)).sum()

    def numpy_weights(self) -> dict[str, np.ndarray]:
        t = lambda p: p.detach().cpu().numpy().astype(np.float64)
        return {"w0": t(self.l0.weight).T.copy(), "b0": t(self.l0.bias), "w1": t(self.l1.weight).T.copy(),
                "b1": t(self.l1.bias), "w2": t(self.l2.weight).T.copy(), "b2": t(self.l2.bias)}


class Critic(nn.Module):
    def __init__(self, in_dim: int, hidden: int) -> None:
        super().__init__()
        self.net = nn.Sequential(_init(nn.Linear(in_dim, hidden), np.sqrt(2)), nn.Tanh(),
                                 _init(nn.Linear(hidden, hidden), np.sqrt(2)), nn.Tanh(),
                                 _init(nn.Linear(hidden, 1), 1.0))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x).squeeze(-1)


def critic_dim(max_agents: int) -> int:
    return OBS_DIM + max_agents * OBS_DIM + max_agents


def critic_inputs(obs: np.ndarray, alive: np.ndarray) -> np.ndarray:
    """(T, M, 18), (T, M) -> (T, M, critic_dim): own obs, all slots' obs, alive mask."""
    T, M, _ = obs.shape
    glob = np.broadcast_to(obs.reshape(T, 1, M * OBS_DIM), (T, M, M * OBS_DIM))
    mask = np.broadcast_to(alive.astype(np.float64)[:, None, :], (T, M, M))
    return np.concatenate([obs, glob, mask], axis=-1)


# ----------------------------------------------------------------------------- rollouts


class _Runner:
    """Maps rollout jobs over a worker pool (spawn) or in-process."""

    def __init__(self, workers: int) -> None:
        self.pool = mp.get_context("spawn").Pool(workers) if workers > 1 else None

    def map(self, jobs: list[dict[str, Any]]) -> list[dict[str, Any]]:
        return self.pool.map(collect_episode, jobs, chunksize=1) if self.pool else [collect_episode(j) for j in jobs]

    def close(self) -> None:
        if self.pool:
            self.pool.close()
            self.pool.join()


def _conflict_cap(cfg: TrainConfig, it: int) -> float:
    ramp = max(1.0, cfg.curriculum_frac * cfg.iterations)
    return cfg.conflict_start + (cfg.conflict_end - cfg.conflict_start) * min(1.0, it / ramp)


def _train_jobs(cfg: TrainConfig, it: int, weights: dict, log_std: np.ndarray) -> list[dict[str, Any]]:
    jobs = []
    cap = _conflict_cap(cfg, it)
    for ep in range(cfg.episodes_per_iter):
        g = np.random.default_rng(np.random.SeedSequence([cfg.run_seed, it, ep, 0]))
        n = int(g.integers(cfg.min_agents, cfg.max_agents + 1))
        conflict = float(g.uniform(0.0, cap))
        seed = TRAIN_SEED_BASE + cfg.run_seed * 1_000_000 + it * 1_000 + ep
        jobs.append({
            "scenario": sample_scenario(seed, n, conflict, cfg.episode_limit_steps).model_dump(mode="json"),
            "max_agents": cfg.max_agents, "rng_entropy": [cfg.run_seed, it, ep, 1], "mode": "policy",
            "weights": weights, "log_std": log_std, "shield": cfg.shield, "keep_arrays": True,
            "profile": cfg.profile,
        })
    return jobs


def _episode_stats(summaries: list[dict[str, Any]]) -> dict[str, float]:
    launched = sum(s["n_launched"] for s in summaries)
    seps = [s["min_separation_m"] for s in summaries if s["min_separation_m"] is not None]
    return {
        "completion_pct": 100.0 * sum(s["n_completed"] for s in summaries) / max(1, launched),
        "aborted": float(sum(s["n_aborted"] for s in summaries)),
        "violation_pair_steps_per_ep": float(np.mean([s["separation_violation_pair_steps"] for s in summaries])),
        "min_separation_m": float(min(seps)) if seps else float("nan"),
        "interventions_per_ep": float(np.mean([s["shield_interventions"] for s in summaries])),
        "fallbacks": float(sum(s["fallback_events"] for s in summaries)),
        "mean_steps": float(np.mean([s["steps"] for s in summaries])),
    }


def validate(cfg: TrainConfig, runner: _Runner, weights: dict) -> dict[str, float]:
    """Greedy policy on the fixed validation set (validation seeds, never trained on)."""
    scen = eval_scenarios(VAL_SEED_BASE, cfg.val_episodes, cfg.max_agents, cfg.val_conflict, cfg.episode_limit_steps)
    jobs = [{"scenario": s.model_dump(mode="json"), "max_agents": cfg.max_agents, "rng_entropy": [0],
             "mode": "greedy", "weights": weights, "shield": cfg.shield, "keep_arrays": False,
             "profile": cfg.profile} for s in scen]
    stats = _episode_stats([r["summary"] for r in runner.map(jobs)])
    # Model-selection heuristic only; it is not a safety metric.
    stats["val_score"] = stats["completion_pct"] - min(100.0, stats["violation_pair_steps_per_ep"])
    return stats


# ----------------------------------------------------------------------------- behaviour cloning


def behaviour_clone(cfg: TrainConfig, actor: Actor, runner: _Runner, log: Any) -> None:
    """Fit the actor mean to the deterministic reference controller on its own (noisy) visits."""
    jobs = []
    for ep in range(cfg.bc_episodes):
        g = np.random.default_rng(np.random.SeedSequence([cfg.run_seed, ep, 7]))
        n = int(g.integers(cfg.min_agents, cfg.max_agents + 1))
        jobs.append({
            "scenario": sample_scenario(BC_SEED_BASE + cfg.run_seed * 1_000 + ep, n, float(g.uniform(0, 0.5)),
                                        cfg.episode_limit_steps).model_dump(mode="json"),
            "max_agents": cfg.max_agents, "rng_entropy": [cfg.run_seed, ep, 8], "mode": "reference",
            "bc_noise": cfg.bc_noise, "shield": False, "keep_arrays": True,
        })
    eps = runner.map(jobs)
    x = np.concatenate([e["obs"][e["alive"]] for e in eps]).astype(np.float32)
    y = np.concatenate([e["labels"][e["alive"]] for e in eps]).astype(np.float32)
    x_t, y_t = torch.from_numpy(x), torch.from_numpy(y)
    opt = torch.optim.Adam(actor.parameters(), lr=cfg.bc_lr)
    gen = torch.Generator().manual_seed(cfg.run_seed)
    for epoch in range(cfg.bc_epochs):
        perm = torch.randperm(len(x_t), generator=gen)
        total = 0.0
        for i in range(0, len(perm), 1024):
            b = perm[i:i + 1024]
            loss = ((actor.mean(x_t[b]) - y_t[b]) ** 2).mean()
            opt.zero_grad(); loss.backward(); opt.step()
            total += float(loss) * len(b)
        if epoch in (0, cfg.bc_epochs - 1):
            log(f"bc epoch {epoch + 1}/{cfg.bc_epochs}: mse {total / len(perm):.5f} on {len(perm)} samples")


# ----------------------------------------------------------------------------- PPO


def _gae(values: np.ndarray, rew: np.ndarray, alive: np.ndarray, v_final: np.ndarray,
         final_alive: np.ndarray, gamma: float, lam: float) -> np.ndarray:
    """GAE per slot. A slot that stops acting before the end terminated (value 0);
    a slot still alive at the episode limit bootstraps from the final observation."""
    T = len(rew)
    adv = np.zeros_like(rew)
    last = np.zeros(rew.shape[1])
    for t in reversed(range(T)):
        nxt_alive = (final_alive if t == T - 1 else alive[t + 1]).astype(np.float64)
        nxt_v = v_final if t == T - 1 else values[t + 1]
        delta = rew[t] + gamma * nxt_v * nxt_alive - values[t]
        last = delta + gamma * lam * nxt_alive * last
        adv[t] = last
    return adv


def _build_batch(cfg: TrainConfig, critic: Critic, episodes: list[dict[str, Any]]) -> dict[str, torch.Tensor]:
    cin, obs, act, adv, ret = [], [], [], [], []
    for e in episodes:
        ci = critic_inputs(e["obs"], e["alive"])
        cf = critic_inputs(e["final_obs"][None], e["final_alive"][None])[0]
        with torch.no_grad():
            v = critic(torch.from_numpy(ci.astype(np.float32))).numpy().astype(np.float64)
            vf = critic(torch.from_numpy(cf.astype(np.float32))).numpy().astype(np.float64)
        a = _gae(v, e["rew"] * cfg.reward_scale, e["alive"], vf, e["final_alive"], cfg.gamma, cfg.gae_lambda)
        m = e["alive"]
        cin.append(ci[m]); obs.append(e["obs"][m]); act.append(e["act"][m]); adv.append(a[m]); ret.append((a + v)[m])
    f = lambda arrs: torch.from_numpy(np.concatenate(arrs).astype(np.float32))
    batch = {"cin": f(cin), "obs": f(obs), "act": f(act), "adv": f(adv), "ret": f(ret)}
    return batch


def _ppo_update(cfg: TrainConfig, actor: Actor, critic: Critic, opt: torch.optim.Optimizer,
                batch: dict[str, torch.Tensor], gen: torch.Generator) -> dict[str, float]:
    with torch.no_grad():
        old_logp = actor.log_prob(batch["obs"], batch["act"])
    adv = batch["adv"]
    adv = (adv - adv.mean()) / (adv.std() + 1e-8)
    n = len(adv)
    stats = {"pi_loss": 0.0, "v_loss": 0.0, "kl": 0.0, "clipfrac": 0.0}
    count = 0
    stop = False
    for _ in range(cfg.epochs):
        perm = torch.randperm(n, generator=gen)
        for i in range(0, n, cfg.minibatch):
            b = perm[i:i + cfg.minibatch]
            logp = actor.log_prob(batch["obs"][b], batch["act"][b])
            ratio = (logp - old_logp[b]).exp()
            surr = torch.min(ratio * adv[b], ratio.clamp(1 - cfg.clip_eps, 1 + cfg.clip_eps) * adv[b])
            pi_loss = -surr.mean()
            v_loss = ((critic(batch["cin"][b]) - batch["ret"][b]) ** 2).mean()
            loss = pi_loss + cfg.vf_coef * v_loss - cfg.ent_coef * actor.entropy()
            opt.zero_grad(); loss.backward()
            nn.utils.clip_grad_norm_(list(actor.parameters()) + list(critic.parameters()), cfg.max_grad_norm)
            opt.step()
            with torch.no_grad():
                kl = float((old_logp[b] - logp).mean())
                stats["pi_loss"] += float(pi_loss); stats["v_loss"] += float(v_loss); stats["kl"] += kl
                stats["clipfrac"] += float(((ratio - 1).abs() > cfg.clip_eps).float().mean())
            count += 1
            if kl > 1.5 * cfg.target_kl:
                stop = True
                break
        if stop:
            break
    return {k: v / max(1, count) for k, v in stats.items()} | {"minibatches": float(count), "n_samples": float(n)}


# ----------------------------------------------------------------------------- driver


def _git_state() -> dict[str, Any]:
    try:
        root = Path(__file__).resolve().parents[3]
        sha = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root, capture_output=True, text=True, check=True).stdout.strip()
        dirty = bool(subprocess.run(["git", "status", "--porcelain", "--", "src", "shared"], cwd=root,
                                    capture_output=True, text=True, check=True).stdout.strip())
        return {"commit": sha, "src_dirty": dirty}
    except Exception:  # noqa: BLE001 - metadata only
        return {"commit": None, "src_dirty": None}


def _metadata(cfg: TrainConfig, weights: dict, it: int, val: dict, total_steps: int, label: str) -> dict[str, Any]:
    digest = weights_sha256(weights)[:10]
    import osqp
    import scipy
    return {
        "artifact_id": f"marl-mappo-s{cfg.run_seed}-it{it}-{digest}",
        "selection": label,
        "algorithm": "MAPPO (parameter-shared actor, centralised critic, CTDE)",
        "trainer_version": TRAINER_VERSION,
        "framework": "PyTorch (custom MAPPO, no CleanRL dependency)",
        "software": {"python": platform.python_version(), "torch": torch.__version__, "numpy": np.__version__,
                     "scipy": scipy.__version__, "osqp": osqp.__version__, "platform": platform.platform()},
        "git": _git_state(),
        "training_seed": cfg.run_seed,
        "iteration": it,
        "total_env_agent_steps": total_steps,
        "n_agents_trained": [cfg.min_agents, cfg.max_agents],
        "episode_limit_steps": cfg.episode_limit_steps,
        "env_version": ENV_VERSION,
        "reward_version": REWARD_VERSION,
        "scenario_family": FAMILY_VERSION,
        "disturbance_tier_trained": ("T1 (ideal; no wind, noise, delay or dropout)" if cfg.profile == "ideal" else
                                    f"T2 profile '{cfg.profile}' (placeholder parameters, simulation.env.PROFILES)"),
        "shield_in_training_loop": cfg.shield,
        "bc_warmstart_from_reference": cfg.bc_warmstart,
        "hyperparameters": asdict(cfg),
        "validation": {k: v for k, v in val.items()},
        "validation_note": "Validation seeds only (never trained on). val_score is a model-selection heuristic, not a safety metric.",
        "limitations": [
            ("Trained and validated on T1 (ideal) dynamics only; no claim about disturbances (T2+)."
             if cfg.profile == "ideal" else
             f"Trained and validated on the simulated T2 '{cfg.profile}' profile (placeholder parameters); "
             "no claim about other disturbance types or real flight data."),
            "Observation is the 18-value ObservationV1 (nearest neighbour only).",
            "Nominal action proposer. Not a safety controller; training reward is not a safety metric.",
            "Finite-seed validation is not evidence of zero risk.",
        ],
    }


def train(cfg: TrainConfig, out_dir: Path, resume: bool = False, quiet: bool = False) -> dict[str, Any]:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(cfg.torch_threads)
    torch.manual_seed(cfg.run_seed)
    log_file = out_dir / "train_log.jsonl"

    def log(msg: str) -> None:
        if not quiet:
            print(msg, flush=True)

    actor = Actor(cfg.hidden, cfg.init_log_std)
    critic = Critic(critic_dim(cfg.max_agents), cfg.hidden)
    opt = torch.optim.Adam([{"params": actor.parameters(), "lr": cfg.lr_actor},
                            {"params": critic.parameters(), "lr": cfg.lr_critic}])
    start_it, total_steps = 0, 0
    best: dict[str, Any] = {"score": -float("inf")}
    ckpt_path = out_dir / "checkpoint.pt"
    runner = _Runner(cfg.workers)
    try:
        if resume and ckpt_path.exists():
            ck = torch.load(ckpt_path, weights_only=False)  # our own checkpoint (holds numpy arrays); never load untrusted ones
            actor.load_state_dict(ck["actor"]); critic.load_state_dict(ck["critic"]); opt.load_state_dict(ck["opt"])
            start_it, total_steps, best = ck["it"], ck["total_steps"], ck["best"]
            # The log is appended every iteration, the checkpoint only every eval_every: drop rows past it.
            kept = [r for r in log_file.read_text().splitlines() if json.loads(r)["iteration"] < start_it]
            log_file.write_text("".join(r + "\n" for r in kept))
            log(f"resumed at iteration {start_it}")
        else:
            (out_dir / "config.json").write_text(json.dumps(asdict(cfg), indent=2))
            log_file.write_text("")
            if cfg.bc_warmstart:
                behaviour_clone(cfg, actor, runner, log)

        t0 = time.perf_counter()
        for it in range(start_it, cfg.iterations):
            frac = 1.0 - (1.0 - cfg.lr_final_frac) * it / max(1, cfg.iterations - 1)
            opt.param_groups[0]["lr"], opt.param_groups[1]["lr"] = cfg.lr_actor * frac, cfg.lr_critic * frac
            w = actor.numpy_weights()
            eps = runner.map(_train_jobs(cfg, it, w, actor.log_std.detach().numpy().astype(np.float64)))
            batch = _build_batch(cfg, critic, eps)
            gen = torch.Generator().manual_seed(int(np.random.SeedSequence([cfg.run_seed, it, 99]).generate_state(1)[0]))
            upd = _ppo_update(cfg, actor, critic, opt, batch, gen)
            total_steps += int(batch["obs"].shape[0])
            stats = _episode_stats([e["summary"] for e in eps])
            ep_ret = float(np.mean([e["rew"].sum() / max(1, e["alive"].any(0).sum()) for e in eps]))
            row = {"iteration": it, "agent_steps": total_steps, "conflict_cap": _conflict_cap(cfg, it),
                   "mean_return_per_agent": ep_ret, "std": float(actor.log_std.exp().mean()),
                   "wall_s": time.perf_counter() - t0, **{f"train_{k}": v for k, v in stats.items()}, **upd}
            if (it + 1) % cfg.eval_every == 0 or it == cfg.iterations - 1:
                val = validate(cfg, runner, actor.numpy_weights())
                row.update({f"val_{k}": v for k, v in val.items()})
                if val["val_score"] >= best["score"]:  # ties go to the later (more trained) checkpoint
                    best = {"score": val["val_score"], "it": it + 1, "weights": actor.numpy_weights(), "val": val}
                torch.save({"actor": actor.state_dict(), "critic": critic.state_dict(), "opt": opt.state_dict(),
                            "it": it + 1, "total_steps": total_steps, "best": best}, ckpt_path)
            with log_file.open("a") as f:
                f.write(json.dumps(row) + "\n")
            msg = (f"it {it:4d} steps {total_steps:8d} ret {ep_ret:7.2f} compl {stats['completion_pct']:5.1f}% "
                   f"viol/ep {stats['violation_pair_steps_per_ep']:6.1f} minsep {stats['min_separation_m']:6.2f} "
                   f"std {row['std']:.3f} kl {upd['kl']:.4f} t {row['wall_s']:.0f}s")
            if "val_val_score" in row:
                msg += (f" | val compl {row['val_completion_pct']:.1f}% viol/ep {row['val_violation_pair_steps_per_ep']:.1f} "
                        f"minsep {row['val_min_separation_m']:.2f} score {row['val_val_score']:.1f}")
            log(msg)
    finally:
        runner.close()

    last_val = json.loads(log_file.read_text().splitlines()[-1])
    last_val = {k[4:]: v for k, v in last_val.items() if k.startswith("val_")}
    last_w = actor.numpy_weights()
    save_policy_artifact(out_dir / "artifact_last", last_w,
                         _metadata(cfg, last_w, cfg.iterations, last_val, total_steps, "last iteration"))
    save_policy_artifact(out_dir / "artifact", best["weights"],
                         _metadata(cfg, best["weights"], best["it"], best["val"], total_steps,
                                   "best validation val_score (validation seeds only)"))
    log(f"best validation iteration {best['it']} (val_score {best['score']:.1f}); artifact -> {out_dir / 'artifact'}")
    return {"best_iteration": best["it"], "best_val": best["val"], "total_agent_steps": total_steps}


def main(argv: list[str] | None = None) -> int:
    import argparse
    p = argparse.ArgumentParser(prog="utm-train", description="Train the MAPPO nominal policy (P2-SA-01).")
    p.add_argument("--out", type=Path, required=True, help="run directory (config, log, checkpoint, artifact/)")
    p.add_argument("--resume", action="store_true")
    for f, v in asdict(TrainConfig()).items():
        if isinstance(v, bool):
            p.add_argument(f"--{f.replace('_', '-')}", dest=f, action=argparse.BooleanOptionalAction, default=v)
        else:
            p.add_argument(f"--{f.replace('_', '-')}", dest=f, type=type(v), default=v)
    a = p.parse_args(argv)
    cfg = TrainConfig(**{f: getattr(a, f) for f in asdict(TrainConfig())})
    train(cfg, a.out, resume=a.resume)
    return 0


if __name__ == "__main__":
    sys.exit(main())
