"""Matched evaluation of a trained policy artifact (P2-SA-01 -> P3 inputs).

Flies the deterministic reference, the unshielded learned policy and the shielded
learned policy on identical scenarios through the project's own ``run_episode``
(so policy, filter, env and metrics are the deployed code paths, not the training
fast path). Reports measured quantities only: completion, minimum separation,
separation-violation pair-steps (v1 threshold, ``INTERFACES.md``), time inside
``d_eff``, interventions, infeasible steps and fallbacks.

This is a finite-seed measurement on simulated ideal (T1) dynamics unless
``--profile`` says otherwise. It is not a guarantee, and zero violations in a
suite does not establish zero risk.

Suites
------
heldout   procedural family, sizes 2..4, conflict 0.8, seeds from --seed-base (default: the
          development TEST seeds, which are not clean held-out for the final design)
shipped   scenarios/*.json (head-on-40, crossing-90, sandwich-3)
swap      6 drones; d0 and d1 swap places --swap-gap metres apart (default 25), the other 4 fly random routes; seeds 24,000,000+
six       6 drones per episode, conflict 0.8, seeds 18,000,000+ (the policy was trained on 2-4)
dense16   16 drones, random starts and goals (same generator as tests/test_random_traffic.py);
          far outside the trained team size
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np
from shared.contracts import ScenarioConfig

from utm_services.policy.learned import LearnedPolicy
from utm_services.policy.reference import ReferencePolicy
from utm_services.safety.filter import FilterConfig
from utm_services.simulation.env import PROFILES, DroneEnv
from utm_services.simulation.runner import VARIANTS, run_episode
from utm_services.training.scenarios import (
    TEST_SEED_BASE,
    eval_scenarios,
    fixed_size_scenarios,
    swap_scenarios,
)

SUITES = ("heldout", "shipped", "dense16", "six", "swap")
# Baseline for context: the reference controller behind the same filter. Not a learned policy.
SHIELDED_REFERENCE = "shielded_reference"
ALL_VARIANTS = (*VARIANTS, SHIELDED_REFERENCE)
_REPO = Path(__file__).resolve().parents[3]


def random_traffic(seed: int, n: int = 16) -> ScenarioConfig:
    """Same generator as tests/test_random_traffic.py (kept in sync by hand; it is a test helper)."""
    rng = np.random.default_rng(seed)
    points: list[np.ndarray] = []
    while len(points) < 2 * n:
        p = np.array([*rng.uniform(300.0, 700.0, 2), rng.uniform(80.0, 120.0)])
        if all(np.linalg.norm(p - q) >= 30.0 for q in points):
            points.append(p)
    drones = [{"agent_id": f"d{k}", "initial_position_m": tuple(points[k]),
               "initial_velocity_mps": (0.0, 0.0, 0.0), "goal_m": tuple(points[n + k])} for k in range(n)]
    return ScenarioConfig(scenario_id=f"random-{n}", seed=seed, episode_limit_steps=1000, drones=drones)


def suite_scenarios(name: str, count: int, seed_base: int | None = None, swap_gap: float = 25.0) -> list[ScenarioConfig]:
    if name == "heldout":
        return eval_scenarios(TEST_SEED_BASE if seed_base is None else seed_base, count, max_agents=4, conflict=0.8, episode_limit_steps=800)
    if name == "shipped":
        return [ScenarioConfig.model_validate_json(p.read_text()) for p in sorted((_REPO / "scenarios").glob("*.json"))]
    if name == "swap":
        return swap_scenarios(24_000_000 if seed_base is None else seed_base, count, 6, swap_gap)
    if name == "six":
        return fixed_size_scenarios(18_000_000 if seed_base is None else seed_base, count, 6)
    if name == "dense16":
        return [random_traffic(seed) for seed in range(seed_base or 0, (seed_base or 0) + count)]
    raise ValueError(f"unknown suite '{name}'")


_FIELDS = ("n_launched", "n_completed", "n_aborted", "n_incomplete", "completion_pct", "steps", "min_separation_m",
           "separation_violation_pair_steps", "time_inside_d_eff_s", "max_violation_depth_m", "shield_interventions",
           "qp_infeasible_steps", "fallback_events", "mean_step_time_ms", "p95_step_time_ms",
           "mean_policy_latency_ms", "p95_policy_latency_ms")


def evaluate(policy: LearnedPolicy, scenarios: list[ScenarioConfig], suite: str, profile: str = "ideal",
             variants: tuple[str, ...] = ALL_VARIANTS, filter_config: FilterConfig | None = None) -> list[dict[str, Any]]:
    rows = []
    make_env = lambda rules: DroneEnv(rules, disturbances=PROFILES[profile])
    for config in scenarios:
        for variant in variants:
            if variant == SHIELDED_REFERENCE:
                result = run_episode(config, "shielded_marl", policy=ReferencePolicy(), policy_id="reference-p1-sa-01-pid",
                                     filter_config=filter_config, keep_records=False, make_env=make_env)
            else:
                result = run_episode(config, variant, policy=policy, policy_id=policy.artifact_id,
                                     filter_config=filter_config, keep_records=False, make_env=make_env)
            rows.append({"suite": suite, "profile": profile, "scenario_id": config.scenario_id, "seed": config.seed,
                         "n_drones": len(config.drones), "variant": variant, "policy_id": result.manifest["policy_id"],
                         **{k: result.summary[k] for k in _FIELDS}})
    return rows


def aggregate(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out = []
    for suite in dict.fromkeys(r["suite"] for r in rows):
        for variant in ALL_VARIANTS:
            rs = [r for r in rows if r["suite"] == suite and r["variant"] == variant]
            if not rs:
                continue
            launched = sum(r["n_launched"] for r in rs)
            out.append({
                "suite": suite, "variant": variant, "episodes": len(rs),
                "completion_pct": 100.0 * sum(r["n_completed"] for r in rs) / launched,
                "aborted": sum(r["n_aborted"] for r in rs),
                "min_separation_m": min(r["min_separation_m"] for r in rs if r["min_separation_m"] is not None),
                "violation_pair_steps": sum(r["separation_violation_pair_steps"] for r in rs),
                "episodes_with_violation": sum(r["separation_violation_pair_steps"] > 0 for r in rs),
                "episodes_inside_d_eff": sum(r["time_inside_d_eff_s"] > 0 for r in rs),
                "time_inside_d_eff_s": sum(r["time_inside_d_eff_s"] for r in rs),
                "interventions": sum(r["shield_interventions"] for r in rs),
                "infeasible_steps": sum(r["qp_infeasible_steps"] for r in rs),
                "fallbacks": sum(r["fallback_events"] for r in rs),
                "mean_steps": float(np.mean([r["steps"] for r in rs])),
                "p95_step_ms_max": max(r["p95_step_time_ms"] for r in rs),
            })
    return out


def markdown_table(agg: list[dict[str, Any]]) -> str:
    cols = [("suite", "suite"), ("variant", "variant"), ("episodes", "eps"), ("completion_pct", "completion %"),
            ("aborted", "aborted"), ("min_separation_m", "min sep (m)"), ("violation_pair_steps", "viol. pair-steps"),
            ("episodes_with_violation", "eps w/ viol."), ("episodes_inside_d_eff", "eps inside d_eff"),
            ("interventions", "interventions"), ("infeasible_steps", "infeasible"), ("fallbacks", "fallbacks"),
            ("mean_steps", "mean steps"), ("p95_step_ms_max", "max p95 step (ms)")]
    fmt = lambda v: f"{v:.2f}" if isinstance(v, float) else str(v)
    lines = ["| " + " | ".join(h for _, h in cols) + " |", "|" + "---|" * len(cols)]
    lines += ["| " + " | ".join(fmt(r[k]) for k, _ in cols) + " |" for r in agg]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="utm-eval-policy", description=__doc__.splitlines()[0])
    p.add_argument("artifact", type=Path, help="policy artifact directory (actor.npz + metadata.json)")
    p.add_argument("--suite", choices=SUITES, action="append", help="repeatable; default: heldout and shipped")
    p.add_argument("--episodes", type=int, default=30, help="scenarios per procedural suite (heldout, dense16)")
    p.add_argument("--seed-base", type=int, default=None,
                   help="first seed of the procedural suites (heldout default 9000000, dense16 default 0)")
    p.add_argument("--swap-gap", type=float, default=25.0, help="distance (m) between the two swapping drones in the swap suite")
    p.add_argument("--profile", choices=PROFILES, default="ideal")
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args(argv)

    policy = LearnedPolicy.load(a.artifact)
    suites = a.suite or ["heldout", "shipped"]
    rows: list[dict[str, Any]] = []
    for suite in suites:
        rows += evaluate(policy, suite_scenarios(suite, a.episodes, a.seed_base, a.swap_gap), suite, a.profile)
        print(f"{suite}: {len(rows)} episode rows", file=sys.stderr, flush=True)
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "episodes.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    agg = aggregate(rows)
    table = markdown_table(agg)
    manifest = {"artifact_id": policy.artifact_id, "weights_sha256": policy.metadata["weights_sha256"],
                "suites": suites, "episodes": a.episodes, "swap_gap_m": a.swap_gap, "seed_base": a.seed_base, "profile": a.profile,
                "filter_config": vars(FilterConfig()), "git": policy.metadata.get("git")}
    (a.out / "manifest.json").write_text(json.dumps(manifest, indent=2, default=str))
    (a.out / "summary.md").write_text(
        f"# Evaluation of `{policy.artifact_id}` (profile: {a.profile})\n\n{table}\n\n"
        "Measured on seeded simulation only. Not a guarantee; zero violations here do not establish zero risk.\n")
    print(table)
    return 0


if __name__ == "__main__":
    sys.exit(main())
