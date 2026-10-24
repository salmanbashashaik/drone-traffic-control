"""Fast smoke tests for the P2-SA-01 training runtime: artifact checks, rollout parity, 3D replay."""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import numpy as np
import pytest

from utm_services.policy.learned import ArtifactError, LearnedPolicy
from utm_services.simulation.runner import run_episode
from utm_services.training.rollout import collect_episode
from utm_services.training.scenarios import swap_scenario

ARTIFACT = Path(__file__).resolve().parents[1] / "artifacts" / "policies" / "marl-mappo-s42-it300-08e745b853"


@pytest.fixture
def artifact(tmp_path: Path) -> Path:
    return Path(shutil.copytree(ARTIFACT, tmp_path / "artifact"))


def _short_scenario():
    return swap_scenario(7, n_agents=2, gap_m=25.0, episode_limit_steps=60)


def test_load_accepts_the_shipped_artifact(artifact: Path) -> None:
    assert LearnedPolicy.load(artifact).artifact_id


def test_load_refuses_tampered_weights(artifact: Path) -> None:
    with np.load(artifact / "actor.npz") as npz:
        weights = {k: np.array(npz[k]) for k in npz.files}
    weights["b2"] = weights["b2"] + 1e-6
    np.savez(artifact / "actor.npz", **weights)
    with pytest.raises(ArtifactError, match="weight hash"):
        LearnedPolicy.load(artifact)


def test_load_refuses_a_wrong_normalisation_fingerprint(artifact: Path) -> None:
    meta_path = artifact / "metadata.json"
    meta = json.loads(meta_path.read_text())
    meta["obs_normalization_fingerprint"] = "0" * 64
    meta_path.write_text(json.dumps(meta))
    with pytest.raises(ArtifactError, match="fingerprint"):
        LearnedPolicy.load(artifact)


def test_load_refuses_a_missing_metadata_file(artifact: Path) -> None:
    (artifact / "metadata.json").unlink()
    with pytest.raises(ArtifactError, match="metadata.json missing"):
        LearnedPolicy.load(artifact)


def test_fast_rollout_matches_run_episode_on_integer_outcomes() -> None:
    config = _short_scenario()
    policy = LearnedPolicy.load(ARTIFACT)
    fast = collect_episode({
        "scenario": config.model_dump(), "max_agents": 2, "rng_entropy": [0], "mode": "greedy",
        "weights": policy.weights, "shield": False, "keep_arrays": False,
    })["summary"]
    full = run_episode(config, "unshielded_marl", policy=policy, policy_id=policy.artifact_id,
                       keep_records=False).summary
    # The env summary's integer outcomes; run_episode adds filter counters on top of them.
    ints = {k for k, v in fast.items() if isinstance(v, int) and not isinstance(v, bool)}
    assert {"steps", "n_completed"} <= ints
    assert {k: fast[k] for k in ints} == {k: full[k] for k in ints}


def test_replay_html_writes_the_3d_viewer(tmp_path: Path) -> None:
    pytest.importorskip("matplotlib")
    from utm_services.training.replay import main

    scenario = tmp_path / "scenario.json"
    scenario.write_text(_short_scenario().model_dump_json())
    out = tmp_path / "out"
    assert main([str(scenario), "--artifact", str(ARTIFACT), "--buildings", "0", "--html", "--out", str(out)]) == 0
    html = (out / "replay3d.html").read_text(encoding="utf-8")
    assert "<html" in html.lower()
