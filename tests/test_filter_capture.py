"""``capture_internals`` changes no filter output: actions are bit-identical with capture on and off.

The capture hook records the filter's internals for the decision log (utm-decision-log). Each
test runs the same inputs twice, once inside ``capture_internals`` and once outside, and
compares every SafeAction field, every FilterDiagnostics field and the caller's latch with
``==`` (exact floats). Covers closed-loop episodes (ideal and realistic disturbances, so noisy
inputs and latches carried over steps) and a fuzz of dense random clusters near the world box.
The decision log's own checks (replay equals the record, backup paths equal the filter's
rollout) are tested at the end.
"""

import json
from pathlib import Path

import numpy as np
import pytest
from shared.contracts import WORLD_XY_MAX_M, WORLD_Z_MAX_M, WORLD_Z_MIN_M, DroneState, MARLAction, ScenarioConfig

from utm_services.evaluation import decision_log as L
from utm_services.policy.reference import ReferencePolicy
from utm_services.safety import filter as F
from utm_services.simulation.env import PROFILES, DroneEnv
from utm_services.simulation.runner import run_episode

ROOT = Path(__file__).resolve().parents[1]


def _same_latch(a: dict, b: dict) -> bool:
    return a.keys() == b.keys() and all(
        np.array_equal(a[k].axis, b[k].axis) and (a[k].signs, a[k].up, a[k].escape, a[k].origin)
        == (b[k].signs, b[k].up, b[k].escape, b[k].origin) for k in a)


def _episode(name: str, profile: str, capture: bool):
    config = ScenarioConfig.model_validate_json((ROOT / "scenarios" / name).read_text())

    def run():
        return run_episode(config, "shielded_marl", policy=ReferencePolicy(), policy_id="reference-standin",
                           make_env=lambda rules: DroneEnv(rules, disturbances=PROFILES[profile]))

    if not capture:
        return run(), None
    with F.capture_internals() as calls:
        return run(), calls


def _untimed(summary: dict) -> dict:
    return {k: v for k, v in summary.items() if "time_ms" not in k and "latency" not in k}


def _check_episode(name: str, profile: str) -> None:
    off, _ = _episode(name, profile, capture=False)
    on, calls = _episode(name, profile, capture=True)
    assert [r["returned_actions"] for r in on.records] == [r["returned_actions"] for r in off.records]
    assert [r["states"] for r in on.records] == [r["states"] for r in off.records]
    assert _untimed(on.summary) == _untimed(off.summary)
    assert [c["timestamp_ms"] for c in calls] == [r["timestamp_ms"] for r in on.records]
    assert any(c["latches"] for c in calls), "the episode must exercise the latch to test it"


@pytest.mark.parametrize(("name", "profile"), [("demo/cross4.json", "ideal"), ("sandwich-3.json", "realistic")])
def test_episode_actions_are_bit_identical_with_capture(name: str, profile: str) -> None:
    _check_episode(name, profile)


@pytest.mark.slow
@pytest.mark.parametrize("profile", ["ideal", "realistic"])
@pytest.mark.parametrize("name", ["head-on-40.json", "crossing-90.json", "sandwich-3.json", "demo/cross4.json",
                                  "demo/ring6.json", "demo/corridor5.json", "demo/city24.json"])
def test_every_scenario_is_bit_identical_with_capture(name: str, profile: str) -> None:
    _check_episode(name, profile)


def _random_step(rng, ids, ts):
    """A dense cluster of drones, some next to the world box walls, with random nominal actions."""
    centre = rng.choice([np.array([500.0, 500.0, 100.0]), np.array([8.0, WORLD_XY_MAX_M - 8.0, WORLD_Z_MIN_M + 4.0]),
                         np.array([500.0, 500.0, WORLD_Z_MAX_M - 4.0])])
    states, actions = [], []
    for aid in ids:
        p = np.clip(centre + rng.uniform(-12.0, 12.0, 3), [0.5, 0.5, WORLD_Z_MIN_M + 0.5],
                    [WORLD_XY_MAX_M - 0.5, WORLD_XY_MAX_M - 0.5, WORLD_Z_MAX_M - 0.5])
        v = rng.uniform(-8.0, 8.0, 3)
        states.append(DroneState(agent_id=aid, position_m=tuple(p), velocity_mps=tuple(v),
                                 goal_m=(500.0, 500.0, 100.0), timestamp_ms=ts))
        actions.append(MARLAction(agent_id=aid, proposed_accel_mps2=tuple(rng.uniform(-3.5, 3.5, 3)), timestamp_ms=ts))
    return states, actions


def test_fuzz_actions_diagnostics_and_latch_are_bit_identical_with_capture() -> None:
    rng = np.random.default_rng(20261024)
    for trial in range(40):
        ids = [f"d{k}" for k in range(int(rng.integers(2, 9)))]
        latch_off, latch_on = {}, {}
        for step in range(5):  # consecutive steps, so latches carried over are compared too
            states, actions = _random_step(rng, ids, 100 * step)
            off = F.apply_filter(states, actions, latch=latch_off)
            with F.capture_internals() as calls:
                on = F.apply_filter(states, actions, latch=latch_on)
            assert on[0] == off[0], (trial, step)
            assert on[1] == off[1], (trial, step)
            assert _same_latch(latch_on, latch_off), (trial, step)
            assert len(calls) == 1 and set(calls[0]["latches"]) == set(latch_on)


def test_capture_is_off_by_default_and_ends_with_its_context() -> None:
    rng = np.random.default_rng(1)
    states, actions = _random_step(rng, ["a", "b"], 0)
    assert F._CAPTURE.get() is None
    with F.capture_internals() as calls:
        F.apply_filter(states, actions, latch={})
    F.apply_filter(states, actions, latch={})
    assert len(calls) == 1 and F._CAPTURE.get() is None


def test_backup_paths_reproduce_the_filter_rollout_bit_for_bit() -> None:
    rng = np.random.default_rng(7)
    config = F.FilterConfig()
    count = 64
    centre = np.where(rng.random((count, 1)) < 0.5, [500.0, 500.0, 100.0], [6.0, 6.0, WORLD_Z_MIN_M + 6.0])
    p_i = centre + rng.uniform(-10.0, 10.0, (count, 3))
    p_j = p_i + rng.uniform(-9.0, 9.0, (count, 3))
    v_i, v_j = rng.uniform(-12.0, 12.0, (count, 3)), rng.uniform(-12.0, 12.0, (count, 3))
    axis = np.where(rng.random((count, 1)) < 0.5, 0.0, rng.standard_normal((count, 3)))
    axis /= np.maximum(np.linalg.norm(axis, axis=1, keepdims=True), 1e-12)
    up = rng.choice([0.0, 1.0, -1.0], count)
    rel, _, _, used, _ = F._backup_rollout(p_i, p_j, v_i, v_j, config, axis, up)
    path_i, path_j, used_numpy = L.backup_paths(p_i, p_j, v_i, v_j, config, axis, up)
    assert np.array_equal(used_numpy, used)
    assert np.array_equal(path_i - path_j, rel, equal_nan=True)


def test_decision_log_replays_the_run_and_refuses_a_tampered_record(tmp_path: Path) -> None:
    config = ScenarioConfig.model_validate_json((ROOT / "scenarios" / "demo" / "cross4.json").read_text())
    result = run_episode(config, "shielded_marl", policy=ReferencePolicy(), policy_id="reference-standin")
    (tmp_path / "manifest.json").write_text(json.dumps(result.manifest, default=str))
    (tmp_path / "records.jsonl").write_text("".join(json.dumps(r) + "\n" for r in result.records))
    header, steps = L.read_decision_log(L.write_decision_log(tmp_path))
    assert header["filter_on"] and header["steps"] == len(result.records) == len(steps)
    assert any(p.get("backup") for s in steps for p in s["pairs"])
    assert max(s["interventions"] for s in steps) > 0
    first = next(s for s in steps if any(d["latch"] for d in s["drones"].values()))
    assert all(d["cluster"] is not None for d in first["drones"].values() if d["latch"])

    records = result.records
    k = next(i for i, s in enumerate(steps) if s["interventions"])
    records[k]["returned_actions"][0]["safe_accel_mps2"][0] += 1e-12
    (tmp_path / "records.jsonl").write_text("".join(json.dumps(r) + "\n" for r in records))
    with pytest.raises(L.ReplayMismatch):
        L.write_decision_log(tmp_path)
