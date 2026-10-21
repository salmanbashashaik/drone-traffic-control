"""Tests for the v1 simulator (simulation/env.py) and variant runner (simulation/runner.py)."""

import pytest
from shared.contracts import MAX_SPEED_MPS, SafeAction, ScenarioConfig

from utm_services.api.app import _default_rules
from utm_services.policy.reference import compute_reference_action
from utm_services.simulation.env import GOAL_RADIUS_M, DroneEnv
from utm_services.simulation.runner import run_episode


def _drone(agent_id, pos, goal, vel=(0.0, 0.0, 0.0)):
    return {"agent_id": agent_id, "initial_position_m": pos, "initial_velocity_mps": vel, "goal_m": goal}


def _config(*drones, limit=200):
    return ScenarioConfig(scenario_id="t", seed=7, episode_limit_steps=limit, drones=list(drones))


def _act(agent_id, ts, accel=(0.0, 0.0, 0.0)):
    return SafeAction(
        agent_id=agent_id, safe_accel_mps2=accel, timestamp_ms=ts, shield_active=False,
        h_min_m2=None, qp_feasible=True, fallback_used=False,
    )


PAIR = (
    _drone("a", (100.0, 100.0, 100.0), (300.0, 100.0, 100.0)),
    _drone("b", (100.0, 200.0, 100.0), (300.0, 200.0, 100.0)),
)


# ---------------------------------------------------------------- env


def test_reset_observation_layout_and_sentinels() -> None:
    env = DroneEnv()
    obs, meta = env.reset(_config(PAIR[0]))
    o = obs["a"].root
    assert len(o) == 18
    assert list(o[6:9]) == [200.0, 0.0, 0.0]  # goal displacement
    assert list(o[9:15]) == [0.0] * 6 and o[15] == -1.0  # no neighbor
    assert o[16] == 0.0 and o[17] == 1.0  # no prior shield, no pair
    assert meta["seed"] == 7 and meta["goal_radius_m"] == GOAL_RADIUS_M

    obs, _ = env.reset(_config(*PAIR))
    assert obs["a"].root[15] == 100.0
    assert list(obs["a"].root[9:12]) == [0.0, 100.0, 0.0]


def test_same_config_reproduces_episode() -> None:
    a = run_episode(_config(*PAIR), "reference")
    b = run_episode(_config(*PAIR), "reference")
    assert a.records == b.records
    def strip(s):
        return {k: v for k, v in s.items() if "time" not in k and "latency" not in k}

    assert strip(a.summary) == strip(b.summary)


def test_goal_completion_ends_episode() -> None:
    s = run_episode(_config(*PAIR), "reference").summary
    assert s["n_completed"] == 2 and s["n_incomplete"] == 0
    assert s["completion_pct"] == 100.0 and s["steps"] < 200


def test_out_of_bounds_aborts_without_observation() -> None:
    env = DroneEnv()
    env.reset(_config(_drone("a", (999.0, 500.0, 100.0), (500.0, 500.0, 100.0), vel=(15.0, 0.0, 0.0))))
    r = env.step([_act("a", 0)])
    assert r.terminations["a"] and "a" not in r.observations
    assert r.infos["a"]["abort_reason"] == "out_of_bounds"
    assert env.done and env.summary()["n_aborted"] == 1


def test_truncation_at_episode_limit() -> None:
    env = DroneEnv()
    env.reset(_config(PAIR[0], limit=2))
    env.step([_act("a", 0)])
    r = env.step([_act("a", 100)])
    assert r.truncations["a"] and not r.terminations["a"] and env.done
    assert env.summary()["n_incomplete"] == 1


@pytest.mark.parametrize(
    "actions, err",
    [
        ([_act("a", 0)], ValueError),  # missing b
        ([_act("a", 0), _act("a", 0)], ValueError),  # duplicate
        ([_act("a", 0), _act("b", 100)], ValueError),  # stale timestamp
        ([_act("a", 0), _act("b", 0), _act("c", 0)], ValueError),  # unknown agent
    ],
)
def test_step_rejects_invalid_actions(actions, err) -> None:
    env = DroneEnv(strict=True)
    env.reset(_config(*PAIR))
    with pytest.raises(err):
        env.step(actions)


def test_speed_cap_scales_velocity_and_logs() -> None:
    env = DroneEnv()
    env.reset(_config(_drone("a", (100.0, 100.0, 100.0), (900.0, 100.0, 100.0), vel=(15.0, 0.0, 0.0))))
    r = env.step([_act("a", 0, accel=(3.5, 0.0, 0.0))])
    assert r.infos["a"]["speed_capped"]
    assert env.states()[0].velocity_mps[0] <= MAX_SPEED_MPS
    assert env.summary()["speed_cap_events"] == 1


def test_separation_violation_is_counted() -> None:
    env = DroneEnv()
    env.reset(_config(
        _drone("a", (100.0, 100.0, 100.0), (900.0, 100.0, 100.0)),
        _drone("b", (103.0, 100.0, 100.0), (900.0, 200.0, 100.0)),
    ))
    r = env.step([_act("a", 0), _act("b", 0)])
    assert r.infos["a"]["separation_violation"]
    s = env.summary()
    assert s["min_separation_m"] == pytest.approx(3.0)
    assert s["separation_violation_pair_steps"] == 2  # reset + one step


def test_geofence_incursion_logged_not_enforced() -> None:
    env = DroneEnv(_default_rules())  # restricted-1: r=50 m around (500, 500)
    _, meta = env.reset(_config(_drone("a", (500.0, 500.0, 100.0), (900.0, 500.0, 100.0))))
    r = env.step([_act("a", 0)])
    assert meta["rules_version"] == "sim-rules-v1"
    assert r.infos["a"]["geofence_incursion"] and not r.terminations["a"]
    assert env.summary()["geofence_incursion_agent_steps"] == 1


# ---------------------------------------------------------------- runner


def test_runner_variants_share_scenario_and_label_policy() -> None:
    ref = run_episode(_config(*PAIR), "reference")
    unsh = run_episode(_config(*PAIR), "unshielded_marl", policy=compute_reference_action, policy_id="stand-in")
    sh = run_episode(_config(*PAIR), "shielded_marl", policy=compute_reference_action, policy_id="stand-in")
    assert ref.manifest["policy_id"] == "reference-p1-sa-01" and ref.manifest["filter"] is None
    assert sh.manifest["filter"] is not None
    # Far-apart pair: the filter passes the reference controller through.
    assert ref.summary["steps"] == unsh.summary["steps"] == sh.summary["steps"]
    assert sh.summary["shield_interventions"] == 0
    assert all(not a["shield_active"] for rec in unsh.records for a in rec["returned_actions"])


def test_runner_rejects_bad_variant_or_missing_policy() -> None:
    with pytest.raises(ValueError):
        run_episode(_config(*PAIR), "bogus")
    with pytest.raises(ValueError):
        run_episode(_config(*PAIR), "shielded_marl")
