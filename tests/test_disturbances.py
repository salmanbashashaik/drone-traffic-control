"""Sprint 1 PR F: T2 disturbance injectors, degraded command handling and seeding."""

import json
from pathlib import Path

import numpy as np
import pytest
from shared.contracts import MAX_ACCEL_COMPONENT_MPS2, SafeAction, ScenarioConfig

from utm_services.policy.reference import ReferencePolicy
from utm_services.simulation import runner
from utm_services.simulation.env import (
    DT_S,
    PROFILES,
    T2_NOMINAL,
    T2_REALISTIC,
    Disturbances,
    DroneEnv,
)
from utm_services.simulation.runner import VARIANTS, run_episode

SCENARIOS = Path(__file__).resolve().parents[1] / "scenarios"


def _drone(agent_id, pos, vel=(0.0, 0.0, 0.0), goal=(900.0, 900.0, 100.0)):
    return {"agent_id": agent_id, "initial_position_m": pos, "initial_velocity_mps": vel, "goal_m": goal}


def _config(*drones, seed=0, limit=200):
    return ScenarioConfig(scenario_id="t", seed=seed, episode_limit_steps=limit, drones=list(drones))


def _act(agent_id, ts, accel=(0.0, 0.0, 0.0)):
    return SafeAction(agent_id=agent_id, safe_accel_mps2=accel, timestamp_ms=ts, shield_active=False,
                      h_min_m2=None, qp_feasible=True, fallback_used=False)


def _velocity(env, agent_id):
    return np.array(next(s.velocity_mps for s in env.states() if s.agent_id == agent_id))


def _position(env, agent_id):
    return np.array(next(s.position_m for s in env.states() if s.agent_id == agent_id))


# ------------------------------------------------------- degraded commands


def test_strict_mode_rejects_missing_and_stale_actions() -> None:
    env = DroneEnv(strict=True)
    env.reset(_config(_drone("a", (100.0, 100.0, 100.0)), _drone("b", (100.0, 200.0, 100.0))))
    with pytest.raises(ValueError):
        env.step([_act("a", 0)])
    with pytest.raises(ValueError):
        env.step([_act("a", 0), _act("b", 100)])


def test_missing_command_holds_last_action_then_brakes_to_hover() -> None:
    env = DroneEnv()  # hold_steps defaults to 3 (300 ms)
    env.reset(_config(_drone("a", (500.0, 500.0, 100.0), vel=(10.0, 0.0, 0.0))))

    modes = [env.step([_act("a", 0, (1.0, 0.0, 0.0))]).infos["a"]["command"]]
    speeds = [_velocity(env, "a")[0]]
    for _ in range(40):  # 10.4 m/s at 3.5 m/s^2 stops within 30 steps
        modes.append(env.step([]).infos["a"]["command"])
        speeds.append(_velocity(env, "a")[0])

    assert modes[:5] == ["received", "hold", "hold", "hold", "brake"]
    assert set(modes[4:]) == {"brake"}
    assert speeds[:4] == pytest.approx([10.1, 10.2, 10.3, 10.4])
    # Max deceleration within the per-axis limit, then zero velocity.
    assert speeds[4] == pytest.approx(10.4 - MAX_ACCEL_COMPONENT_MPS2 * DT_S)
    assert speeds[-1] == 0.0
    assert speeds[-5:] == [0.0] * 5  # hovers
    assert env.summary()["degraded_agent_steps"] == 40


def test_stale_command_is_degraded_not_an_error() -> None:
    env = DroneEnv()
    env.reset(_config(_drone("a", (500.0, 500.0, 100.0))))
    info = env.step([_act("a", 100, (1.0, 0.0, 0.0))]).infos["a"]

    assert info["command"] == "brake"  # no earlier valid command to hold
    assert np.allclose(_velocity(env, "a"), 0.0)


def test_degraded_drone_uses_no_information_about_other_drones() -> None:
    def track(*drones):
        env = DroneEnv()
        env.reset(_config(*drones))
        env.step([_act(d["agent_id"], 0, (0.5, 0.2, 0.0)) for d in drones])
        out = []
        for _ in range(30):
            env.step([])
            out.append(_position(env, "a"))
        return np.array(out)

    a = _drone("a", (500.0, 500.0, 100.0), vel=(8.0, 3.0, 0.0))
    near = _drone("b", (520.0, 505.0, 100.0), vel=(-8.0, 0.0, 0.0))
    assert np.array_equal(track(a), track(a, near))


def test_duplicate_and_unknown_actions_still_raise() -> None:
    env = DroneEnv()
    env.reset(_config(_drone("a", (100.0, 100.0, 100.0))))
    with pytest.raises(ValueError):
        env.step([_act("a", 0), _act("a", 0)])
    with pytest.raises(ValueError):
        env.step([_act("a", 0), _act("z", 0)])


# ------------------------------------------------------------- injectors


def test_wind_is_steady_seeded_and_bounded() -> None:
    def wind_run(seed):
        env = DroneEnv(disturbances=Disturbances(wind_max_mps2=1.0))
        _, meta = env.reset(_config(_drone("a", (500.0, 500.0, 100.0)), seed=seed))
        v = []
        for k in range(5):
            env.step([_act("a", k * 100)])
            v.append(_velocity(env, "a"))
        return np.array(meta["disturbances"]["wind_mps2"]), np.array(v)

    w, v = wind_run(3)
    assert np.all(np.abs(w) <= 1.0) and np.any(w != 0.0)
    assert np.allclose(np.diff(v, axis=0), w * DT_S)  # same wind every step
    assert np.array_equal(wind_run(3)[0], w)
    assert not np.array_equal(wind_run(4)[0], w)


def test_observation_noise_changes_observation_not_ground_truth() -> None:
    env = DroneEnv(disturbances=Disturbances(obs_pos_sigma_m=0.5, obs_vel_sigma_mps=0.1))
    obs, _ = env.reset(_config(_drone("a", (500.0, 500.0, 100.0), vel=(5.0, 0.0, 0.0)), limit=400))
    err_p, err_v = [], []
    for k in range(400):
        o = obs["a"].root
        err_p.append(np.array(o[0:3]) - _position(env, "a"))
        err_v.append(np.array(o[3:6]) - _velocity(env, "a"))
        obs = env.step([_act("a", k * 100)]).observations

    assert np.std(err_p) == pytest.approx(0.5, rel=0.1)
    assert np.std(err_v) == pytest.approx(0.1, rel=0.1)
    assert np.allclose(_velocity(env, "a"), (5.0, 0.0, 0.0))


def test_observation_delay_delivers_the_observation_from_k_steps_ago() -> None:
    env = DroneEnv(disturbances=Disturbances(obs_delay_steps=2))
    obs, _ = env.reset(_config(_drone("a", (500.0, 500.0, 100.0), vel=(10.0, 0.0, 0.0))))
    xs = [obs["a"].root[0]]
    for k in range(5):
        xs.append(env.step([_act("a", k * 100)]).observations["a"].root[0])

    assert xs == pytest.approx([500.0, 500.0, 500.0, 501.0, 502.0, 503.0])


@pytest.mark.parametrize("p, expected", [(0.0, 0), (1.0, 20)])
def test_command_dropout_degrades_dropped_commands(p, expected) -> None:
    env = DroneEnv(disturbances=Disturbances(dropout_p=p))
    env.reset(_config(_drone("a", (500.0, 500.0, 100.0)), limit=20))
    for k in range(20):
        env.step([_act("a", k * 100, (1.0, 0.0, 0.0))])

    s = env.summary()
    assert s["command_dropouts"] == expected
    assert s["degraded_agent_steps"] == expected


@pytest.mark.parametrize("profile", ["nominal", "realistic"])
def test_t2_manifest_records_tier_profile_and_injectors(profile) -> None:
    _, meta = DroneEnv(disturbances=PROFILES[profile]).reset(_config(_drone("a", (500.0, 500.0, 100.0))))
    assert meta["condition_tier"] == "T2"
    assert meta["disturbance_profile"] == profile
    assert meta["disturbances"]["wind_max_mps2"] == 1.0
    assert meta["hold_steps"] == 3


def test_realistic_profile_values() -> None:
    assert T2_REALISTIC == Disturbances(wind_max_mps2=1.0, obs_pos_sigma_m=1.5, obs_vel_sigma_mps=0.3,
                                        obs_delay_steps=3, dropout_p=0.1)
    assert PROFILES["nominal"] is T2_NOMINAL


def test_filter_receives_the_disturbed_states_the_policy_sees(monkeypatch) -> None:
    seen = []
    real = runner.apply_filter

    def spy(states, proposed, cfg):
        seen.append(states)
        return real(states, proposed, cfg)

    monkeypatch.setattr(runner, "apply_filter", spy)
    d = Disturbances(obs_pos_sigma_m=0.5, obs_delay_steps=2)
    res = run_episode(_config(_drone("a", (500.0, 500.0, 100.0), vel=(10.0, 0.0, 0.0)),
                              _drone("b", (500.0, 600.0, 100.0)), limit=10),
                      "shielded_marl", policy=ReferencePolicy(), policy_id="reference-standin",
                      make_env=lambda rules: DroneEnv(rules, disturbances=d))

    for states, rec in zip(seen, res.records, strict=True):
        assert [s.timestamp_ms for s in states] == [rec["timestamp_ms"]] * 2  # filter time check still holds
        assert [s.model_dump(mode="json") for s in states] == rec["observed_states"]
        assert rec["observed_states"] != rec["states"]  # noisy, not the truth
    # Delay: the filter's x for "a" lags truth by about 2 steps (2 m at 10 m/s), not 0.
    lag = res.records[5]["states"][0]["position_m"][0] - res.records[5]["observed_states"][0]["position_m"][0]
    assert 1.0 < lag < 3.0


# ------------------------------------------------------------------ seeds


def _t2_run(seed):
    cfg = ScenarioConfig.model_validate_json((SCENARIOS / "crossing-90.json").read_text())
    cfg = cfg.model_copy(update={"seed": seed})
    res = run_episode(cfg, "reference", make_env=lambda rules: DroneEnv(rules, disturbances=T2_NOMINAL))
    return [json.dumps(r["states"]) for r in res.records]


def test_same_seed_identical_different_seed_different() -> None:
    assert _t2_run(0) == _t2_run(0)
    assert _t2_run(0) != _t2_run(1)


# --------------------------------------------------------- T2 all variants


@pytest.mark.parametrize("profile", ["nominal", "realistic"])
@pytest.mark.parametrize("variant", VARIANTS)
def test_t2_runs_every_variant_to_the_end(variant, profile) -> None:
    cfg = ScenarioConfig.model_validate_json((SCENARIOS / "crossing-90.json").read_text())
    s = run_episode(cfg, variant, policy=ReferencePolicy(), policy_id="reference-standin",
                    make_env=lambda rules: DroneEnv(rules, disturbances=PROFILES[profile]), keep_records=False).summary
    assert s["n_incomplete"] + s["n_completed"] + s["n_aborted"] == 2


@pytest.mark.parametrize("scenario", ["head-on-40", "crossing-90"])
def test_reference_completes_under_steady_wind(scenario) -> None:
    """The integral term removes the steady wind offset, so every drone reaches its goal radius."""
    cfg = ScenarioConfig.model_validate_json((SCENARIOS / f"{scenario}.json").read_text())
    wind = Disturbances(wind_max_mps2=1.0)
    for seed in range(5):
        s = run_episode(cfg.model_copy(update={"seed": seed}), "reference",
                        make_env=lambda rules: DroneEnv(rules, disturbances=wind), keep_records=False).summary
        assert s["n_completed"] == s["n_launched"], seed

