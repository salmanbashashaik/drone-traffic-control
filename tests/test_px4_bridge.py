"""Tests for the PX4 SITL bridge (integration/px4_bridge.py) that run without PX4 or MAVSDK."""

import math

import numpy as np
import pytest
from shared.contracts import MAX_SPEED_MPS, DroneState, MARLAction, SafeAction, ScenarioConfig

from utm_services.integration import px4_bridge
from utm_services.integration.px4_bridge import (
    ALT_OFFSET_M,
    DT_S,
    WORLD_ORIGIN_XY,
    Encounter,
    check_shared_origin,
    h_min_m2,
    ned_to_ours,
    observation,
    ours_to_ned_vector,
    run_encounter,
    setpoint,
    sitl_height,
    summarize,
    warm_up_filter,
)
from utm_services.safety.filter import FilterConfig
from utm_services.simulation.env import DroneEnv

R_EARTH = 6378137.0
RUNNER_RECORD_KEYS = {"episode_id", "scenario_id", "seed", "controller_variant", "timestamp_ms", "states",
                      "observed_states", "proposed_actions", "returned_actions", "infos"}


# ---------------------------------------------------------------- frames


def test_ned_to_ours_axes_offset_and_altitude() -> None:
    pos, vel = ned_to_ours((-40.0, 10.0), (1.0, 2.0, -10.0), (1.0, 2.0, -3.0))
    assert pos == (WORLD_ORIGIN_XY[0] - 40.0 + 2.0, WORLD_ORIGIN_XY[1] + 10.0 + 1.0, 10.0 + ALT_OFFSET_M)
    assert vel == (2.0, 1.0, 3.0)  # east, north, up


def test_vector_round_trip_and_sitl_height() -> None:
    _, vel = ned_to_ours((0.0, 0.0), (0.0, 0.0, 0.0), (1.5, -2.0, 0.7))
    assert ours_to_ned_vector(vel) == pytest.approx((1.5, -2.0, 0.7))
    assert sitl_height(40.0) == 40.0 - ALT_OFFSET_M
    assert sitl_height(30.0) > 0.0  # the world floor is above the SITL ground


def _gps(ref: tuple[float, float], east: float, north: float) -> tuple[float, float]:
    return (ref[0] + math.degrees(north / R_EARTH),
            ref[1] + math.degrees(east / (R_EARTH * math.cos(math.radians(ref[0])))))


def test_check_shared_origin_passes_and_fails() -> None:
    ref = (47.397971, 8.546164)
    spawns = ((-40.0, 0.0), (40.0, 0.0), (0.0, 30.0))
    origins = [_gps(ref, e - spawns[0][0], n - spawns[0][1]) for e, n in spawns]
    ground = [0.02, -0.03, 0.05]
    assert max(check_shared_origin(spawns, origins, ground)) < 0.1
    with pytest.raises(ValueError, match="GPS origins"):
        check_shared_origin(spawns, origins, [0.0, 0.0, -0.4])  # vehicle 2's frame 0.4 m higher
    origins[2] = _gps(ref, 40.0 + 0.3, 30.0)  # vehicle 2 booted 0.3 m east of its spawn offset
    with pytest.raises(ValueError, match="GPS origins"):
        check_shared_origin(spawns, origins, ground)


# ---------------------------------------------------------------- setpoint mapping


def test_setpoint_is_v_plus_a_dt_in_ned_with_feedforward() -> None:
    vel, acc, capped = setpoint((1.0, 2.0, 3.0), (1.0, 0.0, -1.0), 0.1)
    assert vel == pytest.approx((2.0, 1.1, -2.9))
    assert acc == pytest.approx((0.0, 1.0, 1.0))
    assert not capped


def test_setpoint_caps_speed_keeping_direction() -> None:
    vel, acc, capped = setpoint((MAX_SPEED_MPS, 0.0, 0.0), (3.0, 0.0, 0.0), DT_S)
    assert capped
    assert np.linalg.norm(vel) == pytest.approx(MAX_SPEED_MPS)
    assert vel == pytest.approx((0.0, MAX_SPEED_MPS, 0.0))
    assert acc == pytest.approx((0.0, 3.0, 0.0))  # feedforward is a_safe unchanged


# ---------------------------------------------------------------- observation, records


def _drone(agent_id, pos, goal, vel):
    return {"agent_id": agent_id, "initial_position_m": pos, "initial_velocity_mps": vel, "goal_m": goal}


def test_observation_matches_drone_env() -> None:
    env = DroneEnv()
    obs, _ = env.reset(ScenarioConfig(scenario_id="t", seed=0, episode_limit_steps=10, drones=[
        _drone("a", (100.0, 100.0, 100.0), (300.0, 100.0, 100.0), (1.0, 0.0, 0.0)),
        _drone("b", (110.0, 104.0, 102.0), (100.0, 100.0, 100.0), (-1.0, 0.5, 0.0)),
        _drone("c", (110.0, 96.0, 98.0), (120.0, 300.0, 110.0), (0.0, 2.0, -0.5)),
    ]))
    states = env.states()
    for s in states:
        assert observation(states, s.agent_id, False, env.h_min_m2()).root == pytest.approx(obs[s.agent_id].root)


class _FakeFleet:
    """Ideal double integrator per vehicle: each setpoint moves it DT_S at its velocity, then sets it."""

    def __init__(self, enc: Encounter, stale_after_sends: int | None = None) -> None:
        self.pos = [np.array([0.0, 0.0, -(z - ALT_OFFSET_M)]) for z in enc.start_z]
        self.vel = [np.zeros(3) for _ in enc.start_z]
        self.vehicles = [type("V", (), {"flight_mode": "OFFBOARD"})() for _ in enc.start_z]
        self.sends, self.stale_after_sends = 0, stale_after_sends

    def ned(self, i):
        stale = self.stale_after_sends is not None and self.sends >= self.stale_after_sends
        return tuple(self.pos[i]), tuple(self.vel[i]), math.inf if stale else 10.0

    def send(self, i, vel_ned, acc_ned):
        self.pos[i] = self.pos[i] + self.vel[i] * DT_S
        self.vel[i] = np.array(vel_ned)
        self.sends += 1


HEADON = Encounter("headon-test", ("a", "b"), ((-20.0, 0.0), (20.0, 0.0)), (40.0, 40.0),
                   ((520.0, 500.0, 40.0), (480.0, 500.0, 40.0)), max_steps=400)


@pytest.fixture
def no_sleep(monkeypatch):
    monkeypatch.setattr(px4_bridge.time, "sleep", lambda s: None)


def test_run_encounter_records_match_runner_format(no_sleep) -> None:
    warm_up_filter(FilterConfig())  # a cold numba compile would trip the stale-at-send abort
    records, reason = run_encounter(_FakeFleet(HEADON), HEADON, filter_on=True)
    assert reason == "completed" and records[-1]["end_reason"] == "completed"
    env = DroneEnv()
    env.reset(ScenarioConfig(scenario_id="t", seed=0, episode_limit_steps=5, drones=[
        _drone("a", (100.0, 100.0, 100.0), (300.0, 100.0, 100.0), (0.0, 0.0, 0.0))]))
    env_info_keys = set(env.step([SafeAction(agent_id="a", safe_accel_mps2=(0.0, 0.0, 0.0), timestamp_ms=0,
                                             shield_active=False, h_min_m2=None, qp_feasible=True,
                                             fallback_used=False)]).infos["a"])
    for k, rec in enumerate(records):
        assert RUNNER_RECORD_KEYS <= set(rec) and rec["timestamp_ms"] == k * 100
        assert not any(key.startswith("_") for key in rec)
        for s in rec["states"]:
            DroneState.model_validate(s)
        for a in rec["proposed_actions"]:
            MARLAction.model_validate(a)
        for a in rec["returned_actions"]:
            SafeAction.model_validate(a)
        for info in rec["infos"].values():
            assert env_info_keys <= set(info)
            # as DroneEnv: the barrier before the step, i.e. on this record's own states
            assert info["h_min_prev_m2"] == pytest.approx(h_min_m2([DroneState.model_validate(s) for s in rec["states"]]))
            assert {"telemetry_age_ms", "loop_latency_ms", "flight_mode", "v_cmd_ned_mps"} <= set(info)
    assert all(i["status"] == "completed" for i in records[-1]["infos"].values())
    summary = summarize(records, 5.5)
    assert summary["n_completed"] == 2 and summary["shield_interventions"] > 0
    assert summary["separation_violation_pair_steps"] == 0  # ideal plant: the filter keeps d_safe


def test_run_encounter_stale_telemetry_aborts_last_record(no_sleep) -> None:
    records, reason = run_encounter(_FakeFleet(HEADON, stale_after_sends=10), HEADON, filter_on=False)
    assert reason.startswith("stale_telemetry: a")
    assert len(records) == 5  # two sends per step; the step that sees stale telemetry sends nothing
    last = records[-1]["infos"]
    assert all(i["status"] == "aborted" and i["abort_reason"] == reason for i in last.values())
    assert all(i["status"] == "active" for i in records[-2]["infos"].values())


def test_run_encounter_stall_inside_step_aborts_before_send(no_sleep, monkeypatch) -> None:
    fleet = _FakeFleet(HEADON)
    calls = {"n": 0}

    def slow_reference(*args, **kwargs):
        calls["n"] += 1
        if calls["n"] == 7:  # step 3, agent a: the policy stalls 0.6 s between telemetry read and send
            clock["t"] += 0.6
        return real_run_policy(*args, **kwargs)

    clock = {"t": 0.0}
    real_perf = px4_bridge.time.perf_counter
    real_run_policy = px4_bridge.run_policy
    monkeypatch.setattr(px4_bridge.time, "perf_counter", lambda: real_perf() + clock["t"])
    monkeypatch.setattr(px4_bridge, "run_policy", slow_reference)
    records, reason = run_encounter(fleet, HEADON, filter_on=False)
    assert reason.startswith("stale_at_send: step 3")
    assert fleet.sends == 6  # steps 0-2 sent, step 3 sent nothing
    assert len(records) == 3 and records[-1]["end_reason"] == reason
    assert all(i["status"] == "aborted" and i["abort_reason"] == reason for i in records[-1]["infos"].values())
