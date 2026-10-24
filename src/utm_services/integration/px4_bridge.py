"""Bridge between PX4 SITL quadrotors and the backup-set barrier safety filter (integration/px4-bridge).

Each 100 ms step reads the latest PX4 telemetry of every vehicle, converts it to
``DroneState`` in our frame, runs the reference policy and ``apply_filter`` (one
persistent latch per run), and sends every vehicle an offboard setpoint. Every
step is logged in the ``records.jsonl`` format ``utm-run`` writes.

Frames (docs/integration/PX4_SETUP.md, "Frames"):

- Ours: x east, y north, z up, metres; world box from ``shared.contracts``.
- PX4 local NED: each vehicle's origin is its own spawn point (PX4 SITL sets the
  GPS origin where the vehicle boots). The spawn points are given in the Gazebo
  world frame (ENU), so one shared origin is
  ``ours = (WORLD_ORIGIN_XY + spawn_en + (east, north), -down + ALT_OFFSET_M)``.
  ``check_shared_origin`` verifies the spawn offsets against the GPS origins, and
  that every vehicle reads the same height while all stand on the ground.
- Altitude: SITL height above ground h maps to z = h + ALT_OFFSET_M, so the
  world floor z = 30 m is 10 m above the SITL ground.

Setpoint mapping (``setpoint``): a velocity setpoint v_cmd = v_meas + a_safe dt,
speed-clipped to MAX_SPEED_MPS, plus a_safe as acceleration feedforward. The
velocity term alone realises only 10-15 % of a_safe at PX4's default velocity
gain; with the feedforward 77-98 % (scripts/px4/accel_tracking.py, PX4_SETUP.md).

``states`` and ``observed_states`` in the records are the same PX4 EKF
estimate: SITL gives the bridge no separate ground truth.

The pure functions here run without MAVSDK; ``Px4Fleet`` imports it lazily.
"""

from __future__ import annotations

import math
import time
from collections.abc import Sequence
from dataclasses import dataclass
from itertools import combinations
from typing import Any

import numpy as np
from pydantic import ValidationError
from shared.contracts import (
    MAX_SPEED_MPS,
    SAFE_SEPARATION_M,
    SIMULATION_STEP_MS,
    DroneState,
    MARLAction,
    ObservationV1,
    SafeAction,
)

from utm_services.policy.adapter import run_policy
from utm_services.policy.reference import ReferencePolicy
from utm_services.safety.filter import FilterConfig, apply_filter
from utm_services.simulation.env import GOAL_RADIUS_M

WORLD_ORIGIN_XY = (500.0, 500.0)    # our (x, y) of the Gazebo world origin
ALT_OFFSET_M = 20.0                 # our z = SITL height above ground + ALT_OFFSET_M
DT_S = SIMULATION_STEP_MS / 1000.0
TELEMETRY_STALE_MS = 500.0          # state this old when read or before any setpoint is sent, or a step
                                    # starting this late, stops the run and lands every vehicle
ORIGIN_TOLERANCE_M = 0.25           # per vehicle vs vehicle 0; a pair can be off by 0.5 m, the whole filter
                                    # margin, so at the limit no margin is left
PORT_BASE = 14540                   # PX4 instance i sends its offboard link to PORT_BASE + i


# --- frames ---------------------------------------------------------------------------------------

def ned_to_ours(spawn_en: Sequence[float], pos_ned: Sequence[float], vel_ned: Sequence[float]
                ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """PX4 local NED (origin at the vehicle's spawn point) to our position and velocity."""
    n, e, d = pos_ned
    vn, ve, vd = vel_ned
    pos = (WORLD_ORIGIN_XY[0] + spawn_en[0] + e, WORLD_ORIGIN_XY[1] + spawn_en[1] + n, -d + ALT_OFFSET_M)
    return (float(pos[0]), float(pos[1]), float(pos[2])), (float(ve), float(vn), float(-vd))


def ours_to_ned_vector(v: Sequence[float]) -> tuple[float, float, float]:
    """A velocity or acceleration in our frame to NED (no offset)."""
    return float(v[1]), float(v[0]), float(-v[2])


def sitl_height(z: float) -> float:
    """SITL height above ground for our altitude z."""
    return z - ALT_OFFSET_M


def gps_offset_en(origin: tuple[float, float], ref: tuple[float, float]) -> tuple[float, float]:
    """East/north metres of a GPS origin from a reference origin (equirectangular, fine at 100 m)."""
    north = math.radians(origin[0] - ref[0]) * 6378137.0
    east = math.radians(origin[1] - ref[1]) * 6378137.0 * math.cos(math.radians(ref[0]))
    return east, north


def check_shared_origin(spawns_en: Sequence[Sequence[float]], gps_origins: Sequence[tuple[float, float]],
                        ground_down_m: Sequence[float], tol_m: float = ORIGIN_TOLERANCE_M) -> list[float]:
    """3D error (m) of each vehicle's local frame against vehicle 0's.

    Horizontal: spawn offset vs GPS-origin offset from vehicle 0. Vertical: local ``down`` read
    while every vehicle stands on the (flat) ground, vs vehicle 0's. Raises ValueError if any
    error exceeds ``tol_m``: then the local frames do not share the origin that ``ned_to_ours`` assumes.
    """
    errors = []
    for spawn, origin, down in zip(spawns_en, gps_origins, ground_down_m, strict=True):
        e, n = gps_offset_en(origin, gps_origins[0])
        errors.append(math.hypot(e - (spawn[0] - spawns_en[0][0]), n - (spawn[1] - spawns_en[0][1]),
                                 down - ground_down_m[0]))
    if max(errors) > tol_m:
        raise ValueError(f"GPS origins do not match the spawn offsets: errors {errors} m > {tol_m} m")
    return errors


# --- guidance -------------------------------------------------------------------------------------

def setpoint(v_meas: Sequence[float], a_safe: Sequence[float], dt_s: float = DT_S
             ) -> tuple[tuple[float, float, float], tuple[float, float, float], bool]:
    """Offboard setpoint for one step, in NED: (velocity, acceleration feedforward, speed_capped).

    v_cmd = v_meas + a_safe dt, scaled down to MAX_SPEED_MPS if faster. The feedforward is
    a_safe unchanged; PX4 adds it to its velocity-loop output.
    """
    v = np.asarray(v_meas, dtype=float) + np.asarray(a_safe, dtype=float) * dt_s
    speed = float(np.linalg.norm(v))
    capped = speed > MAX_SPEED_MPS
    if capped:
        v *= MAX_SPEED_MPS / speed
    return ours_to_ned_vector(v), ours_to_ned_vector(a_safe), capped


# --- observation, metrics, records ----------------------------------------------------------------

def h_min_m2(states: Sequence[DroneState]) -> float | None:
    """Minimum pairwise barrier ||p_i - p_j||^2 - d_safe^2 (as DroneEnv.h_min_m2)."""
    d = [float(np.linalg.norm(np.subtract(a.position_m, b.position_m))) for a, b in combinations(states, 2)]
    return min(d) ** 2 - SAFE_SEPARATION_M**2 if d else None


def observation(states: Sequence[DroneState], agent_id: str, prev_shield_active: bool,
                h_min: float | None) -> ObservationV1:
    """ObservationV1 with the layout of DroneEnv._observation (kept in step with it)."""
    by_id = {s.agent_id: s for s in states}
    me = by_id[agent_id]
    pos, vel = np.array(me.position_m), np.array(me.velocity_mps)
    others = [aid for aid in by_id if aid != agent_id]
    if others:
        # Nearest by 3D distance, ties broken by lexicographic agent_id.
        nbr = min(others, key=lambda o: (float(np.linalg.norm(np.array(by_id[o].position_m) - pos)), o))
        rel_p = np.array(by_id[nbr].position_m) - pos
        rel_v = np.array(by_id[nbr].velocity_mps) - vel
        dist = float(np.linalg.norm(rel_p))
    else:
        rel_p = rel_v = np.zeros(3)
        dist = -1.0
    barrier = 1.0 if h_min is None else max(-1.0, min(1.0, h_min / SAFE_SEPARATION_M**2))
    values = [*pos, *vel, *(np.array(me.goal_m) - pos), *rel_p, *rel_v, dist,
              1.0 if prev_shield_active else 0.0, barrier]
    return ObservationV1.model_validate([float(v) for v in values])


def step_infos(next_states: Sequence[DroneState], returned: Sequence[SafeAction], status: dict[str, str],
               h_min_prev: float | None, extra: dict[str, dict[str, Any]], abort_reason: str | None = None
               ) -> dict[str, dict[str, Any]]:
    """Per-agent infos in DroneEnv's format, on the states read at the start of the next step.

    ``extra`` holds per-agent ``speed_capped`` and bridge fields (telemetry age, loop latency,
    flight mode, velocity command), which are added to the infos.
    """
    pos = {s.agent_id: np.array(s.position_m) for s in next_states}
    infos = {}
    for act in returned:
        aid = act.agent_id
        others = [float(np.linalg.norm(pos[aid] - p)) for o, p in pos.items() if o != aid]
        nearest = min(others) if others else None
        infos[aid] = {
            "timestamp_ms": act.timestamp_ms + SIMULATION_STEP_MS,
            "status": status[aid],
            "abort_reason": abort_reason,
            "shield_active": act.shield_active,
            "h_min_m2": act.h_min_m2,
            "qp_feasible": act.qp_feasible,
            "fallback_used": act.fallback_used,
            "nearest_separation_m": nearest,
            "separation_violation": nearest is not None and nearest < SAFE_SEPARATION_M,
            "speed_capped": extra[aid]["speed_capped"],
            "geofence_incursion": False,
            "h_min_prev_m2": h_min_prev,
            "command": "received",
            "command_dropped": False,
            **{k: v for k, v in extra[aid].items() if k != "speed_capped"},
        }
    return infos


def summarize(records: Sequence[dict[str, Any]], d_eff: float) -> dict[str, Any]:
    """Episode metrics from the records (the DroneEnv/runner fields that apply, plus SITL timing)."""
    seps, hseps, inside_steps = [], [], 0
    violation_pair_steps = 0
    for rec in records:
        pos = [s["position_m"] for s in rec["states"]]
        d = [float(np.linalg.norm(np.subtract(a, b))) for a, b in combinations(pos, 2)]
        if d:
            seps.append(min(d))
            hseps.append(min(float(np.hypot(a[0] - b[0], a[1] - b[1])) for a, b in combinations(pos, 2)))
            violation_pair_steps += sum(x < SAFE_SEPARATION_M for x in d)
            inside_steps += min(d) < d_eff
    last = records[-1]["infos"] if records else {}
    infos = [i for r in records for i in r["infos"].values()]

    def pct(key: str, q: float) -> float | None:
        vals = [i[key] for i in infos if i.get(key) is not None]
        return float(np.percentile(vals, q)) if vals else None

    return {
        "n_launched": len(last),
        "n_completed": sum(i["status"] == "completed" for i in last.values()),
        "n_steps": len(records),
        "min_separation_m": min(seps) if seps else None,
        "min_horizontal_separation_m": min(hseps) if hseps else None,
        "separation_violation_pair_steps": violation_pair_steps,
        "time_inside_d_eff_s": round(inside_steps * DT_S, 3),
        "d_eff_m": d_eff,
        "shield_interventions": sum(i["shield_active"] for i in infos),
        "fallback_events": sum(i["fallback_used"] for i in infos),
        "qp_infeasible_steps": sum(not all(i["qp_feasible"] for i in r["infos"].values()) for r in records),
        "speed_cap_events": sum(i["speed_capped"] for i in infos),
        "p50_loop_latency_ms": pct("loop_latency_ms", 50),
        "p95_loop_latency_ms": pct("loop_latency_ms", 95),
        "max_loop_latency_ms": pct("loop_latency_ms", 100),
        "p50_telemetry_age_ms": pct("telemetry_age_ms", 50),
        "p95_telemetry_age_ms": pct("telemetry_age_ms", 95),
        "max_telemetry_age_ms": pct("telemetry_age_ms", 100),
        "max_step_overrun_ms": max((r.get("step_overrun_ms", 0.0) for r in records), default=0.0),
    }


def warm_up_filter(filter_config: FilterConfig, n: int = 2) -> float:
    """Run apply_filter on n drones closing on a point so numba compiles its kernels before flight; seconds taken.

    Without this the first in-flight step takes about 14 s in WSL (PX4 meanwhile flies the
    previous setpoint, which MAVSDK keeps re-sending).
    """
    t0 = time.perf_counter()
    dirs = [(math.cos(2 * math.pi * k / n), math.sin(2 * math.pi * k / n)) for k in range(n)]
    states = [DroneState(agent_id=f"w{k}", position_m=(500.0 + 10 * c, 500.0 + 10 * s, 100.0),
                         velocity_mps=(-5 * c, -5 * s, 0.0), goal_m=(500.0 - 40 * c, 500.0 - 40 * s, 100.0),
                         timestamp_ms=0) for k, (c, s) in enumerate(dirs)]
    actions = [MARLAction(agent_id=s.agent_id, proposed_accel_mps2=(0.0, 0.0, 0.0), timestamp_ms=0) for s in states]
    apply_filter(states, actions, filter_config, latch={})
    return time.perf_counter() - t0


# --- link to PX4 ----------------------------------------------------------------------------------

@dataclass
class _Vehicle:
    system: Any
    telemetry: Any
    action: Any
    offboard: Any
    param: Any
    latest: Any = None              # PositionVelocityNed
    received_s: float = math.nan    # time.monotonic() of the latest telemetry
    flight_mode: str = "UNKNOWN"
    subs: tuple = ()


class Px4Fleet:
    """MAVSDK link to PX4 SITL instances 0..n-1 (MAV_SYS_ID i+1 on UDP PORT_BASE+i)."""

    def __init__(self, n: int, timeout_s: float = 60.0) -> None:
        from mavsdk import ComponentType, Configuration, Mavsdk
        from mavsdk.plugins.action import Action
        from mavsdk.plugins.offboard import Offboard
        from mavsdk.plugins.param import Param
        from mavsdk.plugins.telemetry import Telemetry

        self._mav = Mavsdk(Configuration.create_with_component_type(ComponentType.GROUND_STATION))
        for i in range(n):
            self._mav.add_any_connection(f"udpin://0.0.0.0:{PORT_BASE + i}")
        t0 = time.monotonic()
        while True:
            systems = {s.get_system_id(): s for s in self._mav.get_systems() if s.has_autopilot()}
            if all(i + 1 in systems for i in range(n)):
                break
            if time.monotonic() - t0 > timeout_s:
                raise TimeoutError(f"found autopilots {sorted(systems)}, need system ids 1..{n}")
            time.sleep(0.5)
        self.vehicles = []
        for i in range(n):
            s = systems[i + 1]
            v = _Vehicle(s, Telemetry(s), Action(s), Offboard(s), Param(s))
            v.telemetry.set_rate_position_velocity_ned(50.0)
            v.subs = (v.telemetry.subscribe_position_velocity_ned(self._on_pv, v),
                      v.telemetry.subscribe_flight_mode(self._on_mode, v))
            self.vehicles.append(v)

    @staticmethod
    def _on_pv(data: Any, v: _Vehicle) -> None:
        v.latest, v.received_s = data, time.monotonic()

    @staticmethod
    def _on_mode(mode: Any, v: _Vehicle) -> None:
        v.flight_mode = mode.name

    def wait(self, cond, timeout_s: float, what: str) -> None:
        t0 = time.monotonic()
        while not cond():
            if time.monotonic() - t0 > timeout_s:
                raise TimeoutError(f"timeout after {timeout_s} s waiting for {what}")
            time.sleep(0.2)

    def ned(self, i: int) -> tuple[tuple[float, float, float], tuple[float, float, float], float]:
        """Latest (position NED, velocity NED, age in ms) of vehicle i."""
        v = self.vehicles[i]
        latest, received_s = v.latest, v.received_s  # one sample: the MAVSDK callback may replace it meanwhile
        if latest is None:
            return (math.nan,) * 3, (math.nan,) * 3, math.inf
        p, w = latest.position, latest.velocity
        return ((p.north_m, p.east_m, p.down_m), (w.north_m_s, w.east_m_s, w.down_m_s),
                (time.monotonic() - received_s) * 1000.0)

    def gps_origins(self) -> list[tuple[float, float]]:
        out = []
        for v in self.vehicles:
            o = v.telemetry.get_gps_global_origin()
            out.append((o.latitude_deg, o.longitude_deg))
        return out

    def params(self, floats: Sequence[str], ints: Sequence[str]) -> dict[str, float | int]:
        """Parameters of vehicle 0 (all instances run the same airframe defaults)."""
        p = self.vehicles[0].param
        return {**{k: round(p.get_param_float(k), 4) for k in floats}, **{k: p.get_param_int(k) for k in ints}}

    def take_off(self, heights_m: Sequence[float]) -> None:
        """Arm every vehicle and climb to its height above ground; returns once all hover there."""
        from mavsdk.exceptions import MavsdkError
        for i, v in enumerate(self.vehicles):
            self.wait(v.telemetry.health_all_ok, 60, f"vehicle {i} health checks")
            v.action.set_takeoff_altitude(float(heights_m[i]))
        for i, v in enumerate(self.vehicles):
            for _ in range(20):  # PX4 denies arming until it has seen our GCS heartbeat for a moment
                try:
                    v.action.arm()
                    break
                except MavsdkError:
                    time.sleep(1.0)
            else:
                raise RuntimeError(f"vehicle {i}: arming denied for 20 s")
        for v in self.vehicles:
            v.action.takeoff()

        def hovering() -> bool:
            for i, h in enumerate(heights_m):
                p, w, _ = self.ned(i)
                if not (abs(-p[2] - h) < 0.5 and math.hypot(*w) < 0.3):
                    return False
            return True

        self.wait(hovering, 60, "all vehicles hovering at their takeoff height")

    def start_offboard(self) -> None:
        from mavsdk.plugins.offboard import VelocityNedYaw

        for v in self.vehicles:  # PX4 needs a setpoint before it accepts offboard mode
            v.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        for v in self.vehicles:
            v.offboard.start()

    def send(self, i: int, vel_ned: Sequence[float], acc_ned: Sequence[float]) -> None:
        from mavsdk.plugins.offboard import AccelerationNed, PositionNedYaw, VelocityNedYaw

        # NaN position: PX4 does not control position, only velocity with acceleration feedforward.
        self.vehicles[i].offboard.set_position_velocity_acceleration_ned(
            PositionNedYaw(math.nan, math.nan, math.nan, 0.0), VelocityNedYaw(*vel_ned, 0.0),
            AccelerationNed(*acc_ned))

    def land(self) -> None:
        """Stop offboard (PX4 holds), land every vehicle and wait until all are disarmed."""
        from mavsdk.exceptions import MavsdkError

        for i, v in enumerate(self.vehicles):
            try:
                v.offboard.stop()
            except MavsdkError as exc:  # not in offboard (e.g. a failsafe took over): land anyway
                print(f"vehicle {i}: offboard stop failed ({exc}); landing")
        for v in self.vehicles:
            v.action.land()
        self.wait(lambda: not any(v.telemetry.armed() for v in self.vehicles), 120, "all vehicles disarmed")

    def close(self) -> None:
        for v in self.vehicles:
            v.telemetry.unsubscribe_position_velocity_ned(v.subs[0])
            v.telemetry.unsubscribe_flight_mode(v.subs[1])


# --- the closed loop ------------------------------------------------------------------------------

@dataclass(frozen=True)
class Encounter:
    """A demo encounter: agent i flies PX4 instance i from its spawn to its goal (our frame)."""

    scenario_id: str
    agent_ids: tuple[str, ...]
    spawns_en: tuple[tuple[float, float], ...]   # Gazebo world ENU (m), as passed to sitl.sh
    start_z: tuple[float, ...]                   # our altitude after takeoff
    goals_m: tuple[tuple[float, float, float], ...]
    max_steps: int = 900


def run_encounter(fleet: Px4Fleet, enc: Encounter, filter_on: bool,
                  filter_config: FilterConfig | None = None, stop_after_s: float | None = None
                  ) -> tuple[list[dict[str, Any]], str]:
    """Fly the encounter at 10 Hz until every agent is within GOAL_RADIUS_M of its goal.

    Returns the records and the end reason ("completed", "max_steps", "stale_telemetry: ...",
    "stale_at_send: ...", "invalid_state: ...", "overrun: ..." or "stopped"), which is also
    stored as ``end_reason`` in the last record. On "stopped" (``stop_after_s`` reached, for the
    link-loss test) the caller must not land the vehicles: PX4's offboard-loss failsafe acts.
    Completed agents stay in the filter input and hold at their goal under the policy.
    """
    filter_config = filter_config or FilterConfig()
    variant = "shielded_marl" if filter_on else "unshielded_marl"
    episode_id = f"{enc.scenario_id}-px4sitl-{variant}"
    policy = ReferencePolicy()
    latch: dict = {}  # per-pair sideways frames, kept across the run's steps
    status = {aid: "active" for aid in enc.agent_ids}
    prev_shield = {aid: False for aid in enc.agent_ids}
    records: list[dict[str, Any]] = []
    pending: dict[str, Any] | None = None
    t_start = time.monotonic()
    next_tick = t_start

    def finish(reason: str) -> tuple[list[dict[str, Any]], str]:
        if records:
            records[-1]["end_reason"] = reason
        return records, reason

    for k in range(enc.max_steps + 1):
        now = time.monotonic()
        overrun_ms = max(0.0, (now - next_tick) * 1000.0)
        t0 = time.perf_counter()
        ts = k * SIMULATION_STEP_MS
        ages, states, reason = {}, [], None
        for i, aid in enumerate(enc.agent_ids):
            p, v, age = fleet.ned(i)
            ages[aid] = age
            if not age <= TELEMETRY_STALE_MS:
                reason = f"stale_telemetry: {aid} {age:.0f} ms"
                break
            pos, vel = ned_to_ours(enc.spawns_en[i], p, v)
            try:
                states.append(DroneState(agent_id=aid, position_m=pos, velocity_mps=vel,
                                         goal_m=enc.goals_m[i], timestamp_ms=ts))
            except ValidationError as exc:
                reason = f"invalid_state: {aid} {exc.errors()[0]['msg']}"
                break
        if pending is not None and reason is None:
            for s in states:
                if np.linalg.norm(np.subtract(s.position_m, s.goal_m)) <= GOAL_RADIUS_M:
                    status[s.agent_id] = "completed"
            pending["infos"] = step_infos(states, pending.pop("_returned"), status, pending.pop("_h_min_prev"),
                                          pending.pop("_extra"))
            del pending["_states"]
            records.append(pending)
            pending = None
        if pending is not None:  # stopped on bad telemetry: close the last step on its own states
            status = {aid: "aborted" for aid in status}
            pending["infos"] = step_infos(pending.pop("_states"), pending.pop("_returned"), status,
                                          pending.pop("_h_min_prev"), pending.pop("_extra"), abort_reason=reason)
            records.append(pending)
        if reason is None and all(st == "completed" for st in status.values()):
            reason = "completed"
        if reason is None and k == enc.max_steps:
            reason = "max_steps"
        if reason is None and overrun_ms > TELEMETRY_STALE_MS:
            reason = f"overrun: step {k} started {overrun_ms:.0f} ms late"
        if reason is None and stop_after_s is not None and now - t_start >= stop_after_s:
            reason = "stopped"
        if reason is not None:
            return finish(reason)

        h_min = h_min_m2(states)
        proposed: list[MARLAction] = [
            run_policy(observation(states, s.agent_id, prev_shield[s.agent_id], h_min).root, s.agent_id, ts, policy)
            for s in states
        ]
        if filter_on:
            returned, _ = apply_filter(states, proposed, filter_config, latch=latch)
        else:
            returned = [SafeAction(agent_id=a.agent_id, safe_accel_mps2=a.proposed_accel_mps2, timestamp_ms=ts,
                                   shield_active=False, h_min_m2=h_min, qp_feasible=True, fallback_used=False)
                        for a in proposed]
        extra = {}
        for i, (s, act) in enumerate(zip(states, returned, strict=True)):
            # A stall inside the step (solve, GC, scheduling, a blocking send) must not send a setpoint
            # computed on old state, so the age is checked again before every send.
            state_age_ms = max(ages.values()) + (time.perf_counter() - t0) * 1000.0
            if state_age_ms > TELEMETRY_STALE_MS:
                reason = (f"stale_at_send: step {k} state {state_age_ms:.0f} ms old before the setpoint of "
                          f"{s.agent_id}; this step sent to {list(extra)}")
                for info in records[-1]["infos"].values() if records else ():
                    info.update(status="aborted", abort_reason=reason)
                return finish(reason)
            vel_ned, acc_ned, capped = setpoint(s.velocity_mps, act.safe_accel_mps2)
            fleet.send(i, vel_ned, acc_ned)
            extra[s.agent_id] = {"speed_capped": capped, "v_cmd_ned_mps": [round(x, 4) for x in vel_ned],
                                 "telemetry_age_ms": round(ages[s.agent_id], 2),
                                 "flight_mode": fleet.vehicles[i].flight_mode}
        latency_ms = (time.perf_counter() - t0) * 1000.0
        for e in extra.values():
            e["loop_latency_ms"] = round(latency_ms, 3)
        prev_shield = {a.agent_id: a.shield_active for a in returned}
        pending = {
            "episode_id": episode_id,
            "scenario_id": enc.scenario_id,
            "seed": 0,
            "controller_variant": variant,
            "timestamp_ms": ts,
            "states": [s.model_dump(mode="json") for s in states],
            "observed_states": [s.model_dump(mode="json") for s in states],
            "proposed_actions": [a.model_dump(mode="json") for a in proposed],
            "returned_actions": [a.model_dump(mode="json") for a in returned],
            "wall_time_s": round(now - t_start, 4),
            "step_overrun_ms": round(overrun_ms, 3),
            "_states": states,
            "_h_min_prev": h_min,
            "_returned": returned,
            "_extra": extra,
        }
        # A late step shifts the schedule; it never sends a burst of steps to catch up.
        next_tick = max(next_tick + DT_S, time.monotonic())
        time.sleep(max(0.0, next_tick - time.monotonic()))
    raise AssertionError("unreachable: the loop returns at k == max_steps")
