"""PyBullet quadrotor backend for DroneEnv (physics spike).

Same contract, rules, metrics and observations as the point-mass DroneEnv; only
the dynamics change. Each drone is a rigid-body Crazyflie 2.x quadrotor from
gym-pybullet-drones (motor thrust/torque, drag, ground effect, downwash) flown by
the DSL PID position controller at 240 Hz. Every 100 ms step the returned
``SafeAction`` acceleration is integrated into a position/velocity reference that
the PID tracks, so the drone responds with real actuator and attitude lag.

Differences from the point-mass backend:
- The speed cap applies to the reference. A drone whose true speed goes above
  MAX_SPEED_MPS is aborted ("speed_envelope_exceeded"), because DroneState
  cannot represent it.
- Bodies collide. A drone-drone contact aborts both drones ("collision") and
  they fall with motors off. Completed drones are parked on the ground away from
  the airspace.

Setup (Linux/WSL; pybullet has no Windows wheels):
    pip install -e ".[physics]"
    pip install --no-deps "gym-pybullet-drones @ git+https://github.com/utiasDSL/gym-pybullet-drones.git@7ebad1e"

Demo with the 3D window (WSLg on Windows 11):
    python -m utm_services.simulation.pybullet_env head-on-100 shielded_marl
"""

from __future__ import annotations

import time
from collections.abc import Sequence
from typing import Any

import numpy as np
import pybullet as p
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from shared.contracts import MAX_SPEED_MPS, SAFE_SEPARATION_M, AirspaceRulesV1, ScenarioConfig

from utm_services.simulation.env import ABORTED, ACTIVE, DT_S, DroneEnv, StepResult, _Agent

ENV_VERSION = "sim-pybullet-v0"
PYB_FREQ_HZ = 240
SUBSTEPS = round(DT_S * PYB_FREQ_HZ)
# Reference speed limit: below MAX_SPEED_MPS so PID overshoot stays inside the contract bound.
REF_SPEED_MPS = 0.95 * MAX_SPEED_MPS

_COLORS = [(0.12, 0.44, 0.70), (0.48, 0.31, 0.77), (0.06, 0.56, 0.54), (0.69, 0.27, 0.48)]
_AMBER, _RED = (0.94, 0.65, 0.19), (0.95, 0.39, 0.36)


class PyBulletDroneEnv(DroneEnv):
    """DroneEnv with PyBullet quadrotor physics instead of point-mass dynamics."""

    def __init__(
        self,
        rules: AirspaceRulesV1 | None = None,
        gui: bool = False,
        playback_speed: float = 1.0,
        physics: Physics = Physics.PYB_GND_DRAG_DW,
    ) -> None:
        super().__init__(rules)
        self._gui = gui
        self._playback_speed = playback_speed
        self._physics = physics
        self._aviary: CtrlAviary | None = None

    def reset(self, config: ScenarioConfig) -> tuple[dict[str, Any], dict[str, Any]]:
        observations, metadata = super().reset(config)
        self.close()
        self._ids = sorted(self._agents)
        agents = [self._agents[aid] for aid in self._ids]
        self._aviary = CtrlAviary(
            drone_model=DroneModel.CF2X,
            num_drones=len(agents),
            initial_xyzs=np.array([a.position for a in agents]),
            physics=self._physics,
            pyb_freq=PYB_FREQ_HZ,
            ctrl_freq=PYB_FREQ_HZ,
            gui=self._gui,
            user_debug_gui=False,
        )
        self._client = self._aviary.CLIENT
        self._bodies = list(self._aviary.DRONE_IDS)
        for body, a in zip(self._bodies, agents, strict=True):
            p.resetBaseVelocity(body, a.velocity.tolist(), [0, 0, 0], physicsClientId=self._client)
        self._aviary._updateAndStoreKinematicInformation()
        self._pids = [DSLPIDControl(drone_model=DroneModel.CF2X) for _ in agents]
        self._ref = {a.agent_id: (a.position.copy(), a.velocity.copy()) for a in agents}
        self._parked: set[str] = set()
        self._metrics["collisions"] = 0
        if self._gui:
            self._init_view(agents)
        metadata.update(
            env_version=ENV_VERSION,
            dynamics="pybullet cf2x + DSL PID, ref-tracking",
            physics=self._physics.value,
            pyb_freq_hz=PYB_FREQ_HZ,
        )
        return observations, metadata

    def step(self, actions) -> StepResult:
        result = super().step(actions)
        if self._gui:
            self._draw(result)
            time.sleep(DT_S / self._playback_speed)
        return result

    def close(self) -> None:
        if self._aviary is not None:
            self._aviary.close()
            self._aviary = None

    # ------------------------------------------------------------- dynamics

    def _advance(self, agents: Sequence[_Agent], accels: dict[str, np.ndarray]) -> dict[str, bool]:
        dt = 1.0 / PYB_FREQ_HZ
        speed_capped = {a.agent_id: False for a in agents}
        stepping = {a.agent_id for a in agents}
        # Re-anchor the reference on the true state each step: a free-running reference
        # drifts ahead of the lagging drone and the PID then overshoots MAX_SPEED_MPS.
        for a in agents:
            self._ref[a.agent_id] = (a.position.copy(), a.velocity.copy())
        for _ in range(SUBSTEPS):
            rpms = np.zeros((len(self._ids), 4))
            for i, aid in enumerate(self._ids):
                if aid not in stepping or self._agents[aid].status != ACTIVE:
                    continue  # motors off: finished drones are parked, crashed ones fall
                p_ref, v_ref = self._ref[aid]
                v_new = v_ref + accels[aid] * dt
                speed = float(np.linalg.norm(v_new))
                if speed > REF_SPEED_MPS:
                    v_new *= REF_SPEED_MPS / speed
                    speed_capped[aid] = True
                p_ref = p_ref + 0.5 * (v_ref + v_new) * dt
                self._ref[aid] = (p_ref, v_new)
                s = self._aviary._getDroneStateVector(i)
                pid = self._pids[i]
                # Acceleration feed-forward: the DSL PID force is P*pos_e + D*vel_e + m*g, so a
                # target offset of m*a/P adds the commanded m*a (GRAVITY = m*9.8 in BaseControl).
                ff = (pid.GRAVITY / 9.8) * accels[aid] / pid.P_COEFF_FOR
                rpms[i], _, _ = pid.computeControl(
                    control_timestep=dt, cur_pos=s[0:3], cur_quat=s[3:7], cur_vel=s[10:13],
                    cur_ang_vel=s[13:16], target_pos=p_ref + ff, target_vel=v_new,
                )
            self._aviary.step(rpms)
            self._check_collisions(agents)

        for agent in agents:
            i = self._ids.index(agent.agent_id)
            agent.position = np.array(self._aviary.pos[i], dtype=np.float64)
            agent.velocity = np.array(self._aviary.vel[i], dtype=np.float64)
            if agent.status == ACTIVE and np.linalg.norm(agent.velocity) > MAX_SPEED_MPS:
                agent.status, agent.abort_reason = ABORTED, "speed_envelope_exceeded"
        self._park_finished()
        return speed_capped

    def _check_collisions(self, agents: Sequence[_Agent]) -> None:
        for agent in agents:
            if agent.status != ACTIVE:
                continue
            body = self._bodies[self._ids.index(agent.agent_id)]
            for c in p.getContactPoints(bodyA=body, physicsClientId=self._client):
                if c[2] in self._bodies:
                    other = self._agents[self._ids[self._bodies.index(c[2])]]
                    for a in (agent, other):
                        if a.status == ACTIVE:
                            a.status, a.abort_reason = ABORTED, "collision"
                    self._metrics["collisions"] += 1

    def _park_finished(self) -> None:
        """Move completed drones out of the airspace so they are not obstacles."""
        for i, aid in enumerate(self._ids):
            a = self._agents[aid]
            if a.status not in (ACTIVE, ABORTED) and aid not in self._parked:
                p.resetBasePositionAndOrientation(
                    self._bodies[i], [-20.0 - 2.0 * i, -20.0, 0.05], [0, 0, 0, 1], physicsClientId=self._client
                )
                p.resetBaseVelocity(self._bodies[i], [0, 0, 0], [0, 0, 0], physicsClientId=self._client)
                self._parked.add(aid)

    # --------------------------------------------------------------- viewer

    def _init_view(self, agents: Sequence[_Agent]) -> None:
        c = self._client
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=c)
        pts = np.array([a.position for a in agents] + [a.goal for a in agents])
        center = (pts.min(axis=0) + pts.max(axis=0)) / 2
        span = float(np.ptp(pts[:, :2], axis=0).max())
        p.resetDebugVisualizerCamera(
            cameraDistance=max(60.0, 1.1 * span), cameraYaw=30, cameraPitch=-35,
            cameraTargetPosition=center.tolist(), physicsClientId=c,
        )
        # Translucent bubble of radius d_safe/2: two touching bubbles mean a separation violation.
        self._bubbles = []
        for k, a in enumerate(agents):
            vis = p.createVisualShape(
                p.GEOM_SPHERE, radius=SAFE_SEPARATION_M / 2, rgbaColor=[*_COLORS[k % 4], 0.45], physicsClientId=c
            )
            self._bubbles.append(p.createMultiBody(baseVisualShapeIndex=vis, basePosition=a.position.tolist(), physicsClientId=c))
            goal = p.createVisualShape(p.GEOM_SPHERE, radius=1.0, rgbaColor=[*_COLORS[k % 4], 0.9], physicsClientId=c)
            p.createMultiBody(baseVisualShapeIndex=goal, basePosition=a.goal.tolist(), physicsClientId=c)
        for g in self._rules.geofences if self._rules else []:
            vis = p.createVisualShape(
                p.GEOM_CYLINDER, radius=g.radius_m, length=g.ceiling_m - g.floor_m,
                rgbaColor=[*_RED, 0.15], physicsClientId=c,
            )
            p.createMultiBody(
                baseVisualShapeIndex=vis,
                basePosition=[*g.center_xy_m, (g.floor_m + g.ceiling_m) / 2], physicsClientId=c,
            )
        self._labels = [-1] * len(agents)
        self._last = {a.agent_id: a.position.copy() for a in agents}

    def _draw(self, result: StepResult) -> None:
        c = self._client
        for k, aid in enumerate(self._ids):
            a = self._agents[aid]
            info = result.infos.get(aid)
            if info is None:
                continue
            bad = info["fallback_used"] or info["separation_violation"] or a.abort_reason == "collision"
            color = _RED if bad else _AMBER if info["shield_active"] else _COLORS[k % 4]
            p.resetBasePositionAndOrientation(self._bubbles[k], a.position.tolist(), [0, 0, 0, 1], physicsClientId=c)
            p.changeVisualShape(self._bubbles[k], -1, rgbaColor=[*color, 0.45], physicsClientId=c)
            p.addUserDebugLine(self._last[aid].tolist(), a.position.tolist(), _COLORS[k % 4], 2.0, physicsClientId=c)
            self._last[aid] = a.position.copy()
            text = f"{aid} {np.linalg.norm(a.velocity):.1f} m/s" + (f" [{a.abort_reason or a.status}]" if a.status != ACTIVE else "")
            self._labels[k] = p.addUserDebugText(
                text, (a.position + [0, 0, 4]).tolist(), [0.1, 0.1, 0.1], 1.2,
                replaceItemUniqueId=self._labels[k], physicsClientId=c,
            )


# ------------------------------------------------------------------ demo


def _demo_scenarios() -> dict[str, list[dict[str, Any]]]:
    def drone(agent_id, pos, goal, vel=(0.0, 0.0, 0.0)):
        return {"agent_id": agent_id, "initial_position_m": pos, "initial_velocity_mps": vel, "goal_m": goal}

    corners = [(400.0, 400.0), (600.0, 400.0), (600.0, 600.0), (400.0, 600.0)]
    return {
        "head-on-100": [
            drone("a", (450.0, 420.0, 100.0), (850.0, 420.0, 100.0), vel=(15.0, 0.0, 0.0)),
            drone("b", (550.0, 420.0, 100.0), (150.0, 420.0, 100.0), vel=(-15.0, 0.0, 0.0)),
        ],
        "corner-swap": [drone(f"d{k}", (*corners[k], 100.0), (*corners[(k + 2) % 4], 100.0)) for k in range(4)],
    }


if __name__ == "__main__":
    import argparse

    from utm_services.api.app import _default_rules
    from utm_services.policy.reference import compute_reference_action
    from utm_services.simulation.runner import VARIANTS, run_episode

    scenarios = _demo_scenarios()
    ap = argparse.ArgumentParser(description="Fly a demo scenario in the PyBullet 3D window.")
    ap.add_argument("scenario", choices=sorted(scenarios))
    ap.add_argument("variant", choices=VARIANTS)
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed multiplier")
    ap.add_argument("--headless", action="store_true")
    args = ap.parse_args()

    cfg = ScenarioConfig(scenario_id=args.scenario, seed=0, episode_limit_steps=900, drones=scenarios[args.scenario])
    env_holder: list[PyBulletDroneEnv] = []

    def make_env(rules):
        env_holder.append(PyBulletDroneEnv(rules, gui=not args.headless, playback_speed=args.speed))
        return env_holder[-1]

    res = run_episode(
        cfg, args.variant, policy=compute_reference_action, policy_id="reference-standin",
        rules=_default_rules(), keep_records=False, make_env=make_env,
    )
    print({k: v for k, v in res.summary.items() if "time" not in k and "latency" not in k})
    if not args.headless:
        print("Episode done. Close the 3D window to exit.")
        while p.isConnected(env_holder[-1]._client):
            time.sleep(0.2)
    env_holder[-1].close()
