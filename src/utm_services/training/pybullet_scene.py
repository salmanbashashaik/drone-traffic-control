"""PyBullet scene for replays: ground, solid buildings, scaled quadrotor models, trails, snapshots.

``SceneEnv`` is ``PyBulletDroneEnv`` (same Crazyflie rigid-body dynamics, same contract) with a
nicer viewer; it does not change the dynamics, the contract, or the shared backend file.

* Drones are drawn with the Crazyflie mesh scaled ``DRONE_SCALE`` times (a real one is 9 cm
  across, invisible in a 1 km world) and follow the simulated body's position and tilt.
* Buildings are solid boxes in the physics world. The env's own collision check only aborts on
  drone-drone contact, so a drone touching a building is *counted* (``building_contacts``), not
  aborted. The controller has no knowledge of buildings (see ``city.py``).
* ``getContactPoints`` in the GUI sometimes returns ``None``; it is retried and raises if it keeps
  failing, never skipped.
"""

from __future__ import annotations

import time
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import numpy as np
import pybullet as p

from utm_services.simulation.env import ABORTED, ACTIVE, DT_S, DroneEnv, StepResult, _Agent
from utm_services.simulation.pybullet_env import PyBulletDroneEnv
from utm_services.training.city import Building

DRONE_SPAN_M = 22.0                     # drawn tip-to-tip; a real Crazyflie is 9 cm. Exaggerated to be visible.
DRONE_COLORS = [(0.10, 0.45, 0.95), (1.00, 0.55, 0.10), (0.15, 0.70, 0.35), (0.85, 0.20, 0.60),
                (0.55, 0.30, 0.85), (0.10, 0.70, 0.75), (0.90, 0.80, 0.10), (0.45, 0.45, 0.45)]
_RED, _AMBER = (0.95, 0.15, 0.15), (1.0, 0.75, 0.1)
GROUND = (0.80, 0.84, 0.78, 1.0)


def _make_drone(c: int, color: tuple[float, float, float], span_m: float = DRONE_SPAN_M) -> int:
    """Quadrotor from primitives (body, 4 arms in an X, 4 rotor discs): renders in any PyBullet renderer."""
    arm = span_m / 2.0
    body = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.22 * arm, 0.22 * arm, 0.10 * arm], rgbaColor=[*color, 1.0],
                               physicsClientId=c)
    arm_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[arm / 2.0, 0.06 * arm, 0.04 * arm],
                                  rgbaColor=[0.12, 0.12, 0.14, 1.0], physicsClientId=c)
    rotor = p.createVisualShape(p.GEOM_CYLINDER, radius=0.34 * arm, length=0.05 * arm,
                                rgbaColor=[color[0] * 0.5, color[1] * 0.5, color[2] * 0.5, 0.95], physicsClientId=c)
    shapes, pos, orn = [], [], []
    for i in range(4):
        yaw = np.pi / 4 + i * np.pi / 2
        d = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        shapes.append(arm_vis); pos.append((d * arm / 2).tolist()); orn.append(p.getQuaternionFromEuler([0, 0, yaw]))
        shapes.append(rotor); pos.append((d * arm + [0, 0, 0.09 * arm]).tolist()); orn.append([0, 0, 0, 1])
    n = len(shapes)
    return p.createMultiBody(
        baseMass=0, baseVisualShapeIndex=body, basePosition=[0, 0, 0], linkMasses=[0] * n,
        linkCollisionShapeIndices=[-1] * n, linkVisualShapeIndices=shapes, linkPositions=pos, linkOrientations=orn,
        linkInertialFramePositions=[[0, 0, 0]] * n, linkInertialFrameOrientations=[[0, 0, 0, 1]] * n,
        linkParentIndices=[0] * n, linkJointTypes=[p.JOINT_FIXED] * n, linkJointAxis=[[0, 0, 1]] * n, physicsClientId=c)


class SceneEnv(PyBulletDroneEnv):
    def __init__(self, rules=None, gui: bool = False, playback_speed: float = 1.0,
                 buildings: Sequence[Building] = (), snapshot_every: int = 0, snapshot_size: tuple[int, int] = (1280, 800),
                 camera: dict[str, float] | None = None) -> None:
        super().__init__(rules, gui=gui, playback_speed=playback_speed)
        self._buildings = list(buildings)
        self._snap_every = snapshot_every
        self._snap_size = snapshot_size
        self._camera = camera or {}
        self._scene = gui or snapshot_every > 0
        self.snapshots: list[np.ndarray] = []
        self.building_contacts: set[tuple[str, int]] = set()   # (agent_id, building index)

    # ------------------------------------------------------------ lifecycle

    def reset(self, config):
        out = super().reset(config)
        if self._scene and not self._gui:
            self._init_view([self._agents[a] for a in self._ids])
        return out

    def step(self, actions) -> StepResult:
        result = DroneEnv.step(self, actions)          # dynamics + metrics; drawing is done here
        self._count_building_contacts()
        if self._scene:
            self._draw(result)
        if self._gui:
            time.sleep(DT_S / self._playback_speed)
        if self._snap_every and self._step % self._snap_every == 0:
            self.snapshots.append(self._snapshot())
        return result

    def _query(self, fn, *args, **kw):
        for _ in range(50):
            out = fn(*args, physicsClientId=self._client, **kw)
            if out is not None:
                return out
            time.sleep(0.02)
        raise RuntimeError("PyBullet did not answer a query after 50 tries; the check cannot be skipped")

    def _check_collisions(self, agents: Sequence[_Agent]) -> None:
        for agent in agents:
            if agent.status != ACTIVE:
                continue
            body = self._bodies[self._ids.index(agent.agent_id)]
            for c in self._query(p.getContactPoints, bodyA=body):
                if c[2] in self._bodies:
                    other = self._agents[self._ids[self._bodies.index(c[2])]]
                    for a in (agent, other):
                        if a.status == ACTIVE:
                            a.status, a.abort_reason = ABORTED, "collision"
                    self._metrics["collisions"] += 1

    def _count_building_contacts(self) -> None:
        if not self._buildings or not hasattr(self, "_building_bodies"):
            return
        for aid, body in zip(self._ids, self._bodies, strict=True):
            for c in self._query(p.getContactPoints, bodyA=body):
                if c[2] in self._building_bodies:
                    self.building_contacts.add((aid, self._building_bodies.index(c[2])))

    # --------------------------------------------------------------- viewer

    def _init_view(self, agents: Sequence[_Agent]) -> None:
        c = self._client
        for flag in (p.COV_ENABLE_GUI, p.COV_ENABLE_RGB_BUFFER_PREVIEW, p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
                     p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW):
            p.configureDebugVisualizer(flag, 0, physicsClientId=c)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, physicsClientId=c)
        p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0, physicsClientId=c)

        # Ground: tint the built-in plane and add a large visual slab below it.
        p.changeVisualShape(0, -1, textureUniqueId=-1, rgbaColor=list(GROUND), physicsClientId=c)
        slab = p.createVisualShape(p.GEOM_BOX, halfExtents=[900, 900, 0.5], rgbaColor=list(GROUND), physicsClientId=c)
        p.createMultiBody(baseVisualShapeIndex=slab, basePosition=[500, 500, -0.55], physicsClientId=c)
        line = [0.55, 0.60, 0.55, 1.0]
        for v in np.arange(0, 1001, 50):
            w = 0.5 if int(v) % 100 == 0 else 0.25
            for half, pos in (([w, 500, 0.05], [v, 500, 0.06]), ([500, w, 0.05], [500, v, 0.06])):
                g = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=line, physicsClientId=c)
                p.createMultiBody(baseVisualShapeIndex=g, basePosition=pos, physicsClientId=c)

        self._building_bodies = []
        for b in self._buildings:
            tone = 0.80 - 0.45 * min(1.0, b.h / 170.0)
            col = [tone, tone + 0.03, tone + 0.10, 1.0]
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[b.sx / 2, b.sy / 2, b.h / 2], rgbaColor=col, physicsClientId=c)
            col_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[b.sx / 2, b.sy / 2, b.h / 2], physicsClientId=c)
            self._building_bodies.append(p.createMultiBody(
                baseMass=0, baseCollisionShapeIndex=col_shape, baseVisualShapeIndex=vis,
                basePosition=[b.cx, b.cy, b.h / 2], physicsClientId=c))
            roof = p.createVisualShape(p.GEOM_BOX, halfExtents=[b.sx / 2 + 0.4, b.sy / 2 + 0.4, 0.5],
                                       rgbaColor=[0.92, 0.92, 0.94, 1.0], physicsClientId=c)
            p.createMultiBody(baseVisualShapeIndex=roof, basePosition=[b.cx, b.cy, b.h + 0.5], physicsClientId=c)

        self._models, self._bubbles, self._labels, self._last = [], [], [], {}
        for k, a in enumerate(agents):
            color = DRONE_COLORS[k % len(DRONE_COLORS)]
            self._models.append(_make_drone(c, color))
            p.resetBasePositionAndOrientation(self._models[-1], a.position.tolist(), [0, 0, 0, 1], physicsClientId=c)
            halo = p.createVisualShape(p.GEOM_SPHERE, radius=DRONE_SPAN_M * 0.75, rgbaColor=[*color, 0.0], physicsClientId=c)
            self._bubbles.append(p.createMultiBody(baseVisualShapeIndex=halo, basePosition=a.position.tolist(),
                                                   physicsClientId=c))
            # Goal: a faint thin post to the ground and a solid sphere at goal altitude.
            gz = float(a.goal[2])
            post = p.createVisualShape(p.GEOM_CYLINDER, radius=0.35, length=gz, rgbaColor=[*color, 0.22], physicsClientId=c)
            p.createMultiBody(baseVisualShapeIndex=post, basePosition=[a.goal[0], a.goal[1], gz / 2], physicsClientId=c)
            tip = p.createVisualShape(p.GEOM_SPHERE, radius=3.2, rgbaColor=[*color, 0.95], physicsClientId=c)
            p.createMultiBody(baseVisualShapeIndex=tip, basePosition=a.goal.tolist(), physicsClientId=c)
            self._labels.append(-1)
            self._last[a.agent_id] = a.position.copy()

        pts = np.array([a.position for a in agents] + [a.goal for a in agents])
        centre = (pts.min(axis=0) + pts.max(axis=0)) / 2
        span = float(np.ptp(pts[:, :2], axis=0).max())
        self._cam = {"distance": 0.95 * span + 170.0, "yaw": 38.0, "pitch": -36.0,
                     "target": [float(centre[0]), float(centre[1]), 55.0], **self._camera}
        p.resetDebugVisualizerCamera(self._cam["distance"], self._cam["yaw"], self._cam["pitch"], self._cam["target"],
                                     physicsClientId=c)

    def _draw(self, result: StepResult) -> None:
        c = self._client
        for k, aid in enumerate(self._ids):
            a = self._agents[aid]
            info = result.infos.get(aid)
            pos, orn = p.getBasePositionAndOrientation(self._bodies[k], physicsClientId=c)
            p.resetBasePositionAndOrientation(self._models[k], pos, orn, physicsClientId=c)
            color = DRONE_COLORS[k % len(DRONE_COLORS)]
            bad = info is not None and (info["fallback_used"] or info["separation_violation"])
            amber = info is not None and info["shield_active"]
            bub = _RED if bad else _AMBER if amber else color
            p.resetBasePositionAndOrientation(self._bubbles[k], pos, [0, 0, 0, 1], physicsClientId=c)
            p.changeVisualShape(self._bubbles[k], -1, rgbaColor=[*bub, 0.30 if (bad or amber) else 0.0], physicsClientId=c)
            if a.status == ACTIVE and self._step % 2 == 0:
                bead = p.createVisualShape(p.GEOM_SPHERE, radius=1.1, rgbaColor=[*color, 1.0], physicsClientId=c)
                p.createMultiBody(baseVisualShapeIndex=bead, basePosition=list(pos), physicsClientId=c)
            if self._gui:
                self._labels[k] = p.addUserDebugText(aid, [pos[0], pos[1], pos[2] + 16.0], [0.05, 0.05, 0.05], 1.8,
                                                     replaceItemUniqueId=self._labels[k], physicsClientId=c)
            self._last[aid] = np.array(pos)
            if a.status != ACTIVE and self._gui:
                p.addUserDebugText("arrived" if a.status != ABORTED else (a.abort_reason or "aborted"),
                                   [pos[0], pos[1], pos[2] + 10.0], [0.1, 0.1, 0.1], 1.2, lifeTime=0.4, physicsClientId=c)

    def _snapshot(self) -> np.ndarray:
        w, h = self._snap_size
        cam = self._cam
        view = p.computeViewMatrixFromYawPitchRoll(cam["target"], cam["distance"], cam["yaw"], cam["pitch"], 0, 2,
                                                   physicsClientId=self._client)
        proj = p.computeProjectionMatrixFOV(60, w / h, 1.0, 4000.0, physicsClientId=self._client)
        renderer = p.ER_BULLET_HARDWARE_OPENGL if self._gui else p.ER_TINY_RENDERER
        img = p.getCameraImage(w, h, view, proj, renderer=renderer, flags=p.ER_NO_SEGMENTATION_MASK,
                               shadow=1, physicsClientId=self._client)
        return np.reshape(img[2], (h, w, 4))[..., :3].astype(np.uint8)


def save_snapshots(frames: list[np.ndarray], out_dir: Path, gif_name: str = "scene.gif", fps: int = 12) -> list[Path]:
    """Write PNGs for the first/middle/last frames and an animated GIF of all of them."""
    from PIL import Image

    out_dir.mkdir(parents=True, exist_ok=True)
    imgs = [Image.fromarray(f) for f in frames]
    paths = []
    for tag, idx in (("start", 0), ("mid", len(imgs) // 2), ("end", len(imgs) - 1)):
        path = out_dir / f"scene_{tag}.png"
        imgs[idx].save(path)
        paths.append(path)
    imgs[0].save(out_dir / gif_name, save_all=True, append_images=imgs[1:], duration=int(1000 / fps), loop=0)
    return paths


def scene_summary(env: SceneEnv) -> dict[str, Any]:
    return {"building_contacts": sorted(env.building_contacts), "n_snapshots": len(env.snapshots)}
