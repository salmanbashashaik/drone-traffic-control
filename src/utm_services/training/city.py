"""Seeded building layouts for replay scenes (viewer layer; not part of the simulator contract).

v1 has no static obstacles (INTERFACES.md): neither the policy nor the backup-set barrier filter has any
notion of a building, and ``DroneEnv`` ignores them. These buildings are scenery that makes a
replay readable and gives the PyBullet world solid objects. They are *not* avoided by the
controller. What keeps a drone away from them is the layout itself:

* every building is placed at least ``margin_m`` away from each drone's straight
  start-to-goal segment and from its start and goal points, so the nominal route is clear;
* ``clearance_report`` then measures how close the flown tracks actually came, because a
  filter manoeuvre can deflect a drone off its straight route.

The margin is a layout choice, not a bound on the flown track: the safety filter can swing a
drone tens of metres off its straight route (up to about 55 m in the shipped head-on), so a
track can pass close to, or through, a building even though the straight route was clear. A
clearance figure here is a measurement of one simulated run, not a guarantee, and says nothing
about how the system would behave if a building were in the way. Obstacle-aware control would be a separate change to the filter.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

import numpy as np
from shared.contracts import ScenarioConfig

CITY_VERSION = "city-v1"


@dataclass(frozen=True)
class Building:
    cx: float
    cy: float
    sx: float   # footprint size along x (m)
    sy: float   # footprint size along y (m)
    h: float    # roof height above ground (m); the box spans z in [0, h]

    def distance_to(self, points: np.ndarray) -> np.ndarray:
        """Euclidean distance from points (..., 3) to the solid box; 0 inside or on it."""
        p = np.asarray(points, dtype=np.float64)
        dx = np.maximum(np.abs(p[..., 0] - self.cx) - self.sx / 2.0, 0.0)
        dy = np.maximum(np.abs(p[..., 1] - self.cy) - self.sy / 2.0, 0.0)
        dz = np.maximum(np.maximum(0.0 - p[..., 2], p[..., 2] - self.h), 0.0)
        return np.sqrt(dx * dx + dy * dy + dz * dz)

    def corners(self) -> np.ndarray:
        x0, x1 = self.cx - self.sx / 2, self.cx + self.sx / 2
        y0, y1 = self.cy - self.sy / 2, self.cy + self.sy / 2
        return np.array([[x0, y0, 0], [x1, y0, 0], [x1, y1, 0], [x0, y1, 0],
                         [x0, y0, self.h], [x1, y0, self.h], [x1, y1, self.h], [x0, y1, self.h]])


def _segment_samples(a: np.ndarray, b: np.ndarray, step_m: float = 2.0) -> np.ndarray:
    n = max(2, int(np.ceil(np.linalg.norm(b - a) / step_m)) + 1)
    return a + (b - a) * np.linspace(0.0, 1.0, n)[:, None]


def generate_city(config: ScenarioConfig, n_buildings: int = 26, margin_m: float = 60.0,
                  seed: int | None = None) -> list[Building]:
    """Seeded layout around the scenario; deterministic in (config, n_buildings, margin_m, seed).

    Heights are 20-60 m (about a third) or 80-170 m, so many roofs reach the 30-200 m flight band.
    """
    rng = np.random.default_rng(config.seed if seed is None else seed)
    segs = [_segment_samples(np.array(d.initial_position_m), np.array(d.goal_m)) for d in config.drones]
    pts = np.concatenate(segs)
    lo = np.maximum(pts.min(axis=0)[:2] - 190.0, 20.0)
    hi = np.minimum(pts.max(axis=0)[:2] + 190.0, 980.0)
    buildings: list[Building] = []
    for _ in range(40_000):
        if len(buildings) == n_buildings:
            break
        sx, sy = rng.uniform(14.0, 46.0, 2)
        h = float(rng.uniform(20.0, 60.0) if rng.random() < 0.35 else rng.uniform(80.0, 170.0))
        cand = Building(float(rng.uniform(lo[0], hi[0])), float(rng.uniform(lo[1], hi[1])), float(sx), float(sy), h)
        if cand.distance_to(pts).min() < margin_m:
            continue
        if any(abs(cand.cx - b.cx) < (cand.sx + b.sx) / 2 + 12.0 and abs(cand.cy - b.cy) < (cand.sy + b.sy) / 2 + 12.0
               for b in buildings):
            continue
        buildings.append(cand)
    return buildings


def clearance_report(records: list[dict[str, Any]], buildings: list[Building]) -> dict[str, Any]:
    """Measured closeness of the flown tracks to the buildings, from run records (ground-truth states)."""
    if not buildings or not records:
        return {"n_buildings": len(buildings), "min_clearance_m": None, "samples_inside": 0}
    best, inside = np.inf, 0
    for rec in records:
        p = np.array([s["position_m"] for s in rec["states"]])
        if not len(p):
            continue
        d = np.min([b.distance_to(p) for b in buildings], axis=0)
        best = min(best, float(d.min()))
        inside += int((d <= 0.0).sum())
    return {"n_buildings": len(buildings), "min_clearance_m": float(best), "samples_inside": inside}


def city_to_json(buildings: list[Building]) -> list[dict[str, float]]:
    return [asdict(b) for b in buildings]
