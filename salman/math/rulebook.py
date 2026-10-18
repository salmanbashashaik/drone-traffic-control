"""
salman/math/rulebook.py
Ticket: S-02 — Lexicographic Rulebook

Priority order: Safety > Geofence > Mission Progress
Each rule returns a cost and a feasibility flag.
If Safety is violated, Geofence and Progress are irrelevant.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))
from shared.schemas import DroneState, MARLAction


@dataclass
class RulebookResult:
    safety_ok:     bool    # False → immediate override, CBF shield must act
    geofence_ok:   bool    # False → action pushes into exclusion zone
    progress_cost: float   # [0, 1] — 0 is perfect progress, 1 is moving away from goal
    priority_level: int    # 0=safety violation, 1=geofence violation, 2=mission only


@dataclass
class GeofenceCylinder:
    """Exclusion zone: cylinder in x-y plane, all altitudes."""
    center_xy: np.ndarray   # shape (2,)
    radius_m:  float
    min_alt_m: float
    max_alt_m: float

    def contains(self, position: np.ndarray) -> bool:
        """Returns True if position is inside (violating) the geofence."""
        xy_dist = np.linalg.norm(position[:2] - self.center_xy)
        in_xy = xy_dist < self.radius_m
        in_z = self.min_alt_m <= position[2] <= self.max_alt_m
        return in_xy and in_z


class Rulebook:
    """
    Evaluates the three-level priority rule hierarchy.

    Usage:
        rb = Rulebook(d_safe=5.0, geofences=[...], alt_limits=(30, 200))
        result = rb.evaluate(states, proposed_actions)
    """

    def __init__(
        self,
        d_safe: float = 5.0,
        geofences: list[GeofenceCylinder] | None = None,
        alt_limits: tuple[float, float] = (30.0, 200.0),
    ):
        self.d_safe = d_safe
        self.geofences = geofences or []
        self.alt_min, self.alt_max = alt_limits

    def _check_safety(self, states: list[DroneState]) -> bool:
        """Safety rule: no two drones closer than d_safe."""
        for i in range(len(states)):
            for j in range(i + 1, len(states)):
                dist = np.linalg.norm(states[i].position - states[j].position)
                if dist < self.d_safe:
                    return False
        return True

    def _check_geofence(self, state: DroneState, action: MARLAction, dt: float = 0.1) -> bool:
        """Geofence rule: proposed action must not move drone into any exclusion zone."""
        # Simple forward simulation: predict position after one step
        next_vel = state.velocity + action.proposed_accel * dt
        next_pos = state.position + next_vel * dt

        # Check altitude limits
        if not (self.alt_min <= next_pos[2] <= self.alt_max):
            return False

        # Check geofence exclusion cylinders
        for gf in self.geofences:
            if gf.contains(next_pos):
                return False

        return True

    def _compute_progress_cost(self, state: DroneState, action: MARLAction, dt: float = 0.1) -> float:
        """
        Progress cost: normalized cosine between proposed motion and goal direction.
        Returns 0.0 if moving directly toward goal, 1.0 if moving directly away.
        """
        goal_dir = state.goal - state.position
        goal_dist = np.linalg.norm(goal_dir)
        if goal_dist < 1e-3:
            return 0.0  # Already at goal

        goal_dir_norm = goal_dir / goal_dist
        # Proposed velocity direction
        next_vel = state.velocity + action.proposed_accel * dt
        vel_norm = np.linalg.norm(next_vel)
        if vel_norm < 1e-3:
            return 0.5  # Hovering — neutral cost

        motion_dir = next_vel / vel_norm
        cosine = np.dot(motion_dir, goal_dir_norm)
        # Map [-1, 1] → [1, 0]: moving away = 1.0, moving toward = 0.0
        return float((1.0 - cosine) / 2.0)

    def evaluate(
        self,
        states: list[DroneState],
        proposed_actions: list[MARLAction],
    ) -> list[RulebookResult]:
        """
        Evaluate all rules for each drone.

        Returns one RulebookResult per drone.
        If safety is violated at the fleet level, every drone gets safety_ok=False.
        """
        fleet_safe = self._check_safety(states)
        results = []

        state_map = {s.agent_id: s for s in states}

        for action in proposed_actions:
            state = state_map[action.agent_id]
            geo_ok = self._check_geofence(state, action)
            progress = self._compute_progress_cost(state, action)

            if not fleet_safe:
                level = 0
            elif not geo_ok:
                level = 1
            else:
                level = 2

            results.append(RulebookResult(
                safety_ok=fleet_safe,
                geofence_ok=geo_ok,
                progress_cost=progress,
                priority_level=level,
            ))

        return results


# ---------------------------------------------------------------------------
# Self-test — python -m salman.math.rulebook
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    gf = GeofenceCylinder(
        center_xy=np.array([100.0, 100.0]),
        radius_m=20.0,
        min_alt_m=0.0,
        max_alt_m=200.0,
    )
    rb = Rulebook(d_safe=5.0, geofences=[gf])

    # Scenario 1: Two drones too close — safety violation
    s1 = DroneState("d1", np.array([0.,0.,50.]), np.array([1.,0.,0.]), np.array([100.,0.,50.]), 0)
    s2 = DroneState("d2", np.array([3.,0.,50.]), np.array([-1.,0.,0.]), np.array([-100.,0.,50.]), 0)
    a1 = MARLAction("d1", np.array([1.,0.,0.]), 0)
    a2 = MARLAction("d2", np.array([-1.,0.,0.]), 0)

    results = rb.evaluate([s1, s2], [a1, a2])
    print("Scenario 1 — Too close (d=3m, d_safe=5m):")
    for r in results:
        print(f"  {r}")

    # Scenario 2: Flying into geofence
    s3 = DroneState("d3", np.array([85.,100.,50.]), np.array([5.,0.,0.]), np.array([200.,100.,50.]), 0)
    a3 = MARLAction("d3", np.array([2.,0.,0.]), 0)  # accelerating into geofence at x=100

    results2 = rb.evaluate([s3], [a3])
    print("\nScenario 2 — Flying into geofence:")
    for r in results2:
        print(f"  {r}")

    # Scenario 3: Normal flight
    s4 = DroneState("d4", np.array([0.,0.,50.]), np.array([5.,0.,0.]), np.array([200.,0.,50.]), 0)
    a4 = MARLAction("d4", np.array([1.,0.,0.]), 0)

    results3 = rb.evaluate([s4], [a4])
    print("\nScenario 3 — Normal flight toward goal:")
    for r in results3:
        print(f"  {r}")
