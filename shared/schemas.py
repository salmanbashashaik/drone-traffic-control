"""
shared/schemas.py
Shared data types used across all three workstreams.
Do NOT modify without opening a PR and getting sign-off from all owners.
See INTERFACES.md for the full contract.
"""
from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np


@dataclass
class DroneState:
    """State vector for a single drone. See INTERFACES.md § STATE-SCHEMA."""
    agent_id:     str
    position:     np.ndarray   # shape (3,) — x, y, z in meters
    velocity:     np.ndarray   # shape (3,) — vx, vy, vz in m/s
    goal:         np.ndarray   # shape (3,) — current waypoint in meters
    timestamp_ms: int          # simulation time in ms

    def to_dict(self) -> dict:
        return {
            "agent_id":     self.agent_id,
            "position":     self.position.tolist(),
            "velocity":     self.velocity.tolist(),
            "goal":         self.goal.tolist(),
            "timestamp_ms": self.timestamp_ms,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "DroneState":
        return cls(
            agent_id=d["agent_id"],
            position=np.array(d["position"], dtype=float),
            velocity=np.array(d["velocity"], dtype=float),
            goal=np.array(d["goal"], dtype=float),
            timestamp_ms=int(d["timestamp_ms"]),
        )


@dataclass
class MARLAction:
    """Proposed (unfiltered) action from MARL policy. See INTERFACES.md § MARL-ACTION."""
    agent_id:       str
    proposed_accel: np.ndarray   # shape (3,) — m/s²
    timestamp_ms:   int


@dataclass
class SafeAction:
    """CBF-filtered safe action. See INTERFACES.md § CBF-OUTPUT."""
    agent_id:      str
    safe_accel:    np.ndarray   # shape (3,) — m/s²
    timestamp_ms:  int
    shield_active: bool         # True if CBF modified the proposed action
    h_min:         float        # minimum barrier value across all pairwise constraints
    qp_feasible:   bool         # False → fallback controller triggered
    fallback_used: bool         # True if HJ hover fallback was activated
