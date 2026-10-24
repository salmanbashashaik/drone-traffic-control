"""Learned nominal policy: numpy runtime for a trained MARL actor (P2-SA-01).

The actor is a small tanh MLP trained with MAPPO (``utm_services.training``).
At runtime only this module is needed: it has no torch dependency, so the
artifact loads on any machine that can run the simulator, and the forward pass
is plain numpy (an ONNX export for P4 is a straight translation of these ops).

This policy is a nominal action *proposer*. Its output is never labelled safe;
the HOCBF filter sits after it in the shielded variant, and the unshielded
variant is exactly this policy with no filter.

Artifact directory (written by ``save_policy_artifact``)::

    actor.npz       weights, float64 (w0, b0, w1, b1, w2, b2)
    metadata.json   everything needed to reproduce and to refuse a mismatch

``LearnedPolicy.load`` refuses to load when metadata is missing, when the weight
hash differs from the one recorded, when the observation/action contract names
differ, or when the world bounds the policy was trained for differ from the
current ``shared/contracts.py``. Observation normalisation is applied inside the
policy with parameters fixed by ``OBS_NORM_VERSION``; it is the only place the
raw observation indices are rescaled.

Action head: the network outputs a mean in units of ``MAX_ACCEL_COMPONENT_MPS2``.
The policy saturates that mean to [-1, 1] before scaling. That saturation is part
of the actor's parameterisation (the training distribution is a Gaussian over the
same unit), not the adapter silently clipping an invalid action: the adapter
(``policy/adapter.py``) still rejects anything outside the contract.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from shared.contracts import (
    MAX_ACCEL_COMPONENT_MPS2,
    MAX_SPEED_MPS,
    WORLD_XY_MAX_M,
    WORLD_XY_MIN_M,
    WORLD_Z_MAX_M,
    WORLD_Z_MIN_M,
    MARLAction,
    ObservationV1,
)

OBS_DIM = 18
ACT_DIM = 3
OBS_NORM_VERSION = "obs-norm-v3"
# Nearest-neighbour terms (indices 9-15) beyond this range are replaced by the "no neighbour"
# values, so the actor never sees separations it was not trained on. The safety filter is not
# gated by this: it sees every drone.
NEIGHBOUR_GATE_M = 250.0
ARTIFACT_FORMAT = "utm-marl-actor-v1"
WEIGHT_KEYS = ("w0", "b0", "w1", "b1", "w2", "b2")

# Unit the network works in; one network unit is this many m/s^2.
ACTION_UNIT_MPS2 = MAX_ACCEL_COMPONENT_MPS2
# Keep the executed value strictly inside the contract bound (float rounding).
ACTION_SCALE_MPS2 = ACTION_UNIT_MPS2 * (1.0 - 1e-9)

OBS_NORMALIZATION: dict[str, str] = {
    "version": OBS_NORM_VERSION,
    "neighbour_gate": "if index 15 (distance) > 250 m: indices 9-14 := 0 and index 15 := -1 (no-neighbour values) before scaling",
    "indices_0_1": "x, y: v/500 - 1",
    "index_2": "z: (v - 115)/85",
    "indices_3_5": "v/15",
    "indices_6_8": "arcsinh(v/20)/arcsinh(50)  (compressive: keeps signal within a few metres of the goal)",
    "indices_9_11": "v/500",
    "indices_12_14": "v/15",
    "index_15": "v/500 (sentinel -1 -> -0.002)",
    "index_16": "unchanged",
    "index_17": "unchanged",
}

# Per-index affine map: normalised = (raw - shift) * scale.
_SHIFT = np.zeros(OBS_DIM)
_SCALE = np.ones(OBS_DIM)
_SHIFT[0:2], _SCALE[0:2] = 500.0, 1.0 / 500.0
_SHIFT[2], _SCALE[2] = 115.0, 1.0 / 85.0
_SCALE[3:6] = 1.0 / 15.0
_GOAL_SCALE_M = 20.0
_GOAL_NORM = float(np.arcsinh(1000.0 / _GOAL_SCALE_M))  # 1000 m -> 1.0
_SCALE[9:12] = 1.0 / 500.0
_SCALE[12:15] = 1.0 / 15.0
_SCALE[15] = 1.0 / 500.0


def normalize_obs(raw: np.ndarray) -> np.ndarray:
    """Normalise raw ObservationV1 values; accepts shape (..., 18)."""
    raw = np.array(raw, dtype=np.float64)  # copy: the gate below edits it
    if raw.shape[-1] != OBS_DIM:
        raise ValueError(f"expected last dimension {OBS_DIM}, got shape {raw.shape}")
    far = (raw[..., 15] > NEIGHBOUR_GATE_M)[..., None]
    raw[..., 9:15] = np.where(far, 0.0, raw[..., 9:15])
    raw[..., 15:16] = np.where(far, -1.0, raw[..., 15:16])
    out = (raw - _SHIFT) * _SCALE
    out[..., 6:9] = np.arcsinh(raw[..., 6:9] / _GOAL_SCALE_M) / _GOAL_NORM
    return out


def world_bounds() -> dict[str, Any]:
    """The contract constants a policy is trained against; stored in every artifact."""
    return {
        "xy_m": [WORLD_XY_MIN_M, WORLD_XY_MAX_M],
        "z_m": [WORLD_Z_MIN_M, WORLD_Z_MAX_M],
        "max_speed_mps": MAX_SPEED_MPS,
        "max_accel_mps2": MAX_ACCEL_COMPONENT_MPS2,
    }


def actor_mean(weights: dict[str, np.ndarray], x: np.ndarray) -> np.ndarray:
    """Actor forward pass on normalised observations (..., 18) -> mean in network units."""
    h = np.tanh(x @ weights["w0"] + weights["b0"])
    h = np.tanh(h @ weights["w1"] + weights["b1"])
    return h @ weights["w2"] + weights["b2"]


def weights_sha256(weights: dict[str, np.ndarray]) -> str:
    """Hash of the weights themselves (not of the npz container), key order fixed."""
    digest = hashlib.sha256()
    for key in WEIGHT_KEYS:
        arr = np.ascontiguousarray(weights[key], dtype="<f8")
        digest.update(key.encode())
        digest.update(str(arr.shape).encode())
        digest.update(arr.tobytes())
    return digest.hexdigest()


def _check_weight_shapes(weights: dict[str, np.ndarray]) -> int:
    missing = [k for k in WEIGHT_KEYS if k not in weights]
    if missing:
        raise ValueError(f"actor weights missing keys {missing}")
    hidden = weights["w0"].shape[1]
    expected = {
        "w0": (OBS_DIM, hidden), "b0": (hidden,), "w1": (hidden, hidden), "b1": (hidden,),
        "w2": (hidden, ACT_DIM), "b2": (ACT_DIM,),
    }
    for key, shape in expected.items():
        if weights[key].shape != shape:
            raise ValueError(f"actor weight '{key}' has shape {weights[key].shape}, expected {shape}")
        if not np.all(np.isfinite(weights[key])):
            raise ValueError(f"actor weight '{key}' contains NaN or Inf")
    return hidden


def save_policy_artifact(directory: Path, weights: dict[str, np.ndarray], metadata: dict[str, Any]) -> Path:
    """Write actor.npz and metadata.json; fills in the fields this module owns."""
    hidden = _check_weight_shapes(weights)
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    clean = {k: np.asarray(weights[k], dtype=np.float64) for k in WEIGHT_KEYS}
    np.savez(directory / "actor.npz", **clean)
    meta = {
        **metadata,
        "artifact_format": ARTIFACT_FORMAT,
        "obs_contract": "ObservationV1",
        "action_contract": "MARLAction",
        "obs_normalization": OBS_NORMALIZATION,
        "action_unit_mps2": ACTION_UNIT_MPS2,
        "world_bounds_assumed": world_bounds(),
        "actor_architecture": {"layers": [OBS_DIM, hidden, hidden, ACT_DIM], "activation": "tanh"},
        "weights_file": "actor.npz",
        "weights_sha256": weights_sha256(clean),
        "role": "nominal action proposer; not a safety controller; output is not labelled safe",
    }
    (directory / "metadata.json").write_text(json.dumps(meta, indent=2, sort_keys=True))
    return directory


class ArtifactError(ValueError):
    """The artifact is missing, altered, or built for a different contract."""


@dataclass(frozen=True)
class LearnedPolicy:
    """Deterministic runtime policy: PolicyFn-compatible callable plus its metadata."""

    weights: dict[str, np.ndarray]
    metadata: dict[str, Any]

    @property
    def artifact_id(self) -> str:
        return str(self.metadata["artifact_id"])

    @classmethod
    def load(cls, directory: Path | str) -> LearnedPolicy:
        directory = Path(directory)
        meta_path = directory / "metadata.json"
        if not meta_path.is_file():
            raise ArtifactError(f"{directory}: metadata.json missing; a policy cannot be loaded without it")
        metadata = json.loads(meta_path.read_text())
        problems = []
        if metadata.get("artifact_format") != ARTIFACT_FORMAT:
            problems.append(f"artifact_format {metadata.get('artifact_format')!r} != {ARTIFACT_FORMAT!r}")
        if metadata.get("obs_contract") != "ObservationV1" or metadata.get("action_contract") != "MARLAction":
            problems.append("observation/action contract names differ from ObservationV1/MARLAction")
        if metadata.get("obs_normalization") != OBS_NORMALIZATION:
            problems.append(f"observation normalisation differs from {OBS_NORM_VERSION}")
        if metadata.get("world_bounds_assumed") != world_bounds():
            problems.append(
                f"world bounds the policy was trained for {metadata.get('world_bounds_assumed')} "
                f"differ from shared/contracts.py {world_bounds()}"
            )
        if metadata.get("action_unit_mps2") != ACTION_UNIT_MPS2:
            problems.append("action unit differs from MAX_ACCEL_COMPONENT_MPS2")
        if problems:
            raise ArtifactError(f"{directory}: " + "; ".join(problems))
        weights_path = directory / str(metadata.get("weights_file", "actor.npz"))
        if not weights_path.is_file():
            raise ArtifactError(f"{weights_path}: weights file missing")
        with np.load(weights_path) as npz:
            weights = {k: np.array(npz[k], dtype=np.float64) for k in WEIGHT_KEYS if k in npz}
        try:
            _check_weight_shapes(weights)
        except ValueError as exc:
            raise ArtifactError(f"{weights_path}: {exc}") from exc
        if weights_sha256(weights) != metadata.get("weights_sha256"):
            raise ArtifactError(f"{weights_path}: weight hash does not match metadata (file altered or mixed up)")
        return cls(weights=weights, metadata=metadata)

    def action_vector(self, raw_obs: np.ndarray) -> np.ndarray:
        """Deterministic acceleration (m/s^2) for raw observation(s) (..., 18)."""
        mean = actor_mean(self.weights, normalize_obs(raw_obs))
        return np.clip(mean, -1.0, 1.0) * ACTION_SCALE_MPS2

    def __call__(self, obs: ObservationV1, agent_id: str, timestamp_ms: int) -> MARLAction:
        accel = self.action_vector(np.asarray(obs.root, dtype=np.float64))
        if not np.all(np.isfinite(accel)):
            raise ArtifactError(f"policy produced a non-finite action for agent '{agent_id}'")
        ax, ay, az = (float(v) for v in accel)
        if not all(math.isfinite(v) for v in (ax, ay, az)):  # pragma: no cover - belt and braces
            raise ArtifactError("non-finite action")
        return MARLAction(agent_id=agent_id, proposed_accel_mps2=(ax, ay, az), timestamp_ms=timestamp_ms)
