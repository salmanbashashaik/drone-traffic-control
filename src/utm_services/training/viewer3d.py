"""Self-contained interactive 3D replay viewer (one HTML file, opens in any browser, works offline).

Built on a vendored three.js r128 (MIT, ``viewer/LICENSE-three.txt``). Orbit/pan/zoom with the
mouse, preset and follow cameras, play/pause/scrub/speed, trails, labels, see-through buildings.

What is drawn is the recorded run, nothing more: positions, speeds and the filter's per-step
``shield_active`` flag come from ``run_episode`` records. Drone models are enlarged (size slider);
rotor spin is decoration; the drones are drawn level because the point-mass simulator has no
attitude. Buildings are scenery (see ``city.py``); the controller does not see them.
"""

from __future__ import annotations

import json
from html import escape as html_escape
from pathlib import Path
from typing import Any

import numpy as np
from shared.contracts import SAFE_SEPARATION_M, SIMULATION_STEP_MS

from utm_services.training.city import Building

_DIR = Path(__file__).with_name("viewer")
COLORS = ["#1a73e8", "#f57c00", "#2e9e5b", "#d81b60", "#7e57c2", "#00acc1", "#c0a000", "#6d6d6d"]


def build_payload(records: list[dict[str, Any]], ids: list[str], goals: dict[str, tuple[float, ...]],
                  buildings: list[Building], summary: dict[str, Any], title: str, note: str) -> dict[str, Any]:
    n_f, n_d = len(records), len(ids)
    pos = np.full((n_f, n_d, 3), np.nan)
    vel = np.full((n_f, n_d), np.nan)
    shield = np.zeros((n_f, n_d), dtype=bool)
    for t, rec in enumerate(records):
        for s in rec["states"]:
            k = ids.index(s["agent_id"])
            pos[t, k] = s["position_m"]
            vel[t, k] = float(np.linalg.norm(s["velocity_mps"]))
        for aid, info in rec["infos"].items():
            shield[t, ids.index(aid)] = bool(info.get("shield_active", False))
    gone = ~np.isfinite(pos[..., 0])                    # absent from env.states(): finished or aborted
    status = np.full((n_f, n_d), "active", dtype=object)
    for t, rec in enumerate(records):
        for aid, info in rec["infos"].items():
            status[t, ids.index(aid)] = info.get("status", "active")
    # info status describes the state AFTER the step, positions are BEFORE it, so "completed" at frame t
    # means the drone arrives during step t (its last drawn position is one step earlier).
    done_at = np.zeros((n_f, n_d), dtype=bool)
    aborted = np.zeros((n_f, n_d), dtype=bool)
    for k in range(n_d):
        hit = np.flatnonzero(status[:, k] == "completed")
        if len(hit):
            done_at[hit[0]:, k] = True
        bad = np.flatnonzero(status[:, k] == "aborted")
        if len(bad):
            aborted[bad[0]:, k] = True
    arrived = gone | done_at | aborted
    for k in range(n_d):                       # hold the last known position once a drone has finished
        last = None
        for t in range(n_f):
            if np.isfinite(pos[t, k, 0]):
                last = pos[t, k].copy()
            elif last is not None:
                pos[t, k] = last
                vel[t, k] = 0.0
    min_sep, near = [], np.full((n_f, n_d), np.nan)
    for t in range(n_f):
        live = [k for k in range(n_d) if not gone[t, k]]      # drones that flew step t, as the simulator counts them
        best = np.nan
        for i in live:
            for j in live:
                if i < j:
                    d = float(np.linalg.norm(pos[t, i] - pos[t, j]))
                    best = d if np.isnan(best) else min(best, d)
                    near[t, i] = d if np.isnan(near[t, i]) else min(near[t, i], d)
                    near[t, j] = d if np.isnan(near[t, j]) else min(near[t, j], d)
        min_sep.append(best)
    min_clear = [float(min(b.distance_to(pos[t][~gone[t]]).min() for b in buildings))
                 if buildings and (~gone[t]).any() else None for t in range(n_f)]

    allxy = [pos[..., :2].reshape(-1, 2)[np.isfinite(pos[..., :2].reshape(-1, 2)).all(axis=1)],
             np.array([g[:2] for g in goals.values()])]
    for b in buildings:
        allxy.append(np.array([[b.cx - b.sx / 2, b.cy - b.sy / 2], [b.cx + b.sx / 2, b.cy + b.sy / 2]]))
    fxy = np.concatenate(allxy[:2])
    flo, fhi = fxy.min(axis=0), fxy.max(axis=0)
    xy = np.concatenate(allxy)
    ztop = max([b.h for b in buildings] + [float(np.nanmax(pos[..., 2]))] + [g[2] for g in goals.values()]) + 20.0
    lo, hi = xy.min(axis=0) - 60.0, xy.max(axis=0) + 60.0

    def clean(x: Any) -> Any:
        return None if x is None or (isinstance(x, float) and np.isnan(x)) else round(float(x), 2)

    return {
        "title": title, "dt_s": SIMULATION_STEP_MS / 1000.0, "dsafe": SAFE_SEPARATION_M,
        "times_ms": [int(r["timestamp_ms"]) for r in records], "ids": ids, "colors": COLORS,
        "pos": [[[round(float(v), 2) for v in pos[t, k]] for t in range(n_f)] for k in range(n_d)],
        "speed": [[clean(vel[t, k]) for t in range(n_f)] for k in range(n_d)],
        "shield": [[bool(shield[t, k]) for t in range(n_f)] for k in range(n_d)],
        "arrived": [[bool(arrived[t, k] and not aborted[t, k]) for t in range(n_f)] for k in range(n_d)],
        "aborted": [[bool(aborted[t, k]) for t in range(n_f)] for k in range(n_d)],
        "near": [[clean(near[t, k]) for t in range(n_f)] for k in range(n_d)],
        "min_sep": [clean(v) for v in min_sep], "min_clear": [clean(v) for v in min_clear],
        "goals": [list(goals[i]) for i in ids],
        "buildings": [{"cx": b.cx, "cy": b.cy, "sx": b.sx, "sy": b.sy, "h": b.h} for b in buildings],
        "bounds": [float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1]), float(ztop)],
        "flight": [float(v) for v in (*flo, *fhi)],
        "summary_html": (f"{summary['n_completed']}/{summary['n_launched']} drones arrived &middot; "
                         f"min separation {summary['min_separation_m']:.2f} m &middot; "
                         f"violation pair-steps {summary['separation_violation_pair_steps']} &middot; "
                         f"filter interventions {summary['shield_interventions']}"),
        "note": note,
    }


def write_viewer(path: Path, payload: dict[str, Any]) -> None:
    html = (_DIR / "template.html").read_text()
    data = json.dumps(payload, separators=(",", ":")).replace("</", "<\\/")
    html = (html.replace("__TITLE__", html_escape(payload["title"]))
            .replace("__THREE__", (_DIR / "three.min.js").read_text())
            .replace("__ORBIT__", (_DIR / "OrbitControls.js").read_text())
            .replace("__DATA__", data))
    Path(path).write_text(html)
