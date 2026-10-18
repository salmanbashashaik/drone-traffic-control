# VISUALIZATION.md — Simulation UI Spec
> Owner: Aman (builds it) + Anas (hooks simulation output to it)
> No Unity. No C#. Pure Python, browser-viewable.

---

## Why not Unity?

Unity is the standard for drone sim visuals but requires: a C# codebase, a Unity license, a separate install, and a bridging layer to your Python simulation. That's weeks of integration work you don't have.

The alternative stack below gives you a 3D live visualization in under a day, entirely in Python, with the simulation and the UI in the same process.

---

## Chosen stack: Rerun.io

**What it is**: An open-source data visualization SDK for robotics and spatial computing. You log data from Python; it streams to a viewer (standalone app or browser). Built for exactly this use case — multi-agent trajectories, sensor data, time-series overlays.

**Why it fits**:
- One `pip install rerun-sdk` — no separate engine
- Log drone positions, velocities, shields, and metrics in the same Python loop as the simulation
- 3D viewer with timeline scrubbing — you can replay any episode
- Browser mode: `rr.serve()` → opens in Chrome, no install needed for teammates
- Handles 50+ agents in real-time

**Repo**: https://github.com/rerun-io/rerun  
**Docs**: https://www.rerun.io/docs

---

## What the UI must show (ticket UI-01 AC)

| Element | How to implement |
|---------|-----------------|
| 3D drone positions, live | `rr.log("drone/{id}/position", rr.Points3D([x,y,z]))` |
| Drone velocity arrows | `rr.log("drone/{id}/velocity", rr.Arrows3D(...))` |
| Goal position marker | `rr.log("drone/{id}/goal", rr.Points3D([gx,gy,gz], colors=[0,255,0]))` |
| **Drone color = green normally, red when shield fires** | Pass `colors` param — green `[0,255,0]` or red `[255,0,0]` based on `shield_active` flag from `SafeAction` |
| Geofence cylinder | `rr.log("airspace/geofence_001", rr.Cylinders3D(...))` once per episode |
| Shield intervention count (live) | `rr.log("metrics/shield_interventions", rr.Scalar(count))` |
| Min separation distance (live) | `rr.log("metrics/min_separation", rr.Scalar(h_min))` |
| Collision count | `rr.log("metrics/collisions", rr.Scalar(n_collisions))` |
| Chaos injector status | `rr.log("chaos/comm_delay_active", rr.Scalar(1 or 0))` |

---

## Architecture

```
Simulation loop (anas/env/shielded_env.py)
        │
        │  after each step, call:
        ▼
aman/ui/visualizer.py
  └── RerunLogger.log_step(states, safe_actions, metrics)
        │
        └── rr.log(...) calls → Rerun viewer (localhost:9876 or browser)
```

Anas adds **one line** at the end of `ShieldedEnv.step()`:
```python
self.rerun_logger.log_step(states, safe_actions, infos)
```

The `RerunLogger` class lives in `aman/ui/visualizer.py` — Aman owns it.

---

## UI features for the interactive demo

The demo needs to show:
1. Drones flying normally (green)
2. Aman clicks a button → Byzantine drone injected → one drone starts behaving erratically
3. The CBF shield fires → that drone turns red → separation maintained
4. Metrics panel shows real-time: interventions, min separation, throughput

For the interactive control panel (add drones, toggle chaos), use **Rerun's Blueprint API** to add UI panels, or wrap with a lightweight **Gradio** interface that controls the simulation and streams the Rerun embed.

**Gradio option** (simpler for demo):
```
pip install gradio
```
Gradio gives you sliders (`n_drones: 1–50`), toggles (`chaos: on/off`), and embeds the Rerun viewer in an iframe. All in a browser. Demo-ready in < 1 day.

---

## Files Aman needs to create

```
aman/ui/
├── README.md          ← this spec, condensed
├── visualizer.py      ← RerunLogger class — log_step(), log_episode_start()
└── demo_app.py        ← Gradio app — sliders, toggles, embedded Rerun view
```

---

## Files Anas needs to modify

```
anas/env/shielded_env.py   ← add optional RerunLogger hook in step()
anas/evaluation/run_stress_test.py  ← pass --visualize flag to enable logging
```

---

## Install commands (Aman documents these in aman/ui/README.md)

```bash
pip install rerun-sdk
pip install gradio        # for the interactive control panel
```

---

## Alternative options (if Rerun causes issues)

| Option | Pros | Cons |
|--------|------|------|
| **Matplotlib 3D + FuncAnimation** | Zero new deps, already in stack | Slow for 50+ drones, not interactive |
| **Plotly Dash** | Good charts, browser-native | 3D agent animation is clunky |
| **Three.js via FastAPI WebSocket** | Best browser 3D visuals | Requires JavaScript, 1–2 days extra |
| **PyBullet GUI** | Built-in physics + visualization | Couples physics to rendering, hard to decouple from Flightmare |
| **Rerun.io** ✅ recommended | Native robotics tool, 3D, timeline, Python-only | Newer project, API changes between versions |

---

## Demo video integration (ticket M-11)

The 3-minute demo video is recorded directly from the Rerun viewer or Gradio app using OBS. No separate recording setup needed.

Timeline of the demo:
```
0:00–0:40  Normal flight — 50 drones, green, reaching goals
0:40–1:20  Chaos injected — Byzantine drone fires, comm delay enabled
1:20–2:00  Shield activates — affected drones turn red, metrics spike
2:00–2:40  Recovery — shield keeps separation, zero collisions
2:40–3:00  Metrics summary screen: Throughput 90%, Collisions 0, Interventions 45
```
