# aman/ — Your Work
**Branch**: `aman/infra-api` | **Role**: Infrastructure, API, Visualization, Business

You own four things simultaneously: cloud infrastructure, the Sovereign Core API (the mock airspace authority), the simulation visualizer/UI, and all business outputs (white paper, pitch deck, demo video, outreach). The team cannot deploy without you and cannot pitch without you.

---

## Your 14 tickets

| ID | Week | File to create | What it is |
|----|------|----------------|------------|
| M-01 | 1 | `infra/setup.sh` | Server provisioning script (AWS/GCP, Python 3.11, Docker) |
| M-02 | 1 | `infra/install_sim.sh` | Install BlueSky + Flightmare on the server |
| M-03 | 1 | `docs/technical_abstract.md` | 1-page: what we're building and why it's different |
| M-04 | 1 | `docs/competitors.md` | 5 companies, named contacts, outreach email template |
| M-05 | 2 | `api/sovereign_core.py` | FastAPI airspace authority — rules, NOTAMs, drone registration |
| M-06 | 2 | `api/sim_client.py` | Python client the simulation calls to fetch airspace rules |
| M-07 | 2 | `docs/white_paper_draft.md` | Sections 1–3: Intro, Problem, Architecture |
| UI-01 | 2–3 | `ui/visualizer.py` + `ui/demo_app.py` | Rerun.io live 3D viewer + Gradio control panel |
| M-08 | 3 | `analysis/plot_results.py` | 4 publication-quality charts from stress test CSV |
| M-09 | 3 | `docs/pitch_deck_outline.md` | 10-slide outline for Gamma.app |
| M-10 | 4 | `docs/white_paper_final.md` → PDF | Final white paper, 10–15 pages |
| M-11 | 4 | `docs/demo_script.md` + video | 3-minute demo recording via OBS |
| M-12 | 4 | `docs/outreach_log.md` | Send 5 emails, log replies |
| M-13 | 4 | (PR into dev) | Repo cleanup: every dir has a README, no secrets committed |

---

## Sovereign Core API spec (M-05)

FastAPI server on port 8000. Three endpoints (see `INTERFACES.md § API-AIRSPACE` for exact JSON schemas):

| Endpoint | Purpose |
|----------|---------|
| `GET /airspace/rules` | Returns geofences, altitude limits, separation minimum — called once per episode |
| `GET /airspace/notam` | Returns active temporary restrictions |
| `POST /drone/register` | Registers a drone at episode start, returns transponder code |

The rules JSON is read by Salman's `rulebook.py` — do not rename fields without updating `INTERFACES.md` and coordinating with Salman.

---

## Visualization / UI spec (UI-01)

Full spec is in `VISUALIZATION.md`. Summary:

**Tool**: Rerun.io (`pip install rerun-sdk`)  
**Why**: Python-native, 3D, browser-viewable, built for robotics. No Unity, no C#.  
**Control panel**: Gradio (`pip install gradio`) — sliders for drone count, toggles for chaos injectors, embedded Rerun viewer.

What the viewer must show:
- Live 3D drone positions
- Drone color: **green** = normal, **red** = shield currently active
- Goal markers
- Geofence cylinder
- Real-time metrics: intervention count, min separation, throughput

You write `aman/ui/visualizer.py` with a `RerunLogger` class. Anas calls `log_step()` from inside the simulation loop. See `INTERFACES.md § RERUN-LOG` for the method signature — agree it with Anas before building.

---

## White paper structure (M-07 / M-10)

```
1. Abstract (150 words)
2. Introduction — why UTM needs formal safety
3. Architecture — MARL + CBF + fallback (use the mermaid diagram from README)
4. CBF Safety Shield — pull math from salman/docs/cbf_derivation.ipynb
5. MARL Policy — pull from anas/docs
6. Evaluation — stress test results, charts from M-08
7. Edge Deployment — ONNX benchmark from A-09
8. Conclusion & Future Work
9. References — the 32 from the PDF report
```

LLM prompt for Section 2:
```
Draft the Introduction section of a technical white paper titled
"Runtime Assurance for Autonomous Drone Traffic: A Hybrid MARL + CBF Architecture."
Audience: FAA regulators and deep-tech VCs.
Use these terms correctly: HOCBF, runtime assurance, reachability, control-affine system.
400 words max. Cite the FAA UTM ConOps v2.0 and NASA MARL papers.
```

---

## Charts required (M-08)

From `results/stress_test_50drones.csv` (Anas produces this):

1. **Throughput vs drone density** — bar chart, x-axis = N (10, 25, 50), y-axis = % reaching goal
2. **Shield interventions over time** — line chart, x = timestep, y = cumulative interventions
3. **Min separation CDF** — x = separation in meters, y = probability — show safety margin clearly
4. **Fallback rate vs communication latency** — x = delay_ms, y = fallback triggers per episode

CSV column names are fixed — see `INTERFACES.md § SHIELD-LOG`.

---

## Competitor research targets (M-04)

Research these five companies using Perplexity:
- **Zipline** — medical drone delivery, fixed-wing
- **Wing (Alphabet)** — package delivery, DAA systems
- **Skypuzzler** — UTM software platform
- **Joby Aviation** — eVTOL, autonomy stack
- **Shield AI** — autonomous military/civil drones (relevant for CBF/formal methods angle)

For each: what problem they solve, their tech approach (public info only), and the name + LinkedIn of their Head of Autonomy or Engineering.

---

## Demo video timeline (M-11)

Record from the Rerun/Gradio viewer using OBS:
```
0:00–0:40  50 drones flying normally, green, reaching goals
0:40–1:20  Byzantine drone injected — one drone erratic, others start reacting
1:20–2:00  CBF shield fires — affected drones go red — separation holds
2:00–2:40  Comm delay injected — MARL performance drops, shield compensates
2:40–3:00  Metrics summary: Throughput 90%, Collisions 0, Interventions 45
```

---

## Week 1 done means

- [ ] Server accessible by all three teammates via SSH
- [ ] `setup.sh` executed successfully, Python + Docker confirmed
- [ ] `technical_abstract.md` reviewed and approved by Salman + Anas
- [ ] `competitors.md` complete with 5 companies + contacts
- [ ] PR opened into `dev`
