# aman/ — Your Work
**Branch**: `aman/infra-api` | **Role**: Infrastructure + API + UI + Business

You make the whole thing deployable, visible, and sellable. Without you the simulation has no server to run on, no airspace rules to read, no visual to show investors, and no paper to submit. You are also the only person talking to the outside world — outreach, white paper, pitch deck.

---

## Your 13 tickets

| ID | Week | Deliverable | What it is |
|----|------|-------------|------------|
| M-01 | 1 | `infra/setup.sh` + `infra/server.md` | Provision AWS/GCP server, document access |
| M-02 | 1 | `infra/install_sim.sh` | Install BlueSky + Flightmare on server |
| M-03 | 1 | `docs/technical_abstract.md` | 1-page: what we're building and why |
| M-04 | 1 | `docs/competitors.md` | 5 companies, named contacts, outreach template |
| M-05 | 2 | `api/sovereign_core.py` | FastAPI airspace authority server |
| M-06 | 2 | `api/sim_client.py` | Python client sim uses to fetch airspace rules |
| M-07 | 2 | `docs/white_paper_draft.md` | Sections 1–3: Intro, Problem, Architecture |
| UI-01 | 2–3 | `ui/visualizer.py` + `ui/demo_app.py` | Rerun.io 3D viewer + Gradio control panel |
| M-08 | 3 | `analysis/plot_results.py` → 4 PNGs | Charts from stress test CSV |
| M-09 | 3 | `docs/pitch_deck_outline.md` | 10-slide outline for Gamma.app |
| M-10 | 4 | `docs/white_paper_final.md` → PDF | Final white paper, 10–15 pages |
| M-11 | 4 | `docs/demo_script.md` + video | 3-minute demo recording |
| M-12 | 4 | `docs/outreach_log.md` | 5 outreach emails sent, replies logged |
| M-13 | 4 | PR into dev | Full repo cleanup |

---

## Directory layout

```
aman/
├── infra/
│   ├── setup.sh              ← M-01
│   ├── install_sim.sh        ← M-02
│   └── server.md             ← M-01: IP + SSH docs (no keys in git)
├── api/
│   ├── sovereign_core.py     ← M-05
│   └── sim_client.py         ← M-06
├── ui/
│   ├── visualizer.py         ← UI-01: RerunLogger class
│   └── demo_app.py           ← UI-01: Gradio control panel
├── analysis/
│   └── plot_results.py       ← M-08
└── docs/
    ├── technical_abstract.md ← M-03
    ├── competitors.md        ← M-04
    ├── white_paper_draft.md  ← M-07
    ├── pitch_deck_outline.md ← M-09
    ├── white_paper_final.md  ← M-10
    ├── demo_script.md        ← M-11
    └── outreach_log.md       ← M-12
```

---

## Sovereign Core API (M-05)

FastAPI server on port 8000. Three endpoints — full JSON schemas in `INTERFACES.md § API-AIRSPACE`:

| Endpoint | Called by | Purpose |
|----------|-----------|---------|
| `GET /airspace/rules` | Simulation (via sim_client.py) once per episode | Geofences, altitude limits, separation minimum |
| `GET /airspace/notam` | Simulation at episode start | Temporary restrictions |
| `POST /drone/register` | Simulation per drone at episode start | Returns transponder code |

Salman's `rulebook.py` reads the JSON from `GET /airspace/rules` — do not rename fields without updating `INTERFACES.md` and telling Salman.

---

## Visualization UI (UI-01)

Full spec in `VISUALIZATION.md`. The short version:

**Tools**: Rerun.io (3D live stream) + Gradio (browser control panel). Install: `pip install rerun-sdk gradio`. No Unity, no C#, no JavaScript.

**You write** `aman/ui/visualizer.py` — a `RerunLogger` class with one public method:
```
log_step(states, safe_actions, step_metrics)
```
Anas calls this at the end of every `ShieldedEnv.step()`. The method signature is in `INTERFACES.md § RERUN-LOG` — agree it with Anas before Week 3.

**You write** `aman/ui/demo_app.py` — a Gradio app with:
- Slider: number of drones (1–50)
- Toggles: CommDelay on/off, ByzantineDrone on/off, WindGust on/off
- Embedded Rerun viewer (or link to it)
- Live metrics panel: interventions, min separation, throughput

**Anas adds** one `self.rerun_logger.log_step(...)` call at the end of `ShieldedEnv.step()`.

Drone color rule: green by default, red when `shield_active == True` in the `SafeAction` for that drone.

---

## Charts to produce (M-08)

From `results/stress_test_50drones.csv` (Anas produces this, Week 3):

| Chart | X axis | Y axis |
|-------|--------|--------|
| Throughput vs density | N (10, 25, 50) | % reaching goal |
| Shield interventions over time | Timestep | Cumulative interventions |
| Min separation CDF | Separation (m) | Probability |
| Fallback rate vs latency | delay_ms | Fallbacks per episode |

Use matplotlib or seaborn. Save as PNG at 300 DPI to `results/figures/`. These go directly into the white paper and pitch deck.

---

## White paper structure (M-07 / M-10)

```
1. Abstract (150 words)
2. Introduction — UTM problem, why RL alone isn't enough
3. Architecture — MARL + CBF + fallback (use mermaid diagram from README)
4. RL Policy — pull from Salman's training spec + W&B results
5. CBF Safety Shield — pull math from salman/docs/cbf_derivation.ipynb
6. Evaluation — stress test results + charts from M-08
7. Edge Deployment — ONNX benchmark from A-07
8. Conclusion & Future Work
9. References (32 from the PDF report)
```

LLM prompt for Section 2:
```
Draft the Introduction of a white paper titled
"Runtime Assurance for Autonomous Drone Traffic: A Hybrid MARL + CBF Architecture."
Audience: FAA regulators and deep-tech VCs.
Use: HOCBF, runtime assurance, reachability, control-affine system.
Under 400 words. Reference FAA UTM ConOps v2.0 and NASA MARL papers.
```

---

## Competitor research (M-04)

Use Perplexity to research these five and find one named contact (LinkedIn, Head of Autonomy or Engineering) per company:

| Company | Angle |
|---------|-------|
| Zipline | Medical delivery, formal safety angle |
| Wing (Alphabet) | Package delivery, DAA systems |
| Skypuzzler | UTM software platform |
| Joby Aviation | eVTOL, autonomy stack |
| Shield AI | Autonomous platforms, CBF/formal methods audience |

---

## Demo video timeline (M-11)

3 minutes, record from Rerun/Gradio with OBS:
```
0:00–0:40  50 drones flying, green, reaching goals normally
0:40–1:20  Byzantine drone injected → erratic movement, MARL starts failing
1:20–2:00  CBF shield fires → affected drones turn red → separation holds
2:00–2:40  Comm delay added → MARL degrades further, shield compensates
2:40–3:00  Summary screen: Throughput 90%, Collisions 0, Interventions 45
```

---

## Week 1 done means

- [ ] Server running, all three can SSH in
- [ ] `technical_abstract.md` reviewed by Salman + Anas
- [ ] `competitors.md` with 5 companies + contacts
- [ ] PR into `dev`
