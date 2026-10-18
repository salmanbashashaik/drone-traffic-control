# aman/ — Infrastructure, API & Business
**Owner**: Aman | **Branch**: `aman/infra-api`

You own **three things simultaneously**: the cloud infrastructure, the Sovereign Core API (the mock airspace authority), and all the business-facing outputs (white paper, pitch deck, video, outreach). The team cannot deploy without you and cannot pitch without you.

## Your tickets

| Ticket | Week | File | Status |
|--------|------|------|--------|
| M-01 | 1 | `infra/setup.sh` | Starter provided |
| M-02 | 1 | `infra/install_sim.sh` | You create |
| M-03 | 1 | `docs/technical_abstract.md` | You create |
| M-04 | 1 | `docs/competitors.md` | You create |
| M-05 | 2 | `api/sovereign_core.py` | Starter provided |
| M-06 | 2 | `api/sim_client.py` | You create |
| M-07 | 2 | `docs/white_paper_draft.md` | You create |
| M-08 | 3 | `analysis/plot_results.py` | You create |
| M-09 | 3 | `docs/pitch_deck_outline.md` | You create |
| M-10 | 4 | `docs/white_paper_final.md` | You create |
| M-11 | 4 | `docs/demo_script.md` | You create |
| M-12 | 4 | `docs/outreach_log.md` | You create |
| M-13 | 4 | Repo cleanup | PR into dev |

## Directory layout

```
aman/
├── infra/
│   ├── setup.sh           ← M-01: server provisioning script
│   ├── install_sim.sh     ← M-02: BlueSky + Flightmare install
│   └── server.md          ← M-01: IP/access docs (no keys in git!)
├── api/
│   ├── sovereign_core.py  ← M-05: FastAPI airspace authority
│   └── sim_client.py      ← M-06: Python client for simulation
├── analysis/
│   └── plot_results.py    ← M-08: generates 4 charts from stress test CSV
└── docs/
    ├── technical_abstract.md  ← M-03
    ├── competitors.md         ← M-04
    ├── white_paper_draft.md   ← M-07
    ├── pitch_deck_outline.md  ← M-09
    ├── white_paper_final.md   ← M-10
    ├── demo_script.md         ← M-11
    └── outreach_log.md        ← M-12
```

## Interface you must honor

**API contract**: `INTERFACES.md § API-AIRSPACE` — do NOT change endpoint paths without a PR.
**Input from Anas**: `results/stress_test_50drones.csv` — column names fixed, see `INTERFACES.md § SHIELD-LOG`.
**Input from Salman**: White paper math — pull from `salman/docs/cbf_derivation.ipynb` and his PR comments.

## API quick start

```bash
# Install deps
pip install fastapi uvicorn httpx

# Run the API
uvicorn aman.api.sovereign_core:app --reload --port 8000

# Test it
curl http://localhost:8000/airspace/rules
curl http://localhost:8000/airspace/notam
curl -X POST http://localhost:8000/drone/register \
  -H "Content-Type: application/json" \
  -d '{"agent_id": "drone_001", "operator": "sim", "mission_type": "delivery"}'
```

## sim_client.py skeleton (ticket M-06)

```python
import httpx
from shared.schemas import DroneState

BASE_URL = "http://localhost:8000"

def fetch_airspace_rules() -> dict:
    resp = httpx.get(f"{BASE_URL}/airspace/rules")
    resp.raise_for_status()
    return resp.json()

def register_drone(agent_id: str) -> str:
    resp = httpx.post(f"{BASE_URL}/drone/register", json={
        "agent_id": agent_id, "operator": "sim", "mission_type": "delivery"
    })
    return resp.json()["transponder_code"]
```

## White Paper structure (M-07 / M-10)

```
1. Abstract (150 words)
2. Introduction — why UTM needs formal safety
3. Architecture — MARL + CBF + fallback diagram
4. CBF Shield — pull math from salman/docs/cbf_derivation.ipynb
5. MARL Policy — pull from anas/docs
6. Evaluation — 50-drone stress test, chaos injection results
7. Edge Deployment — ONNX benchmark from A-09
8. Conclusion & Future Work
9. References (use the 32 from the PDF report)
```

Use LLM prompt:
```
Draft Section 2 of a technical white paper titled "Runtime Assurance for Autonomous Drone Traffic: A Hybrid MARL + CBF Architecture". 
The audience is FAA regulators and deep-tech VCs. Use the vocabulary: HOCBF, runtime assurance, reachability, control-affine system.
Keep it under 400 words.
```

## Week 1 definition of done

- [ ] Server is running, team can SSH in
- [ ] `setup.sh` executed successfully on server
- [ ] `technical_abstract.md` written and reviewed by Salman + Anas
- [ ] `competitors.md` complete with 5 companies + contacts
- [ ] PR opened into `dev`
