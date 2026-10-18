# TASKS.md — 30-Day Execution Plan
> Proof-of-Concept phase. Goal by Day 30: working simulation demo, white paper, pitch deck.
> Each ticket is atomic — one owner, one deliverable, one acceptance criterion.
> Work on your branch. Open a PR into `dev` when the AC is met.

---

## Branch ownership
| Person | Branch | Role |
|--------|--------|------|
| Salman | `salman/cbf-safety` | Math — CBF safety layer |
| Anas | `anas/marl-policy` | ML — MARL training & simulation engine |
| Aman | `aman/infra-api` | Infra, API, business outputs, UI |

---

## WEEK 1 — Foundation & Baseline (Days 1–7)
**Gate**: Sim runs 10 drones. CBF math defined. Server up. Abstract written.

---

### S-01 · Pairwise CBF barrier function
- **Owner**: Salman
- **Deliverable**: `salman/math/cbf_pairwise.py`
- **What to build**: Implement `h_ij(x)`, its gradient, and Lie derivatives `Lf_h`, `Lg_h` — no QP yet, pure NumPy.
- **AC**: Script runs standalone. Given two drone positions, prints `h_ij`, `Lf_h`, `Lg_h`. Positive h means safe.

---

### S-02 · Lexicographic Rulebook
- **Owner**: Salman
- **Deliverable**: `salman/math/rulebook.py`
- **What to build**: Three-level priority encoder: Safety > Geofence > Mission Progress. Each rule returns a cost scalar and a boolean feasibility flag.
- **AC**: `rulebook.evaluate(state, action)` works. Three hand-crafted test cases pass (collision, geofence breach, normal flight).

---

### S-03 · CBF derivation notebook
- **Owner**: Salman
- **Deliverable**: `salman/docs/cbf_derivation.ipynb`
- **What to build**: Jupyter notebook walking through: control-affine model → first-order CBF → HOCBF for relative degree 2 → QP formulation. Feeds directly into the white paper.
- **AC**: Notebook renders without errors. All equations match the PDF report.

---

### A-01 · PettingZoo drone environment (10 drones)
- **Owner**: Anas
- **Deliverable**: `anas/env/drone_env.py`
- **What to build**: Custom `ParallelEnv`. State = position + velocity + goal. Action = 3D acceleration. Reward: +1 reach goal, −100 collision. 1km × 1km × 200m airspace.
- **AC**: `env.reset()` and `env.step(actions)` work for N=10. Random policy runs 100 episodes without Python crash.

---

### A-02 · Baseline MARL policy (no shield)
- **Owner**: Anas
- **Deliverable**: `anas/training/train_baseline.py` + checkpoint at `anas/checkpoints/baseline_v1.pt`
- **What to build**: PPO/IPPO policy trained for 500k steps via RLlib or TorchRL. Log to W&B. Expect collisions — this is the unsafe baseline.
- **AC**: W&B run exists. Checkpoint saved. Episode reward improving over training.

---

### A-03 · Action schema definition
- **Owner**: Anas
- **Deliverable**: `anas/env/action_schema.py`
- **What to build**: `MARLAction` dataclass — `agent_id`, `proposed_accel (3,)`, `timestamp_ms`. This is the exact struct Salman's CBF-QP receives.
- **AC**: Dataclass defined and JSON-serializable. Committed to `shared/schemas.py` and documented in `INTERFACES.md`.

---

### M-01 · Provision cloud simulation server
- **Owner**: Aman
- **Deliverable**: `aman/infra/setup.sh` + `aman/infra/server.md`
- **What to build**: Shell script to provision an AWS EC2 `c5.2xlarge` (Ubuntu 22.04). Install Python 3.11, Docker, Git. `server.md` documents the IP and SSH access — no private keys in git.
- **AC**: All three teammates can SSH into the server. Python and Git confirmed working.

---

### M-02 · Install simulation stack
- **Owner**: Aman
- **Deliverable**: `aman/infra/install_sim.sh`
- **What to build**: Script to install BlueSky ATM simulator and Flightmare physics engine on the server. Document any version conflicts found.
- **AC**: `import bluesky` and `import flightmare` both succeed on the server.

---

### M-03 · Technical Abstract (1 page)
- **Owner**: Aman
- **Deliverable**: `aman/docs/technical_abstract.md`
- **What to build**: One page. What we're building, why it's different (CBF shield on MARL), who the audience is. No undefined jargon.
- **AC**: Reviewed and signed off by Salman (math) and Anas (ML) before end of Week 1.

---

### M-04 · Competitor research & outreach targets
- **Owner**: Aman
- **Deliverable**: `aman/docs/competitors.md`
- **What to build**: Research Zipline, Wing, Skypuzzler, Joby, Shield AI. For each: product focus, tech approach (from public info), head of autonomy/engineering name from LinkedIn. Draft outreach email template.
- **AC**: 5 companies, at least one named contact each.

---

## WEEK 2 — Safety Shield & Tactical Layer (Days 8–14)
**Gate**: CBF-QP runs inside simulation loop. Shielded 10-drone run = zero collisions.

---

### S-04 · CBF-QP solver
- **Owner**: Salman
- **Deliverable**: `salman/cbf/qp_solver.py`
- **What to build**: CVXPY QP — minimize deviation from proposed actions subject to all pairwise CBF constraints. Public API: `solve_cbf_qp(states, proposed_actions) → list[SafeAction]`.
- **AC**: Given 10 drones with 3 pairs near-collision, returns safe actions. Runtime < 50ms for N=10. Unit tests pass.
- **Needs**: S-01 complete, A-03 schema agreed.

---

### S-05 · High-Order CBF (HOCBF)
- **Owner**: Salman
- **Deliverable**: `salman/cbf/hocbf.py`
- **What to build**: Position separation is relative degree 2 under acceleration control. Implement the HOCBF auxiliary variable `ψ₁` and extend the QP from S-04.
- **AC**: Test: two drones approaching at 10 m/s with 2m separation — solver applies braking before collision.
- **Needs**: S-04 complete.

---

### S-06 · Shield intervention logger
- **Owner**: Salman
- **Deliverable**: `salman/cbf/shield_logger.py`
- **What to build**: Logs every time the shield modifies an action. CSV columns: `timestep`, `agent_id`, `proposed_accel_xyz`, `safe_accel_xyz`, `h_value`, `constraint_active`, `qp_feasible`, `fallback_used`. Plotter included.
- **AC**: Logger writes CSV. Visualizer script renders shield interventions over time. Aman's analysis scripts depend on this column schema — do not rename columns without updating INTERFACES.md.

---

### A-04 · Integrate CBF shield into simulation loop
- **Owner**: Anas
- **Deliverable**: `anas/env/shielded_env.py`
- **What to build**: Wrapper around `drone_env.py`. After MARL outputs actions, pass through `solve_cbf_qp()`. Return shielded actions to physics. MARL doesn't know it was overridden.
- **AC**: 10-drone episode with baseline policy + shield runs to completion. Logger records interventions. Zero collisions.
- **Needs**: S-04, A-02.

---

### A-05 · Re-train with shield awareness
- **Owner**: Anas
- **Deliverable**: `anas/training/train_shielded.py` + checkpoint `shielded_v1.pt`
- **What to build**: Add a small penalty for triggering the shield (`-0.5 × shield_delta_norm`). Re-train 500k steps. Policy learns to be more conservative near other drones.
- **AC**: Shield intervention rate drops vs baseline. W&B comparison logged.
- **Needs**: A-04.

---

### M-05 · Sovereign Core mock API
- **Owner**: Aman
- **Deliverable**: `aman/api/sovereign_core.py`
- **What to build**: FastAPI server on port 8000. Endpoints: `GET /airspace/rules` (geofences, altitude limits, separation min), `GET /airspace/notam`, `POST /drone/register`. Dockerfile included.
- **AC**: `curl localhost:8000/airspace/rules` returns valid JSON. Starts with `uvicorn`.

---

### M-06 · Simulation client for the API
- **Owner**: Aman
- **Deliverable**: `aman/api/sim_client.py`
- **What to build**: Python client the simulation calls at episode start to fetch airspace rules. Rules feed into Salman's Rulebook (S-02).
- **AC**: Changing a rule in the API changes sim behavior within one episode.
- **Needs**: M-05, S-02.

---

### M-07 · White Paper draft (Sections 1–3)
- **Owner**: Aman
- **Deliverable**: `aman/docs/white_paper_draft.md`
- **What to build**: Introduction, Problem Statement, Architecture Overview. Pull CBF math from S-03 notebook. Include architecture diagram.
- **AC**: 4–5 pages written. Salman reviews math sections.

---

## WEEK 3 — Falsification Stress Test (Days 15–21)
**Gate**: 50-drone simulation under chaos. Data exported. Safety holds. Charts generated.

---

### S-07 · Latency analysis
- **Owner**: Salman
- **Deliverable**: `salman/analysis/latency_analysis.py`
- **What to build**: Analytically bound how a 2-second comm delay degrades CBF guarantees. For max speed 15 m/s and 2s delay, compute required `d_safe` inflation. Output goes directly into the white paper.
- **AC**: Script prints: "At 2s delay and 15 m/s, expand d_safe by X meters to maintain guarantee."

---

### S-08 · HJ reachability fallback trigger
- **Owner**: Salman
- **Deliverable**: `salman/cbf/fallback_trigger.py`
- **What to build**: If `h(x) < threshold` AND QP is infeasible, trigger hover-in-place fallback. Log as "critical intervention."
- **AC**: In a test where 3 drones converge and QP becomes infeasible — no crash, fallback fires.

---

### A-06 · Scale to 50 drones
- **Owner**: Anas
- **Deliverable**: `anas/training/train_scale50.py`
- **What to build**: Scale shielded env to N=50. Optimize QP solver call — prune distant pairs, batch constraints. Target: full step loop < 200ms.
- **AC**: 50-drone episode runs 1000 steps without timeout. Per-step timing profiled and logged.
- **Needs**: A-05, S-05.

---

### A-07 · Chaos injection module
- **Owner**: Anas
- **Deliverable**: `anas/chaos/chaos_injector.py`
- **What to build**: Three injectors togglable mid-episode: (1) `CommDelay` — stales state fed to policy. (2) `ByzantineDrone` — broadcasts random false telemetry. (3) `WindGust` — noise on physics.
- **AC**: Each injector toggles on/off. Under chaos, MARL degrades; CBF shield maintains safety.

---

### A-08 · Stress test runner & metrics export
- **Owner**: Anas
- **Deliverable**: `anas/evaluation/run_stress_test.py` + `results/stress_test_50drones.csv`
- **What to build**: 100 episodes, N=50, all three chaos injectors active. Collect: throughput, collisions, shield interventions, fallback triggers, min separation.
- **AC**: CSV saved. Summary printed: "Throughput: X%. Collisions: Y. Shield interventions: Z."

---

### M-08 · Data analysis & visualization
- **Owner**: Aman
- **Deliverable**: `aman/analysis/plot_results.py` → 4 charts in `results/figures/`
- **What to build**: From the stress test CSV — (1) Throughput vs drone density. (2) Shield interventions over time. (3) Min separation CDF. (4) Fallback rate vs latency level.
- **AC**: 4 publication-quality PNGs saved.
- **Needs**: A-08 CSV (column schema fixed in INTERFACES.md — do not change without coordination).

---

### M-09 · Seed Pitch Deck outline
- **Owner**: Aman
- **Deliverable**: `aman/docs/pitch_deck_outline.md`
- **What to build**: 10-slide outline for Gamma.app: Problem, Solution, Architecture, Demo Results (use charts from M-08), Team, Market Size, Go-to-Market, Competition, Ask. Key bullets per slide.
- **AC**: Outline complete, ready to drop into Gamma.app.

---

### UI-01 · Simulation visualizer (browser-based, no Unity)
- **Owner**: Aman (Anas hooks simulation output to it)
- **Deliverable**: `aman/ui/README.md` (plan + architecture) + `aman/ui/visualizer.py`
- **What to build**: See `VISUALIZATION.md` for the full spec. Short version: Python + Rerun.io for live 3D drone positions streamed from inside the sim loop. Browser-viewable. No Unity, no C#.
- **AC**: Simulation runs 10 drones, browser shows live 3D positions, drones change color when shield fires.
- **Needs**: A-04 (shielded env), Anas adds one `rr.log()` call per step.

---

## WEEK 4 — Package & Outreach (Days 22–30)
**Gate**: White paper final. Demo video recorded. Repo professional. Outreach sent.

---

### S-09 · Technical blog post
- **Owner**: Salman
- **Deliverable**: `salman/docs/blog_cbf_shield.md`
- **What to build**: ~1500 words. The shield feasibility problem, HOCBF solution, code snippets. Target: Medium or Substack.
- **AC**: Reviewed by Anas. Ready to publish.

---

### S-10 · White paper math review
- **Owner**: Salman
- **Task**: Review all math in Aman's `white_paper_final.md`. Verify equations match implementation. Add formal theorem: "CBF shield maintains safety under bounded delay δt if d_safe is expanded by v_max · δt."
- **AC**: PR comment approval on Aman's final white paper PR.

---

### A-09 · ONNX export & edge benchmark
- **Owner**: Anas
- **Deliverable**: `anas/edge/export_onnx.py` + `results/edge_benchmark.md`
- **What to build**: Export shielded MARL policy to ONNX. Benchmark inference latency on CPU, GPU, and simulated Jetson Orin (ONNX Runtime thread-limited). Target: < 10ms per step.
- **AC**: Benchmark table saved. ONNX model at `anas/checkpoints/shielded_v1.onnx`.

---

### A-10 · Technical blog post
- **Owner**: Anas
- **Deliverable**: `anas/docs/blog_edge_marl.md`
- **What to build**: ~1500 words. MARL edge optimization: network architecture, ONNX export, quantization options, latency vs safety tradeoff.
- **AC**: Reviewed by Salman. Ready to publish.

---

### M-10 · Finalize White Paper (10–15 pages)
- **Owner**: Aman
- **Deliverable**: `aman/docs/white_paper_final.md` → PDF via Pandoc
- **What to build**: Full paper: Abstract, Intro, Architecture, CBF Shield, MARL Policy, Evaluation, Edge Deployment, Conclusion, References (32 from the PDF report). IEEE/arXiv format.
- **AC**: Salman approves math (S-10). All charts included. PDF generated.

---

### M-11 · Demo video
- **Owner**: Aman
- **Deliverable**: `aman/docs/demo_script.md` + recorded video
- **What to build**: 3-minute demo. Show normal flight → inject Byzantine drone → MARL starts failing → CBF shield fires (drones change color) → metrics show zero collisions. Record with OBS.
- **AC**: Video recorded. Link in `demo_script.md`.
- **Needs**: UI-01 (visualizer with color change), A-08.

---

### M-12 · Outreach emails
- **Owner**: Aman
- **Deliverable**: `aman/docs/outreach_log.md`
- **What to build**: Personalized emails to 5 contacts from M-04. Subject: "Proving safe autonomous drone deconfliction under C2 latency." Attach white paper.
- **AC**: 5 sent, log updated with dates and any replies.
- **Needs**: M-10 (white paper final).

---

### M-13 · Repo cleanup
- **Owner**: Aman
- **Task**: Every directory gets a README. Top-level README updated with demo GIF, architecture diagram, setup instructions, links to paper and blog. Add `CONTRIBUTING.md`. No dead files, no committed secrets.
- **AC**: Repo looks professional on GitHub. `.gitignore` covers `.pt`, `.onnx`, large CSVs.

---

## Dependency map

```
S-01 ──────────────────────────────► S-04 ──► S-05 ──► S-07
                                       │
S-02 ──────────────────────────────► M-06       │
                                                 ▼
A-01 ──► A-02 ──► A-03 ──────────► A-04 ──► A-05 ──► A-06 ──► A-07 ──► A-08
                                     │                                    │
M-01 ──► M-02 ──► M-05 ──► M-06     │                                    ▼
                                     ▼                              M-08 ──► M-09
                                   UI-01 ──────────────────────────► M-11
                                                                       │
                                                          S-10 + A-09 + M-08
                                                                       │
                                                                     M-10 ──► M-12
```

---

## Weekly sync (30 min, every Sunday)

1. Each person demos their merged work — live, not slides.
2. Any blocked ticket? Pair program or reassign.
3. Walk the dependency map — are interfaces honored?
4. Agree on next week's priority order.

**Rule**: Stuck for > 4 hours → open a GitHub Discussion, not a DM. The whole team learns from it.
