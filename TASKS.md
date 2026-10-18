# TASKS.md — 30-Day Execution Plan
> Proof-of-Concept phase. Goal by Day 30: working simulation demo, white paper, pitch deck.
> Each ticket is atomic — one owner, one deliverable, one acceptance criterion.
> Work on your branch. Open a PR into `dev` when the AC is met.

---

## Roles
| Person | Branch | Role |
|--------|--------|------|
| **Salman** | `salman/cbf-safety` | RL + Formal Safety Math |
| **Anas** | `anas/marl-policy` | ML + Edge AI + Simulation Engine |
| **Aman** | `aman/infra-api` | Infrastructure + API + UI + Business |

---

## WEEK 1 — Foundation & Baseline (Days 1–7)
**Gate**: Sim env runs. CBF math defined on paper. Baseline policy training started. Server up.

---

### S-01 · Pairwise CBF barrier function
- **Owner**: Salman
- **Deliverable**: `salman/math/cbf_pairwise.py`
- **What**: Implement `h_ij(x)`, gradient, Lie derivatives `Lf_h` and `Lg_h` — pure NumPy, no QP yet.
- **AC**: Script runs standalone. Given two drone positions, prints `h_ij`, `Lf_h`, `Lg_h`. Positive h = safe.

---

### S-02 · Lexicographic Rulebook
- **Owner**: Salman
- **Deliverable**: `salman/math/rulebook.py`
- **What**: Three-level priority: Safety > Geofence > Mission Progress. Returns cost scalar + feasibility flag per rule.
- **AC**: `rulebook.evaluate(state, action)` works. Three hand-crafted test cases pass.

---

### S-03 · CBF derivation notebook
- **Owner**: Salman
- **Deliverable**: `salman/docs/cbf_derivation.ipynb`
- **What**: Jupyter notebook: control-affine model → first-order CBF → HOCBF for relative degree 2 → QP. This feeds directly into the white paper.
- **AC**: Renders without errors. Equations match the PDF report.

---

### S-04 · Design RL training pipeline (architecture + reward spec)
- **Owner**: Salman
- **Deliverable**: `salman/rl/training_spec.md`
- **What**: Document the full RL setup: policy network architecture (shared vs. independent), observation space layout (must match `INTERFACES.md § ENV-OUTPUT`), reward function design, training framework choice (TorchRL/RLlib), hyperparameters, W&B logging plan. No code yet — spec first.
- **AC**: Anas has reviewed and confirmed the env matches this spec. Document merged into `dev`.

---

### A-01 · PettingZoo drone environment (10 drones)
- **Owner**: Anas
- **Deliverable**: `anas/env/drone_env.py`
- **What**: Custom `ParallelEnv`. State = position + velocity + goal. Action = 3D acceleration. 1km × 1km × 200m airspace. Reward: +1 reach goal, −100 collision, small negative distance shaping.
- **AC**: `env.reset()` and `env.step(actions)` work for N=10. Random policy runs 100 episodes without crash.

---

### A-02 · Action schema definition
- **Owner**: Anas (coordinates with Salman)
- **Deliverable**: `shared/schemas.py` — `MARLAction` dataclass
- **What**: `MARLAction` fields: `agent_id`, `proposed_accel (3,)`, `timestamp_ms`. Agrees with Salman's CBF solver input.
- **AC**: Committed to `shared/schemas.py`. `DroneState` and `SafeAction` stubs also added (full implementation by S-01 / S-05).

---

### M-01 · Provision cloud server
- **Owner**: Aman
- **Deliverable**: `aman/infra/setup.sh` + `aman/infra/server.md`
- **What**: AWS EC2 `c5.2xlarge`, Ubuntu 22.04. Python 3.11, Docker, Git. IP + SSH access documented.
- **AC**: All three teammates SSH in successfully.

---

### M-02 · Install simulation stack
- **Owner**: Aman
- **Deliverable**: `aman/infra/install_sim.sh`
- **What**: Install BlueSky + Flightmare on the server. Document version conflicts.
- **AC**: Both import successfully on the server.

---

### M-03 · Technical Abstract
- **Owner**: Aman
- **Deliverable**: `aman/docs/technical_abstract.md`
- **What**: One page. What we're building, why it's different. No jargon without definition.
- **AC**: Reviewed and signed off by Salman + Anas before end of Week 1.

---

### M-04 · Competitor research & outreach targets
- **Owner**: Aman
- **Deliverable**: `aman/docs/competitors.md`
- **What**: Zipline, Wing, Skypuzzler, Joby, Shield AI. Product focus, tech approach, named contact per company. Outreach email template.
- **AC**: 5 companies, at least one named contact each.

---

## WEEK 2 — Safety Shield + Policy Training (Days 8–14)
**Gate**: CBF-QP solver built. Shielded env integrated. Policy training underway with W&B logs.

---

### S-05 · CBF-QP solver
- **Owner**: Salman
- **Deliverable**: `salman/cbf/qp_solver.py`
- **What**: CVXPY QP — minimize deviation from proposed actions subject to all pairwise CBF constraints. Public API: `solve_cbf_qp(states, proposed_actions) → list[SafeAction]`.
- **AC**: Given 10 drones with near-collision pairs, returns safe actions. Runtime < 50ms for N=10. Unit tests pass.
- **Needs**: S-01.

---

### S-06 · High-Order CBF (HOCBF)
- **Owner**: Salman
- **Deliverable**: `salman/cbf/hocbf.py`
- **What**: Position separation is relative degree 2 under acceleration input. Implement HOCBF with auxiliary variable `ψ₁`. Extend S-05's QP.
- **AC**: Test: two drones approaching at 10 m/s, 2m separation — solver applies braking before collision.
- **Needs**: S-05.

---

### S-07 · Shield intervention logger
- **Owner**: Salman
- **Deliverable**: `salman/cbf/shield_logger.py`
- **What**: Logs every shield intervention to CSV. Columns fixed (see `INTERFACES.md § SHIELD-LOG`). Plotter script included.
- **AC**: Logger writes CSV. Plotter renders interventions over time.

---

### S-08 · Train baseline RL policy (no shield)
- **Owner**: Salman
- **Deliverable**: `salman/rl/train_baseline.py` + checkpoint `salman/checkpoints/baseline_v1.pt`
- **What**: PPO/IPPO policy trained for 500k steps. Log to W&B. Expect collisions — this is the unsafe baseline that proves the shield is necessary.
- **AC**: W&B run exists. Checkpoint saved. Episode reward improving.
- **Needs**: A-01 (env), S-04 (training spec agreed with Anas).

---

### A-03 · Shielded environment wrapper
- **Owner**: Anas
- **Deliverable**: `anas/env/shielded_env.py`
- **What**: Wrapper around `drone_env.py`. After policy outputs actions, passes them through `solve_cbf_qp()` before physics. Policy doesn't know it was overridden. Also calls `RerunLogger.log_step()` hook.
- **AC**: 10-drone episode with random policy + shield runs to completion. Logger records interventions. Zero collisions.
- **Needs**: S-05, A-01.

---

### A-04 · Optimize environment for scale
- **Owner**: Anas
- **Deliverable**: `anas/env/drone_env.py` (update) — performance profiling notes in `anas/docs/perf_notes.md`
- **What**: Profile the step loop. Prune distant pairs before passing to the QP. Target: < 200ms/step at N=50 before any training.
- **AC**: Profiler output shows per-step breakdown. N=50 step time measured and documented.

---

### M-05 · Sovereign Core mock API
- **Owner**: Aman
- **Deliverable**: `aman/api/sovereign_core.py`
- **What**: FastAPI on port 8000. `GET /airspace/rules`, `GET /airspace/notam`, `POST /drone/register`. Dockerfile included.
- **AC**: `curl localhost:8000/airspace/rules` returns valid JSON. Starts with `uvicorn`.

---

### M-06 · Simulation client for the API
- **Owner**: Aman
- **Deliverable**: `aman/api/sim_client.py`
- **What**: Python client the simulation calls at episode start to fetch airspace rules. Feeds Salman's Rulebook (S-02).
- **AC**: Changing a rule in the API changes sim behavior within one episode.
- **Needs**: M-05, S-02.

---

### M-07 · White Paper draft (Sections 1–3)
- **Owner**: Aman
- **Deliverable**: `aman/docs/white_paper_draft.md`
- **What**: Introduction, Problem Statement, Architecture Overview. Pull CBF math from S-03 notebook.
- **AC**: 4–5 pages. Salman reviews math.

---

## WEEK 3 — Stress Test & Shielded Training (Days 15–21)
**Gate**: 50-drone simulation under chaos. Shielded policy trained. Data exported. Charts generated.

---

### S-09 · Train shielded RL policy
- **Owner**: Salman
- **Deliverable**: `salman/rl/train_shielded.py` + checkpoint `shielded_v1.pt`
- **What**: Re-train with shield in the loop. Add penalty for triggering the shield (`-0.5 × shield_delta_norm`). Policy learns to be conservative near other drones.
- **AC**: Shield intervention rate lower than baseline. W&B comparison logged. Checkpoint saved.
- **Needs**: S-05, A-03.

---

### S-10 · Latency analysis
- **Owner**: Salman
- **Deliverable**: `salman/analysis/latency_analysis.py`
- **What**: Analytically bound how a 2-second comm delay degrades CBF guarantees. At 15 m/s + 2s delay, compute required `d_safe` expansion. Output goes into the white paper.
- **AC**: Script prints: "At 2s delay and 15 m/s, expand d_safe by X meters."

---

### S-11 · HJ reachability fallback trigger
- **Owner**: Salman
- **Deliverable**: `salman/cbf/fallback_trigger.py`
- **What**: If `h(x) < threshold` AND QP is infeasible → hover in place, log as critical intervention.
- **AC**: Test with 3 converging drones — fallback fires, no crash, logged.

---

### A-05 · Scale to 50 drones + chaos injection
- **Owner**: Anas
- **Deliverable**: `anas/chaos/chaos_injector.py` + `anas/training/train_scale50.py`
- **What**: Three chaos injectors — CommDelay, ByzantineDrone, WindGust — all togglable mid-episode. Scale the shielded env to N=50. Target: < 200ms/step.
- **AC**: Each injector toggles. 50-drone episode runs 1000 steps without timeout.
- **Needs**: A-03.

---

### A-06 · Stress test runner & metrics export
- **Owner**: Anas
- **Deliverable**: `anas/evaluation/run_stress_test.py` + `results/stress_test_50drones.csv`
- **What**: 100 episodes, N=50, all three chaos injectors active. Collect: throughput, collisions, shield interventions, fallbacks, min separation, step time.
- **AC**: CSV saved with fixed column schema (see `INTERFACES.md § SHIELD-LOG`). Summary printed.
- **Needs**: A-05, S-09 checkpoint.

---

### M-08 · Data analysis & charts
- **Owner**: Aman
- **Deliverable**: `aman/analysis/plot_results.py` → 4 PNGs in `results/figures/`
- **What**: (1) Throughput vs drone density. (2) Shield interventions over time. (3) Min separation CDF. (4) Fallback rate vs latency level.
- **AC**: 4 publication-quality charts saved.
- **Needs**: A-06 CSV.

---

### M-09 · Pitch deck outline
- **Owner**: Aman
- **Deliverable**: `aman/docs/pitch_deck_outline.md`
- **What**: 10-slide outline for Gamma.app — Problem, Solution, Architecture, Demo Results, Team, Market, GTM, Competition, Ask.
- **AC**: Outline complete, charts from M-08 referenced.

---

### UI-01 · Simulation visualizer
- **Owner**: Aman (builds it) + Anas (hooks sim to it)
- **Deliverable**: `aman/ui/visualizer.py` + `aman/ui/demo_app.py`
- **What**: Rerun.io for live 3D drone positions + Gradio for the control panel. Drones turn red when shield fires. Live metrics. See `VISUALIZATION.md` for full spec.
- **AC**: 10-drone sim streams to browser. Drone color changes on shield activation. Slider controls N.
- **Needs**: A-03 (shielded env + `log_step` hook).

---

## WEEK 4 — Package & Outreach (Days 22–30)
**Gate**: White paper final. Demo video recorded. Repo clean. Outreach sent.

---

### S-12 · Technical blog post (CBF + RL)
- **Owner**: Salman
- **Deliverable**: `salman/docs/blog_cbf_rl.md`
- **What**: ~1500 words. How combining RL with a CBF shield works, the feasibility problem, HOCBF solution, how shielded training reduces intervention rate. Target: Medium or Substack.
- **AC**: Reviewed by Anas. Ready to publish.

---

### S-13 · White paper math review
- **Owner**: Salman
- **Task**: Review all math in Aman's `white_paper_final.md`. Verify equations match implementation. Add formal theorem on delay margin.
- **AC**: PR comment approval on Aman's white paper PR.

---

### A-07 · ONNX export & edge benchmark
- **Owner**: Anas
- **Deliverable**: `anas/edge/export_onnx.py` + `results/edge_benchmark.md`
- **What**: Export shielded policy to ONNX. Benchmark: CPU, GPU, simulated Jetson Orin (thread-limited). Target: < 10ms/step.
- **AC**: Benchmark table saved. ONNX model at `anas/checkpoints/shielded_v1.onnx`.
- **Needs**: S-09 checkpoint.

---

### A-08 · Technical blog post (edge ML)
- **Owner**: Anas
- **Deliverable**: `anas/docs/blog_edge_ml.md`
- **What**: ~1500 words. MARL architecture decisions for edge, ONNX export, quantization, latency vs safety tradeoff.
- **AC**: Reviewed by Salman. Ready to publish.

---

### M-10 · Finalize White Paper (10–15 pages)
- **Owner**: Aman
- **Deliverable**: `aman/docs/white_paper_final.md` → PDF via Pandoc
- **What**: Full paper: Abstract, Intro, Architecture, CBF+RL, Simulation, Evaluation, Edge Deployment, Conclusion, References (32 from PDF).
- **AC**: Salman approves math (S-13). All charts included. PDF generated.

---

### M-11 · Demo video
- **Owner**: Aman
- **Deliverable**: `aman/docs/demo_script.md` + recorded video
- **What**: 3-minute recording from the Rerun/Gradio viewer. Normal flight → chaos injected → shield fires (drones go red) → zero collisions. Record with OBS.
- **AC**: Video recorded. Link in `demo_script.md`.
- **Needs**: UI-01, A-06.

---

### M-12 · Outreach emails
- **Owner**: Aman
- **Deliverable**: `aman/docs/outreach_log.md`
- **What**: Personalized emails to 5 contacts from M-04. Attach white paper.
- **AC**: 5 sent, log updated.
- **Needs**: M-10.

---

### M-13 · Repo cleanup
- **Owner**: Aman
- **Task**: Every directory has a README. Top-level README updated with demo GIF, setup instructions, links to paper + blog posts. `CONTRIBUTING.md` added. No secrets committed.
- **AC**: Repo looks professional on GitHub.

---

## Dependency map

```
S-01 ──────────────────────► S-05 ──► S-06 ──► S-10
                               │
S-02 ──► M-06                  │
                               ▼
A-01 ──► A-02 ──────────────► A-03 ──────────────► A-05 ──► A-06
          │                    │                              │
          └──► S-08 (baseline) │                              ▼
                               └──► S-09 (shielded) ──►  M-08 ──► M-09
                                         │
                              A-07 (ONNX) ◄── needs S-09 checkpoint
                                         │
M-01 ──► M-02 ──► M-05 ──► M-06         │
                                         ▼
                                   UI-01 ──► M-11
                                              │
                              S-13 + A-07 + M-08
                                              │
                                            M-10 ──► M-12
```

---

## Weekly sync (30 min, every Sunday)

1. Each person demos merged work — live, not slides.
2. Blocked ticket? Pair program or reassign.
3. Walk the dependency map — interfaces honored?
4. Agree on next week's priority order.

**Rule**: Stuck > 4 hours → open a GitHub Discussion, not a DM.
