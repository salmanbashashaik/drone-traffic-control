# Project Task Board — Drone Traffic Control PoC
> 30-day execution plan. Each ticket is atomic: one person, one deliverable, one acceptance criterion.
> Branch model: `main` ← `dev` ← `salman/cbf-safety` | `anas/marl-policy` | `aman/infra-api`
> PRs go into `dev`. `dev` → `main` only on weekly milestones.

---

## Legend
- **Status**: `[ ]` open · `[~]` in progress · `[x]` done · `[!]` blocked
- **Owner**: S = Salman · A = Anas · M = Aman
- **Branch**: work lives on your branch; merge to `dev` when the AC is met

---

## WEEK 1 — Foundation & Baseline (Days 1–7)
**Milestone gate**: Simulation runs with 10 drones. CBF equations defined on paper. Infra server is reachable.

---

### S-01 · Define pairwise CBF safety constraint
- **Owner**: Salman | **Branch**: `salman/cbf-safety`
- **File**: `salman/math/cbf_pairwise.py`
- **Task**: Write the barrier function `h_ij(x) = ||p_i - p_j||² - d_safe²`. Implement `h`, `∇h`, and confirm `h(x) ≥ 0` is the safe set.
- **AC**: A standalone Python script runs, takes two 3D position vectors, and returns `h_ij`, `Lf_h`, `Lg_h`. No CVXPY yet — pure NumPy.
- **Inputs needed**: None (pure math).
- **Output contract**: See `INTERFACES.md § CBF-OUTPUT`.

---

### S-02 · Define Lexicographic Rulebook
- **Owner**: Salman | **Branch**: `salman/cbf-safety`
- **File**: `salman/math/rulebook.py`
- **Task**: Encode the three-level priority: `Safety > Geofence > Mission Progress`. Each rule returns a scalar penalty/cost and a feasibility flag.
- **AC**: `rulebook.evaluate(state, action)` returns a `RulebookResult(safety_ok, geofence_ok, progress_cost)` dataclass. Unit test with 3 hand-crafted scenarios.
- **Inputs needed**: Agreed state schema from `INTERFACES.md § STATE-SCHEMA`.

---

### S-03 · Document math in LaTeX notebook
- **Owner**: Salman | **Branch**: `salman/cbf-safety`
- **File**: `salman/docs/cbf_derivation.ipynb`
- **Task**: Jupyter notebook walking through the full CBF-QP derivation (control-affine model, Lie derivatives, the QP formulation). This feeds directly into the white paper.
- **AC**: Notebook renders without errors. All equations match the PDF report's formulation.

---

### A-01 · Stand up PettingZoo MARL environment (10 drones)
- **Owner**: Anas | **Branch**: `anas/marl-policy`
- **File**: `anas/env/drone_env.py`
- **Task**: Custom PettingZoo `ParallelEnv` for N drones in a 3D box (1km × 1km × 200m). State = position + velocity + goal. Action = 3D acceleration command. Simple reward: `+1` for reaching goal, `-100` for collision.
- **AC**: `env.reset()` and `env.step(actions)` work for N=10. Random policy runs 100 episodes without crashing Python.
- **Inputs needed**: `STATE-SCHEMA` from `INTERFACES.md`.
- **Output contract**: See `INTERFACES.md § ENV-OUTPUT`.

---

### A-02 · Train baseline MARL policy (no shield)
- **Owner**: Anas | **Branch**: `anas/marl-policy`
- **File**: `anas/training/train_baseline.py`
- **Task**: Train a PPO or IPPO policy with RLlib or TorchRL for 500k steps. Log to Weights & Biases. Save checkpoint. Expect collisions — the goal is to have a policy that tries to reach goals but has no safety.
- **AC**: W&B run exists. Checkpoint saved at `anas/checkpoints/baseline_v1.pt`. Episode reward > -50 (it's learning something).
- **Inputs needed**: `A-01` complete.

---

### A-03 · Define action output format for CBF handoff
- **Owner**: Anas | **Branch**: `anas/marl-policy`
- **File**: `anas/env/action_schema.py`
- **Task**: Define `MARLAction` dataclass: `agent_id`, `proposed_accel: np.ndarray (3,)`, `timestamp_ms: int`. This is the exact struct Salman's CBF-QP will receive.
- **AC**: Dataclass defined, serializable to JSON, and documented in `INTERFACES.md § MARL-ACTION`.

---

### M-01 · Provision cloud simulation server
- **Owner**: Aman | **Branch**: `aman/infra-api`
- **File**: `aman/infra/setup.sh`
- **Task**: Spin up an AWS EC2 `c5.2xlarge` (or GCP equivalent). Install: Python 3.11, Docker, Git, CUDA drivers (optional). Clone the repo. Confirm all teammates can SSH in.
- **AC**: `ssh ubuntu@<IP> "python3 --version && git --version"` succeeds. IP and key path documented in `aman/infra/server.md` (no secrets in git — use `.gitignore`).

---

### M-02 · Install simulation stack (BlueSky + Flightmare)
- **Owner**: Aman | **Branch**: `aman/infra-api`
- **File**: `aman/infra/install_sim.sh`
- **Task**: Install BlueSky ATM simulator and Flightmare physics engine on the server. Verify both launch without errors. Document known version conflicts.
- **AC**: `python -c "import bluesky"` and `python -c "import flightmare"` both succeed on the server.

---

### M-03 · Draft 1-page Technical Abstract
- **Owner**: Aman | **Branch**: `aman/infra-api`
- **File**: `aman/docs/technical_abstract.md`
- **Task**: One page. What we are building, why it matters, what makes it different (CBF shield on top of MARL). No jargon without definition.
- **AC**: Reviewed by Salman (math correctness) and Anas (ML correctness) before Week 1 ends.

---

### M-04 · Competitor research & outreach targets
- **Owner**: Aman | **Branch**: `aman/infra-api`
- **File**: `aman/docs/competitors.md`
- **Task**: Research Zipline, Wing, Skypuzzler, Joby, Shield AI. For each: product focus, tech stack (public info), head of autonomy/engineering (LinkedIn). Template outreach email drafted.
- **AC**: 5 companies documented with at least one named contact each.

---

## WEEK 2 — Safety Shield & Tactical Layer (Days 8–14)
**Milestone gate**: CBF-QP solver runs in the simulation loop. Shielded policy has zero collisions on 10-drone test.

---

### S-04 · Implement CBF-QP solver (CVXPY)
- **Owner**: Salman | **Branch**: `salman/cbf-safety`
- **File**: `salman/cbf/qp_solver.py`
- **Task**: Implement `solve_cbf_qp(state, proposed_actions) → safe_actions`. Uses CVXPY to solve the QP: minimize deviation from proposed actions subject to all pairwise CBF constraints.
- **AC**: Given 10 drones with 3 pairs in near-collision, solver returns safe actions. Runtime < 50ms for N=10. Unit tests pass.
- **Inputs needed**: `S-01` output, `A-03` action schema.

---

### S-05 · High-Order CBF (HOCBF) for acceleration control
- **Owner**: Salman | **Branch**: `salman/cbf-safety`
- **File**: `salman/cbf/hocbf.py`
- **Task**: Position separation with acceleration input is relative degree 2. Implement the HOCBF construction with an auxiliary variable `ψ₁`. Extend the QP from S-04.
- **AC**: HOCBF constraint added to QP. Test: drone approaching at 10 m/s with 2m separation — solver applies braking.
- **Inputs needed**: `S-04` complete.

---

### S-06 · Shield intervention logging
- **Owner**: Salman | **Branch**: `salman/cbf-safety`
- **File**: `salman/cbf/shield_logger.py`
- **Task**: Log every time the shield modifies an action. Fields: `timestep`, `agent_id`, `proposed_accel`, `safe_accel`, `h_value`, `constraint_active`.
- **AC**: Logger writes to CSV. Visualizer script plots shield interventions over time. This data feeds Week 3 stress test.

---

### A-04 · Integrate CBF shield into simulation loop
- **Owner**: Anas | **Branch**: `anas/marl-policy`
- **File**: `anas/env/shielded_env.py`
- **Task**: Wrap `drone_env.py`. After MARL outputs actions, pass them through `S-04`'s `solve_cbf_qp`. Return shielded actions to the physics engine. MARL never knows it was overridden — transparent shield.
- **AC**: 10-drone episode with baseline policy + shield runs to completion. `shield_logger` records interventions. Zero collisions.
- **Inputs needed**: `S-04` and `A-02`.

---

### A-05 · Re-train MARL policy with shield awareness (reward shaping)
- **Owner**: Anas | **Branch**: `anas/marl-policy`
- **File**: `anas/training/train_shielded.py`
- **Task**: Add a small penalty for triggering the shield (`-0.5 * shield_delta_norm`). Re-train for 500k steps. The policy should learn to be more conservative near other drones.
- **AC**: Shield intervention rate drops vs baseline. W&B comparison logged. Checkpoint saved as `shielded_v1.pt`.
- **Inputs needed**: `A-04`.

---

### M-05 · Build Sovereign Core mock API (FastAPI)
- **Owner**: Aman | **Branch**: `aman/infra-api`
- **File**: `aman/api/sovereign_core.py`
- **Task**: FastAPI server. Endpoints: `GET /airspace/rules` returns geofence JSON, `GET /airspace/notam` returns active NOTAMs, `POST /drone/register` registers a drone. Runs on port 8000.
- **AC**: `curl localhost:8000/airspace/rules` returns valid JSON. Server starts with `uvicorn`. Dockerfile included.
- **Output contract**: See `INTERFACES.md § API-AIRSPACE`.

---

### M-06 · Connect simulation to Sovereign Core API
- **Owner**: Aman | **Branch**: `aman/infra-api`
- **File**: `aman/api/sim_client.py`
- **Task**: Python client that the simulation calls at episode start to fetch current airspace rules. Rules feed into Salman's rulebook (`S-02`).
- **AC**: Simulation episode fetches rules from API. Changing a rule in the API changes simulation behavior within one episode.
- **Inputs needed**: `M-05`, `S-02`.

---

### M-07 · Begin White Paper draft
- **Owner**: Aman | **Branch**: `aman/infra-api`
- **File**: `aman/docs/white_paper_draft.md`
- **Task**: Sections 1–3 drafted: Introduction, Problem Statement, Architecture Overview. Pull CBF math from Salman's notebook (`S-03`). Include the architecture diagram from the README.
- **AC**: 4–5 pages written. Salman reviews math sections.

---

## WEEK 3 — Falsification Stress Test (Days 15–21)
**Milestone gate**: 50-drone simulation with chaos injection. Data exported. Safety holds.

---

### S-07 · Feasibility analysis under communication delay
- **Owner**: Salman | **Branch**: `salman/cbf-safety`
- **File**: `salman/analysis/latency_analysis.py`
- **Task**: Analytically bound how stale state information degrades CBF guarantees. For a 2-second delay and max speed of 15 m/s, compute the required safety margin expansion (`d_safe` inflation).
- **AC**: Script outputs: "At 2s delay and 15 m/s, expand d_safe by X meters to maintain guarantee." This goes directly into the white paper.

---

### S-08 · HJ Reachability fallback trigger
- **Owner**: Salman | **Branch**: `salman/cbf-safety`
- **File**: `salman/cbf/fallback_trigger.py`
- **Task**: Implement a simplified reachability check: if `h(x) < h_threshold` AND the QP is infeasible, trigger the fallback controller (hover in place). Log as a "critical intervention."
- **AC**: Fallback triggers in a test where 3 drones converge and the QP becomes infeasible. No crash.

---

### A-06 · Scale to 50 drones
- **Owner**: Anas | **Branch**: `anas/marl-policy`
- **File**: `anas/training/train_scale50.py`
- **Task**: Scale the shielded environment to N=50. Optimize the QP solver call (batch pairwise constraints, prune distant pairs). Target: full step loop < 200ms.
- **AC**: 50-drone episode runs 1000 steps without timeout. Profile output shows per-step timing.
- **Inputs needed**: `A-05`, `S-05`.

---

### A-07 · Chaos injection module
- **Owner**: Anas | **Branch**: `anas/marl-policy`
- **File**: `anas/chaos/chaos_injector.py`
- **Task**: Three injectors: (1) `CommDelay(agent_id, delay_ms)` — stalens the state fed to that agent's policy. (2) `ByzantineDrone(agent_id)` — broadcasts random false telemetry. (3) `WindGust(magnitude, duration)` — adds noise to physics.
- **AC**: Each injector is a wrapper that can be toggled on/off mid-episode. Test confirms MARL policy degrades under chaos; CBF shield maintains safety.

---

### A-08 · Collect and export stress test metrics
- **Owner**: Anas | **Branch**: `anas/marl-policy`
- **File**: `anas/evaluation/run_stress_test.py`
- **Task**: Run 100 episodes at N=50 with all three chaos injectors active. Collect: throughput (% drones reaching goal), collisions, shield interventions, fallback triggers, min separation distance.
- **AC**: Results CSV saved at `results/stress_test_50drones.csv`. Summary printed: "Throughput: X%. Collisions: Y. Shield interventions: Z."

---

### M-08 · Data analysis & visualization
- **Owner**: Aman | **Branch**: `aman/infra-api`
- **File**: `aman/analysis/plot_results.py`
- **Task**: Using matplotlib/seaborn, generate 4 charts from `A-08` output: (1) Throughput vs drone density. (2) Shield interventions over time. (3) Min separation CDF. (4) Fallback rate vs latency level.
- **AC**: 4 publication-quality PNG charts saved to `results/figures/`.
- **Inputs needed**: `A-08` CSV.

---

### M-09 · Build Seed Pitch Deck (10 slides)
- **Owner**: Aman | **Branch**: `aman/infra-api`
- **File**: `aman/docs/pitch_deck_outline.md`
- **Task**: Outline 10 slides for Gamma.app generation: Problem, Solution, Architecture, Demo Results (use charts from M-08), Team, Market Size, Go-to-Market, Competition, Ask. Draft key bullet points for each slide.
- **AC**: Outline complete. Charts from `M-08` referenced. Ready to drop into Gamma.app for visual generation.

---

## WEEK 4 — Package & Outreach (Days 22–30)
**Milestone gate**: White paper final, demo video recorded, repo clean, outreach sent.

---

### S-09 · Technical blog post (CBF shield feasibility)
- **Owner**: Salman | **Branch**: `salman/cbf-safety`
- **File**: `salman/docs/blog_cbf_shield.md`
- **Task**: ~1500 words. Explain the shield feasibility problem (what happens when no safe action exists?) and how HOCBF + fallback solves it. Include code snippets from `qp_solver.py`. Target: Medium or Substack.
- **AC**: Reviewed by Anas for accuracy. Published or ready to publish.

---

### S-10 · Final math review pass for White Paper
- **Owner**: Salman | **Branch**: `salman/cbf-safety`
- **Task**: Review all math in Aman's white paper draft. Verify equations match implementation. Add formal theorem statement for "CBF shield maintains safety under bounded delay δt if d_safe is expanded by v_max · δt."
- **AC**: White paper math section approved by Salman (comment in PR).

---

### A-09 · ONNX export and edge inference benchmark
- **Owner**: Anas | **Branch**: `anas/marl-policy`
- **File**: `anas/edge/export_onnx.py`
- **Task**: Export the shielded MARL policy to ONNX. Benchmark inference latency on CPU (laptop), GPU (server), and simulate Jetson Orin (use ONNX Runtime with thread limits). Target: < 10ms per step.
- **AC**: ONNX model saved at `anas/checkpoints/shielded_v1.onnx`. Benchmark table printed and saved to `results/edge_benchmark.md`.

---

### A-10 · Technical blog post (MARL edge optimization)
- **Owner**: Anas | **Branch**: `anas/marl-policy`
- **File**: `anas/docs/blog_edge_marl.md`
- **Task**: ~1500 words. How the MARL policy was optimized for edge deployment: policy network architecture decisions, ONNX export, quantization options, latency vs safety tradeoff. Target: Medium or Substack.
- **AC**: Reviewed by Salman.

---

### M-10 · Finalize White Paper (10–15 pages)
- **Owner**: Aman | **Branch**: `aman/infra-api`
- **File**: `aman/docs/white_paper_final.md`
- **Task**: Incorporate: math from `S-10`, edge specs from `A-09`, charts from `M-08`. Sections: Abstract, Intro, Architecture, CBF Shield, MARL Policy, Evaluation, Conclusion, References (use the 32 from the PDF). Target journal style: IEEE or arXiv preprint format.
- **AC**: All sections complete. Peer-reviewed internally. PDF generated via Pandoc.

---

### M-11 · Demo video script & recording
- **Owner**: Aman | **Branch**: `aman/infra-api`
- **File**: `aman/docs/demo_script.md`
- **Task**: 3-minute demo. Script: (1) show 50 drones flying normally, (2) inject Byzantine drone — show MARL starting to fail, (3) CBF shield lights up (drones change color on intervention), (4) show metrics: zero collisions. Record using OBS or screen capture.
- **AC**: Video recorded and saved (link in `aman/docs/demo_script.md`). Drones visually change color on shield intervention.
- **Inputs needed**: `A-08` simulation, color-coding hook from Anas.

---

### M-12 · Outreach emails sent
- **Owner**: Aman | **Branch**: `aman/infra-api`
- **File**: `aman/docs/outreach_log.md`
- **Task**: Send personalized emails to the 5 contacts from `M-04`. Subject: "Proving safe autonomous drone deconfliction under C2 latency." Attach white paper PDF. Log date sent and any replies in `outreach_log.md`.
- **AC**: 5 emails sent. Log updated.
- **Inputs needed**: `M-10` (white paper), `M-04` (contacts).

---

### M-13 · Clean up GitHub repo
- **Owner**: Aman | **Branch**: `dev` (merge all branches first)
- **Task**: Every directory has a `README.md`. Top-level `README.md` updated with: demo GIF, architecture diagram, setup instructions, links to white paper and blog posts. Add `CONTRIBUTING.md`. Add `.github/PULL_REQUEST_TEMPLATE.md`.
- **AC**: Repo looks professional on GitHub. No dead files, no passwords/keys committed, `.gitignore` covers `*.pt`, `*.onnx`, `*.csv` (large files).

---

## Dependency Graph

```
S-01 → S-04 → S-05 → S-07
              ↓
S-02 → M-06   A-04 → A-05 → A-06 → A-07 → A-08
              ↑              ↓
A-01 → A-02 → A-03          M-08 → M-09
                             ↓
M-01 → M-02 → M-05 → M-06   M-10 (S-10 + A-09 + M-08)
```

---

## Weekly Sync Agenda (30 min, every Sunday)

1. Each person demos their merged work (live, not slides).
2. Any blocked tickets — reassign or pair program.
3. Review the dependency graph — are interfaces honored?
4. Agree on next week's priority order.

**Rule**: If you are stuck for > 4 hours, you open a GitHub Discussion, not a DM. That way the team learns from the blocker.
