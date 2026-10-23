# P3-SA-01 — Literature: Consistent Backup Sets for Multi-Drone Safety Filters

| Field | Value |
|---|---|
| Task | Research lane, backlog BL-03 to BL-07 |
| Status | Draft for review by Salman. Nothing here changes `src/utm_services/safety/`. |
| Date | 2026-10-23 |
| Builds on | [`P2-SA-02-lateral-escape.md`](./P2-SA-02-lateral-escape.md) §4–§5, `docs/results/pr12-final-report.md`, `docs/results/pr12-b1-review-2026-10-23.md` |
| Followed by | [`P3-SA-02-consistent-backup.md`](./P3-SA-02-consistent-backup.md) |

## 0. Why this note exists

The current filter is a centralised QP with a backup-set barrier (P2-SA-02 §4.3). Its backup is built pair by pair. For each pair, `_backup_rollout` flies both drones along an evasive direction chosen for that pair, with relative-position feedback. A drone that belongs to two pairs therefore gets two backups, and they can point in opposite directions. PR #12 added latches, a 2-colouring and an escape stage to reduce the damage. The review of PR #12 showed that these patches fail in crowds:

- the bent and staged sandwiches give the squeezed drone contradictory sideways directions (HIGH-1, BL-10);
- the escape axis flips where the line of sight crosses x = ±y (MEDIUM-1, BL-11);
- recoverable hard-fuzz seeds with 5 and 6 drones (1177 at 4.765 m, 4828 at 5.369 m) still enter `d_eff` (BL-08).

The root cause is recorded as BL-03: the pairwise backups credit a drone with escapes that it cannot fly at the same time. This note reviews the literature that addresses that problem, and the related backlog items BL-04 to BL-07, so that the fix rests on published results instead of new patches.

## 1. How the sources were checked

Every source below was opened at its primary location (arXiv, DOI landing page or publisher). The "Read" line of each entry says how much of it was read:

- **Full text**: every page, including proofs and experiments.
- **Main text**: the body of the paper. Appendices or late sections may have been skimmed.
- **Abstract only**: only the abstract and the bibliographic data were checked. For these sources this note claims nothing beyond what the abstract says. They must be read in full before any design step relies on them.

The notation is mapped to ours: `x_i` is the state of drone `i` (position and velocity), `u` is the per-axis acceleration bound (3.5 m/s²), `d_eff = 5.5 m`, `h_ij` is a pair barrier, `π_i` is a backup controller, `Φ^t_π(x)` is the flow of the closed loop under `π` for time `t`, and `S_0` is the set into which the backup ends (for us, hover).

## 2. Sources

### 2.1 Chen, Singletary, Ames 2021 — guaranteed obstacle avoidance for multi-robot operations with limited actuation (backup CBF, multi-agent)

- Y. Chen, A. Singletary, A. D. Ames, "Guaranteed Obstacle Avoidance for Multi-Robot Operations With Limited Actuation: A Control Barrier Function Approach," *IEEE Control Systems Letters*, 5(1):127–132, January 2021. DOI [10.1109/LCSYS.2020.3000748](https://doi.org/10.1109/LCSYS.2020.3000748).
- Read: full text.

**What it guarantees.** Each agent has a backup controller `π_i(x_i)` that depends only on its own state and is known to every agent. The backup drives the agent into `S_0`, a set of equilibria inside the safe set (for quadrotors, hover). The pairwise safety constraint is evaluated along the two agents' own backup trajectories `Φ^t_{π_i}(x_i)` and `Φ^t_{π_j}(x_j)`, at a finite set of sample times on the horizon. The paper gives a centralised QP (its eq. (9)) and a decentralised QP (its eq. (10)) in which each agent takes half of the `α(h_ij)` term of every pair constraint. Theorem 1 states that the decentralised QP is feasible whenever the backup set constraints hold, and that the joint solution of the decentralised problems is feasible for the centralised one. The reason is simple: every agent flying its own backup action satisfies every constraint at once, because all pair constraints were computed from the same per-agent trajectories.

**Assumptions.**
- Assumption 1: the safety constraints decompose into pairwise terms.
- The backup is fixed and known to all agents a priori, and it depends only on the agent's own state.
- `S_0` consists of equilibria, so the pair distance stays constant after the backup horizon.
- Continuous-time constraints are replaced by a finite set of sample times. The paper refers to a robust-sampling result (its ref. [28]) for the guarantee between samples.

**How it handles 3+ agents.** With one backup per agent there is no conflict: three or more agents simply give more pair rows, all computed from the same trajectories. Section IV treats agents with several candidate backups. It shows that choices made pair by pair can be incompatible. Its Fig. 1 has three agents where agent 1 cannot tell that its choice is invalid without information about agents 2 and 3. This is exactly our BL-03. The paper's remedy is a broadcast scheme (Algorithm 1): each agent computes the set of its candidates compatible with the candidates already broadcast by its neighbours, solves the QP for each compatible candidate, keeps the one with the smallest intervention, and broadcasts it.

**Experiments.** Three and six robots on the Robotarium; a 17-state quadrotor simulation with a stop-and-level backup; QPs solved with OSQP.

**Mapping.**
- BL-03: direct fit. One backup per drone, shared by all its rows, with a compatible selection when there are several candidates.
- BL-04: the constraint is imposed at every sample time on the horizon, not only at the closest-approach step. This is the reference for a window around `k*`.
- BL-05: not covered beyond the sampling remark.
- BL-06, BL-07: not covered; the paper assumes exact models.

### 2.2 Singletary, Swann, Jimenez Rodriguez, Ames 2022 — safe drone flight with time-varying backup controllers

- A. Singletary, A. Swann, I. F. Jimenez Rodriguez, A. D. Ames, "Safe Drone Flight with Time-Varying Backup Controllers," *IEEE/RSJ IROS 2022*, pp. 4577–4584. DOI [10.1109/IROS47612.2022.9981741](https://doi.org/10.1109/IROS47612.2022.9981741), arXiv [2207.05220](https://arxiv.org/abs/2207.05220).
- Read: main text (pp. 1–7).

**What it guarantees.** A time-varying backup controller (TBC, its eq. (9)) flies a manoeuvre `u_M` for `τ ≤ T_M`, blends to the backup `u_B` over a time `δ`, and then flies `u_B`. A time offset `τ0` records where in the manoeuvre the agent is (eqs. (14)–(16)). Theorem 1 states that the safe set certified by the TBC contains the one certified by `u_B` alone, so a TBC is never more conservative than its backup. Algorithm 1 resets `τ0` only when the whole manoeuvre, flown from now, is certified safe; otherwise the clock keeps running. Algorithm 2 lets the agent switch to another manoeuvre only once it is in the backup phase (`t − τ0 ≥ T_M`), so the commanded backup changes at most once every `T_M`. Theorem 2 gives decentralised safety when every agent knows the other agents' states, and the paper states that if several agents have several manoeuvres, the current manoeuvre must be known by all agents.

**Assumptions.** Exact dynamics and full state knowledge of the other agents. The backup `u_B` is a velocity controller that stops the drone, with `h_B = −sqrt(v²) + 0.1`. The manoeuvres in the paper are "carry on" and "evade" (upwards).

**How it handles 3+ agents.** The theory is written for many agents; the hardware experiment uses two quadrotors.

**Mapping.**
- BL-03: gives the form of each per-drone candidate (evade, then stop) and the rule that every agent must use the same, known current manoeuvre.
- Reversals under noise (BL-06 strict xfail at σ 1.5): Algorithms 1 and 2 are a principled hysteresis. A drone keeps its manoeuvre clock and may only change manoeuvre in the backup phase. This replaces ad hoc latches.
- BL-04: the TBC is certified over its whole horizon.
- BL-05 to BL-07: not covered.

### 2.3 Chen, Jankovic, Santillo, Ames 2021 — backup control barrier functions

- Y. Chen, M. Jankovic, M. Santillo, A. D. Ames, "Backup Control Barrier Functions: Formulation and Comparative Study," *IEEE CDC 2021*, pp. 6835–6841. DOI [10.1109/CDC45484.2021.9683111](https://doi.org/10.1109/CDC45484.2021.9683111), arXiv [2104.11332](https://arxiv.org/abs/2104.11332).
- Read: abstract only.

**What the abstract states.** Under mild assumptions the backup CBF has relative degree 1. With a good backup controller it gives an invariant set close to the maximal one. QP feasibility is guaranteed.

**Mapping.** Supports the use of a rollout-based barrier with first-order rows (which our filter already uses) and the claim that the backup QP is feasible by construction. Feeds BL-03 and BL-04 only after a full read.

### 2.4 Jankovic, Santillo 2021 — collision avoidance and liveness of multi-agent systems with CBF-based controllers

- M. Jankovic, M. Santillo, "Collision Avoidance and Liveness of Multi-agent Systems with CBF-based Controllers," *IEEE CDC 2021*, pp. 6822–6828. DOI [10.1109/CDC45484.2021.9682854](https://doi.org/10.1109/CDC45484.2021.9682854), arXiv [2012.10261](https://arxiv.org/abs/2012.10261).
- Read: main text (pp. 1–6).

**What it guarantees.** The paper compares several multi-agent CBF policies: Centralized QP (its eq. (14)), Decentralized Follower (15), Decentralized Reciprocal (16, each agent takes half of each pair term), and two coordinated variants, CCS2 (17) and PCCA (18, 19). Proposition 1 states that the Centralized, CCS2 and PCCA policies are always feasible in the safe set `C*`. The proof builds a braking solution by induction, `u_i − u_j = −λ1 v_ij`.

**Counter-example for 3+ agents.** Fig. 2 shows the Decentralized Reciprocal policy failing with three agents. The middle agent is stationary; its own braking has no effect, its QP becomes infeasible, and the two outer agents collide. This is our sandwich geometry. In the paper's 5-agent Monte Carlo study, the two decentralised policies lacked liveness and were infeasible in about 30 % of the runs, while PCCA matched the centralised policy.

**Robust form.** The paper uses a robust CBF condition `L_f h − ‖L_p h‖ w̄ + L_g h u + α(h) ≥ 0` for a bounded disturbance `‖p‖ ≤ w̄`.

**Mapping.**
- BL-03: shows that splitting pair constraints without coordination fails in exactly the sandwich, and that a centralised or coordinated policy stays feasible. Our filter is centralised, so feasibility is available, but only if the pair rows share one joint backup.
- BL-06: the robust CBF row is the simplest disturbance margin.
- BL-04, BL-05, BL-07: not covered.

### 2.5 van Wijk, Coogan, Molnar, Majji, Hobbs 2024 — disturbance-robust backup control barrier functions

- D. E. J. van Wijk, S. Coogan, T. G. Molnar, M. Majji, K. L. Hobbs, "Disturbance-Robust Backup Control Barrier Functions: Safety Under Uncertain Dynamics," *IEEE Control Systems Letters*, 8:2817–2822, 2024. DOI [10.1109/LCSYS.2024.3514998](https://doi.org/10.1109/LCSYS.2024.3514998), arXiv [2409.07700](https://arxiv.org/abs/2409.07700).
- Read: main text (arXiv HTML).

**What it guarantees.** For a disturbance bounded by `‖d‖ ≤ ξ`, the true trajectory under the backup stays inside a tube around the nominal rollout. The tube radius is `δ_max(τ) = ξ/L_cl · (e^{L_cl τ} − 1)` for a closed loop with Lipschitz constant `L_cl`, or `ξ/k_b · (1 − e^{−k_b τ})` when the backup closed loop is contracting with rate `k_b`. The barrier along the rollout is tightened by the tube, and the QP row gets a robustness term `η = ξ ‖∇h Φ‖`.

**Assumptions and limits.** Single agent. Remark 2 states that QP feasibility is not guaranteed under the robust constraints.

**How it handles 3+ agents.** It does not.

**Mapping.**
- BL-06: the per-step tube margin for wind and model error, applied along the per-drone rollout.
- BL-07: the same tube applied to each world-box wall gives a per-wall wind budget.
- The contracting form needs a backup with velocity feedback (for example a PD brake); a bang-bang brake is not contracting, so the exponential form would apply.

### 2.6 Wang, Ames, Egerstedt 2016/2017 — safety barrier certificates for collision-free multi-robot systems

- L. Wang, A. D. Ames, M. Egerstedt, "Safety Barrier Certificates for Heterogeneous Multi-Robot Systems," *American Control Conference 2016*, pp. 5213–5218. DOI [10.1109/ACC.2016.7526486](https://doi.org/10.1109/ACC.2016.7526486), arXiv [1609.00651](https://arxiv.org/abs/1609.00651). Read: main text (arXiv rendering).
- Journal version: L. Wang, A. D. Ames, M. Egerstedt, "Safety Barrier Certificates for Collisions-Free Multirobot Systems," *IEEE Transactions on Robotics*, 33(3):661–674, 2017. DOI [10.1109/TRO.2017.2659727](https://doi.org/10.1109/TRO.2017.2659727). Read: bibliographic data only.

**What it guarantees.** A braking-distance barrier for double integrators, `h_ij = sqrt(2(α_i + α_j)(‖Δp‖ − D_s)) + Δpᵀ Δv / ‖Δp‖`, with a cubic class-K term `γ h³`. In the decentralised form each robot takes a share of the pair constraint proportional to its braking capability, `α_i / (α_i + α_j)`.

**Limits stated in the paper.** The authors discuss that the QP can become infeasible when many robots crowd together, and that deadlock is not resolved.

**Mapping.** This is the barrier family behind our braking barrier (P0-SA-01). It shows why braking-only barriers are conservative in crowds and why we moved to backup sets. It gives the capability-weighted split as an alternative to the half split for the decentralised case.

### 2.7 Robust and input-to-state-safe CBFs under bounded disturbances

- S. Kolathaya, A. D. Ames, "Input-to-State Safety With Control Barrier Functions," *IEEE Control Systems Letters*, 3(1):108–113, 2019. DOI [10.1109/LCSYS.2018.2853698](https://doi.org/10.1109/LCSYS.2018.2853698), arXiv [1803.03035](https://arxiv.org/abs/1803.03035). Read: abstract only. The abstract states that under a bounded disturbance the state stays in a set near the safe set, whose size depends on the disturbance bound (input-to-state safety, ISSf).
- A. Alan, A. J. Taylor, C. R. He, G. Orosz, A. D. Ames, "Safe Controller Synthesis With Tunable Input-to-State Safe Control Barrier Functions," *IEEE Control Systems Letters*, 6:908–913, 2022. DOI [10.1109/LCSYS.2021.3087443](https://doi.org/10.1109/LCSYS.2021.3087443), arXiv [2103.08041](https://arxiv.org/abs/2103.08041). Read: abstract only. The abstract introduces tunable ISSf CBFs (TISSf), whose robustness term can be reduced where the state is far from the boundary, to lower conservatism.
- M. Jankovic, "Robust Control Barrier Functions for Constrained Stabilization of Nonlinear Systems," *Automatica*, 96:359–367, 2018. DOI [10.1016/j.automatica.2018.07.004](https://doi.org/10.1016/j.automatica.2018.07.004). Read: abstract only. The abstract introduces robust CBFs for systems with disturbances and combines them with control Lyapunov functions in a QP.

**Mapping.** BL-06 and BL-07. These give the theory for a disturbance margin in each row. ISSf trades an exact guarantee for a bounded violation, which matches the BL-06 wording "keep `d_safe`" (5 m) while the filter aims at `d_eff` (5.5 m): the 0.5 m margin can be sized as the ISSf violation bound. None of them covers multiple agents or backup sets, so the tube of §2.5 is the better fit for our rollout rows.

### 2.8 Sampled-data and input delays

- A. Singletary, Y. Chen, A. D. Ames, "Control Barrier Functions for Sampled-Data Systems with Input Delays," *IEEE CDC 2020*, pp. 804–809. DOI [10.1109/CDC42340.2020.9304281](https://doi.org/10.1109/CDC42340.2020.9304281), arXiv [2005.06418](https://arxiv.org/abs/2005.06418).
- Read: abstract only.

**What the abstract states.** The paper gives a method for robust set invariance of sampled-data systems with zero-order hold, state uncertainty and input delay, using constraints that stay affine in the input.

**Mapping.** BL-05 (actuator lag in the backup rows and in the world barrier). Our filter already uses a 100 ms zero-order hold in its rollout, so this is the natural reference for adding lag. It must be read in full before use.

### 2.9 Scalable backup sets

- T. Gurriet, M. Mote, A. Singletary, P. Nilsson, E. Feron, A. D. Ames, "A Scalable Safety Critical Control Framework for Nonlinear Systems," *IEEE Access*, 8:187249–187275, 2020. DOI [10.1109/ACCESS.2020.3025248](https://doi.org/10.1109/ACCESS.2020.3025248).
- Read: already cited in P2-SA-02 §4; not re-read for this note.

**Mapping.** The backup-set method our filter is built on. Single-system framework; it does not address the multi-agent compatibility problem of §2.1.

### 2.10 Found but not relied on

These were found while searching. Only their bibliographic data and abstracts were seen, and this note draws no conclusion from them.

- M. Jankovic, M. Santillo, Y. Wang, *IEEE Transactions on Control Systems Technology*, 32:705–712, 2024. DOI [10.1109/TCST.2023.3324531](https://doi.org/10.1109/TCST.2023.3324531), arXiv [2207.04915](https://arxiv.org/abs/2207.04915). Journal follow-up on multi-agent CBF collision avoidance and liveness.
- D. E. J. van Wijk et al., arXiv [2503.15734](https://arxiv.org/abs/2503.15734). Disturbance observers combined with backup CBFs.

## 3. What the literature says about our failures

1. **The pairwise backup is the defect, not the colouring.** In CSA 2021 the backup belongs to the agent, not the pair. All pair constraints of agent `i` read the same trajectory `Φ^t_{π_i}(x_i)`. Then "every agent flies its backup" is one action that satisfies every row at once, and feasibility follows (CSA Theorem 1; Jankovic–Santillo Proposition 1 for the braking case). Our `_backup_rollout` computes a different trajectory for drone `i` in every pair it is in. No colouring of pair directions can restore the property in general, because the colouring only chooses signs of pairwise directions; it does not make drone `i` fly one trajectory.
2. **Several candidates per drone need a compatible selection.** A single fixed backup per drone (for example "brake to hover") is consistent but conservative, and the sandwich shows that braking alone is not enough. CSA 2021 §IV and Singletary 2022 both keep a small library of manoeuvres per agent and require a joint, known selection. That selection is a small combinatorial problem; CSA solves it by broadcast and smallest intervention.
3. **Decentralised splitting without coordination fails in sandwiches.** Jankovic–Santillo Fig. 2 is the sandwich. A decentralised version of our filter must broadcast the chosen manoeuvre (Singletary Theorem 2) and split each row (CSA eq. (10)).
4. **Hysteresis has a principled form.** Singletary 2022 Algorithms 1 and 2 switch manoeuvres only in the backup phase and only reset the clock when the full manoeuvre is certified. That is the property our latch was trying to get.
5. **The axis flip is a frame problem.** In CSA 2021 the backup depends only on the agent's own state. A candidate set defined in the world frame has no line-of-sight frame, so the flip at x = ±y (BL-11) cannot occur. This is our inference, not a claim of the papers.
6. **Disturbance, lag and walls are separate, compatible layers.** The tube of van Wijk et al. tightens each rollout constraint, including the world box; the sampled-data result handles lag; neither changes the backup structure. They can be added after BL-03.
7. **Physics still bounds the sandwich.** No method recovers the head-on sandwich above the best joint trajectory (about 4.18 m mid, P2-SA-02 §3). A consistent backup can only approach that bound.

## 4. Gap → best-fit method → what we must adapt

| Gap (backlog) | Best-fit method | What we must adapt |
|---|---|---|
| Pairwise backups credit a drone with incompatible escapes (BL-03; sandwich, bent and staged sandwiches BL-10, seeds 1177 and 4828 BL-08) | CSA 2021: one backup `π_i(x_i)` per agent, all pair rows on the same per-agent trajectories; §IV Algorithm 1 for compatible selection | Define a small per-drone candidate library in the world frame. Replace the broadcast loop by a deterministic centralised selection, because our filter is centralised. Our rows are first-order rollout rows at the closest-approach step, not the paper's sampled constraint set; keep that form but take sensitivities per drone. Clip every rollout step by the world box. |
| Direction reversals under noise (BL-06 strict xfail at σ 1.5); latch bookkeeping | Singletary 2022 TBC, Algorithms 1 and 2 | Store each drone's current candidate and manoeuvre clock in the latch. Reset the clock only when the fresh manoeuvre is certified; switch candidates only in the backup phase or when the current one fails. Add a margin band so that noise alone cannot trigger a switch. |
| 45° axis flip (BL-11) | Backups depending only on own state (CSA 2021) | World-frame candidate directions instead of line-of-sight axes. Our inference; no paper states it for this case. |
| Rows only at `k*` (BL-04) | Constraint at every sample time on the horizon (CSA 2021); backup CBF relative degree 1 (Chen et al. 2021, abstract only) | A window of rows around `k*`, or all sample times with a cost check. Inter-sample guarantee needs the robust-sampling reference, which this note did not read. |
| Actuator lag in the rows and in the world barrier (BL-05) | Sampled-data CBFs with input delay (Singletary, Chen, Ames 2020; abstract only) | Read in full. Then model the lag inside the per-drone rollout, which is cheap because the rollout depends only on the drone's own state. |
| Disturbance margin (BL-06) | Disturbance-robust backup CBF tube (van Wijk et al. 2024); robust CBF row (Jankovic–Santillo 2021); ISSf (Kolathaya–Ames 2019, Alan et al. 2022, abstract only) | Extend the tube to pairs: the pair margin grows by `δ_i(τ) + δ_j(τ)`. Feasibility is no longer guaranteed (van Wijk Remark 2), so keep the slack and report it. A contracting PD brake gives a bounded tube; the bang-bang brake does not. |
| World-box wind budget (BL-07) | Same tube applied to each wall constraint | Per-wall margin `δ(τ)` along the per-drone rollout, sized for 1.0 m/s² per axis (T2). |
| Decentralised operation | CSA 2021 eq. (10) and Theorem 1; Singletary 2022 Theorem 2 | Broadcast the chosen candidate and its clock; each drone takes half of each row. With noisy, differing observations the drones can disagree on the selection, so agreement needs the broadcast, not local recomputation. |
| Unrecoverable sandwiches | None; physical bound (P2-SA-02 §3) | Report each case against the best joint trajectory, not against `d_eff`. |
