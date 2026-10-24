# Skeptical review of P3-SA-03 (adaptive margin), 2026-10-24

Reviewer: Claude Fable 5.1, run as a fresh reviewer that did not write any of this work. Branch `research/adaptive-margin` at commit `c12b4b1`. Read in full: `docs/formulation/P3-SA-03-real-world-envelope.md` (the design note), `docs/results/p3-sa-03-interim-2026-10-24.md`, `research/adaptive_margin/` (`envelope.py`, `margin.py`, `tracker.py`, `sim.py`, `run.py`, `check.py`, `results/summary.md`, the `runs2-*.jsonl` files and the two `partial-96c7db4-*.jsonl` files), the branch diff of `docs/DECISIONS.md`, and the parts of main's `src/utm_services/safety/filter.py` that the margin leans on (`FilterConfig`, `_backup_steps`, `_backup_max_steps`, `_raw_world_limits`, the backup rollout and constraint rows). The matrix was not rerun. The 1168 result rows were recounted with a script written for this review; the scenario geometry was recomputed from `sim.scenario`; the constants were recomputed with SciPy; five section 2 sources were fetched and read.

Status of this document: historical record of one review. It changes nothing on the branch except adding itself.

## Verdict

**Not ready to move into main's filter.** The prototype is honest about most of its limits, the result tables are correct to the row, and the source quotes check out. But the margin that the rows measure is the wrong quantity in three ways that each change the safety claim, not just its presentation:

1. The GNSS term projects the vertical position error onto horizontal separation. Under SPS and SBAS this alone is 85 to 95 % of the margin and most of the `absorb` shortfall. The spacing and capacity figures for those grades, and the "up to 16 m short" result, are artefacts of this choice (blocking issue B1).
2. The margin depends on the pair's state (closing speed, lost-command streak), so the safe set shrinks while drones are already inside it. The rate limit turns the resulting infeasibility into an alarm instead of preventing it. This is a recursive-feasibility gap, and the fix is a design change, not a tuning (B2).
3. The barrier assumes every drone can deliver u_max = 3.5 m/s² in every horizontal direction. The note's own airframe model says the registered fleet cannot do that above about 8.8 m/s. The `headroom` alarm fires in 132 of 144 nominal calm runs for this reason, and the simulator clips the thrust in 2 to 6 runs per in-limit condition and 15 to 18 per take-off grade. The margin derivation (A_REL − ρ) rests on an authority the drones do not have (B3).

Beyond these, the lag term is understated by a factor the note itself names (B4), the protection-level rule of §3.10 would admit SBAS although it fails the rule's own time-to-alert condition (B5), and the per-check risk has no flight-hour meaning (B6). None of these is hidden by the authors; several are in §7 or §8 as options or founder items. The point of this review is that they are not optional for a filter on main: each one changes a safety threshold or the shape of the constraint, so each needs a founder decision before the code moves.

What is ready: the tracker (Van Loan discretisation, Joseph update, NIS window), the command log and lost-command bound, the delay prediction, the lag identifier as a diagnostic, the monitor as a reporting layer, and the harness that produced reproducible rows at a fixed commit. These can move as diagnostics without any safety claim attached.

## Blocking issues

### B1. The isotropic GNSS ball turns vertical error into horizontal spacing

§3.2 and `margin.margin` bound the relative position error by a ball of radius K √λ_max(S_i(T) + S_j(T)) and add that radius to d_eff along the line of sight. For the GNSS sets of §2.1 the largest eigenvalue is the vertical one: σ_v = 6.633 m (SPS) and 3.878 m (SBAS) against σ_h = 3.268 m and 3.105 m. With K = 4.033 and two independent trackers:

| grade | K √2 σ_v (isotropic, as built) | K √2 σ_h (horizontal) | matrix median margin |
|---|---|---|---|
| SPS | 37.8 m | 18.6 m | 38.95 m |
| SBAS | 22.1 m | 17.7 m | 23.26 m |
| RTK | 0.17 m | 0.10 m | 1.34 m |

The remaining 1.1 m is EPS_LAG + EPS_LIN + δ_sample + the lost-command term. So the SPS margin is the vertical GNSS floor, and the SBAS margin is mostly the vertical floor. Every pair in the matrix is level (Z_CENTRE 115 m, `climb2` apart), so the vertical error is almost orthogonal to the line of sight for almost every pair, and the margin is roughly twice what the Gaussian model needs along the direction that matters.

Consequences in the rows: the SPS and SBAS lane spacing (44.0 m and 28.3 m), the capacity figures (66 and 280 drones per km), the `wall` alarm rate (34 to 40 of 47 SPS runs), the `margin` alarm in all of B7 (the honest worst-site σ_v = 16.8 m gives 96 m, over the 48.57 m cap, while σ_h alone would give 35 m, under it), and the large `absorb` deficits all follow from the vertical σ. The founder's decision 1 (use guaranteed 95 % figures) is not the cause; the projection is.

What the model actually supports: the relative error along a unit direction n is Gaussian with variance nᵀ(S_i + S_j)n, so the one-dimensional bound K₁ √(nᵀ S n) with K₁ = √χ²₁(0.999) = 3.29 holds per direction. The line of sight turns during the backup, so a sound replacement is the maximum of that bound over the rollout's line-of-sight directions, or the horizontal 2-D ellipse (K₂ = 3.72) for the horizontal part plus a separate vertical check. This is a change to the constraint's shape, so it needs a founder decision and a new matrix; the 1168 rows cannot be reinterpreted.

### B2. The margin is state-dependent, so the safe set shrinks under the drones; the rate limit reports this instead of preventing it

The brief asks whether the `absorb` shortfalls are a recursive-feasibility failure. Two different things are mixed in the `absorb` rows, and only one of them is.

**(a) Inadmissible initial geometry (not recursive feasibility).** Recomputing `sim.scenario` gives the initial minimum pair distance per seed. In the `random` family: seed 0 starts 9.8 m apart, seed 5 17.9 m, seed 1 29.1 m, seed 2 46.9 m, seed 4 4.9 m (excluded as unrecoverable). Under SPS the required d_eff is 44 m, under SBAS 28 m. These pairs are inside d_eff at t = 0, so no filter can hold the required margin; the backup rollout's minimum distance is simply the current distance. Every large deficit is one of these: SPS `random` seed 5, deficit 16.37 and 14.04 m, `absorb` first raised at 2.0 s (the first monitored cycle); `random` seed 0, 12.17 and 10.10 m, at 2.0 s; `random` seed 1, 1.77 and 1.53 m. The margin in force then is whatever the geometry allows (m_app = m*), and the pair separates over the run. This is a clearance failure: the scenario was admitted with less than d_eff of spacing. The fix is an admissibility check at clearance (initial separation at least 5 m + m_req plus what the rollout needs), which is strategic spacing, and the in-limit filter of `run.table` should exclude scenarios that are inadmissible at the grade's margin as it already excludes unrecoverable ones. The headline "up to 16 m short under GPS" should be withdrawn; it measures the scenario, not the mechanism.

**(b) A rising requirement in a converging ring (recursive feasibility).** In `ring6` under SPS, 22 of 24 runs raise `absorb` between 4.7 and 8.3 s with deficits 0.5 to 3.5 m, and in `ring4` 12 of 16 runs with deficits up to 0.8 m. Initial spacing there is 68 to 172 m, so the geometry is admissible. The rows show m_req rising above the median as the ring converges: `ring6` seed 2 has margin_p50 39.03 m, margin_max 40.93 m, margin_req_max 44.08 m. The requirement grows because T = (V_c + Kσ_vc)/(A_REL − ρ) grows with the closing speed (velocity covariance enters S(T) as T²), and the lost-command term grows with the loss streak. The safe set {h ≥ 0} at the new m_req is smaller than the one the drones were kept in, so states that were safe become infeasible. main's argument that the backup is always a feasible command assumes a fixed d_eff; §3.9 says so and sketches why the rate limit keeps the next QP feasible. The sketch is right as far as it goes: it keeps the QP feasible by refusing to raise the margin. But the claim being certified is "the pair stays outside 5 m + m_req with probability 1 − P_RISK", and in an `absorb` cycle m_app < m_req, so the claim is not in force. The design converts a hole in the guarantee into an alarm. In the rows this is 183 to 189 cycles per SPS condition (about 0.4 % of adaptive cycles) and 93 to 97 under SBAS.

**Minimal fix.** Remove the state dependence from the parts of m_req that can rise during an encounter, so the safe set does not shrink while a pair is inside it:

- Evaluate T at its worst case for the pair at clearance, that is at V_c = 2 v_max (or simply at T_MAX = 10.9 s, main's own cap), and keep it for the pair. The cost is a few metres under SPS and SBAS and centimetres under RTK, because the velocity floor is small (σ_vel = 0.102 m/s). Combined with B1 the SPS margin still falls.
- Use the design's one-loss bound of §3.6 as a constant term, and treat a second consecutive loss as a `link` event (the monitor already does) rather than as a margin rise.
- Keep the rate limit as a guard for the remaining slow drift from the covariance, with `absorb` as a fault, not a routine alarm.
- Add the admissibility check of (a).

Anticipatory margin (raising m early from the forecast closing speed) is a weaker fix: it still rests on the forecast and still produces the same shrinking set when the forecast is low. Strategic spacing alone fixes (a) and not (b).

**A related accounting error.** `run.table` counts `absorb` as "an alarm before any LoS". In B8 SBAS `ring4` seed 0 the only alarm before the 12.89 s loss is `absorb` at 8.4 s, a self-reported failure to hold the margin, not a detection of the fault. The B8 row should read 5 of 16, not 6, and `absorb` should be excluded from the warning count everywhere.

### B3. The fleet cannot deliver the filter's u_max at the matrix's cruise speeds

`headroom` compares the drag ‖D(ŵ − v̂)‖ + Kσ with a_phys − u_max. For the M350 (a_phys = 5.66 m/s², D = 0.246 s⁻¹) the drag alone exceeds 2.16 m/s² above 8.8 m/s airspeed. The scenarios draw speeds in 5 to 15 m/s, so in calm air the alarm fires in 132 of 144 nominal calm adaptive runs at the first monitored cycle (the other 12 are slow draws), and in 46 of 47 in each India condition. The note reads this as a chronic nuisance (§6.1, founder item 4) and `run._first_alarm` discards it at warm-up.

The alarm is right and the dismissal is wrong. The margin derivation (§3.3) uses A_REL = √2 u_max = 4.95 m/s² as the pair's relative braking, and main's `_backup_steps` derives its horizon from u_max. If the airframe cannot add u_max to the drag, the backup brakes slower than assumed, the stopping time is longer than T, and the tube bound S(T) is evaluated too early. The simulator confirms the authority is really missing: `_authority` clips the thrust command, and `saturation` fires in 2 to 6 runs per in-limit condition and 15 to 18 runs per take-off grade. The matrix shows zero losses because the vertical-σ margin of B1 is tens of metres wide; under RTK, where the margin is 1.3 m, the minimum separation in calm air is 6.52 m against d_eff of 6.34 m, leaving 0.2 m. That is not a demonstration that the barrier holds with the available authority.

Before this moves to main, either the filter's u_max becomes airframe- and airspeed-dependent (u_avail = a_phys − D|w − v|, with the backup horizon and A_REL derived from it) or the cleared cruise speed is bounded so the registered drag fits (about 8.8 m/s for the M350 in calm air, less in wind). Both are threshold changes for the founder. Making the check directional (option in founder item 4) does not restore the authority the backup needs.

### B4. EPS_LAG covers one command switch; the backup makes at least two

§3.5 derives EPS_LAG = ½ τ_a² · 2 · DC_MAX = 0.758 m from the asymptotic position error of a first-order lag after one command step of Δc = 2√3 u_max. The derivation is correct for one step; I reproduced the asymptote. But main's evasive backup brakes and then holds, so the relative motion sees at least two switches (the first command change and the end of braking), and the escape stage can add a third. §7 admits "at least two". A term with a known factor-of-two hole is not a bound. The fix is to multiply by the number of switches the backup can make within T (two at least; the rollout can count them), or to simulate the lag inside the backup rollout as main's `lag_s` was meant to and §6.1 shows it does not (`baseline_lag` equals `baseline` in 284 of 288 pairs, because `lag_s` only lengthens the horizon). Also the step size is inconsistent between terms: EPS_LAG uses Δc = 2√3 u_max = 12.1 m/s², DU_NOW uses 2√2 · max a_phys = 19.4 m/s². The command the drone receives includes the drag trim, so its change can exceed 2√3 u_max; the two terms should use the same bound.

### B5. The protection-level rule of §3.10 admits SBAS although SBAS fails the rule's own conditions

The rule (design only, not prototyped) requires a qualifying PL to have "time to alert shorter than the filter's horizon T", then says "under §2.5 this is SBAS today". Kim et al. 2016 Table 2, which the note itself cites, gives APV-I a time to alert of 10 s. T is at most 10.9 s (T_MAX) and for a head-on pair at matrix speeds about 4 to 6 s. So SBAS does not qualify for most pairs by the rule as written. Second, PL / K_PL with K_PL = 6.0 or 5.33 recovers d_major or d_up, and the rule then applies K = 4.033. That treats the broadcast σ as a Gaussian overbound valid at every quantile. SBAS σ bounds are designed as CDF overbounds, so re-scaling is defensible, but the integrity risk they bound is 2 × 10⁻⁷ per 150 s approach, and the note does not translate that to the per-100-ms check it needs. Third, Blanch et al. 2012 is an ARAIM paper; quoting it for how "an SBAS receiver" computes PLs is a loose attribution (the sentence is the paper's description of the SBAS precedent, which is fine, but the ARAIM PL construction is different and the note does not use it). The u-blox conclusion is right: a 5 % TMIR per epoch cannot serve a 10⁻³ per check requirement, and the release note says the PL is validated only for RTK with RTCM input.

Why blocking: the rule is a founder decision recorded as settled, and as written it would let SBAS into the dense-corridor path on a condition it does not meet. It must be corrected before it is prototyped.

### B6. P_RISK = 10⁻³ per check per pair has no flight-hour meaning, and K must be re-sized before any threshold enters main

At 10 Hz one pair makes 36 000 checks per hour; at 10⁻³ per check the expected number of exceedances is 36 per pair-hour. The note says this in §7 and §8 (founder item 3). But the matrix's margins, spacings and capacities are all computed at K = 4.033, and that K would be the number that moves into main. For a per-hour target the union bound over checks gives, per pair:

| target per pair-hour | per-check risk | K | SPS margin (isotropic / horizontal) | RTK |
|---|---|---|---|---|
| 10⁻³ per check (as built) | 10⁻³ | 4.03 | 37.8 / 18.6 m | 0.17 m |
| 10⁻³ | 2.8 × 10⁻⁸ | 6.17 | 57.9 / 28.5 m | 0.26 m |
| 10⁻⁵ | 2.8 × 10⁻¹⁰ | 6.89 | 64.6 / 31.8 m | 0.29 m |
| 10⁻⁷ | 2.8 × 10⁻¹² | 7.54 | 70.7 / 34.8 m | 0.32 m |

The union bound is loose because the per-run GNSS bias makes consecutive checks correlated, but no tighter argument is given, and a correlated error works against the claim as well as for it: a bias that exceeds the ball once exceeds it for the whole encounter. Note that the SPS margin at any per-hour target exceeds the 48.57 m cap if the isotropic ball of B1 is kept. The founder owns P_RISK; this review's point is that the rows measured at K = 4.033 do not transfer to the K that main would need.

## Non-blocking issues

- **N1. Lag identifier false alarms.** `lag` fires with the declared lag in force: B1 15 of 16 runs at 2.1 to 2.2 s (squall), B2 10 of 16, B5 3 of 16 during the uplink gap, SPS India 6 of 47, RTK India 6 of 47, and 8 to 11 of 47 in every take-off grade. The design level is 10⁻³ per test with Bonferroni over five lags; the observed rate in wind and during acceleration is of order 10⁻¹ per run. The likely cause is the regressor: wind acceleration and the authority clip enter the residual as lag would. Until this is fixed the response wired to `lag` (take the drone out of dense flows) would act on false alarms. The Bonferroni limit itself (F₁,₅₆(1 − 2 × 10⁻⁴) = 15.84) is correct; overlapping windows make the per-run false-alarm rate higher than per test, which the note does not state.
- **N2. Covariance at T only.** The bound uses S(T) for every τ ≤ T. van Wijk's Lemma 1 requires ε_τ ≥ L_h δ_max(τ) for all τ, which S(T) satisfies only if the predicted position variance is non-decreasing in τ. With the drag plant and velocity uncertainty it is in practice, but `check.py` does not test it. One check (λ_max(S(τ)) non-decreasing on the grid) would close it.
- **N3. δ_sample uses the estimated relative speed without Kσ.** Negligible (centimetres) but inconsistent with the rest of the derivation.
- **N4. The station is given the true initial trim.** `sim.episode` passes a0 = −D(w(0) − v) to the tracker and the command log. A real station has the first command, not the drone's trim. Small, in-model, worth one line in §7.
- **N5. `monitor` checks the wall limits at the requested margin, `absorbable` at the previous margin.** The configuration in force is m_app. Also the capacity grid puts lanes at the corridor walls where the `wall` alarm says the backup's A_REL does not hold; the lane count should exclude one standoff per wall. The arithmetic (80 / 8 / 3 lanes, 12 400 / 280 / 66 drones per km) reproduces.
- **N6. The baseline comparison is not a regression of main.** The baseline feeds raw, biased, delayed measurements with no tracker into a filter that main documents as needing BL-06 and BL-07 before any such use. "main lost 99 of 141 in-limit runs" is a fair statement of motivation and an unfair statement about main. §6.6 is good work on the RTK calm diagnostic; its conclusion (the drag trim hurts) also means the baseline's plant model is not main's, so even the baseline's own numbers are partly about the harness.
- **N7. The B1 `lag` alarm is counted in the "case alarm" column** ("`lag` 15 (2.2 s)") next to `wind`; the note calls it a false alarm two paragraphs later. It should not appear as a case alarm.
- **N8. "The monitor caught every fault in B1–B7 before any loss" (summary item 3)** is vacuous where there were no losses and false for B6 RTK (14 of 16). With 16 runs per case the rule of three bounds the miss rate below 19 % at 95 %, so "16 of 16" supports "the detector works", not "the detector always warns". §6.2's prose is more careful than the summary.
- **N9. van Wijk's Lemma 1 has a second condition** (ε_b ≥ L_hb δ_max(T) on the terminal backup set) that the note drops because main's barrier has no terminal invariant set. That is a real difference from the cited guarantee: without a terminal set, the robust backup argument only covers the rollout horizon, which is main's known limit (`_backup_max_steps` docstring: "a loose bound, not a proof"). State it.
- **N10. Singletary, Chen and Ames assume a constant, known delay and no input loss.** Algorithm 1 predicts from the buffer of sent inputs; Theorem 2 assumes an accurate model; Corollary 1 covers state uncertainty only. The variable playout delay and the lost-command bound are the note's own extension and should be labelled as such.
- **N11. GAGAN 7.6 m 95 %.** ICAO-hosted GAGAN papers (APANPIRG/22 WP31, the 2012 India validation paper) repeat 7.6 m horizontal and vertical as the Annex 10 APV-I accuracy. I could not check the Annex 10 text itself. The conversion to σ is right if the figure is right.
- **N12. SPS velocity.** "≤ 0.2 m/sec 95 % velocity error, any axis" is in Table 3.8-3 as quoted. It is a global average with a position/velocity/time solution available, not a worst-site figure; the note uses it for SPS_WORST too.
- **N13. Capacity needs the median margin, but margins rise in encounters.** §6.4 says so. With B1 fixed the table changes anyway.
- **N14. `random` seeds 0, 1 and 5 are not in-limit scenarios for SBAS and SPS** (see B2a). Excluding them changes the in-limit counts from 47 to 44 per condition for those grades and removes the two smallest minimum separations (9.68 m in every SBAS and SPS condition is `random` seed 0, a pair that started 9.8 m apart).
- **N15. `headroom` first time is discarded only when ≤ 2.0 s**, and the row keeps only the first time of each alarm, so a run with `headroom` at 2.0 s and again at 11 s counts as no `headroom` warning. §7 says this. The alarm rows should keep all times, or at least the first after warm-up.

## Answers to the six review items

### 1. Re-derivation of the margin

Terms present: tube K √λ_max(S_i + S_j) at T; lost-command bounds lost_i + lost_j with DU_NOW; EPS_LAG; δ_sample(V_rel); EPS_LIN. Constants reproduce: K = 4.0331, A_REL = 4.950 m/s², T_MAX = 109 × 0.1 = 10.9 s (main's `_backup_max_steps`), M_MAX = 85 − 255/7 = 48.57 m (main's `FilterConfig` bound), EPS_LAG = 0.758 m, DU_NOW = 0.097 m, EPS_LIN = 0.137 m (PR #12 measurement, carried over without re-measurement on the drag plant). The drag plant's T formula and the three-iteration fixed point on ρ are consistent with `margin.margin`; the `authority` alarm when ρ ≥ A_REL is the right fallback.

Wrongly bounded: the GNSS term (B1), T's state dependence (B2), u_max availability (B3), EPS_LAG switches and Δc (B4). Missing: nothing is double-counted; the drag's own uncertainty enters through ρ and through P_ww consistently. What is missing is a term for the model error of the drag itself (shape and coefficient; §7 names it), and the covariance's time dependence (N2). δ_sample lacks its Kσ (N3). The airspeed-drag plant is implemented consistently between tracker and simulator, which is also why it proves little: `check.check_drag` tests the tracker against its own plant.

### 2. GNSS integrity

HPL = 6.0 d_major and VPL = 5.33 d_up are quoted correctly from Kim et al. 2016 Eq. 5 (confirmed from the paper), and APV-I HAL 40 m, VAL 50 m, TTA 10 s, integrity 1 − 2 × 10⁻⁷ per approach are Table 2. The u-blox quotation is verbatim from UBX-21047459 §2.6 and the "does not qualify" conclusion holds. The SBAS conclusion does not hold against the rule's own TTA condition (B5). The SBAS spacing figure of 28.3 m is the vertical σ projected horizontally (B1); with the horizontal σ and K₁ it would be about 15 m before the PL rule, and the PL rule would raise it again (the broadcast d_major under APV-I is typically larger than the measured 3.1 m σ).

### 3. Statistics

K = √χ²₃(0.999) = 4.033, χ²₆₀(0.999) = 99.61, F₁,₅₆(1 − 2 × 10⁻⁴) = 15.84 are correct. The ball of radius K √λ_max contains the 1 − P_RISK ellipsoid, so the bound is conservative given a Gaussian error. The Gaussian assumption is in-model: the per-run GNSS bias is drawn from N(0, σ) once and held, so within a run the error is biased and consecutive checks are correlated; 10⁻³ per check then says nothing per run or per hour (B6). Rule of three: 0 of 282 bounds the per-run loss rate below 1.06 %, 0 of 141 below 2.1 %, 0 of 16 below 19 %, at 95 %. The note states the first; the beyond-limit cases should carry the third. Per-check versus per-flight-hour: see B6.

### 4. Recount

All 1168 rows are at `486f3b4bd66c`, not dirty, no duplicate (family, condition, seed, mode) keys. `random` seed 4 is the single unrecoverable scenario (ideal 4.51 m), so 47 per in-limit condition and 282 adaptive in-limit runs. Every number in `summary.md` and §6.1 to §6.4 reproduces: LoS 0 in all adaptive in-limit rows; baseline LoS 17 / 2 / 39 / 11 / 43 / 10; runs with any alarm 43 / 46 / 43 / 46 / 45 / 46; runs with an alarm other than `headroom` 5 / 7 / 25 / 28 / 36 / 40; absorb runs, cycles and largest deficit 4 (22, 3.16), 2 (6, 0.54), 13 (93, 4.89), 13 (97, 3.14), 17 (183, 16.37), 18 (189, 14.04); median margins 1.34 / 1.43 / 23.26 / 23.28 / 38.95 / 38.95; minimum separations 6.52 / 6.60 / 9.68 / 9.69 / 9.68 / 9.68; "13 of 282 raised no alarm at all" and "141 leaving headroom aside"; the per-condition alarm lists of §6.1; B1 to B8 alarm counts, first-alarm medians (B2 9.55 s rounds to 9.6; B4 9.9 s; B5 9.4 s; B6 SBAS 10.5 s; B6 RTK 11.45 s rounds to 11.5), minimum separations (B6 17.55 m and 7.93 m), the two B8 losses (`ring4` seed 0 at 12.89 s, 2.36 m, `absorb` at 8.4 s; `ring4` seed 5 at 11.76 s, 3.10 m, no alarm but `headroom` at 2.0 s), B6 RTK `headon2` seeds 2 and 3 without `lag`; take-off medians 4.22 / 23.53 / 39.13 m, "alarm in the last 5 s" 4 / 11 / 17, absorb 2 (13, 1.96), 11 (78, 2.11), 20 (166, 4.02) and the other-alarm lists; capacity 80 / 8 / 3 lanes and 12 400 / 280 / 66; `baseline_lag` equal to `baseline` in 284 of 288 pairs, the other four within 0.153 m; the 119 partial rows at `96c7db4d5897` identical in minimum separation, loss time and alarms to the final rows.

Claims that do not follow from the rows: "alarm before every loss in B1–B7" (no losses in B1–B7, so the statement is empty; B6 RTK warned in 14 of 16); "caught every fault in B1–B7" (B6 RTK); B8 SBAS "6 of 16 alarm before any LoS" counts `absorb` (N7, B2); the B1 "case alarm" column includes a false `lag` alarm (N7).

### 5. The absorb shortfalls

Answered under B2: the 10 to 16 m deficits are inadmissible initial geometry in the `random` family (not recursive feasibility, fix at clearance); the 0.3 to 3.5 m deficits in the ring families are a shrinking safe set from a state-dependent margin (recursive feasibility, fix by evaluating T at its worst case per pair and making the lost-command term constant). Rate limiting alone is not a fix; it is the mechanism that converts the hole into an alarm. Anticipatory margin is weaker than a worst-case T. Strategic spacing fixes (a) only. Fixing B1 shrinks both kinds under SPS and SBAS by about half.

### 6. Source spot-checks (five, plus one)

| Source | Quote in §2 | Check |
|---|---|---|
| GPS SPS PS 5th ed. 2020, Table 3.8-3 | ≤ 8 m / ≤ 13 m 95 % global average; ≤ 15 m / ≤ 33 m worst site; ≤ 0.2 m/sec 95 % velocity error, any axis | exact; intro "horizontal accuracy of 3 meters or better" also present |
| Barrios-Muñoz et al. 2024, arXiv 2411.09666v2, Table III | RTT median 54.0 / 55.90 / 58.03 / 59.91 ms; mean 62.09 / 58.65 / 59.98 / 66.41; 5th percentile 42.74 / 53.57 / 54.50 / 53.42 | all twelve numbers exact; Benidorm, LTE B3 and NR n78 confirmed |
| Kim et al. 2016, JPNT 5(4), Eq. 5 and Table 2 | HPL = K_H d_major, VPL = K_V d_up, K_H = 6.0, K_V = 5.33 (PA mode); APV-I HAL 40 m, VAL 50 m, TTA 10 s | exact |
| u-blox UBX-21047459 R01 §2.6 | "TMIR of 5 [%MI/epoch] ... 5% probability of having misleading information per epoch"; position only; validated for RTK with RTCM | verbatim |
| van Wijk et al. 2024, arXiv 2409.07700, Lemmas 1 and 2 | δ_max(τ) = (ξ / L_cl)(e^{L_cl τ} − 1); ε_τ ≥ L_h δ_max(τ) | exact; Lemma 1's second condition ε_b on the terminal set is not carried (N9); author list correct |
| PX4 parameter reference | MPC_TILTMAX_AIR 45° default; EKF2_ACC_NOISE 0.35 m/s² default; COM_OF_LOSS_T "Time-out to wait when offboard connection is lost" | the first two confirmed from the live page; the COM_OF_LOSS_T default value did not come through my extraction, description confirmed |

Singletary, Chen and Ames 2020: Algorithm 1 and Corollary 1 are as described; the paper assumes a constant, known delay of n Δt and no input loss (N10). GAGAN 7.6 m: see N11.

## What must change first

In order, each a founder decision because each changes a threshold or the constraint:

1. **Direction-aware GNSS term** in place of K √λ_max (B1). Re-run the matrix; the SPS and SBAS spacing, capacity, `wall`, `margin` (B7) and `absorb` results all change.
2. **Worst-case T and constant lost-command term per pair, plus an admissibility check at clearance** (B2). Exclude inadmissible scenarios from the in-limit set; stop counting `absorb` as a warning.
3. **Authority**: airframe- and airspeed-dependent u_max in the barrier and backup horizon, or a cruise-speed cap that keeps the registered drag inside a_phys − u_max (B3). Until then `headroom` is a safety alarm, not a nuisance.
4. **EPS_LAG for the backup's switches**, with one Δc bound shared with DU_NOW (B4), or the lag simulated inside the rollout.
5. **Correct §3.10**: SBAS APV-I does not meet the TTA < T condition; state the per-check translation of the PL's integrity risk, or drop the rule to "PL as a radius with risk added" (B5).
6. **Size K from a per-flight-hour target** before any constant enters main (B6); record that the matrix numbers at K = 4.033 are per-check and do not transfer.
7. Then the non-blocking list, N1 (lag identifier false alarms) first, since its response removes drones from flows.

What can move to main now, as diagnostics with no safety claim: the tracker and its self-checks, the command log, the delay prediction, the lag identifier's F statistic as a reported quantity, and the monitor's `wind`, `link`, `nis`, `saturation` and `wall` reports.

## Method and limits of this review

Row counts come from a script over `runs2-0.jsonl` and `runs2-1.jsonl` (1168 rows) and the two partial files (119 rows), reproducing `run.table`'s definitions (`_first_alarm` dropping `headroom` at ≤ 2.0 s; in-limit = ideal minimum separation ≥ 5 m). Initial geometry comes from re-implementing `sim.scenario` with its seeds and constants (FAMILIES order, Z_CENTRE 115 m, T_C_S 12 s). Constants were recomputed with SciPy. Sources were read from the publishers' PDFs or pages; the Kim et al. table values were read from the journal's HTML, where the table layout is partly jumbled, and I checked the APV-I row against the integrity value's known ICAO figure. I did not rerun any simulation, did not read `research/consistent_backup/harness.py`, and did not re-derive the Van Loan discretisation or the Dryden constants beyond confirming the code paths exist; the earlier review on this branch (`p3-sa-03-review-2026-10-24.md`) recounted those. Nothing on the branch was changed except the addition of this file.
