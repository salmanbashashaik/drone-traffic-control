# P3-SA-03 — Adaptive safety margin from measured real-world conditions

## Summary

Main's filter keeps every pair at least `5 m + margin_m` apart. Today `margin_m` is a fixed 0.5 m, and the filter acts on states that it assumes are exact and current. This note replaces that fixed number with a margin computed every 100 ms from live quantities: the tracker's covariance, the predicted velocities, the wind estimate, the age of the unconfirmed commands and the actuator lag. An envelope monitor states, for each condition, the limit inside which the margin is valid and the safe response outside it. Every physical input is either a cited published value (§2) or measured live. The main free choice is the risk level per check, P_RISK = 10⁻³, which is tied to the 99.9 % command-and-control reliability in 3GPP TS 22.125. The other modelling choices (which source figure to take, window lengths, floors) are listed in §8.

Findings (details in §6):

1. **Inside the limits, no loss of separation, in-model.** The adaptive prototype kept every pair apart in 282 recoverable runs under in-limit conditions (three GNSS grades, calm and India wind). The monitor stayed silent in 160 of them, and those 160 are the strict evidence. The tracker's model matches the simulated plant, so this is an in-model check (§6.1). Main as configured today (0.5 m, no delay or lag compensation) lost separation in 81 of the 141 calm-air runs. The baseline's zero losses in wind are a simulation artefact (§6.3).
2. **The margin follows the GNSS grade.** The median margin is 1.4 m with RTK, 12.9 m with SBAS (GAGAN) and 15.8 m with GPS alone. With SBAS or GPS the extra keep-out pushes drones towards the box edges, and the `wall` alarm fires in 28 % of those runs.
3. **The monitor caught six of seven beyond-limit cases** (squall, gust front, white GNSS noise, downlink and uplink outages, worst-site GPS), each by the alarm meant for it. It **did not catch a doubled actuator lag (B6)**: the margin assumes a known lag and nothing measures it.
4. **No beyond-limit case lost separation,** because all ran under SBAS, whose 13 m margin absorbed the errors. The matrix therefore does not show that the alarms precede a loss that would otherwise happen.
5. **The India mean wind (4.17 m/s at 100 m) sits at the envelope's edge.** The `wind` alarm fires in 58 % of the in-limit India runs. This and the risk level P_RISK need the founder's decision (§8).

| Field | Value |
|---|---|
| Task | P3-SA-03: real-world envelope, adaptive margin, tracker, monitor, prototype |
| Status | Research prototype; nothing in `src/` or `tests/` changes |
| Date | 2026-10-24 |
| Builds on | P3-SA-01 (literature), P2-SA-02 (backup filter), PR #12 (`EPS_LIN`, latch) |
| Code | `research/adaptive_margin/` (`envelope.py`, `tracker.py`, `margin.py`, `sim.py`, `run.py`, `check.py`) |
| Results | `research/adaptive_margin/results/runs-*.jsonl`, `research/adaptive_margin/results/summary.md` |

## 1. Problem

Main's `apply_filter` certifies separation for states that are exact, current and flown exactly as commanded. A real ground station sees none of that:

- Positions come from GNSS with metre-level error, and the error may be a slowly varying bias, not white noise.
- Telemetry arrives 20–60 ms late over cellular links, and a few messages are lost.
- Commands reach the drone late, and a few are lost. A lost command leaves the previous one in force.
- Wind pushes each drone by an unknown, changing acceleration.
- The autopilot follows an acceleration command through an attitude loop with a lag of a few hundred milliseconds.

The fixed 0.5 m does not depend on any of these. It is too small when they are large and wasteful when they are small. The task is a margin that follows measured conditions, together with a monitor that says when the conditions leave the range where the margin can be trusted.

## 2. Real-world data

Each figure is marked **primary** (read in the source itself) or **secondary** (read through a summary, mirror or search result). Per-axis standard deviations σ are converted from the published statistic with the Gaussian factors in `envelope.py`:

| Published statistic | Conversion |
|---|---|
| CEP (2-D, 50 %) | σ = CEP / 1.1774 |
| R95 (2-D, 95 %) | σ = R95 / 2.4477 |
| 1-D 95 % | σ = value / 1.960 |
| 1-D 50 % (R50 vertical) | σ = value / 0.6745 |

### 2.1 GNSS accuracy

| Set | Source | Published | σ_h (m) | σ_v (m) | σ_vel (m/s) | Status |
|---|---|---|---|---|---|---|
| `RTK` | u-blox ZED-F9P-02B datasheet (UBX-21023276) | RTK 0.01 m + 1 ppm CEP horizontal, 0.01 m + 1 ppm R50 vertical; 0.05 m/s velocity | 0.017 | 0.030 | 0.074 | primary |
| `SBAS` | ISRO, GAGAN status, ICG-11 2016 ([unoosa.org/pdf/icg/2016/icg11/05.pdf](https://www.unoosa.org/pdf/icg/2016/icg11/05.pdf)) | APV-1 bound < 7.6 m 95 %; measured about 3.0 m horizontal and 4.0 m vertical, 95 % | 1.226 | 2.041 | 0.102 | primary |
| `SPS` | GPS SPS Performance Standard, 5th ed., 2020 ([gps.gov](https://www.gps.gov/sites/default/files/2025-07/2020-SPS-performance-standard.pdf)) | "well-designed GPS receivers have been achieving horizontal accuracy of 3 meters or better and vertical accuracy of 5 meters or better 95% of the time" (2018 signal) | 1.226 | 2.551 | 0.102 | primary |
| `SPS_WORST` | same, Table 3.8-3 | worst site: 15 m horizontal, 33 m vertical, 95 %; velocity 0.2 m/s 95 % any axis | 6.128 | 16.837 | 0.102 | primary |

The `SPS` set uses the 2018 performance expectation, not the standard. The standard itself (Table 3.8-3, global average) is 8 m horizontal and 13 m vertical, 95 %. The `SBAS` and `SPS` velocity σ come from the SPS standard's 0.2 m/s, because the GAGAN source gives no velocity figure.

For comparison, the DJI FlyCart 30 and Agras T40 specifications give hovering accuracy of ±10 cm with RTK and ±60 cm horizontal / ±30 cm vertical without it (primary, [dji.com](https://www.dji.com/global/flycart-30/specs)). That is control accuracy, not position accuracy, so it is not used.

**Gap.** No source gives the error's correlation time. The simulator draws a constant bias per episode, and the tracker gets no credit for averaging position (§4.3).

### 2.2 Cellular link: latency and loss

| Source | Where | Figure | Status |
|---|---|---|---|
| Barrios-Muñoz et al. 2024, arXiv [2411.09666](https://arxiv.org/abs/2411.09666) v2, Table III | Spain, dense urban, phone on a drone | RTT median 54.0 ms on the ground, 55.90 / 58.03 / 59.91 ms at 20 / 40 / 60 m; mean 62.09 / 58.65 / 59.98 / 66.41 ms | primary |
| Zulkifley et al. 2021, *Sensors* 21(8) 2848, [doi:10.3390/s21082848](https://doi.org/10.3390/s21082848), Table 2 | Malaysia, LTE | average delay 25.4 ms (0 m), 24.73 ms (40 m), 23.44 ms (80 m), 36.63 ms (170 m); "up to 94 ms" | primary (Europe PMC full text) |
| TRAI press release 24/2026 ([trai.gov.in](https://trai.gov.in/sites/default/files/2026-02/PR_No.24of2026.pdf)) | Delhi, ground drive test, 6–9 Jan 2026 | 4G/5G p50 latency: Airtel 20.80 ms, MTNL 21.65 ms, RJIL 20.62 ms, VIL 38.98 ms | primary; packet-drop values are not legible in the PDF's text layer |
| Yang et al. 2018, arXiv [1803.11048](https://arxiv.org/abs/1803.11048) | LTE-Advanced field trial | latency concentrated at 400–500 ms at 300 m | secondary (search summary) |
| 3GPP TS 22.125 V19.1.0, Table 7.2-1 ([itecspec.com](https://www.itecspec.com/3gpp/22.125/s/7.2)) | requirement | command and control: 99.9 % reliability; end-to-end latency 40 ms (direct stick), 1 s (waypoints), 5 s (automatic flight on UTM) | secondary (mirror of the spec) |

**Model.** One-way latency is log-normal with median 59.91 / 2 = 29.96 ms and σ_ln = √(2 ln(66.41 / 59.91)) = 0.454, fitted to the 60 m row of Barrios-Muñoz (median and mean). The link is assumed symmetric. The 99.9 % quantile is 122 ms, so commands play out at a fixed P = 130 ms (13 ticks of 10 ms), and a command that arrives later is dropped. Loss per message is 10⁻³ (3GPP), independent.

**Gap.** No airborne cellular measurement from India was found. TRAI's figures are ground drive tests. They are lower than the Spanish airborne figures, so the Spanish figures are used.

### 2.3 Wind at 0–120 m over India

| Source | Figure | Status |
|---|---|---|
| CEEW, 100 m wind over India 1979–2019 ([ceew.in/node/1908](https://www.ceew.in/node/1908)) | annual mean falling about 0.88 km/h per decade; only Gujarat and Tamil Nadu average above 15 km/h | secondary (the page returned 403; read through a search summary) |
| MIL-F-8785C low-altitude Dryden, via [MathWorks](https://www.mathworks.com/help/aeroblks/drydenwindturbulencemodelcontinuous.html) | σ_w = 0.1 W20; σ_u = σ_v = σ_w / (0.177 + 0.000823 h)^0.4; L_u = L_v = h / (0.177 + 0.000823 h)^1.2, L_w = h (h in ft); W20 = 15 / 30 / 45 kt for light / moderate / severe | secondary (a reference implementation of the standard) |
| WMO squall definition (1962), via the ANAC aviation glossary and Wikipedia | rise of at least 8 m/s (16 kt) to at least 11 m/s (22 kt), lasting at least 1 min | secondary |
| IS 875 Part 3 (2015), via [infralens.in](https://infralens.in/code/IS-875-Part-3-2015/clause/6-2) | basic wind speed V_b (3 s gust, 10 m, open terrain, 50-year return) 33 m/s in zone I and higher elsewhere; height factor k2 = 1.20 at 100 m in terrain category 2 | secondary |

**Model.** The in-limit Indian case is a mean wind of 15 km/h = 4.17 m/s (the CEEW threshold that only two states exceed on average) in a random horizontal direction, with light Dryden turbulence at the encounter height of 115 m: σ_u = 1.03 m/s, σ_w = 0.77 m/s, L_u = 272 m, L_w = 115 m. Each drone's gust is a first-order Gauss–Markov process with time constant L / U. The beyond-limit squall uses an 11 m/s mean with moderate turbulence (σ_u = 2.06 m/s). IS 875's design gusts are structural extremes far outside any drone's envelope, so they are cited for scale only.

**Gaps.** No 100 m gust statistics for India were found (an ERA5 gust analysis would supply them). The 120 m ceiling comes from the brief; the DGCA rule text was not fetched.

### 2.4 Indian drones: speed, acceleration, lag

| Source | Figure | Status |
|---|---|---|
| Garuda Aerospace Agri Kisan drone (GA-AD), [garudaaerospace.com](https://www.garudaaerospace.com/products/precision-agriculture-drones/agri-kisan-drone) | rated speed 5 m/s; 24.56 kg gross; 8 kg payload | primary; no wind or GNSS figure |
| DJI Agras T40, [dji.com](https://www.dji.com/global/t40/specs) | max wind resistance 6 m/s; obstacle avoidance at ≤ 7 m/s; MTOW 90–101 kg | primary |
| DJI FlyCart 30, [dji.com](https://www.dji.com/global/flycart-30/specs) | max horizontal speed 20 m/s (windless, 30 kg); ascent 5 m/s; wind resistance 12 m/s; MTOW 95 kg | primary |
| PX4 parameter reference (`MPC_TILTMAX_AIR`, `COM_OF_LOSS_T`) | maximum tilt 45° by default; offboard-loss timeout 1.0 s, after which the failsafe acts | primary (saved reference page) |
| PX4 `MC_ROLL_TC` (former parameter), via QGroundControl metadata | roll time constant range 0.15–0.25 s | secondary; no longer in the current reference |
| Eschmann, Albani, Loianno 2024, arXiv [2404.07837](https://arxiv.org/abs/2404.07837) | Crazyflie 2.1 motor time constant T_m = 0.072 s | primary; a 27 g drone, so motor lag only |
| Faessler, Franchi, Scaramuzza 2018, RA-L 3(2), [arXiv 1712.02402](https://arxiv.org/abs/1712.02402) | mass-normalised rotor drag d_x = 0.544 s⁻¹, d_y = 0.386 s⁻¹ (circle), 0.491 / 0.236 s⁻¹ (lemniscate); 0.610 kg quadrotor | primary |

**Model.**

- **Speed.** The contract's 15 m/s covers the Garuda and T40 figures. The FlyCart 30's 20 m/s is above it; the contract is not changed here.
- **Physical acceleration.** At 45° tilt the horizontal acceleration reaches g tan 45° = 9.81 m/s². Per component (the box inscribed in that disc) this is a_phys = 6.93 m/s². The filter uses u_max = 3.5 m/s² per component, so 3.43 m/s² is left as headroom for cancelling wind.
- **Actuator lag.** First order with τ_a = 0.25 s, the upper end of the PX4 range.
- **Wind to acceleration.** d = D·w with D = 0.544 s⁻¹, the largest Faessler coefficient, on all three axes. Wind of 6.3 m/s uses up the headroom.

**Gaps.**

- No attitude-lag measurement exists for an Indian delivery or agricultural drone.
- D is from one 0.61 kg quadrotor. For 25–95 kg drones it is unknown.
- No Indian drone publishes an acceleration limit.

## 3. Adaptive margin

### 3.1 What the margin must cover

The filter runs at 10 Hz on the ground. At cycle t it computes commands that the drones apply at t + P. For each pair it requires separation ≥ d_eff = 5 m + m at the filter's own predicted states, and it has a backup manoeuvre (brake away along the line of sight) that it can always fall back to. The margin m must cover the gap between that certificate and the true motion. The gap has five sources:

1. the error of the state estimate, propagated open loop over the backup;
2. lost commands;
3. the actuator lag;
4. motion between 100 ms samples;
5. the filter's own linearisation (measured in PR #12).

Each is bounded below from live quantities, and the bounds are added.

### 3.2 The tube (van Wijk et al. 2024)

van Wijk, Coogan, Molnar, Majji and Hobbs ([arXiv 2409.07700](https://arxiv.org/abs/2409.07700), "Disturbance-Robust Backup Control Barrier Functions") bound the distance between the disturbed and nominal backup flows by a tube, ‖φ_b^d(τ, x) − φ_b^n(τ, x)‖ ≤ δ_max(τ) = (ξ / L_cl)(e^{L_cl τ} − 1) for a disturbance bounded by ξ (Lemma 2). They then tighten the safety function by ε_τ ≥ L_h δ_max(τ) along the backup (Lemma 1). For pair separation, h = ‖r‖ − d_eff has L_h = 1, so the margin is the tube's width.

Two changes make the tube adaptive and probabilistic:

- **Covariance in place of a fixed bound.** The deviation is the estimate error carried open loop through the kinematics. With the tracker's model (§4, a triple integrator per axis), the error covariance at a time τ after the action time t_a is S(τ) = Φ(τ) P(t_a) Φ(τ)ᵀ + Q(τ) (exact discretisation). This is the linear-Gaussian counterpart of δ_max: P(t_a) replaces the initial error, and the disturbance-estimate uncertainty P_dd together with the process noise q replaces ξ.
- **A per-check bound in place of a worst case.** The relative position error of two independent trackers is Gaussian with covariance S_i + S_j. It lies inside a ball of radius K √λ_max(S_i + S_j), with K = √χ²₃(1 − P_RISK) = 4.033, with probability at least 1 − P_RISK. Assuming S grows with τ, its value at the end of the backup, τ = T, bounds every earlier τ (§7 notes when this can fail).

This trades van Wijk's deterministic guarantee for a probability per check. The choice of P_RISK is the founder's (§8).

### 3.3 Backup stopping time T

Away from the walls, main's backup brakes each drone by at least u_max / √2 along the line of sight (`_backup_steps`). The pair therefore closes with deceleration at least A_REL = √2 u_max = 4.95 m/s². The feedforward (§3.7) cancels the estimated wind. What it misses is bounded at the same risk by ρ = K √λ_max(P_dd,i + P_dd,j + diag(q_i + q_j) T). The closing speed is bounded by V_c + K σ_vc, where σ_vc is the closing-speed standard deviation from the trackers. So

  T = (V_c + K σ_vc) / (A_REL − ρ).

ρ grows with T, so the pair is solved by three fixed-point iterations, and T is capped at main's rollout cap T_MAX = 10.9 s. If ρ ≥ A_REL, the braking promise fails: the monitor raises `authority`.

### 3.4 Prediction through the delay (Singletary, Chen and Ames 2020)

Singletary, Chen and Ames ([arXiv 2005.06418](https://arxiv.org/abs/2005.06418), "Control Barrier Functions for Sampled-Data Systems with Input Delays") handle an input delay by predicting the state at the moment the new input takes effect, from the buffer of inputs already sent, and enforcing the barrier condition there (Algorithm 1). Corollary 1 keeps safety when the prediction is uncertain, provided the invariant set is robust to that uncertainty.

Here:

- The buffer is the command log of each drone (§4.2), and the prediction runs from the latest measurement to the playout tick t + P.
- The robustness of Corollary 1 is the K √λ_max term of §3.2, computed with the position covariance floored at the receiver's value (§4.3).
- Algorithm 1 assumes that every buffered input reaches the plant. A lost command breaks that assumption, and §3.6 bounds its effect.

### 3.5 Actuator lag as an equivalent delay

The plant follows the command through a first-order lag, a' = (c − a) / τ_a. The filter's model has no lag state. After the playout tick, the prediction therefore holds the actuator output a0 plus the disturbance estimate for τ_a more seconds, in closed form. The filter then acts on the state at t_a = t + P + τ_a as if its command took effect at once.

For a step of size Δc, the lagged position minus the delayed position is Δc τ_a² (½ − e^{−t/τ_a}) for t ≥ τ_a, and it lies in [0, ½ τ_a² |Δc|] for every t. It tends to the upper end (`check.check_lag_bound`). Both the command and the actuator output lie in [−u_max, u_max] per component, so |Δc| ≤ 2√3 u_max per drone, and for the pair

  EPS_LAG = ½ τ_a² · 2 · 2√3 u_max = 0.758 m.

### 3.6 Lost commands

The 3GPP reliability of 99.9 % puts two consecutive losses of one drone's commands at 10⁻⁶, below P_RISK. The margin therefore covers one lost command per drone, and the monitor raises `link` at a streak of two.

A lost command c_k leaves c_{k−1} in force for its 100 ms. This creates a velocity error of |c_k − c_{k−1}| · 0.1 s from the middle of that segment, which is held open loop to t_a + T. `lost_bound` takes the worst such error over the commands that are not yet confirmed. A command sent in the current cycle is not yet in the log. It is covered by DU_NOW = ½ · 2√3 a_phys · 0.1² = 0.120 m per drone, and from the next cycle on it is counted with the others.

### 3.7 Sampling and linearisation

- **Sampling.** The filter checks separation every 100 ms. Two drones passing in a straight line at relative speed V_rel that are d apart at both samples come closest midway, at √(d² − (0.05 V_rel)²). Keeping that above 5 m needs δ_sample(V_rel) = √(5² + (0.05 V_rel)²) − 5. This is 0.22 m at V_rel = 30 m/s.
- **Linearisation.** The PR #12 ablation measured a gap of EPS_LIN = 0.137 m between the filter's linearised barrier and the true separation (`docs/results/pr12-final-report.md`). It is measured, not proven.

### 3.8 The pair margin and the filter margin

  m_ij = K √λ_max(S_i(T) + S_j(T)) + lost_i + lost_j + EPS_LAG + δ_sample(V_rel) + EPS_LIN

Main's filter takes one `margin_m` for all pairs. It therefore gets the largest pair margin. This is conservative for every other pair. Main's `FilterConfig` bounds `margin_m` at 48.57 m (the world-box bound at the default barrier offset). A larger margin is clamped and the monitor raises `margin`.

**Feedforward.** The filter plans in still air. The wrapper sends c = u − d̂, clipped to ±a_phys, so the drone's own acceleration cancels the estimated wind. If the clip acts, the monitor raises `saturation`.

## 4. Tracker

`tracker.py` keeps one Kalman filter per drone.

### 4.1 Model

The state is x = (p, v, d) per axis, with d the wind disturbance acceleration:

  p' = v,  v' = a_act + d,  d' = white noise of intensity q,  a_act' = (c − a_act) / τ_a.

The applied command c is known (§4.2), so a_act is a known input and not a filter state. The mean is propagated with the plant's own 10 ms step (`kin_step`). The covariance uses the exact (Φ, Q) of the triple integrator (`transition`, checked against Van Loan's matrix exponential in `check.check_transition`).

The wind state is identified from the residual between commanded and measured motion. The velocity innovation left after the known command is accounted for is attributed to d.

### 4.2 Delayed and out-of-order data

Measurements z = (p, v) carry their measurement time. The tracker keeps the posterior at the time of the latest fused measurement. An older message is dropped. To act, the tracker predicts from that time to the playout tick through the known commands (§3.4), then through the equivalent delay (§3.5).

`CommandLog` holds the switches that the drone reports as applied (its echo, up to the measurement time) and, after that time, the commands sent and not yet confirmed. A planned command missing from the echo is counted as lost and extends the loss streak. *Shortcut:* the echo carries the whole switch history, which is free in simulation. A real link would acknowledge each command.

### 4.3 Tuning from the data, not by hand

- **Measurement noise R.** From the GNSS set of §2.1.
- **Disturbance noise and prior.** From the forecast mean wind U and the Dryden class: q = 2 (D σ_g)² / τ_g per axis, the intensity whose stationary Gauss–Markov variance matches the gust's, and P_dd(0) = (D U)² + (D σ_g)², a wind of the forecast speed in an unknown direction. q is floored at 10⁻⁶ and P_dd(0) at 10⁻⁴ so that calm conditions stay positive definite.
- **Update.** Joseph form.
- **Consistency.** The innovation statistic (NIS, 6 degrees of freedom per message) is summed over 10 messages (1 s). `nis` is raised when the sum exceeds χ²₆₀(1 − P_RISK) = 99.6. On in-model data the mean NIS is within (5.6, 6.4) (`check.check_nis`).
- **Position floor.** The covariance used by the margin never shows a position σ below the receiver's. The sources give no error correlation time (§2.1), and a constant bias would not average out. The filter update itself uses the unfloored covariance.

## 5. Envelope monitor

The margin of §3 is valid only while its assumptions hold. `margin.monitor` checks them every cycle after a 2 s entry period (the tracker's convergence, 20 messages). The safe responses are proposals. The prototype records the alarms and does not act on them.

| Condition | Live quantity | Limit | Physical meaning of the limit | Safe response (proposed) |
|---|---|---|---|---|
| `wind` | per axis, \|d̂\| + K √P_dd at T | ≤ a_phys − u_max = 3.43 m/s² | about 6.3 m/s of wind at D = 0.544 s⁻¹, together with the uncertainty | Stop new departures in the area. Drones in flight hold or land: the filter's braking promise is no longer covered. |
| `authority` | ρ (§3.3) | < A_REL = 4.95 m/s² | wind uncertainty outweighs the pair's braking | As for `wind`. |
| `saturation` | \|u − d̂\| per axis | ≤ a_phys = 6.93 m/s² | 45° tilt cannot deliver the command | As for `wind`. |
| `nis` | Σ of the last 10 NIS | ≤ χ²₆₀(0.999) = 99.6 | the GNSS error or the wind is outside the tracker's model (GNSS noise larger than reported, a jump in the fix, a gust front); a steady bias is not seen | Treat the drone's position as unknown: set its σ to the worst-site figure (`SPS_WORST`), widen the margin, take it out of the flow and land it. |
| `link` | age of the latest confirmed command; loss streak | ≤ 1.0 s (`COM_OF_LOSS_T`); streak < 2 | the drone is about to fall back to its own failsafe; or the single-loss bound of §3.6 is broken | Treat the drone as non-cooperative. Reserve its reachable set from the last estimate and move the others away. |
| `wall` | any world-box acceleration limit active | none active | the backup's A_REL holds only away from the walls | Keep traffic one stopping distance from the box edges, or plan with the reduced A_REL there. |
| `margin` | required pair margin | ≤ 48.57 m (`FilterConfig` bound) | the filter cannot express the margin | Deconflict strategically: wider spacing or a ground hold. |

Conditions without a monitor:

- an actuator lag longer than modelled (§6, B6);
- a drag coefficient larger than D;
- correlated GNSS errors between drones;
- **a GNSS bias larger than the receiver reports.** The margin trusts the receiver's reported accuracy. A steady bias moves the fix and the tracker together, so the NIS cannot see it, whatever its size.

Each would need a measurement that the ground station does not have (§7).

## 6. Results

Setup (`sim.py`, `run.py`): six encounter families (head-on, crossing, climbing, rings of 4 and 6, random) with 8 seeds each, 20 s episodes, a 10 ms plant and a 100 ms ground-station cycle. One scenario of the 48 (random, seed 4) is not recoverable even for main on exact, current states: its drones are already inside 5 m at the first tick. It is excluded from the in-limit table. Recoverability was judged at main's default 0.5 m margin, not at the adaptive margin. The full tables are in `research/adaptive_margin/results/summary.md`; per-run rows are in `runs-*.jsonl`.

- **Adaptive:** the prototype of §3–§5 (tracker, prediction through the delay, adaptive margin, wind feedforward, monitor).
- **Baseline:** main as it is configured today: the latest raw measurement, the default `FilterConfig` (0.5 m margin, `lag_s` = 0), no delay compensation and no feedforward. Main's `lag_s` could be set to the plant's 0.25 s; that variant was not run.

### 6.1 Inside the limits

| condition | mode | runs | LoS | runs with alarm | alarms | margin p50 (m) | min sep (m) |
|---|---|---|---|---|---|---|---|
| RTK, calm | adaptive | 47 | **0** | 2 | wall 2 | 1.40 | 6.47 |
| RTK, calm | baseline | 47 | 7 | – | – | 0.5 | 2.53 |
| RTK, India wind | adaptive | 47 | **0** | 29 | wind 28, wall 4, margin 1 | 2.72 | 6.73 |
| RTK, India wind | baseline | 47 | 0 | – | – | 0.5 | 5.97 |
| SBAS, calm | adaptive | 47 | **0** | 16 | wall 16 | 12.86 | 9.59 |
| SBAS, calm | baseline | 47 | 37 | – | – | 0.5 | 1.48 |
| SBAS, India wind | adaptive | 47 | **0** | 28 | wind 27, wall 7, margin 3 | 12.95 | 9.69 |
| SBAS, India wind | baseline | 47 | 0 | – | – | 0.5 | 5.84 |
| SPS, calm | adaptive | 47 | **0** | 19 | wall 19 | 15.76 | 9.59 |
| SPS, calm | baseline | 47 | 37 | – | – | 0.5 | 0.65 |
| SPS, India wind | adaptive | 47 | **0** | 28 | wind 27, wall 10, margin 3 | 15.84 | 9.69 |
| SPS, India wind | baseline | 47 | 0 | – | – | 0.5 | 5.74 |

"min sep" is the smallest separation over all runs of the row. For SBAS and SPS it is set by the starting geometry (random, seed 0, whose drones start about 9.6 m apart, already inside the margin), not by the margin. "India wind" is the CEEW mean of 4.17 m/s at 100 m with light Dryden turbulence.

**Pass criterion 1 is met, in-model:** no loss of separation in 282 adaptive runs under in-limit conditions. The monitor raised an alarm in 122 of them, so by the monitor's own judgment only 160 runs (56 of 141 in the India wind) stayed inside the limits throughout; their smallest separation was 6.52 m. Zero losses in 160 runs bounds the per-run loss rate below about 2 % at 95 % confidence.

This is an in-model check. The tracker's mean model is the plant's own step (`kin_step`), its disturbance noise q matches the simulated gust process, the lag is known exactly, the command echo is exact and the GNSS error has the reported σ. What the in-limit runs test is the prediction through the delay, the lost commands, the noise and the margin's sizing, not the model's fidelity to a real drone. Under RTK the smallest separation (6.47 m) sat at the margin's edge (5 + 1.47 m), so the margin was close to fully used.

The baseline lost separation in 81 of 141 calm runs. The losses come from the fixed margin together with the uncompensated 130 ms delay, the unconfigured lag (`lag_s` = 0) and the stale measurement, not from the margin alone.

What the alarms in this table mean:

- **`wind` fires in 82 of 141 India runs (58 %), in 57 of them at the first allowed cycle (2.0 s), the rest by 9.2 s.** The condition was chosen as in-limit by its forecast, but the monitor judges a large share of these runs outside the envelope. The limit (|d̂| + K√P_dd ≤ 3.43 m/s², about 6.3 m/s of wind) leaves 2.1 m/s for the gusts and the estimate's uncertainty at K = 4.03. The India mean wind is therefore at the envelope's edge, not inside it (§8, item 2).
- **`wall` fires in calm and in wind, and more often as the margin grows** (RTK 6, SBAS 23, SPS 29 of 94 runs). It fires mostly in the ring and random families, where the keep-out of 13–16 m pushes the drones towards the box edges, late in the run (median about 16 s). It says that the backup's A_REL is no longer guaranteed there, not that separation was lost.
- **`margin` fires in 7 India runs:** the required margin exceeded 48.57 m and was clamped, so the margin's claim did not hold in those cycles. Five fire at 2.0 s, which points to the wind estimate not yet having converged at the end of the entry period rather than to gusts. The number of clamped cycles was not logged. No loss followed.
- Main's QP was infeasible and used its fallback in 2 of the 282 adaptive runs (SPS calm 1, RTK calm 1) and in 14 of the 141 calm baseline runs.

### 6.2 Outside the limits

All beyond-limit cases run on SBAS with the India wind, so each also carries the background `wind` alarms of §6.1. The table separates the alarm that belongs to the case.

| case | what breaks | runs | LoS | case alarm: runs, median time | background alarms |
|---|---|---|---|---|---|
| B1 squall, 11 m/s mean, moderate turbulence | wind | 16 | 0 | `authority` 16, `margin` 16, `wind` 16, at 2.0 s; `saturation` 14 | wall 9 |
| B2 gust front, +8 m/s on one drone at 9 s | wind model | 16 | 0 | `nis` 16, at 9.4 s | wind 14, saturation 3, wall 5, margin 1 |
| B3 GNSS white noise at 5 σ, no bias | GNSS model | 16 | 0 | `nis` 16, at 2.0 s | wind 10, margin 1 |
| B4 downlink out 3 s at 9 s | link | 16 | 0 | `link` 16, at 9.9 s | wind 9, wall 4, margin 1 |
| B5 uplink out 1.5 s at 9 s | link | 16 | 0 | `link` 16 at 9.4 s (loss streak), `nis` 16 at 10.5 s | wind 16, wall 2, margin 1 |
| B6 actuator lag 0.5 s (2×) | lag | 16 | 0 | **none** | wind 11, margin 1 |
| B7 GPS worst site (15 / 33 m, 95 %) | GNSS level | 16 | 0 | `margin` 16, at 2.0 s (margin clamped at 48.57 m) | wall 15, wind 9 |

Time 2.0 s is the first cycle the monitor is read (§5), so "at 2.0 s" means "from the start". The events that begin at 9 s are caught 0.4–0.9 s later: the gust front by `nis`, the downlink outage by `link` (the 1.0 s age limit) and the uplink outage by `link` (the loss streak of two).

Two cases are caught partly by construction. In B7 the ground station is told the worst-site σ, so `margin` fires because the honest σ makes the margin exceed its bound; a receiver that reported nominal σ while erring at the worst-site level would not be caught (§5, a steady bias). In B1 the tracker's noise q and prior come from the true 11 m/s and moderate turbulence class, as a forecast would give them.

**Pass criterion 2 is met for B1–B5 and B7, and not for B6.**

- **B6 failure: the assumption that breaks is a known actuator lag, τ_a = 0.25 s.** EPS_LAG (§3.5) and the tracker's mean model both use it. With the true lag at 0.5 s, the lag term alone should be 4 × 0.758 = 3.03 m, since it grows with τ_a². No quantity in the monitor measures the lag. The 11 `wind` alarms are the same background as in §6.1 (SBAS India: 27 of 47 in-limit runs), not a reaction to the lag. Nothing was tuned; §8 item 3 proposes a lag monitor.
- **No beyond-limit case lost separation, so the matrix does not show the margin failing.** Every case ran under SBAS, where the GNSS term dominates a margin of about 13 m. The B6 lag error, about 2.3 m beyond EPS_LAG, is small next to it: B6's smallest separation was 13.0 m. Under RTK (margin 1.4–2.7 m, smallest in-limit separation 6.47 m) the same error would use up most of the 1.47 m of slack, and B6 was not run under RTK. Pass criterion 2 is therefore shown as "the monitor fires", not as "the monitor fires before a loss that would otherwise happen".
- **B1 separations are large (median 47.6 m) for the same reason as the baseline in §6.3:** at 11 m/s the feedforward saturates, the part of the wind it cannot cancel carries all drones downwind together, and the encounters partly dissolve.

### 6.3 Why the baseline has no losses in wind

The baseline loses separation in calm air (81 of 141 runs) and never in the India wind. This is an artefact of the plant, not a property of main. The plant applies the wind as a constant push, d = D (W + gust), without the opposing drag on the drone's own speed (§7). The India mean wind gives D · U = 0.544 × 4.17 = 2.27 m/s². The baseline sends main's command without cancelling it, so in a few seconds every drone reaches the 15 m/s speed cap heading downwind. All drones then fly nearly the same velocity and the encounter dissolves: the baseline's median minimum separation in the India rows is 39.9 m. The adaptive prototype cancels the estimated wind (§3.8), so its encounters keep their geometry. The baseline-versus-adaptive comparison is valid in calm air only.

## 7. Known limits

Data:

- **No airborne GNSS or cellular data from India.** The GNSS figures are receiver and system specifications (§2.1). The latency figures are from European and Malaysian trials, with only TRAI ground-level medians for India (§2.2). No source gives a GNSS error correlation time, which is why the position floor exists.
- **Wind at drone height is extrapolated.** The India mean wind comes from CEEW's 100 m resource maps, and the gusts from the MIL-F-8785C Dryden model, not from Indian measurements at 0–120 m (§2.3).
- **Drag coefficient from one small quad.** D = 0.544 s⁻¹ is measured on a 0.61 kg quadrotor (Faessler et al. 2018). Indian agricultural and delivery drones are 10–60 kg with different frontal areas. A larger D makes the same wind push harder, and nothing in the monitor would notice it until `wind` or `nis` fires.

Model and simulation:

- **The disturbance has no airspeed drag term.** The plant applies d = D (W + gust) and leaves out −D v. An uncancelled wind therefore accelerates a drone until the speed cap. This is what made the baseline's India encounters dissolve (§6.3). The adaptive runs cancel the wind with feedforward and are not affected in the same way, but the plant is not a faithful drag model.
- **Independent gusts.** Each drone gets its own gust process. The Dryden correlation length is about 270 m, so drones a few tens of metres apart really feel almost the same gust. This overstates the relative gust and so makes the margin conservative.
- **Constant GNSS bias in the nominal runs.** The bias is drawn once per drone per run and held. White noise is tested only as a beyond-limit case (B3). Correlated errors between drones (the same satellites, the same ionosphere) are not modelled. They would cancel in the relative position, so the margin is conservative on that count too, but the monitor cannot see them.
- **Full command echo.** The drone reports its whole command history with each telemetry message. A real link would acknowledge commands one at a time, and a lost acknowledgement would look like a lost command.
- **Lag is assumed, not measured.** τ_a = 0.25 s is the top of PX4's `MC_ROLL_TC` range. The ground station has no live measurement of it, and B6 shows that a doubled lag is not detected as lag (§6.2).

Method:

- **Probability per check, not a worst case.** The margin covers the tracker's error at K = 4.033 standard deviations (P_RISK = 10⁻³ per check per pair). Van Wijk et al. bound the error deterministically. This note does not. Over many checks and many pairs the chance that some check is exceeded is much higher than 10⁻³; whether a single exceeded check leads to a loss of separation was not measured.
- **The safe responses are not executed.** The prototype records alarms only. The beyond-limit runs therefore show that the monitor fires before any loss of separation, not that the proposed responses keep the drones apart.
- **Small sample.** 47 recoverable scenarios per in-limit condition and 8–16 runs per beyond-limit condition. Zero losses in the 160 monitor-silent in-limit runs puts the per-run loss rate below about 2 % at 95 % confidence (rule of three), nothing tighter.
- **In-model only.** The tracker's model is the plant's model (§6.1). A real drone's dynamics, gusts and GNSS errors will differ from both.
- **A margin that changes over time.** §3 bounds each cycle on its own. A backup-filter guarantee also needs the next cycle to stay feasible while d_eff grows, and the margin does jump (one RTK calm run went from a median of 1.48 m to a peak of 7.94 m). DU_NOW covers only the displacement of a lost command within its own 100 ms, and relies on the next cycle counting the command's velocity error. No recursive-feasibility argument is given, and the cycles where the margin rose faster than the barrier can absorb were not counted.
- **EPS_LAG is a bound for one command switch** (§3.5). The backup switches command more than once; the bound for several switches is not derived.
- **The covariance-at-T step** (§3.2) assumes the predicted position covariance grows with the horizon. A negative position–velocity cross-covariance could make it shrink for a while; the maximum over the horizon was not taken.
- **The NIS self-check uses white GNSS noise** (`check.check_nis`), while the nominal runs use a steady bias, so the 5.6–6.4 mean-NIS result describes the tracker, not the runs.
- **a_phys per axis.** The 45° tilt limit is applied to each axis separately, including vertical. The combined demand on the airframe is not limited.
- **No code version in the result rows.** The shards were resumed after the harness speed-cap fix (DECISIONS). The fix changes only the recoverability check, not the episodes, but the rows carry no git hash to prove which code wrote them.
- **`EPS_LIN` is measured, not proven** (PR #12).

## 8. For the founder

These need a decision; none was changed in this work.

1. **Confirm P_RISK = 10⁻³ per check.** It is the one free number. It sets K, the NIS bound and the single-loss rule (§3.6). It is tied to the 3GPP 99.9 % C2 reliability, but that is a link figure, not a safety target. A safety target per flight hour would set it properly.
2. **The India mean wind sits at the envelope's edge.** At 4.17 m/s the `wind` alarm fires in 58 % of the in-limit India runs (§6.1), because the wind limit (about 6.3 m/s including uncertainty) leaves little room for gusts. Either the envelope accepts that India operations will often be outside it, or the drones need more acceleration authority than the 45° tilt assumed here (a_phys = 6.93 m/s²), or U_MAX must drop. Each of these changes a contract or a safety threshold.
3. **A slower autopilot than modelled is not detected (B6).** With the actuator lag doubled, the only alarms were the background `wind` alarms of the India wind (11 of 16 runs) and one `margin`. Nothing measures the lag itself, and B6 was run only under SBAS, whose large margin hid the extra error (§6.2). A per-drone lag identification (from the command echo and the measured acceleration) would be the fix; it is a new monitor and so a safety-claim change.
4. **The SPS figures use the 3/5 m expectation, not the 8/13 m standard.** The 2020 SPS Performance Standard commits to a global-average 95 % accuracy of ≤ 8 m horizontal and ≤ 13 m vertical, while its introduction says well-designed receivers achieved 3 / 5 m with the 2018 signal. The prototype uses the 3 / 5 m figure for the SPS condition and the worst-site 15 / 33 m for B7. With the standard's figure the position term of the margin would be about 2.6 times larger (8/3 and 13/5). Which figure the claim rests on is your call.
5. **SBAS and SPS margins are 13–16 m.** Separation at these margins is real but costly: the `wall` alarm fires in-limit because the bigger keep-out pushes drones toward the box edges (§6.1). RTK brings the margin to 1.4–2.7 m. Requiring RTK (or a similar correction) in dense airspace is a product decision.
6. **The baseline comparison in wind is not valid** (§6.3). Main as configured today (0.5 m, `lag_s` = 0, no delay compensation) had losses of separation in calm air under SBAS and SPS errors (37 of 47 runs each) and under RTK (7 of 47). These losses come from the delay, the lag and the GNSS error together, not from the margin alone; main with `lag_s` = 0.25 s was not run. The India rows of the baseline show no losses only because the simulated wind carried the drones out of the encounter.
7. **The 2 s entry period changes how the monitor is scored.** It was added after the first runs showed every alarm firing at t = 0, while the tracker had not converged. Alarms before 2 s are not counted, for criterion 2 or for "loss with the monitor silent". It is an evaluation choice and needs your sign-off.
8. **Other modelling choices besides P_RISK.** These were made by the research lane and logged in DECISIONS; each moves the margin or the monitor:
   - the NIS window of 10 messages and the 2 s entry period;
   - the 99.9 % latency quantile for the playout delay;
   - the floors on q (10⁻⁶) and P_dd(0) (10⁻⁴);
   - τ_a = 0.25 s, the top of PX4's range;
   - D = 0.544 s⁻¹, the largest coefficient in Faessler et al.;
   - the light turbulence class for the India wind;
   - the CEEW 15 km/h figure at 100 m, read through a secondary source;
   - the 3 / 5 m SPS figure (item 4).
9. **Trusting the receiver's reported accuracy** is an envelope assumption without a monitor (§5). A receiver whose real error is a larger steady bias than it reports is not detected. A test case with a 5σ bias under nominal reported σ was not run.
