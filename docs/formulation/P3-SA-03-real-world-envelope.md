# P3-SA-03 — Adaptive safety margin from measured real-world conditions

## Summary

Main's filter keeps every pair at least `5 m + margin_m` apart. Today `margin_m` is a fixed 0.5 m, and the filter acts on states that it assumes are exact and current. This note replaces that fixed number with a margin computed every 100 ms from live quantities: the tracker's covariance, the predicted velocities, the wind estimated through each drone's drag, the age of the unconfirmed commands and the actuator lag. The margin may rise only as fast as the barrier can absorb. An envelope monitor states, for each condition, the limit inside which the margin is valid and the safe response outside it. Every physical input is either a cited published value (§2) or measured live. The founder's decisions of 2026-10-24 (1–7, and four on the interim report) are applied; the remaining free choices are listed in §8.

Findings (details in §6; all in simulation, in-model):

1. **Inside the limits, no loss of separation.** The adaptive prototype kept every pair apart in 282 recoverable runs (three GNSS grades, calm and India wind) and in 141 runs that start from a takeoff track. Main as configured today lost separation in 99 of 141 calm runs and 23 of 141 India-wind runs on the same plant.
2. **The margin follows the GNSS grade.** With the guaranteed accuracy figures the median margin is 1.3–1.4 m with RTK, 23 m with SBAS (GAGAN) and 39 m with GPS alone. On a reference corridor that is about 12 400, 280 and 66 drones per km. Under the protection-level rule (§3.10) the RTK figure is not available to a dense corridor until an integrity-capable RTK product is verified.
3. **The monitor caught every fault in B1–B7 before any loss**, including a doubled actuator lag, which the lag monitor now finds (16 of 16 runs under SBAS, 14 of 16 under RTK). The alarm often came less than a second before the encounter, and the safe responses were not executed.
4. **A receiver that under-reports its error five times (B8) caused a silent loss of separation** in one of 16 SBAS runs, and a loss 4.5 s after an `absorb` alarm in another. No monitor in the design sees a steady bias. It is stated as a limit of single-receiver GNSS without integrity (§3.10).
5. **Two monitor results weaken the per-cycle claim.** `headroom` fires in almost every run because it is isotropic, and the rate limit left the margin up to 16 m short of the required one in some GPS cycles (`absorb`). Neither led to a loss here.
6. **Main's RTK baseline rose from 7 to 17 losses** after the plant gained drag. §6.6 gives the diagnostic.

| Field | Value |
|---|---|
| Task | P3-SA-03: real-world envelope, adaptive margin, tracker, monitor, prototype |
| Status | Research prototype; nothing in `src/` or `tests/` changes |
| Date | 2026-10-24 |
| Builds on | P3-SA-01 (literature), P2-SA-02 (backup filter), PR #12 (`EPS_LIN`, latch) |
| Code | `research/adaptive_margin/` (`envelope.py`, `tracker.py`, `margin.py`, `sim.py`, `run.py`, `check.py`) |
| Results | `research/adaptive_margin/results/runs2-*.jsonl` (commit `486f3b4bd66c`), `research/adaptive_margin/results/summary.md`; interim report `docs/results/p3-sa-03-interim-2026-10-24.md` |

## 1. Problem

Main's `apply_filter` certifies separation for states that are exact, current and flown exactly as commanded. A real ground station sees none of that:

- Positions come from GNSS with metre-level error, and the error may be a slowly varying bias, not white noise.
- Telemetry arrives 20–60 ms late over cellular links, and a few messages are lost.
- Commands reach the drone late, and a few are lost. A lost command leaves the previous one in force.
- Wind pushes each drone, and the push depends on the drone's own airspeed: the drag is d = D (w − v), with w the wind velocity and v the drone's velocity (founder decision 2a). Uncancelled wind therefore drifts a drone towards the wind speed, not to the speed cap.
- The autopilot follows an acceleration command through an attitude loop with a lag of a few hundred milliseconds, and the lag of a given drone is not known exactly.
- Each drone type has its own top speed, tilt limit and wind limit.

The fixed 0.5 m does not depend on any of these. It is too small when they are large and wasteful when they are small. The task is a margin that follows measured conditions, together with a monitor that says when the conditions leave the range where the margin can be trusted.

## 2. Real-world data

Each figure is marked **primary** (read in the source itself) or **secondary** (read through a summary, mirror or search result). Every quote below was checked against its source on 2026-10-24 (founder decision 7); the check column says what was found. Per-axis standard deviations σ are converted from the published statistic with the Gaussian factors in `envelope.py`:

| Published statistic | Conversion |
|---|---|
| CEP (2-D, 50 %) | σ = CEP / 1.1774 |
| R95 (2-D, 95 %) | σ = R95 / 2.4477 |
| 1-D 95 % | σ = value / 1.960 |
| 1-D 50 % (R50 vertical, F9P velocity) | σ = value / 0.6745 |

### 2.1 GNSS accuracy

**Rule (founder decision 1).** The claim rests on the figure each augmentation level *guarantees*: the GPS SPS standard for GPS alone, the GAGAN service requirement for SBAS, the receiver specification for RTK. Measured or typical performance is quoted for comparison only.

| Set | Source | Guaranteed figure used | σ_h (m) | σ_v (m) | σ_vel (m/s) | Check |
|---|---|---|---|---|---|---|
| `RTK` | u-blox ZED-F9P-02B data sheet, UBX-21023276 R03, Tables 1 and 3 | RTK 0.01 m + 1 ppm CEP horizontal, 0.01 m + 1 ppm R50 vertical, taken at a 10 km baseline (0.02 m); velocity 0.05 m/s ("50 % at 30 m/s for dynamic operation") | 0.017 | 0.030 | 0.074 | primary, verified |
| `SBAS` | ISRO, GAGAN status, ICG-11, 7 Nov 2016 ([unoosa.org/pdf/icg/2016/icg11/05.pdf](https://www.unoosa.org/pdf/icg/2016/icg11/05.pdf)), "Precision Approach – APV 1.0 Service" | requirement < 7.6 m 95 % bound horizontal and vertical (measured: ~3.0 m and ~4.0 m) | 3.105 | 3.878 | 0.102 | primary, verified |
| `SPS` | GPS SPS Performance Standard, 5th ed., April 2020 ([gps.gov](https://www.gps.gov/sites/default/files/2025-07/2020-SPS-performance-standard.pdf)), Table 3.8-3 | global average ≤ 8 m horizontal, ≤ 13 m vertical, 95 %; velocity 0.2 m/s 95 % any axis | 3.268 | 6.633 | 0.102 | primary, verified |
| `SPS_WORST` | same, Table 3.8-3 | worst site: 15 m horizontal, 33 m vertical, 95 % | 6.128 | 16.837 | 0.102 | primary, verified |

The SPS standard's introduction also says that "with current (2018) Signal-in-Space (SIS) accuracy, well-designed GPS receivers have been achieving horizontal accuracy of 3 meters or better and vertical accuracy of 5 meters or better 95% of the time" (verified). The previous version of this note used that expectation; under the founder's rule it is now comparison only. The GAGAN source gives no velocity figure, so `SBAS` takes the SPS standard's 0.2 m/s.

For comparison, the DJI FlyCart 30 specification gives hovering accuracy of ±10 cm horizontal and vertical with RTK, and ±60 cm horizontal / ±30 cm vertical without it (primary, verified, [dji.com](https://www.dji.com/global/flycart-30/specs)). That is control accuracy, not position accuracy, so it is not used.

**Gap.** No source gives the error's correlation time. The simulator draws a constant bias per episode, and the tracker gets no credit for averaging position (§4.3).

### 2.2 Cellular link: latency and loss

| Source | Where | Figure | Check |
|---|---|---|---|
| Barrios-Muñoz et al., "Evaluating 5G Networks for U-Space Applications", arXiv [2411.09666](https://arxiv.org/abs/2411.09666) v2 (7 Apr 2025), Table III | Benidorm, Spain, dense urban; Qualipoc phone on a drone; 4G and 5G | RTT median 54.0 ms (ground UE), 55.90 / 58.03 / 59.91 ms at 20 / 40 / 60 m; mean 62.09 / 58.65 / 59.98 / 66.41 ms; 5th percentile 42.74 / 53.57 / 54.50 / 53.42 ms | primary, verified |
| Zulkifley, Behjati, Nordin, Zakaria 2021, "Mobile Network Performance and Technical Feasibility of LTE-Powered Unmanned Aerial Vehicle", *Sensors* 21(8) 2848, [doi:10.3390/s21082848](https://doi.org/10.3390/s21082848), Table 2 | Malaysia, LTE | average delay 25.4 ms (0 m), 24.73 ms (40 m), 23.44 ms (80 m), 36.63 ms (170 m); abstract: "latency increased up to 94 ms" | primary (Europe PMC full text PMC8073316), verified |
| TRAI press release 24/2026 ([trai.gov.in](https://trai.gov.in/sites/default/files/2026-02/PR_No.24of2026.pdf)) | Delhi, ground drive test, 6–9 Jan 2026 | "Latency (Overall): Airtel, MTNL, RJIL and VIL 50th percentile latency is 20.80 ms, 21.65 ms, 20.62 ms, 38.98 ms"; MTNL is 3G, the others 4G/5G | primary, verified; the earlier "4G/5G" label was wrong for MTNL |
| Yang et al. 2018, "A Telecom Perspective on the Internet of Drones: From LTE-Advanced to 5G", arXiv [1803.11048](https://arxiv.org/abs/1803.11048) | commercial LTE-Advanced network, 2.6 GHz; 32-byte ping from the drone | "most of the latency data samples are concentrated between 200 ms and 300 ms at the height of 50 m or 100 m. When the drones fly up to 300 m, the latency data samples are concentrated between 400 ms and 500 ms" | primary, verified (was secondary) |
| 3GPP TS 22.125 Rel-19, Table 7.2-1 ([itecspec.com](https://www.itecspec.com/3gpp/22.125/s/7.2)) | requirement | command and control: 99.9 % reliability; end-to-end latency 40 ms (direct stick steering), 1 s (steer to waypoints), 5 s (automatic flight on UTM) | secondary (mirror of the spec; the mirror shows Rel-19 but no V19.x.y number), figures verified on the mirror |

**Model.** One-way latency is log-normal with median 59.91 / 2 = 29.96 ms and σ_ln = √(2 ln(66.41 / 59.91)) = 0.454, fitted to the 60 m row of Barrios-Muñoz (median and mean). The link is assumed symmetric. The 99.9 % quantile is 122 ms, so commands play out at a fixed P = 130 ms (13 ticks of 10 ms), and a command that arrives later is dropped. Loss per message is 10⁻³ (3GPP), independent.

**Gaps.**

- No airborne cellular measurement from India was found. TRAI's figures are ground drive tests. They are lower than the Spanish airborne figures, so the Spanish figures are used.
- **The model is fitted to a 2024 dense-urban 5G network.** Yang et al.'s 2018 LTE-Advanced trial measured round trips of 200–300 ms at 50–100 m. On such a network most commands would arrive after the 130 ms playout and be dropped; the `link` alarm would fire throughout. The playout delay P must be set from the operating network's own measured latency, not from this note's figure (§7).

### 2.3 Wind at 0–120 m over India

| Source | Figure | Check |
|---|---|---|
| CEEW 2021, "Assessing and Planning for Variability in India's Wind Resource" ([PDF](https://www.ceew.in/sites/default/files/ceew-study-on-impact-of-climate-change-on-wind-power-generation-capacity-india.pdf)) | annual average wind speeds at 100 m, 1979–2020; "Annual average wind speeds have reduced at a rate of 0.88 kmph with every decade"; "We observed mean annual wind speeds of over 15 kmph only in Gujarat and southern Tamil Nadu" | primary, verified (was secondary; the period is 1979–2020, not 1979–2019) |
| MIL-F-8785C low-altitude Dryden, via [MathWorks](https://www.mathworks.com/help/aeroblks/drydenwindturbulencemodelcontinuous.html) | σ_w = 0.1 W20; σ_u = σ_v = σ_w / (0.177 + 0.000823 h)^0.4; L_u = L_v = h / (0.177 + 0.000823 h)^1.2, L_w = h (h in ft); W20 "typically" 15 / 30 / 45 kt for light / moderate / severe | secondary (a reference implementation of the standard), verified |
| WMO squall definition, via Wikipedia ("Squall", WMO 1962) and the ANAC aviation glossary (WMO vocabulary, 1992) | rise of at least 8 m/s (16 kt) to at least 11 m/s (22 kt), lasting at least 1 min | secondary, verified in both; the original WMO text was not found |
| IS 875 Part 3 (2015), via [infralens.in](https://infralens.in/code/IS-875-Part-3-2015/clause/6-2) | basic wind speed V_b: "peak gust speed averaged over a short time duration of about 3 seconds", "at 10 m height above mean ground level in open terrain (Category 2)", 50-year return period; zones 33–55 m/s; height factor k2 at 100 m = 1.24 in terrain category 2 (1.20 in category 3) | secondary, verified; the earlier "1.20 in category 2" was wrong |

**Model.** The in-limit Indian case is a mean wind of 15 km/h = 4.17 m/s (the CEEW figure that only two regions exceed on average) in a random horizontal direction, with light Dryden turbulence at the encounter height of 115 m: σ_u = 1.03 m/s, σ_w = 0.77 m/s, L_u = 272 m, L_w = 115 m. Each drone's gust is a first-order Gauss–Markov process with time constant L / U (about 65 s horizontal and 28 s vertical). The beyond-limit squall uses an 11 m/s mean with moderate turbulence (σ_u = 2.06 m/s). IS 875's design gusts are structural extremes far outside any drone's envelope, so they are cited for scale only.

**Gaps.** No 100 m gust statistics for India were found (an ERA5 gust analysis would supply them). The 120 m ceiling comes from the brief; the DGCA rule text was not fetched.

### 2.4 Drones: registration data, drag, authority, lag

**Rule (founder decision 2b).** Each drone's wind limit is its declared maximum wind resistance, a registration field. Its drag coefficient and acceleration authority are derived from the same published data (§3.3), not taken from a research quadrotor.

| Type | Source (retrieved 2026-10-24) | Max wind resistance | Max tilt | Max horizontal speed | MTOW | Check |
|---|---|---|---|---|---|---|
| DJI Matrice 350 RTK | [enterprise.dji.com/matrice-350-rtk/specs](https://enterprise.dji.com/matrice-350-rtk/specs) | 12 m/s | 30° (25° in N mode with forward vision) | 23 m/s | 9.2 kg | primary, verified |
| DJI Matrice 30 | [enterprise.dji.com/matrice-30/specs](https://enterprise.dji.com/matrice-30/specs) | 12 m/s | 35° (25° in N mode with forward vision) | 23 m/s | 4.069 kg | primary, verified |
| DJI Mavic 3 Enterprise | [enterprise.dji.com/mavic-3-enterprise/specs](https://enterprise.dji.com/mavic-3-enterprise/specs) | 12 m/s, footnote: "during takeoff and landing" | 35° sport, 30° normal | 21 m/s forward (sport; 20 side, 19 back); 15 m/s normal | 1.05 kg | primary, verified |
| DJI FlyCart 30 | [dji.com/global/flycart-30/specs](https://www.dji.com/global/flycart-30/specs) | 12 m/s (zero altitude, windless, 30 kg payload) | 30° | 20 m/s (same conditions) | 95 kg | primary, verified |
| DJI Agras T40 | [dji.com/global/t40/specs](https://www.dji.com/global/t40/specs) | 6 m/s | not published | not published (obstacle sensing ≤ 7 m/s) | 90 kg spraying, 101 kg spreading | primary, verified; cannot register (no tilt or speed) |
| Garuda Aerospace Agri Kisan (GA-AD) | [garudaaerospace.com](https://www.garudaaerospace.com/products/precision-agriculture-drones/agri-kisan-drone) | not published | not published | rated speed 5 m/s | 24.56 kg gross; 8 kg payload | primary, verified; cannot register |

A type without all three of wind, tilt and speed cannot register in the prototype (`envelope.FLEET` holds the four DJI types). Neither Indian-market agricultural drone checked publishes them. Registration in a real service would have to require them.

Autopilot and dynamics references:

| Source | Figure | Check |
|---|---|---|
| PX4 parameter reference (saved page): `MPC_TILTMAX_AIR`, `COM_OF_LOSS_T`, `EKF2_ACC_NOISE` | maximum tilt in air 45° by default; offboard connection loss timeout 1.0 s by default; accelerometer noise 0.35 m/s² by default | primary, verified |
| PX4 `MC_ROLL_TC` (former parameter), via QGroundControl's `PX4TuningComponentCopter.qml` (mirrors of the QGC source) | roll time constant slider 0.15–0.25 s, step 0.01; one revision notes the parameters "seem to have disappeared from PX4 firmware" | secondary, verified on the mirrors; no longer in the current reference |
| Eschmann, Albani, Loianno 2024, arXiv [2404.07837](https://arxiv.org/abs/2404.07837) | Crazyflie 2.1 motor time constant T_m = 0.072 s (the manufacturer's step response gives 0.073 s) | primary, verified; a 27 g drone, so motor lag only |
| Faessler, Franchi, Scaramuzza 2018, RA-L 3(2) 620–626, [arXiv 1712.02402](https://arxiv.org/abs/1712.02402) | mass-normalised rotor drag d_x = 0.544 s⁻¹, d_y = 0.386 s⁻¹ (circle), 0.491 / 0.236 s⁻¹ (lemniscate); "The platform weights 610 g" | primary, verified; no longer used for D |

**Model.**

- **Speed.** The contract's 15 m/s is below every registered type's top speed. It is not changed here.
- **Authority and drag** are derived per type in §3.3: a_phys = g tan(tilt) and D = a_phys / v_max. For the fleet a_phys is 5.66 m/s² (30°) or 6.87 m/s² (35°), and D is 0.246–0.327 s⁻¹, about half of Faessler's 0.544 s⁻¹.
- **Actuator lag.** First order with τ_a = 0.25 s, the upper end of the former `MC_ROLL_TC` range. It is now checked live (§5, `lag`).

**Gaps.**

- No attitude-lag measurement exists for a delivery or agricultural drone of this size.
- No manufacturer publishes a drag coefficient or an acceleration limit; both are derived.
- The Mavic 3 Enterprise's 12 m/s is stated for takeoff and landing only. The prototype uses it as the en-route limit too.
- DJI's speed and wind figures are for zero altitude, still air and (FlyCart 30) a stated payload. Density at 120 m changes them by about 1 %; payload changes them more and is not modelled.
### 2.5 GNSS integrity: protection levels (founder, interim review)

The accuracy figures of §2.1 say how large the error usually is. They do not bound it, and a receiver that under-reports its error (B8) passes every check in §5. Aviation solved this with *integrity*: the receiver outputs a protection level (PL), a bound on its position error that holds with a stated probability, and raises an alert within a time to alert when it cannot. This section records what exists, from sources read on 2026-10-24.

| Source | What it states | Under which assumptions | Check |
|---|---|---|---|
| Kim, Han, Kim, Kim, Kee, Choi, Choi, Lee, "A Study of SBAS Position Domain Analysis Method: WAAS and EGNOS Performance Evaluation", JPNT 5(4), 203–211, 2016, [doi:10.11003/JPNT.2016.5.4.203](https://doi.org/10.11003/JPNT.2016.5.4.203) | The SBAS user computes HPL = K_H · d_major and VPL = K_V · d_up (Eq. 5), where d_major and d_up are the semi-major and vertical axes of the position error ellipse from the weighted least-squares geometry (Eq. 2–4) and "K_H and K_V … [are] given to 6.0 and 5.33, respectively in the precision approach (PA) mode". The range variance is σ²_range = σ²_flt + σ²_UIRE + σ²_air + σ²_tropo (Eq. 1): σ_flt and σ_UIRE come from the broadcast integrity information (UDRE, GIVE), σ_tropo from a model of elevation, and "σ²_air refers to a variance of multipath and noise of pseudorange measurement". The algorithm is in RTCA DO-229D, appendix J. Table 2 (ICAO SARPs 2006): APV-I needs HAL 40 m, VAL 50 m, time to alert 10 s, integrity 1–2 × 10⁻⁷ per approach (150 s). | The broadcast UDRE and GIVE bound the satellite and ionosphere errors; σ_air is a receiver-class model, not a measurement. | primary, verified |
| Blanch, Walter, Enge, Lee, Pervan, Rippl, Spletter, "Advanced RAIM User Algorithm Description: Integrity Support Message Processing, Fault Detection, Exclusion, and Protection Level Calculation", ION GNSS 2012 | An SBAS receiver computes "a Vertical Protection Level (VPL) and a Horizontal Protection Level (HPL) (which are 10⁻⁷ error bounds on the vertical and horizontal position errors respectively)". ARAIM computes protection levels in the receiver from several constellations, with no augmentation signal, using an Integrity Support Message (σ_URA, σ_URE, b_nom, P_sat, P_const), solution separation and a χ² test; nominal error variance C_int(i,i) = σ²_URA,i + σ²_tropo,i + σ²_user,i. It allows that "multiple simultaneous faults cannot be ruled out" and enumerates fault modes up to a probability threshold. Reference values include PHMI 10⁻⁷ and LPV-200 VAL 35 m. | Fault probabilities from the ISM; σ_user is again a receiver model. ARAIM is a user algorithm, not an operating service. | primary, pages 1–4 read |
| u-blox, ZED-F9P FW 1.00 HPG 1.30 Release Note, UBX-21047459 R01, 17 Dec 2021, §2.6 | "ZED-F9P reports a high-precision GNSS position protection level (HPG PL) with a Target Misleading Information Risk (TMIR) of 5 [%MI/epoch], in other words a 5% probability of having misleading information per epoch." It "currently supports only position protection level output", and the confidence level "has been validated for RTK operation with RTCM input correction streams." Product ZED-F9P-04B. | Validated by u-blox under its own operating conditions. | primary, verified |
| Swift Navigation, Skylark corrections and Starling positioning engine | Vendor pages state an ISO 26262 ASIL-B process, protection levels down to about 1 m and a target integrity risk down to 10⁻⁷ per hour. | Vendor claims; no independent validation was found, and the product summary PDF could not be read. | vendor-reported, not verified |
| Zhu, Marais, Bétaille, Berbineau, "GNSS Position Integrity in Urban Environments: A Review of Literature", IEEE T-ITS 2018 (HAL hal-01709519) | Aviation integrity assumes many visible satellites and one fault at a time. In urban areas, multipath and non-line-of-sight signals break both assumptions. | – | secondary (search summary); the paper itself could not be fetched |

**What follows for the margin.**

- **Only SBAS gives an integrity bound at the margin's risk today.** Its PL is a 10⁻⁷ bound against the faults the SBAS monitors (satellite clock and orbit, ionosphere). That is tighter than P_RISK = 10⁻³ per check. GAGAN's APV 1.0 service (§2.1) targets the ICAO APV-I row of Kim et al.'s Table 2.
- **The ZED-F9P's PL does not qualify.** A 5 % misleading-information risk per epoch is looser than P_RISK by a factor of 50. It is a 95 % bound, not an integrity bound in the aviation sense.
- **Integrity RTK exists only as vendor claims.** Swift's figures, if they held, would meet the rule. Nothing found checks them independently.
- **No PL covers local errors by measurement.** In both SBAS and ARAIM the receiver's own multipath and noise enter through a model (σ_air, σ_user). Near buildings at 0–120 m this is the error that dominates. Zhu et al. say the aviation assumptions do not hold there (secondary).

## 3. Adaptive margin

This section was rewritten after the Fable review of `af58833` (`review/adaptive-margin`) and the founder's decisions B1–B6 and N1 of 2026-10-24. Each change names the review item it answers.

### 3.1 What the margin must cover

The filter runs at 10 Hz on the ground. At cycle t it computes commands that the drones apply at t + P. For each pair it requires that the pair's backup manoeuvre (brake away along the line of sight) keeps separation ≥ d_eff = 5 m + m over the whole rollout. The margin m must cover the gap between that certificate and the true motion. The gap now has four sources:

1. the error of the state estimate, propagated open loop over the backup, along the directions in which the backup separates the pair (§3.2);
2. lost commands (§3.6);
3. motion between 100 ms samples (§3.7);
4. the filter's own linearisation, measured in PR #12 (§3.7).

The actuator lag, the drag and the airframe's thrust limit are no longer margin terms. They are flown inside the backup rollout (§3.5, review B4). Two rules concern the conditions rather than the size: each drone flies no faster than its cleared cruise speed (§3.3, review B3), and a scenario is admitted only if its states already satisfy the margin it needs (§3.9, review B2). A third rule concerns time: the margin may not rise faster than the barrier can absorb (§3.9).

### 3.2 The tube (van Wijk et al. 2024) and the risk per pair-hour

van Wijk, Coogan, Molnar, Majji and Hobbs ([arXiv 2409.07700](https://arxiv.org/abs/2409.07700), "Disturbance-Robust Backup Control Barrier Functions") bound the distance between the disturbed and nominal backup flows by a tube, ‖φ_b^d(τ, x) − φ_b^n(τ, x)‖ ≤ δ_max(τ) = (ξ / L_cl)(e^{L_cl τ} − 1) for a disturbance bounded by ξ (Lemma 2). They then tighten the safety function by ε_τ ≥ L_h δ_max(τ) along the backup (Lemma 1). For pair separation, h = ‖r‖ − d_eff has L_h = 1, so the margin is the tube's width.

Three changes make the tube adaptive and probabilistic:

- **Covariance in place of a fixed bound.** The deviation is the estimate error carried open loop through the drone's model (§4.1). The error covariance at a time τ after the action time t_a is S(τ) = Φ(τ) P(t_a) Φ(τ)ᵀ + Q(τ), with (Φ, Q) the exact discretisation of the linear model. P(t_a) replaces the initial error, and the wind uncertainty P_ww with its process noise q replaces ξ.
- **Along the lines of sight, not a ball (review B1).** The barrier is a function of the distance ‖r‖ only, so to first order an error δr changes it by n·δr, with n = r / ‖r‖ the unit line of sight. The relevant spread is therefore the standard deviation of the relative error along n, √(nᵀ S n) with S = S_i(T) + S_j(T), not the largest eigenvalue of S. The barrier is the minimum over the rollout, so the line of sight that matters may turn during the backup (a pair that escapes vertically ends up separated vertically). The term takes the largest spread over the current line of sight and every line of sight along the pair's backup rollout:

  gnss_ij = K1 · max_k √(n_kᵀ S n_k).

  This replaces K √λ_max(S), which charged the vertical GNSS error (larger than the horizontal one for every grade) to horizontal encounters. Known limit: the lines of sight come from the estimated rollout, not the true one; the difference is second order in the error.
- **Risk per pair-hour (review B6).** The founder set the target as a risk per pair-hour and asked for both 10⁻⁵ and 10⁻⁷. The margin is checked once per pair per 100 ms cycle, 36 000 times per pair-hour. By the union bound, a per-check risk q = target / 36 000 keeps the pair-hour risk at most the target, whatever the correlation between checks. The quantiles follow from q:
  - K1 = √χ²₁(1 − q) for a one-dimensional term (the GNSS term along n, the closing speed in T, the sampling term);
  - K2 = √χ²₂(1 − q) for a horizontal two-dimensional check (`wind`, `headroom` and the cleared speed of §3.3).

| target per pair-hour | q per check | K1 | K2 |
|---|---|---|---|
| 10⁻⁵ | 2.8 × 10⁻¹⁰ | 6.31 | 6.63 |
| 10⁻⁷ | 2.8 × 10⁻¹² | 6.99 | 7.30 |

The union bound is conservative: consecutive checks of one pair are strongly correlated, so the true pair-hour risk of the margin terms is lower than the target. Whether the bound is tight enough to be useful was not studied. The earlier rows at K = 4.033 (`runs2-*`, P_RISK = 10⁻³ per check, 3-D chi-square) are a per-check result only and are kept as a record.

The fault detectors (`nis`, `lag`, `link`) keep their level P_RISK = 10⁻³ per test. Their level sets a false-alarm rate, not a separation risk, so they are not tied to the pair-hour target.

### 3.3 Drag, authority, the cleared speed and the stopping time T

**Plant (founder decision 2a).** Each drone flies v' = a_act + D (w − v), with w the wind velocity, v the ground velocity and D the mass-normalised drag (1/s). The drag acts on the airspeed w − v, so a wind that is not cancelled carries a drone towards the wind's own velocity, not to the speed cap (`check.check_drag`: a constant thrust c settles the speed at w + c / D).

**Airframe from registration data (founder decision 2b).** No manufacturer publishes D or an acceleration limit, so both are derived from the declared figures of §2.4:

- At the maximum tilt θ_max the thrust holds the weight and leaves a horizontal acceleration a_phys = g tan θ_max.
- At the top speed v_max in still air that acceleration balances the drag, so D = a_phys / v_max.

| Type | θ_max | v_max (m/s) | a_phys (m/s²) | D (1/s) | wind limit (m/s) |
|---|---|---|---|---|---|
| DJI Matrice 350 RTK | 30° | 23 | 5.66 | 0.246 | 12 |
| DJI Matrice 30 | 35° | 23 | 6.87 | 0.299 | 12 |
| DJI Mavic 3 Enterprise | 35° | 21 | 6.87 | 0.327 | 12 |
| DJI FlyCart 30 | 30° | 20 | 5.66 | 0.283 | 12 |

Each drone in a run is one of these types, drawn at random (`envelope.FLEET`). Three properties of the derivation matter:

- **D is an upper bound when v_max is a software limit.** If the airframe could fly faster than v_max at full tilt, the true drag at v_max is less than a_phys, and the true D is smaller.
- **Linear drag overstates the drag below v_max.** If the real drag grows with the square of the airspeed, it equals a_phys at v_max and is smaller than D·|w − v| everywhere below. The cleared speed and `headroom` are therefore conservative. The tracker's mean model is not, but the simulation uses the same linear D, so this gap is not exercised (§7).
- **Vertical authority is assumed.** The spec pages give no climb acceleration. The thrust command is clipped to a horizontal disc of radius a_phys and to |c_z| ≤ a_phys (`sim._authority`). *Shortcut, recorded in the code:* revisit with thrust-to-weight data.

**Feedforward.** The filter plans in the contract's drag-free model (v' = u). The ground station sends the thrust c = u − D (ŵ − v̂), which cancels the estimated drag at the predicted state, clipped to the airframe (`saturation` alarm, §5).

**Available acceleration and the cleared cruise speed (review B3).** After cancelling the drag, an airframe has u_avail = a_phys − D |w − v| left for manoeuvring. Main's backup asks for up to u_max per component, and the stopping time below assumes that each drone can brake at u_max |v| / v_max on top of its drag. At airspeed at most W + v, with W the forecast mean wind plus K2 σ of its gusts, that holds when

  a_phys − D (W + v) ≥ u_max v / v_max,  that is  v ≤ v_clr = (a_phys − D W) / (D + u_max / v_max),

capped at the contract's 15 m/s. This is an operating rule: the station clears each drone to cruise no faster than v_clr for its type and the forecast (`sim.cleared_speed`). Every scenario of §6 is flown at the cleared speeds, scaled so that the drones meet at the same point and time as in the uncleared scenario (`sim.cleared`).

| Type | calm, either target | India wind, 10⁻⁵ | India wind, 10⁻⁷ |
|---|---|---|---|
| DJI Matrice 350 RTK | 11.8 | 6.2 | 5.8 |
| DJI Matrice 30 | 12.9 | 6.7 | 6.4 |
| DJI Mavic 3 Enterprise | 12.3 | 5.8 | 5.4 |
| DJI FlyCart 30 | 11.0 | 4.9 | 4.6 |

(m/s.) The India wind (4.17 m/s mean, light turbulence) roughly halves the cleared speed. The squall case B1 is cleared on the India forecast, so the squall exceeds what the clearance assumed; that is the case's point. The `headroom` alarm (§5) checks the same inequality live, from the estimated drag and its uncertainty, and it is now a safety alarm. Known limit: the filter itself may command a drone above v_clr during an evasive manoeuvre; `headroom` then reports it.

The backup rollout (§3.5) also flies each drone with its real thrust limit, so a manoeuvre the airframe cannot fly at its airspeed brakes slower in the certificate, as it does on the plant.

**Stopping time T (review B2).** T is the horizon over which the estimate error is propagated (§3.2) and the lost-command error is held (§3.6). It is now the worst case per pair, not a value that follows the current closing speed: the pair closes at up to 2 v_max plus K1 σ_vc (σ_vc from the two receivers' velocity σ) and brakes at the relative deceleration √2 u_max, which the cleared speed keeps available:

  T = (2 v_max + K1 σ_vc) / (√2 u_max) + τ_a.

With RTK velocity accuracy T ≈ 6.3 s at either target. The earlier T, solved from the current closing speed with a fixed-point iteration on the drag uncertainty, could shrink as a pair approached; the reviewer showed that a constant T removes a path by which the margin fell just before an encounter. The `authority` alarm, which flagged the iteration's failure, is removed with it.

### 3.4 Prediction through the delay (Singletary, Chen and Ames 2020)

Singletary, Chen and Ames ([arXiv 2005.06418](https://arxiv.org/abs/2005.06418), "Control Barrier Functions for Sampled-Data Systems with Input Delays") handle an input delay by predicting the state at the moment the new input takes effect, from the buffer of inputs already sent, and enforcing the barrier condition there (Algorithm 1). Corollary 1 keeps safety when the prediction is uncertain, provided the invariant set is robust to that uncertainty.

Here:

- The buffer is the command log of each drone (§4.2). The prediction runs from the latest measurement to the playout tick t + P, and returns the actuator output a0 at that tick as well as the position and velocity.
- The robustness of Corollary 1 is the GNSS term of §3.2, computed with the position covariance floored at the receiver's value (§4.3).
- Algorithm 1 assumes that every buffered input reaches the plant. A lost command breaks that assumption, and §3.6 bounds its effect.

### 3.5 Actuator lag inside the backup rollout (review B4)

The plant follows the thrust command through a first-order lag, a' = (c − a) / τ_a, with τ_a the declared lag (0.25 s, §2.4). The first version held the actuator output for τ_a after playout (an equivalent delay) and added a bound EPS_LAG = 0.758 m for the lag-minus-delay error. The reviewer found that bound valid for one command switch only, while the backup switches many times.

The lag is now simulated where it matters: inside the backup rollout that the barrier evaluates (`rollout.py`). For each pair, each drone flies main's evasive manoeuvre (main's own `_limits_kernel` and `_evasive_kernel`, the same latch handling and the same stop rule) through the chain

  c = clip(u − D (ŵ − v)),  a' = (c − a) / τ_a,  v' = a + D (ŵ − v),  p' = v,

starting from the predicted actuator output a0, with the drone's own D, a_phys and estimated wind. Each 100 ms step is exact (matrix exponential, c and ŵ held over the step; `check.check_step_matrices`). The QP row's sensitivity is the position response at the critical step to a unit change of the first step's acceleration through the lag, in place of main's (k* + 0.5) dt².

`rollout.backup_constraints` replaces main's `_backup_constraints` while the adaptive mode runs; the baseline modes run main's own rollout unchanged. `src/` is not edited: `apply_filter` looks the function up as a module global, and the prototype rebinds it. The lagged rollout flies a shorter escape than main's for the same states (`check.check_lagged_rollout`), so the same margin now demands more of the current state.

EPS_LAG and DU_NOW (the cover for a command sent this cycle) are removed. No separate lag term remains, so the review's "one shared Δc bound for any remaining term" has no term to apply to. The only fixed term left is EPS_LIN (§3.7).

### 3.6 Lost commands

The 3GPP reliability of 99.9 % puts two consecutive losses of one drone's commands at 10⁻⁶. The margin covers one lost command per drone, and the monitor raises `link` at a streak of two.

A lost command leaves its predecessor in force for one 100 ms segment. Both lie in the airframe's thrust set (a horizontal disc of radius a_phys and |c_z| ≤ a_phys), so they differ by at most dc_max = 2√2 a_phys. The resulting velocity error, at most 0.1 dc_max, is held open loop from the start of the segment to the end of the horizon. The lag only delays the error, so the bound holds through it.

**Constant per drone (review B2).** The term no longer looks at which commands are still unconfirmed. It assumes the worst: the lost segment started as early as the station's knowledge allows. The station knows the applied commands up to the latest telemetry it has. With one message lost and the next still in flight (up to P), that knowledge is at most AGE_S = 2 P + 2 cycles = 0.46 s older than the action time. So

  lost_i = 0.1 · dc_max,i · (AGE_S + T).

At T = 6.3 s this is 10.8 m for the 30° airframes and 13.2 m for the 35° ones, so 21.7–26.3 m per pair. Telemetry older than AGE_S raises `link`. This term now dominates the RTK margin (§6, §8).

### 3.7 Sampling and linearisation

- **Sampling.** The filter checks separation every 100 ms. Two drones passing in a straight line at relative speed V that are d apart at both samples come closest midway, at √(d² − (0.05 V)²). Keeping that above 5 m needs δ_sample(V) = √(5² + (0.05 V)²) − 5. V is the predicted relative speed plus K1 times its standard deviation from the trackers.
- **Linearisation.** The PR #12 ablation measured a gap of EPS_LIN = 0.137 m between the filter's linearised barrier and the true separation (`docs/results/pr12-final-report.md`). It is measured, not proven.

### 3.8 The pair margin and the filter margin

  m_ij = K1 · max_k √(n_kᵀ (S_i(T) + S_j(T)) n_k) + lost_i + lost_j + δ_sample(|v_rel| + K1 σ_vrel) + EPS_LIN

Main's filter takes one `margin_m` for all pairs. It therefore gets the largest pair margin, m_req. This is conservative for every other pair. Main's `FilterConfig` bounds `margin_m` at 48.57 m (the world-box bound at the default barrier offset). A larger margin is clamped and the monitor raises `margin`.

### 3.9 Admissibility and raising the margin (founder decision 4, review B2)

A margin computed afresh each cycle can jump: a widening covariance or a lost message raises m_req from one cycle to the next. Main's backup barrier is h = min over the backup rollout of ‖r‖ − (5 + m). If the margin rises by more than the current h, the states the drones are already in violate the new barrier, and main's argument that a safe command exists at the next cycle (the backup itself) no longer applies.

**Absorbable margin.** Each cycle computes, from the lagged rollout of §3.5 with the latched escape frames and the previous cycle's configuration (`margin.absorbable`),

  m* = min over pairs of (min over the rollout of ‖r‖) − 5.

**Admissibility at clearance (review B2).** At the first cycle after the 2 s track convergence (§4.4), the scenario is *admissible* if m* ≥ m_req, using the unclamped m_req. An admissible scenario starts with every pair's backup already outside the margin it needs. A scenario that is not admissible would need the margin to rise against the states, which the rate limit below cannot do safely; it is excluded from the in-limit set in every mode, and the count is reported (§6). In operation this is a clearance check: a station does not clear two drones onto routes whose initial separation is below 5 m + m_req plus what the rollout needs.

**Rate limit.** The filter applies

  m_app = min(m_req, max(m*, m_prev)).

A falling m_req is applied at once. A rising one is applied only up to m*, and never below the previous margin.

**Why the next cycle stays feasible (sketch).**

1. If m_req ≤ m_prev, the margin falls or stays. The barrier set at m_app contains the one at m_prev, so whatever was feasible remains feasible.
2. If m_req > m_prev and m* ≥ m_prev, then m_app ≤ m*. By the definition of m*, the backup from the current predicted states keeps every pair at least 5 + m_app apart over the rollout, so h ≥ 0 holds at the new margin. Main's backup-barrier construction then gives a feasible command this cycle (the backup), and the argument repeats at the next cycle.
3. If m* < m_prev, the states already violate the previous margin (an estimation error or a disturbance larger than covered). The margin is held at m_prev, not raised, and main's QP or its fallback handles the cycle as it would today.

**What is counted.** Whenever m_app < m_req the monitor raises `absorb`, and the shortfall is recorded. With the constant T and lost terms, m_req changes far less from cycle to cycle than before, so `absorb` now mostly marks a scenario that was admissible but tight. `absorb` is not counted as an alarm before a loss of separation anywhere in §6 (review B2): it says the margin in force is short, not that a fault is present.

**Caveats.**

- m* uses the previous margin's wall standoff and rollout horizon. The configuration at m_app can differ slightly; the difference is not bounded here.
- m* is computed on the predicted states, so feasibility holds for the estimate, which is the same footing as the rest of the filter's certificate.
- The latch is read, not updated. If main re-latches a pair in the same cycle, its rollout differs from the one m* used.
- Anticipation (raising the margin early when a rise is foreseeable) was not implemented.

### 3.10 Integrity, protection levels and the remaining design changes (review B5)

**SBAS does not qualify for the margin's time to alert.** An integrity bound is useful to the margin only if the receiver alerts before the bound can be exceeded for the length of the horizon over which the margin relies on it, so the time to alert must be shorter than T. SBAS APV-I specifies a time to alert of 10 s (Kim et al. 2016, Table 2, ICAO SARPs). T is about 6.3 s (§3.3). A 10 s alert can arrive after the backup has run its course, so SBAS APV-I does not meet the rule. The previous version of this section said SBAS qualified; that was wrong.

**No positioning grade qualifies for dense corridors today.** SBAS fails the time to alert. The ZED-F9P's protection level is a 5 % bound per epoch (§2.5). Integrity RTK exists only as vendor claims. A corridor whose lane spacing rests on a GNSS margin therefore has no positioning grade whose integrity was verified against this rule. This is the founder's open item 1 (§8).

**How a protection level would enter the margin.** A PL is stated with an integrity risk per approach or per hour, not per check. Two ways keep the budget honest:

- *Per-check translation.* If the PL's integrity risk is p_PL per exposure time t_PL, and the faults it covers are independent between exposures, the risk per 100 ms check is at most p_PL · (0.1 s / t_PL) when one exposure is at least one check long. For SBAS APV-I (2 × 10⁻⁷ per 150 s approach) that is about 1.3 × 10⁻¹⁰ per check, comparable to q at 10⁻⁵ per pair-hour. The derivation needs the independence the PL's own analysis assumes, and it is stated, not prototyped.
- *PL as a radius.* The pair's GNSS term becomes PL_i + PL_j along the line of sight, and the PL's own integrity risk per pair-hour (2 p_PL per pair, scaled to the hour) is added to the margin's risk budget, so that the margin terms keep target − 2 p_PL. This needs no independence assumption and is the rule this note proposes. It is not in the prototype.

Either way the position floor stays as the receiver's reported accuracy when no qualifying PL exists, and B8 applies.

**B8 is a stated limit.** A single receiver without integrity can report a σ five times smaller than its true error. A steady bias moves the fix and the track together, so the NIS cannot see it (§5). The design does not detect this fault and adds no bespoke bias detector (founder decision). It is a limit of single-receiver GNSS without integrity.

**Lag: declared upper bound until identified (documented, not prototyped).** The lag monitor cannot tell lags apart while the command is steady (§5). The founder's interim design: each drone declares an upper bound τ_max beside τ_a, and the backup rollout of §3.5 flies τ_max until the identifier reports the lag *consistent*, then τ_a. The identifier reports *consistent*, *mismatch* (the `lag` alarm) or *not identifiable* (the window's non-centrality λ = Σ‖Δy‖² / σ²_acc for the nearest alternative is below the value that gives the F test 90 % power). It is not prototyped because no source gives a τ_max for any fleet type (§2.4). With the lag now inside the rollout, the change would be one parameter of `rollout.CTX`.

## 4. Tracker

`tracker.py` keeps one Kalman filter per drone.

### 4.1 Model

The state is x = (p, v, w) per axis, with w the wind velocity at the drone:

  p' = v,  v' = a_act + D (w − v),  w' = white noise of intensity q,  a_act' = (c − a_act) / τ_a.

D is the drone's registered drag (§3.3). The applied command c is known (§4.2), so a_act is a known input and not a filter state. The mean is propagated with the plant's own 10 ms step (`kin_step`, including the speed cap). The covariance uses the exact (Φ, Q) of the linear (p, v, w) model (`transition`, checked against Van Loan's matrix exponential in `check.check_transition`); it ignores the speed cap.

The wind is identified from the residual between commanded and measured motion: the velocity innovation left after the known command is attributed to D (w − v).

### 4.2 Delayed and out-of-order data

Telemetry messages z = (p, v) carry their measurement time, the drone's measured acceleration (IMU) and the echo of the commands it applied. The tracker keeps the posterior at the time of the latest fused measurement. An older message is dropped. To act, the tracker predicts from that time to the playout tick through the known commands and the lag (§3.4). It returns the actuator output at that tick, from which the backup rollout starts (§3.5).

`CommandLog` holds the switches that the drone reports as applied (its echo, up to the measurement time) and, after that time, the commands sent and not yet confirmed. A planned command missing from the echo is counted as lost and extends the loss streak. *Shortcut:* the echo carries the whole switch history, which is free in simulation. A real link would acknowledge each command.

### 4.3 Tuning from the data, not by hand

- **Measurement noise R.** From the GNSS set of §2.1.
- **Wind noise and prior.** From the forecast mean wind U and the Dryden class: q = 2 σ_g² / τ_g per axis, the intensity whose stationary Gauss–Markov variance matches the gust's, and P_ww(0) = U² + σ_g², a wind of the forecast speed in an unknown direction. q is floored at 10⁻⁶ m²/s³ and P_ww(0) at 10⁻² m²/s² so that calm conditions stay positive definite.
- **Update.** Joseph form.
- **Consistency.** The innovation statistic (NIS, 6 degrees of freedom per message) is summed over 10 messages (1 s). `nis` is raised when the sum exceeds χ²₆₀(1 − P_RISK) = 99.6. On in-model data with white GNSS noise the mean NIS is within (5.6, 6.4) (`check.check_nis`).
- **Position floor.** The covariance used by the margin never shows a position σ below the receiver's. The sources give no error correlation time (§2.1), and a constant bias would not average out. The filter update itself uses the unfloored covariance.

### 4.4 Track convergence (founder decision 5)

**Rule.** No clearance before the track has converged. The wind prior at the start of a track is the full forecast speed in an unknown direction, so at the first cycle the wind uncertainty is large and the monitor's wind-based checks fire by construction. The monitor is read from WARMUP_S = 2 s after the track starts (20 telemetry messages, two NIS windows). Alarms raised at the first cycle are reported separately (`alarms_t0`, "wind at t0" in §6), so the effect of the prior is visible.

In operation the track starts at takeoff, long before an encounter. The takeoff runs (§6) model this: the track starts 15 s before the encounter run's t = 0, with the drone hovering for 3 s and then accelerating along its route at 2 m/s² to the scenario's cruise velocity.

## 5. Envelope monitor

The margin of §3 is valid only while its assumptions hold. `margin.monitor` checks them every cycle from the track-convergence time on (§4.4). The safe responses are proposals. The prototype records the alarms and does not act on them.

| Condition | Live quantity | Limit | Physical meaning of the limit | Safe response (proposed) |
|---|---|---|---|---|
| `wind` | horizontal ‖ŵ‖ + K2 σ (largest eigenvalue of the horizontal wind covariance at T) | ≤ the drone's declared max wind resistance (12 m/s for the fleet) | the wind, with its uncertainty, exceeds what the manufacturer says the drone can fly in | Stop new departures in the area. Drones in flight hold or land. |
| `headroom` (safety alarm) | u_avail = a_phys − ‖D (ŵ − v̂)‖ − K2 σ, horizontal, σ at T | ≥ u_max ‖v̂‖ / v_max | the thrust left after the drag no longer gives the braking that T assumes (§3.3); the drone flies above its cleared speed for the wind it meets | Slow the drone to its cleared speed for the measured wind; stop new departures in the area until it is back inside. |
| `saturation` | thrust command clipped by `_authority` | not clipped | the airframe cannot deliver the command | As for `wind`. |
| `lag` | F statistic of the lag identification (below) | ≤ F₁,₅₃(1 − P_RISK / 5) = 15.97 | the drone answers commands with a lag other than the declared one | Fly the drone's backup rollout at the largest lag in the grid (§3.10) and take the drone out of dense flows until it is re-identified. |
| `nis` | Σ of the last 10 NIS | ≤ χ²₆₀(0.999) = 99.6 | the GNSS error or the wind is outside the tracker's model (GNSS noise larger than reported, a jump in the fix, a gust front); a steady bias is not seen | Treat the drone's position as unknown: set its σ to the worst-site figure (`SPS_WORST`), widen the margin, take it out of the flow and land it. |
| `link` | age of the latest confirmed command; loss streak; age of the latest telemetry | ≤ 1.0 s (`COM_OF_LOSS_T`); streak < 2; ≤ AGE_S = 0.46 s | the drone is about to fall back to its own failsafe; or the single-loss bound of §3.6 is broken | Treat the drone as non-cooperative. Reserve its reachable set from the last estimate and move the others away. |
| `wall` | any world-box acceleration limit active | none active | the backup's A_REL holds only away from the walls | Keep traffic one stopping distance from the box edges, or plan with the reduced A_REL there. |
| `margin` | required margin m_req | ≤ 48.57 m (`FilterConfig` bound) | the filter cannot express the margin | Deconflict strategically: wider spacing or a ground hold. Not counted as a warning in §6: it marks a condition outside the limits, not a fault. |
| `absorb` | m_req − m_app (§3.9) | 0 | the margin in force is short of the one required this cycle | Hold new clearances in the area until the deficit is gone. Not counted as a warning in §6 (review B2). |

The `authority` alarm of the first version is removed with the fixed-point T it guarded (§3.3).

**Wind against the declared limit, and headroom apart (founder decision 2b, review B3).** The `wind` check compares the estimated wind with the registration figure, as the founder's rule asks. It does not by itself protect the margin's assumptions. `headroom` does: T assumes that each drone can brake at u_max |v| / v_max on top of cancelling its drag (§3.3), and `headroom` checks that the thrust left, u_avail with its K2 σ uncertainty, still gives that braking at the drone's estimated speed. It is the live form of the cleared-speed rule and is now a safety alarm: when it fires, the stopping time on which the margin rests is not guaranteed. The two are kept separate because they mean different things. `wind` says the drone is outside its manufacturer's envelope. `headroom` says the drone may be inside it but is flying too fast for the wind it meets. The check uses the magnitude of the drag, so it is conservative where the drag helps the braking.

**Lag identification (founder decision 3, review N1).** Each telemetry message carries the drone's IMU acceleration (noise 0.35 m/s², PX4's `EKF2_ACC_NOISE`). The regressor is a_meas + D v_meas, which is the actuator output plus D w. It is compared with the actuator output that the echoed thrust commands give through a first-order lag, for the declared τ_a and for 0.5, 0.75, 1.5, 2 and 3 times it (`LagMonitor`). Two errors of the first version are fixed (review N1):

- The regressor used the tracker's wind estimate, so a wind estimate still converging, or a wind changing over the window, entered the residual as a lag would. The wind is now not taken from the tracker. Over the last 20 samples (2 s) each axis gets a free offset and a free linear trend, which absorb D w for a wind that is constant or changes at a constant rate over the window.
- The echo reported the commanded thrust before the airframe's clip, so a saturated command looked like a slow lag. The drone now echoes the thrust it applies, after the clip (`sim.py`), including during its own braking failsafe.

An F test with 1 and 53 degrees of freedom asks whether any alternative fits better than the declared lag, at P_RISK split over the five alternatives (Bonferroni). The windows overlap, so the false-alarm rate per run is higher than P_RISK per test. Without command changes all candidates agree and the test cannot fire, so a drone flying a steady command is not checked. With the lag now flown inside the backup rollout (§3.5), a wrong lag makes the certificate wrong whenever the backup is the binding constraint, not only after a command change; the design answer, a declared upper-bound lag until the identifier reports the lag identified, is in §3.10 and not in the prototype. `check.check_lag_id` shows the identifier finds a doubled lag under a square-wave command and stays quiet at the declared one.

Conditions without a monitor:

- a drag coefficient larger than the registered D, or a drag that is not linear;
- correlated GNSS errors between drones;
- **a GNSS bias larger than the receiver reports.** The margin trusts the receiver's reported accuracy. A steady bias moves the fix and the tracker together, so the NIS cannot see it, whatever its size (B8, §6). This is a stated limit of single-receiver GNSS without integrity; a receiver protection level narrows it (§2.5, §3.10).

Each would need a measurement that the ground station does not have (§7).

## 6. Results

Setup (`sim.py`, `run.py`): six encounter families (head-on, crossing, climbing, rings of 4 and 6, random) with 8 seeds each, 22 s episodes with the encounter at t = 12 s, a 10 ms plant and a 100 ms ground-station cycle. The plant has linear drag D (w − v) with each drone's registered D, a first-order actuator lag and the tilt-authority clip (decision 2a, §3.3). One scenario of the 48 (random, seed 4) is not recoverable even for main on exact, current states: its drones are inside 5 m at the first tick. It is left out of the in-limit counts. The matrix has 1168 rows, all written at commit `486f3b4bd66c` with a clean tree (`git` and `dirty` in every row, decision 7). Per-run rows are in `research/adaptive_margin/results/runs2-*.jsonl`; the tables are `research/adaptive_margin/results/summary.md`.

- **Adaptive:** the prototype of §3–§5: tracker, prediction through the delay, adaptive margin with the rate limit, drag trim, monitor.
- **Baseline:** main as configured today: the latest raw measurement, the default `FilterConfig` (0.5 m margin, backup barrier), no delay compensation. On the drag plant the baseline adds the trim D v_route so that a drone can hold its route speed at all, and its command is clipped to the airframe's authority like every other command.
- **Baseline_lag:** the same with `lag_s` = 0.25 s (decision 7). It gave the same result as the baseline in 284 of 288 run pairs; the other 4 differ by at most 0.15 m in minimum separation, and every count in the tables is the same. With the backup barrier, main's `lag_s` only lengthens the rollout horizon, and the rollout already runs until the pair stops closing; the `lag_s` · closing-speed reserve applies to the `braking` barrier only. Main therefore budgets for almost no lag in either configuration. The tables show the baseline row only.

"Alarm" counts every alarm from the track-convergence time on (§4.4). "Wind at t0" counts runs whose first cycle raised the `wind` alarm, from the prior (decision 2c). "Alarm before LoS" leaves out a `headroom` alarm raised at warm-up (founder, interim review; `run._first_alarm`).

### 6.1 Inside the limits

| condition | mode | runs | LoS | runs with any alarm | runs with an alarm other than `headroom` | absorb: runs (cycles, largest deficit m) | margin p50 (m) | min sep (m) |
|---|---|---|---|---|---|---|---|---|
| RTK, calm | adaptive | 47 | **0** | 43 | 5 | 4 (22, 3.16) | 1.34 | 6.52 |
| RTK, calm | baseline | 47 | 17 | – | – | – | 0.5 | 1.23 |
| RTK, India wind | adaptive | 47 | **0** | 46 | 7 | 2 (6, 0.54) | 1.43 | 6.60 |
| RTK, India wind | baseline | 47 | 2 | – | – | – | 0.5 | 4.62 |
| SBAS, calm | adaptive | 47 | **0** | 43 | 25 | 13 (93, 4.89) | 23.26 | 9.68 |
| SBAS, calm | baseline | 47 | 39 | – | – | – | 0.5 | 0.28 |
| SBAS, India wind | adaptive | 47 | **0** | 46 | 28 | 13 (97, 3.14) | 23.28 | 9.69 |
| SBAS, India wind | baseline | 47 | 11 | – | – | – | 0.5 | 1.23 |
| GPS SPS, calm | adaptive | 47 | **0** | 45 | 36 | 17 (183, 16.37) | 38.95 | 9.68 |
| GPS SPS, calm | baseline | 47 | 43 | – | – | – | 0.5 | 0.09 |
| GPS SPS, India wind | adaptive | 47 | **0** | 46 | 40 | 18 (189, 14.04) | 38.95 | 9.68 |
| GPS SPS, India wind | baseline | 47 | 10 | – | – | – | 0.5 | 1.22 |

Alarms in the adaptive rows (runs): RTK calm `headroom` 43, `absorb` 4, `saturation` 3, `lag` 1; RTK India `headroom` 46, `lag` 6, `nis` 4, `absorb` 2, `wall` 1; SBAS calm `headroom` 43, `wall` 23, `absorb` 13, `saturation` 2, `lag` 1; SBAS India `headroom` 46, `wall` 27, `absorb` 13, `lag` 6, `saturation` 2, `nis` 1; SPS calm `headroom` 43, `wall` 34, `absorb` 17, `saturation` 6, `lag` 1; SPS India `headroom` 46, `wall` 40, `absorb` 18, `lag` 6, `saturation` 4, `nis` 1. Every India run raised `wind` at t0, from the prior (decision 2c); no in-limit run raised `wind` after warm-up. "min sep" is the smallest separation over all runs of the row. For SBAS and SPS it is set by the starting geometry (random, seed 0, whose drones start about 9.7 m apart), not by the margin.

**Pass criterion 1 is met, in-model:** no loss of separation in 282 adaptive runs under in-limit conditions. The strict evidence is thinner than in the first run. Only 13 of the 282 runs raised no alarm at all, because `headroom` fires in 43–46 of 47 runs of every condition. Leaving `headroom` aside, 141 runs raised no alarm (RTK 82 of 94, SBAS 41, SPS 18). Zero losses in 141 runs bounds the per-run loss rate below about 2 % at 95 % confidence. The tracker's model is the plant's model, so this is an in-model check (§7).

What the alarms mean:

- **`headroom` fires at warm-up in almost every run, in calm air as in wind.** It asks for the filter's u_max in every horizontal direction on top of the drag at the current velocity (§5). At the scenarios' cruise speeds the drag alone exceeds a_phys − u_max for most of the fleet (above 8.8 m/s for the M350). The check is isotropic and so conservative: the backup brakes along the line of sight, where drag helps. As it stands it carries almost no information in-limit, and the decision on it is the founder's (§8).
- **`absorb` fires where the margin is large.** The rate limit (§3.9) held the applied margin below the required one in 4 and 2 RTK runs (largest deficit 3.16 m), 13 SBAS runs (4.89 m) and 17–18 SPS runs (16.37 m). In those cycles the margin that the risk claim needs was not in force. No loss followed, but the claim did not hold in those cycles.
- **`wall` fires as the margin grows** (RTK 1, SBAS 50, SPS 74 of 94 runs). Keep-outs of 23–39 m push the drones towards the box edges, where the backup's A_REL is not guaranteed.
- **`wind` does not fire after warm-up.** With the founder's limit (the declared 12 m/s, decision 2b) the India mean wind of 4.17 m/s is well inside. The first run's 58 % `wind` rate came from the earlier limit on the uncancelled acceleration, which `headroom` now carries.
- **`lag` fires in 1 calm and 6 India runs per grade**, with the drone's lag exactly the declared one: false alarms. At P_RISK = 10⁻³ per cycle over about 200 cycles per run, some are expected. Why the India runs have more was not isolated; a wind estimate that drifts within the 2 s window is the likely cause, as on the takeoff tracks (§5).
- Main's QP was infeasible and used its fallback in 5 of the 282 adaptive runs (SPS 4, SBAS 1).

### 6.2 Outside the limits

All beyond-limit cases run with the India wind and under SBAS, except B7 (GPS worst site) and the `_rtk` variants. Two families (`headon2`, `ring4`) with 8 seeds each.

| case | what breaks | LoS | alarm before any LoS | case alarm: runs, first time (median) | other alarms |
|---|---|---|---|---|---|
| B1 squall, 11 m/s mean, moderate turbulence | wind | 0 | 16 | `wind` 16 at 2.0 s; `lag` 15 (2.2 s) | headroom 16, wall 4, saturation 2 |
| B2 gust front, +8 m/s on one drone at 9 s | wind model | 0 | 16 | `nis` 16, first alarm 9.6 s | absorb 14, wall 14, lag 10, saturation 9, wind 9 |
| B3 GNSS white noise at 5 σ | GNSS model | 0 | 16 | `nis` 16 at 2.0 s | absorb 9, wall 7, lag 2, saturation 2 |
| B4 downlink out 3 s at 9 s | link | 0 | 16 | `link` 16, 9.9 s | wall 9, absorb 2 |
| B5 uplink out 1.5 s at 9 s | link | 0 | 16 | `link` 16, 9.4 s; `nis` 16 | absorb 15, wall 14, saturation 12, lag 3 |
| B6 actuator lag 0.5 s (2×), SBAS | lag | 0 | 16 | `lag` 16, 10.5 s | nis 15, wall 13, absorb 6, saturation 2 |
| B6 actuator lag 0.5 s (2×), RTK | lag | 0 | 14 | `lag` 14, 11.5 s | nis 12 |
| B7 GPS worst site (15 / 33 m, 95 %) | GNSS level | 0 | 16 | `margin` 16 at 2.0 s | wall 16, lag 2 |
| B8 bias 5 σ, reported σ nominal, SBAS | receiver integrity | **2** | 6 | none specific; `absorb` 3, `wall` 6 | – |
| B8 bias 5 σ, reported σ nominal, RTK | receiver integrity | 0 | 0 | none | – |

`headroom` at warm-up fires in 14–16 runs of every case and is not counted as a warning. Time 2.0 s is the first cycle the monitor is read (§4.4).

**Pass criterion 2 is met for B1–B7.** Each fault raised the alarm meant for it in every run, except B6 under RTK (14 of 16). No case B1–B7 lost separation.

- **B6 is now caught by the lag monitor (decision 3).** In the first run nothing measured the lag. Under SBAS the `lag` alarm fired in all 16 runs, at 7.4–12.0 s (median 10.5 s). Under RTK it fired in 14 of 16, at 9.8–11.9 s (median 11.5 s). The encounter is at 12 s, so the alarm often came less than 1 s before it, and it was the margin, not the alarm, that kept these runs apart (smallest separation 17.6 m under SBAS, 7.93 m under RTK).
- **B6 RTK detection gap.** `headon2` seeds 2 and 3 had no `lag` alarm. A head-on pair flies an almost constant command until the encounter, and the lag monitor needs command changes to tell lags apart (§5). Both runs kept more than 9 m. The design answer, a declared upper-bound lag until the identifier reports the lag identified, is in §3.10 and not in these rows.
- **B8 under SBAS lost separation in 2 of 16 runs, one of them silent.** `ring4` seed 0 lost separation at 12.89 s (minimum 2.36 m), 4.5 s after an `absorb` alarm at 8.4 s. `ring4` seed 5 lost separation at 11.76 s (minimum 3.10 m) with no alarm except `headroom` at warm-up. A bias five times the reported σ moves the fix and the track together, so the NIS cannot see it (§5). The claim that a beyond-limit fault raises an alarm before a loss of separation does not hold for B8. Under the founder's decision this is a stated limit of single-receiver GNSS without integrity (§3.10), not a monitor to be added.
- **B8 under RTK** had no loss and no alarm: a 5 σ bias at RTK accuracy is a few centimetres, inside the margin.
- **B7 is caught by construction.** The ground station is told the worst-site σ, so `margin` fires because the honest σ makes the margin exceed 48.57 m. A receiver that reported nominal σ at that error is B8.
- **B1 now raises `wind` against the declared limit and `lag` at 2.2 s.** The actuator lag is the declared one, so this `lag` alarm is a false alarm, probably from the moderate turbulence; the cause was not isolated.

### 6.3 Takeoff tracks (decision 5)

The track starts 15 s before the run's t = 0, with the drone hovering for 3 s and then accelerating to its cruise velocity (§4.4). India wind, all 47 recoverable scenarios per grade.

| grade | LoS | margin p50 (m) | alarm in the last 5 s before the encounter | absorb: runs (cycles, largest deficit m) | other alarms (runs) |
|---|---|---|---|---|---|
| RTK | 0 | 4.22 | 4 | 2 (13, 1.96) | headroom 47, saturation 15, lag 11, nis 8, wall 6 |
| SBAS | 0 | 23.53 | 11 | 11 (78, 2.11) | headroom 46, wall 28, saturation 15, lag 8, nis 1 |
| GPS SPS | 0 | 39.13 | 17 | 20 (166, 4.02) | headroom 46, wall 43, saturation 18, lag 9, nis 1 |

No loss of separation. With a converged track, `headroom` fires at warm-up in only 3 of 141 runs; it fires once the drone reaches cruise speed (from 4.0 s on in the RTK runs), which is consistent with the cruise drag driving it, not by the prior. The RTK median margin is 4.22 m against 1.43 m for the same scenarios started at cruise; the cause was not isolated. `saturation` fires in 15–18 runs per grade. In 4 (RTK) to 17 (SPS) runs an alarm fired in the last 5 s before the encounter.

### 6.4 Corridor capacity per GNSS grade (decision 1)

Reference corridor 100 m wide and 30 m high, square lane grid at spacing s = 5 m + the median margin of the in-limit India runs.

| grade | margin p50 (m) | lane spacing s (m) | lanes | drones per km of corridor |
|---|---|---|---|---|
| RTK | 1.43 | 6.4 | 80 | 12 400 |
| SBAS (GAGAN) | 23.28 | 28.3 | 8 | 280 |
| GPS SPS | 38.95 | 44.0 | 3 | 66 |

These are geometric upper bounds from a median margin, not a capacity claim: the margin rises above its median in closing encounters, and the `wall` and `absorb` alarms show the larger margins are already costly. Under the protection-level rule of §3.10, the RTK row is not available to a corridor until an integrity-capable RTK product is verified (§8).

### 6.5 Changes from the earlier runs

**The first run (`f152ada`, the previous version of this section).** The founder's decisions 1–7 changed the plant, the GNSS figures and the monitor, so most numbers moved. Each change and its cause:

| quantity | first run | this run | cause |
|---|---|---|---|
| adaptive LoS, in-limit | 0 of 282 | 0 of 282 | – |
| margin p50, SBAS / SPS | 12.9 / 15.8 m | 23.3 / 39.0 m | decision 1: guaranteed 95 % figures (GAGAN 7.6 m, SPS 8 / 13 m) in place of measured or expected ones |
| margin p50, RTK calm / India | 1.40 / 2.72 m | 1.34 / 1.43 m | decision 2a: the wind enters through each drone's registered drag (0.25 s⁻¹ for the M350) in place of D = 0.544 s⁻¹ on a constant push, so the wind's uncertainty adds less to the predicted error |
| adaptive min sep, RTK India | 6.73 m | 6.60 m | follows the smaller margin; still above 5 m + the margin in force |
| `wind` alarm, India in-limit | 82 of 141 runs | 0 after warm-up | decision 2b: limit is the declared 12 m/s wind resistance; `headroom` carries the old acceleration check |
| `headroom` | – | 43–46 of 47 per condition | new check (decision 2b) |
| `wall`, SBAS / SPS | 23 / 29 of 94 | 50 / 74 of 94 | larger margins (decision 1) |
| baseline LoS, India wind | 0 / 0 / 0 | RTK 2, SBAS 11, SPS 10 | decision 2a removes the artefact of the first run's §6.3: the constant push carried every baseline drone to the 15 m/s cap downwind and dissolved the encounters. The baseline-versus-adaptive comparison in wind is now valid. |
| baseline LoS, calm, SBAS / SPS | 37 / 37 | 39 / 43 | decision 1 (larger GNSS errors) |
| baseline LoS, calm, RTK | 7 | 17 | the drag plant and its trim; diagnostic in §6.6 |
| B6 (lag 2×, SBAS) | no case alarm | `lag` 16 of 16 | decision 3, lag identification |
| B6 under RTK, B8 | not run | §6.2 | decisions 3 and 7 |

No adaptive safety metric got worse except the RTK India minimum separation (6.73 to 6.60 m), which follows from the smaller margin. The baseline got worse in every calm row, and each change has a cause above.

**The stopped run (`96c7db4`, 119 rows, `results/partial-96c7db4-*.jsonl`).** The founder stopped it so the matrix could run on main's faster filter (#22). All 119 rows reappear in this run with the same minimum separation and the same loss time, to the stored precision. The rebase changed no result in those rows.

### 6.6 Why main's RTK calm baseline went from 7 to 17 losses (founder, interim review item 3)

The diagnostic reruns the 47 recoverable RTK calm baseline scenarios with the baseline's plant and command changed one at a time, by text substitution in a copy of `sim.py` (the repository is not edited; the script is a scratch file and not committed).

| variant | what changes | LoS | min sep (m) | runs saturated |
|---|---|---|---|---|
| A, as in the matrix | drag plant, trim D v_route, authority clip | 17 | 1.23 | 20 |
| B, no drag | D = 0, so no trim; clip kept | 7 | 2.20 | 0 |
| C, trim from the measured velocity | trim D v_meas, which cancels the drag at the measured speed | 15 | 2.44 | 21 |
| D, no trim | drag plant, command u only | 0 | 8.87 | 0 |
| E, first run's plant | D = 0 and no clip, with this run's random streams | 7 | 2.20 | 0 |

Variant A reproduces the matrix row exactly.

- **The drag plant explains the rise.** Without drag (B) the baseline loses separation in 7 runs again, the first run's count. B and E give the same runs, so the authority clip never acts without drag. E also shows that this run's random streams alone do not change the count; the 7 scenarios differ from the first run's 7 because the streams differ.
- **The cost is authority, more than the direction of the trim.** To hold its route speed against drag a drone needs D v of thrust, up to 3.7 m/s² for the M350 at 15 m/s against its 5.66 m/s². The authority clip then cuts the filter's braking: 20 of 47 runs saturated, and 14 of the 17 losses are among them. Trimming against the measured velocity (C), which removes the push back towards route speed while the filter brakes, still loses 15. The trim along the route accounts for about 2 of the 10 added losses.
- **Without any trim (D) the encounters do not happen.** The drag slows every drone towards zero before the encounter (mean minimum separation 104 m), so D is not a usable baseline.

Main's filter assumes a double integrator with the full A_REL available for braking. On a plant where cruising costs a share of the thrust, that assumption fails in the cruise-heavy encounters (the rings and random families). The adaptive prototype kept every pair apart in the same scenarios. Its larger margin, its prediction through the delay and its drag-aware trim all differ from the baseline; which of them matters here was not isolated. This is a property of main's filter on a drag plant, not of the adaptive margin, and it relates to BL-06 and BL-07 (§8).

### 6.7 What the results do and do not show

- In-limit and in-model, the adaptive margin kept every pair apart under three GNSS grades, in calm and in the India wind, from a cold start and from a takeoff track.
- Every beyond-limit fault in B1–B7 raised its alarm before any loss, but the alarm often came less than a second before the encounter (B2, B4, B5, B6). The safe responses were not executed, so the runs do not show that acting on the alarm in time keeps drones apart.
- B8 shows a loss of separation with no warning when a receiver under-reports its error. This is the main open safety limit, and §3.10 narrows it only for receivers with a qualifying protection level.
- `headroom` fires in nearly every run, and `absorb` shows cycles where the margin in force was up to 16 m short of the required one under GPS. Neither caused a loss here; both weaken the claim that the margin holds every cycle.

## 7. Known limits

Data:

- **No airborne GNSS or cellular data from India.** The GNSS figures are receiver and system specifications (§2.1). The latency figures are from a Spanish and a Malaysian trial, with only TRAI ground-level medians for India (§2.2). No source gives a GNSS error correlation time, which is why the position floor exists.
- **The latency model comes from one modern network.** It is fitted to a 2024 dense-urban 5G and 4G trial (median RTT about 60 ms). A 2018 LTE-Advanced trial measured 200–300 ms at 50–100 m (Yang et al., §2.2). On such a network the 130 ms playout would drop most commands, and the `link` alarm would fire throughout. The playout delay has to be set from the operating network's measured latency.
- **Wind at drone height is extrapolated.** The India mean wind comes from CEEW's 100 m resource maps, and the gusts from the MIL-F-8785C Dryden model, not from Indian measurements at 0–120 m (§2.3).
- **Drag and authority are derived, not measured.** No manufacturer publishes either. D = a_phys / v_max assumes the top speed is drag-limited at full tilt and that drag is linear in airspeed (§3.3). Payload and air density change both and are not modelled. The Mavic 3 Enterprise's 12 m/s wind figure is stated for takeoff and landing only and is used en route.
- **Vertical authority is assumed** equal to the horizontal one (`sim._authority`).
- **Two of six types checked cannot register.** The DJI Agras T40 and the Garuda Agri Kisan publish no tilt angle, and the T40 no top speed (§2.4).

Model and simulation:

- **In-model drag.** The plant and the tracker share the same linear drag with the same registered D. A real drone's drag differs in coefficient and shape, and nothing in the monitor checks D directly; a larger D shows up only through `nis`, `headroom` or `wind`.
- **Independent gusts.** Each drone gets its own gust process. The Dryden correlation length is about 270 m, so drones a few tens of metres apart really feel almost the same gust. This overstates the relative gust and so makes the margin conservative.
- **Constant GNSS bias in the nominal runs.** The bias is drawn once per drone per run and held. White noise is tested only as a beyond-limit case (B3). Correlated errors between drones (the same satellites, the same ionosphere) are not modelled. They would cancel in the relative position, so the margin is conservative on that count too, but the monitor cannot see them.
- **Full command echo.** The drone reports its whole command history with each telemetry message. A real link would acknowledge commands one at a time, and a lost acknowledgement would look like a lost command.
- **The lag monitor sees only command changes.** A drone flying a steady command is not checked (§5). Its 20-sample window and its grid of five alternatives are design choices; a lag between grid points is detected only through its nearest neighbour.

Method:

- **Probability per check, not a worst case.** The margin covers the tracker's error at K = 4.033 standard deviations (P_RISK = 10⁻³ per check per pair). Van Wijk et al. bound the error deterministically. This note does not. Over many checks and many pairs the chance that some check is exceeded is much higher than 10⁻³; whether a single exceeded check leads to a loss of separation was not measured. P_RISK is a provisional placeholder until a target safety level per flight hour is set (§8).
- **The safe responses are not executed.** The prototype records alarms only. The beyond-limit runs therefore show whether the monitor fires before a loss of separation, not that the proposed responses keep the drones apart.
- **Small sample.** 47 recoverable scenarios per in-limit condition and 16 runs per beyond-limit condition. Zero losses in n runs bounds the per-run loss rate below about 3 / n at 95 % confidence (rule of three), nothing tighter.
- **In-model only.** The tracker's model is the plant's model (§6.1). A real drone's dynamics, gusts and GNSS errors will differ from both.
- **Recursive feasibility is sketched, not proven** (§3.9). The absorbable margin uses the previous configuration's wall standoff and horizon and a read-only latch. Anticipation of margin rises was not implemented.
- **EPS_LAG is a bound for one command switch** (§3.5). The backup switches command more than once; the bound for several switches is not derived.
- **The covariance-at-T step** (§3.2) assumes the predicted position covariance grows with the horizon. A negative position–velocity cross-covariance could make it shrink for a while; the maximum over the horizon was not taken.
- **The NIS self-check uses white GNSS noise** (`check.check_nis`), while the nominal runs use a steady bias, so the 5.6–6.4 mean-NIS result describes the tracker, not the runs.
- **`EPS_LIN` is measured, not proven** (PR #12).
- **The interim-review design changes are not prototyped** (§3.10). The protection-level rule and the declared upper-bound lag are design text only; every result in §6 uses the reported σ and the nominal τ_a.
- **"Alarm before LoS" leaves out the warm-up `headroom` alarm** (founder, interim review). Rows keep only the first time of each alarm, so a `headroom` alarm raised at warm-up and again later is not counted either (`run._first_alarm`).
- **Integrity sources are thin outside aviation.** The SBAS PL equations and the ICAO APV-I figures are primary. The ZED-F9P PL is primary but only 95 %. Integrity RTK rests on vendor claims, and the urban-integrity review was read only through a summary (§2.5).

## 8. For the founder

Decisions 1–7 and the four decisions on the interim report are applied above. These items remain open; none was changed in this work.

1. **RTK and the dense-corridor rule.** Under §3.10, a corridor whose spacing rests on the RTK margin needs a protection level at RTK accuracy with integrity risk at most P_RISK. None was verified: the ZED-F9P's PL is a 95 % bound, and integrity RTK exists only as vendor claims (§2.5). Until one is verified, the 12 400 drones per km of §6.4 are not available; SBAS gives 280. Options: commission an independent check of an integrity-RTK product, accept SBAS spacing in dense corridors, or define a different rule for RTK.
2. **Where τ_max comes from.** The declared upper-bound lag (§3.10) needs a figure per airframe. No source gives one for the fleet. At τ_max = 2 τ_a the lag term grows from 0.76 m to 3.0 m, which matters under RTK (margin 1.3–1.4 m) and not under SBAS. Whether the operator declares it, a type test measures it, or a fleet-wide default applies is your call. The rule is not prototyped.
3. **P_RISK per flight hour.** P_RISK = 10⁻³ per check is provisional (decision 6). Over many checks and pairs the chance that some check is exceeded is much higher. A target safety level per flight hour would set it.
4. **`headroom` fires in almost every run** (43–46 of 47 per in-limit condition, §6.1), because it asks for the filter's u_max in every horizontal direction on top of the cruise drag. Options: make it directional (along the line of sight, where the backup brakes), lower the cruise speed so the drag fits, or keep it and accept that it carries no in-limit information. The first changes a monitor threshold.
5. **`absorb` deficits up to 16 m under GPS SPS** (17–18 of 47 runs; up to 4.9 m under SBAS and 3.2 m under RTK). In those cycles the rate limit kept the margin below what the risk claim needs. No loss followed. Anticipating margin rises (§3.9) or keeping GPS-only drones out of encounters would close it; both are design changes.
6. **The lag alarm can come too late.** Under RTK the `lag` alarm missed 2 of 16 doubled-lag runs, and where it fired it came a median 0.5 s before the encounter (§6.2). The declared-τ_max rule (item 2) is the design answer.
7. **B8 is a silent loss of separation** under SBAS when the receiver under-reports its error five times (§6.2). It is stated as a limit (§3.10). Its operational consequence is that the margin's claim holds only for receivers whose reported accuracy can be trusted, which today means a qualifying protection level.
8. **Main's filter on the drag plant.** As configured, main lost separation in 17 of 47 RTK calm runs, 39 under SBAS and 43 under GPS SPS, and in 2–11 India-wind runs per grade (§6.1). The cause of the RTK rise is in §6.6. This concerns main's filter, not the adaptive margin, and connects to BL-06 and BL-07 (disturbance margin).
9. **Modelling choices still in force,** each logged in DECISIONS: the NIS window of 10 messages and the 2 s convergence time; the 99.9 % latency quantile for the playout delay; the floors on q and P_ww(0); τ_a = 0.25 s; drag and authority derived from top speed and tilt; the light turbulence class for the India wind; the lag monitor's 2 s window and five-point grid.
