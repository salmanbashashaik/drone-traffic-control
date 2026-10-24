# P3-SA-03 — Adaptive safety margin from measured real-world conditions

## Summary

Main's filter keeps every pair at least `5 m + margin_m` apart. Today `margin_m` is a fixed 0.5 m, and the filter acts on states that it assumes are exact and current. This note replaces that fixed number with a margin computed every 100 ms from live quantities. The terms are: the tracker's error along the lines of sight of each pair's backup manoeuvre, a worst-case stopping time per pair, a cover for one lost command per drone, and a backup rollout flown through each drone's actuator lag, drag and thrust limit. A tighter lost-command cover, carried through the drag and bounded by the filter's own command box, was evaluated and is parked: reported, not adopted (§8 item 1); the a5b5264 cover stands. The margin is sized for a risk per pair-hour, at 10⁻⁵ and at 10⁻⁷ (founder B6). Each drone flies no faster than a cleared cruise speed for its airframe and the forecast wind, and a scenario is cleared only if its states already satisfy the margin it needs. The margin may rise only as fast as the barrier can absorb. An envelope monitor states, for each condition, the limit inside which the margin is valid and the safe response outside it. Every physical input is either a cited published value (§2) or measured live. The founder's decisions of 2026-10-24 are applied: 1–7, four on the interim report, B1–B6 and N1 on the Fable review, and 3(a)–(c) on the Fable re-review. The remaining choices are listed in §8.

Findings (details in §6; all in simulation, in-model; 2336 rows at commit `e06d9414dd95`, compared with the previous matrix at `a5b5264`; the two targets give the same picture):

1. **No loss of separation in any admissible in-limit run, at either target.** 259 runs at 10⁻⁵ and 248 at 10⁻⁷ (three GNSS grades, calm and India wind), and 135 and 132 takeoff runs. On the same scenarios main as configured today lost separation in 110 and 107 runs.
2. **The tighter lost-command term cut the margin by 15–17 m.** Carried through the drag (re-review 3a) and bounded by the filter's box plus the drag trim (3b), the term is 7.5–7.7 m per pair, against 22.1–26.8 m. With RTK the median margin is 8.0 m in calm air and 18.1 / 19.2 m in the India wind (25 and 35–36 m before). SBAS now fits main's 48.57 m bound at both targets: median 39–44 m, requested at clearance 38–43 m. GPS SPS still requests 45–52 m and runs at the cap in most runs.
3. **The station's own commands never came near the new bound.** The largest difference between consecutive commands was 0.78 of the bound, and the new `dc` alarm fired in no run.
4. **The tighter term is reported, not adopted: with it the certificate fails for seconds in SBAS six-drone rings.** Main's backup certificate (every pair's predicted backup outside d_eff) was violated in 82 / 66 admissible in-limit runs, 480 / 410 cycles, by up to 1.94 m. Under the a5b5264 term it was 36 / 31 runs, 67 / 54 cycles, at most 0.06 m. The worst is SBAS calm ring6: 1.2–3.3 s around the encounter, 2–3 pairs at once. Main's QP also fell back in 6 admissible runs (0 before). Separating the causes (§6.1): PR #26's KKT-exact solver removes the fallbacks but not the violations. Freezing the margin at its clearance value removes the violations (at most 0.055 m). **The cause is the margin rising during the encounter, up to what the barrier can just absorb, so the fix belongs in the rate limit (§3.9).** No loss followed (smallest separation 31 m). Not tuned; the term is parked (§8 item 1).
5. **Capacity is zero in the 100 m × 30 m reference corridor in the India wind, at both targets.** One RTK layer needs 36–38 m of height, and a corridor that tall gives 3 lanes (about 120–130 drones per km). In calm air RTK fits 14 lanes, 1064 drones per km. The old per-check figures (12 400 / 280 / 66) stay withdrawn.
6. **The beyond-limit results did not change in kind.** No case lost separation. B1–B5 raised their alarm in every run. The doubled lag (B6) was found in 5–13 of 16 runs (8–10 before). B7 and B8 have no warning of their own, and B8 is still masked by margins wider than a 5 σ bias.

| Field | Value |
|---|---|
| Task | P3-SA-03: real-world envelope, adaptive margin, tracker, monitor, prototype |
| Status | Research prototype; nothing in `src/` or `tests/` changes |
| Date | 2026-10-24 |
| Builds on | P3-SA-01 (literature), P2-SA-02 (backup filter), PR #12 (`EPS_LIN`, latch) |
| Code | `research/adaptive_margin/` (`envelope.py`, `tracker.py`, `margin.py`, `rollout.py`, `sim.py`, `run.py`, `check.py`) |
| Results | `research/adaptive_margin/results/runs4-{0..3}.jsonl` (2336 rows, commit `e06d9414dd95`, clean tree), rerun with certificate diagnostics as `runs5-{0..3}.jsonl` (`9f3dcad8d0d8`) and, for the a5b5264 term, `runs5h-{0..3}.jsonl` (`f01961701893`); tables `results/summary.md` (`runs5`) and `summary-runs5h.md`; the cause diagnostics `diag-b1-solver.jsonl`, `diag-b1-frozen.jsonl`. Previous matrix: `runs3-{0..3}.jsonl` (`b6139175fdb3`, the a5b5264 results; `ideal_min_sep` re-recorded at `d602f592d04e`, R1) and `results/summary-runs3.md`. Older rows, kept as a record and not used: `runs2-*.jsonl` (K = 4.033, per check only), `partial-b375131-*.jsonl`. Reviews on `review/adaptive-margin`: `docs/results/p3-sa-03-fable-review-2026-10-24.md`, `p3-sa-03-fable-rereview-2026-10-24.md` |

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

T = 6.445 s with RTK velocity accuracy at 10⁻⁵ (6.459 s at 10⁻⁷) and 6.495 / 6.515 s with the SBAS and SPS velocity figure. The earlier T, solved from the current closing speed with a fixed-point iteration on the drag uncertainty, could shrink as a pair approached; the reviewer showed that a constant T removes a path by which the margin fell just before an encounter. The `authority` alarm, which flagged the iteration's failure, is removed with it.

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

A lost command leaves its predecessor in force for one 100 ms segment (PX4 hold-last). The station learns of the loss only from the drone's command echo, at most AGE_S = 2 P + 2 cycles = 0.46 s after the segment started (one message lost, the next still in flight; telemetry older than AGE_S raises `link`). If the backup must start now, the backup from the station's estimate carries the lost segment's error for the whole horizon. The term is constant per drone (review B2): it assumes the lost segment started AGE_S before the action time, whatever the commands still unconfirmed are.

**The first version (`a5b5264`, rows `runs3`).** The two commands differ by at most dc_max = 2√2 a_phys (both lie in the airframe's thrust set), and the velocity error 0.1 dc_max was held for AGE_S + T:

  lost_i = 0.1 · 2√2 a_phys,i · (AGE_S + T).

With T = 6.45–6.52 s (§3.3) this is 11.06 m for the 30° airframes and 13.41 m for the 35° ones at RTK and 10⁻⁵ (up to 11.17 / 13.55 m under SBAS and SPS at 10⁻⁷), 22.1–26.8 m per pair. It was the largest term of every RTK margin (§6). The Fable re-review found the bound correct under the model and loose in two places, and the founder accepted tightening both (DECISIONS, 2026-10-24).

**What exists (literature first).** Sources marked *secondary* were read through abstracts or search summaries only. This session could not fetch arXiv: the network policy blocks it.

| Source | What it proves | Under which assumptions | Use here |
|---|---|---|---|
| Singletary, Chen, Ames 2020, CDC, [arXiv 2005.06418](https://arxiv.org/abs/2005.06418) (Algorithm 1, Corollary 1; read in the Fable re-review) | Safety of a sampled-data CBF controller with an input delay: predict the state at the moment the input acts, from the buffer of sent inputs, and enforce the barrier there; robust to a bounded prediction error | Delay known and an integer number of periods; every input arrives; bounded prediction error | Prediction through the delay (§3.4). Loss is outside the paper: the lost-command term is this note's own extension |
| van Wijk, Coogan, Molnar, Majji, Hobbs 2024, L-CSS, [arXiv 2409.07700](https://arxiv.org/abs/2409.07700) (Lemmas 1, 2) | Backup CBF stays safe under a bounded disturbance if the safety function is tightened along the backup by the tube δ_max(τ) = (ξ / L_cl)(e^{L_cl τ} − 1) | Disturbance bounded by ξ; closed-loop backup flow Lipschitz with constant L_cl; terminal backup set | The lost segment is a bounded input disturbance of known area. Item (a) replaces the generic growth bound by the exact response of the linear lag-and-drag chain, which decays instead of growing |
| Quevedo and Nešić 2011, IEEE TAC 56(2):370–375, doi:10.1109/TAC.2010.2095950 (*secondary*) | Input-to-state stability of packetized predictive control: the controller sends a sequence of inputs, the actuator plays the buffer on dropouts | Bounded number of consecutive dropouts; constrained nonlinear plant with bounded disturbances. Stability, not a barrier | §8 option (b) (a command plan per message); not used in the prototype |
| Akbarzadeh, Ashoori, Mamduhi, Lavaei 2026, [arXiv 2606.24148](https://arxiv.org/abs/2606.24148) (*secondary*) | Packetized safety controllers from control barrier certificates (sum-of-squares) with a plant-side buffer; probabilistic safety over a finite horizon | Discrete-time polynomial stochastic systems; downlink losses Bernoulli; delayed uplink | The closest safety analogue of a command plan with loss; not the hold-last case here |
| Seidel, Anand, Allgöwer 2026, [arXiv 2601.00494](https://arxiv.org/abs/2601.00494) (*secondary*, abstract) | Barrier functions for weakly-hard dropout patterns, with hold or zero actuation during open-loop intervals | Losses bounded within a window ((m, k) constraints) | The honest loss model once the operating link's burst statistics are measured (§7) |
| Dean et al. 2020, CoRL (PMLR 155); Cosner et al. 2021, IROS, [arXiv 2104.14030](https://arxiv.org/abs/2104.14030) (*secondary*) | Measurement-robust CBFs: safety with a bounded state-estimate error, combined with backup sets (Cosner) | Bounded measurement error; Lipschitz barrier and dynamics | Item (c) below |
| Beger, Chen, Hirche 2026, [arXiv 2603.29792](https://arxiv.org/abs/2603.29792) (*secondary*) | Where to enforce a CBF in a delayed networked loop: a local filter on fresh measurements tolerates larger disturbances than predictive constraints in a remote MPC | Delayed network; remote MPC | Supports the onboard-layer direction (§8); not used here |

No source found bounds the effect of a single lost hold-last command on a backup certificate, or bounds the change between consecutive ground-station commands that carry a drag trim. Items (a) and (b) are this note's own, built on the van Wijk tube.

**(a) The error decays through the drag.** The plant is linear in the perturbation: a thrust error δc held for one segment drives the actuator through the lag and the velocity through the drag, δa' = (δc − δa) / τ_a and δv' = δa − D δv, whatever the wind and the state (the drag D (w − v) is linear in v, so the wind estimate and its uncertainty do not enter). The position error at time t after the segment starts is |δc| g(t), with g the position response of that chain to a unit pulse of 0.1 s, computed exactly by matrix exponential (`margin.pulse_gain`). For a positive pulse the velocity stays non-negative, so g is non-decreasing and its value at AGE_S + T bounds the error over the whole horizon. Its limit is 0.1 / D; with the M350's D = 0.246 s⁻¹, g(6.9 s) = 0.326 s against the 0.69 s of the held error, a factor of 2.1 (2.3–2.6 for the other types).

The founder's decision asked for "the tracker's drag estimate and its K σ bound". Under the linear drag model the perturbation decays at the registered D, which the tracker does not estimate and which carries no σ. The K σ of the wind and velocity estimates changes the drag force, not its decay rate. So no K σ term enters (a). The real exposure is model error: a true drag coefficient below the registered D, or drag that is not linear, slows the decay (§7). D is derived from the declared top speed and is an upper bound when that speed is a software limit (§3.3), which is the unsafe direction for (a). This is recorded as a limit and a founder item, not hidden in a factor (§8).

The speed cap and the thrust clip are projections onto convex sets, so they cannot enlarge a perturbation, and the bound holds through them. Like the tube of §3.2, the response is taken open loop: the evasive backup's own feedback is not credited.

**(b) The command difference is bounded by the filter's box and the trim, not by the airframe.** The station sends c = clip(u − D (ŵ − v̂)), with u the filter's acceleration, which main bounds per axis by u_max = 3.5 m/s². The clip is a projection onto the airframe's convex thrust set and so does not enlarge differences. For two consecutive commands:

  |c_k − c_{k−1}| ≤ |u_k − u_{k−1}| + D |Δ(v̂ − ŵ)| ≤ 2√3 u_max + D |Δ(v̂ − ŵ)|.

The first part is 12.12 m/s². The trim change has two parts between the predictions of consecutive cycles, which are 100 ms apart:

- *Propagation.* The predicted velocity moves at the model's acceleration a_act + D (ŵ − v̂), with a_act in the thrust set (|a_act| ≤ √2 a_phys) and the airspeed at most v_max + the declared wind limit (the contract's 15 m/s plus 12 m/s; `wind` checks the second). The predicted wind has no drift. So the propagation adds at most 0.1 (√2 a_phys + D (v_max + w_lim)), 0.36–0.61 m/s² after the factor D for the fleet.
- *Correction.* Each Kalman update moves the estimate by K ν, whose covariance is K S Kᵀ. Along v − w its size is at most K3 √λ_max(M K S Kᵀ Mᵀ), with M = [0, I, −I], K3 = √χ²₃(1 − q) for a three-dimensional vector at the per-check risk q, and N_UPD = 2 updates per cycle. A third update in one cycle is rare, not impossible: the Fable re-check puts it at about 5.6 × 10⁻⁸ per drone-cycle (2 × 10⁻³ per drone-hour). The bound still covers it, because three independent corrections have a combined spread of √3 times one, and N_UPD = 2 > √3.
- *Discovery of a lost command* (Fable re-check). When the echo reveals a lost command, the command log is rewritten and the predicted velocity jumps by up to 0.0755 dc_i, which adds 0.24–0.32 m/s² (2–2.5 %) to the trim change in that cycle. The formula does not include it. It matters only if a second loss follows within about five cycles. The exact `dc` check covers it: across 1462 lost uplink commands per target the largest ratio was 0.78.

So

  dc_i = min(2√2 a_phys,i, 2√3 u_max + D_i [0.1 (√2 a_phys,i + D_i (v_max + w_lim,i)) + N_UPD K3 √λ_max(M C_i Mᵀ)]),

with C_i the correction covariance of drone i's latest update. To keep the term constant during an encounter (B2), dc_i is fixed per drone at clearance (the first cycle at or after the 2 s convergence) and held for the run. The station knows every command it sends, so it checks the assumption exactly. A new monitor condition `dc` fires when a sent command differs from the drone's previous one by more than dc_i. Each row records the largest ratio |Δc| / dc_i after clearance. `dc` counts as a warning: it says that the lost-command term's assumption failed. Two limits: the correction part rests on the tracker's Gaussian model at K3, and the propagation part on the airspeed bound, which `wind` checks only horizontally.

**The term after (a) and (b):**

  lost_i = dc_i · g_i(AGE_S + T).

With RTK this is about 3.5–4.2 m per drone (`check.check_pulse_gain` and `check.check_dc_bound` test the pieces), against 11.06–13.41 m before. The matrix values are in §6.

**Horizon.** g is evaluated at AGE_S + T. That bounds the error over the backup only while the barrier's critical step k* lies within T (64 steps). The rollout can run 69–111 steps, and by 11.56 s g is up to 17 % larger. In the episodes the Fable re-check instrumented, k* was at most 36. The same limit applied to the earlier term (it concerns T, review B2), and it is listed in §7.

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

**SBAS does not qualify for the margin's time to alert.** An integrity bound is useful to the margin only if the receiver alerts before the bound can be exceeded for the length of the horizon over which the margin relies on it, so the time to alert must be shorter than T. SBAS APV-I specifies a time to alert of 10 s (Kim et al. 2016, Table 2, ICAO SARPs). T is 6.45–6.52 s (§3.3). A 10 s alert can arrive after the backup has run its course, so SBAS APV-I does not meet the rule. The previous version of this section said SBAS qualified; that was wrong.

**No positioning grade qualifies for dense corridors today.** SBAS fails the time to alert. The ZED-F9P's protection level is a 5 % bound per epoch (§2.5). Integrity RTK exists only as vendor claims. A corridor whose lane spacing rests on a GNSS margin therefore has no positioning grade whose integrity was verified against this rule. See founder items 3 and 8 (§8).

**How a protection level would enter the margin.** A PL is stated with an integrity risk per approach or per hour, not per check. Two ways keep the budget honest:

- *Per-check translation.* If the PL's integrity risk is p_PL per exposure time t_PL, and the faults it covers are independent between exposures, the risk per 100 ms check is at most p_PL · (0.1 s / t_PL) when one exposure is at least one check long. For SBAS APV-I (2 × 10⁻⁷ per 150 s approach) that is about 1.3 × 10⁻¹⁰ per check, comparable to q at 10⁻⁵ per pair-hour. The derivation needs the independence the PL's own analysis assumes, and it is stated, not prototyped.
- *PL as a radius.* The pair's GNSS term becomes PL_i + PL_j along the line of sight, and the PL's own integrity risk per pair-hour (2 p_PL per pair, scaled to the hour) is added to the margin's risk budget, so that the margin terms keep target − 2 p_PL. This needs no independence assumption and is the rule this note proposes. It is not in the prototype.

Either way the position floor stays as the receiver's reported accuracy when no qualifying PL exists, and B8 applies.

**B8 is a stated limit.** A single receiver without integrity can report a σ five times smaller than its true error. A steady bias moves the fix and the track together, so the NIS cannot see it (§5). The design does not detect this fault and adds no bespoke bias detector (founder decision). It is a limit of single-receiver GNSS without integrity.

**Lag: declared upper bound until identified (documented, not prototyped).** The lag monitor cannot tell lags apart while the command is steady (§5). The founder's interim design: each drone declares an upper bound τ_max beside τ_a, and the backup rollout of §3.5 flies τ_max until the identifier reports the lag *consistent*, then τ_a. The identifier reports *consistent*, *mismatch* (the `lag` alarm) or *not identifiable* (the window's non-centrality λ = Σ‖Δy‖² / σ²_acc for the nearest alternative is below the value that gives the F test 90 % power). It is not prototyped because no source gives a τ_max for any fleet type (§2.4). With the lag now inside the rollout, the change would be one parameter of `rollout.CTX`.

### 3.11 Using the next measured state (design note only, §8 option c)

Not prototyped (founder decision). The question is whether the station may bound a lost command's effect only until it sees it, instead of over the whole horizon.

**What the measurement gives.** A velocity error of 0.1 dc_i ≈ 1.2 m/s is twelve or more times the receivers' velocity σ (0.074–0.102 m/s), so the next telemetry message shows it, about 0.1–0.2 s after the segment, well before the echo confirms the loss (up to AGE_S). The tracker's update then corrects most of the estimate's error. What is left is (I − K_v) δv plus the noise the gain lets through.

**What it does not give.** Measuring the error makes it known, not absent. If the loss falls on the cycle at which the backup must start, the drone's true backup starts with the error, and the barrier at the next cycle is evaluated at a state whose backup is shorter by the error's effect. Recursive feasibility of a backup barrier (Gurriet et al. 2018; Chen, Singletary, Ames 2021) needs the next state on the certified backup's flow, or within the margin of it. So (c) does not remove the horizon part; it changes what bounds it. The bound becomes the closed-loop backup's sensitivity to an initial velocity error, after the station has re-planned from the measured state, plus the error accumulated in the detection window.

**What it needs, in order.**

1. *A closed-loop tube for the evasive backup.* van Wijk et al. need a Lipschitz closed-loop flow (L_cl). Main's evasive backup brakes bang-bang with a stop rule and a latch. The P3-SA-01 review records that a bang-bang brake gives no bounded tube while a contracting PD brake does. Without a Lipschitz backup there is no theorem to apply. With one, the sensitivity of a braking backup to an initial velocity error δv is about δv · (t_det + v / u_max) at speed v: at cruise speeds this is comparable to (a)'s δv (1 − e^{−DT}) / D, so the gain over (a) may be small.
2. *A measurement-robust barrier condition at the next cycle* (Dean et al. 2020; Cosner et al. 2021), with the estimate error after the update bounded at the per-check risk, which needs the worst-case Kalman gain under the position floor (`Tracker.floored`) to be derived.
3. *A recursive-feasibility step under a state jump.* Show that if the QP was feasible at cycle k with margin m and the state moves by at most the detection-window error before cycle k + 1, the backup from the new estimate keeps h ≥ 0 at the same margin. This is the B2 argument with a jump term. The sampled-data robust CBFs (Singletary 2020, Theorem 1; the doubly robust sampled-data CBF of arXiv 2309.08050, *secondary*) bound the state over one period and would supply the jump bound.
4. *Re-prediction from the pre-loss posterior.* `CommandLog.confirm` corrects the log when the echo arrives, but the tracker's posterior is not re-propagated from before the loss (Fable re-review). The applied-command echo of contract v1.6 (founder D1) and a stored pre-loss posterior would give the exact correction.

Conclusion of the note: (c) is sound only with a Lipschitz backup, so it depends on a change to main's evasive kernel. Its expected gain over (a) and (b) is small at cruise speeds. Option (b) of §8 (a command plan per message) waits for a consistent per-drone backup (BL-03) and an onboard-layer ADR.

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
| `dc` | the difference between consecutive thrust commands the station sends to one drone | ≤ dc_i, the bound fixed at clearance (§3.6) | the lost-command term's assumption no longer holds: a lost command could cost more than the margin covers | Hold new clearances in the area and widen the drone's term to the airframe bound 2√2 a_phys until the cause is found. Counted as a warning. |
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

Setup (`sim.py`, `run.py`): six encounter families (head-on, crossing, climbing, rings of 4 and 6, random), 8 seeds each, 22 s episodes with the encounter at t = 12 s, a 10 ms plant and a 100 ms ground-station cycle. The plant has linear drag D (w − v) with each drone's registered D, a first-order actuator lag and the tilt-authority clip (§3.3). Every scenario is flown at the cleared cruise speeds (§3.3), scaled so that the drones meet at the same point and time as in the uncleared scenario. Every job runs at both risk targets: 10⁻⁵ per pair-hour (K1 = 6.31, K2 = 6.63, K3 = 6.89) and 10⁻⁷ (K1 = 6.99, K2 = 7.30, K3 = 7.54).

**Provenance.** The current matrix (`runs4`; rerun as `runs5` with the certificate diagnostics, below) has 2336 rows: per target, 864 in-limit (6 families × 8 seeds × 6 conditions × 3 modes), 160 beyond-limit and 144 takeoff. All were written at commit `e06d9414dd95` with a clean tree, on one machine (Linux, 4 cores, Python 3.13, NumPy 2.5.4, SciPy 1.18.1, OSQP 0.6.7.post3), in four shards (`--shard k --of 4`). It differs from the previous matrix (`runs3`, commit `b6139175fdb3`, the results of `a5b5264`) only in the lost-command term (§3.6, re-review 3a and 3b), the `dc` check, and the recoverability fix below. Same scenarios, seeds and random streams. The baseline rows are identical to `runs3`'s in every field. Per-run rows are in `results/runs4-{0..3}.jsonl`; the tables are in `results/summary-runs3.md` for `runs3`.

**Rerun with certificate diagnostics (Fable re-check B1).** Rows now record main's own `pair_violation_depth_m` from clearance on: `viol_cycles`, `viol_first_s`, `viol_last_s`, `viol_max_m` and `viol_pairs_max` (the cycles on which a pair's predicted backup separation is inside d_eff = 5 m + the margin in force, so the certificate does not hold). They also record `escape_failed_cycles` and `min_sep_clear`, the separation from clearance on. Both lost-command terms were rerun with them on the same jobs. `runs5-{0..3}.jsonl` (the tighter term) at `9f3dcad8d0d8` reproduces `runs4` in every earlier field. `runs5h-{0..3}.jsonl` (the a5b5264 term, `--lost held`) at `f01961701893` reproduces `runs3` in every earlier field; the code between the two commits differs only in runner flags and a hook that is inactive with main's filter. Tables: `results/summary.md` (`runs5`), `results/summary-runs5h.md`.

**Medians.** Every p50 in this section is `run.table`'s median: for an even number of runs, the upper of the two middle values. Up to 0.12 m of the earlier version's requested-margin medians used the mean of the two (Fable re-review N-a); they are restated here in `run.table`'s convention.

**Recoverability (re-review R1).** `sim.recoverable` used to run main's filter while `rollout.CTX` still held the previous adaptive episode's drones. When that episode had fewer drones, the lagged rollout raised a `KeyError` and main flew its evasive fallback; otherwise it flew another scenario's airframes and wind. `recoverable` now clears `CTX` first (commit `d602f592d04e`). `run.py --rerecord` recomputed `ideal_min_sep` in the `runs3` rows at that commit, stamped `ideal_git`, without rerunning anything else. 1443 of 2336 rows (1036 of 1184 adaptive) changed, by up to 4.35 m. **No scenario changed class:** random seed 4 is the only one below 5 m (2.24–3.49 m), and the closest in-limit scenario is at 5.58 m. The `runs3` tables regenerated from the corrected rows are identical to the `a5b5264` tables. The column now depends on the scenario only. The re-review counted 895 differing adaptive rows and a closest in-limit value of 5.51–5.54 m on its laptop; the difference was not investigated (a second platform with NumPy 1.26, a different tolerance in the count, or both).

**Admissibility (review B2).** At the first cycle after the 2 s track convergence, a scenario is admissible if the backup rollout from its states already keeps every pair 5 m + m_req apart (§3.9). With the smaller lost-command term far more scenarios clear:

| condition | admissible of 47, 10⁻⁵ (`a5b5264`) | 10⁻⁷ (`a5b5264`) |
|---|---|---|
| RTK, calm | 47 (45) | 47 (45) |
| RTK, India wind | 45 (43) | 44 (43) |
| SBAS, calm | 44 (40) | 43 (39) |
| SBAS, India wind | 43 (29) | 43 (23) |
| GPS SPS, calm | 43 (39) | 42 (39) |
| GPS SPS, India wind | 37 (22) | 29 (14) |
| takeoff, RTK / SBAS / SPS (India) | 46 / 45 / 44 (45 / 43 / 43) | 45 / 44 / 43 (45 / 43 / 43) |

23 and 34 recoverable in-limit runs are not admissible, against 64 and 79 before.

**Counting.** "Warning" means any alarm from the track-convergence time on, except `absorb` and `margin` (founder B2), which say that the margin in force is short of the requested one, not that a fault is present. `headroom` counts from the first monitored cycle (founder B3); `dc` counts (§3.6). "No warning" means no alarm other than `absorb` or `margin`.

### 6.1 Inside the limits

Admissible encounters only. Each cell gives 10⁻⁵ / 10⁻⁷.

| condition | mode | runs | LoS | runs with a warning (kinds) | margin in force p50 (m) | requested at clearance p50 (m) | GNSS / lost-command term p50 (m) | min sep (m) |
|---|---|---|---|---|---|---|---|---|
| RTK, calm | adaptive | 47 / 47 | **0 / 0** | 29 / 29 (headroom 27, wall 4, lag 1) | 8.03 / 8.06 | 10.18 / 10.45 | 0.29 / 7.55 — 0.32 / 7.56 | 12.83 / 12.91 |
| RTK, calm | baseline | 47 / 47 | 18 / 18 | – | 0.5 | – | – | 3.51 / 3.51 |
| RTK, India wind | adaptive | 45 / 44 | **0 / 0** | 5 / 7 (nis 5 / 7, saturation 0 / 1) | 18.07 / 19.22 | 17.78 / 18.92 | 10.26 / 7.63 — 11.47 / 7.65 | 22.81 / 23.91 |
| RTK, India wind | baseline | 45 / 44 | 1 / 1 | – | 0.5 | – | – | 4.79 / 4.86 |
| SBAS, calm | adaptive | 44 / 43 | **0 / 0** | 35 / 34 (wall 26, headroom 24, saturation 1 / 2, lag 1 / 0) | 39.44 / 42.23 | 37.51 / 40.55 | 31.80 / 7.55 — 34.51 / 7.56 | 31.17 / 34.17 |
| SBAS, calm | baseline | 44 / 43 | 37 / 36 | – | 0.5 | – | – | 0.28 / 0.28 |
| SBAS, India wind | adaptive | 43 / 43 | **0 / 0** | 15 / 20 (wall 15 / 20, saturation 1) | 40.75 / 43.94 | 39.84 / 43.24 | 32.74 / 7.67 — 36.17 / 7.69 | 33.98 / 36.99 |
| SBAS, India wind | baseline | 43 / 43 | 10 / 11 | – | 0.5 | – | – | 1.50 / 1.39 |
| GPS SPS, calm | adaptive | 43 / 42 | **0 / 0** | 39 / 38 (wall 33 / 32, headroom 24 / 23, lag 1) | 48.57 (cap) | 45.24 / 48.54 | 46.76 / 7.56 — 51.78 / 7.57 | 32.20 / 32.20 |
| GPS SPS, calm | baseline | 43 / 42 | 39 / 38 | – | 0.5 | – | – | 0.09 / 0.09 |
| GPS SPS, India wind | adaptive | 37 / 29 | **0 / 0** | 23 / 17 (wall 23 / 17, saturation 1, lag 1 / 0) | 48.57 (cap) | 48.86 / 51.88 | 45.21 / 7.67 — 49.20 / 7.69 | 37.28 / 37.05 |
| GPS SPS, India wind | baseline | 37 / 29 | 5 / 3 | – | 0.5 | – | – | 1.84 / 2.49 |

`baseline_lag` gave the same result as `baseline` in 572 of 576 run pairs (the other 4 within 0.15 m, no loss), so the table shows `baseline` only.

**Pass criterion 1 is met, in-model, at both targets:** no loss of separation in 259 admissible adaptive runs at 10⁻⁵ and 248 at 10⁻⁷. 113 and 103 of them raised no warning other than `absorb` or `margin`. The RTK and SBAS rows (179 and 177 runs) ran with the margin the target asks for in every cycle: the requested margin never exceeded the filter's 48.57 m bound. In the SPS rows it exceeded the bound in 71 of 80 and 71 of 71 runs, so those rows are results for a 48.57 m margin. For the claim itself, 0 losses in 179 runs bounds the per-run loss rate below about 1.7 % at 95 % confidence (rule of three).

Against `a5b5264`:

- **The lost-command term fell from 22.1–27.1 m to 6.9–8.3 m per pair** (per-run values; the condition medians are 24.5–24.7 and 7.6–7.7 m). Re-review 3a (the drag decay) cuts it by a factor 2.1–2.6. 3b (the box bound, dc_i = 12.5–13.6 m/s² against 16.0–19.4 m/s²) cuts it by a further factor 1.2–1.5. The GNSS term's formula is unchanged, but its condition medians moved by up to 1.2 m (30.62 to 31.80 m, SBAS calm) because the admissible sets and the trajectories changed.
- **The RTK margin fell by 15–17 m:** 8.0 m in calm air (24.9 before), 18.1 / 19.2 m in the India wind (34.7 / 35.8). In the wind the GNSS term (10–11 m, the wind uncertainty through the drag over T) is now the larger part.
- **SBAS fits the filter's bound at both targets.** The requested margin at clearance is 38–43 m, against 54–59 m before, and `margin` fired in no SBAS run. SPS still requests 45–52 m.
- **The smallest separations fell with the margins** (RTK calm 12.83 m against 27.56 m; SBAS calm 31.17 against 43.40 m), and stayed far from 5 m.
- **`absorb` rose where the margin left the cap.** SBAS 16–21 runs (largest deficit 0.91 / 1.08 m), RTK India 8 / 9 (≤ 0.13 m), SPS 3 / 0. Before, SBAS was clamped at 48.57 m in every cycle and could not rise, so it showed no `absorb`. `absorb` is not a warning (B2), but each such cycle is one where the claim was short by that amount.
- **The `dc` alarm fired in no run of the matrix.** The largest observed ratio |Δc| / dc_i after clearance was 0.37 in-limit and 0.44 on takeoff under RTK, 0.78 under SBAS, 0.48 / 0.41 under SPS in-limit (0.58 on its takeoff tracks) and 0.76 in the beyond-limit cases. The bound is a worst case: in these encounters the station's commands change by at most about three quarters of it.
- **The certificate fails in the SBAS six-drone rings (Fable re-check B1).** Counted from clearance on, in admissible in-limit runs (10⁻⁵ / 10⁻⁷):

  | | runs with a violation | cycles | deepest (m) | runs deeper than 0.1 m | fallback runs |
  |---|---|---|---|---|---|
  | tighter term (`runs5`), all in-limit | 82 / 66 of 259 / 248 | 480 / 410 | 1.94 / 1.74 | 16 / 10 | 3 / 3 |
  | of which SBAS calm | 20 / 20 of 44 / 43 | 228 / 219 | 1.94 / 1.74 | 11 / 9 | 2 / 2 |
  | of which SBAS India | 24 / 23 of 43 / 43 | 159 / 136 | 0.16 / 0.16 | 4 / 1 | 0 / 0 |
  | a5b5264 term (`runs5h`), all in-limit | 36 / 31 of 218 / 203 | 67 / 54 | 0.06 / 0.05 | 0 / 0 | 0 / 0 |
  | main as configured (baseline, 0.5 m), same sets | 67 / 63 of 259 / 248 | 393 / 386 | 1.94 / 1.94 | 57 / 56 | 8 / 7 |

  RTK and SPS rows stay at the a5b5264 level (at most 0.25 m, mostly below 0.1 m). No run lost separation.

  *Three causes separated* on SBAS calm ring6 seeds 0–7 and SBAS India ring6 seeds 0, 1 and 3, at both targets (22 runs; `results/diag-b1-*.jsonl`):

  | variant | runs with a violation | cycles (largest in one run) | deepest (m) | fallback runs (cycles) | margin p50 range (m) |
  |---|---|---|---|---|---|
  | (a) main's filter as now (`runs5`) | 22 | 421 (34) | 1.94 | 4 (5) | 39.9–46.9 |
  | (b) PR #26's KKT-exact solver (`perf/solver-exact` `ed6dcec`, filter blob `49e8eea56d5b`, step deadline off), same margin | 22 | 411 (34) | 3.05 | 0 | 39.9–46.9 |
  | (c) margin frozen at its clearance value (diagnostic only, `--freeze-margin`) | 14 | 36 (4) | 0.055 | 0 | 37.4–43.7 |

  (b) removes every fallback but not the violations (the deepest grows to 3.05 m in SBAS calm ring6 seed 1). (c) brings them to the level of the a5b5264 term. The cause is therefore the margin moving during the encounter, not main's solver. The rate limit (§3.9) lets the applied margin rise each cycle up to m*, the largest the current backup can hold. That places the barrier exactly at its edge, h = 0, so the next cycle's linearisation and sampling errors show as violations. Under the a5b5264 term the SBAS margin sat at the 48.57 m cap and could not rise. The fix belongs in the rate limit, for example raising the margin only at the barrier's own decay rate, and not in the lost-command term. Not tuned here. Frozen-margin rows were written at `91b02b999b29` and `05399403cdcd` (a results-only commit landed between its two invocations); PR #26 rows at `91b02b999b29` in a scratch worktree with that branch's `filter.py` and `shared/contracts.py`.
- **Main's QP fell back in 6 admissible in-limit runs** (SBAS calm ring6 seeds 1 and 3, SPS India cross2 seed 5, each at both targets). Each was one cycle at 6.9–9.0 s with OSQP at its iteration limit or "solved inaccurate", and main applied its evasive fallback for that cycle; no loss followed (smallest separations 34.2–57.9 m). Five are new. The sixth, SPS India cross2 seed 5 at 10⁻⁷, fell back identically in `runs3`, where it was not admissible. B3 ring4 seed 1 at 10⁻⁷ falls back in both matrices (beyond-limit). Main's baseline falls back in 8 / 7 runs (9 / 8 cycles), unchanged. With PR #26's solver all six run without a fallback (§8 item 1).
- **Other counts that rose,** none with a loss: `saturation` in 3 / 5 in-limit runs (0 before), `lag` false alarms in 4 / 2 (1 / 1), `nis` in 5 / 7 RTK India runs (3 / 6).
- **The baseline is unchanged row for row;** its counts change only because the admissible set grew (110 and 107 losses in the larger sets).

### 6.2 Outside the limits

All beyond-limit cases run in the India wind and under SBAS, except B7 (GPS worst site) and the `_rtk` variants. Two families (`headon2`, `ring4`) with 8 seeds each. All 16 runs are counted; admissibility is reported and not used to filter. Each cell gives 10⁻⁵ / 10⁻⁷. The `a5b5264` values are in brackets where they changed.

| case | what breaks | admissible | LoS | warning before any LoS | case alarm: runs, first time (median s) | min sep (m) |
|---|---|---|---|---|---|---|
| B1 squall, 11 m/s mean, moderate turbulence | wind | 16 / 14 (9 / 8) | 0 / 0 | 16 / 16 | `wind` 16 / 16, 2.0 s | 45.91 / 45.22 |
| B2 gust front, +8 m/s on one drone at 9 s | wind model | 16 / 16 (14 / 9) | 0 / 0 | 16 / 16 | `nis` 16 / 16, 9.6 / 9.5 s | 37.14 / 40.635 (45.46) |
| B3 GNSS white noise at 5 σ | GNSS model | 16 / 16 (15 / 10) | 0 / 0 | 16 / 16 | `nis` 16 / 16, 2.0 s | 38.46 / 41.59 (47.68 / 46.92) |
| B4 downlink out 3 s at 9 s | link | 16 / 16 (14 / 9) | 0 / 0 | 16 / 16 | `link` 16 / 16, 9.3 s | 38.75 / 41.84 (45.68 / 45.93) |
| B5 uplink out 1.5 s at 9 s | link | 16 / 16 (14 / 9) | 0 / 0 | 16 / 16 | `link` 16 / 16, 9.4 s | 37.77 / 40.70 (45.83 / 45.86) |
| B6 actuator lag 0.5 s (2×), SBAS | lag | 16 / 16 (14 / 9) | 0 / 0 | 16 / 12 (14 / 14) | `lag` 13 / 7 (9 / 10), 8.0 / 7.2 s | 38.27 / 41.53 (45.91 / 45.84) |
| B6 actuator lag 0.5 s (2×), RTK | lag | 16 / 16 | 0 / 0 | 16 / 16 | `lag` 5 / 8 (10 / 8), 10.2 / 10.6 s | 22.97 / 24.04 (36.95 / 37.96) |
| B7 GPS worst site (15 / 33 m, 95 %) | GNSS level | 6 / 3 (2 / 1) | 0 / 0 | 14 / 14 | `margin` 16 / 16 (not a warning) | 13.56 / 12.19 |
| B8 bias 5 σ, reported σ nominal, SBAS | receiver integrity | 16 / 16 (9 / 9) | 0 / 0 | 13 / 13 | none specific (`wall` 13 / 13) | 21.84 / 23.255 (25.10 / 25.77) |
| B8 bias 5 σ, reported σ nominal, RTK | receiver integrity | 16 / 16 | 0 / 0 | 0 / 0 | none | 22.86 / 23.93 (36.59 / 37.60) |

- **No case lost separation at either target**, and the `dc` alarm fired in none.
- **B1–B5 raised their case alarm in all 16 runs**, at the same times as in `a5b5264`. With SBAS margins now below the cap, B2–B5 also raise `absorb` in 8–16 runs (deficits up to 1.2 m): the faults raise the requested margin, and the rate limit lets it rise only as fast as the barrier absorbs.
- **B6 detection moved both ways:** SBAS 13 / 7 of 16 (9 / 10 before), RTK 5 / 8 (10 / 8). The smaller margins change how much the commands vary, which is what the lag identifier needs (§5). With 16 runs per cell, these shifts are within what the sample can resolve; the N1 trade-off of §8 stands.
- **B7** is unchanged: no warning of its own, smallest separation 12.2 m.
- **B8 under SBAS still lost no separation.** The margin is now 43–46 m (48.57 m before), still wider than a 5 σ bias (5 × 3.1–3.9 m per drone); the smallest separation is 21.8 m. B8 remains masked, not solved.

### 6.3 Takeoff tracks (decision 5)

The track starts 15 s before the run's t = 0, with the drone hovering for 3 s and then accelerating to its cruise velocity (§4.4). India wind, admissible scenarios only. Each cell gives 10⁻⁵ / 10⁻⁷.

| grade | runs | LoS | runs with a warning | alarm of any kind in the last 5 s before the encounter | absorb: runs (cycles, largest deficit m) | margin in force p50 (m) | min sep (m) |
|---|---|---|---|---|---|---|---|
| RTK | 46 / 45 | **0 / 0** | 6 / 5 (nis, lag) | 7 / 10 | 14 / 14 (18 / 20, 0.15 / 0.12) | 17.80 / 18.95 | 10.72 / 23.94 |
| SBAS | 45 / 44 | **0 / 0** | 30 / 30 (wall 27 / 27) | 19 / 14 | 27 / 22 (186 / 157, 0.60 / 0.59) | 40.02 / 43.36 | 35.00 / 40.42 |
| GPS SPS | 44 / 43 | **0 / 0** | 38 / 37 (wall 36 / 35) | 8 / 9 | 4 / 0 (27, 0.55) | 48.57 (cap) | 31.37 / 32.29 |

No loss of separation. The RTK 10.72 m at 10⁻⁵ is `random` seed 0, which became admissible with the smaller margin (requested 17.87 m against m* = 18.71 m at clearance). Its two closest drones hover 10.7 m apart at the track's first tick, 15 s before the encounter and before clearance. After clearance the pair stays at least 22.5 m apart against a d_eff of 22.9 m. The row's `min_sep` covers the whole episode, warm-up included, so this value is the starting geometry, not the margin's result.

### 6.4 Corridor capacity per GNSS grade (decision 1, review N5)

Reference corridor 100 m wide and 30 m high. Lanes on a square grid at spacing s = 5 m + the median margin in force of the in-limit runs, drones along each lane at s. Each lane keeps at least one margin from each wall: the margin is also main's wall standoff (`FilterConfig.margin_m`), and the `wall` alarm shows that the backup's braking is not guaranteed nearer the wall.

| grade, wind | target | margin p50 (m) | lane spacing s (m) | lanes in 100 m × 30 m | drones per km | height for one layer (m) | `a5b5264` margin p50 (m) |
|---|---|---|---|---|---|---|---|
| RTK, India wind | 10⁻⁵ | 18.07 | 23.1 | **0** | **0** | 36.1 | 34.68 |
| RTK, India wind | 10⁻⁷ | 19.22 | 24.2 | **0** | **0** | 38.4 | 35.83 |
| RTK, calm | 10⁻⁵ / 10⁻⁷ | 8.03 / 8.06 | 13.0 / 13.1 | **14** (7 × 2) | **1064** | 16.1 | 24.89 / 24.97 |
| SBAS, India wind | 10⁻⁵ / 10⁻⁷ | 40.75 / 43.94 | 45.8 / 48.9 | 0 | 0 | 81.5 / 87.9 | 48.57 (cap) |
| GPS SPS, India wind | either | 48.57 (cap) | 53.6 | 0 | 0 | 97.1 | 48.57 (cap) |

`summary.md` gives the India-wind rows only. The calm-air RTK row is computed with the same `run._lanes` from the calm-air medians.

- **In the India wind no grade fits a lane in the reference corridor at either target.** RTK needs 36–38 m of height for one layer. A corridor 100 m wide and that tall would hold 3 RTK lanes, about 129 drones per km at 10⁻⁵ and 123 at 10⁻⁷.
- **In calm air RTK fits 14 lanes, 1064 drones per km** at both targets. The difference between calm and wind is the GNSS term's wind part (10–11 m), which the tracker carries from the drag model over T.
- The earlier per-check figures (12 400, 280 and 66 drones per km, `runs2`, K = 4.033) stay withdrawn (§6.5).

### 6.5 Changes from the earlier runs

**`a5b5264` (`runs3`) to this matrix.** Only the lost-command term changed (re-review 3a, 3b), plus the `dc` check and the R1 column fix:

| quantity | `a5b5264`, 10⁻⁵ / 10⁻⁷ | this matrix, 10⁻⁵ / 10⁻⁷ | cause |
|---|---|---|---|
| adaptive LoS, in-limit | 0 of 218 / 0 of 203 | 0 of 259 / 0 of 248 | – (more admissible runs) |
| lost-command term per pair | 22.1–26.8 m | 7.5–7.7 m | drag decay (3a) and box bound (3b) |
| margin p50, RTK calm / India | 24.9 / 34.7; 25.0 / 35.8 m | 8.0 / 18.1; 8.1 / 19.2 m | the lost term |
| SBAS requested at clearance | 54–59 m (above the cap) | 38–43 m (below the cap) | the lost term |
| admissible in-limit runs | 218 / 203 | 259 / 248 | smaller requested margins |
| smallest separation, RTK calm | 27.56 / 27.64 m | 12.83 / 12.91 m | smaller margin |
| `absorb`, in-limit | RTK India 11 / 15 runs, ≤ 0.13 m | SBAS 16–21 runs, ≤ 1.08 m; RTK India 8 / 9, ≤ 0.13 m | SBAS margin no longer clamped |
| certificate violations, admissible in-limit | 36 / 31 runs, 67 / 54 cycles, ≤ 0.06 m | 82 / 66 runs, 480 / 410 cycles, ≤ 1.94 m | the margin rising during the encounter (rate limit, §6.1) |
| QP fallback, admissible in-limit runs | 0 | 6 (3 scenarios × 2 targets) | OSQP iteration limit near the barrier's edge; none with PR #26's solver (§6.1) |
| saturation / lag false alarms / RTK India nis, in-limit runs | 0 / 1 / 3; 0 / 1 / 6 | 3 / 4 / 5; 5 / 2 / 7 | smaller margins, more manoeuvring |
| `dc` alarm | – | 0 runs (largest ratio 0.78) | new check (3b) |
| B6 lag detected, SBAS / RTK | 9 / 10, 10 / 8 of 16 | 13 / 7, 5 / 8 of 16 | command variation changed with the margins |
| capacity, RTK India / calm, drones per km | 0 / 0 | 0 / 1064 | the lost term |

Three safety metrics got worse: the certificate violations (§6.1, the largest change), the QP fallback count in admissible runs (0 to 6), and the smallest separations, which follow the smaller margins by design. The B6 detection count moved both ways. No run lost separation. The tighter term is therefore reported, not adopted (§8 item 1).

**`runs2` (`486f3b4bd66c`, 1168 rows, K = 4.033, per check only).** The record of the per-check matrix, kept in `results/`. Its margins (RTK 1.3–1.4 m, SBAS 23 m, SPS 39 m), capacities (12 400 / 280 / 66 drones per km), `absorb` deficits up to 16 m and `headroom` rates are about K = 4.033 per check, an isotropic GNSS ball and lanes at the walls, and are superseded (reviews B1, B2, B6, N5).

**The laptop start (`b375131`, 157 rows, `results/partial-b375131-*.jsonl`).** Reproduced by `runs3` in every field except `ideal_min_sep` (R1) and the wall time.

### 6.6 Why main's RTK calm baseline went from 7 to 17 losses (founder, interim review item 3; measured on `runs2`)

This diagnostic was run on the `runs2` plant at uncleared speeds and was not repeated. At the cleared speeds the RTK calm baseline loses separation in 18 of 47 recoverable scenarios (the baseline rows of `runs3` and `runs4` are identical), so the conclusion is unchanged.

The diagnostic reruns the 47 recoverable RTK calm baseline scenarios with the baseline's plant and command changed one at a time, by text substitution in a copy of `sim.py` (the repository is not edited; the script is a scratch file and not committed).

| variant | what changes | LoS | min sep (m) | runs saturated |
|---|---|---|---|---|
| A, as in the matrix | drag plant, trim D v_route, authority clip | 17 | 1.23 | 20 |
| B, no drag | D = 0, so no trim; clip kept | 7 | 2.20 | 0 |
| C, trim from the measured velocity | trim D v_meas, which cancels the drag at the measured speed | 15 | 2.44 | 21 |
| D, no trim | drag plant, command u only | 0 | 8.87 | 0 |
| E, first run's plant | D = 0 and no clip, with this run's random streams | 7 | 2.20 | 0 |

Variant A reproduces the `runs2` matrix row exactly.

- **The drag plant explains the rise.** Without drag (B) the baseline loses separation in 7 runs again, the first run's count. B and E give the same runs, so the authority clip never acts without drag. E also shows that the random streams alone do not change the count.
- **The cost is authority, more than the direction of the trim.** To hold its route speed against drag a drone needs D v of thrust, up to 3.7 m/s² for the M350 at 15 m/s against its 5.66 m/s². The authority clip then cuts the filter's braking: 20 of 47 runs saturated, and 14 of the 17 losses are among them. Trimming against the measured velocity (C) still loses 15.
- **Without any trim (D) the encounters do not happen.** The drag slows every drone towards zero before the encounter, so D is not a usable baseline.

Main's filter assumes a double integrator with the full A_REL available for braking. On a plant where cruising costs a share of the thrust, that assumption fails in the cruise-heavy encounters. This is a property of main's filter on a drag plant, not of the adaptive margin, and it relates to BL-06 and BL-07 (§8).

### 6.7 What the results do and do not show

- In-limit and in-model, the adaptive design kept every admissible pair apart at both risk targets, under three GNSS grades, in calm air and in the India wind, from a cold start and from a takeoff track.
- With the tighter lost-command term, the RTK and SBAS rows ran with the margin the target asks for. The SPS rows still ran at main's 48.57 m bound, and show only that such a margin keeps these encounters apart.
- The RTK margin is 8 m in calm air and 18–19 m in the India wind. Calm-air RTK gives 1064 drones per km in the reference corridor. In wind no grade fits it: the wind uncertainty carried through the drag is now the largest RTK term.
- Main's QP fell back in 6 admissible runs at the new margins. The fallback is unverified and no loss followed. The design's claim rests on main's QP staying solvable, and at 39–46 m margins in six-drone rings it did not always.
- Every fault in B1–B5 raised its alarm before the encounter. B6 was found in 5–13 of 16 runs. B7 and B8 have no warning of their own. The safe responses were not executed.
- B8 shows no loss only because the SBAS margin (43–46 m) is wider than a 5 σ bias. It remains the main open safety limit.

## 7. Known limits

Data:

- **No airborne GNSS or cellular data from India.** The GNSS figures are receiver and system specifications (§2.1). The latency figures are from a Spanish and a Malaysian trial, with only TRAI ground-level medians for India (§2.2). No source gives a GNSS error correlation time, which is why the position floor exists.
- **The latency model comes from one modern network.** It is fitted to a 2024 dense-urban 5G and 4G trial (median RTT about 60 ms). A 2018 LTE-Advanced trial measured 200–300 ms at 50–100 m (Yang et al., §2.2). On such a network the 130 ms playout would drop most commands, and AGE_S, and with it the lost-command term, would grow. The playout delay has to be set from the operating network's measured latency.
- **Wind at drone height is extrapolated.** The India mean wind comes from CEEW's 100 m resource maps, and the gusts from the MIL-F-8785C Dryden model, not from Indian measurements at 0–120 m (§2.3).
- **Drag and authority are derived, not measured.** No manufacturer publishes either. D = a_phys / v_max assumes the top speed is drag-limited at full tilt and that drag is linear in airspeed (§3.3). Payload and air density change both and are not modelled. The cleared speeds of §3.3 rest on these derived figures. The Mavic 3 Enterprise's 12 m/s wind figure is stated for takeoff and landing only and is used en route.
- **Vertical authority is assumed** equal to the horizontal one (`sim._authority`).
- **Two of six types checked cannot register.** The DJI Agras T40 and the Garuda Agri Kisan publish no tilt angle, and the T40 no top speed (§2.4).

Model and simulation:

- **In-model drag.** The plant, the tracker and the backup rollout share the same linear drag with the same registered D. A real drone's drag differs in coefficient and shape, and nothing in the monitor checks D directly; a larger D shows up only through `nis`, `headroom` or `wind`. Because the rollout now flies the drag, a wrong D also makes the certificate wrong, not only the margin.
- **The lag inside the rollout is the declared lag.** The rollout flies τ_a = 0.25 s (§3.5). A drone with a longer lag brakes later than its certificate says, and the lag monitor found a doubled lag in only 5–13 of 16 runs (§6.2). The declared upper-bound lag of §3.10 would close this and is not prototyped.
- **Independent gusts.** Each drone gets its own gust process. The Dryden correlation length is about 270 m, so drones a few tens of metres apart really feel almost the same gust. This overstates the relative gust and so makes the margin conservative.
- **Constant GNSS bias in the nominal runs.** The bias is drawn once per drone per run and held. White noise is tested only as a beyond-limit case (B3). Correlated errors between drones (the same satellites, the same ionosphere) are not modelled. They would cancel in the relative position, so the margin is conservative on that count too, but the monitor cannot see them.
- **Full command echo.** The drone reports its whole command history with each telemetry message. A real link would acknowledge commands one at a time, and a lost acknowledgement would look like a lost command. A braking echo at the same tick as a lost planned command counts that command as delivered (`CommandLog.delivered`).
- **The lag monitor sees only command changes,** and with the N1 trend term it loses part of what it sees (§6.2). A drone flying a steady command, such as a head-on pair at the cleared speed, is not checked. Its 20-sample window and its grid of five alternatives are design choices; a lag between grid points is detected only through its nearest neighbour.
- **The station is given the true initial trim** (review N4). `sim.episode` starts the tracker and the command log from the drone's true trim −D (w(0) − v). A real station has only its first command. In-model and small.
- **Recoverability check fixed after the fact (R1).** The `runs3` column `ideal_min_sep` was re-recorded at `d602f592d04e`, not rerun with the episodes; no scenario changed class (§6). `runs4` was computed with the fix.

Method:

- **Risk per pair-hour by the union bound.** The margin terms use K1 and K2 for a per-check risk q = target / 36 000 (§3.2). The bound holds whatever the correlation between checks, so it is conservative. How conservative was not studied, and the margins of §6 suggest it costs a great deal. The target covers the margin terms of one pair; a traffic-level target per flight hour would also have to count pairs per flight hour.
- **The lost-command term is open loop and rests on derived quantities.** It carries one lost segment through the lag and the registered drag to the end of the horizon (§3.6). A true drag below the registered D slows the decay (founder item 4). The command-difference bound rests on the tracker's Gaussian model at K3 and on an airspeed bound; the station checks it exactly (`dc`), and it held in every run. The evasive backup's own feedback is not credited (§3.11).
- **The filter's margin bound caps SPS.** Main's `FilterConfig` cannot express a margin above 48.57 m. SPS requests 45–52 m, so in most SPS runs the margin in force is the cap and the risk claim is not in force (§6.1). SBAS now stays below it.
- **Admissibility is checked once, at clearance.** The test uses the unclamped margin at the first cycle after the 2 s convergence. It says nothing about states later in the run, which the rate limit and `absorb` cover (§3.9).
- **The safe responses are not executed.** The prototype records alarms only. The beyond-limit runs therefore show whether the monitor fires, not that the proposed responses keep the drones apart. In this matrix the margins are so large that they decide the beyond-limit outcome on their own.
- **Small sample.** 29–47 admissible scenarios per in-limit condition and 16 runs per beyond-limit condition. Zero losses in n runs bounds the per-run loss rate below about 3 / n at 95 % confidence (rule of three), nothing tighter. The claim at the target rests on the 179 / 177 RTK and SBAS runs per target.
- **The rate limit lets the certificate fail.** Raising the margin up to m* each cycle puts the barrier at its edge, and in SBAS six-drone rings the certificate then fails for up to 3.3 s by up to 1.94 m (§6.1). Main's QP also fell back in 6 admissible runs at these margins, none with PR #26's solver. The certificate holds only while the barrier keeps slack and the QP solves.
- **`min_sep` covers the whole episode,** warm-up included. On takeoff tracks it can be the starting geometry, before clearance (§6.3).
- **In-model only.** The tracker's model is the plant's model (§6.1). A real drone's dynamics, gusts and GNSS errors will differ from both.
- **Recursive feasibility is sketched, not proven** (§3.9). The absorbable margin uses the previous configuration's wall standoff and horizon and a read-only latch. Anticipation of margin rises was not implemented. With the constant terms `absorb` is now rare (≤ 0.13 m, RTK India only).
- **Lines of sight from the estimated rollout.** The direction-aware GNSS term (§3.2) takes the lines of sight from the rollout of the predicted states, not of the true ones; the difference is second order in the error and is not bounded here.
- **The NIS self-check uses white GNSS noise** (`check.check_nis`), while the nominal runs use a steady bias, so the 5.6–6.4 mean-NIS result describes the tracker, not the runs.
- **`EPS_LIN` is measured, not proven** (PR #12), and was measured on main's drag-free plant, not on the lagged rollout.
- **The interim-review design changes are not prototyped** (§3.10). The protection-level rule and the declared upper-bound lag are design text only; every result in §6 uses the reported σ and the nominal τ_a.
- **Rows keep only the first time of each alarm** (review N15). A `headroom` alarm at 2.0 s that recurs later is visible only through the run's alarm-cycle count (§6.1).
- **Integrity sources are thin outside aviation.** The SBAS PL equations and the ICAO APV-I figures are primary. The ZED-F9P PL is primary but only 95 %. Integrity RTK rests on vendor claims, and the urban-integrity review was read only through a summary (§2.5).

## 8. For the founder

Decisions 1–7, the four on the interim report, B1–B6 and N1 on the Fable review and 3(a)–(c) on the Fable re-review are applied above. These items remain open; none was changed in this work.

1. **The tighter lost-command term is parked: reported, not adopted.** Your condition on the fallbacks is met: with PR #26's KKT-exact solver (`perf/solver-exact` `ed6dcec`, step deadline off) the six fallback runs ran with 0 fallbacks:

   | run | target | fallbacks | min sep (m) | margin p50 (m) | certificate violations (cycles, deepest m) |
   |---|---|---|---|---|---|
   | SBAS calm ring6 seed 1 | 10⁻⁵ / 10⁻⁷ | 0 / 0 | 34.13 / 37.71 | 41.17 / 44.31 | 34, 3.05 / 19, 0.15 |
   | SBAS calm ring6 seed 3 | 10⁻⁵ / 10⁻⁷ | 0 / 0 | 41.21 / 44.60 | 41.22 / 44.65 | 19, 0.20 / 21, 0.25 |
   | SPS India cross2 seed 5 | 10⁻⁵ / 10⁻⁷ | 0 / 0 | 55.14 / 57.91 | 46.16 / 48.57 | 1, 0.004 / 2, 0.03 |

   But the fallbacks were the visible part of a larger problem the Fable re-check found (B1). With the tighter term, main's backup certificate fails in 82 / 66 admissible runs, by up to 1.94 m. The cause is the margin rising during the encounter under the rate limit, not the solver, as the frozen-margin diagnostic shows (§6.1). This was not resolved in this round, so under your time-box the term is parked as "reported, not adopted"; the a5b5264 term stands. The fix belongs in the rate limit of §3.9: raise the margin no faster than the barrier's own class-K rate, so it never sits at h = 0. That is a design change, needs literature first, and needs a new matrix. Until then the RTK numbers of §6 (8 m calm, 18–19 m in wind) are what the term would give, not what the design claims.2. **Capacity in wind.** In calm air RTK now fits 14 lanes in the 100 m × 30 m corridor (1064 drones per km). In the India wind none fits, because the RTK margin's GNSS term (10–11 m) is the wind uncertainty carried through the drag over T (§6.4). Shortening that horizon or tightening the wind model is the next lever. Option (c) of §3.11 is not: it needs a Lipschitz backup in main and promises little over (a) and (b).
3. **SPS exceeds main's margin bound;** SBAS now fits it. SPS requests 45–52 m, above the 48.57 m `FilterConfig` allows (§6.1). With B5 (§3.10), neither SBAS nor SPS qualifies for dense corridors on integrity grounds.
4. **The drag decay rests on the registered D (re-review 3a).** The perturbation decays at D, which is derived from the declared top speed and is an upper bound when that speed is a software limit (§3.3). A smaller true D slows the decay and enlarges the lost-command error. Your decision asked for the tracker's K σ bound on the drag: under linear drag there is no estimate of D to bound, so none was applied (§3.6). A type test of D, or a declared lower bound D_lo per airframe, would close this.
5. **The `dc` bound (re-review 3b)** rests on the tracker's Gaussian model at K3 for the correction part and on an airspeed bound that `wind` checks only horizontally. The station checks every command exactly, and the bound held with at least 21 % to spare in every run. Whether `dc` should raise the drone's term to the airframe bound automatically (the proposed response in §5) is a rule change.
6. **The lag identifier: false alarms or detection** (unchanged from the previous round). N1 keeps the false alarms low. Detection of a doubled lag is 5–13 of 16 runs (§6.2); an offset-only regressor found 14–16 of 16 at `a5b5264` (scratch diagnostic). The declared upper-bound lag τ_max (§3.10) needs a figure per airframe that no source gives.
7. **`headroom` in calm air:** 24–27 runs per grade (§6.1), none in the India wind. At the calm-air cleared speed the braking inequality of §3.3 holds with equality (there is no forecast gust reserve), so the K2 σ of the tracker's drag estimate trips it. Options: clear calm-air speeds with the same K2 σ reserve that `headroom` uses, or give `headroom` a tolerance; the first changes an operating threshold.
8. **B8 is masked, not solved,** and **B7 has no warning of its own** (`margin` is not a warning). Unchanged from the previous round.
9. **Option (b) of the earlier §8 (a command plan per message)** waits for a consistent per-drone backup (BL-03) and an onboard-layer ADR (founder, re-review). The applied-command echo (contract v1.6, D1) is accepted and not built.
10. **Main's filter on the drag plant.** The baseline rows are unchanged: main as configured loses 18 of 47 RTK calm runs and 39–43 of 47 SBAS and SPS calm runs (§6.1, §6.6); BL-06 and BL-07.
11. **Modelling choices still in force,** each logged in DECISIONS: the NIS window of 10 messages and the 2 s convergence time; the 99.9 % latency quantile for the playout delay; AGE_S = 0.46 s; N_UPD = 2 updates per cycle; the floors on q and P_ww(0); τ_a = 0.25 s; drag and authority derived from top speed and tilt; the light turbulence class for the India wind; the lag monitor's 2 s window, five-point grid and trend term; the detectors at P_RISK = 10⁻³ per test.
