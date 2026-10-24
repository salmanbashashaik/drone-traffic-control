# Adversarial review: integration/px4-bridge (HEAD fb65281, worktree C:\Users\hp\Desktop\dtc-px4)

Reviewer: fresh subagent (Fable 5.1), read-only. No SITL started. Tests run: `pytest tests/test_px4_bridge.py -n0` = 8 passed (29.8 s). Fast suite collects 201 (claim correct).
Scope check: `git diff origin/main...HEAD --stat` touches nothing under `src/utm_services/safety/` or `shared/contracts.py`. Confirmed.

## Verdict: NEEDS_CHANGES

One HIGH (the abort guard does not do what the docs say it does), several MEDIUM provenance gaps in the numbers, and a records-format inconsistency. No BLOCKER: nothing here is a safety claim beyond "simulation measurement", and the committed data reproduces every §10 number.

## Findings

### HIGH-1. Overrun abort cannot catch a compute stall inside a step; docs say it does
- `src/utm_services/integration/px4_bridge.py:59,426,461-462,507`
- Problem. `overrun_ms` is measured at step start against `next_tick`. After a slow step, line 507 sets `next_tick = max(next_tick + DT_S, monotonic())`, so the next step starts with `overrun_ms ≈ 0`. The stale check (line 433) runs only on the age at telemetry read, before the filter solve. A stall between read and `fleet.send` (QP solve, numba compile, GC, WSL scheduling) is never aborted; the setpoint is sent on a state that is up to the stall length old. Only sleep overshoot (OS not waking the thread) trips the 500 ms guard.
- Evidence. Probe with `_FakeFleet`-style plant and `apply_filter` monkeypatched to sleep 0.8 s on call 3: `reason: max_steps`, `loop_latency_ms = [2932, 99, 905, 8, 28, 36]`, `step_overrun_ms = [0, 0, 41, 0, 25, 3]`. The 905 ms step (and the uncompiled 2932 ms first step) passed. Line 59 comment "or a step this late, stops the run" and `docs/DECISIONS.md:88` ("in the first headon2 run that one step was 13.8 s late") describe exactly the stall this guard does not catch; with the current code that run would have continued, sending a 14 s-old velocity.
- Fix. Before each `fleet.send`, abort when `ages[aid] + (perf_counter() - t0) * 1000 > TELEMETRY_STALE_MS` (age at send time). Add the test: monkeypatch `apply_filter` to sleep 0.6 s once, assert `reason.startswith("stale")` and that no send happened for that step. Reword line 59 and DECISIONS to say what is guarded.

### MEDIUM-1. Shared-origin check: tolerance too loose for the margin, no altitude check
- `px4_bridge.py:60,92-93`; `docs/DECISIONS.md` origin paragraph; `PX4_SETUP.md` §6
- Problem. `ORIGIN_TOLERANCE_M = 1.0` per vehicle allows up to 2 m pairwise frame error against a 0.5 m filter margin (d_eff 5.5 m). Only lat/lon are compared; `altitude_m` of each GPS origin is ignored, yet cross3-on separation relies on dz 4.87 m at the a–c minimum (3D 7.221 m, horizontal 5.332 m) and b–c has horizontal 0.998 m with dz 8.65 m. An unchecked per-vehicle altitude-origin offset would move the drones' true vertical separation without the filter seeing it.
- Evidence. Measured errors 0.066–0.141 m (manifests), so today the frames agree; the check just would not catch the failures that matter.
- Fix. Compare the altitude of `get_gps_global_origin()` too; tighten `tol_m` to ≤ 0.25 m (half the margin); or translate each vehicle by its measured offset instead of asserting it small.

### MEDIUM-2. §7 tracking percentages and the "13.8 s late" step have no committed evidence
- `docs/integration/PX4_SETUP.md` §7 accel_tracking table ("13 % / 10 %", "77 % / 83 %", "73–98 %", "71–72 / 99", "40–41 / 94"); `docs/DECISIONS.md:85,88`; `scripts/px4/accel_tracking.py`
- Problem. No output of `accel_tracking.py` is committed, and the headon2 run with the 13.8 s step is not in `docs/results/px4/`. The in-flight tracking table (median 84–87 %) I reproduced from `records.jsonl`; the bench numbers I cannot. These numbers justify the deviation from the brief (feedforward).
- Fix. Commit `accel_tracking.py` stdout (one file per setpoint mode) next to the results, and the aborted run or its summary.json, or delete the specific percentages and keep the qualitative claim.

### MEDIUM-3. Link-loss timeline: two of the anchor times are not in the committed data
- `docs/integration/PX4_SETUP.md:269-270`; `docs/results/px4/headon2-linkloss/manifest.json`, `watch.log`; `px4_demo.py:145-154`
- Problem. "1792824779.465 last fresh setpoint" comes from bridge stdout that is not saved (manifest has no `last_send_unix`); "≈ 779.82 exit" is derived as failsafe − 1.0 s (COM_OF_LOSS_T), so the "1.35 s" row is circular to about 0.35 s. That 0.35 s is the bridge writing records, summary, replay3d.html before `os._exit`, during which MAVSDK keeps re-sending the last setpoint; it is not PX4 detection time.
- Evidence. OFFBOARD mode seen in watch.log 776.8–777.3; last record `wall_time_s` 2.9002; 777.3 + 2.9 ≈ 779.47 — plausible, not recorded.
- Fix. Write `last_send_unix` and a `time.time()` taken right before `os._exit` into manifest.json (the `_write` call already runs before exit); re-state the row as "≈1.0 s after last re-sent setpoint, ≈1.35 s after last fresh step". Also "about 31 m" (line 272) vs log "30 m above destination", and "landed at their own launch points" (line 284) is not supported by watch.log (no horizontal position logged); soften or log it.

### MEDIUM-4. `h_min_prev_m2` in bridge records is post-step; env's is pre-step
- `px4_bridge.py:155-181`; caller in `run_encounter`; `src/utm_services/simulation/env.py:272,350`
- Problem. `step_infos` receives `h_min_m2(states)` of the next states and stores it as `h_min_prev_m2`. `DroneEnv.step` stores `_h_min(stepping)` taken before the step. Any tool that reads `utm-run` records (`scripts/verify/*`, replay) and compares `h_min_prev_m2` with `h_min_m2` gets the wrong sign of the trend on bridge records. Record 0: `h_min_prev_m2` equals h_min of record 1's states.
- Fix. Pass the pre-step `h_min_m2(states)` (compute before the filter call) and add one assertion to `test_run_encounter_record_format`.

### LOW-1. `Px4Fleet.ned` reads `v.latest` twice (line 310); callback (line 292) can swap between reads → position from one sample, velocity from the next. Harmless at 50 Hz, but one local `d = v.latest` fixes it.
### LOW-2. `px4_demo.py:112-113`: `warm_up_filter` and `Px4Fleet(...)` run outside the `try` (line 114). If either raises (MAVSDK connect timeout is the common case), `sitl.sh stop` never runs and SITL stays up.
### LOW-3. `Px4Fleet.land` (line 369-380) raises `TimeoutError` after 120 s; inside the `except BaseException` path in `px4_demo.py` this replaces the original exception. Chain it (`raise ... from`) or log the original first.
### LOW-4. Hover criterion `abs(-p[2] - h) < 0.5 and hypot(*w) < 0.3` (line 347) is fragile: PX4 settles at takeoff altitude − ~0.3 to 0.5 m in SITL; a 19.50 m hover for h=20 never passes and the run times out instead of starting. Widen to 1.0 m or test |vz| as well.
### LOW-5. Records of runs that end by `max_steps`, `stopped` or `overrun` carry no `aborted` marker; only `stale_telemetry` sets `abort_reason` in infos. `summarize` reads `end_reason` from the caller, so the file alone does not say why it ended. Put `end_reason` into the last record.
### LOW-6. HOCBF name reintroduced: `px4_bridge.py:1`, `PX4_SETUP.md:4`. CLAUDE.md glossary: the name is wrong (relative degree 1); use "backup-set barrier filter".
### LOW-7. `PROTOCOL_NOTES.md:84` says neighbours "are switched to their own backups by the server ... because the certificate holds only when every drone of a pair flies its backup". That is stricter than §2 (unconfirmed drone treated as flying its backup, fixed rows) and than CSA Theorem 1 in P3-SA-02 §5, which certifies a pair when the *other* drone flies any filtered action, as long as the committed backup stays feasible. Pick one statement; §2's is the correct one. Line 97 "Backups assume lag_s = 0" ignores that the CBK horizon already includes a 5-step lag allowance (P3-SA-02 §2); say which allowance the bridge would use.
### LOW-8. `warm_up_filter` compiles the two-drone path only; a first three-drone `apply_filter` call may still trigger a (shorter) numba specialisation. With HIGH-1 fixed this becomes an abort instead of a late setpoint, which is the right outcome, but note it in §8.

## Verified correct (no action)
- Safety package and contracts untouched (diff stat).
- Frames: `ned_to_ours`/`ours_to_ned_vector` sign and axis mapping, `WORLD_ORIGIN_XY`, `ALT_OFFSET_M`, round trip test; observation equals `DroneEnv`'s for the same state (test).
- Setpoint: v_cmd = v_meas + a_safe·dt clipped to 15 m/s keeping direction; `speed_capped` flag; clip shown in test.
- Latch dict persists across steps and is passed to every `apply_filter` call; completed drones stay in `states` so the filter still sees them (stacking in filter-off runs is fair: `ReferencePolicy` ignores neighbours by design).
- §10 table reproduced from records: headon2-on min 3D 8.461 m (step 60, z 38.52/45.79), min H 4.205; cross3-on a–b 7.383/H 6.02, a–c 7.221/H 5.332, b–c 8.493/H 0.998 (dz 8.65); headon2-off 10.456/H 0.266; cross3-off 10.074/H 0.183; linkloss 58.296. Zero d_safe violations; `shield_active` counts match. Peak horizontal speed 12.39–13.09 m/s ("12.4 to 13.1" OK); peak |vz| 3.16 m/s.
- §1 latency/age percentiles (p50/p95/max) match `summary.json` for all five runs; §5 budget sums correctly.
- §7 in-flight tracking table (projection definition, |a_cmd| > 1 m/s²) reproduced exactly.
- cross3 "~1 m horizontal, separation vertical" is disclosed in §10; the text does not claim horizontal separation.
- §12 filter/PX4 mismatches list (MPC_XY_VEL_MAX 12 vs 15, Z 3/1.5, lag_s 0, re-send, RTL) is complete against what I found.
- PROTOCOL_NOTES §4 citations of P3-SA-02 (candidate set 0 brake + 26 sign vectors × {1 s, 2 s}, TBC clock τ0, acceptance bar 3 of 4, 273 ms at N=16, 2 of 20 σ=1.5 runs enter d_eff, §9 question 7) match the branch document.
- CLAUDE.md/STATUS "201 tests": `pytest --collect-only -m "not slow"` = 201.
- No proof or flight-readiness language found; every result is labelled SITL measurement.

## Test gaps
- No test for a stall inside a step (HIGH-1).
- No test of `step_infos`/`summarize` semantics against `DroneEnv` infos (MEDIUM-4).
- Feedforward term never checked by a test (`_FakeFleet` ignores `acc_ned`).
- Completed-drone-stays-in-filter-input only implicit in the record-format test.

## Not checked
- SITL not run; all PX4 behaviour taken from committed logs. `accel_tracking.py` not executed. MAVSDK re-send-after-exit behaviour taken from the docs' own account.

## Disposition (author, 2026-10-24)

All runs in `docs/results/px4/` were re-run with the fixed code after this review.

- **HIGH-1: fixed.** Before any setpoint is sent, `run_encounter` now checks the age of the state at send time: telemetry age + time since the step's snapshot. Above 500 ms it sends nothing for that step, marks the last record `aborted` and ends with `stale_at_send` (`px4_bridge.py`). The new test `test_run_encounter_stall_inside_step_aborts_before_send` fakes a 0.6 s stall inside step 3. It fails on fb65281 and passes now. The comments, `PX4_SETUP.md` §8 and DECISIONS now say what each guard catches.
- **MEDIUM-1: fixed.** `ORIGIN_TOLERANCE_M` is 0.25 m, so a pair can disagree by at most 0.5 m, the filter margin. The vertical check uses each vehicle's on-ground EKF height against vehicle 0, not the GPS-origin altitude.
- **MEDIUM-2: fixed.** `accel_tracking.py` output is committed in `docs/results/px4/accel-tracking/` (defaults, plus two runs with `MPC_ACC_HOR=5`), and §7 quotes those files. The 13.8 s late step is not reproducible after the warm-up fix. DECISIONS now quotes the measured cold warm-up (manifest `warm_up_s` of the re-run) instead.
- **MEDIUM-3: fixed.** The manifest records `last_step_unix` for every run, and `exit_unix` right before `os._exit`. `link_loss_watch.py` logs north/east from launch. §11 quotes only those recorded times.
- **MEDIUM-4: fixed.** `h_min_prev_m2` is the pre-step value, as in `DroneEnv`. The record test asserts `h_min_prev_m2 == h_min_m2(record's own states)`.
- **LOW-1, -2, -3, -5, -6, -7, -8: fixed.** One sample read; warm-up and fleet creation inside `try`; the original exception is printed before landing; `end_reason` is in the last record; HOCBF wording removed; PROTOCOL_NOTES restated per §2 and CSA Theorem 1, and the CBK lag allowance named; the warm-up uses the run's drone count.
- **LOW-4: not changed.** The hover criterion passed in every run so far, both before and after the review. Widen it if a run times out at takeoff.
- **Test gaps, not added:** feedforward term (`_FakeFleet` ignores `acc_ned`); completed drone staying in the filter input as its own test.
