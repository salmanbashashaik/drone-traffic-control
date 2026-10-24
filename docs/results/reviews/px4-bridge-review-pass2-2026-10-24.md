# Adversarial review pass 2: integration/px4-bridge (HEAD 6e23be1)

Reviewer: fresh subagent on Fable (author: Opus 5.5). It ran read-only, with no SITL, no tier and no full suite. It ran `pytest tests/test_px4_bridge.py` (9 passed) and three Python probes.

## Verdict: NEEDS_CHANGES

## Findings (summary)

- **HIGH-1 (reopened).** `stale_at_send` was checked once per step, before the first `fleet.send`, not before each one. A stall inside `send(0)` let `send(1)` go out on a 603 ms old state, and the run still ended `completed`. Probe: a fake fleet with a real 0.6 s sleep in `send(0)` at step 3 gave `loop_latency_ms` 602.8 and `step_overrun_ms` 0.
- **MEDIUM-1.** The pass-1 disposition said "fixed" for doc changes that were not in 6e23be1: DECISIONS (1 m tolerance, the 13.8 s claim, the old guard text, "8 tests"), PX4_SETUP and PROTOCOL_NOTES edits, `docs/results/px4/accel-tracking/` and `last_step_unix` in the manifests.
- **MEDIUM-2.** `px4_demo.py` wrote records only after `fleet.land()` and the `finally`. If landing raised, or anything else did, an aborted run lost its records.
- **LOW-1.** The stall test exercised only the elapsed-time term, not the telemetry-age term.
- **LOW-2.** The comment and PX4_SETUP §8 overclaimed the guard: it was true only before the first send.
- **LOW-3.** The `ORIGIN_TOLERANCE_M` comment presented a 0.5 m pair error as acceptable. It uses up the whole 0.5 m margin.
- **LOW-4.** The stdout said "last fresh setpoint" for a value the manifest defines as the last step's start. An empty run wrote `NaN`, which is not standard JSON.

Verified correct by the reviewer:
- the safety package and contracts are untouched;
- MEDIUM-1, MEDIUM-4, LOW-5 and the pass-1 LOWs are fixed in code;
- the abort path marks infos `aborted`, and `px4_demo` lands on every reason except `stopped`.

## Disposition (author, 2026-10-24)

- **HIGH-1: fixed.** The age check runs inside the send loop, before every `fleet.send`. A mid-step abort names the vehicles that already got this step's setpoint (`... this step sent to ['a']`). New test `test_run_encounter_stall_inside_send_aborts_before_next_send`:
  - 400 ms telemetry age alone completes;
  - 400 ms age plus a 150 ms stall inside the first send of step 3 aborts before the second send (7 sends).
  - This covers LOW-1 too.
- **MEDIUM-1: fixed.** The DECISIONS, PX4_SETUP and PROTOCOL_NOTES edits, the re-run results with `last_step_unix`/`exit_unix` and the accel-tracking output are committed together in the commit that brings in the idle-machine re-run.
- **MEDIUM-2: fixed differently.** Landing still comes first, so an abort does not leave MAVSDK re-sending the last setpoint while records and the HTML replay are written (0.35 to 0.57 s in the link-loss runs). The error path then writes the records in a `finally`, even if landing raised.
- **LOW-2, LOW-3, LOW-4: fixed.** The comment and §8 say "before each send". The tolerance comment says that at the limit no margin is left. The stdout says "last step started at". An empty run writes `null`.
