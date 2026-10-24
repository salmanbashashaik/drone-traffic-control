# Adversarial review: decision viewer capture hook and replay (PR #27)

- Reviewer: Claude Fable 5.1 (fresh subagent, read-only)
- Date: 2026-10-24
- Branch / commit: `demo/decision-viewer` @ `b1ee602`, against `origin/main` @ `a8dcc5c`
- Worktree: `C:\Users\hp\Desktop\dtc-viz`

## Scope

1. `src/utm_services/safety/filter.py`: `_CAPTURE` ContextVar and `capture_internals()`; claim that capture off is unchanged and capture on is bit-identical.
2. `tests/test_filter_capture.py`: whether the bit-identity tests would catch a bad hook.
3. `src/utm_services/evaluation/decision_log.py`: offline replay and `ReplayMismatch`; whether the replay feeds the filter the live inputs and whether a logged field can be wrong while the check passes; PX4 records.

Viewers (`viz_cesium.py`, `viz_rerun.py`) and docs were out of scope and not reviewed.

## Verdict

No BLOCKER, no HIGH. The hook is placed after the `SafeAction` list is built (`filter.py:1652`), reads one `ContextVar` and copies values (`axis.tolist()`, immutable tuples/floats/str), so with capture off the only change is one `ContextVar.get()` per call. The replay feeds the filter the recorded `observed_states` and `proposed_actions`, the manifest `FilterConfig` and a latch dict carried across every step from `{}`, and compares every replayed `SafeAction` of every drone on every step with the recorded one with `==` on all fields (`decision_log.py:136`). Two MEDIUM findings (mixed truth/observed frames in the log; no test on the solver-error fallback path) and some LOW ones. Safe to merge from the safety-filter side once the MEDIUMs are either fixed or recorded as known limits.

## Findings

### MEDIUM-1: per-pair log entries mix the truth frame and the observed frame
- `src/utm_services/evaluation/decision_log.py:159-191` (`state = {... observed}`, `entry = {"pair": ..., "sep_m": dist[(a, b)], ..., "backup_min_m": ..., "backup": ...}`), and per drone `pos`/`nearest` at lines 146-153.
- `sep_m`, `min_sep_m`, `pos`, `nearest` come from the recorded truth states; `backup`, `backup_min_m`, `backup_steps` and the latch `side` vectors are computed from the observed states (the filter's inputs). Under the realistic profile observed differs from truth on every step (my run: 337 of 337 steps, GNSS sigma 1.5 m). A reader of the log or the viewers can see `sep_m < d_eff` next to `backup_min_m > d_eff` for the same pair, and the drawn backup path starts up to a few metres off the drone marker, and draw a wrong conclusion about what the filter saw. The module docstring says "positions are the recorded truth; internals from the recorded filter inputs" but no field says which frame it is in.
- Not a replay-correctness bug: the check is unaffected.
- Suggested fix (smallest): add `"obs_pos"` (and `"obs_vel"`) per drone so viewers can start backup paths from it, and one header line, e.g. `"frames": "pos/sep_m/nearest: recorded truth; backup*, latch side: filter input (observed)"`. Or at least state it in `VIEWERS.md` section 1.

### MEDIUM-2: the solver-error fallback path is not covered by any bit-identity test
- `tests/test_filter_capture.py` (whole file). No test forces `_solve_hocbf_qp` to raise, so the `qp_feasible=False` / `_fallback_cluster` branch of `apply_filter` is never run under capture. A hook that, say, read `pair_slack` before it was reset on that branch would pass the suite. OSQP errors do not occur in the fuzz (40x5 steps: 0 fallbacks).
- Verified by probe (ran code): monkeypatching `F._solve_hocbf_qp` to raise, 120 steps of 2-6 converging drones over 4 consecutive steps each, all with a non-empty evasive cluster: actions, diagnostics and latch bit-identical with capture on and off; `calls[0]["escape_pairs"] == []` as expected on that path.
- Suggested fix: add that probe as a test (about 15 lines, `monkeypatch.setattr(F, "_solve_hocbf_qp", boom)`).

### LOW-1: a record with zero drones raises `IndexError`, not `ReplayMismatch`
- `decision_log.py:139` (`call = calls[0]`). `apply_filter([], [], ...)` returns at `filter.py:1450` before the capture point, so `calls` is empty. Verified by running a hand-made record: `IndexError: list index out of range`. The runner never writes such a record (the loop ends on `env.done`, which is set when no agent remains) and the PX4 bridge only records steps where `apply_filter` ran (read `px4_bridge.py:440-513`). Hand-edited logs only.
- Fix: `call = calls[0] if calls else None`, or raise `ReplayMismatch` with a clear message.

### LOW-2: the equality check treats `-0.0` and `0.0` as equal
- `decision_log.py:136`. `==` on JSON-loaded floats. Probe: flipping the sign of 152 zero components in `returned_actions` of a cross4 run still replays. Physically the same command, so no safety effect; but the docstring says "exactly". NaN would be refused (nan != nan).
- Fix, if the wording matters: compare `json.dumps(..., sort_keys=True)` strings, or leave and soften "exactly" to "equal".

### LOW-3: context-manager edge cases not tested (nested, exception exit, threads)
- `tests/test_filter_capture.py:105` tests only default-off and reset after a normal exit. Verified by probe: nested contexts record to the inner list only (outer=1, inner=1), `_CAPTURE` is `None` after an exception exit (`finally: reset(token)`), and a `threading.Thread` started inside the context sees `None` and does not append to the outer list. Also no test for per-step aliasing; by reading, `axis.tolist()` plus immutable tuples/str/float means no captured object is shared with `latch` (probe: 0 aliased of 17 captured latches).
- Fix: optional 8-line test for nested + exception exit.

### LOW-4: "cluster" in the log is the connected component of latched pairs only
- `decision_log.py:155-157` (`_clusters(list(call["latches"]))`). Pairs in the QP without a latch, and the fallback's `_fallback_cluster`, are not part of it. Consistent with `_update_latches`' own notion, but `VIEWERS.md` calls it "conflict cluster" without defining it.
- Fix: one sentence in `VIEWERS.md` section 1.

### LOW-5: truth-state fields are unchecked by design
- `pos`, `sep_m`, `min_sep_m`, `nearest`, `intervened`, `fallback_used` are copied from the record. Probe: moving a truth position by 5 m still replays (expected: truth is not a filter input); flipping `shield_active` in a record is refused (it is a `SafeAction` field, so the comparison catches it); perturbing an observed position by 1e-9 m is refused; changing `margin_m` to 0.6 in the manifest is refused. So every field that the filter produced is checked; fields that only the simulator produced are trusted. Worth one line in the docstring.

### Notes (no action needed)
- PX4: records are built at `px4_bridge.py:513` only after `apply_filter` ran, `observed_states == states` (EKF), timestamps `k*100`, completed agents stay in the filter input, and a `stale_at_send` abort drops the current step's record after `apply_filter` mutated the latch, but it is always the last step so replay is unaffected. I could not run a real PX4 log (Linux only); I replayed PX4-shaped records (`backend: px4-sitl-gazebo`, observed == states, `filter` on and `None`) and both paths work.
- Pre-existing, not introduced here: on a solver error with an empty evasive cluster, `diag.qp_feasible` is `False` but every `SafeAction.qp_feasible` is `True` (`filter.py` builds it as `not in_fallback`). Out of scope; mention for the maintainers.
- Replay determinism across machines or OSQP/numba versions is a documented known limit; the log records `run_code` and `log_code` but not library versions. A latch that differed on replay without changing any action is possible only under non-determinism, which the same limit covers.
- `FilterConfig(**manifest["filter"])` round-trips: `vars()` of a frozen dataclass of floats/int/str/None through JSON; validators on `DroneState`/`MARLAction` only check bounds, so re-validation is idempotent.

## What I ran versus read
- Ran: `pytest tests/test_filter_capture.py -n0` (6 passed, 42 s); probe scripts in `C:\Users\hp\AppData\Local\Temp\claude\c--Users-hp-Desktop-drone-traffic-control\a97fc250-21b1-4810-9453-130ff93123e0\scratchpad\probe_capture.py` and `probe_fallback.py`: fallback bit-identity (120 steps), escape-stage coverage of the fuzz (61 of 200 steps have escape pairs, 81 have slack, 0 escape failures), nested/exception/thread isolation, aliasing, realistic-profile `sandwich-3` replay through `write_decision_log` (337 steps, 77 with dropped commands, drone count falling 3 to 1 as drones complete, 46 interventions), PX4-shaped replay on and off, `-0.0` tamper, four tamper cases, empty-step record.
- Read only: capture-off path unchanged; capture placement and copies; early return at `filter.py:1450`; runner and PX4 record construction; `_backup_constraints` axis/up construction versus the replay's (identical: zero axis / 0.0 up for unlatched pairs, same `_pair_states` gather); latch carry; contract validators; `git_state` failure handling.
- Not run: slow tier, full/quick verification tiers (author's), viewers, a real PX4 SITL log.

Skipped: the viewer modules and docs beyond the frame question, and any timing. Risk to note: the realistic-profile frame mix (MEDIUM-1) can mislead a human reading the viewers; the filter itself is unaffected.

## Fixes applied (author, 2026-10-24, after this review)

- MEDIUM-1: the log header has a `frames` field naming which fields are truth and which come from the observed state; the `decision_log.py` docstring and `VIEWERS.md` say the same. No `obs_pos` field was added.
- MEDIUM-2: `test_solver_error_fallback_is_bit_identical_with_capture` injects a solver error into `_solve_hocbf_qp` and compares actions, diagnostics and latch with capture on and off on converging clusters; it asserts the evasive fallback was reached.
- LOW-1: `decision_steps` raises `ReplayMismatch` unless exactly one filter call was captured.
- LOW-2: the replay compares the JSON text of replayed and recorded actions, so `-0.0` and `0.0` differ.
- LOW-3: `test_capture_is_off_by_default_and_ends_with_its_context` now covers exit by exception and nested contexts. Threads not tested.
- LOW-4: "cluster" is defined in `VIEWERS.md` and in a code comment.
- LOW-5: the docstring says truth fields are copied from the record and not checked.
- Note on `SafeAction.qp_feasible` on a solver error: pre-existing filter behaviour, not changed here (the only edit in `safety/` is the capture hook).
