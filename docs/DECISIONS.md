# Decision log

Decisions taken under the "Decision authority" section of `CLAUDE.md`. Each entry says what was decided, why, and who decided. Decisions that needed the founder are marked as such.

## 2026-10-23 (PR #12)

- **`scripts/verify/probe.py` margin sweep.** The sweep keeps the valid margins 0.5 m and 5 m and now asserts that `FilterConfig(margin_m=50)` and `FilterConfig(margin_m=90)` raise `ValueError`. The validation added in PR #12 rejects margins above 48.57 m at the default barrier offset, so the old sweep crashed. Decided by the founder; logged as the model for tooling fixes of this kind.
- **Vertical reversal xfails at σ = 1.5 m.** If the braking row XPASSes, its xfail is removed and it becomes a normal test; the backup row stays `xfail(strict=True)` while it still fails. The P2-SA-02 sentence on reversals is corrected with the measured numbers. Decided by the founder; logged as the model for test-structure changes of this kind.
- **`squeeze2.py` wind rows.** The constant-wind floor rows leave the world box (0.228 m backup; 0.099 m braking with 1 fallback and 1 solver error). This is the known world-barrier wind limit (BL-07, part of the disturbance work in BL-06) and is outside the PR #12 acceptance bar. Decided by the founder.
- **Invariance change documented, not fixed.** `invariance.py` (seed 7, 3000 trials) gives 1634/216640 violating steps, worst 0.26 m, 1/2708 entering `d_eff` at 883b9c2, and 3659/216640, worst 2.06 m, 3/2708 on the current tree. An ablation showed that the larger gap comes from the script's own pre-latch h model, and the entries from the script passing no latch. With the filter's own h the worst gap is 0.13–0.14 m. With a persistent latch, as the runner uses, none of the three pairs enters `d_eff`. Following the founder's rule for item 8 (latch deliberately deviates from the old escape, forward invariance holds in fuzz), this is documented in P2-SA-02 §5.4. The scripts are left unchanged, because the founder asked for a report, not a fix.
- **Hard-fuzz seed 3810 (2 drones, 3.568 m) is not a merge blocker.** `recov.py 3810`: best joint 3.735 m, not recoverable, so it is not a failure of the turn-right rule.
- **BL-05 added to `TASKS.md`; BL-03 to BL-06 assigned to Salman.** Founder instruction. The acceptance criteria text for BL-05 was written by Claude following the pattern of the other rows.
