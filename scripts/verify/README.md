# Safety filter verification scripts

Independent checks of `utm_services.safety.filter`, written by the PR #12 reviewer. They are
measurements, not tests: they print results and never fail the suite. Run them from the repository
root after `pip install -e ".[dev]"`. Python puts this folder on `sys.path`, so the scripts import each
other (`fuzz`, `optbound`, `invariance`, `probe`) without extra setup. `apply_filter` requires a `latch`
dict (UTM-077). The closed-loop scripts (`fuzz.py`, `probe.py`, `vnoise.py`, `hnoise.py`, `invariance.py`,
`inv2.py`, `det.py`) pass one per run or trial, as the episode runner does; `inv_ablate.py none` passes a
fresh one every step.

| Script | What it checks | Command |
| --- | --- | --- |
| `fuzz.py` | Closed-loop fuzz of `apply_filter` on random 2–6 drone encounters, zero nominal acceleration. Reports fallbacks, solver errors, exceptions, runs inside d_eff, and backup rollout steps and cap hits. `HARD=1` uses shorter time to approach. Prints one line per run in seed order and appends one JSONL row per run to `--out` (default `fuzz_<barrier>_<start>_<n>[_hard].jsonl` or `fuzz_<barrier>_<tier>.jsonl` in `$FUZZ_OUT`, default the current directory); a rerun with the same file resumes. `--tier quick` (413 runs) or `--tier full` (10000 runs) runs both modes. A run stops early once nothing it records can change (`--no-early-stop` to fly all steps). | `python scripts/verify/fuzz.py 5000 backup 100 8` (runs, barrier, steps, processes, [first seed]); `python scripts/verify/fuzz.py --tier quick` |
| `compare_results.py` | Per-seed check of a fuzz JSONL against `docs/results/pr12-b1-2026-10-23.jsonl`: bit for bit, or `--tolerance 1e-4` for numerical changes (integer fields and the seeds inside d_eff identical, min_sep within the tolerance). Exits 1 on any mismatch. | `python scripts/verify/compare_results.py fuzz_backup_quick.jsonl` |
| `tier.py` | Runs a verification tier: `quick` (fuzz quick tier and exact compare) or `full` (fuzz full tier and exact compare, then every other script in this table at its command, in parallel, except `invariance.py`), or `ondemand` (`invariance.py` only). Logs in `verify_<tier>/`. Verification is change-scoped: quick while iterating, full once per PR. | `python scripts/verify/tier.py quick` |
| `vertical.py` | Vertical head-on (60 m and 120 m, 10 m/s each), both barriers. Any argument prints the steps inside 40 m. | `python scripts/verify/vertical.py` |
| `vsweep.py` | Line-of-sight elevation sweep with lateral offsets and two speeds, both barriers. `*` marks min separation below d_eff, `F` a fallback. | `python scripts/verify/vsweep.py` |
| `floor.py` | Head-on near the altitude floor, both barriers: aborts and lowest altitude. | `python scripts/verify/floor.py` |
| `targeted.py` | Break attempts: world bounds, symmetric rings, vertical encounters, forced solver error (fallback only). | `python scripts/verify/targeted.py` |
| `optbound.py` | Independent max-min-separation optimiser (SLSQP, 100 ms zero-order hold). `sandwich [starts]` or `headon`. | `python scripts/verify/optbound.py sandwich 40` |
| `headon.py` | Head-on minimum start distance for d_eff at 30 m/s closing: closed form and optimiser. | `python scripts/verify/headon.py` |
| `recov.py` | Whether hard fuzz seeds that entered d_eff were recoverable at all. | `python scripts/verify/recov.py 2704 252` |
| `invariance.py` | Per-step decay and forward invariance of the backup barrier, 2 drones, serial (about 165 min). Not in the full tier: run with `tier.py ondemand`. | `python scripts/verify/invariance.py` |
| `inv2.py` | Parallel version of `invariance.py`. | `python scripts/verify/inv2.py 3000` |
| `horizon.py` | Whether the adaptive backup horizon (66 to 109 steps) ever overestimates the backup's minimum distance, against a 300-step minimum. | `python scripts/verify/horizon.py` |
| `probe.py` | Re-review probes: world-limit sweep (`limits`), single drone pushed into walls and corners (`walls`), pairs squeezed against walls (`squeeze`), vertical line of sight (`vertical`), determinism (`det`). | `python scripts/verify/probe.py [section]` |
| `squeeze2.py` | More floor, ceiling and corner squeezes, a three-drone floor stack and a pair under downward wind, both barriers. Imports `probe.py`. | `python scripts/verify/squeeze2.py` |
| `vnoise.py` | Vertical head-on with position noise σ 0, 0.5 and 1.5 m, 20 seeds each, both barriers: worst min separation, runs below d_eff and d_safe, and sideways-direction reversals. Passes one filter latch per run, like the episode runner. | `python scripts/verify/vnoise.py` |
| `hnoise.py` | Horizontal control for `vnoise.py`: the same head-on along x. | `python scripts/verify/hnoise.py` |
| `det.py` | Hashes of filtered actions for fixed scenarios. Run it in separate processes and compare the output. | `python scripts/verify/det.py` |
| `scale_bench.py` | Step time of `apply_filter` at N = 16-128, dense converging groups and city spread: p50/p95 and a time split (QP build, OSQP setup, solve, escape, `_update_latches`, rollout, other), with a hash of every filtered action to compare versions. Each config in its own process; `--src` measures another checkout unchanged. Not in a tier (timing, not a check). | `python scripts/verify/scale_bench.py --out docs/results/scale-<date>.md`; `--n 64 --density dense --cluster 2 4 8 16 32 64` |
