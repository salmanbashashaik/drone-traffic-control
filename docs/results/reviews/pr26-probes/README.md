# PR #26 review probes

Targeted probes behind `../pr26-review-2026-10-24.md`. Run each from the repo root with the project installed (`pip install -e ".[dev]"`), for example:

    python docs/results/reviews/pr26-probes/probe2c_where.py

They import `common.py` from this directory, so run them with this directory on `sys.path` or from inside it:

    cd docs/results/reviews/pr26-probes && python probe2c_where.py

| File | Attack | What it shows |
|---|---|---|
| `probe1_cross.py` | 1 | 300 random multi-cluster scenes, one cluster forced to miss: every cross-cluster row holds, kept clusters bit-identical, evasive inside the world box |
| `probe2_overrun.py` | 2 | wall time of `_solve_daqp`, `_solve_osqp`, `_solve_exact` past a 1-5 ms limit at N = 16, 32, 64 |
| `probe2b_starve.py` | 2 | 64-ring plus a far 3-drone cluster, deadline on: the far cluster misses too |
| `probe2c_where.py` | 2 | the same scene, time per phase (latches, backup, solves, evasive) and each cluster's time left at entry |
| `probe2d_bench_first.py` | 2, 6 | `scale_bench`'s own scenes, steps 0-7 with the phase split (the bench drops steps 0-4) |
| `probe2e_prof.py` | 2 | cProfile of `_update_latches` on a 64-drone first step |
| `probe3_exact.py` | 3 | `head-on-40` closed loop, deadline off vs on (standard and 10 s budget): records bit-identical; `daqp.solve` accepts `time_limit` |
| `probe5_cert.py` | 5 | `_kkt_certified` against OSQP at eps 1e-3 to 1e-9, with and without polish, and against perturbed optima, on a 3-ring, a collinear squeeze (both stages) and a 16-ring escape QP |
